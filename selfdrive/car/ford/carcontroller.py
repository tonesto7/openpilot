from os import path
from openpilot.common.params import Params
import json

from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.ford import fordcan
from openpilot.selfdrive.car.ford.values import CANFD_CAR, CarControllerParams
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX

LongCtrlState = car.CarControl.Actuators.LongControlState
VisualAlert = car.CarControl.HUDControl.VisualAlert


def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                           current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, CarControllerParams)

  return clip(apply_curvature, -CarControllerParams.CURVATURE_MAX, CarControllerParams.CURVATURE_MAX)

def hysteresis(current_value, old_value, target, stdDevLow: float, stdDevHigh: float):
  if target - stdDevLow < current_value < target + stdDevHigh:
    result = old_value
  elif current_value <= target - stdDevLow:
    result = 1
  elif current_value >= target + stdDevHigh:
    result = 0

  return result

def actuators_calc(self, brake):
  ts = self.frame * DT_CTRL
  brake_actuate = hysteresis(brake, self.brake_actuate_last, self.brake_actutator_target, self.brake_actutator_stdDevLow, self.brake_actutator_stdDevHigh)
  self.brake_actuate_last = brake_actuate

  precharge_actuate = hysteresis(brake, self.precharge_actuate_last, self.precharge_actutator_target, self.precharge_actutator_stdDevLow, self.precharge_actutator_stdDevHigh)
  if precharge_actuate and not self.precharge_actuate_last:
    self.precharge_actuate_ts = ts
  elif not precharge_actuate:
    self.precharge_actuate_ts = 0

  if (
      precharge_actuate and 
      not brake_actuate and
      self.precharge_actuate_ts > 0 and 
      brake > (self.precharge_actutator_target - self.precharge_actutator_stdDevLow) and 
      (ts - self.precharge_actuate_ts) > (200 * DT_CTRL) 
    ):
    precharge_actuate = False

  self.precharge_actuate_last = precharge_actuate

  return precharge_actuate, brake_actuate

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.CAN = fordcan.CanBus(CP)
    self.frame = 0

    self.apply_curvature_last = 0
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.gac_tr_cluster_last = -1
    self.gac_tr_cluster_last_ts = 0
    self.brake_actuate_last = 0
    self.precharge_actuate_last = 0
    self.precharge_actuate_ts = 0
    self.json_data = {}
    jsonFile = path.join(path.dirname(path.abspath(__file__)), "tuning.json")

    if path.exists(jsonFile):
      f = open(jsonFile)
      self.json_data = json.load(f)
      f.close()

    self.brake_actutator_target = -0.1
    if 'brake_actutator_target' in self.json_data:
      self.brake_actutator_target = self.json_data['brake_actutator_target']

    # Activates at self.brake_actutator_target - self.brake_actutator_stdDevLow
    # Default: -0.5
    self.brake_actutator_stdDevLow = 0.2
    if 'brake_actutator_stdDevLow' in self.json_data:
      self.brake_actutator_stdDevLow = self.json_data['brake_actutator_stdDevLow']

    # Deactivates at self.brake_actutator_target + self.brake_actutator_stdDevHigh
    # Default: 0
    self.brake_actutator_stdDevHigh = 0.1
    if 'brake_actutator_stdDevHigh' in self.json_data:
      self.brake_actutator_stdDevHigh = self.json_data['brake_actutator_stdDevHigh']

    self.precharge_actutator_target = -0.1
    if 'precharge_actutator_target' in self.json_data:
      self.precharge_actutator_target = self.json_data['precharge_actutator_target']

    # Activates at self.precharge_actutator_target - self.precharge_actutator_stdDevLow
    # Default: -0.25
    self.precharge_actutator_stdDevLow = 0.1
    if 'precharge_actutator_stdDevLow' in self.json_data:
      self.precharge_actutator_stdDevLow = self.json_data['precharge_actutator_stdDevLow']

    # Deactivates at self.precharge_actutator_target + self.precharge_actutator_stdDevHigh
    # Default: 0
    self.precharge_actutator_stdDevHigh = 0.1
    if 'precharge_actutator_stdDevHigh' in self.json_data:
      self.precharge_actutator_stdDevHigh = self.json_data['precharge_actutator_stdDevHigh']

    self.brake_0_point = 0
    if 'brake_0_point' in self.json_data:
      self.brake_0_point = self.json_data['brake_0_point']

    self.brake_converge_at = -1.5
    if 'brake_converge_at' in self.json_data:
      self.brake_converge_at = self.json_data['brake_converge_at']

    self.brake_clip = self.brake_actutator_target - self.brake_actutator_stdDevLow

    def apply_ford_steer_angle_limits(self, desired_curvature, CS):
      # convert curvature to steering angle
      apply_steer = self.VM.get_steer_from_curvature(desired_curvature, CS.out.vEgo, 0.0)

      # rate limit steering angle
      steer_up = apply_steer * self.apply_steer_last > 0. and abs(apply_steer) > abs(self.apply_steer_last)
      rate_limit = CarControllerParams.STEER_RATE_LIMIT_UP if steer_up else CarControllerParams.STEER_RATE_LIMIT_DOWN
      max_angle_diff = interp(CS.out.vEgo, rate_limit.speed_points, rate_limit.max_angle_diff_points)
      apply_steer = clip(apply_steer, self.apply_steer_last - max_angle_diff, self.apply_steer_last + max_angle_diff)

      # convert back to curvature
      apply_curvature = self.VM.calc_curvature(math.radians(apply_steer), CS.out.vEgo, 0.0)
      self.steer_rate_limited = apply_curvature != desired_curvature

      return apply_curvature, apply_steer

  def update(self, CC, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        # apply rate limits, curvature error limit, and clip to signal range
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
        apply_curvature = apply_ford_curvature_limits(actuators.curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw)
      else:
        apply_curvature = 0.

      self.apply_curvature_last = apply_curvature

      if self.CP.carFingerprint in CANFD_CAR:
        # TODO: extended mode
        mode = 1 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0xF
        can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -apply_curvature, 0., counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., 0., -apply_curvature, 0.))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      # Both gas and accel are in m/s^2, accel is used solely for braking
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      gas = accel
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS
      stopping = CC.actuators.longControlState == LongCtrlState.stopping

      # Calculate targetSpeed
      targetSpeed = actuators.speed
      if not CC.longActive and hud_control.setSpeed:
        targetSpeed = hud_control.setSpeed

      precharge_actuate, brake_actuate = actuators_calc(self, accel)
      brake = accel
      if brake < 0 :
        brake = interp(accel, [ CarControllerParams.ACCEL_MIN, self.brake_converge_at, self.brake_clip], [CarControllerParams.ACCEL_MIN, self.brake_converge_at, self.brake_0_point])

      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, brake, stopping, brake_actuate, precharge_actuate, v_ego_kph=V_CRUISE_MAX))

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))
    # send acc ui msg at 5Hz or if ui state changes
    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      if self.gac_tr_cluster_last > -1 and self.gac_tr_cluster_last != CS.gac_tr_cluster:
        self.gac_tr_cluster_last_ts = self.frame * DT_CTRL

      gapUiOn = 1 if ( self.gac_tr_cluster_last_ts > 0 and (self.frame * DT_CTRL - self.gac_tr_cluster_last_ts) < (400 * DT_CTRL) ) else 0

      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                         fcw_alert, CS.out.cruiseState.standstill, hud_control,
                                         CS.acc_tja_status_stock_values, gapUiOn, CS.gac_tr_cluster))
      self.gac_tr_cluster_last = CS.gac_tr_cluster

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert

    new_actuators = actuators.copy()
    new_actuators.curvature = self.apply_curvature_last

    self.frame += 1
    return new_actuators, can_sends
