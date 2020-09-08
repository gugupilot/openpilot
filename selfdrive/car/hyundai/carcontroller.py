from numpy import interp, math

from cereal import car, messaging
from common.op_params import opParams
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.carstate import GearShifter
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.pathplanner import LANE_CHANGE_SPEED_MIN

from common.travis_checker import travis

splmoffsetmphBp = [30, 40, 55, 60, 70]
splmoffsetmphV = [0, 0, 0, 0, 0]

splmoffsetkphBp = [30, 40, 55, 70]
splmoffsetkphV = [0, 0, 0, 0]

VisualAlert = car.CarControl.HUDControl.VisualAlert


def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):

  sys_warning = (visual_alert == VisualAlert.steerRequired)
  if sys_warning:
      sys_warning = 4 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 3

  if enabled or sys_warning:
      sys_state = 3
  else:
      sys_state = 1

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.current_veh_speed = 0
    self.lfainFingerprint = CP.lfaAvailable
    self.last_button_frame = 0
    self.button_cnt = 0
    self.vdiff = 0
    self.nosccradar = CP.radarOffCan

    self.sm = 0
    self.smartspeed = 0
    self.recordsetspeed = 20.
    self.setspeed = 0
    self.smartspeed_old = 0
    self.smartspeedupdate = False
    self.stopcontrolupdate = False
    self.fixed_offset = 0
    self.button_res_stop = self.button_set_stop = 0

    self.curvature_factor = 1.

    if not travis:
      self.sm = messaging.SubMaster(['liveMapData'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart):

    self.lfa_available = True if self.lfainFingerprint or self.car_fingerprint in FEATURES["send_lfa_mfa"] else False

    self.high_steer_allowed = True if self.car_fingerprint in FEATURES["allow_high_steer"] else False

    # Steering Torque
    new_steer = actuators.steer * SteerLimitParams.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and ((abs(CS.out.steeringAngle) < 90.) or self.high_steer_allowed)

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 55 * CV.KPH_TO_MS and self.car_fingerprint == CAR.HYUNDAI_GENESIS and not CS.mdpsHarness:
      lkas_active = False

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning =\
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph  else 60
    if clu11_speed > enabled_speed or not lkas_active or CS.out.gearShifter != GearShifter.drive:
      enabled_speed = clu11_speed
    else:
      if CS.is_set_speed_in_mph:
        self.current_veh_speed = int(CS.out.vEgo * CV.MS_TO_MPH)
      else:
        self.current_veh_speed = int(CS.out.vEgo * CV.MS_TO_KPH)
    can_sends = []
    self.clu11_cnt = frame % 0x10

    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.lfa_available, 0))

    if CS.mdpsHarness: # send lkas11 bus 1 if mdps
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.lfa_available, 1))

      can_sends.append(create_clu11(self.packer, frame, CS.mdpsHarness, CS.clu11, Buttons.NONE, enabled_speed, self.clu11_cnt))

    if pcm_cancel_cmd and not self.nosccradar:
      self.vdiff = 0.
      can_sends.append(create_clu11(self.packer, frame, 0, CS.clu11, Buttons.CANCEL, self.current_veh_speed, self.clu11_cnt))
    elif CS.out.cruiseState.standstill and CS.vrelative > 0:
      self.vdiff += (CS.vrelative - self.vdiff)
      if self.vdiff > 1. or CS.lead_distance > 8.:
        can_sends.append(create_clu11(self.packer, frame, 0, CS.clu11, Buttons.RES_ACCEL, self.current_veh_speed, self.clu11_cnt))
    else:
      self.vdiff = 0.

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.lfa_available:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    # Speed Limit Related Stuff  Lot's of comments for others to understand!

    if not travis:
      self.sm.update(0)
      op_params = opParams()
      dat = self.sm['radarState'].leadOne

      speed_unit = CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH
      self.setspeed = CS.out.cruiseState.speed * speed_unit

      if not CS.radar_obj_valid and dat.status and dat.vLead < 3. \
              and  CS.out.cruiseState.enabled and not CS.out.gasPressed:
        aRel = (dat.vLead**2 - CS.out.vEgo**2)/(2 * dat.dRel)
        print("aRel", aRel)
        if aRel < -.5 or self.stopcontrolupdate:
          self.stopcontrolupdate = True
          print("STOPPED VEHICLE")
          self.stopspeed = 20 if CS.is_set_speed_in_mph else 30
          if not self.stopcontrolupdate:
            self.button_cnt = 0
            self.recordsetspeed = self.setspeed
      else:
        if self.setspeed != self.recordsetspeed and self.stopcontrolupdate and  CS.out.cruiseState.enabled \
                and CS.radar_obj_valid and not CS.out.gasPressed:
          self.stopspeed = self.recordsetspeed
        else:
          self.stopcontrolupdate = False
          self.recordsetspeed = 0


      if self.sm['liveMapData'].distToTurn < 350:
        self.curvature_factor = op_params.get('curvature_factor')

        curvature = abs(self.sm['liveMapData'].curvature)
        radius = 1 / max(1e-4, curvature) * self.curvature_factor
        if radius > 500:
          c = 0.9  # 0.9 at 1000m = 108 kph
        elif radius > 250:
          c = 3.5 - 13 / 2500 * radius  # 2.2 at 250m 84 kph
        else:
          c = 3.0 - 2 / 625 * radius  # 3.0 at 15m 24 kph
        v_curvature_map = math.sqrt(c * radius) * speed_unit
      else:
        v_curvature_map = 250

      if self.sm['liveMapData'].speedLimitValid and enabled and CS.out.cruiseState.enabled and op_params.get('smart_speed'):
        self.smartspeed = self.sm['liveMapData'].speedLimit * speed_unit
        self.fixed_offset = interp(self.smartspeed, splmoffsetmphBp, splmoffsetmphV) if CS.is_set_speed_in_mph else \
                            interp(self.smartspeed, splmoffsetkphBp, splmoffsetkphV)
        self.smartspeed = max(self.smartspeed + int(self.fixed_offset), 20) if CS.is_set_speed_in_mph else \
                          max(self.smartspeed + int(self.fixed_offset), 30)
        self.smartspeed = min(self.smartspeed, max(self.smartspeed - 1.5 * self.fixed_offset, v_curvature_map))

        if self.smartspeed_old != self.smartspeed:
          self.smartspeedupdate = True
          self.button_cnt = 0

        self.smartspeed_old = self.smartspeed
      else:
        self.smartspeed_old = 0
        self.smartspeedupdate = False

      framestoskip = 10
      if self.stopcontrolupdate:
        speedtospam = self.stopspeed
      elif self.smartspeedupdate:
        speedtospam = self.smartspeed
      else:
        speedtospam = self.setspeed

      if (frame - self.last_button_frame) > framestoskip and (self.smartspeedupdate or self.stopcontrolupdate):
        if (self.setspeed > (speedtospam * 1.005)) and (CS.cruise_buttons != 4):
          can_sends.append(create_clu11(self.packer, frame, 0, CS.clu11, Buttons.SET_DECEL, self.current_veh_speed, self.button_cnt))
          if CS.cruise_buttons == 1:
             self.button_res_stop += 2
          else:
             self.button_res_stop -= 1
        elif (self.setspeed < (speedtospam / 1.005)) and (CS.cruise_buttons != 4):
          can_sends.append(create_clu11(self.packer, frame, 0, CS.clu11, Buttons.RES_ACCEL, self.current_veh_speed, self.button_cnt))
          if CS.cruise_buttons == 2:
             self.button_set_stop += 2
          else:
             self.button_set_stop -= 1
        else:
          self.button_res_stop = self.button_set_stop = 0

        if (abs(speedtospam - self.setspeed) < 0.5) or (self.button_res_stop >= 50) or (self.button_set_stop >= 50):
          self.smartspeedupdate = False
          self.stopcontrolupdate = False

        self.button_cnt += 1
        if self.button_cnt > 5:
          self.last_button_frame = frame
          self.button_cnt = 0
      else:
        self.button_set_stop = self.button_res_stop = 0

    return can_sends
