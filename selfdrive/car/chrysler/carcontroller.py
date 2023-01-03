from opendbc.can.packer import CANPacker
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, \
  create_lkas_heartbit, create_wheel_buttons_command, \
  acc_log, create_acc_1_message, create_das_3_message, create_das_4_message, create_chime_message
from selfdrive.car.chrysler.values import RAM_CARS, PRE_2019, CarControllerParams

from selfdrive.controls.lib.drive_helpers import V_CRUISE_MIN, V_CRUISE_MIN_IMPERIAL
from common.conversions import Conversions as CV
from common.cached_params import CachedParams
from common.params import Params, put_nonblocking
from cereal import car
from selfdrive.car.chrysler.interface import CarInterface, GAS_RESUME_SPEED
import math
from common.numpy_fast import clip
#from common.op_params import opParams

ButtonType = car.CarState.ButtonEvent.Type
LongCtrlState = car.CarControl.Actuators.LongControlState

V_CRUISE_MIN_IMPERIAL_MS = V_CRUISE_MIN_IMPERIAL * CV.KPH_TO_MS
V_CRUISE_MIN_MS = V_CRUISE_MIN * CV.KPH_TO_MS
AUTO_FOLLOW_LOCK_MS = 3 * CV.MPH_TO_MS
ACC_BRAKE_THRESHOLD = 2 * CV.MPH_TO_MS

# LONG PARAMS
LOW_WINDOW = CV.MPH_TO_MS * 5
SLOW_WINDOW = CV.MPH_TO_MS * 20
COAST_WINDOW = CV.MPH_TO_MS * 2

# accelerator
ACCEL_TORQ_SLOW = 40  # add this when going SLOW
# ACCEL_TORQ_MAX = 360
UNDER_ACCEL_MULTIPLIER = 1.
TORQ_RELEASE_CHANGE = 0.35
UNDER_ACCEL_THRESHOLD = 0.3
START_ADJUST_ACCEL_FRAMES = 100
CAN_DOWNSHIFT_ACCEL_FRAMES = 200
ADJUST_ACCEL_COOLDOWN_MAX = 1
MIN_TORQ_CHANGE = 2
ACCEL_TO_NM = 1200
TORQ_BRAKE_MAX = -0.1

# braking
BRAKE_CHANGE = 0.06

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.next_lkas_control_change = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

    self.settingsParams = Params()
    self.cachedParams = CachedParams()
    self.auto_resume = self.settingsParams.get_bool('jvePilot.settings.autoResume')
    self.minAccSetting = V_CRUISE_MIN_MS if self.settingsParams.get_bool("IsMetric") else V_CRUISE_MIN_IMPERIAL_MS
    self.round_to_unit = CV.MS_TO_KPH if self.settingsParams.get_bool("IsMetric") else CV.MS_TO_MPH
    self.steerNoMinimum = self.settingsParams.get_bool("jvePilot.settings.steer.noMinimum")

    self.autoFollowDistanceLock = None
    self.button_frame = 0
    self.last_target = 0

    # long
    self.accel_steady = 0
    self.last_brake = None
    self.last_torque = 0.
    self.under_accel_frame_count = 0
    self.vehicleMass = CP.mass
    self.max_gear = None

  def update(self, CC, CS):
    can_sends = []

    lkas_active = CC.latActive and self.lkas_control_bit_prev and CC.enabled

    # cruise buttons
    das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

    # Lane-less button
    if CS.button_pressed(ButtonType.lkasToggle, False):
      CC.jvePilotState.carControl.lkasButtonLight = not CC.jvePilotState.carControl.lkasButtonLight
      self.settingsParams.put("jvePilot.settings.lkasButtonLight",
                              "1" if CC.jvePilotState.carControl.lkasButtonLight else "0")
      CC.jvePilotState.notifyUi = True
    if self.frame % 10 == 0:
      new_msg = create_lkas_heartbit(self.packer, 1 if CC.jvePilotState.carControl.lkasButtonLight else 0, CS.lkasHeartbit)
      can_sends.append(new_msg)

    self.wheel_button_control(CC, CS, can_sends, CC.enabled, das_bus, CC.cruiseControl.cancel, CC.cruiseControl.resume)

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert, self.hud_count, CS.lkas_car_model, CS.auto_high_beam))
        self.hud_count += 1

    # steering
    # TODO: can we make this more sane? why is it different for all the cars?
    low_steer_models = self.CP.carFingerprint in PRE_2019
    lkas_control_bit = self.lkas_control_bit_prev
    if self.steerNoMinimum:
      lkas_control_bit = CC.enabled or low_steer_models
    elif CS.out.vEgo > self.CP.minSteerSpeed:
      lkas_control_bit = True
    elif self.CP.carFingerprint in RAM_CARS:
      if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
        lkas_control_bit = False
    elif not low_steer_models:
      if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
        lkas_control_bit = False

    # EPS faults if LKAS re-enables too quickly
    lkas_control_bit = lkas_control_bit and (self.frame > self.next_lkas_control_change)

    if not lkas_control_bit and self.lkas_control_bit_prev:
      self.next_lkas_control_change = self.frame + 200
    self.lkas_control_bit_prev = lkas_control_bit

    # steer torque
    new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
    if not lkas_active or not lkas_control_bit:
      apply_steer = 0
    self.apply_steer_last = apply_steer

    new_actuators = CC.actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX

    accel = self.acc(CC, CS, can_sends, CC.enabled)
    if accel is not None:
      new_actuators.accel = accel

    can_sends.append(create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit))

    self.frame += 1

    return new_actuators, can_sends

  def wheel_button_control(self, CC, CS, can_sends, enabled, das_bus, cancel, resume):
    button_counter = CS.button_counter
    if button_counter == self.last_button_frame:
      return
    self.last_button_frame = button_counter

    self.button_frame += 1

    if self.CP.openpilotLongitudinalControl:
      if CS.button_pressed(ButtonType.accOnOff, False):
        CS.longAvailable = not CS.longAvailable
        CS.longEnabled = False

      if CS.longAvailable:
        if cancel or CS.button_pressed(ButtonType.cancel) or CS.out.brakePressed:
          CS.longEnabled = False
        elif CS.button_pressed(ButtonType.accelCruise) or \
            CS.button_pressed(ButtonType.decelCruise) or \
            CS.button_pressed(ButtonType.resumeCruise):
          CS.longEnabled = True

      accDiff = None
      if CS.button_pressed(ButtonType.followInc, False):
        if CC.jvePilotState.carControl.accEco < 2:
          accDiff = 1
      elif CS.button_pressed(ButtonType.followDec, False):
        if CC.jvePilotState.carControl.accEco > 0:
          accDiff = -1
      if accDiff is not None:
        CC.jvePilotState.carControl.accEco += accDiff
        put_nonblocking("jvePilot.carState.accEco", str(CC.jvePilotState.carControl.accEco))
        CC.jvePilotState.notifyUi = True

    else:
      button_counter_offset = 1
      buttons_to_press = []
      if cancel:
        buttons_to_press = ['ACC_Cancel']
      elif not CS.button_pressed(ButtonType.cancel):
        follow_inc_button = CS.button_pressed(ButtonType.followInc)
        follow_dec_button = CS.button_pressed(ButtonType.followDec)

        if CC.jvePilotState.carControl.autoFollow:
          follow_inc_button = CS.button_pressed(ButtonType.followInc, False)
          follow_dec_button = CS.button_pressed(ButtonType.followDec, False)
          if (follow_inc_button and follow_inc_button.pressedFrames < 50) or \
             (follow_dec_button and follow_dec_button.pressedFrames < 50):
            CC.jvePilotState.carControl.autoFollow = False
            CC.jvePilotState.notifyUi = True
        elif (follow_inc_button and follow_inc_button.pressedFrames >= 50) or \
             (follow_dec_button and follow_dec_button.pressedFrames >= 50):
          CC.jvePilotState.carControl.autoFollow = True
          CC.jvePilotState.notifyUi = True

        if enabled and not CS.out.brakePressed:
          button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
          if button_counter_offset is not None:
            if self.auto_resume and (resume or (CS.out.standstill and not CS.out.cruiseState.enabled)):

                buttons_to_press = ["ACC_Resume"]

            elif CS.out.cruiseState.enabled:  # Control ACC
              buttons_to_press = [self.auto_follow_button(CC, CS), self.hybrid_acc_button(CC, CS)]

      buttons_to_press = list(filter(None, buttons_to_press))
      if buttons_to_press is not None and len(buttons_to_press) > 0:
        new_msg = create_wheel_buttons_command(self.packer, das_bus, button_counter + button_counter_offset, buttons_to_press)
        can_sends.append(new_msg)

  def hybrid_acc_button(self, CC, CS):
    experimental_mode = self.cachedParams.get_bool("ExperimentalMode", 1000) and self.cachedParams.get_bool('jvePilot.settings.lkasButtonLight', 1000)
    acc_boost = 0 if experimental_mode else 2 * CV.MPH_TO_MS  # add extra speed so ACC does the limiting
    target = self.acc_hysteresis(CC.jvePilotState.carControl.vTargetFuture + acc_boost)

    # Move the adaptive curse control to the target speed
    eco_limit = None
    if CC.jvePilotState.carControl.accEco == 1:  # if eco mode
      eco_limit = self.cachedParams.get_float('jvePilot.settings.accEco.speedAheadLevel1', 1000)
    elif CC.jvePilotState.carControl.accEco == 2:  # if eco mode
      eco_limit = self.cachedParams.get_float('jvePilot.settings.accEco.speedAheadLevel2', 1000)

    if eco_limit:
      target = min(target, CS.out.vEgo + (eco_limit * CV.MPH_TO_MS))

    # ACC Braking
    diff = CS.out.vEgo - target
    if diff > ACC_BRAKE_THRESHOLD and abs(target - CC.jvePilotState.carControl.vMaxCruise) > ACC_BRAKE_THRESHOLD:  # ignore change in max cruise speed
      target -= diff

    target = math.ceil(min(CC.jvePilotState.carControl.vMaxCruise, target) * self.round_to_unit)
    current = round(CS.out.cruiseState.speed * self.round_to_unit)
    minSetting = round(self.minAccSetting * self.round_to_unit)

    if target < current and current > minSetting:
      return 'ACC_Decel'
    elif target > current:
      return 'ACC_Accel'

  def auto_follow_button(self, CC, CS):
    if CC.jvePilotState.carControl.autoFollow:
      crossover = [0,
                   self.cachedParams.get_float('jvePilot.settings.autoFollow.speed1-2Bars', 1000) * CV.MPH_TO_MS,
                   self.cachedParams.get_float('jvePilot.settings.autoFollow.speed2-3Bars', 1000) * CV.MPH_TO_MS,
                   self.cachedParams.get_float('jvePilot.settings.autoFollow.speed3-4Bars', 1000) * CV.MPH_TO_MS]

      if CS.out.vEgo < crossover[1]:
        target_follow = 0
      elif CS.out.vEgo < crossover[2]:
        target_follow = 1
      elif CS.out.vEgo < crossover[3]:
        target_follow = 2
      else:
        target_follow = 3

      if self.autoFollowDistanceLock is not None and abs(crossover[self.autoFollowDistanceLock] - CS.out.vEgo) > AUTO_FOLLOW_LOCK_MS:
        self.autoFollowDistanceLock = None  # unlock

      if CC.jvePilotState.carState.accFollowDistance != target_follow and (self.autoFollowDistanceLock or target_follow) == target_follow:
        self.autoFollowDistanceLock = target_follow  # going from close to far, use upperbound

        if CC.jvePilotState.carState.accFollowDistance > target_follow:
          return 'ACC_Distance_Dec'
        else:
          return 'ACC_Distance_Inc'

  # T = (mass x accel x velocity x 1000)/(.105 x Engine rpm)
  def acc(self, CC, CS, can_sends, enabled):
    if not CS.longControl or self.frame % 2 != 0:
      return None

    can_sends.append(create_acc_1_message(self.packer, 0, self.frame / 2))
    can_sends.append(create_acc_1_message(self.packer, 2, self.frame / 2))

    if self.frame % 6 == 0:
      state = 0
      if CS.out.cruiseState.available:
        state = 2 if CS.out.cruiseState.enabled else 1
      can_sends.append(create_das_4_message(self.packer, 0, state, CC.jvePilotState.carControl.vMaxCruise))
      can_sends.append(create_das_4_message(self.packer, 2, state, CC.jvePilotState.carControl.vMaxCruise))

    if self.frame % 50 == 0:
      # tester present - w/ no response (keeps radar disabled)
      can_sends.append((0x753, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0))

    if self.frame % 100 == 0:
      can_sends.append(create_chime_message(self.packer, 0))
      can_sends.append(create_chime_message(self.packer, 2))

    if not enabled or not CS.out.cruiseState.enabled:
      self.last_brake = None
      self.last_torque = None
      self.max_gear = None

      can_sends.append(create_das_3_message(self.packer, self.frame / 2, 0, CS.out.cruiseState.available, CS.out.cruiseState.enabled, False, False, 8, False, 0))
      can_sends.append(create_das_3_message(self.packer, self.frame / 2, 2, CS.out.cruiseState.available, CS.out.cruiseState.enabled, False, False, 8, False, 2))

      return None

    #long
    self.accel = clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    override_request = CS.out.gasPressed or CS.out.brakePressed
    if not override_request:
      #braking
      if CC.actuators.accel < - 0.1:
        accel_req = 0
        decel_req = 0
        torque = None
        decel = self.acc_brake(self.accel)
        self.max_gear = 8
      #accelerating
      else:
        time_for_sample = 0.15
        torque_limits = 50
        drivetrain_efficiency = 0.85
        self.last_brake = None
        accel_req = 1
        decel_req = 0
        # delta_accel = CC.actuators.accel - CS.out.aEgo

        # distance_moved = ((delta_accel * time_for_sample**2)/2) + (CS.out.vEgo * time_for_sample)
        # torque = (self.CP.mass * delta_accel * distance_moved * time_for_sample)/((drivetrain_efficiency * CS.engineRpm * 2 * math.pi) / 60)

        # # force (N) = mass (kg) * acceleration (m/s^2)
        # force = self.CP.mass * delta_accel
        # # distance_moved (m) =  (acceleration(m/s^2) * time(s)^2 / 2) + velocity(m/s) * time(s)
        # distance_moved = ((delta_accel) * (time_for_sample**2))/2) + (CS.out.vEgo * time_for_sample)
        # # work (J) = force (N) * distance (m)
        # work = force * distance_moved
        # # Power (W)= work(J) * time (s)
        # power = work * time_for_sample
        # # torque = Power (W) / (RPM * 2 * pi / 60)
        # torque = power/((drivetrain_efficiency * CS.engineRpm * 2 * math.pi) / 60)
        desired_velocity = ((self.accel-CS.out.aEgo) * time_for_sample) + CS.out.vEgo
        # kinetic energy (J) = 1/2 * mass (kg) * velocity (m/s)^2
        # use the kinetic energy from the desired velocity - the kinetic energy from the current velocity to get the change in velocity
        kinetic_energy = ((self.CP.mass * desired_velocity **2)/2) - ((self.CP.mass * CS.out.vEgo**2)/2)
        # convert kinetic energy to torque
        # torque(NM) = (kinetic energy (J) * 9.55414 (Nm/J) * time(s))/RPM
        torque = (kinetic_energy * 9.55414 * time_for_sample)/(drivetrain_efficiency * CS.engineRpm + 0.001)
        torque = clip(torque, -torque_limits, torque_limits) # clip torque to -6 to 6 Nm for sanity

        if CS.engineTorque < 0 and torque > 0:
          #If the engine is producing negative torque, we need to return to a reasonable torque value quickly.
          # rough estimate of external forces in N
          total_forces = 650
          #torque required to maintain speed
          torque = (total_forces * CS.out.vEgo * 9.55414)/(CS.engineRpm * drivetrain_efficiency + 0.001)

        #If torque is positive, add the engine torque to the torque we calculated. This is because the engine torque is the torque the engine is producing.
        else:
          torque += CS.engineTorque

        decel = None

    else:
      self.last_torque = None
      self.last_brake = None
      decel_req = None
      accel_req = 0
      torque = None
      self.max_gear = 8

 
    can_sends.append(
      create_das_3_message(self.packer, self.frame / 2, 0,
                           CS.out.cruiseState.available,
                           CS.out.cruiseState.enabled,
                           accel_req,
                           torque,
                           self.max_gear,
                           decel_req,
                           decel))
    can_sends.append(
      create_das_3_message(self.packer, self.frame / 2, 2,
                           CS.out.cruiseState.available,
                           CS.out.cruiseState.enabled,
                           accel_req,
                           torque,
                           self.max_gear,
                           decel_req,
                           decel))



  def acc_brake(self, aTarget):
    brake_target = aTarget
    if self.last_brake is None:
      self.last_brake = min(0., brake_target / 2)
    else:
      tBrake = brake_target
      lBrake = self.last_brake
      if tBrake < lBrake:
        diff = min(BRAKE_CHANGE, (lBrake - tBrake) / 2)
        self.last_brake = max(lBrake - diff, tBrake)
      elif tBrake - lBrake > 0.01:  # don't let up unless it's a big enough jump
        diff = min(BRAKE_CHANGE, (tBrake - lBrake) / 2)
        self.last_brake = min(lBrake + diff, tBrake)
    return self.last_brake

  def acc_hysteresis(self, new_target):
    if new_target > self.last_target:
      self.last_target = new_target
    elif new_target < self.last_target - 0.75 * CV.MPH_TO_MS:
      self.last_target = new_target

    return self.last_target

