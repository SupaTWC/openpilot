#!/usr/bin/env python3
from cereal import car
from panda import Panda
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.chrysler.values import CAR, DBC, RAM_HD, RAM_DT
from selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params
from selfdrive.car.disable_ecu import disable_ecu

ButtonType = car.CarState.ButtonEvent.Type
GAS_RESUME_SPEED = 1.

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, experimental_long=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "chrysler"

    ret.dashcamOnly = candidate in RAM_HD

    ret.radarOffCan = DBC[candidate]['radar'] is None

    ret.steerActuatorDelay = 0.4
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.chrysler)]
    if candidate in RAM_HD:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_HD
    elif candidate in RAM_DT:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_DT

    ret.minSteerSpeed = 0  # m/s
    if candidate in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
      # TODO: allow 2019 cars to steer down to 13 m/s if already engaged.
      ret.minSteerSpeed = 17.5  # m/s 17 on the way up, 13 on the way down once engaged.

    # Chrysler
    if candidate in (CAR.PACIFICA_2017_HYBRID, CAR.PACIFICA_2018, CAR.PACIFICA_2018_HYBRID, CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020):
      ret.mass = 2242. + STD_CARGO_KG
      ret.wheelbase = 3.089
      ret.steerRatio = 16.2  # Pacifica Hybrid 2017
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      ret.lateralTuning.pid.kf = 0.00006

    # Jeep
    elif candidate in (CAR.JEEP_CHEROKEE, CAR.JEEP_CHEROKEE_2019):
      ret.mass = 2242 + STD_CARGO_KG
      ret.wheelbase = 2.91
      ret.steerRatio = 16.7
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      ret.lateralTuning.pid.kf = 0.00006

      ret.enableBsm = True

    # Ram
    elif candidate == CAR.RAM_1500:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.88
      ret.steerRatio = 16.3
      ret.mass = 2493. + STD_CARGO_KG
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.minSteerSpeed = 14.5
      if car_fw is not None:
        for fw in car_fw:
          if fw.ecu == 'eps' and fw.fwVersion[:8] in (b"68312176", b"68273275"):
            ret.minSteerSpeed = 0.

    elif candidate == CAR.RAM_HD:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.785
      ret.steerRatio = 15.61
      ret.mass = 3405. + STD_CARGO_KG
      ret.minSteerSpeed = 16
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, 1.0, False)

    else:
      raise ValueError(f"Unsupported car: {candidate}")

    if Params().get_bool("jvePilot.settings.steer.noMinimum"):
      ret.minSteerSpeed = -0.1

    ret.centerToFront = ret.wheelbase * 0.44

    # starting with reasonable value for civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)

    ret.openpilotLongitudinalControl = True  # kind of...
    ret.pcmCruiseSpeed = False  # Let jvePilot control the pcm cruise speed

    ret.enableBsm |= 720 in fingerprint[0]

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.openpilotLongitudinalControl:
      disable_ecu(logcan, sendcan, bus=0, addr=0x753, com_cont_req=b'\x28\x81\x01', response_offset = -0x280)

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low],
                                       gas_resume_speed=GAS_RESUME_SPEED, pcm_enable=False)

    if c.enabled and ret.brakePressed and ret.standstill and not self.disable_auto_resume:
      events.add(car.CarEvent.EventName.accBrakeHold)
    else:
      # Low speed steer alert hysteresis logic
      if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 0.5):
        self.low_speed_alert = True
      elif ret.vEgo > (self.CP.minSteerSpeed + 1.):
        self.low_speed_alert = False

      if self.low_speed_alert:
        events.add(car.CarEvent.EventName.belowSteerSpeed)

    if self.CS.button_pressed(ButtonType.cancel):
      events.add(car.CarEvent.EventName.buttonCancel)  # cancel button pressed
    elif ret.cruiseState.enabled and not self.CS.out.cruiseState.enabled:
      events.add(car.CarEvent.EventName.pcmEnable)  # cruse is enabled
    elif (not ret.cruiseState.enabled) and (ret.vEgo > GAS_RESUME_SPEED or (self.CS.out.cruiseState.enabled and (not ret.standstill))):
      events.add(car.CarEvent.EventName.pcmDisable)  # give up, too fast to resume

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    return self.CC.update(c, self.CS)
