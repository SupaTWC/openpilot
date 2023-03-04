from opendbc.can.packer import CANPacker
from common.realtime import DT_CTRL
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, create_cruise_buttons, acc_command, create_acc_1_message, create_das_4_message, create_chime_message#, create_lkas_heartbit
from selfdrive.car.chrysler.values import CAR, RAM_CARS, RAM_DT, RAM_HD, PAC_HYBRID, CarControllerParams
from cereal import car
from common.numpy_fast import clip, interp
from common.conversions import Conversions as CV
from common.params import Params, put_nonblocking
from cereal import car

from common.op_params import opParams
from selfdrive.car.chrysler.chryslerlonghelper import cluster_chime, accel_hysteresis, accel_rate_limit, \
  cruiseiconlogic, setspeedlogic, SET_SPEED_MIN, DEFAULT_DECEL, STOP_GAS_THRESHOLD, START_BRAKE_THRESHOLD, \
  STOP_BRAKE_THRESHOLD, START_GAS_THRESHOLD, CHIME_GAP_TIME, ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX
          

ButtonType = car.CarState.ButtonEvent.Type
LongCtrlState = car.CarControl.Actuators.LongControlState
# braking
BRAKE_CHANGE = 0.06

GearShifter = car.CarState.GearShifter

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.frame = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

    # long
    self.go_sent = 0
    self.accel = 0
    self.resume_pressed = 0
    self.button_frame = 0
    self.last_button_frame = 0
    self.op_params = opParams()

    #hybrid long
    self.accel_lim_prev = 0.
    self.accel_lim = 0.
    self.accel_steady = 0.
    self.accel_active = False
    self.decel_active = False
    self.go_req = False    
    self.stop_req = False

  def update(self, CC, CS):
    can_sends = []
    if self.CP.carFingerprint in RAM_CARS:
      lkas_active = CC.latActive and not CS.lkasdisabled
    else: lkas_active = CC.latActive and self.lkas_control_bit_prev and CC.enabled

    # cruise buttons
    if self.CP.carFingerprint in RAM_CARS:
      if (CS.button_counter != self.last_button_frame):
        das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0
        self.last_button_frame = CS.button_counter
        if self.CP.carFingerprint in RAM_CARS:
          if CS.cruise_cancel:
            can_sends.append(create_cruise_buttons(self.packer, CS.button_counter, das_bus, CS.cruise_buttons, cancel=True))
          else:
            can_sends.append(create_cruise_buttons(self.packer, CS.button_counter, das_bus, CS.cruise_buttons, cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

        # ACC cancellation
        elif CC.cruiseControl.cancel:
          can_sends.append(create_cruise_buttons(self.packer, CS.button_counter+1, das_bus, CS.cruise_buttons, cancel=True))

        # ACC resume from standstill
        elif CC.cruiseControl.resume:
          can_sends.append(create_cruise_buttons(self.packer, CS.button_counter+1, das_bus, CS.cruise_buttons, resume=True))
    else: #PAC/JEEP
      if CS.button_pressed(ButtonType.accOnOff, False):
        CS.longAvailable = not CS.longAvailable
        CS.longEnabled = False

      if CS.longAvailable:
        if CS.button_pressed(ButtonType.cancel) or CS.out.brakePressed or (CS.longEnabled and not CC.enabled):
          CS.longEnabled = False
        elif (CS.button_pressed(ButtonType.accelCruise) or \
            CS.button_pressed(ButtonType.decelCruise) or \
            CS.button_pressed(ButtonType.resumeCruise)) and CS.out.gearShifter == GearShifter.drive:
          CS.longEnabled = True
    # steering
    if self.frame % 2 == 0:
      
      lkas_control_bit = self.lkas_control_bit_prev
      # TODO: can we make this more sane? why is it different for all the cars?
      if self.CP.carFingerprint in RAM_DT:
        if CS.out.vEgo >= self.CP.minEnableSpeed and CS.out.vEgo <= self.CP.minEnableSpeed + 0.5:
          lkas_control_bit = True
        if (self.CP.minEnableSpeed >= 14.5)  and (CS.out.gearShifter != GearShifter.drive) :
          lkas_control_bit = False
      elif CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.carFingerprint in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
        lkas_control_bit = CC.enabled
        # if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
        #   lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_HD:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200) and not CS.out.steerFaultTemporary and not CS.out.steerFaultPermanent

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame

      # steer torque
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
      if not lkas_active or not lkas_control_bit or not self.lkas_control_bit_prev:
        apply_steer = 0
      self.apply_steer_last = apply_steer
      self.lkas_control_bit_prev = lkas_control_bit

      can_sends.append(create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit))
      #LONG
      if self.CP.carFingerprint not in RAM_CARS: #placeholder for oplong enabled
        if self.CP.carFingerprint not in PAC_HYBRID:
          self.accel = clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    
          if CC.actuators.accel < -0.05: #- self.op_params.get('brake_threshold'):
            accel_req = False
            decel_req = False
            torque = None
            if CS.out.vEgo > 1:
              decel = self.accel * 1.1
            else: decel = self.accel
            max_gear = 8
            self.go_sent = 0
            self.resume_pressed = 0

            
          elif CS.out.gasPressed:
            accel_req = False
            decel_req = False
            torque = CS.engineTorque
            decel = None
            max_gear = 9
            self.go_sent = 10
            self.resume_pressed = 0

          else:
            torque_at_1 = 25 #at accel == 1.0
            max_torque = 38
            
            decel_req = False
            max_gear = 9
            if CS.out.vEgo < 0.1 and self.accel > 0:
              torque = max(15,((self.accel) * torque_at_1)*1)
            # elif CS.out.vEgo < 0.2 and self.accel > 0: 
            #   torque = ((self.accel) * torque_at_1)*1
            # elif CS.out.vEgo < 5.3 and self.accel > 0: 
            #   torque = ((self.accel) * torque_at_1)*1
            else: torque = (self.accel- max(CS.out.aEgo,0)) * torque_at_1
            # if CS.out.vEgo > 5: 
            # if torque < 0: #send acc_go when torque is > 0 again
            #   self.go_sent = 0
            if CS.out.vEgo > CC.hudControl.setSpeed * 0.9 and torque > 0: 
              torque *=0.4
            elif CS.out.vEgo > 16 and torque > 0:
              torque *= 0.55  
            elif CS.out.vEgo > 9 and torque > 0:
              torque *= 0.7

            torque = clip(torque,-10, max_torque)
            # if (self.go_sent < 10 and self.accel >0):
            #if torque > 0 and (CS.out.vEgo < 0.1 or self.go_sent < 10):
            if torque>3:
              accel_req = 1 
              #self.go_sent +=1
            else: accel_req = 0
            if CS.engineTorque < 0 and torque > 0:
              torque = 14
            else:
              torque += CS.engineTorque
              torque = max(round(torque,2), -1) #Min total engine torque requested 
            decel = None
            
            
          
          override_request = CS.out.brakePressed or not CS.longEnabled or not CS.out.cruiseState.enabled
          if override_request:
            decel_req = None
            accel_req = 0
            torque = None
            max_gear = 9
            decel = 4
            self.go_sent = 0
            self.resume_pressed = 0

            
          can_sends.append(acc_command(self.packer, self.frame / 2, 0,
                              CS.out.cruiseState.available,
                              CS.longEnabled,
                              accel_req,
                              torque,
                              max_gear,
                              decel_req,
                              decel,
                              0, 1))
          can_sends.append(acc_command(self.packer, self.frame / 2, 2,
                              CS.out.cruiseState.available,
                              CS.longEnabled,
                              accel_req,
                              torque,
                              max_gear,
                              decel_req,
                              decel,
                              0,1))
          can_sends.append(create_acc_1_message(self.packer, 0, self.frame / 2))
          can_sends.append(create_acc_1_message(self.packer, 2, self.frame / 2))

          # if self.frame % 10 == 0:
          #   new_msg = create_lkas_heartbit(self.packer, 0, CS.lkasHeartbit)
          #   can_sends.append(new_msg)
        elif self.CP.carFingerprint in PAC_HYBRID:
          ACCEL_TO_NM = 1200
          self.accel_lim_prev = self.accel_lim
          self.decel_val = DEFAULT_DECEL
          self.trq_val = 20

          long_stopping = CC.actuators.longControlState == LongCtrlState.stopping

          apply_accel = CC.actuators.accel if CS.longEnabled else 0

          accmaxBp = [20, 30, 50]
          accmaxhyb = [ACCEL_MAX, 1., .5]
          hybridStandstill = CS.out.vEgo < 0.001
          if long_stopping and hybridStandstill and not CS.out.gasPressed:
            self.stop_req = True
          else:
            self.stop_req = False

          if not self.stop_req and hybridStandstill and CS.longEnabled:
            self.go_req = True
          else:
            self.go_req = False

          apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady)
          accel_max_tbl = interp(1, accmaxBp, accmaxhyb)

          apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, accel_max_tbl)

          self.accel_lim = apply_accel
          apply_accel = accel_rate_limit(self.accel_lim, self.accel_lim_prev, hybridStandstill)

          if CS.longEnabled and not CS.out.gasPressed and not self.go_req and\
                  (self.stop_req
                  or (apply_accel <= min(20/ACCEL_TO_NM, START_BRAKE_THRESHOLD))
                  or (self.decel_active and ((CS.out.brake > 10.) or (1 < 0.)) and
                      (apply_accel < max((20 + 20.)/ACCEL_TO_NM, STOP_BRAKE_THRESHOLD)))):
            self.decel_active = True
            self.decel_val = apply_accel
            if self.decel_val_prev > self.decel_val and not self.done:
              self.decel_val = accel_rate_limit(self.decel_val, self.decel_val_prev, hybridStandstill)
              self.accel = self.decel_val
            else:
              self.done = True

            self.decel_val_prev = self.decel_val
          else:
            self.decel_active = False
            self.done = False
            self.decel_val_prev = CS.out.aEgo

          if CS.longEnabled and not CS.out.brakePressed and not (hybridStandstill and (self.stop_req or self.decel_active)) and\
                  (apply_accel >= max(START_GAS_THRESHOLD, (20 + 20.)/ACCEL_TO_NM)
                  or self.accel_active and not self.decel_active and apply_accel > (20 - 20.)/ACCEL_TO_NM):
            self.trq_val = apply_accel * ACCEL_TO_NM
            self.accel = apply_accel

            if 300 > self.trq_val > 20:
              self.accel_active = True
            else:
              self.trq_val = 20
              self.accel_active = False
          else:
            self.accel_active = False

          torque = 1547.75
          self.enabled_prev = CS.longEnabled
          can_sends.append(acc_command(self.packer, self.frame / 2, 0,
                              CS.out.cruiseState.available,
                              CS.longEnabled,
                              self.go_req,
                              torque,
                              0,
                              self.decel_active,
                              self.decel_val,
                              0, 1))
          can_sends.append(acc_command(self.packer, self.frame / 2, 2,
                              CS.out.cruiseState.available,
                              CS.longEnabled,
                              self.go_req,
                              torque,
                              0,
                              self.decel_active,
                              self.decel_val,
                              0,1))
          can_sends.append(create_acc_1_message(self.packer, 0, self.frame / 2, self.accel_active, self.trq_val))
          can_sends.append(create_acc_1_message(self.packer, 2, self.frame / 2, self.accel_active, self.trq_val))
#COMMON LONG COMMANDS for Pacifica/Jeep
        if self.frame % 6 == 0:
          state = 0
          if CS.out.cruiseState.available:
            state = 2 if CS.out.cruiseState.enabled else 1 #1/2 for regular cc, 3/4 for ACC
          can_sends.append(create_das_4_message(self.packer, 0, state, CC.hudControl.setSpeed)) #need to double check setSpeed
          can_sends.append(create_das_4_message(self.packer, 2, state, CC.hudControl.setSpeed))

        if self.frame % 50 == 0:
          # tester present - w/ no response (keeps radar disabled)
          can_sends.append((0x753, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0))
          
        if self.frame % 100 == 0:
          can_sends.append(create_chime_message(self.packer, 0))
          can_sends.append(create_chime_message(self.packer, 2))
          
      
    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert, self.hud_count, CS.lkas_car_model, CS))
        self.hud_count += 1

    #resume button control
    # if (CS.out.vEgo < 0.6 and self.accel > 0.1):

    #   #if self.accel > 0 and (CS.out.vEgo < 0.1 or CS.accBrakePressed):
    #   if CS.button_counter % 6 == 0:
    #     if (CS.button_counter != self.last_button_frame):
    #       self.last_button_frame = CS.button_counter
    #       can_sends.append(create_cruise_buttons(self.packer, CS.button_counter+1, 0, CS.cruise_buttons, resume=True))
    #jve resume button control
    button_counter = CS.button_counter
    if button_counter != self.last_button_frame:
      self.last_button_frame = button_counter
      self.button_frame += 1
      button_counter_offset = 1
      if (CS.out.vEgo < 0.01 and CS.accBrakePressed):
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          # can_sends.append(create_wheel_buttons_command(self.packer, 0, CS.button_counter + button_counter_offset, "ACC_Resume"))
          can_sends.append(create_cruise_buttons(self.packer, CS.button_counter+button_counter_offset, 0, CS.cruise_buttons, resume=True))

    self.frame += 1

    new_actuators = CC.actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX

    return new_actuators, can_sends

