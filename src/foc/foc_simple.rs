use fixed::{traits::Fixed, types::I16F16} ;
use rtt_target::rprintln;

use crate::{
  foc::{EDir, EFocMode},
  tools::foc_pid::FocPid,
  EFocAngle, EFocSimpleError, Result, ShaftPosition,
};
use crate::{COMMAND_QUEUE_LEN, EFocCommand};
use rtic_sync::channel::Receiver;

pub struct FocSimple {
  // user control
  receiver: Receiver<'static, EFocCommand, COMMAND_QUEUE_LEN>,
  // user request
  shaft_position_req: ShaftPosition,
  foc_mode: EFocMode,
  calibration_state: ECalibrateState,
  torque: I16F16, // target for torque in %
  target_pid: FocPid,
  inner_loop_hz:i32,  // inner loop frequency in hz
  // angle sensor
  shaft_position_act: ShaftPosition,
  angle: EFocAngle,
  velocity: I16F16, // rad / s. Filtered with a lowpass filter of ca 10 Hrz
  nr_poles: I16F16,
  // current sensor
  // internal state
  electrical_offset: I16F16, // offset of the angle sensor with respect to the poles of the motor in radians.
  speed_req: I16F16,
  speed_acc: I16F16, // in rad/100 ms
  speed_act: I16F16,
  speed_10ms_sec : I16F16,
  temp: I16F16,
  // hall sensor related state
  hall_error_count: usize,
  hall_step_idx: u8,  // the lowlevel hall state from 0 upto 5 are valid
  hall_step_base: u8, // After eah full state transition ( 6 hall steps, this will increment or decrement)
  hall_step_prev: u8,  
  hall_step_max: u8,  // nr_poles * 6
  hall_step_angle: I16F16, // TAU / (hall_state_max)  
}

#[derive(Debug)]
#[repr(u8)]
enum ECalibrateState {
  Init = 0,
  FindDirection,
  FindOffset,
  ReturnToStart,
}

impl FocSimple {
  pub fn new(receiver: Receiver<'static, EFocCommand, COMMAND_QUEUE_LEN>,) -> FocSimple {
    FocSimple {
      receiver,
      // user request parameters
      shaft_position_req: ShaftPosition::new(),
      torque: I16F16::ZERO,
      foc_mode: EFocMode::Idle,
      electrical_offset: I16F16::ZERO,
      velocity: I16F16::ZERO, // in rad per second
      shaft_position_act: ShaftPosition::new(),
      nr_poles: I16F16::ONE,
      inner_loop_hz: 5_000,
      // internal state
      calibration_state: ECalibrateState::Init,
      target_pid: FocPid::new(I16F16::ONE, I16F16::ONE, I16F16::ONE),
      angle: EFocAngle::SensorLess,
      temp: I16F16::ZERO,
      speed_req: I16F16::ZERO,
      speed_acc: I16F16::ONE / 10, // in rad/100 ms
      speed_act: I16F16::ZERO,
      speed_10ms_sec :I16F16::ONE / 100, // 0.01 second       
      // hall sensor related state
      hall_error_count: 9,
      hall_step_idx: 0,  // the lowlevel hall state from 0 upto 5 are valid
      hall_step_base: 0, // After eah full state transition ( 6 hall steps, this will increment or decrement)
      hall_step_max: 6,  // nr_poles * 6
      hall_step_prev: 0,
      hall_step_angle: I16F16::ONE/6, 
    }
  }

  /// Velocity mode: Set the speed in rad/sec, positive or negative
  /// Speed wil increment or decrement until this target value is reached
  pub fn set_speed(&mut self, speed: I16F16) {
    self.speed_req = speed;
  }
  /// Torque mode: set the torque in range -1 ..1
  /// Calibration mode: Set the torque for calibration
  /// Torque is set immediatly
  pub fn set_torque(&mut self, torque: I16F16) {
    self.torque = torque;
  }
  /// Angle mode: set the angle in range 0 .. TAU
  /// Angle wil be set immediatly. Speed of change can be regulated with torque limit
  pub fn set_angle(&mut self, angle: I16F16) {
    if let EFocMode::Angle(_) = self.foc_mode {
      self.shaft_position_req.angle = angle;
    }
  }
  /// Angle mode: set the position with shaft position. Can be positive or negative
  /// Angle wil be set immediatly. Speed of change can be regulated with torque limit
  pub fn set_position(&mut self, position: ShaftPosition) {
    if let EFocMode::Angle(_) = self.foc_mode {
      self.shaft_position_req = position;
    }
  }
  /// acceleration in rad/sec2
  pub fn set_acceleration(&mut self, acc: I16F16) {
    self.speed_acc = acc / 100; // update is done with 100 hrz
  }
  /// only to be used for test function. Be carefull with setting the value manually!
  /// Incorrect use can damage the motor.
  pub fn set_electrical_offset(&mut self, offset: I16F16) {
    self.electrical_offset = offset;
  }
  pub fn set_nr_poles(&mut self, nr: usize) {
    self.nr_poles = I16F16::from_num(nr);
    self.hall_step_max = nr as u8 * 6;
    self.hall_step_angle = I16F16::TAU / (self.hall_step_max as i32) ;
  }

  /// return the busy flag, to see if the calibration is finished, or the target angle is reached
  pub fn is_idle(&self) -> bool {
    self.foc_mode == EFocMode::Idle
  }

  pub fn get_velocity(&self) -> I16F16 {
    self.velocity
  }
  pub fn get_position_act(&self) -> &ShaftPosition {
    &self.shaft_position_act
  }
  pub fn get_position_req(&self) -> &ShaftPosition {
    &self.shaft_position_req
  }
  pub fn set_position_req(&mut self, req: ShaftPosition) {
    self.shaft_position_req = req;
  }

  pub fn set_foc_mode(&mut self, mode: EFocMode) -> Result<()> {
    rprintln!("Set Foc mode");
    self.foc_mode = mode;
    match self.foc_mode {
      EFocMode::Calibration(param) => match param {
        Some(p) => {
          self.electrical_offset = p.zero;
          self.shaft_position_act.set_inversed(p.dir == EDir::Ccw);
          self.foc_mode = EFocMode::Idle;
        }
        None => {
          // assume a angle sensor is present. 
          self.angle = EFocAngle::SensorValue(I16F16::ZERO);
          self.electrical_offset = I16F16::ZERO;
          self.shaft_position_act.set_inversed(false);
          self.calibration_state = ECalibrateState::Init;
          if self.torque < I16F16::ONE/10 {
            self.torque = I16F16::ONE/4;
          }

        }
      },
      EFocMode::Angle(param) => {
        self.target_pid = FocPid::new(param.p, param.i, param.d);
        self.target_pid.set_integral_max(I16F16::ONE * 3);

        self.shaft_position_req = self.shaft_position_act.clone();
      }
      EFocMode::Velocity(param) => {
        self.speed_req = I16F16::ZERO;
        self.speed_act = I16F16::ZERO;
        self.shaft_position_req = self.shaft_position_act.clone();
        self.target_pid = FocPid::new(param.p, param.i, param.d);
        self.target_pid.set_integral_max(I16F16::ONE * 20);
      }
      EFocMode::Torque(param) => {
        self.torque = I16F16::ZERO;
        self.target_pid = FocPid::new(param.p, param.i, param.d);
        self.target_pid.set_integral_max(I16F16::ONE);
      }
      EFocMode::Idle => (),
      EFocMode::Error(e) => return Err(e),
    }
    Ok(())
  }

  /// update the state of the foc controller. as fast as possible
  /// Returned is a tuple of the electrical angle, and the requested torque
  pub fn inner_loop(&mut self) -> Result<(I16F16, I16F16)> {
    let electrical_angle = self.nr_poles * self.shaft_position_act.angle - self.electrical_offset;
    match self.foc_mode {
      EFocMode::Idle => Ok((electrical_angle, I16F16::ZERO)),
      EFocMode::Error(e) => Err(e),
      EFocMode::Calibration(_) => match self.angle {
        EFocAngle::SensorLess => Err(EFocSimpleError::NoAngleSensor),
        _ => self.do_calibration(),
      },

      EFocMode::Angle(_) => {
        match self.angle {
          EFocAngle::SensorLess => Err(EFocSimpleError::NoAngleSensor),
          _ => {
            // Compare actual position with requested
            let torque = self
              .target_pid
              .update_position(&self.shaft_position_req, &self.shaft_position_act);
            Ok((electrical_angle, torque))
          }
        }
      }
      EFocMode::Velocity(_) => {
        match self.angle {
          EFocAngle::SensorLess => {
            // note that in sensorless mode the shaft_position_req is the electrical angle of the shaft
            let delta_electrical_angle = self.nr_poles * self.speed_act / self.inner_loop_hz;
            self.shaft_position_req.inc(delta_electrical_angle); // increment requested angle in rad/s
                                                                 // set the torque with the torque limit function
            let request_angle = self.shaft_position_req.get_angle();
            Ok((request_angle, I16F16::ONE / 4))
          }
          _ => {
            // increment the requested shaft position, but only if the diff with the actual shaft position is not too big
            let diff = self.shaft_position_req.compare(&self.shaft_position_act);
            if diff.abs() < I16F16::PI {
              let delta_angle = self.speed_act / self.inner_loop_hz;
              self.shaft_position_req.inc(delta_angle);
            }
            let requested_torque = self
              .target_pid
              .update_position(&self.shaft_position_req, &self.shaft_position_act);
            Ok((electrical_angle, requested_torque))
          }
        }
      }
      EFocMode::Torque(_) => match self.angle {
        EFocAngle::SensorLess => Err(EFocSimpleError::NoAngleSensor),
        _ => Ok((electrical_angle, self.torque)),
      },
    }
  }

  /// calculate the velocity. This function should be called exact each 10 ms.
  /// The low pass filter frequency is 10 hrz
  pub fn update_velocity(&mut self) {
    // update the actual requested speed  with a fixed frequency of preferable 100 hz
    self.update_speed();
    let position_delta = self.shaft_position_act.delta();
    let velocity_current = position_delta / self.speed_10ms_sec; // in rad per second
    // filter the velocity with a low pass filter
    self.velocity = (velocity_current + 19 * self.velocity) / 20;
  }

  #[inline]
  fn update_speed(&mut self) {
    let req = self.speed_req;
    if self.speed_acc == 0 {
      self.speed_act = req;
    } else {
      let mut act = self.speed_act;
      if act > req {
        act -= self.speed_acc;
        if act < req {
          act = req;
        }
      } else if act < req {
        act += self.speed_acc;
        if act > req {
          act = req;
        }
      } // do nothing if equal
      self.speed_act = act;
    }
  }

  /// Calculate the direction of the sensor in relation to the direction of the motor
  /// If needed invert the direction of the sensor
  /// Returned is a tuple of electrical angle and torque
  fn do_calibration(&mut self) -> Result<(I16F16, I16F16)> {
    let req = self.shaft_position_req;
    let act = self.shaft_position_act;

    match self.calibration_state {
      ECalibrateState::Init => {
        self.shaft_position_req.reset();
        self.shaft_position_act.reset();
        self.electrical_offset = I16F16::ZERO;
        self.calibration_state = ECalibrateState::FindDirection;
      }
      ECalibrateState::FindDirection => {
        // state end condition after exact 1 electrical turn
        if req.rotations > 1 {
          // check motor did move
          if act.rotations == 0 && act.angle == I16F16::ZERO {
            self.calibration_state = ECalibrateState::Init;
            self.foc_mode = EFocMode::Error(EFocSimpleError::NoMotorMovement);
            rprintln!("End condition No motor movement detected");
          } else {
            // set the direction
            if act.get_position() < 0 {
              // reverse the direction in the driver. Motor must run in the same dir as the sensor
              rprintln!("Direction inversed");
              self.shaft_position_act.set_inversed(true);
            } else {
              rprintln!("Direction not inversed");
              self.shaft_position_act.set_inversed(false);
            }

            self.calibration_state = ECalibrateState::FindOffset;
          }
        } else {
          // rotate with 10 rad/sec positive
          self.shaft_position_req.inc((I16F16::ONE * 10)/ self.inner_loop_hz);
        }
      }
      ECalibrateState::FindOffset => {
        // state end condition after exact 2 electrical turns + 3/4 TAU
        if req.rotations > 2 && req.angle > 3 * I16F16::FRAC_TAU_4 {
          // determin electrical offset with current offset == 0
          let offset = self.shaft_position_act.get_angle() * self.nr_poles;
          // normalize to 0 .. TAU
          self.electrical_offset = ShaftPosition::clamp(offset);
          self.calibration_state = ECalibrateState::ReturnToStart;
          rprintln!("Electrical offset:{}", self.electrical_offset);
        } else {
          // rotate with 10 rad/sec positive
          self.shaft_position_req.inc((I16F16::ONE * 10)/ self.inner_loop_hz);
        }
      }
      ECalibrateState::ReturnToStart => {
        // end conditions at start positon plu TAU/2
        if req.rotations == 0 && req.angle < I16F16::FRAC_TAU_2 {
          self.foc_mode = EFocMode::Idle;
          rprintln!("Calibraton finished");
        } else {
          // rotate with 10 rad/sec negative
          self.shaft_position_req.inc((I16F16::ONE * -10)/ self.inner_loop_hz);
        }
      }
    }
    Ok((self.shaft_position_req.get_angle(), self.torque.abs()))
  }

  // HALL related stuff
  /// calculate from the raw hall state the functional hall state 0-1-2-3-4-5
  /// incoming raw state is the hall sensor state in the first 3 bits. Valid transitions: 1, 3, 2 ,6 ,4 ,5
  pub fn update_angle_from_hall(&mut self, hall_state: i8) {
    if hall_state < 0  {
      self.hall_error_count += 1;
      rprintln!("Error Invalid hall state {}", hall_state,);
    } else {
      let prev = self.hall_step_prev;
      self.hall_step_prev = hall_state as u8;

      match hall_state {
        0 => {
          if prev == 5 {
            self.hall_step_base += 6;
            if self.hall_step_base >= self.hall_step_max {
              self.hall_step_base = 0;
            }
          } else if prev != 1 {
            rprintln!("Error change new:{} old:{}", hall_state, prev);
            self.hall_error_count += 1;
          }
        }
        5 => {
          if prev == 0 {
            if self.hall_step_base < 6 {
              self.hall_step_base = self.hall_step_max - 6;
            } else {
              self.hall_step_base -= 6;
            }
          } else if prev != 4 {
            rprintln!("Error change new:{} old:{}", hall_state, prev);
            self.hall_error_count += 1;
          }
        }
        _ => ()
        };
    
      let hall_state_idx = self.hall_step_base + hall_state as u8;
      // calculate the not interpolated angle
      let angle = self.hall_step_angle * hall_state_idx as i32;
      self.angle = EFocAngle::SensorValue(angle);
      self.shaft_position_act.update_shaft_angle(angle);
    }
  }  
  // user interface commands
  pub fn check_receiver(&mut self) {
    if let Ok(message) = self.receiver.try_recv() {
      match message {
          EFocCommand::FocMode(mode) => {
            _ = self.set_foc_mode(mode);
            ()
          }
          EFocCommand::ShaftPosition(shaft_pos) => self.set_position_req(shaft_pos),
          EFocCommand::Speed(speed) => self.set_speed(speed),
          EFocCommand::SpeedAcc(acc) => self.set_acceleration(acc),
          EFocCommand::Torque(t) => self.set_torque(t),
          EFocCommand::Angle(a) => self.set_angle(a),
          EFocCommand::TorqueLimit(_tl) => (), //self.foc_pwm.set_torque_limit(tl),
          EFocCommand::NrPoles(n) => self.set_nr_poles(n as usize),
          EFocCommand::ErrorCount => {
            rprintln!("Total error count:{}", self.hall_error_count);
            self.hall_error_count = 0
          }        
      }
    }
  }
}
