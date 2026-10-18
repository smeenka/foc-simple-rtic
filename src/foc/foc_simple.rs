use fixed::{types::{I16F16,I6F26, I32F32}} ;
use heapless::Vec;
use rtt_target::rprintln;

use crate::{
  EFocLimit, EFocMonitor, EFocSimpleError, EPid, Result, ShaftPosition, foc::{EDir, EFocMode}, tools::foc_pid::FocPid,
};
use crate::{COMMAND_QUEUE_RX, COMMAND_QUEUE_TX, EFocCommand};
use rtic_sync::channel::{Receiver, Sender};

pub struct FocSimple {
  // user control
  receiver: Receiver<'static, EFocCommand, COMMAND_QUEUE_TX>,
  sender_retour: Sender<'static, EFocCommand, COMMAND_QUEUE_RX>,
  enabled: bool,
  target: I16F16,
  // user request
  shaft_position_req: ShaftPosition,
  foc_mode: EFocMode,
  calibration_state: ECalibrateState,
  torque: I16F16, // target for torque in %
  pid_current_d: FocPid,
  pid_current_q: FocPid,
  pid_angle: FocPid,
  pid_velocity: FocPid,
  inner_loop_hz:i32,  // inner loop frequency in hz
  // angle sensor
  shaft_position_act: ShaftPosition,
  velocity: I16F16, // rad / s. Filtered with a lowpass filter of ca 10 Hrz
  nr_poles: i32,
  open_loop_velocity: bool,
  // current sensor
  // internal state
  electrical_offset: I16F16, // offset of the angle sensor with respect to the poles of the motor in radians.
  electrical_angle: I16F16,
  speed_req: I16F16,
  speed_acc: I16F16, // in rad/100 ms
  speed_act: I16F16,
  speed_10ms_sec : I16F16,
  speed_delta_inc: I6F26, // the increment at each step of the inner loop of the requested angle
  speed_delta_max: I6F26, // the maximum speed increment in angle mode
  // torque
  torque_limit_neg: I16F16,
  torque_limit_pos: I16F16,

  temp: I16F16,
  // hall sensor related state
  hall_error_count: usize,
  hall_step_idx: u8,  // the lowlevel hall state from 0 upto 5 are valid
  hall_step_base: u8, // After eah full state transition ( 6 hall steps, this will increment or decrement)
  hall_step_prev: u8,  
  hall_step_max: u8,  // nr_poles * 6
  hall_step_angle: I6F26, // TAU / (hall_state_max)  
  // monitoring
  monitor_set: [bool;7],
  monitor_sample: u32,
  monitor_count: i32,
  monitor_v_d: I16F16,
  monitor_v_q: I16F16,
  monitor_i_d: I16F16,
  monitor_i_q: I16F16,
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
  pub fn new(receiver: Receiver<'static, EFocCommand, COMMAND_QUEUE_TX>,sender_retour:Sender<'static, EFocCommand, COMMAND_QUEUE_RX>) -> FocSimple {
    FocSimple {
      receiver,
      sender_retour,
      enabled: false,
      target: I16F16::ZERO,
      // user request parameters
      shaft_position_req: ShaftPosition::new(),
      foc_mode: EFocMode::Idle,
      electrical_offset: I16F16::ZERO,
      electrical_angle: I16F16::ZERO,
      velocity: I16F16::ZERO, // in rad per second
      shaft_position_act: ShaftPosition::new(),
      nr_poles: 1,  
      inner_loop_hz: 5_000,
      open_loop_velocity: true,
      // internal state
      calibration_state: ECalibrateState::Init,
      pid_current_d: FocPid::default(),
      pid_current_q: FocPid::default(),
      pid_angle: FocPid::default(),
      pid_velocity: FocPid::default(),
      temp: I16F16::ZERO,
      speed_req: I16F16::ZERO,
      speed_acc: I16F16::ONE / 10, // in rad/100 ms
      speed_act: I16F16::ZERO,
      speed_10ms_sec :I16F16::ONE / 100, // 0.01 second       
      speed_delta_inc: I6F26::ONE/ 25000,
      speed_delta_max: I6F26::ONE/ 25000,
      // torque
      torque_limit_neg: I16F16::ONE/ -2,
      torque_limit_pos: I16F16::ONE/  2,
      torque: I16F16::ONE/4,

      // hall sensor related state
      hall_error_count: 9,
      hall_step_idx: 0,  // the lowlevel hall state from 0 upto 5 are valid
      hall_step_base: 0, // After eah full state transition ( 6 hall steps, this will increment or decrement)
      hall_step_max: 6,  // nr_poles * 6
      hall_step_prev: 0,
      hall_step_angle: I6F26::ONE/6, 
      monitor_set: [false;7],
      monitor_count:1,
      monitor_sample:0, // default OFF
      monitor_v_d: I16F16::ZERO,
      monitor_v_q: I16F16::ONE/10,
      monitor_i_d: I16F16::ONE/100,
      monitor_i_q: I16F16::ONE/1000,
    }
  }

  /// Set the target value
  /// Angle mode: set the angle in range 0 .. TAU
  /// Torque mode: set the torque in range torque_limit_neg .. torque_limit_pos
  pub fn set_target(&mut self, target: I16F16) {
    self.target = target;
    match self.foc_mode {
      EFocMode::Angle(_) => self.shaft_position_req.angle = I6F26::from_num(target).clamp(I6F26::ZERO, I6F26::TAU),
      EFocMode::Torque => self.torque = target.clamp(self.torque_limit_neg, self.torque_limit_pos),
      EFocMode::Velocity(_) => self.speed_req = target,
      _ => ()
    }
  }
  /// Angle mode: set the position with shaft position. Can be positive or negative
  /// Angle wil be set immediatly. Speed of change can be regulated with torque limit
  pub fn set_position(&mut self, position: ShaftPosition) {
    if let EFocMode::Angle(_) = self.foc_mode {
      self.shaft_position_req = position;
    }
  }
  /// only to be used for test function. Be carefull with setting the value manually!
  /// Incorrect use can damage the motor.
  pub fn set_electrical_offset(&mut self, offset: I16F16) {
    self.electrical_offset = offset;
  }
  pub fn set_nr_poles(&mut self, nr: usize) {
    if let EFocMode::Idle = self.foc_mode {
      self.nr_poles = nr as i32;
      self.hall_step_max = nr as u8 * 6;
      self.hall_step_angle = I6F26::TAU / (self.hall_step_max as i32) ;
    }
  }
  pub fn set_torque_limit(&mut self, torque: I16F16) {
    let torq = torque.clamp(I16F16::ZERO, I16F16::ONE);
    self.torque_limit_neg = -1 * torq;
    self.torque_limit_pos = torq;
    self.torque = self.torque.clamp(self.torque_limit_neg, self.torque_limit_pos);
  }
  /// return the busy flag, to see if the calibration is finished, or the target angle is reached
  pub fn is_idle(&self) -> bool {
    self.foc_mode == EFocMode::Idle
  }
  pub fn set_inner_loop_hz(&mut self, hz: usize) {
    self.inner_loop_hz = hz as i32;
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

  pub fn foc_mode(&mut self, mode: EFocMode, _do_set: bool) -> EFocMode {
    rprintln!("Set Foc mode:{:?}", mode);
    self.foc_mode = mode;
    match self.foc_mode {
      EFocMode::Calibration(param) => match param {
        Some(p) => {
          self.electrical_offset = p.zero;
          self.shaft_position_act.set_inversed(p.dir == EDir::Ccw);
          self.foc_mode = EFocMode::Idle;
        }
        None => {
          self.shaft_position_act.set_inversed(false);
          self.calibration_state = ECalibrateState::Init;
          if self.torque.abs() > 0.4 {
            self.torque = I16F16::ONE/4;
          }
        }
      },
      EFocMode::Angle(_param) => {
        self.pid_angle.set_integral_max(I16F16::ONE * 3);
        self.shaft_position_req = self.shaft_position_act.clone();
      }
      EFocMode::Velocity(open_loop) => {
        self.open_loop_velocity = open_loop;
        self.speed_req = I16F16::ZERO;
        self.speed_act = I16F16::ZERO;
        self.speed_delta_inc = I6F26::ZERO;
        self.shaft_position_req = self.shaft_position_act.clone();
        self.pid_velocity.set_integral_max(I16F16::ONE * 20);
      }
      EFocMode::Torque => {
        self.torque = I16F16::ZERO;
      }
      EFocMode::Idle => (),
      EFocMode::Error(_e) => (),
    }
    self.foc_mode
  }

  fn pid_param(&mut self, parm: EPid, do_set: bool) -> EPid{
    let result = match parm {
      EPid::CurrentD(p) => EPid::CurrentD(self.pid_current_d.pid_param(p, do_set)),
      EPid::CurrentQ(p) => EPid::CurrentQ(self.pid_current_q.pid_param(p, do_set)),
      EPid::Angle(p) => EPid::Angle(self.pid_angle.pid_param(p, do_set)),
      EPid::Velocity(p) => EPid::Velocity(self.pid_velocity.pid_param(p, do_set)),
    };
    result
  }
  fn enabled(&mut self, is_enabled: bool, do_set: bool) -> bool{
    if do_set {
      self.enabled = is_enabled;
    }
    self.enabled
  }

  fn limits(&mut self, limit: EFocLimit, do_set: bool) -> EFocLimit{
    let result = match limit {
      EFocLimit::Current(c) => {
        if do_set {
        }
        EFocLimit::Current(c)
      }
      EFocLimit::Voltage(t) => { 
        if do_set {
          self.set_torque_limit(t)
        }
        EFocLimit::Voltage(self.torque_limit_pos)
      }
      EFocLimit::Velocity(v) => {
        if do_set {
          self.speed_delta_max = self.calculate_speed_delta_inc(v)
        }
        EFocLimit::Voltage(I16F16::from_num(self.speed_delta_max) * self.inner_loop_hz)
      }
      EFocLimit::Acceleration(a) => {
        if do_set {
          self.speed_acc = a / 100; // update is done with 100 hrz
        }
        EFocLimit::Acceleration(self.speed_acc * 100)
      }
    };
    result
  }

  fn monitor(&mut self, monitor: EFocMonitor, do_set: bool) -> EFocMonitor{
    let result = match monitor {
      EFocMonitor::DownSample(c) => {
        if do_set {
          self.monitor_sample = c;
        }
        EFocMonitor::DownSample(self.monitor_sample)
      }
      EFocMonitor::Set(t) => { 
        if do_set {
          self.monitor_set = t
        }
        EFocMonitor::Set(t)
      }
      _ => monitor
    };
    result
  }
  fn exec_monitor(&mut self) {
    self.monitor_count = self.monitor_sample as i32;
    let mut result = Vec::new();
    for i in 0..7 {
      if self.monitor_set[i] {
        result.push(
          match i {
          6 => self.target,
          5 => self.monitor_v_q,
          4 => self.monitor_v_d,
          3 => self.monitor_i_q,
          2 => self.monitor_i_d,
          1 => self.velocity,
          0 => I16F16::from_num(self.shaft_position_act.angle),
          _ => I16F16::ZERO
        });
      } 
    }
    self.sender_retour.try_send(EFocCommand::Monitor(EFocMonitor::Sample(result), false)).unwrap();

  }


  /// update the state of the foc controller. as fast as possible
  /// Returned is a tuple of the electrical angle, and the requested torque
  pub fn inner_loop(&mut self) -> Result<(I16F16, I16F16)> {
    match self.foc_mode {
      EFocMode::Idle => Ok((self.electrical_angle, I16F16::ZERO)),
      EFocMode::Error(e) => Err(e),
      EFocMode::Calibration(_) => self.do_calibration(),
      EFocMode::Angle(_) => {
        // Compare actual position with requested
        self.monitor_v_d = self
          .pid_angle
          .update_position(&self.shaft_position_req, &self.shaft_position_act)
          .clamp(self.torque_limit_neg, self.torque_limit_pos);
        Ok((I16F16::from_num(self.electrical_angle), self.monitor_v_d))
      }
      EFocMode::Velocity(_) => {
        if self.open_loop_velocity {
            // note that in sensorless mode the shaft_position_req is the electrical angle of the shaft
            let delta_electrical_angle = self.nr_poles * self.speed_delta_inc;
            self.shaft_position_req.inc(delta_electrical_angle); // increment requested angle in rad/s
                                                                 // set the torque with the torque limit function
            let request_angle = I16F16::from_num(self.shaft_position_req.get_angle());
            self.monitor_v_d = self.torque;
            Ok((request_angle, self.torque))
        } else {
          // increment the requested shaft position, but only if the diff with the actual shaft position is not too big
          let diff = self.shaft_position_req.compare(&self.shaft_position_act);
          if diff.abs() < I16F16::PI {
            let delta_angle = self.speed_delta_inc;
            self.shaft_position_req.inc(delta_angle);
          }
            self.monitor_v_d = self
            .pid_velocity
            .update_position(&self.shaft_position_req, &self.shaft_position_act)
            .clamp(self.torque_limit_neg, self.torque_limit_pos);
          Ok((I16F16::from_num(self.electrical_angle), self.monitor_v_d))
        }
      }
      EFocMode::Torque => {
        self.monitor_v_d = self.torque;
        Ok((I16F16::from_num(self.electrical_angle), self.torque))
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

    if self.monitor_sample > 0 {  
      self.monitor_count -= 1;
      if self.monitor_count <= 0 {
        self.exec_monitor();
      }
    }
  }

  #[inline]
  fn update_speed(&mut self) {
    let req = self.speed_req;
    if self.speed_acc == 0 {
      self.set_speed_act(req);
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
      self.set_speed_act(act);
    }
  }
  // set the actual speed, and calculate the speed_delta, with high precision
  fn set_speed_act(&mut self, act: I16F16) {
    self.speed_act = act;
    self.speed_delta_inc = self.calculate_speed_delta_inc(act);
  }
  //  calculate the speed_delta, from the speed, with high precision
  fn calculate_speed_delta_inc(&mut self, speed: I16F16) -> I6F26 {
    let act_64 = I32F32::from_num(speed);
    // keep precision by doing the divide with 32 fractional bits
    let act_fraction = act_64 / (self.inner_loop_hz as i64);
    // transform to 26 bits fraction
    I6F26::from_num(act_fraction)
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
          self.shaft_position_req.inc((I6F26::ONE * 10)/ self.inner_loop_hz);
        }
      }
      ECalibrateState::FindOffset => {
        // state end condition after exact 2 electrical turns + 3/4 TAU
        if req.rotations > 2 && req.angle > 3 * I16F16::FRAC_TAU_4 {
          // determin electrical offset with current offset == 0
          let mut offset = I16F16::from_num(self.shaft_position_act.get_angle()) * self.nr_poles;
          // normalize to 0 .. TAU
          while offset > I16F16::TAU {
            offset -= I16F16::TAU;
          }
          self.electrical_offset = offset;
          self.calibration_state = ECalibrateState::ReturnToStart;
          rprintln!("Electrical offset:{}", self.electrical_offset);
        } else {
          // rotate with 10 rad/sec positive
          self.shaft_position_req.inc((I6F26::ONE * 10)/ self.inner_loop_hz);
        }
      }
      ECalibrateState::ReturnToStart => {
        // end conditions at start positon plu TAU/2
        if req.rotations == 0 && req.angle < I16F16::FRAC_TAU_2 {
          self.foc_mode = EFocMode::Idle;
          rprintln!("Calibraton finished");
        } else {
          // rotate with 10 rad/sec negative
          self.shaft_position_req.inc((I6F26::ONE * -10)/ self.inner_loop_hz);
        }
      }
    }
    Ok((I16F16::from_num(self.shaft_position_req.get_angle()), self.torque.abs()))
  }

  pub fn update_angle_from_sensor(&mut self, measured_angle: I16F16) {
    let angle = I6F26::from_num(measured_angle);
    self.shaft_position_act.update_shaft_angle(angle);
    self.electrical_angle = measured_angle * self.nr_poles - self.electrical_offset;
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
      self.shaft_position_act.update_shaft_angle(angle);
      self.electrical_angle = I16F16::from_num(angle) * self.nr_poles - self.electrical_offset;
    }
  }  
  // user interface commands
  pub fn check_receiver(&mut self) {
    if let Ok(message) = self.receiver.try_recv() {
      let m = match message {
          EFocCommand::Pid(pid, setter) => Some(EFocCommand::Pid(self.pid_param(pid, setter), false)),
          EFocCommand::MotionControl(mode, setter ) => Some(EFocCommand::MotionControl(self.foc_mode(mode, setter), false)),
          EFocCommand::Limit(limit, setter) => Some(EFocCommand::Limit(self.limits(limit, setter),false)),
          EFocCommand::NrPoles(n) => {
            self.set_nr_poles(n as usize);
            None}
          EFocCommand::Target(target, _torque, _velocity) =>{
            self.set_target(target);
            None
          }
          EFocCommand::ErrorCount => {
            rprintln!("Total error count:{}", self.hall_error_count);
            self.hall_error_count = 0;
            None
          }
          EFocCommand::EnableStatus(enable, setter)  => Some(EFocCommand::EnableStatus(self.enabled(enable, setter), false)),      
          EFocCommand::Monitor(m, setter) => Some(EFocCommand::Monitor(self.monitor(m,setter), false)),
      };
      if let Some(message) = m {
        self.sender_retour.try_send(message).unwrap();
      }
    }
  }
}
