use fixed::{types::{I16F16,I6F26, I48F16}} ;
use rtt_target::rprintln;

use crate::{
  EFocSimpleError, Result, ShaftPosition, foc::{ EFocCurrentMode, EFocMode, PwmDriver}, tools::foc_pid::FocPid,
};
use crate::{COMMAND_QUEUE_RX, COMMAND_QUEUE_TX, EFocCommand};
use crate::foc::foc_pwm::FocPwm;
use rtic_sync::channel::{Receiver, Sender};

pub struct FocSimple<D: PwmDriver> {
  // user control
  pub(crate) receiver: Receiver<'static, EFocCommand, COMMAND_QUEUE_TX>,
  pub(crate) sender_retour: Sender<'static, EFocCommand, COMMAND_QUEUE_RX>,

  // internal state for the velocity controller 
  pub(crate) speed_acc: I16F16, // in rad/100 ms
  pub(crate) speed_act: I16F16,
  pub(crate) speed_10ms_sec : I16F16,
  pub(crate) speed_delta_inc: I6F26, // the increment at each step of the inner loop of the requested angle
  pub(crate) speed_delta_max: I6F26, // the maximum speed increment in angle mode
  // output of the velocity controller
  pub(crate) velocity: I16F16, // rad / s. Filtered with a lowpass filter of ca 10 Hrz


  // input state for the mode controller
  pub(crate) target: I16F16,
  pub(crate) shaft_position_req: ShaftPosition,

  // internal state for the mode controller
  pub(crate) nr_poles: i32,
  pub(crate) inner_loop_hz:i32,  // inner loop frequency in hz
  pub(crate) shaft_position_act: ShaftPosition,
  pub(crate) electrical_offset: I16F16, // offset of the angle sensor with respect to the poles of the motor in radians.
  pub(crate) pid_angle: FocPid,
  pub(crate) pid_velocity: FocPid,
  pub(crate) foc_mode: EFocMode,
  pub(crate) angle_inversed: bool,
  pub(crate) temp: I16F16,
  pub(crate) calibration_state: ECalibrateState,
 
 // Output state for the mode controller, input state for the current controller
  pub(crate) voltage_request :I16F16,
  pub(crate) electrical_angle: I16F16,

  // internal state for the current controller
  pub(crate) foc_pwm: FocPwm<D>,
  pub(crate) current_mode: EFocCurrentMode, 
  pub(crate) voltage_q :I16F16,
  pub(crate) voltage_d :I16F16,
  pub(crate) current_q :I16F16,
  pub(crate) current_d :I16F16,
  pub(crate) voltage_q_limit_neg: I16F16,
  pub(crate) voltage_q_limit_pos: I16F16,
  pub(crate) pid_current_d: FocPid,
  pub(crate) pid_current_q: FocPid,

  // hall sensor related state
  pub(crate) hall_error_count: usize,
  hall_step_idx: u8,  // the lowlevel hall state from 0 upto 5 are valid
  hall_step_base: u8, // After eah full state transition ( 6 hall steps, this will increment or decrement)
  hall_step_prev: u8,  
  hall_step_max: u8,  // nr_poles * 6
  hall_step_angle: I6F26, // TAU / (hall_state_max)  
  // monitoring
  pub(crate) monitor_set: [bool;7],
  pub(crate) monitor_reload: u32,
  pub(crate) monitor_count: i32,
}

#[derive(Debug)]
#[repr(u8)]
pub(crate) enum ECalibrateState {
  Init = 0,
  FindDirection,
  FindOffset,
  ReturnToStart,
}

impl<D> FocSimple<D>
where
  D: PwmDriver {
  pub fn new(
    receiver: Receiver<'static, EFocCommand, COMMAND_QUEUE_TX>,
    sender_retour:Sender<'static, EFocCommand, COMMAND_QUEUE_RX>,
    foc_pwm: FocPwm<D>) -> FocSimple<D> {
    FocSimple {
      receiver,
      sender_retour,

      // internal state for the velocity controller 
      speed_acc: I16F16::ONE / 10, // in rad/100 ms
      speed_act: I16F16::ZERO,
      speed_10ms_sec :I16F16::ONE / 100, // 0.01 second       
      speed_delta_inc: I6F26::ONE/ 25000,
      speed_delta_max: I6F26::ONE/ 25000,
      // output of the velocity controller
      velocity: I16F16::ZERO, // in rad per second
      // input state for the mode controller
      target: I16F16::ZERO,
      shaft_position_req: ShaftPosition::new(),

      // internal state for the mode controller
      nr_poles: 1,  
      inner_loop_hz: 5_000,
      shaft_position_act: ShaftPosition::new(),
      electrical_offset: I16F16::ZERO,
      pid_angle: FocPid::default(),
      pid_velocity: FocPid::default(),
      foc_mode: EFocMode::Idle,
      temp: I16F16::ZERO,
      calibration_state: ECalibrateState::Init,
      angle_inversed: false,

      // Output state for the mode controller, input state for the current controller
      voltage_request :I16F16::ZERO,
      electrical_angle: I16F16::ZERO,
  
      // internal state for the current controller
      foc_pwm,
      current_mode: EFocCurrentMode::Voltage, 
      voltage_q :I16F16::ZERO,
      voltage_d :I16F16::ZERO,
      current_q :I16F16::ZERO,
      current_d :I16F16::ZERO,
      voltage_q_limit_neg: I16F16::ONE/-2,
      voltage_q_limit_pos: I16F16::ONE/2,
      pid_current_d: FocPid::default(),
      pid_current_q: FocPid::default(),
  
      // hall sensor related state
      hall_error_count: 9,
      hall_step_idx: 0,  // the lowlevel hall state from 0 upto 5 are valid
      hall_step_base: 0, // After eah full state transition ( 6 hall steps, this will increment or decrement)
      hall_step_max: 6,  // nr_poles * 6
      hall_step_prev: 0,
      hall_step_angle: I6F26::ONE/6, 
      // monitoring
      monitor_set: [false;7],
      monitor_count:1,
      monitor_reload:0, // default OFF
    }
  }

  /// Set the target value
  /// Angle mode: set the position. The actual position is never reset, so beware that after velocity or torque mode, the amount of rotations can be vast 
  /// Torque mode: set the torque in range torque_limit_neg .. torque_limit_pos
  pub fn set_target(&mut self, target: I16F16) {
    self.target = target;
    match self.foc_mode {
      EFocMode::Angle(_) => self.shaft_position_req.set_position(I48F16::from_num(target)),
      EFocMode::Torque => {
        self.target = target.clamp(self.voltage_q_limit_neg, self.voltage_q_limit_pos);
        self.voltage_request = self.target;
      }
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
  pub fn set_nr_poles(&mut self, nr: usize) {
    if let EFocMode::Idle = self.foc_mode {
      self.nr_poles = nr as i32;
      self.hall_step_max = nr as u8 * 6;
      self.hall_step_angle = I6F26::TAU / (self.hall_step_max as i32) ;
    }
  }
  pub fn set_voltage_limit(&mut self, voltage: I16F16) {
    let v = voltage.clamp(I16F16::ZERO, I16F16::ONE);
    self.voltage_q_limit_neg = -1 * v;
    self.voltage_q_limit_pos = v;
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

  /// update the state of the foc controller. as fast as possible
  /// Returned is a vector with the 3 pwm  values, in range -1..1
  pub fn update_foc(&mut self) -> Result<()> {
    // first do the foc mode (voltage, angle or torque)
    match self.foc_mode {
      EFocMode::Idle => self.voltage_request =  I16F16::ZERO,
      EFocMode::Error(e) => return Err(e),
      EFocMode::Calibration(_) => self.do_calibration()?,
      EFocMode::Angle(_) => {
        // Compare actual position with requested
        self.voltage_request = self
          .pid_angle
          .update_position(&self.shaft_position_req, &self.shaft_position_act)
          .clamp(self.voltage_q_limit_neg, self.voltage_q_limit_pos);
      }
      EFocMode::Velocity(true) => {
            // note that in sensorless mode the shaft_position_req is the electrical angle of the shaft
            let delta_electrical_angle = self.nr_poles * self.speed_delta_inc;
            self.shaft_position_req.inc(delta_electrical_angle); // increment requested angle in rad/s
                                                                 // set the torque with the torque limit function
            self.electrical_angle = I16F16::from_num(self.shaft_position_req.get_angle());
      }
      EFocMode::Velocity(false) => {
          // increment the requested shaft position, but only if the diff with the actual shaft position is not too big
          let diff = self.shaft_position_req.compare(&self.shaft_position_act);
          if diff.abs() < I16F16::PI {
            let delta_angle = self.speed_delta_inc;
            self.shaft_position_req.inc(delta_angle);
          }
            self.voltage_request = self
            .pid_velocity
            .update_position(&self.shaft_position_req, &self.shaft_position_act)
            .clamp(self.voltage_q_limit_neg, self.voltage_q_limit_pos);
      }
      EFocMode::Torque => {
      },
    }
    // be sure to limit the requested torque voltage to the given limits

    // then the current loop
    // sin and cos needed for both the forward loop (v_q, v_d) to pwm, as for the reverse (measured current to i_1, idt
    let (sin_angle, cos_angle) = cordic::sin_cos(self.electrical_angle);
    match self.current_mode {
      EFocCurrentMode::Voltage => {
        self.voltage_q = self.voltage_request;
      }
      EFocCurrentMode::CurrentFeedback => {
        self.voltage_q = self.voltage_request/4;
      }
    }
    // send the values to the output stage
    self.foc_pwm.update_pwm(sin_angle, cos_angle, self.voltage_q, self.voltage_d)
  }

  /// Calculate the direction of the sensor in relation to the direction of the motor
  /// If needed invert the direction of the sensor
  /// Returned is a tuple of electrical angle and torque
  fn do_calibration(&mut self) -> Result<()> {
    let req = self.shaft_position_req;
    let act = self.shaft_position_act;

    match self.calibration_state {
      ECalibrateState::Init => {
        self.shaft_position_req.reset();
        self.shaft_position_act.reset();
        self.angle_inversed = false;
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
              self.angle_inversed = true
            } else {
              rprintln!("Direction not inversed");
              self.angle_inversed = false
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
          rprintln!("Calibration finished");
        } else {
          // rotate with 10 rad/sec negative
          self.shaft_position_req.inc((I6F26::ONE * -10)/ self.inner_loop_hz);
        }
      }
    }
    self.electrical_angle = I16F16::from_num(self.shaft_position_req.get_angle());
    self.voltage_request = self.voltage_q_limit_pos/2;
    Ok(())
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
      let mut angle = self.hall_step_angle * hall_state_idx as i32;
      if self.angle_inversed {
        angle = I6F26::TAU - angle;
      }
      self.shaft_position_act.update_shaft_angle(angle);
      self.electrical_angle = I16F16::from_num(angle) * self.nr_poles - self.electrical_offset;
    }
  }  


}
