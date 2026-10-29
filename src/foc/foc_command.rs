use fixed::types::{I6F26, I16F16};

use crate::{EFocCommand, EFocLimit, EFocMonitor, EPid, foc::{EDir, EFocCurrentMode, EFocMode, PwmDriver, foc_simple::{ECalibrateState, FocSimple}}};
use rtt_target::rprintln;

impl<D> FocSimple<D>  where D: PwmDriver {
    // user interface commands
  pub fn handle_command_queue(&mut self) {
    if let Ok(message) = self.receiver.try_recv() {
      let m = match message {
          EFocCommand::Pid(pid, setter) => Some(EFocCommand::Pid(self.pid_param(pid, setter), false)),
          EFocCommand::MotionControl(mode, setter ) => Some(EFocCommand::MotionControl(self.foc_mode(mode, setter), false)),
          EFocCommand::CurrentControl(mode, setter ) => Some(EFocCommand::CurrentControl(self.current_mode(mode, setter), false)),
          EFocCommand::Modulation(m) => {
            self.foc_pwm.set_modulation(m);
            None
          },
          EFocCommand::SensorOffset(offset, setter ) => Some(EFocCommand::SensorOffset(self.sensor_offset(offset, setter),false)),
          EFocCommand::Limit(limit, setter) => Some(EFocCommand::Limit(self.limits(limit, setter),false)),
          EFocCommand::NrPoles(n) => {
            self.set_nr_poles(n as usize);
            None}
          EFocCommand::Target(target, _torque, _velocity) =>{
            let t:f32 = target.to_num();
            rprintln!("Set target to: {:?}", t);
            self.set_target(target);
            None
          }
          EFocCommand::ErrorCount => {
            //rprintln!("Total error count:{}", self.hall_error_count);
            self.hall_error_count = 0;
            None
          }
          EFocCommand::EnableStatus(enable, setter)  => Some(EFocCommand::EnableStatus(self.enabled(enable, setter), false)),      
          EFocCommand::Monitor(m, setter) => Some(EFocCommand::Monitor(self.monitor(m,setter), false)),
      };
      if let Some(message) = m {
        _ = self.sender_retour.try_send(message);
      }
    }
  }

  pub fn foc_mode(&mut self, mode: EFocMode, do_set: bool) -> EFocMode {
    if do_set {
      rprintln!("Set Foc mode:");
      self.foc_mode = mode;
      self.target = I16F16::ZERO;
      self.voltage_request = self.target;
      self.voltage_d = I16F16::ZERO;

      match &self.foc_mode {
        EFocMode::Calibration(param) => match param {
          Some(p) => {
            self.electrical_offset = p.zero;
            self.angle_inversed = p.dir == EDir::Ccw;
            self.foc_mode = EFocMode::Idle;
          }
          None => {
            self.angle_inversed = false;
            self.calibration_state = ECalibrateState::Init;
          }
        },
        EFocMode::Angle(_param) => {
          self.pid_angle.set_integral_max(I16F16::ONE * 3);
        }
        EFocMode::Velocity(_) => {
          self.speed_act = I16F16::ZERO;
          self.speed_delta_inc = I6F26::ZERO;
          self.pid_velocity.set_integral_max(I16F16::ONE * 20);
          self.voltage_request = self.voltage_q_limit_pos;
        }
        EFocMode::Torque => (),     
        EFocMode::Idle => (),
        EFocMode::Error(_e) => (),
      }
      self.shaft_position_req = self.shaft_position_act.clone();
    }
    self.foc_mode.clone()
  }


  pub fn current_mode(&mut self, mode: EFocCurrentMode, do_set: bool) -> EFocCurrentMode {
    if do_set {
      self.current_mode = mode;
    }
    self.current_mode.clone()
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
      self.foc_pwm.set_enabled(is_enabled);//self.enabled = is_enabled;
    }
    self.foc_pwm.enabled()
  }

  fn sensor_offset(&mut self, offset: I16F16, do_set: bool) -> I16F16{
    if do_set {
      self.electrical_offset = offset.abs();
      self.angle_inversed = offset.is_negative();
    }
    if self.angle_inversed {
      self.electrical_offset * -I16F16::ONE 
    } else {
      self.electrical_offset 
    }
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
          self.set_voltage_limit(t)
        }
        EFocLimit::Voltage(self.voltage_q_limit_pos)
      }
      EFocLimit::Velocity(v) => {
        if do_set {
          self.speed_delta_max = self.calculate_speed_delta_inc(v.abs())
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
          self.monitor_reload = c;
        }
        EFocMonitor::DownSample(self.monitor_reload)
      }
      EFocMonitor::Set(t) => { 
        if do_set {
          rprintln!("Set monitor to {:?}", t);
          self.monitor_set = t
        }
        EFocMonitor::Set(t)
      }
      _ => monitor
    };
    result
  }


}

