use fixed::{types::{I16F16,I6F26, I32F32}} ;
use heapless::Vec;
//use rtt_target::rprintln;
use crate::{EFocCommand, EFocMonitor, foc::{PwmDriver, foc_simple::FocSimple}};

impl<D> FocSimple<D>  where D: PwmDriver {
  pub fn update_100hz(&mut self){
    // calculate the actual speed from the acceleration and requested speed
    let req = self.target;
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
    // calculate and filter the velocity from the shaft velocity
    let position_delta = self.shaft_position_act.delta();
    let velocity_current = position_delta / self.speed_10ms_sec; // in rad per second
    // filter the velocity with a low pass filter
    self.velocity = (velocity_current + 19 * self.velocity) / 20;

    // monitor stuff
    if self.monitor_reload > 0 {  
      self.monitor_count -= 1;
      if self.monitor_count <= 0 {
        // reload monitor counter
        self.monitor_count = self.monitor_reload as i32;
        /* 
        let mut result = Vec::new();
        for i in 0..7 {
          if self.monitor_set[i] {
            _ = result.push(
              match i {
              0 => self.target,
              1 => self.voltage_request,
              2 => self.voltage_d,
              3 => self.current_q,
              4 => self.current_d,
              5 => self.velocity,
              6 => I16F16::from_num(self.shaft_position_act.angle),
              _ => I16F16::ZERO
            });
          } 
        }
        */
        //_ = self.sender_retour.try_send(EFocCommand::Monitor(EFocMonitor::Sample(result), false));
      }
    }
  }

  // set the actual speed, and calculate the speed_delta, with high precision
  fn set_speed_act(&mut self, act: I16F16) {
    self.speed_act = act;
    self.speed_delta_inc = self.calculate_speed_delta_inc(act);
  }
  //  calculate the speed_delta, from the speed, with high precision
  pub (crate)fn calculate_speed_delta_inc(&mut self, speed: I16F16) -> I6F26 {
    let act_64 = I32F32::from_num(speed);
    // keep precision by doing the divide with 32 fractional bits
    let act_fraction = act_64 / (self.inner_loop_hz as i64);
    // transform to 26 bits fraction
    I6F26::from_num(act_fraction)
  }

}