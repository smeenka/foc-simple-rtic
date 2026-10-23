use fixed::types::I16F16;
use foc::park_clarke::{self};

use crate::foc::{EModulation, PwmDriver};
use crate::{Result};

pub struct FocPwm<D: PwmDriver> {
  driver: D,
  enabled: bool,
  modulation: EModulation,
  max_pwm: u16,
  max_torque_neg: I16F16,
  max_torque_pos: I16F16,
}

impl<D> FocPwm<D>
where
  D: PwmDriver,
{
  pub fn new(mut driver: D, max_pwm: u16, max_torque: f32) -> Self {
    let max_torque = I16F16::from_num(max_torque.clamp(0.0, 1.0));
    driver.set_enabled(false);
    FocPwm {
      modulation: EModulation::Sinusoidal,
      enabled: false,
      driver,
      max_pwm,
      max_torque_neg: -1 * max_torque,
      max_torque_pos: max_torque,
    }
  }

  pub fn set_enabled(&mut self, enabled: bool) {
    self.driver.set_enabled(enabled);
    self.enabled = enabled;
  }
  pub fn enabled(&self) -> bool{
    self.enabled
  }
  pub fn set_modulation(&mut self, modulation: EModulation) {
    self.modulation = modulation
  }
  pub fn modulation(&self) -> EModulation {
    self.modulation.clone()
  }
 
  /// update the actual duty cycle values  for the pwm drivers, based on the given torque (v_q) and v_d and electrical angle
  // v_q: desired torque, v_d: desired flux
  pub fn update_pwm(&mut self, sin_angle: I16F16, cos_angle: I16F16, v_q: I16F16, v_d: I16F16) -> Result<()> {
    let torque = v_q.clamp(self.max_torque_neg, self.max_torque_pos);
    // Inverse Park transform
    let orthogonal_voltage = park_clarke::inverse_park(
      cos_angle,
      sin_angle,
      park_clarke::RotatingReferenceFrame { d: v_d, q: torque },
    );
    // convert to the pwm values, in range -1 .. 1, with the modulate function inline
    // Returns a value between -1 and 1 for each channel.
    // The content of this function is copied from the foc library, v0.3.0
    // because it was impossible to reuse the code
    let vectors = 
    match self.modulation {
      EModulation::Sinusoidal => {
        let voltages = park_clarke::inverse_clarke(orthogonal_voltage);
        [voltages.a, voltages.b, voltages.c]
      }
      EModulation::SpaceVector => {
        // Convert alpha/beta to x/y/z
        let sqrt_3_alpha = I16F16::SQRT_3 * orthogonal_voltage.alpha;
        let beta = orthogonal_voltage.beta;
        let x = beta;
        let y = (beta + sqrt_3_alpha) / 2;
        let z = (beta - sqrt_3_alpha) / 2;

        // Calculate which sector the value falls in
        let sector: u8 = match (x.is_positive(), y.is_positive(), z.is_positive()) {
          (true, true, false) => 1,
          (_, true, true) => 2,
          (true, false, true) => 3,
          (false, false, true) => 4,
          (_, false, false) => 5,
          (false, true, false) => 6,
        };

        // Map a,b,c values to three phase
        let (ta, tb, tc);
        match sector {
          1 | 4 => {
            ta = x - z;
            tb = x + z;
            tc = -x + z;
          }
          2 | 5 => {
            ta = y - z;
            tb = y + z;
            tc = -y - z;
          }
          3 | 6 => {
            ta = y - x;
            tb = -y + x;
            tc = -y - x;
          }
          _ => unreachable!("invalid sector"),
        }
        [ta, tb, tc]
      }
      /* next is not yet working, it will cause damage to the motor
      EModulation::Square => {
        let voltages = park_clarke::inverse_clarke(orthogonal_voltage);

        [voltages.a.signum(), voltages.b.signum(), voltages.c.signum()]
      }
      */
      EModulation::Trapezoidal => {
        let voltages = park_clarke::inverse_clarke(orthogonal_voltage);
        [
          (voltages.a * 2).round_to_zero().signum(),
          (voltages.b * 2).round_to_zero().signum(),
          (voltages.c * 2).round_to_zero().signum(),
        ]
      }
    };

    // convert to range 1 .. max_pwm value for setting the duty cycles on the timers
    let duties = vectors.map(|val| {
      (((val + I16F16::ONE) * (self.max_pwm as i32)) / 2)
        .round()
        .saturating_to_num::<u16>()
    });
    // and set duty_cyles on the timers, as implemented by the user
    self.driver.set_pwm(duties)
  }

}
