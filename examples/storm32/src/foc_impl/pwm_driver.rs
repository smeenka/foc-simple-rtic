use foc_simple_rtic::foc::PwmDriver;
use foc_simple_rtic::Result;
use stm32f1xx_hal::pac::{TIM2, TIM3, TIM4};
use stm32f1xx_hal::timer::PwmChannel;

pub struct LeftDriverImpl {
    a: PwmChannel<TIM3, 3>,
    b: PwmChannel<TIM3, 2>,
    c: PwmChannel<TIM3, 1>,
}

impl LeftDriverImpl {
    pub fn new(
        mut a: PwmChannel<TIM3, 3>,
        mut b: PwmChannel<TIM3, 2>,
        mut c: PwmChannel<TIM3, 1>,
    ) -> Self {
        a.enable();
        b.enable();
        c.enable();
        LeftDriverImpl { a, b, c }
    }
}

impl PwmDriver for LeftDriverImpl {
    fn set_pwm(&mut self, phases: [u16; 3]) -> Result<()> {
      self.a.set_duty(phases[0]);
      self.b.set_duty(phases[1]);
      self.c.set_duty(phases[2]);
      Ok(())
    }
    fn set_enabled(&mut self, enabled: bool) {}
}

pub struct RightDriverImpl {
    a: PwmChannel<TIM2, 1>,
    b: PwmChannel<TIM2, 2>,
    c: PwmChannel<TIM2, 3>,
}

impl RightDriverImpl {
    pub fn new(
        mut a: PwmChannel<TIM2, 1>,
        mut b: PwmChannel<TIM2, 2>,
        mut c: PwmChannel<TIM2, 3>,
    ) -> Self {
        a.enable();
        b.enable();
        c.enable();
        RightDriverImpl { a, b, c }
    }
}

impl PwmDriver for RightDriverImpl {
    fn set_pwm(&mut self, phases: [u16; 3]) -> Result<()> {
      self.a.set_duty(phases[0]);
      self.b.set_duty(phases[1]);
      self.c.set_duty(phases[2]);
      Ok(())
    }
    fn set_enabled(&mut self, enabled: bool) {}
}
