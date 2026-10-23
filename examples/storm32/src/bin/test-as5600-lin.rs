#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use rtt_target::{rprint, rprintln, rtt_init_default};
use stm32f1xx_hal::gpio::Pin;
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::pac::{TIM1, TIM2, TIM3, TIM4, TIM6};
use stm32f1xx_hal::timer::pwm_input::PwmInput;
use stm32f1xx_hal::timer::Tim2NoRemap;
use fixed::types::{I16F16, I6F26};

systick_monotonic!(Mono, 1_000);
// test the linearity of the as5600 sensor
#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
  use super::*;
  use fixed::types::I16F16;
  use foc_simple_rtic::foc::foc_simple::FocSimple;
  use foc_simple_rtic::foc::foc_pwm::FocPwm;
  use foc_simple_rtic::foc::{EModulation };
  use foc_simple_rtic::{EFocAngleSensor, SensorAngle };
  use rtic_monotonics::fugit::{Instant, Rate, RateExtU32};
  use storm32::foc_impl::pwm_driver::{LeftDriverImpl, RightDriverImpl};
  use storm32::foc_impl::{FocSerialImpl, SensorAngleLeft, SensorAngleRight, SensorCurrentImpl};
  use stm32f1::Periph;
  use stm32f1xx_hal::afio::usart1::Rx;
  use stm32f1xx_hal::gpio::{PA15, PB10, PB12, PB14};
  use stm32f1xx_hal::i2c::I2c;
  use stm32f1xx_hal::i2c::Instance;
  use stm32f1xx_hal::pac::usart1::RegisterBlock;
  use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;
  use stm32f1xx_hal::time::Bps;
  use stm32f1xx_hal::timer::CounterHz;
  use stm32f1xx_hal::timer::Event;
  use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB2, PC13},
    pac::Peripherals,
    rcc::{self, Clocks, RccExt},
    time::KiloHertz,
    timer::pwm_input::{Configuration, ReadMode},
    timer::{Channel, PwmExt, Tim1NoRemap, Tim2NoRemap, Tim3NoRemap, Tim4NoRemap, Timer},
  };
  use rtic_sync::{channel::*, make_channel};

  #[shared]
  struct Shared {
  }

  #[local]
  struct Local {
    led: PB12<Output<PushPull>>,
    pwm_l: FocPwm<LeftDriverImpl>,
    pwm_r: FocPwm<RightDriverImpl>,
    sensor_l: SensorAngleLeft,
    sensor_r: SensorAngleRight,
    timer1ms: CounterHz<TIM1>,
    counter: usize,
    angle: I6F26
  }

  #[init]
  fn init(mut cx: init::Context) -> (Shared, Local) {
    // Setup clocks
    let mut flash = cx.device.FLASH.constrain();
    let mut rcc = cx
      .device
      .RCC
      .freeze(rcc::Config::hsi().sysclk(72.MHz()).pclk1(36.MHz()), &mut flash.acr);

    // Initialize Mono interrupt & clocks
    Mono::start(cx.core.SYST, 36_000_000);

    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);
    rprintln!("Test command shell with 2 foc motors loaded");

    let mut gpioa = cx.device.GPIOA.split(&mut rcc);
    let mut gpiob = cx.device.GPIOB.split(&mut rcc);
    let mut gpioc = cx.device.GPIOC.split(&mut rcc);
    let mut afio = cx.device.AFIO.constrain(&mut rcc);

    let led = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

    // Instantiate TIM1 that will be used to create the 5 Khz  clock
    let mut timer1ms = cx.device.TIM1.counter_hz(&mut rcc);
    timer1ms.start(5000.Hz()).unwrap();
    // Generate an interrupt when the timer expires
    timer1ms.listen(Event::Update);

    // disable jtag
    let (pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // angle sensors
    // Configure I2C1 for right wheel
    let scl = gpiob.pb6;
    let sda = gpiob.pb7;
    let mut i2c_r = I2c::new(
      cx.device.I2C1,
      (scl, sda),
      stm32f1xx_hal::i2c::Mode::standard(400.kHz()),
      &mut rcc,
    );
    //type is: I2c<Periph<RegisterBlock, 1073763328>>
    // wrap as5600 object to make the SensorAngle interface available
    let sensor_r = SensorAngleRight::new(i2c_r);

    // Configure I2C2 for left wheel
    let scl = gpiob.pb10;
    let sda = gpiob.pb11;
    let mut i2c_l = I2c::new(
      cx.device.I2C2,
      (scl, sda),
      stm32f1xx_hal::i2c::Mode::standard(400.kHz()),
      &mut rcc,
    );
    // wrap as5600 object to make the SensorAngle interface available
    let sensor_l = SensorAngleLeft::new(i2c_l);


    // left wheel
    let a7 = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let b0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let b1 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);
    let pins3 = (a7, b0, b1);

    let mut pwm3 = cx
      .device
      .TIM3
      .pwm_hz::<Tim3NoRemap, _, _>(pins3, &mut afio.mapr, 25.kHz(), &mut rcc);

    let channels3 = pwm3.split();

    // Enable clock on each of the channels
    let max = channels3.0.get_max_duty();
    rprintln!("Max duty:{}", max);
    let pwm_driver_l = LeftDriverImpl::new(channels3.2, channels3.1, channels3.0);
    let pwm_l = FocPwm::new(pwm_driver_l,  max, 0.4);

    // right wheel
    // a0 is not used for pwm, but needed to let the pwm2.split() succeed
    // a1 connected to m2B
    let a1 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let a2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let a3 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);
    let pins3 = (a1, a2, a3);
    let pwm2 = cx
      .device
      .TIM2
      .pwm_hz::<Tim2NoRemap, _, _>(pins3, &mut afio.mapr, 25.kHz(), &mut rcc);

    let channels2 = pwm2.split();

    let pwm_driver_r = RightDriverImpl::new(channels2.0, channels2.1, channels2.2);
    let pwm_r = FocPwm::new(pwm_driver_r,  max, 0.4);

    // Schedule the blinking task
    blink_task::spawn().ok();
    hz100_task::spawn().ok();

    let counter = 0;
    (
      Shared { },
      Local {
        led,
        pwm_l,
        pwm_r,
        sensor_l,
        sensor_r,
        timer1ms,
        counter,
        angle: I6F26::ZERO
      },
    )
  }

  #[task(local = [led], priority = 2)]
  async fn blink_task(cx: blink_task::Context) {
    let mut instant = Mono::now();
    loop {
      cx.local.led.toggle();
      instant += 1.secs();
      Mono::delay_until(instant).await;

    }
  }
  #[task( priority = 2)]
  async fn hz100_task(mut cx: hz100_task::Context) {
    let mut instant = Mono::now();

    loop {
      instant += 10.millis();
      Mono::delay_until(instant).await;
    }
  }


  // ticker task generating signals with 5000 hrz
  #[task(binds=TIM1_UP, local=[timer1ms, sensor_l, sensor_r, pwm_l, pwm_r, counter, angle],  priority = 3)]
  fn ticker_task(mut cx: ticker_task::Context) {
    let mut counter = cx.local.counter;
    let mut angle = cx.local.angle;
    let torque = I16F16::ONE/5;
    let nr_poles = -7;

 
    *angle += I6F26::ONE/5000;
    if *angle > I6F26::TAU {
      *angle = I6F26::ZERO;
    }
      let angle_16 = I16F16::from_num(*angle);
      let (sin_angle, cos_angle) = cordic::sin_cos(angle_16);      
    _ = cx.local.pwm_l.update_pwm(sin_angle, cos_angle, torque, I16F16::ZERO);
    _ = cx.local.pwm_r.update_pwm(sin_angle, cos_angle, torque, I16F16::ZERO);

    if * counter > 49 {
      let measured_angle_l = cx.local.sensor_l.get_angle().unwrap();
      let measured_angle_r = cx.local.sensor_r.get_angle().unwrap();
      let d_l = measured_angle_l - angle_16;
      let d_r = measured_angle_r - angle_16;
      rprintln!(",angle:,{}, \tleft: meas:,{}, \td:,{},  \t\tright: meas:,{}, \td:,{} ", angle, measured_angle_l, d_l, measured_angle_r, d_r);
      //rprintln!("a:,angle");
      *counter = 0;
    } else {
      *counter += 1;
    }
    // high speed inner loop for left
    cx.local.timer1ms.clear_interrupt(Event::Update);
  }
}
