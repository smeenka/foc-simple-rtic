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
use stm32f1xx_hal::pac::{TIM1, TIM2, TIM3, TIM4};
use stm32f1xx_hal::timer::pwm_input::PwmInput;
use stm32f1xx_hal::timer::Tim2NoRemap;


systick_monotonic!(Mono, 1_000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
  use super::*;
  use fixed::types::I16F16;
  use foc_simple_rtic::foc::foc_simple::FocSimple;
  use foc_simple_rtic::foc::{EFocMode, EModulation};
  use foc_simple_rtic::EFocAngleSensor;
  use foc_simple_rtic::SensorAngle;  
  use rtic_monotonics::fugit::{Instant, Rate, RateExtU32};
  use storm32::foc_impl::{SensorAngleLeft, SensorAngleRight};
  use stm32f1::Periph;
  use stm32f1xx_hal::gpio::{PA15, PB10, PB12, PB14};
  use stm32f1xx_hal::i2c::I2c;
  use stm32f1xx_hal::i2c::Instance;
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

  #[shared]
  struct Shared {
    position: u16,
  }

  #[local]
  struct Local {
    led: PB12<Output<PushPull>>,
    timer1ms: CounterHz<TIM1>,
    counter: usize,
    sensor_left: SensorAngleLeft,
    sensor_right: SensorAngleRight
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
    rprintln!("Test as5600 angle sensors for both wheels");

    let mut gpioa = cx.device.GPIOA.split(&mut rcc);
    let mut gpiob = cx.device.GPIOB.split(&mut rcc);
    let mut gpioc = cx.device.GPIOC.split(&mut rcc);
    let mut afio = cx.device.AFIO.constrain(&mut rcc);

    let led = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

    // Instantiate TIM1 that will be used to create the 1 ms clock
    let mut timer1ms = cx.device.TIM1.counter_hz(&mut rcc);
    timer1ms.start(1000.Hz()).unwrap();
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
    let sensor_right = SensorAngleRight::new(i2c_r);

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
    let sensor_left = SensorAngleLeft::new(i2c_l);

    // left wheel
    let a7 = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let b0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let b1 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);
    let pins3 = (a7, b0, b1);


    // right wheel
    // a0 is not used for pwm, but needed to let the pwm2.split() succeed


    // Schedule the blinking task
    blink_task::spawn().ok();
    let counter = 0;
    (
      Shared { position: 0 },
      Local {
        led,
        timer1ms,
        counter,
        sensor_left,
        sensor_right
      },
    )
  }

  #[task(local = [led], priority = 1)]
  async fn blink_task(cx: blink_task::Context) {
    let mut instant = Mono::now();
    loop {
      cx.local.led.toggle();
      instant += 1.secs();
      Mono::delay_until(instant).await;

    }
  }

  // ticker task generating signals with 1000 hrz and 100 hrz
  #[task(binds=TIM1_UP, local=[timer1ms, counter, sensor_left, sensor_right], priority = 3)]
  fn ticker_task(cx: ticker_task::Context) {
    let mut counter = cx.local.counter;
    let left_angle = cx.local.sensor_left.get_angle();
    let right_angle = cx.local.sensor_right.get_angle();

    if *counter >= 100 {
      *counter = 0;
      if let Ok(la) = left_angle {
        if let Ok(ra) = right_angle {
          rprintln!("l:{:?} \t r:{:?}", la, ra);
        }
      }
    } else {
      *counter += 1;
    }
    cx.local.timer1ms.clear_interrupt(Event::Update);
  }
}
