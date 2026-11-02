#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use defmt::info;
use foc_simple_rtic::foc::PwmDriver;
use panic_rtt_target as _;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use rtt_target::{rprintln, rtt_init, ChannelMode};
use stm32f1xx_hal::gpio::Pin;
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::pac::TIM2;
use stm32f1xx_hal::timer::pwm_input::PwmInput;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
  use f103_cb::foc_impl::pwm_driver::PwmDriverImpl;
  use fixed::types::I16F16;
  use foc_simple_rtic::foc::{foc_pwm::FocPwm, EModulation};
  use rtic_monotonics::fugit::{Rate, RateExtU32};
  use stm32f1xx_hal::pac::TIM1;
  use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;
  use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB2, PC13},
    pac::Peripherals,
    rcc::{self, Clocks, RccExt},
    time::KiloHertz,
    timer::pwm_input::{Configuration, ReadMode},
    timer::CounterHz,
    timer::{Channel, Event, PwmExt, Tim1NoRemap, Tim2NoRemap, Timer},
  };

  use super::*;

  /* bring dependencies into scope */

  #[shared]
  struct Shared {
    foc_pwm: FocPwm<PwmDriverImpl>,
  }

  #[local]
  struct Local {
    led: PB2<Output<PushPull>>,
    timer2: CounterHz<TIM2>,
    angle: I16F16,
  }

  #[init]
  fn init(mut cx: init::Context) -> (Shared, Local) {
    // Setup clocks
    let mut flash = cx.device.FLASH.constrain();
    let mut rcc = cx
      .device
      .RCC
      .freeze(rcc::Config::hsi().sysclk(72.MHz()).pclk1(36.MHz()), &mut flash.acr);
    let mut afio = cx.device.AFIO.constrain(&mut rcc);

    // Initialize the systick interrupt & obtain the token to prove that we did
    Mono::start(cx.core.SYST, 36_000_000);

    let channels = rtt_init! {
      up: {
          0: { size: 256,  mode: ChannelMode::NoBlockSkip, name: "terminal", section: ".rtt_data" }
          1: { size: 1024, mode: ChannelMode::NoBlockSkip, name: "defmt", section: ".rtt_data" }
      }
      down: {
          0: { size: 16, name: "terminal", section: ".rtt_data" }
      }
      section_cb: ".rtt_data"
    };
    rtt_target::set_print_channel(channels.up.0);
    rtt_target::set_defmt_channel(channels.up.1);
    rprintln!("Test foc_pwm module");

    let mut gpioa = cx.device.GPIOA.split(&mut rcc);
    let mut gpiob = cx.device.GPIOB.split(&mut rcc);
    let mut gpioc = cx.device.GPIOC.split(&mut rcc);

    // onboard led of the blcd driver
    let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);

    // pwm channels
    let a0 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
    let b0 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let c0 = gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh);
    let pins = (a0, b0, c0);

    // channel enable
    let ea0 = gpioc.pc10.into_push_pull_output(&mut gpioc.crh);
    let eb0 = gpioc.pc11.into_push_pull_output(&mut gpioc.crh);
    let ec0 = gpioc.pc12.into_push_pull_output(&mut gpioc.crh);

    // disable jtag
    //t (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // Configure TIM1 into PWM output mode.
    let mut pwm = cx
      .device
      .TIM1
      .pwm_hz::<Tim1NoRemap, _, _>(pins, &mut afio.mapr, 25.kHz(), &mut rcc);
    pwm.enable(Channel::C1);
    pwm.enable(Channel::C2);
    pwm.enable(Channel::C3);
    let channels = pwm.split();

    // Figure out the max duty cycle of the channels
    let max = channels.0.get_max_duty();
    rprintln!("Max duty:{}", max);

    let mut pwm_driver = PwmDriverImpl::new(
      channels.0,
      channels.1,
      channels.2,
      ea0.erase(),
      eb0.erase(),
      ec0.erase(),
    );
    pwm_driver.set_enabled(true);
    let mut foc_pwm = FocPwm::new(pwm_driver, max, 0.4);

    // Instantiate TIM2 that will be used to create the a fast ticker, simulating the pwm  clock
    let mut timer2 = cx.device.TIM2.counter_hz(&mut rcc);
    timer2.start(25000.Hz()).unwrap();
    // Generate an interrupt when the timer expires
    timer2.listen(Event::Update);

    // Schedule the blinking task
    blink_task::spawn().ok();
    motor_task::spawn().ok();

    (
      Shared { foc_pwm },
      Local {
        led,
        timer2,
        angle: I16F16::ZERO,
      },
    )
  }

  #[task(local = [led])]
  async fn blink_task(cx: blink_task::Context) {
    loop {
      cx.local.led.toggle();
      Mono::delay(1000.millis()).await;
    }
  }
  #[task(shared=[foc_pwm])]
  async fn motor_task(mut cx: motor_task::Context) {
    rprintln!("motor  with sinusoidal driver, 1/4 torque, one turn per second");
    loop {
      // 2. Run the FOC algorithm
      //cx.local.foc_pwm.update(curr_angle, I16F16::ONE, None).unwrap();
      Mono::delay(5.secs()).await;
      rprintln!("Disable motor");
      cx.shared.foc_pwm.lock(|pwm| {
        pwm.set_enabled(false);
      });
      Mono::delay(5.secs()).await;
      rprintln!("Enable motor");
      cx.shared.foc_pwm.lock(|pwm| {
        pwm.set_enabled(true);
      });
    }
  }
  // ticker task generating signals with 25 Khrz
  #[task(binds=TIM2, local=[timer2, angle], shared=[foc_pwm], priority = 4)]
  fn timer_task(mut cx: timer_task::Context) {
    let mut ang = *cx.local.angle;
    ang += I16F16::TAU / 1000;
    if ang >= I16F16::TAU {
      ang -= I16F16::TAU;
    }
    *cx.local.angle = ang;

    let (sin_angle, cos_angle) = cordic::sin_cos(ang);

    // 2. Run the FOC algorithm
    cx.shared.foc_pwm.lock(|pwm| {
      pwm.update_pwm(sin_angle, cos_angle, I16F16::ONE, I16F16::ZERO);
    });
    cx.local.timer2.clear_interrupt(Event::Update);
  }
}
