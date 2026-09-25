#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use rtt_target::{rprintln, rtt_init_default};
use stm32f1xx_hal::gpio::Pin;
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::pac::TIM2;
use stm32f1xx_hal::timer::pwm_input::PwmInput;
use stm32f1xx_hal::timer::Tim2NoRemap;
use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;

/// PWM input monitor type
pub(crate) type PwmMonitor = PwmInput<TIM2>;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
  use rtic_monotonics::fugit::{Rate, RateExtU32};
  use fixed::types::I16F16;
  use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{
      Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA15, PA2, PA4, PB10, PB13, PB14, PB2, PC13,
    },
    pac::Peripherals,
    rcc::{self, Clocks, RccExt},
    time::KiloHertz,

    timer::pwm_input::{Configuration, ReadMode},
    timer::{Tim2NoRemap, Timer},
    timer::{CounterHz, Event}
  };
  use foc_simple_rtic::foc::foc_simple::FocSimple;
  use foc_simple_rtic::foc::FocHallSensor;

  use super::*;

  /* bring dependencies into scope */

  #[shared]
  struct Shared {
    foc_simple: FocSimple,
  }

  #[local]
  struct Local {
    led: PB2<Output<PushPull>>,
    button: PC13<Input<PullUp>>,
    hall1: PA15<Input<PullUp>>,
    hall2: PB14<Input<PullUp>>,
    hall3: PB10<Input<PullUp>>,
    timer2: CounterHz<TIM2>,
    rotations: i32,
    angle: I16F16
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

    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);
    rprintln!("Test hall sensors and EXTI interrupt with user button on PC13");

    let mut gpioa = cx.device.GPIOA.split(&mut rcc);
    let mut gpiob = cx.device.GPIOB.split(&mut rcc);
    let mut gpioc = cx.device.GPIOC.split(&mut rcc);

    // Setup LED
    let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);

    // disable jtag
    let (pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let mut button = gpioc.pc13.into_pull_up_input(&mut gpioc.crh);
    button.make_interrupt_source(&mut afio);
    button.enable_interrupt(&mut cx.device.EXTI);
    button.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

    let mut hall1 = pa15.into_pull_up_input(&mut gpioa.crh);
    hall1.make_interrupt_source(&mut afio);
    hall1.enable_interrupt(&mut cx.device.EXTI);
    hall1.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

    let mut hall2 = gpiob.pb14.into_pull_up_input(&mut gpiob.crh);
    hall2.make_interrupt_source(&mut afio);
    hall2.enable_interrupt(&mut cx.device.EXTI);
    hall2.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

    let mut hall3 = gpiob.pb10.into_pull_up_input(&mut gpiob.crh);
    hall3.make_interrupt_source(&mut afio);
    hall3.enable_interrupt(&mut cx.device.EXTI);
    hall3.trigger_on_edge(&mut cx.device.EXTI, Edge::RisingFalling);

    // Instantiate TIM2 that will be used to create the a fast ticker, simulating the pwm  clock
    let mut timer2 = cx.device.TIM2.counter_hz(&mut rcc);
    timer2.start(25000.Hz()).unwrap();
    // Generate an interrupt when the timer expires
    timer2.listen(Event::Update);

    // Schedule the blinking task
    blink::spawn().ok();

    let mut foc_simple = FocSimple::new();
    foc_simple.set_nr_poles(4);
    (
      Shared {
        foc_simple,
      },
      Local {
        led,
        button,
        hall1,
        hall2,
        hall3,
        timer2,
        rotations:0,
        angle: I16F16::ZERO
      },
    )
  }

  #[task(local = [led])]
  async fn blink(cx: blink::Context) {
    loop {
      cx.local.led.toggle();
      Mono::delay(1000.millis()).await;
    }
  }

// ticker task generating signals with 5000 hrz and 100 hrz
  #[task(binds=TIM2, local=[timer2, rotations, angle], shared=[foc_simple], priority = 4)]
  fn timer_task(mut cx: timer_task::Context) {
    cx.shared.foc_simple.lock(|foc| {
      let shaft = foc.get_position_act();
      let rot = shaft.get_rotations();
      let ang = shaft.get_angle();
      if rot != *cx.local.rotations || ang != *cx.local.angle {
        *cx.local.rotations = rot;
        *cx.local.angle = ang;
        rprintln!("r:{} a:{}", rot, ang);
      }
    });
    cx.local.timer2.clear_interrupt(Event::Update);
  }


  /// note that  C13 is connected to EXTI3
  #[task(binds=EXTI15_10,  local=[button, hall1, hall2, hall3], shared=[foc_simple])]
  fn button_pressed(mut ct: button_pressed::Context) {
    let h1 = ct.local.hall1;
    let h2 = ct.local.hall2;
    let h3 = ct.local.hall3;
    if ct.local.button.check_interrupt() {
      ct.local.button.clear_interrupt_pending_bit();
      rprintln!("EXTI15_10 Button on C13  pressed");
    } else {
      h1.clear_interrupt_pending_bit();
      h2.clear_interrupt_pending_bit();
      h3.clear_interrupt_pending_bit();
      let mut phase = 0;
      if h1.is_high() {
        phase |= 1;
      }
      if h2.is_high() {
        phase |= 2;
      }
      if h3.is_high() {
        phase |= 4;
      }
      let state =FocHallSensor::calc_hall_state(phase);
      ct.shared.foc_simple.lock(|foc| {
        foc.update_angle_from_hall(state);
        let s = foc.get_position_act();
        rprintln!("Hall state now:{} rotations: {} angle:{}", phase, s.get_rotations(), s.get_angle());
      });
    }
  }
}
