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
 use foc_simple_rtic::EFocLimit;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
  use super::*;
  use fixed::types::I16F16;
  use foc_simple_rtic::foc::foc_simple::FocSimple;
  use foc_simple_rtic::foc::EFocMode;
  use foc_simple_rtic::foc::{foc_pwm::FocPwm, EModulation, FocHallSensor};
  use foc_simple_rtic::{EFocCommand, COMMAND_QUEUE_RX, COMMAND_QUEUE_TX};
  use f103_cb::foc_impl::pwm_driver::PwmDriverImpl;
  use rtic_monotonics::fugit::{Instant, Rate, RateExtU32};
  use stm32f1xx_hal::gpio::{PA15, PB10, PB14};
  use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB2, PC13},
    pac::Peripherals,
    rcc::{self,Clocks, RccExt},
    time::KiloHertz,
    timer::{Channel, PwmExt, Tim1NoRemap, Tim2NoRemap, Timer, Event},
    timer::{CounterHz}
  };
  use rtic_sync::{channel::*, make_channel};
  use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;  /* bring dependencies into scope */

  #[shared]
  struct Shared {
    foc_simple: FocSimple<PwmDriverImpl>
  }

  #[local]
  struct Local {
    led: PB2<Output<PushPull>>,
    hall1: PA15<Input<PullUp>>,
    hall2: PB14<Input<PullUp>>,
    hall3: PB10<Input<PullUp>>,
    timer2: CounterHz<TIM2>,
    sender: Sender<'static, EFocCommand, COMMAND_QUEUE_TX>

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

    // Initialize Mono interrupt & clocks
    Mono::start(cx.core.SYST, 36_000_000);

    let channels = rtt_init_default!();
    rtt_target::set_print_channel(channels.up.0);
    rprintln!("init");

    let mut gpioa = cx.device.GPIOA.split(&mut rcc);
    let mut gpiob = cx.device.GPIOB.split(&mut rcc);
    let mut gpioc = cx.device.GPIOC.split(&mut rcc);

    let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);

    // disable jtag
    let (pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // Instantiate TIM2 that will be used to create the a fast ticker, simulating the pwm  clock
    let mut timer2 = cx.device.TIM2.counter_hz(&mut rcc);
    timer2.start(5000.Hz()).unwrap();
    // Generate an interrupt when the timer expires
    timer2.listen(Event::Update);

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

    // pwm channels
    let a0 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
    let b0 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let c0 = gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh);
    let pins = (a0, b0, c0);

    // channel enable
    let ea0 = gpioc.pc10.into_push_pull_output(&mut gpioc.crh);
    let eb0 = gpioc.pc11.into_push_pull_output(&mut gpioc.crh);
    let ec0 = gpioc.pc12.into_push_pull_output(&mut gpioc.crh);

    let mut pwm = cx
      .device
      .TIM1
      .pwm_hz::<Tim1NoRemap, _, _>(pins, &mut afio.mapr, 25.kHz(), &mut rcc);
    pwm.enable(Channel::C1);
    pwm.enable(Channel::C2);
    pwm.enable(Channel::C3);
    let channels = pwm.split();
    // Enable clock on each of the channels
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


    let mut foc_pwm = FocPwm::new(pwm_driver, max, 0.4);
    foc_pwm.set_enabled(true);

    let (sender, receiver) = make_channel!(EFocCommand, COMMAND_QUEUE_TX);
    let (retour_sender, retour_receiver) = make_channel!(EFocCommand, COMMAND_QUEUE_RX);


    let mut foc_simple = FocSimple::new(receiver, retour_sender, foc_pwm);
    foc_simple.set_nr_poles(4);

    // Schedule the blinking task
    blink_task::spawn().ok();
    motor_task::spawn().ok();
    ticker_task::spawn().ok();

    (
      Shared { foc_simple },
      Local {
        led,
        hall1,
        hall2,
        hall3,
        sender,
        timer2
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

  // task checking the incoming commands from the queue
  #[task(shared=[foc_simple], priority = 1)]
  async fn ticker_task(mut cx: ticker_task::Context) {
    rprintln!("Starting ticker task");
    let mut now = Mono::now();
    loop {
      cx.shared.foc_simple.lock(|foc| {
        foc.update_100hz();
        foc.handle_command_queue();
      });
      now += 10.millis();
      Mono::delay_until(now).await;
    }
  }


  // motor task, waiting for incoming signals for 1 ms, 10 ms, hall sensor and commands
  #[task(local = [sender], priority = 2)]
  async fn motor_task(cx: motor_task::Context) {
    rprintln!("BLCD motor  with sinusiode driver, 1/4 torque, 1 rad/s");
    let mut sender = cx.local.sender;
    sender.try_send(EFocCommand::MotionControl(EFocMode::Calibration(None), true)).ok();
    rprintln!("Calibration in progress");
    Mono::delay_until(Mono::now() + 5.secs()).await;
    
    rprintln!("Calibration finished");
    sender.try_send(EFocCommand::MotionControl(EFocMode::Velocity(false), true)).ok();
    sender.try_send(EFocCommand::Limit(EFocLimit::Acceleration(I16F16::ONE * 10), true)).ok();
    loop {
        let mut instant = Mono::now();
        sender.try_send(EFocCommand::Target(I16F16::TAU, None,None)).ok();
        rprintln!("Motor 60 RPM");
        Mono::delay_until(instant+ 10.secs()).await;

        sender.try_send(EFocCommand::Target(I16F16::ZERO, None,None)).ok();
        rprintln!("Motor stopping");
        Mono::delay_until(instant+ 15.secs()).await;

        sender.try_send(EFocCommand::Target(-1 * I16F16::TAU, None,None)).ok();
        rprintln!("Motor negative 60 RPM");
        Mono::delay_until(instant+ 20.secs()).await;

        sender.try_send(EFocCommand::Target(I16F16::ZERO, None,None)).ok();
        rprintln!("Motor stopped");
        Mono::delay_until(instant+ 25.secs()).await;
    }
  }

  //  hall sensor
  #[task(binds=EXTI15_10,  local=[hall1, hall2, hall3 ], shared=[foc_simple], priority = 3)]
  fn hall_sensor(mut cx: hall_sensor::Context) {
    let h1 = cx.local.hall1;
    let h2 = cx.local.hall2;
    let h3 = cx.local.hall3;
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
    cx.shared.foc_simple.lock(|foc| {
      foc.update_angle_from_hall(state);
    });
  }
  // ticker task generating signals with 25 Khrz 
  #[task(binds=TIM2, local=[timer2],shared=[foc_simple], priority = 4)]
  fn timer_task(mut cx: timer_task::Context) {
    cx.shared.foc_simple.lock(|foc| {
      match  foc.update_foc(){
        Ok(_) => (),
        Err(e) => rprintln!("Error:{:?}", e)
      };
    });
    cx.local.timer2.clear_interrupt(Event::Update);
  }

}
