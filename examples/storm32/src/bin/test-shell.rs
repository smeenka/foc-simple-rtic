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
  use foc_simple_rtic::foc::foc_pwm::FocPwm;
  use foc_simple_rtic::foc::{EFocMode, EModulation };
  use foc_simple_rtic::{EFocAngleSensor, SensorAngle, COMMAND_QUEUE_RX, COMMAND_QUEUE_TX, EFocCommand, FocCommandLine,};
  use rtic_monotonics::fugit::{Instant, Rate, RateExtU32};
  use storm32::foc_impl::pwm_driver::{LeftDriverImpl, RightDriverImpl};
  use storm32::foc_impl::{FocSerialImpl, SensorAngleLeft, SensorAngleRight, SensorCurrentImpl};
  use stm32f1::Periph;
  use stm32f1xx_hal::afio::usart2::Rx;
  use stm32f1xx_hal::gpio::{PA15, PB10, PB12, PB14};
  use stm32f1xx_hal::i2c::I2c;
  use stm32f1xx_hal::i2c::Instance;
  use stm32f1xx_hal::pac::usart2::RegisterBlock;
  use stm32f1xx_hal::prelude::_stm32f4xx_hal_timer_TimerExt;
  use stm32f1xx_hal::serial::SerialExt;
  use stm32f1xx_hal::serial::{Config, Serial};
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
    foc_l: FocSimple<LeftDriverImpl>,
    foc_r: FocSimple<RightDriverImpl>,
  }

  #[local]
  struct Local {
    led: PB12<Output<PushPull>>,
    sensor_l: SensorAngleLeft,
    sensor_r: SensorAngleRight,
    command_line: FocCommandLine<FocSerialImpl>,
    timer1: CounterHz<TIM1>,
    timer4: CounterHz<TIM4>,
    counter: usize,
    side: bool,
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

    // Instantiate TIM1 that will be used to create the 1 Khz  clock for the inner loop
    let mut timer1 = cx.device.TIM1.counter_hz(&mut rcc);
    timer1.start(1000.Hz()).unwrap();
    // Generate an interrupt when the timer expires
    timer1.listen(Event::Update);

    // Instantiate TIM4 that will be used to create the 5000 hrz clock to read out the analog sensors
    let mut timer4 = cx.device.TIM4.counter_hz(&mut rcc);
    timer4.start(500.Hz()).unwrap();
    // Generate an interrupt when the timer expires
    timer4.listen(Event::Update);


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



    // Set up the usart device 1 (with the bluetooth HC08 connected).
    let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pin_rx = gpioa.pa10;

    let serial = cx
      .device
      .USART1
      .serial((pin_tx, pin_rx), Config::default().baudrate(Bps(115_200)), &mut rcc);
    // Split the serial struct into a receiving and a transmitting part
    let (tx, rx) = serial.split();
    let parser = FocSerialImpl::new(tx, rx);

    // setup the message channels for left and right
    let (sender_l, receiver_l) = make_channel!(EFocCommand, COMMAND_QUEUE_TX);
    let (sender_r, receiver_r) = make_channel!(EFocCommand, COMMAND_QUEUE_TX);
    let (sender_retour, receiver_retour) = make_channel!(EFocCommand, COMMAND_QUEUE_RX);

    let senders = [sender_l, sender_r];
    let command_line = FocCommandLine::new(parser, senders, receiver_retour);

    //              motor 0       motor 1    motor 2
    // ---------------------------------------------
    // A            pB1 tim3      pa6 ?     pB9 tim4
    // B            pB0 tim3      pA3 tim2  pA1 tim2
    // C            pA7 tim3      pA2 tim2  pB8 tim4

    // only timer 2 and 3 are used for PWM. Left motor connected to M0 right motor to m1b, m1c and m2b

    // left wheel
    // m0a -> pb1  m0b-> pb0 m0c
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
    let pwm_l = FocPwm::new(pwm_driver_l, max, 1.0);

    // right wheel
    // a0 is not used for pwm, but needed to let the pwm2.split() succeed
    // a1 connected to m2B
    let a1 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let a2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let a3 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);
    let pins2 = (a1, a2, a3);
    let pwm2 = cx
      .device
      .TIM2
      .pwm_hz::<Tim2NoRemap, _, _>(pins2, &mut afio.mapr, 25.kHz(), &mut rcc);

    let channels2 = pwm2.split();

    let pwm_driver_r = RightDriverImpl::new(channels2.0, channels2.1, channels2.2);
    let pwm_r = FocPwm::new(pwm_driver_r, max, 1.0);

    // setup the foc_simple implementations
    let mut foc_l = FocSimple::new(receiver_l, sender_retour.clone(), pwm_l);
    foc_l.set_nr_poles(7);
    foc_l.set_inner_loop_hz(1000);

    let mut foc_r = FocSimple::new(receiver_r, sender_retour, pwm_r);
    foc_r.set_nr_poles(7);
    foc_r.set_inner_loop_hz(1000);



    // Schedule the blinking task
    command_task::spawn().ok();
    //blink_task::spawn().ok();
    hz100_task::spawn().ok();

    let counter = 0;
    (
      Shared { foc_l, foc_r },
      Local {
        led,
        sensor_l,
        sensor_r,
        command_line,
        timer1,
        timer4,
        counter,
        side: false,
      },
    )
  }
  /* 
  #[task(local = [led], priority = 2)]
  async fn blink_task(cx: blink_task::Context) {
    let mut instant = Mono::now();
    loop {
      cx.local.led.toggle();
      instant += 1.secs();
      Mono::delay_until(instant).await;

    }
  }
  */
  #[task(shared = [foc_l, foc_r], priority = 2)]
  async fn hz100_task(mut cx: hz100_task::Context) {
    let mut now = Mono::now();
    loop {
      cx.shared.foc_l.lock(|foc| {
        foc.update_100hz();
        foc.handle_command_queue();
      });      
      cx.shared.foc_r.lock(|foc| {
        foc.update_100hz();
        foc.handle_command_queue();
      });
      now += 10.millis();
      Mono::delay_until(now).await;
    }
  }

  #[task(local = [command_line], priority = 1)]
  async fn command_task(context: command_task::Context) {
    context.local.command_line.task().await;
  }

  // ticker task generating signals with 1000 hrz
  #[task(binds=TIM1_UP, local=[timer1 ], shared=[foc_l, foc_r], priority = 4)]
  fn tim1_task(mut cx: tim1_task::Context) {
    // high speed inner loop for left
    cx.shared.foc_l.lock(|foc| {
      match  foc.update_foc(){
        Ok(_) => (),
        Err(e) => rprintln!("Error:{:?}", e)
      };
    });
    // high speed inner loop for right
     
    cx.shared.foc_r.lock(|foc| {
      match  foc.update_foc(){
        Ok(_) => (),
        Err(e) => rprintln!("Error:{:?}", e)
      };
    });
    
    cx.local.timer1.clear_interrupt(Event::Update);
  }

  // ticker task generating signals with 200 hrz
  #[task(binds=TIM4, local=[timer4, sensor_l, sensor_r, led, counter, side], shared=[foc_l, foc_r],  priority = 3)]
  fn tim4_task(mut cx: tim4_task::Context) {
    let mut count= cx.local.counter;
    if *count > 499 {
      *count  = 0;
      cx.local.led.toggle();
    } else {
      *count +=1;
    }
    cx.local.side.toggle();
    if *cx.local.side {
      let angle_l = cx.local.sensor_l.get_angle();
        if let Ok(a) = angle_l {
          cx.shared.foc_l.lock(|foc| {
           foc.update_angle_from_sensor(a);
          });
      } else {
        rprintln!("l ang err")
      };      
    } else {
      let angle_r = cx.local.sensor_r.get_angle();
      if let Ok(a) = angle_r {
        cx.shared.foc_r.lock(|foc| {
          foc.update_angle_from_sensor(a);
        });
      } else {
        rprintln!("r ang err")
      };      
    }
    cx.local.timer4.clear_interrupt(Event::Update);
  }


}
