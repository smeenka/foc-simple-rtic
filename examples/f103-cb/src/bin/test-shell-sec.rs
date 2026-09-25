#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use core::fmt::Write;
use panic_rtt_target as _;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use rtt_target::{rprintln, rtt_init_default};
use stm32f1xx_hal::gpio::Pin;
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::pac::TIM2;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
  use rtic_f103_cb::foc_impl::FocSerialImpl;
  use rtic_monotonics::fugit::{Rate, RateExtU32};
  use stm32f1xx_hal::{
    afio::AfioExt,
    flash::FlashExt,
    gpio::{Edge, ExtiPin, GpioExt, Input, Output, PullUp, PushPull, PA0, PA10, PA2, PA4, PB2, PC13},
    pac::{Peripherals, USART2},
    rcc::{self, Clocks, RccExt},
    serial::{Config, Rx, Serial, Tx},
    time::{Bps, KiloHertz},
    timer::{Tim2NoRemap, Timer},
  };
  use rtic_sync::{channel::*, make_channel};
  use foc_simple_rtic::{FocCommandLine, EFocCommand, COMMAND_QUEUE_LEN};

  use super::*;

  /* bring dependencies into scope */

  #[shared]
  struct Shared {}

  #[local]
  struct Local {
    led: PB2<Output<PushPull>>,
    button: PC13<Input<PullUp>>,
    command_line: FocCommandLine<FocSerialImpl>,
    receiver0: Receiver<'static, EFocCommand, COMMAND_QUEUE_LEN>,
    receiver1: Receiver<'static, EFocCommand, COMMAND_QUEUE_LEN>
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
    rprintln!("Test Serial serial2 interface over usb");

    let mut gpioa = cx.device.GPIOA.split(&mut rcc);
    let mut gpiob = cx.device.GPIOB.split(&mut rcc);
    let mut gpioc = cx.device.GPIOC.split(&mut rcc);

    // Setup LED
    let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);

    // disable jtag
    let (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let mut button = gpioc.pc13.into_pull_up_input(&mut gpioc.crh);
    button.make_interrupt_source(&mut afio);
    button.enable_interrupt(&mut cx.device.EXTI);
    button.trigger_on_edge(&mut cx.device.EXTI, Edge::Falling);

    // USART2
    let pin_tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let pin_rx = gpioa.pa3;

    // Set up the usart device.
    let serial = Serial::new(
      cx.device.USART2,
      (pin_tx, pin_rx),
      Config::default().baudrate(Bps(115_200)),
      &mut rcc,
    );

    // Split the serial struct into a receiving and a transmitting part
    let (tx, rx) = serial.split();

    let parser = FocSerialImpl::new(tx, rx);
    let (sender0, receiver0) = make_channel!(EFocCommand, COMMAND_QUEUE_LEN);
    let (sender1, receiver1) = make_channel!(EFocCommand, COMMAND_QUEUE_LEN);
    let senders = [sender0,sender1];

    let command_line = FocCommandLine::new(parser, senders);

    // Schedule the blinking task
    blink_task::spawn().ok();
    command_line_task::spawn().ok();
    receiver0_task::spawn().ok();
    receiver1_task::spawn().ok();
    (Shared {}, Local { led, button, command_line, receiver0, receiver1 })
  }

  #[task(local = [led], priority = 2)]
  async fn blink_task(cx: blink_task::Context) {
    loop {
      cx.local.led.toggle();
      Mono::delay(1000.millis()).await;
    }
  }
  #[task(local = [receiver0], priority = 2)]
  async fn receiver0_task(cx: receiver0_task::Context) {
    rprintln!("Receiver0` task started");
    while let Ok(command) = cx.local.receiver0.recv().await {
        rprintln!("Receiver for motor0. got: {:?}",  command);
    }
    rprintln!("Receiver0 task finished");
  }
  #[task(local = [receiver1], priority = 2)]
  async fn receiver1_task(cx: receiver1_task::Context) {
    rprintln!("Receiver1 task started");
    while let Ok(command) = cx.local.receiver1.recv().await {
        rprintln!("Receiver motor 1.Got: {:?}", command);
    }
    rprintln!("Receiver1 task finished");
  }

  #[task(local = [command_line], priority = 1)]
  async fn command_line_task(context: command_line_task::Context) {
    rprintln!("Command line  task started");
    context.local.command_line.task().await;
    rprintln!("Command line  task finished");
  }

  #[task(binds=EXTI15_10,  local=[button] , priority = 2)]
  fn button_pressed(mut context: button_pressed::Context) {
    context.local.button.clear_interrupt_pending_bit();
    rprintln!("EXTI15_10 Button on C13  pressed");
  }

}

