#![deny(unsafe_code)]
#![allow(warnings)]
#![no_main]
#![no_std]

use core::fmt::Write;
use defmt::info;
use defmt_rtt as _;
use panic_halt as _;
use rtic::app;
use rtic_monotonics::systick::prelude::*;
use stm32f1xx_hal::gpio::Pin;
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::pac::TIM2;

systick_monotonic!(Mono, 1000);

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
  use super::*;
  use f103_cb::foc_impl::{FocSerialTxImpl, FocSerialRxImpl};
  use foc_simple_rtic::cdnet::{CdnetSlaveEvent, CdnetSlaveCommand, CdnetPacket};
  use foc_simple_rtic::{COMMAND_QUEUE_LEN, COMMAND_QUEUE_NR, EVENT_QUEUE_LEN, EFocEvent, EFocCommand, EResultValue};
  use rtic_monotonics::fugit::{Rate, RateExtU32};
  use rtic_sync::{channel::*, make_channel};
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

  /* bring dependencies into scope */

  #[shared]
  struct Shared {}

  #[local]
  struct Local {
    led: PC13<Output<PushPull>>,
    cdnet_event: CdnetSlaveEvent<FocSerialTxImpl>,
    cdnet_command: CdnetSlaveCommand<FocSerialRxImpl>,
    receiver0: Receiver<'static, EFocCommand, COMMAND_QUEUE_LEN>,
    receiver1: Receiver<'static, EFocCommand, COMMAND_QUEUE_LEN>,
    receiver2: Receiver<'static, EFocCommand, COMMAND_QUEUE_LEN>,
    event00: Sender<'static, EFocEvent, EVENT_QUEUE_LEN>,
    event01: Sender<'static, EFocEvent, EVENT_QUEUE_LEN>,
    event02: Sender<'static, EFocEvent, EVENT_QUEUE_LEN>,
    event1: Sender<'static, EFocEvent, EVENT_QUEUE_LEN>,
    event2: Sender<'static, EFocEvent, EVENT_QUEUE_LEN>,
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

    let mut gpioa = cx.device.GPIOA.split(&mut rcc);
    let mut gpiob = cx.device.GPIOB.split(&mut rcc);
    let mut gpioc = cx.device.GPIOC.split(&mut rcc);

    // Setup LED
    //let led = gpiob.pb2.into_push_pull_output(&mut gpiob.crl);

    // disable jtag
    // let (_pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

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

    let serial_tx = FocSerialTxImpl::new(tx);
    let serial_rx = FocSerialRxImpl::new(rx);

    let (sender0, receiver0) = make_channel!(EFocCommand, COMMAND_QUEUE_LEN);
    let (sender1, receiver1) = make_channel!(EFocCommand, COMMAND_QUEUE_LEN);
    let (sender2, receiver2) = make_channel!(EFocCommand, COMMAND_QUEUE_LEN);
    let (sender_ev, receiver_ev) = make_channel!(EFocEvent, EVENT_QUEUE_LEN);

    let command_queues = [sender0, sender1, sender2];
    let cdnet_command = CdnetSlaveCommand::new(serial_rx, command_queues);
    let cdnet_event  = CdnetSlaveEvent::new(serial_tx, 0x20, receiver_ev);


    // Schedule the blinking task
    blink_task::spawn().ok();
    cdnet_event_task::spawn().ok();
    cdnet_command_task::spawn().ok();
    debug_send_task::spawn().ok();
    receiver0_task::spawn().ok();
    receiver1_task::spawn().ok();
    receiver2_task::spawn().ok();
    graph_send_task::spawn().ok();
    (
      Shared {},
      Local {
        led,
        cdnet_event,
        cdnet_command,
        receiver0,
        receiver1,
        receiver2,
        event00: sender_ev.clone(),
        event01: sender_ev.clone(),
        event02: sender_ev.clone(),
        event1: sender_ev.clone(),
        event2: sender_ev
      },
    )
  }

  #[task(local = [led], priority = 2)]
  async fn blink_task(cx: blink_task::Context) {
    loop {
      cx.local.led.toggle();
      Mono::delay(1000.millis()).await;
    }
  }

  #[task(local = [cdnet_event], priority = 2)]
  async fn cdnet_event_task(context: cdnet_event_task::Context) {
    info!("Cdnet event task started");
    context.local.cdnet_event.task().await;
    info!("Cdnet slave event finished");
  }
  #[task(local = [cdnet_command], priority = 1)]
  async fn cdnet_command_task(context: cdnet_command_task::Context) {
    info!("Cdnet command task started");
    context.local.cdnet_command.task().await;
    info!("Cdnet slave event finished");
  }

  #[task(local = [event00], priority = 2)]
  async fn debug_send_task(cx: debug_send_task::Context) {
    loop {
      Mono::delay(10.secs()).await;
      cx.local.event00.send(CdnetPacket::debug_packet(0x20, "Message 1 send now ")).await;
      Mono::delay(10.millis()).await;
      cx.local.event00.send(CdnetPacket::debug_packet(0x20, "Message 2 send now ")).await;
      Mono::delay(10.millis()).await;
      cx.local.event00.send(CdnetPacket::debug_packet(0x20, "Message 3 send now ")).await;
      Mono::delay(10.millis()).await;
      cx.local.event00.send(CdnetPacket::debug_packet(0x20, "Message 4 send now ")).await;
    }
  }

  #[task(local = [event02], priority = 2)]
  async fn graph_send_task(cx: graph_send_task::Context) {
    let mut time = 0;
    let mut data = 90;
    let mut sign = 1;
    loop {
      let packet = CdnetPacket::graph_packet(0x20, 0, time, data);
      let packet1 = CdnetPacket::graph_packet(0x20, 1, time, data);
      if data > 500{
        sign = -1; ; 
      }
      if data < -500 {
        sign = 1;
      } 
      data -=sign;
      time+=1;
      cx.local.event02.send(packet).await;
      cx.local.event02.send(packet1).await;
      Mono::delay(100.millis()).await;
    }
  }



  #[task(local = [receiver0, event01], priority = 2)]
  async fn receiver0_task(cx: receiver0_task::Context) {
    info!("Receiver0` task started");
    let mut data0 = 1;
    loop {
      if let Ok(mut packet) = cx.local.receiver0.recv().await {
        info!("Receiver for motor0. got command packet");
        match packet.function() {
          1 => {
            info!("Return info packet to host ");
            let mut result = packet.take_packet();
            CdnetPacket::add_message(&mut result, "Cdnet Test Device");
            _ = cx.local.event01.try_send(result).ok()
          }
          5 => {
              info!("Receiver motor 1.Got: command:{:?}", packet.function());
              let mut nr_send = 0;
              while nr_send < packet.reg_len() {
                packet.add_i32(data0);
                data0 += 1;
                nr_send +=4;
              }
            _ = cx.local.event01.try_send(packet.take_packet()).ok()
          }
          _ => ()
        };
        
      }
    }
    info!("Receiver0 task finished");
  }
  #[task(local = [receiver1, event1], priority = 2)]
  async fn receiver1_task(cx: receiver1_task::Context) {
    info!("Receiver1 task started");
    let mut data1 = 0x101;
    loop {
      if let Ok(mut packet) = cx.local.receiver1.recv().await {
        info!("Receiver motor 1.Got: command:{:?}", packet.function());
        let mut nr_send = 0;
        while nr_send < packet.reg_len() {
          packet.add_i32(data1);
          data1 += 1;
          nr_send +=4;
        }
       _ = cx.local.event1.try_send(packet.take_packet()).ok()
      }
    }
    info!("Receiver1 task finished");
  }
  #[task(local = [receiver2, event2], priority = 2)]
  async fn receiver2_task(cx: receiver2_task::Context) {
    info!("Receiver2` task started");
    let mut data2 = 0x202;
    loop {
      if let Ok(mut packet) = cx.local.receiver2.recv().await {
        info!("Receiver motor 1.Got: command:{:?} reg_len:{}", packet.function(), packet.reg_len());
        let mut nr_send = 0;
        while nr_send < packet.reg_len() {
          packet.add_i32(data2);
          data2 += 1;
          nr_send +=4;
        }
       _ = cx.local.event2.try_send(packet.take_packet()).ok()
      }
    }
    info!("Receiver2 task finished");
  }

}


