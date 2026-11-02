use crate::{
  COMMAND_QUEUE_LEN, COMMAND_QUEUE_NR, CD_MESSAGE_LEN, CD_PACKET_LEN, EFocCommand, EFocEvent, EVENT_QUEUE_LEN, FocSerialRx, FocSerialTx, cdnet::CdnetPacket
};
use rtic_sync::channel::{Sender, Receiver};
use crc16::{MODBUS, State};
use defmt::{info, debug};

const DEBUG_PACKET_LEN_NOCRC:usize = CD_PACKET_LEN -2;  

pub struct CdnetSlaveCommand<C: FocSerialRx> {
  // user request
  serial: C,
  command_queues: [Sender<'static, EFocCommand, COMMAND_QUEUE_LEN>; COMMAND_QUEUE_NR]
}
enum EParseState {
    SenderId,
    RecieverId,
    Length,
    Payload,
    Checksum1,
    Checksum2
}

impl<C> CdnetSlaveCommand<C>
where
  C: FocSerialRx,
{
  pub fn new(serial: C, command_queues: [Sender<'static, EFocCommand, COMMAND_QUEUE_LEN>; COMMAND_QUEUE_NR]) -> Self {
    CdnetSlaveCommand {
      serial,
      command_queues,
    }
  }
  pub async fn task(&mut self) {
    let mut rx_buffer  = [0_u8;32];
    let mut parse_state = EParseState::SenderId;
    let mut packet_idx = 0_usize;
    let mut packet_crc = 0_u16;
    let mut packet = [0_u8;128];
    
    loop {
      if let Ok(n)= self.serial.receive(&mut rx_buffer).await {
        for idx in 0..n {
            let c = rx_buffer[idx];
            parse_state = match parse_state {
                EParseState::SenderId => {
                    packet[0] = c;
                    EParseState::RecieverId
                }
                EParseState::RecieverId => {
                    packet[1] =c;
                    EParseState::Length
                }
                EParseState::Length => {
                    packet[2] =  c;
                    packet_idx = 3;
                    EParseState::Payload
                }
                EParseState::Payload => {
                    packet[packet_idx] = c;
                    packet_idx += 1;
                    if packet_idx >= ((packet[2] + 3) as usize) {
                        EParseState::Checksum1
                    } else {
                        parse_state
                    }
                }
                EParseState::Checksum1 => {
                    packet_crc = c as u16; // little endian,  firstbyte is low value
                    EParseState::Checksum2
                }
                EParseState::Checksum2 => {
                    packet_crc += (c as u16) * 256; // little endian, second byte is high value
                    let buf = &packet[0..((packet[2] + 3) as usize)];
                    let calculated = State::<MODBUS>::calculate(buf);
                    //info!("CRC calculated:{} crc:{}", calculated, packet_crc);
                    if calculated == packet_crc {
                        self.parse_packet(&packet); // note the the full packet is referenced, not the buf
                    } else {    
                        info!("CRC error.");
                    }
                    EParseState::SenderId
                }

            }
          }
        }else {
          info!("Receive error");
        }
      }
    }

    fn parse_packet (&mut self, buf:&[u8]) {
      let packet = CdnetPacket::new(buf);  
      let function_nr = buf[4];
      match function_nr {  // function byte
        1 => {
          self.command_queues[0].try_send(packet).ok();
        }
        5 => {
          let queue_nr = packet.queue_nr() as usize;
          if queue_nr < self.command_queues.len() {
            self.command_queues[queue_nr].try_send(packet).ok();
          }
        }
        6 => {info!("Fast read/write, Not implemented yet");}
        _ => {info!("Unknown function: {}. Error!", function_nr);}
      }
    }
  }

pub struct CdnetSlaveEvent<C: FocSerialTx> {
  // user request
  serial: C,
  event_queue: Receiver<'static, EFocEvent, EVENT_QUEUE_LEN>,
  id: u8
}

impl<C> CdnetSlaveEvent<C>
where
  C: FocSerialTx,
{
  pub fn new(serial: C, id: u8, event_queue:Receiver<'static, EFocEvent, EVENT_QUEUE_LEN> ) -> Self {
    CdnetSlaveEvent {
      serial,
      event_queue,
      id
    }
  }
  pub async fn task(&mut self) {

    info!("Starting CdnetSlave Event task");
    loop {
      if let Ok(mut packet) = self.event_queue.recv().await {
        if packet[3] == 5 {
          info!("Received packet:{:?}", &packet);
        }
        _ = self.serial.send(CdnetPacket::prepare_return_packet(&mut packet));
      }
    }
  }
}
