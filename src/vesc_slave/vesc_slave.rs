use crate::vesc_slave::vesc_packets::{FwVersion, VescValues};
use crate::{
  FocSerial, Result
};
use crate::vesc_slave::{EVescCommCommand, SerialBufferType};
use rtt_target::{rprintln};
use crc16::{State,XMODEM};
pub struct VescSlave<C: FocSerial> {
  // user request
  serial: C,
}
enum EParseState {
    StartOfPacket,
    Length1,
    Payload,
    Checksum1,
    Checksum2,
    EndOfPacket,
   
}

impl<C> VescSlave<C>
where
  C: FocSerial,
{
  pub fn new(serial: C) -> Self {
    VescSlave {
      serial,
    }
  }
  fn send(&mut self, message: &str) {
    let bytes = message.as_bytes();
    _ = self.serial.send(bytes);
  }  
  pub async fn task(&mut self) {
    rprintln!("\r\nStarting Foc VeEsc slave serial Parser task\r\n");
    let mut rx_buffer  = [0_u8;32];
    let mut parse_state = EParseState::StartOfPacket;
    let mut packet_len = 0_u8;
    let mut packet_idx = 0_usize;
    let mut packet_crc = 0_u16;
    let mut packet = [0_u8;128];
    
    loop {
      if let Ok(n)= self.serial.receive(&mut rx_buffer).await {
        for idx in 0..n {
            let c = rx_buffer[idx];
            parse_state = match parse_state {
                EParseState::StartOfPacket => 
                    match c {
                        2 => EParseState::Length1,
                        3 => {rprintln!("Packet len > 255 not yet supported. Skipping");
                            parse_state}
                        _ => {rprintln!("Searching start. Skipping:{:x}",c);
                            parse_state
                        }
                    }
                EParseState::Length1 => {
                    packet_len = c;
                    //rprintln!("len:{}", c);
                    packet_idx = 0;
                    EParseState::Payload
                }
                EParseState::Payload => {
                    packet[packet_idx] = c;
                    //rprintln!("p:{:x}", c);
                    packet_idx += 1;
                    if packet_idx >= (packet_len as usize) {
                        EParseState::Checksum1
                    } else {
                        parse_state
                    }
                }
                EParseState::Checksum1 => {
                    packet_crc = (c as u16) * 256; // big endian, first byte is high value
                    //rprintln!("CRC h:{:x}", c);
                    EParseState::Checksum2
                }
                EParseState::Checksum2 => {
                    //rprintln!("CRC l:{:x}", c);
                    packet_crc += c as u16; // big endian, second byte is low value
                    EParseState::EndOfPacket

                }
                EParseState::EndOfPacket => {
                    if c == 3 {
                        let buf = &packet[0..(packet_len as usize)];
                        let calculated = State::<XMODEM>::calculate(buf);
                        //rprintln!("CRC calculated:{} crc:{}", calculated, packet_crc);
                        if calculated == packet_crc {
                            self.handle_packet(&buf);
                        } else {    
                            rprintln!("CRC error. NOT Skipping frame");
                        }
                    } else {
                        rprintln!("Invalid packet end: {:x}. Skipping packet", c);
                    }
                    EParseState::StartOfPacket
                }

            }
        }
      }else {
        rprintln!("Receive error");
      }
    }
  }


  fn handle_packet(&mut self, packet: &[u8]) {
    match EVescCommCommand::from_u8(packet[0]) {
        Ok(command) => { 
            rprintln!("Command:{:?}, len:{}", command, packet.len());
            match command {
                EVescCommCommand::FwVersion => {
                    let mut packet = FwVersion::default();
                    packet.major = 7;
                    packet.minor = 1;
                    packet.hw = [b'R',b'u',b's',b't',b' ',b'V',b'E',b's',b'c',b'!'];
                    self.send_packet(packet.serialize()).unwrap();
                }
                EVescCommCommand::GetAppConf => (),
                EVescCommCommand::GetMcConf => (),
                EVescCommCommand::Alive => (),
                EVescCommCommand::GetValues => {
                    let packet = VescValues::default();
                    self.send_packet(packet.serialize()).unwrap();
                }
            _ => {rprintln!("Not supported yet!:{:?}", command);}
            }
        }
        Err(_) => {rprintln!("Vesc parse error");}
    }
  }

  fn send_packet(&mut self, packet:SerialBufferType) -> Result<()> {
    let buf2 = [0x2,  packet.len() as u8 ] ;
    self.serial.send(&buf2).unwrap();
    self.serial.send(&packet).unwrap();
    let calculated = State::<XMODEM>::calculate(&packet);
    let buf3 = [(calculated >> 8) as u8,  (calculated & 0xFF) as u8, 0x3 ];
    self.serial.send(&buf3)
  }
}
