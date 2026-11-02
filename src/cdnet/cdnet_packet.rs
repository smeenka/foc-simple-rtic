use crate::{CD_MESSAGE_LEN, CD_PACKET_LEN, EFocEvent, EResultValue};
use crc16::{MODBUS, State};
use defmt::{info};

#[derive(Clone, Debug)]
pub struct CdnetPacket {    
    packet: [u8; CD_PACKET_LEN],
    sub_function: u8,
    reg_len: u8,      // nr of bytes to return
    register: u8,     // only the LSB of the u16 register is send to the motor
    queue_nr: u8,     // MSB of the register defines the queue to which the LSB register will be send
    idx : usize
}

impl CdnetPacket {
    pub fn new(buf: &[u8]) -> Self {
        let mut packet = [0; CD_PACKET_LEN];

        packet[0] = buf[1];  //slave id
        packet[1] = buf[0];  //host id
        packet[2] = buf[2]; // message length
        packet[3] = buf[4];  // function: info
        packet[4] = buf[3];  // host port

        let sub_function = buf[5];
        let reg_len = buf[8];

        let register = buf[6];      // LSB  of u16 register address
        let queue_nr = buf[7];      // MSB of register, defines the queue to which te register will be send
        let mut error = 0_u8;
        if packet[3] == 5 {
            match sub_function {
                0 | 1  => {  // read and read defailt
                    if buf[2] != 6 {
                        error = 1;
                        info!("Packet len not 6!");
                    }
                    packet[2] = reg_len + 5;
                }
                0x20_| 0xA0 => { // write with and without ack
                    let plen = buf[2]as usize;
                    for i in 5..plen {
                        if i < CD_MESSAGE_LEN {
                            packet[i] = buf[i]; 
                        }
                    } 
                    if sub_function == 0x20 { // for no response leave lenght to zero
                        packet[2] = 3;
                    }
                }
                _ => {info!("Unknown sub function");}
            }
        }
        CdnetPacket {
            packet,
            sub_function,
            reg_len,      // nr of bytes to return
            register,     // only the LSB of the u16 register is send to the motor
            queue_nr,     // MSB of the register defines the queue to which the LSB register will be send
            idx:6
        }
    }

    pub fn len(&self) -> u8 {
        self.packet[2]
    }
    pub fn function(&self) -> u8 {
        self.packet[3]
    }
    pub fn sub_function(&self) -> u8 {
        self.sub_function
    }
    pub fn is_read_reg(&self) -> bool{
        self.packet[3] == 5 && self.sub_function == 0 
    }
    pub fn is_read_default(&self) -> bool{
        self.packet[3] == 5 && self.sub_function == 1 
    }
    pub fn is_write_reg(&self) -> bool{
        (self.packet[3] == 5)  && match self.sub_function {
            0x20 | 0xA0 => true,
            _ => false
        }
    }
    pub fn reg_len(&self) -> u8{
        self.reg_len
    }
    pub fn queue_nr(&self) -> u8 {
        self.queue_nr
    }
    pub fn reset_idx(&mut self) {
        self.idx = 5
    }
    /// take packet from this object and invalidate (drop_) the object
    pub fn take_packet(self) -> EFocEvent {
        self.packet
    }
    pub fn add_data(&mut self, data:&[u8]) {
        for d in data {
            if self.idx   < CD_MESSAGE_LEN + 3 {
                self.packet[self.idx] = *d;
                self.idx += 1;    
            }
        }
        self.packet[2] = self.idx as u8  - 3;
    }
    pub fn add_i32(&mut self, data_i32:i32) {
        let data = data_i32.to_le_bytes(); 
        self.add_data(&data);
    }

    pub fn prepare_return_packet(packet: &mut [u8])  -> &[u8]  {
        let mut mlen = packet[2] as usize;
        if mlen > CD_MESSAGE_LEN {
            mlen = CD_MESSAGE_LEN;
            packet[2] = mlen as u8;
        }
        let plen = mlen + 3;
        let crc = State::<MODBUS>::calculate(&packet[0..plen]);
        packet[plen] = (crc & 0xFF) as u8;
        packet[plen+ 1 ] = ((crc >> 8) & 0xFF) as u8;
        &packet[0..(plen + 2)]
    }


    pub fn debug_packet(my_id: u8, message: &str) -> EFocEvent {
        let mut packet = [0; CD_PACKET_LEN];
        packet[0] = my_id ;    // function: info
        packet[3] = 0x40 ;   // host port 
        packet[4] = 0x9  ;   // function port
        CdnetPacket::add_message(&mut packet, message);
        packet
    }
    pub fn add_message(packet: &mut [u8], message: &str) {
        let mut idx = 5;
        for b in message.as_bytes() {
            if idx < (CD_MESSAGE_LEN -1) {
                packet[idx] = *b;
                idx += 1;
            }
        }
        packet[idx] = 10; // add newline
        idx += 1;
        packet[2] = idx as u8  - 3; // the lenght of the content only, without crc and the 3 start bytes 
    }  
    pub fn graph_packet(my_id: u8, channel: u8, time: u16, data_i32: i32) -> EFocEvent {
        let mut packet = [0; CD_PACKET_LEN];
        packet[0] = my_id ;    // function: info
        packet[3] = 0x40 | channel ;   // host port 
        packet[4] = 0xA  ;   // funcion
        packet[5] = time as u8 & 0xFF;
        packet[6] = (time >> 8)  as u8 & 0xFF;
        let data = data_i32.to_le_bytes(); 
        let mut idx = 7;
        for d in data {
            packet[idx] = d;
            idx += 1;    
        }
        packet[2] = idx as u8  - 3;
        packet
    }
}

