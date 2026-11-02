use heapless::Vec;

mod cdnet_slave;
mod cdnet_packet;

// exports
pub use cdnet_slave::{CdnetSlaveEvent, CdnetSlaveCommand};
pub use cdnet_packet::{CdnetPacket};

pub type SerialBufferType = Vec<u8, 128>;

#[derive(Debug,  Clone)]
#[repr(u8)]
pub enum ECdnetPort {
    InfoQuery = 0,
    RegReadWrite = 5,
    FastReadWrite = 6,
    FirmwareUpgrade = 8,
    DebugPrint = 9,
    WaveForm = 0x0A,
    ImageReport = 0x10
}
pub enum ECdNetReadWrite {
    Read =  0,
    ReadDefault = 1,
    Write = 0x20, 
    WriteNoRepsonse= 0xA0

}

