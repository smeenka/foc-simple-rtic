use crate::vesc_slave::{EVescCommCommand, EVescFault, SerialBufferType};
use heapless::Vec;

pub struct SerialBuffer {
    buffer: SerialBufferType
}
impl SerialBuffer {
    fn new (id: EVescCommCommand) -> Self{
        let mut buffer = Vec::new();
        buffer.push(id.value()).unwrap();
        SerialBuffer {
            buffer
        }
    }
    fn append_f32(self, d:f32, divisor: u32) -> Self {
        let data = (d * divisor as f32) as u32;
        self.push_u32(data)
    }
    fn append_f16(self, d:f32, divisor: u32) -> Self {
        let data = (d * divisor as f32) as u16;
        self.push_u16(data)
    }
    fn append_u32(self, d:u32, divisor: u32) -> Self {
        let data = d * divisor;
        self.push_u32(data)
    }
    fn append_u16(self, d:u16, divisor: u16) -> Self {
        let data = (d * divisor) as u16;
        self.push_u16(data)
    }
    fn append_u8(mut self, data: u8) -> Self{
        if self.buffer.len() < 127 {
            self.buffer.push(data).unwrap();
        }
        self
    }
    fn append_array(mut self, data: &[u8]) -> Self{
        
        for i in data {
            if self.buffer.len() < 127 {
                self.buffer.push(*i).unwrap();
            }
        }
        self
    }

    fn push_u32(mut self, data: u32) -> Self{
        if self.buffer.len() < 124 {
            self.buffer.push(((data >>24) & 0xFF ) as u8).unwrap();
            self.buffer.push(((data >>16) & 0xFF ) as u8).unwrap();
            self.buffer.push(((data >>8) & 0xFF ) as u8).unwrap();
            self.buffer.push(((data >>0) & 0xFF ) as u8).unwrap();
        } 
        self
    }
    fn push_u16(mut self, data: u16) -> Self{
        if self.buffer.len() < 124 {
            self.buffer.push(((data >>8) & 0xFF ) as u8).unwrap();
            self.buffer.push(((data >>0) & 0xFF ) as u8).unwrap();
        }
        self
    }
    pub fn buffer(self) -> Vec<u8,128> {
        self.buffer 
    }
    
}



#[derive(Debug)]
pub struct FwVersion {
    /// Major version
    pub major: u8,
    /// Minor version
    pub minor: u8,
    /// Hardware version
    pub hw: [u8; 10],
    /// 96 bit ID of MCU
    pub uuid: [u8; 12]
}

impl FwVersion {
    pub fn default () -> Self {
        FwVersion {
            major: 0,
            minor:1,
            hw: [0x41; 10],
            uuid: [0x42; 12]
        }
    }
    pub fn serialize (&self) -> SerialBufferType{
        SerialBuffer::new(EVescCommCommand::FwVersion)
        .append_u8(self.major)
        .append_u8(self.minor)
        .append_array(&self.hw)
        .append_array(&self.uuid)
        .buffer
    }   
}

#[derive(Debug)]
pub struct VescValues {
    /// FET temperature in C
    pub temp_fet: f32,
    /// Motor temperature in C
    pub temp_motor: f32,
    /// Motor current in A
    pub motor_current: f32,
    /// Input current in A
    pub input_current: f32,
    /// 
    pub id: f32,
    /// 
    pub iq: f32,
    /// Motor duty cycle
    pub duty_cycle: f32,
    /// Motor RPM
    pub rpm: f32,
    /// Input voltage in V
    pub input_voltage: f32,
    /// Amp hours drawn in Ah
    pub amp_hours: f32,
    /// Amp hours charged in Ah
    pub amp_hours_charged: f32,
    /// Watt hours drawn in Wh
    pub watt_hours: f32,
    /// Watt hours charged in Wh
    pub watt_hours_charged: f32,
    /// Motor tachometer
    pub tachometer: u32,
    /// Absolute reading of motor tachometer
    pub tachometer_abs: u32,
    /// Fault state of controller
    pub fault: EVescFault,
    /// Motor position ?
    pub pid_pos: f32,
    /// ID of controller
    pub controller_id: u8,
}

impl VescValues {
    pub fn default() -> Self {
        VescValues {
            temp_fet: 123.0,
            temp_motor: 456.0,
            motor_current: 20.2_f32,
            input_current: 10.1,
            id: 12.2,
            iq: 50.5,
            duty_cycle: 0.9,
            rpm: 3.3,
            input_voltage: 55.5,
            amp_hours: 7007.0,
            amp_hours_charged: 600.6,
            watt_hours: 500.5,
            watt_hours_charged: 300.0,
            tachometer: 100,
            tachometer_abs: 100,
            fault: EVescFault::None,
            pid_pos: 2.22,
            controller_id: 0,
        }
    }

    pub fn serialize (&self) -> SerialBufferType {
        SerialBuffer::new(EVescCommCommand::GetValues)
        .append_f16(self.temp_fet, 10)
        .append_f16(self.temp_motor, 10)
        .append_f32(self.motor_current, 100)
        .append_f32(self.input_current, 100)
        .append_f32(self.id, 100)
        .append_f32(self.iq, 100)
        .append_f16(self.duty_cycle, 1_000)
        .append_f32(self.rpm, 1)
        .append_f32(self.input_voltage, 10)
        .append_f32(self.amp_hours, 10_000)
        .append_f32(self.amp_hours_charged, 10_000)
        .append_f32(self.watt_hours, 10_000)
        .append_f32(self.watt_hours_charged, 10_000)
        .append_u32(self.tachometer, 1)
        .append_u32(self.tachometer_abs, 1)
        .append_u8(self.fault.value())
        .append_f32(self.pid_pos, 1_000_000)
        .append_u8(self.controller_id)
        .buffer
    }
}
