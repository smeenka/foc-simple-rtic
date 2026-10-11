use crate::{EFocSimpleError, Result, vesc_slave::vesc_packets::VescValues};
use fixed::types::{I16F16, I6F26, I48F16};
use heapless::Vec;

mod vesc_packets;
mod vesc_slave;

// exports
pub use vesc_slave::VescSlave;

pub type SerialBufferType = Vec<u8, 128>;

#[derive(Debug,  Clone)]
#[repr(u8)]
pub enum EVescCommCommand {
    FwVersion = 0,
    JumpToBootloader,
    EraseNewApp,
    WriteNewAppData,
    GetValues,
    SetDuty,
    SetCurrent,
    SetCurrentBrake,
    SetRpm,
    SetPos,
    SetHandbrake = 10,
    SetDetect,
    SetServoPos,
    SetMcConf,
    GetMcConf,
    GetMcConfDefault,
    SetAppConf,
    GetAppConf,
    GetAppConfDefault,
    SamplePrint,
    TerminalCmd = 20,
    DetectMotorParam,
    DetectMotorRL,
    DetectMotorFluxLinkage,
    DetectEncoder,
    DetectHallFoc,
    Reboot,
    Alive,
    GetDecodedPpm,
    GetDecodedAdc,
    GetDecodedChuck = 30,
    ForwardCan,
    SetChuckData,
    CustomAppData,
    NrfStartPairing,
}

impl EVescCommCommand {
    fn from_u8(b:u8) -> Result<Self> {
        match b {
        0 => Ok(EVescCommCommand::FwVersion),
        1 => Ok(EVescCommCommand::JumpToBootloader),
        2 => Ok(EVescCommCommand::EraseNewApp),
        3 => Ok(EVescCommCommand::WriteNewAppData),
        4 => Ok(EVescCommCommand::GetValues),
        5 => Ok(EVescCommCommand::SetDuty),
        6 => Ok(EVescCommCommand::SetCurrent),
        7=> Ok(EVescCommCommand::SetCurrentBrake),
        8 => Ok(EVescCommCommand::SetRpm),
        9 => Ok(EVescCommCommand::SetPos),
        10 => Ok(EVescCommCommand::SetHandbrake),
        11 => Ok(EVescCommCommand::SetDetect),
        12 => Ok(EVescCommCommand::SetServoPos),
        13 => Ok(EVescCommCommand::SetMcConf),
        14 => Ok(EVescCommCommand::GetMcConf),
        15 => Ok(EVescCommCommand::GetMcConfDefault),
        16 => Ok(EVescCommCommand::SetAppConf),
        17 => Ok(EVescCommCommand::GetAppConf),
        18 => Ok(EVescCommCommand::GetAppConfDefault),
        19 => Ok(EVescCommCommand::SamplePrint),
        20 => Ok(EVescCommCommand::TerminalCmd),
        21 => Ok(EVescCommCommand::DetectMotorParam),
        22 => Ok(EVescCommCommand::DetectMotorRL),
        23 => Ok(EVescCommCommand::DetectMotorFluxLinkage),
        24 => Ok(EVescCommCommand::DetectEncoder),
        25 => Ok(EVescCommCommand::DetectHallFoc),
        26 => Ok(EVescCommCommand::Reboot),
        27 => Ok(EVescCommCommand::Alive),
        28 => Ok(EVescCommCommand::GetDecodedPpm),
        29 => Ok(EVescCommCommand::GetDecodedAdc),
        30  => Ok(EVescCommCommand::GetDecodedChuck),
        31 => Ok(EVescCommCommand::ForwardCan),
        32 => Ok(EVescCommCommand::SetChuckData),
        33 => Ok(EVescCommCommand::CustomAppData),
        34 => Ok(EVescCommCommand:: NrfStartPairing),
        _  => Err(EFocSimpleError::VescParseError)
        }
    }
    fn value(&self) -> u8 {
        self.clone() as u8
    }

}

/// Controller faults
#[derive(Debug,  Clone)]
#[repr(u8)]
pub enum EVescFault {
    /// No faults
    None,
    /// Input voltage too high
    OverVoltage,
    /// Input voltage too low
    UnderVoltage,
    /// DRV error
    Drv,
    /// Current too high
    AbsOverCurrent,
    /// FET temperature too high
    OverTempFet,
    /// Motor temperature too high
    OverTempMotor,
}
impl EVescFault {
    fn from_u8(b:u8) -> Result<Self> {
        match b {
        0 => Ok(EVescFault::None),
        1 => Ok(EVescFault::OverVoltage),
        2 => Ok(EVescFault::UnderVoltage),
        3 => Ok(EVescFault::Drv),
        4 => Ok(EVescFault::AbsOverCurrent),
        5 => Ok(EVescFault::OverTempFet),
        6 => Ok(EVescFault::OverTempMotor),
        _ => Err(EFocSimpleError::VescParseError)
        }
    }
    fn value(&self) -> u8 {
        self.clone() as u8
    }
}

pub enum EVescCommand {
    GetValues(VescValues),
    SetServoPosition(I6F26), // set the servo angle, in range 0..1
    SetPosition(I48F16), // set the rotor position in absolute value (rotations and angle )
}