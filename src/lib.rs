#![no_std]
#![allow(dead_code)]
use core::future::Future;
use fixed::types::I16F16;
use heapless::Vec;
//use crate::{ tools::shaft_position::ShaftPosition};
use crate::cdnet::CdnetPacket;

//pub use crate::tools::foc_command_line::FocCommandLine;
//pub mod foc;
pub mod tools;
pub mod cdnet;

pub const CD_MESSAGE_LEN:usize = 30;
pub const CD_PACKET_LEN:usize = 35;
pub const EVENT_QUEUE_LEN:usize = 10; // events to the user interface
pub const COMMAND_QUEUE_LEN:usize = 4; // commands from the user interface
pub const COMMAND_QUEUE_NR: usize = 3;  // channel 0 for application, 1 for motor 1 2 for motor 2 etc

#[derive(Clone, Debug)]
pub enum EPidParameter {
  P(I16F16), // proportional gain
  I(I16F16), // integration gain
  D(I16F16), // derivative gain
  Limit(I16F16), // saturation limit
  Ramp(I16F16), // ramp parameter
  Filter(I16F16), // low pass filtering time constant
}

#[derive(Clone, Debug)]
pub enum EPid {
  CurrentQ(EPidParameter),
  CurrentD(EPidParameter),
  Velocity(EPidParameter),
  Angle(EPidParameter),
}

#[derive(Clone, Debug)]
pub enum EFocLimit {
  Current(I16F16), // abs current limit in ampere
  Voltage(I16F16), // abs voltage limit as output by the motion controller 0..1
  Velocity(I16F16), // abs velocity limit in radians/sec
  Acceleration(I16F16)// acceleration limit in velocity mode in radians/sec2
}

#[derive(Clone, Debug)]
pub enum EFocMonitor {
  DownSample(u32),   // down sample. 0 is off
  Set([bool;7]),     // enable /disable the 7 7 monitored variablen 
  Get(I16F16),          // request to return a monitor value
  Sample(Vec<I16F16, 7>)
}

pub type EFocCommand = CdnetPacket;

#[derive(Clone, Debug)]
pub enum EResultValue {
  I16F16(I16F16),
  Integer(i32),
  None
}
pub type EFocEvent = [u8; CD_PACKET_LEN];

#[derive(Clone, Debug, PartialEq)]
pub enum EFocSimpleError {
  Uninitialized,
  NoMagWarning,
  DataError,
  SerialError,
  AngleSensorError,
  MotorNrError,
  NoMotorMovement,
  NoAngleSensor,
  NoCurrentSensor,
  NotImplementedYet,
  NoData,
  SerialParseError,
  VescParseError,
  VescCrcError,
  Error,
}
pub type Result<T> = core::result::Result<T, EFocSimpleError>;

/// The trait for getting the shaft angle of the motor from some sensor
pub trait SensorAngle {
  /// Retrieve the angle from the sensor, in a synchrounous way
  /// Return on success the raw angle from the sensor, in radians from 0 .. 2 pi
  fn get_angle(&mut self) -> Result<I16F16>;
}


pub type MessageRx = [u8; 64];
pub struct MessageTx {
  pub data: [u8; 64],
  pub len: usize,
}

pub trait FocSerial {
  /// return the function which can read a chunk of bytes from the stream
  /// If Ok the ok contains the actual number of bytes read
  fn receive(&mut self, buffer: &mut [u8]) -> impl Future<Output = Result<usize>>;
  fn send(&mut self, buffer: &[u8]) -> Result<()>;
}

pub trait FocSerialRx {
  /// return the function which can read a chunk of bytes from the stream
  /// If Ok the ok contains the actual number of bytes read
  fn receive(&mut self, buffer: &mut [u8]) -> impl Future<Output = Result<usize>>;
}

pub trait FocSerialTx {
  fn send(&mut self, buffer: &[u8]) -> Result<()>;
}

