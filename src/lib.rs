#![no_std]
#![allow(dead_code)]
use core::future::Future;
use fixed::types::I16F16;
use heapless::Vec;
use crate::{foc::{EFocCurrentMode, EFocMode, EModulation}, tools::shaft_position::ShaftPosition};

pub use crate::tools::foc_command_line::FocCommandLine;
pub mod foc;
pub mod tools;

pub const COMMAND_QUEUE_RX:usize = 50; // foc => command_line
pub const COMMAND_QUEUE_TX:usize = 10; // command_line => foc

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

#[derive(Clone, Debug)]
pub enum EFocCommand {
  Pid(EPid, bool),               // Get or Set a pid paramter
  Limit(EFocLimit, bool),       // Get or Set a foc limit   
  MotionControl(EFocMode, bool),// Get or set the mode for the motion controller
  CurrentControl(EFocCurrentMode, bool),// Get or set the mode for the current controller
  Monitor(EFocMonitor, bool),         // Set the monitor status
  EnableStatus(bool, bool),     // Get or set the enable status, first bool is enable status, second bool the setter
  SensorOffset(I16F16, bool),  // get or set the sensor offset, in range -TAU ..TAU
  Target(I16F16, Option<I16F16>, Option<I16F16>),  // target speed, angle or torque with optional torque limit and optional velocity limit
  NrPoles(u8),                  // idle mode: set the nr of poles. Note: a calibration is needed after changing the number of poles
  Modulation(EModulation),
  ErrorCount,                   // show the error count on the rtt channel, and reset to zero
}

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

