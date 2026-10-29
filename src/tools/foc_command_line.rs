use core::fmt::Write;
use fixed::types::{I16F16};
use heapless::String;
use rtic_sync::channel::{Sender, Receiver};
use crate::{COMMAND_QUEUE_RX, COMMAND_QUEUE_TX, EFocLimit, EFocMonitor, EFocSimpleError, EPid, EPidParameter, Result};

use crate::foc::MAX_MOTOR_NR;
use crate::{
  foc::{CalParams, EFocMode, EModulation, EFocCurrentMode},
  EFocCommand, FocSerial
};
use rtt_target::rprintln;

#[derive(Clone, Debug, Copy, PartialEq)]
#[repr(usize)]
enum EFocModeLocal {
  Idle = 0,
  Calibration,
  Velocity,
  OpenLoopVelocity,
  Angle,
  Torque,
}

// for spec  see https://docs.simplefoc.com/commander_motor
pub struct FocCommandLine<C: FocSerial> {
  // user request
  serial: C,
  senders: [Sender<'static, EFocCommand, COMMAND_QUEUE_TX>; MAX_MOTOR_NR],
  receiver: Receiver<'static, EFocCommand, COMMAND_QUEUE_RX>,
  motor_nr: usize,
  foc_mode: EFocModeLocal,
  param_c: Option<CalParams>,
  verbose: u32,
}

impl<C> FocCommandLine<C>
where
  C: FocSerial,
{
  pub fn new(serial: C, senders: [Sender<'static, EFocCommand, COMMAND_QUEUE_TX>; MAX_MOTOR_NR], receiver:Receiver<'static, EFocCommand, COMMAND_QUEUE_RX> ) -> Self {
    FocCommandLine {
      serial,
      senders,
      receiver,
      foc_mode: EFocModeLocal::Idle,
      param_c: None,
      motor_nr: 0,
      verbose: 0
    }
  }
  pub async fn task(&mut self) {
    rprintln!("Command Line task\r\n");
    let mut line = [0_u8; 64];
    let mut line_idx = 0;
    loop {
      let mut word = [0_u8; 32];
      match self.receiver.try_recv(){
        Ok(m) => self.handle_from_foc(m),
        Err(_) => ()
      }
      if let Ok(len) = self.serial.receive(&mut word).await {
        for idx in 0..len {
          let a = word[idx];
          match a {
              10 | 13 => {
                // skip CR or LF if on position 0. Thats the way to handle CRLF or LFCR
                if line_idx > 0 {
                  if self.verbose & 2 == 2 {
                    rprintln!(" Line:{:?}",&line[0..line_idx]);
                  }
                  if let Err(e) = self.handle_line(&line[0..line_idx]){
                    if self.verbose & 1 == 1 {
                      rprintln!("Error:{:?} line:{:?}", e, &line[0..line_idx]);
                    }
                  };
                  line_idx = 0
                }
              }
            _ => {
              line[line_idx] = a;
              if line_idx < line.len() -2 {
                line_idx += 1;
              }
            }
          }
        }
      }
    }
  }

  fn handle_line(&mut self, line: &[u8]) -> Result<()>{
    let mut linestr:String<128> = String::new();
    for b in line {
      if let Some(c) = char::from_u32(*b as u32) {
        _ = linestr.push(c)
      }
    }
    match &linestr[0..1]{ 
      "?" => { 
        self.send("?0: motor 0\r\n?1: motor 1\r\n");
        return Ok(())
      }
      _ => ()
    }
    if linestr.len() > 1 {
      self.parse_line(linestr.as_str()) 
    } else  {
      Err(EFocSimpleError::SerialError)
    }  
  }
  
  fn parse_line(&mut self, m: &str) -> Result<()>{
    match &m[0..1] {
      "0" => self.motor_nr = 0,
      "1" => self.motor_nr = 1,
      _  => return Err(EFocSimpleError::MotorNrError)
    }
    let command = match &m[1..2] {
      "Q" | "D" | "V" | "A"=> self.handle_pid_param(m),
      "R" => Err(EFocSimpleError::NotImplementedYet),
      "I" => Err(EFocSimpleError::NotImplementedYet),
      "K" => Err(EFocSimpleError::NotImplementedYet),
      "S" => self.handle_sensor_offsets(m),
      "L" => self.handle_limits(m),
      "M" => self.handle_monitor(m),
      "C" => self.handle_motion_mode(m),
      "T" => self.handle_current_mode(m),
      "W" => self.handle_modulation(m),
      "F" => self.handle_init_foc(m),
      "E" => self.handle_motor_status(m),
      "p" => self.set_nr_poles(m, "Motor Poles count:"),
      _ => self.handle_target(m)
    };
    match command {
      Ok(c) => {
        match c {
          EFocCommand::EnableStatus(_, false) => (),
          _ => {
            if self.verbose & 4 == 4 {
              rprintln!("command:");
            }
          }
        }
        self.senders[self.motor_nr].try_send(c).ok();
        Ok(())
      }
      Err(e) => Err(e)
    }
  }

  fn send(&mut self, message: &str) {
    let bytes = message.as_bytes();
    _ = self.serial.send(bytes);
  }

  fn handle_pid_param(&mut self, m: &str) -> Result<EFocCommand>{
    let value = match m.len() {
      1 | 2 => return Err(EFocSimpleError::SerialParseError),
      3 => I16F16::ZERO,
      _ => self.parse_float(&m[3..], "Pid Parameter")?
    };
    let pid_param = match &m[2..3] {
      "P" => EPidParameter::P(value),
      "I" => EPidParameter::I(value),
      "D" => EPidParameter::D(value),
      "L" => EPidParameter::Limit(value),
      "R" => EPidParameter::Ramp(value),
      "F" => EPidParameter::Filter(value),
      _ => return Err(EFocSimpleError::NotImplementedYet)
    };
    let pid = match &m[1..2] {
      "Q" => EPid::CurrentQ(pid_param),
      "D" => EPid::CurrentD(pid_param),
      "V" => EPid::Velocity(pid_param),
      "A" => EPid::Angle(pid_param),
      _ => return Err(EFocSimpleError::NotImplementedYet)
    };
    Ok(EFocCommand::Pid(pid, m.len() > 3))
  } 

  fn handle_limits(&mut self, m: &str) -> Result<EFocCommand>{
    let value = match m.len() {
      1 | 2 => return Err(EFocSimpleError::SerialParseError),
      3 => I16F16::ZERO,
      _ => self.parse_float(&m[3..], "Limit")?
    };

    let limit = match &m[2..3] {
      "C" => EFocLimit::Current(value),
      "U" => EFocLimit::Voltage(value),
      "V" => EFocLimit::Velocity(value),
      "A" => EFocLimit::Acceleration(value),
      _ => return Err(EFocSimpleError::NotImplementedYet)
    };
    Ok(EFocCommand::Limit(limit, m.len() > 3))
  } 
  fn handle_modulation(&mut self, m: &str) -> Result<EFocCommand>{
    if m.len() < 4 {
      return Err(EFocSimpleError::SerialParseError)
    }
    let modul = match &m[2..4] {
        "T0" => EModulation::Sinusoidal,
        "T1" => EModulation::Trapezoidal,
        "T2" => EModulation::SpaceVector,
        // "T3" => EModulation::Square, causes the motor to behave bad
        _ => return Err(EFocSimpleError::NotImplementedYet)
    };
    Ok(EFocCommand::Modulation(modul))
  } 


  fn handle_sensor_offsets(&mut self, m: &str) -> Result<EFocCommand>{
    let value = match m.len() {
      1 | 2 => return Err(EFocSimpleError::SerialParseError),
      3 => I16F16::ZERO,
      _ => self.parse_float(&m[3..], "Sensor Offset")?
    };
    match &m[2..3] {
      "M" => Ok(EFocCommand::SensorOffset(value, m.len() > 3)),
      _ => Err(EFocSimpleError::NotImplementedYet)
    }
  } 
  fn handle_monitor(&mut self, m: &str) -> Result<EFocCommand>{
    let value = match m.len() {
      1 | 2 => return Err(EFocSimpleError::SerialParseError),
      3 => 0,
      _ => self.parse_integer(&m[3..], "Monitor")?
    };
    let command = match &m[2..3] {
      "D" => EFocMonitor::DownSample(value as u32),
      "C" => EFocMonitor::Set([false;7]),
      "S" => {
        if m.len() < 10 {
          return Err(EFocSimpleError::SerialParseError)
        }
        let mut var_set = [false;7];
        for i in 0..7 {
          if &m[i+3..i+4] == "1" {
            var_set[i] = true;
          }
        }
        EFocMonitor::Set(var_set)
      }
      "G" => EFocMonitor::Get(I16F16::from_num(value)),
      "V" => {self.verbose = value as u32;
        rprintln!("Set Verbose to:{}", value);
        return Err(EFocSimpleError::NotImplementedYet)
      }
      _ => return Err(EFocSimpleError::NotImplementedYet)    
    };
    Ok(EFocCommand::Monitor(command, m.len() > 3))
  } 

  fn handle_motor_status(&mut self, m: &str) -> Result<EFocCommand>{
    let enabled = match m.len() {
      2 => false,   // getter case
      3 => match &m[2..3] { // setter case
        "0" => false,
        "1" => true,
        _ => return Err(EFocSimpleError::SerialParseError),
      }
      _  => return Err(EFocSimpleError::SerialParseError),
    };
    Ok(EFocCommand::EnableStatus(enabled, m.len() ==3 ))
  } 

  fn handle_motion_mode(&mut self, m: &str) -> Result<EFocCommand>{
    let mode = match m.len() {
      3 => {               // setter case
        match &m[2..3] {
          "D" | "T" | "5" | "6" => return Err(EFocSimpleError::NotImplementedYet),
          "0" => EFocMode::Torque,
          "1" => EFocMode::Velocity(false),
          "2" => EFocMode::Angle(false),
          "3" => EFocMode::Velocity(true),
          "4" => EFocMode::Angle(true),
          _ => return Err(EFocSimpleError::NotImplementedYet)
        }
      }
      _ => return Err(EFocSimpleError::SerialParseError)
    };
    Ok(EFocCommand::MotionControl(mode, true))
  } 

  fn handle_current_mode(&mut self, m: &str) -> Result<EFocCommand>{
    let mode = match m.len() {
      3 => {               // setter case
        match &m[2..3] {
          "0" => EFocCurrentMode::Voltage,
          "2" => EFocCurrentMode::CurrentFeedback,
          _ => return Err(EFocSimpleError::NotImplementedYet)
        }
      }
      _ => return Err(EFocSimpleError::SerialParseError),
    };
    Ok(EFocCommand::CurrentControl(mode, true))
  } 


  fn handle_init_foc(&mut self, m: &str) -> Result<EFocCommand>{
    match m.len() {
      3 => {               
        match &m[2..3] {
          "R" => Ok(EFocCommand::MotionControl(EFocMode::Calibration(None), true)),
          _ => Err(EFocSimpleError::NotImplementedYet)
        }
      }
     _ => return Err(EFocSimpleError::SerialParseError)
    }
  } 

  fn handle_target(&mut self, m: &str) -> Result<EFocCommand> {  
    let value = self.parse_float(&m[1..], "Target")?;
    let mut velocity= None;
    let mut torque = None;

    let words = m.split(" ");
    let mut idx = 0;
    for word in words {
      match idx {
        0 => (),
        1 => torque = Some(self.parse_float(word, "Torque limit")?),
        2 => velocity = Some(self.parse_float(word, "Velocity limit")?),
        _ => () // ignore the rest
      }
      idx +=1;
    }
    Ok(EFocCommand::Target(value, torque, velocity))
  }

  fn set_nr_poles(&mut self, word: &str, text: &str) -> Result<EFocCommand>{
    let f = self.parse_float(word, text)?; 
    let poles = f.to_num();
    Ok(EFocCommand::NrPoles(poles))
  }

  fn parse_float(&mut self, word: &str, text: &str) -> Result<I16F16> {
    match word.parse::<I16F16>() {
      Err(_) => {
        rprintln!("Error: For:{} unable to parse float:{}", text, word);
        Err(EFocSimpleError::SerialParseError)
      }
      Ok(f) => {
        Ok(f)
      }
    }
  }
  fn parse_integer(&mut self, word: &str, text: &str) -> Result<i32> {
    let f = self.parse_float(word, text)?;
    let result = f.int().to_num();
    Ok(result)
  }


  fn handle_from_foc(&mut self, f:EFocCommand) {
    let mut msg: String<100> = String::new();
    match f {
      EFocCommand::Pid(p, _) => {
        let (param, pid_id) = match p {
            EPid::CurrentQ(pid_param) => (pid_param, "Q"),
            EPid::CurrentD(pid_param)=> (pid_param, "D"),
            EPid::Velocity(pid_param)=> (pid_param, "V"),
            EPid::Angle(pid_param)=> (pid_param, "A"),
        };
        let (pp, parm_id) = match param  {
          EPidParameter::P(v)=> (v, "P"),
          EPidParameter::I(v)=> (v, "I"),
          EPidParameter::D(v)=> (v, "D"),
          EPidParameter::Limit(v)=> (v, "L"),
          EPidParameter::Ramp(v)=> (v, "R"),
          EPidParameter::Filter(v)=> (v, "F"),
        };
        let result:f32 = pp.to_num();
        _ = core::write!(&mut msg, "{}{}{}{:.4}\r\n",self.motor_nr,pid_id,parm_id,result);
      }
      EFocCommand::Limit(l,_) => {
        let (pp, limit_id) = match l {
          EFocLimit::Current(v)=> (v, "C"),
          EFocLimit::Voltage(v)=> (v, "U"),
          EFocLimit::Velocity(v)=> (v, "V"),
          EFocLimit::Acceleration(v)=> (v, "A"),
        };
        let result:f32 = pp.to_num();
        _ = core::write!(&mut msg, "{}L{}{:.4}\r\n",self.motor_nr, limit_id,result);
      }
      EFocCommand::SensorOffset(s,_) => {
        let result:f32 = s.to_num();
        _ = core::write!(&mut msg, "{}SM{:.4}\r\n",self.motor_nr, result);
      }
      EFocCommand::MotionControl(m,_) => {
        let mode = match m {
          EFocMode::Torque => "0",
          EFocMode::Velocity(false) => "1",
          EFocMode::Angle(false)=> "2",
          EFocMode::Velocity(true)=> "3",
          EFocMode::Angle(true)=> "4",
          _ => "6"
        };
        _ =core::write!(&mut msg, "{}C{}\r\n",self.motor_nr,mode); 
      }
      EFocCommand::EnableStatus(s,_) => {
        _ = core::write!(&mut msg, "{}E{}\r\n",self.motor_nr, if s {1} else {0});
      }
      EFocCommand::Monitor(EFocMonitor::Sample(samples), _) => {
        let len = samples.len() as isize - 1;
        _ = core::write!(&mut msg, "{:?}",self.motor_nr);
        let mut i = 0;
        for m in samples {
            if i < len {
              _ =core::write!(&mut msg, "{:.4}\t",m);
            } else {
              _ = core::write!(&mut msg, "{:.4}",m);
            } 
            i += 1;
        }
        _ = core::write!(&mut msg, "{:?}\r\n",self.motor_nr);
      } 
      _ => ()
    }
    self.send(&msg);
  }
}
