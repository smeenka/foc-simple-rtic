use core::fmt::Write;
use fixed::types::{I16F16, I6F26};
use heapless::String;
use rtic_sync::channel::Sender;
use crate::COMMAND_QUEUE_LEN;

use crate::foc::MAX_MOTOR_NR;
use crate::{
  foc::{CalParams, EDir, EFocMode},
  EFocCommand, PidParam, FocSerial,ShaftPosition
};

#[derive(Clone, Debug, Copy, PartialEq)]
#[repr(usize)]
enum EFocModeLocal {
  Idle = 0,
  Calibration,
  Velocity,
  Angle,
  Torque,
}

pub struct FocCommandLine<C: FocSerial> {
  // user request
  serial: C,
  senders: [Sender<'static, EFocCommand, COMMAND_QUEUE_LEN>; MAX_MOTOR_NR],
  motor_nr: usize,
  foc_mode: EFocModeLocal,
  param_v: PidParam,
  param_a: PidParam,
  param_t: PidParam,
  param_c: Option<CalParams>,
}

impl<C> FocCommandLine<C>
where
  C: FocSerial,
{
  pub fn new(serial: C, senders: [Sender<'static, EFocCommand, COMMAND_QUEUE_LEN>; MAX_MOTOR_NR]) -> Self {
    let param_v = PidParam::new_fp(0.1, 0.0, 0.0);
    let param_a = PidParam::new_fp(0.4, 0.0, 0.0);
    let param_t = PidParam::new_fp(1.0, 0.0, 0.0);
    FocCommandLine {
      serial,
      senders,
      foc_mode: EFocModeLocal::Idle,
      param_a,
      param_t,
      param_v,
      param_c: None,
      motor_nr: 0,
    }
  }
  pub async fn task(&mut self) {
    self.send("\r\nStarting Foc Serial Parser task\r\n");
    self.help();
    let mut line = [0_u8; 128];
    let mut line_idx = 0;
    loop {
      let mut word = [0_u8; 32];
      if let Ok(len) = self.serial.receive(&mut word).await {
        for idx in 0..len {
          let a = word[idx];
          if a == 10 || a == 13 || a == 32 {
            self.handle_line(&line[0..line_idx]);
            line_idx = 0;
          } else {
            line[line_idx] = a;
            line_idx += 1;
          }
        }
      }
    }
  }

  fn handle_line(&mut self, line: &[u8]) {
    match core::str::from_utf8(line) {
      Ok(gotmsg) => {
        let words = gotmsg.split(" ");
        for word in words {
          self.handle_word(word);
        }
      }
      Err(_) => self.send("UTF8 error while parsing incoming line\r\n")
    }
  }
  fn handle_word(&mut self, m: &str) {
    if m.len() < 2 {
      self.send("Word must be at least 2 characters\r\n");
      return;
    }
    match &m[0..2] {
      "he" => self.help(),
      "ts" => self.set_speed(m, "Speed:"),
      "ta" => self.set_angle(m, "Angle:"),
      "tp" => self.set_position(m, "Position:"),
      "tt" => self.set_torque(m, "Torque:"),
      "tl" => self.set_torque_limit(m, "Torque Limit:"),
      "pa" => self.set_acceleration(m, "Speed Acceleration:"),
      "mi" => self.select_mode(EFocModeLocal::Idle),
      "mc" => self.select_mode(EFocModeLocal::Calibration),
      "mv" => self.select_mode(EFocModeLocal::Velocity),
      "ma" => self.select_mode(EFocModeLocal::Angle),
      "mt" => self.select_mode(EFocModeLocal::Torque),
      "m0" => self.select_motor(0),
      "m1" => self.select_motor(1),
      "np" => self.set_nr_poles(m, "Motor Poles count:"),
      "kp" => self.set_param(&m),
      "ki" => self.set_param(&m),
      "kd" => self.set_param(&m),
      "co" => self.set_calibration_offset(&m),
      "ec" => {
        _ = self.senders[self.motor_nr].try_send(EFocCommand::ErrorCount).ok();
      },
      _ => self.help(),
    }
  }

  fn send(&mut self, message: &str) {
    let bytes = message.as_bytes();
    _ = self.serial.send(bytes);
  }

  fn send_nl(&mut self) {
    let bytes = "\r\n".as_bytes();
    _ = self.serial.send(bytes);
  }

  pub fn send_usize(&mut self, data: usize) {
    let mut msg: String<10> = String::new();
    core::write!(&mut msg, "{}", data).unwrap();
    self.send(&msg);
  }

  pub fn send_i16f16(&mut self, data: I16F16) {
    let mut msg: String<20> = String::new();
    //let data: f32 = data.to_num();
    core::write!(&mut msg, "{}", data).unwrap();
    self.send(&msg);
  }

  fn select_mode(&mut self, mode: EFocModeLocal) {
    self.send("Select mode:");
    self.foc_mode = mode;
    match mode {
      EFocModeLocal::Angle => self.send("Angle\r\n"),
      EFocModeLocal::Velocity => self.send("Velocity\r\n"),
      EFocModeLocal::Torque => self.send("Torque\r\n"),
      EFocModeLocal::Calibration => self.send("Calibration\r\n"),
      EFocModeLocal::Idle => self.send("Idle\r\n"),
    };
    match mode {
      EFocModeLocal::Angle => {
        self.senders[self.motor_nr].try_send(EFocCommand::FocMode(EFocMode::Angle(self.param_a))).ok()
      }
      EFocModeLocal::Velocity => {
        self.senders[self.motor_nr].try_send(EFocCommand::FocMode(EFocMode::Velocity(self.param_v))).ok()
      }
      EFocModeLocal::Torque => {
        self.senders[self.motor_nr].try_send(EFocCommand::FocMode(EFocMode::Torque(self.param_t))).ok()
      }
      EFocModeLocal::Calibration => {
        self.senders[self.motor_nr].try_send(EFocCommand::FocMode(EFocMode::Calibration(None))).ok()
      }
      EFocModeLocal::Idle => self.senders[self.motor_nr].try_send(EFocCommand::FocMode(EFocMode::Idle)).ok(),
    };
  }
  fn select_motor(&mut self, m: usize) {
    self.send("Select motor:");
    self.send_usize(m);
    self.send_nl();
    self.motor_nr = m;
  }
  fn set_speed(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      self.senders[self.motor_nr].try_send(EFocCommand::Speed(f)).ok();
    }
  }
  fn set_angle(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      self.senders[self.motor_nr].try_send(EFocCommand::Angle(f)).ok();
    }
  }
  fn set_position(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      let r = f.checked_div(I16F16::TAU).unwrap();
      // round in the correct way, for positive rotations to -infinity, for negative towarde +infinity
      let rotations = if r >= 0 {r.floor()} else {r.ceil()};
      let angle = f.checked_rem(I16F16::TAU).unwrap();
      let mut shaft = ShaftPosition::new();
      shaft.set_shaft(rotations.to_num(), I6F26::from_num(angle));
      self.senders[self.motor_nr].try_send(EFocCommand::ShaftPosition(shaft)).ok();
    }
  }
  fn set_torque(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      self.senders[self.motor_nr].try_send(EFocCommand::Torque(f)).ok();
    }
  }
  fn set_acceleration(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      self.senders[self.motor_nr].try_send(EFocCommand::SpeedAcc(f)).ok();
    }
  }

  fn set_torque_limit(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      self.senders[self.motor_nr].try_send(EFocCommand::TorqueLimit(f)).ok();
    }
  }
  fn set_nr_poles(&mut self, word: &str, text: &str) {
    if let Some(f) = self.parse_float(word, text) {
      let poles = f.to_num();
      self.senders[self.motor_nr].try_send(EFocCommand::NrPoles(poles)).ok();
    }
  }

  fn parse_float(&mut self, word: &str, text: &str) -> Option<I16F16> {
    self.send(text);
    match word[2..].parse::<I16F16>() {
      Err(_) => {
        self.send("Unable to parse float after command\r\n");
        None
      }
      Ok(f) => {
        self.send_i16f16(f);
        self.send_nl();
        Some(f)
      }
    }
  }

  fn set_param(&mut self, word: &str) {
    match word[2..].parse::<I16F16>() {
      Err(_) => self.send("Unable to parse float after command\r\n"),
      Ok(f) => {
        let mode = match self.foc_mode {
          EFocModeLocal::Angle => "Angle",
          EFocModeLocal::Velocity => "Velocity",
          EFocModeLocal::Torque => "Torque",
          _ => "Invalid, choose mode first",
        };
        let p = match self.foc_mode {
          EFocModeLocal::Angle => &mut self.param_a,
          EFocModeLocal::Velocity => &mut self.param_v,
          EFocModeLocal::Torque => &mut self.param_t,
          _ => &mut PidParam::new(I16F16::ZERO, I16F16::ZERO, I16F16::ZERO),
        };
        match &word[0..2] {
          "kp" => p.set_p(f),
          "ki" => p.set_i(f),
          "kd" => p.set_d(f),
          _ => (),
        };
        self.send("Set pid parameters for mode:");
        self.send(mode);
        self.send(" ");
        self.send(&word[0..2]);
        self.send(" = ");
        self.send_i16f16(f);
        self.send_nl();
        self.select_mode(self.foc_mode);
      }
    }
  }

  fn set_calibration_offset(&mut self, word: &str) {
    match word[2..].parse::<I16F16>() {
      Err(_) => self.send("Unable to parse float after command\r\n"),
      Ok(f) => {
        self.send("Set electrical offset to:");
        self.send_i16f16(f);
        self.send_nl();
        let dir = if f >= 0 {EDir::Cw} else {EDir::Ccw};
        self.param_c = Some(CalParams::new(dir, I16F16::from_num(f.abs())));
        self.senders[self.motor_nr].try_send(
          EFocCommand::FocMode(
            EFocMode::Calibration(self.param_c.clone())
          )).ok();
      }
    }
  }

  pub fn help(&mut self) {
    self.send("\r\nHow to use  ... \r\n");
    self.send("  he        -- help this message\r\n");
    self.send("  ts<float> -- set target speed in turns/s\r\n");
    self.send("  ta<float> -- set target angle. Range 0 .. TAU\r\n");
    self.send("  tp<float> -- set target position Range -31K . 31K TAU \r\n");
    self.send("  tt<float> -- set target torque. Range 0..1\r\n");
    self.send("  tl<float> -- set max torque limit. Range 0..1\r\n");
    self.send("  pa<float> -- set speed acceleration in turns/sec2\r\n");
    self.send("  np<int>   -- set motor pole count\r\n");
    self.send("  mc        -- mode calibration. Start with this function!\r\n");
    self.send("  mi        -- mode idle\r\n");
    self.send("  mv        -- mode velocity\r\n");
    self.send("  mt        -- mode torque\r\n");
    self.send("  m0        -- select motor 0\r\n");
    self.send("  m1        -- select motor 1\r\n");
    self.send("  co        -- calibration offset. Only for test cases!\r\n");
    self.send("  kp<float> -- pid P\r\n");
    self.send("  ki<float> -- pid I\r\n");
    self.send("  kd<float> -- pid D\r\n");
    self.send("  ec        -- Show error count on rtt channel and reset\r\n");
  }
  }
