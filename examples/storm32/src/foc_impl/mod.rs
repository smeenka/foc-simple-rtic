pub mod pwm_driver;

use as5600::As5600;
use core::future::Future;
use fixed::types::I16F16;
use foc_simple_rtic::SensorAngle;
use foc_simple_rtic::{EFocSimpleError, FocSerial, Result, SensorCurrent};
use stm32f1::Periph;
use stm32f1xx_hal::i2c::I2c;
use stm32f1xx_hal::pac::i2c1::RegisterBlock;
use stm32f1xx_hal::{pac::USART1, serial::Tx};
use stm32f1xx_hal::serial::Rx;


pub struct FocSerialImpl {
    tx: Tx<USART1>,
    rx: Rx<USART1>,
}

impl FocSerialImpl {
    pub fn new(tx: Tx<USART1>, rx: Rx<USART1>) -> Self {
        FocSerialImpl { tx, rx }
    }
    async fn wait_receive(&mut self, buffer: &mut [u8]) -> Result<usize> {
        let mut idx = 0;
        while self.rx.is_rx_not_empty() {
            match self.rx.read() {
                Ok(c) => {
                    buffer[idx] = c;
                    idx += 1;
                }
                Err(_err) => (),
            }
        }
        Ok(idx)
    }
}

impl FocSerial for FocSerialImpl {
    fn receive(&mut self, buffer: &mut [u8]) -> impl Future<Output = Result<usize>> {
        // return the function, do not await here
        self.wait_receive(buffer)
    }

    fn send(&mut self, buffer: &[u8]) -> Result<()> {
        match self.tx.bwrite_all_u8(buffer) {
            Ok(()) => Ok(()),
            Err(_) => Err(EFocSimpleError::SerialError),
        }
    }
}



// dummy implementation
pub struct SensorCurrentImpl {}

impl SensorCurrentImpl {
  pub fn new() -> Self {
    SensorCurrentImpl {}
  }
}
impl SensorCurrent for SensorCurrentImpl {
  fn get_current(&mut self) -> Result<[I16F16; 3]> {
    Ok([I16F16::ZERO; 3])
  }
}
pub struct SensorAngleLeft {
  sensor: As5600<I2c<Periph<RegisterBlock, 1073764352>>>,
}

impl SensorAngleLeft {
  pub fn new(i2c: I2c<Periph<RegisterBlock, 1073764352>>) -> Self {
    let sensor = As5600::new(i2c);
    SensorAngleLeft { sensor }
  }
}

impl SensorAngle for SensorAngleLeft {
  fn get_angle(&mut self) -> Result<I16F16> {
    match self.sensor.angle() {
      Err(_) => Err(EFocSimpleError::AngleSensorError),
      Ok(a) => {
        let angle = ((a as i32) * I16F16::TAU) / 4096;
        Ok(angle)
      }
    }
  }
}

pub struct SensorAngleRight {
  sensor: As5600<I2c<Periph<RegisterBlock, 1073763328>>>,
}

impl SensorAngleRight {
  pub fn new(i2c: I2c<Periph<RegisterBlock, 1073763328>>) -> Self {
    let sensor = As5600::new(i2c);
    SensorAngleRight { sensor }
  }
}

impl SensorAngle for SensorAngleRight {
  fn get_angle(&mut self) -> Result<I16F16> {
    match self.sensor.angle() {
      Err(_) => Err(EFocSimpleError::AngleSensorError),
      Ok(a) => {
        let angle = ((a as i32) * I16F16::TAU) / 4096;
        Ok(angle)
      }
    }
  }
}
