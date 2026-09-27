use fixed::types::{I6F26, I48F16, I16F16};

#[derive(Clone, Debug, Copy, PartialEq)]
/// Shaft postions consists of a rotations and angle. The angle can count for -31 to 31 (6 bits), with a precision of 26 float bits
/// This precision is needed, because we work with very small increments in this libray. 
/// For example 60 RPM is TAU/s. With an inner loop of 25Khz the increment per tick will become 0.00025. So we need more than 16 bits for the float.
pub struct ShaftPosition {
  pub angle: I6F26,
  pub rotations: i32,
  angle_prev: I6F26,
  rotations_prev: i32,
  jump: bool, // set when the angle jumps with a difference bigger than TAU/6
  inversed: bool,
}
impl ShaftPosition {
  pub fn new() -> Self {
    ShaftPosition {
      angle: I6F26::ZERO,
      rotations: 0,
      angle_prev: I6F26::ZERO,
      rotations_prev: 0,
      jump: false,
      inversed: false,
    }
  }
  /// mark the sensor for this motor as inversed or not inversed.
  pub fn set_inversed(&mut self, inv: bool) {
    self.inversed = inv;
  }
  pub fn is_inversed(&self) -> bool {
    self.inversed
  }
  pub fn inc(&mut self, increment: I6F26) {
    self.jump = false;
    self.angle += increment;
    if self.angle >= I6F26::TAU {
      // we have an overflow case while incrementing
      self.rotations += 1;
      self.angle -= I6F26::TAU;
      self.jump = true;
    } else if self.angle < I6F26::ZERO {
      // we have an overflow case while decrementing
      self.rotations -= 1;
      self.angle += I6F26::TAU;
      self.jump = true;
    }
  }
  pub fn has_jumped(&self) -> bool {
    self.jump
  }
  pub fn get_angle(&self) -> I6F26 {
    self.angle
  }
  pub fn set_angle(&mut self, angle: I6F26) {
    self.angle = angle
  }
  pub fn get_rotations(&self) -> i32 {
    self.rotations
  }
  pub fn reset(&mut self) {
    self.rotations = 0;
    self.angle = I6F26::ZERO;
    self.jump = false;
  }
  pub fn set_shaft(&mut self, rotations: i32, angle: I6F26) {
    self.rotations = rotations;
    self.angle = angle;
  }
  pub fn get_position(&self) -> I48F16 {
    I48F16::from_num(self.rotations) * I48F16::TAU + I48F16::from_num(self.angle)
  }

  /// bring the given angle in range of 0..TAU
  pub fn clamp(angle: I6F26) -> I6F26 {
    let mut result = angle;
    while result >= I6F26::TAU {
      result -= I6F26::TAU;
    }
    while result < I6F26::ZERO {
      result += I6F26::TAU;
    }
    result
  }
  /// compare this shaftposition with other. Return difference as an I6F26. Ohter is substracted from this
  pub fn compare(&self, other: &ShaftPosition) -> I16F16 {
    let rotations_diff = (self.rotations - other.get_rotations()) * I16F16::TAU;
    let angle_diff = self.angle - other.get_angle();
    rotations_diff + I16F16::from_num(angle_diff)
  }
  /// Get the delta shaftposition compared with previous stored one. Set the previous to now
  pub fn delta(&mut self) -> I16F16 {
    let rotations_diff = (self.rotations - self.rotations_prev) * I16F16::TAU;
    let angle_diff = self.angle - self.angle_prev;
    self.angle_prev = self.angle;
    self.rotations_prev = self.rotations;
    rotations_diff + I16F16::from_num(angle_diff)
  }

  /// update the shaft angle with the new measured angle. Update the rotation, and check if there is a jump
  pub fn update_shaft_angle(&mut self, new_angle: I6F26) {
    let angle = if self.inversed {
      I6F26::TAU - new_angle
    } else {
      new_angle
    };
    let delta_angle = angle - self.angle;
    self.angle = angle;
    self.jump = false;
    if delta_angle > I6F26::FRAC_TAU_3 {
      // we have an overflow case while rotating CCW
      self.rotations -= 1;
      self.jump = true;
    } else if delta_angle < -I6F26::FRAC_TAU_3 {
      // we have an overflow case while rotating CW
      self.rotations += 1;
      self.jump = true;
    }
  }
}
