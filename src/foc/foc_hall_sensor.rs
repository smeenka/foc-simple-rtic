// lookup table from the raw hall state (1-3-2-6-4-5  as index to get the hall state )
// -1 indicates invalid hall state
//                                   0  1  2  3  4  5  6   7
const HALL_STATE_TABLE: [i8; 8] = [-1, 0, 2, 1, 4, 5, 3, -1];
pub struct FocHallSensor {
}

impl FocHallSensor {
  /// calculate from the raw hall state, the functional hall state 0-1-2-3-4-5
  /// incoming raw state is the hall sensor state in the first 3 bits. Valid transitions: 1, 3, 2 ,6 ,4 ,5
  /// Return -1 if the raw state is invalid
  #[inline]
  pub fn calc_hall_state(raw_state: usize) -> i8 {
    if  raw_state > 7 {
      -1
    } else {
      HALL_STATE_TABLE[raw_state]
    }  
  }
}
