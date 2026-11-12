// Servo struct: handles physical limits, offset calibration, tick conversion
use pwm_pca9685::{Channel};
use crate::types::ArmTypes;

#[derive(Debug, Clone, Copy)]
pub struct Servo {
    pub channel: Channel,
    pub leg_id: usize,
    pub arm_type: ArmTypes
}

impl Servo {
    pub fn new(channel: Channel, leg_id: usize, arm_type: ArmTypes) -> Self {
        Self {
            channel,
            leg_id,
            arm_type,
        }
    }
}
