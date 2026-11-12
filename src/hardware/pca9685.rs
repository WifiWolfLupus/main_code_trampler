// Low-level I2C commands for the PWM board

use embedded_hal_bus::i2c::RefCellDevice;
use pwm_pca9685::{Pca9685};
use crate::config::{LEG_COXA_INVERT_RIGHT, LEG_COXA_MAX, LEG_COXA_MIN, LEG_COXA_ZERO_OFFSET, LEG_FEMUR_INVERT_RIGHT, LEG_FEMUR_MAX, LEG_FEMUR_MIN, LEG_FEMUR_ZERO_OFFSET, LEG_TIBIA_INVERT_RIGHT, LEG_TIBIA_MAX, LEG_TIBIA_MIN, LEG_TIBIA_ZERO_OFFSET, SERVO_MAX_TICKS, SERVO_MIN_TICKS};
use crate::hardware::FastI2c;
use crate::types::ArmTypes;

use crate::{config::NUMBER_OF_LEGS,config::NUMBER_SERVOS_PER_LEG, hardware::servo::Servo, types::JointAngles};

pub struct ServoDriver<'a> {
    pub servos: [Servo; NUMBER_SERVOS_PER_LEG * NUMBER_OF_LEGS],
    pub pwm_right: Pca9685<RefCellDevice<'a, FastI2c<'a>>>,
    pub pwm_left: Pca9685<RefCellDevice<'a, FastI2c<'a>>>
}
impl<'a> ServoDriver<'a> {
    pub fn new(servos: [Servo; NUMBER_SERVOS_PER_LEG * NUMBER_OF_LEGS], pwm_right: Pca9685<RefCellDevice<'a, FastI2c<'a>>>, pwm_left: Pca9685<RefCellDevice<'a, FastI2c<'a>>>) -> Self{
        Self {
            servos,
            pwm_right,
            pwm_left,
        }
    }

    pub fn apply_joint_angles(&mut self, joint_angles: &[JointAngles; NUMBER_OF_LEGS]){
        for i in 0..NUMBER_OF_LEGS {
            let angles = joint_angles[i];
            let right_side = i % 2 != 0; // Odd leg id = right side
            let pwm_mod = if right_side {&mut self.pwm_right} else {&mut self.pwm_left};
            Self::set_servo_angle(&self.servos[NUMBER_SERVOS_PER_LEG * i], angles.coxa, pwm_mod, right_side);
            Self::set_servo_angle(&self.servos[NUMBER_SERVOS_PER_LEG * i + 1], angles.femur, pwm_mod, right_side);
            Self::set_servo_angle(&self.servos[NUMBER_SERVOS_PER_LEG * i + 2], angles.tibia, pwm_mod, right_side);
        }
    }
    fn set_servo_angle(servo: &Servo, angle: f32, pwm: &mut Pca9685<RefCellDevice<'_, FastI2c<'_>>>, right_side: bool){
        let physically_adjusted_angle = match servo.arm_type {
            ArmTypes::Coxa => Self::offset_with_clamp_and_invert(angle, LEG_COXA_ZERO_OFFSET, LEG_COXA_MIN, LEG_COXA_MAX, Self::invert_check(LEG_COXA_INVERT_RIGHT, right_side) || true),
            ArmTypes::Femur => Self::offset_with_clamp_and_invert(angle, LEG_FEMUR_ZERO_OFFSET, LEG_FEMUR_MIN, LEG_FEMUR_MAX, Self::invert_check(LEG_FEMUR_INVERT_RIGHT, right_side)),
            ArmTypes::Tibia => Self::offset_with_clamp_and_invert(angle, LEG_TIBIA_ZERO_OFFSET, LEG_TIBIA_MIN, LEG_TIBIA_MAX, Self::invert_check(LEG_TIBIA_INVERT_RIGHT, right_side) || !right_side), 
        };
        let ticks = Self::map_angle_to_ticks(physically_adjusted_angle);
        if let Err(_) = pwm.set_channel_on_off(servo.channel, 0, ticks){
            log::error!("Write to channel failed on Servo PCA Driver");
        }
    }
    fn invert_check(invert_hardware: bool, right_side: bool) -> bool {
        right_side && invert_hardware
    }
    fn offset_with_clamp_and_invert(angle: f32, offset: f32, min_angle: f32, max_angle: f32, invert: bool) -> f32{
        let ideal_allowed_val = angle.clamp(min_angle, max_angle);
        let val = ideal_allowed_val - offset;
        if invert { -val } else { val }
    }
    fn map_angle_to_ticks(angle: f32) -> u16{
        let ticks_per_degree = (SERVO_MAX_TICKS - SERVO_MIN_TICKS) as f32 / 180.0;
        let res = ((angle + 90.0).clamp(0.0,180.0) * ticks_per_degree).round() as u16 + SERVO_MIN_TICKS; // Shift angle into 0-180° range with security clamp
        res
    }
}