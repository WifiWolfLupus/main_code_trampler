#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct JointAngles {
    pub coxa: f32,
    pub femur: f32,
    pub tibia: f32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum ArmTypes{
    Coxa,
    Femur,
    Tibia
}

use smart_leds::{RGB8};
use crate::{config::NUMBER_OF_LEDS, hardware::FastI2c};

pub type StatusColors = [RGB8; NUMBER_OF_LEDS];

use embedded_hal_compat::Reverse;
use embedded_hal_bus::i2c::RefCellDevice;

pub type MpuType<'a> = Reverse<RefCellDevice<'a, FastI2c<'a>>>;