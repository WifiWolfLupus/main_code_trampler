// THE GLUE: All cross-thread enums (MotionIntent, TelemetryData)
use crate::{config::NUMBER_OF_LEGS, types::JointAngles};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionIntent {
    Walk {
        vx: f32,
        vy: f32,
        omega: f32
    },
    Stop
}

#[derive(Debug, Clone, Copy)]
pub struct PostureIntent {
    pub standing_height: f32
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RollIntent {
    NoRoll,
    Roll {x: f32, y: f32} // In rad
}

#[derive(Debug, Clone, Copy, Default)]
pub struct TelemetryData {
    pub angle_x_rad: f32,
    pub angle_y_rad: f32
}

#[derive(Debug, Clone, Copy, Default)]
pub struct LegStates {
    pub leg_states: u8
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ControllerInput {
    // Both joysticks normalized [-1.0, 1.0]
    pub stick_lx: f32,
    pub stick_ly: f32,
    pub stick_rx: f32,
    pub stick_ry: f32,

    // Shoulder levers (Triggers) normalized [0.0, 1.0]
    pub trigger_l: f32,
    pub trigger_r: f32,

    // Packed bitmask for all 10 buttons
    pub buttons: u16,
}

impl ControllerInput {
    // Bitmask constants matching the Arduino Co-Processor payload
    const MASK_A: u16        = 1 << 0;
    const MASK_B: u16        = 1 << 1;
    const MASK_X: u16        = 1 << 2;
    const MASK_Y: u16        = 1 << 3;
    const MASK_BUMPER_L: u16 = 1 << 4;
    const MASK_BUMPER_R: u16 = 1 << 5;
    const MASK_DPAD_UP: u16  = 1 << 6;
    const MASK_DPAD_DOWN: u16= 1 << 7;
    const MASK_DPAD_LEFT: u16= 1 << 8;
    const MASK_DPAD_RIGHT: u16= 1 << 9;

    // Helper methods for clean access in other tasks
    #[inline] pub fn btn_a(&self) -> bool { (self.buttons & Self::MASK_A) != 0 }
    #[inline] pub fn btn_b(&self) -> bool { (self.buttons & Self::MASK_B) != 0 }
    #[inline] pub fn btn_x(&self) -> bool { (self.buttons & Self::MASK_X) != 0 }
    #[inline] pub fn btn_y(&self) -> bool { (self.buttons & Self::MASK_Y) != 0 }
    
    #[inline] pub fn bumper_l(&self) -> bool { (self.buttons & Self::MASK_BUMPER_L) != 0 }
    #[inline] pub fn bumper_r(&self) -> bool { (self.buttons & Self::MASK_BUMPER_R) != 0 }
    
    #[inline] pub fn dpad_up(&self) -> bool { (self.buttons & Self::MASK_DPAD_UP) != 0 }
    #[inline] pub fn dpad_down(&self) -> bool { (self.buttons & Self::MASK_DPAD_DOWN) != 0 }
    #[inline] pub fn dpad_left(&self) -> bool { (self.buttons & Self::MASK_DPAD_LEFT) != 0 }
    #[inline] pub fn dpad_right(&self) -> bool { (self.buttons & Self::MASK_DPAD_RIGHT) != 0 }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LegHardwareCommand {
    pub angles: [JointAngles; NUMBER_OF_LEGS],
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ChangeAutomaticLeveling {
    pub automatic_state: bool,
}
