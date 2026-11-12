use core::f32::consts::PI;

pub const MIN_GAIT_CYCLE_TIME: f32 = 0.30;          // Fastest cadence at max speed
pub const MAX_GAIT_CYCLE_TIME: f32 = 2.0;           // Slowest cadence at creep speed
pub const STOP_PHASE_THRESHOLD: f32 = 0.1;
pub const MAX_GAIT_EXPECTED_STEP_SIZE: f32 = 150.0; // Normalization threshold for input velocity
pub const STEP_HEIGHT: f32 = 40.0;                  // Peak Z lift during swing 
pub const MAX_ROTATION_SPEED: f32 = 2.5;

pub const NUMBER_OF_LEGS: usize = 6;
// Coxa, Femur, Tibia in mm
pub const LEG_DIMENSIONS: [f32; 3] = [34.9, 65.0, 111.75];
pub const LEG_GROUP_LEFT_MEMBERS: [bool; 6] = [true, false, false, true, true, false]; 
// X / Y / Z / ANGLE for Leg 0, 1, 2, ... Lengths in mm and angles in degrees with degrees being measured from the X-Axis (to the right of the robot) 
pub const LEG_BODY_OFFSETS: [(f32, f32, f32, f32); 6] = [(-52.0, 117.0, 0.0, 135.0), (52.0, 117.0, 0.0, 45.0), (-88.0, 0.0, 0.0, 180.0), (88.0, 0.0, 0.0, 0.0), (-52.0, -117.0, 0.0, -135.0), (52.0, -117.0, 0.0, -45.0)];
pub const CLOCK_SPEED_SEC: f32 = 0.02; // 20ms clock speed
pub const BUTTON_DEBOUNCE_TIME: f32 = 0.1;

pub const SERVO_MIN_TICKS: u16 = 102;
pub const SERVO_MAX_TICKS: u16 = 512;
pub const PRESCALE_50HZ: u8 = 121;
pub const LEFT_PCA_ADDRESS: u8 = 0x40;
pub const RIGHT_PCA_ADDRESS: u8 = 0x41;
pub const NUMBER_SERVOS_PER_LEG: usize = 3;

// Angles in degree in ideal measures from ideal zero point
pub const LEG_COXA_MIN: f32 = -50.0;
pub const LEG_COXA_MAX: f32 = 50.0;
pub const LEG_FEMUR_MIN: f32 = -45.0;
pub const LEG_FEMUR_MAX: f32 = 45.0;
pub const LEG_TIBIA_MIN: f32 = -130.0;
pub const LEG_TIBIA_MAX: f32 = 0.0;

pub const LEG_NOMINAL_Z_DEFAULT: f32 = -100.0;
pub const LEG_Z_MAX: f32 = -150.0;
pub const LEG_Z_MIN: f32 = -70.0; 
pub const MAX_ROLL_RAD: f32 = PI / 6.0;

pub const CHANGE_HEIGHT_SPEED: f32 = 20.0 / (1.0 / CLOCK_SPEED_SEC); // mm / tick 

pub const LEG_TIBIA_ZERO_OFFSET: f32 = -85.0; // The Servo 90° (Center) is at -55°
pub const LEG_FEMUR_ZERO_OFFSET: f32 = -5.0;
pub const LEG_COXA_ZERO_OFFSET: f32 = -10.0;

// Invert means that positive movement requires negative servo movement
pub const LEG_COXA_INVERT_RIGHT: bool = true;
pub const LEG_FEMUR_INVERT_RIGHT: bool = true; 
pub const LEG_TIBIA_INVERT_RIGHT: bool = false; 

pub const NUMBER_OF_LEDS: usize = 3;
pub const DEFAULT_LED_COLOR: [u8; NUMBER_OF_LEDS] = [255,0,0];
pub const RGB_STEP_ADVANCE: u16 = 1;

// Low Pass Filter - Telemetry Data
pub const ALPHA_FILTER: f32 = 0.15;

// PID Controller - Tilt controll
// In rad/s
pub const P_TERM_LEVELING: f32 = 2.0;
pub const D_TERM_LEVELING: f32 = 0.5;

pub const LEVELING_TARGET: f32 = 0.0; // in rad
pub const REACHED_TARGET_AREA: f32 = 0.0;
