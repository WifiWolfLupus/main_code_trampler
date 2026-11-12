use crate::config::{LEVELING_TARGET, MAX_ROLL_RAD, REACHED_TARGET_AREA};

pub struct LevelingController {
    p_portion: f32,
    d_portion: f32,
    last_error: f32,
    pub current_roll_absolute: f32 // Provides the simulated joystick input for tilt in the specified direction
}

impl LevelingController {
    pub fn new(p_p: f32, d_p: f32) -> Self {
        Self { p_portion: p_p, d_portion: d_p, last_error: 0.0, current_roll_absolute: 0.0 }
    }

    pub fn perform(&mut self, current_tilt: f32, t_diff: f32) -> f32 {
        let error = LEVELING_TARGET - current_tilt;

        if error.abs() > REACHED_TARGET_AREA {
            let p_term = error * self.p_portion;
            let d_term = ((error - self.last_error) / t_diff) * self.d_portion;
            let controller_res = p_term + d_term;

            self.last_error = error;

            // The "external" integral over the controller output => produces positional target and removes need for I-Term in controller
            self.current_roll_absolute += controller_res * t_diff; // integral over angular velocity to achieve positional angle

            self.current_roll_absolute = self.current_roll_absolute.clamp(-MAX_ROLL_RAD, MAX_ROLL_RAD);
        }
        else{
            self.last_error = error;
        }
        self.current_roll_absolute    
    }
}