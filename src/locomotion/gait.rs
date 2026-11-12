// State machine: generates XYZ foot positions over time

use crate::types::Point3D;
use crate::messages::MotionIntent;
use crate::config::{STOP_PHASE_THRESHOLD};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LegGroup {
    TripodLeft,  // (Front-Left: 0, Middle-Right: 3, Rear-Left: 4)
    TripodRight, // (Front-Right: 1, Middle-Left: 2, Rear-Right: 5)
}

pub struct TripodGait {
    min_cycle_time_sec: f32, // Fastest cadence at max speed
    max_cycle_time_sec: f32, // Slowest cadence at slowest speed
    max_expected_speed: f32,
    step_height: f32,
    max_stride_radius: f32,  // Maximum allowable X/Y combination in mm
    current_phase: f32,      // Normalized cycle progress [0.0; 1.0]
}

impl TripodGait {
    pub fn new(min_time: f32, max_time: f32, max_speed: f32, step_height: f32, max_stride: f32) -> Self {
        Self {
            min_cycle_time_sec: min_time,
            max_cycle_time_sec: max_time,
            max_expected_speed: max_speed,
            step_height,
            max_stride_radius: max_stride,
            current_phase: 0.0,
        }
    }

    pub fn update_max_stride_radius(&mut self, max_stride: f32){
        self.max_stride_radius = max_stride; 
    }
    
    pub fn get_current_phase(&self) -> f32{
        self.current_phase
    }

    /// Advances the internal gait phase based on the current command intensity - Called once per loop tick
    pub fn update_clock(&mut self, dt_sec: f32, intent: &MotionIntent) {
        let cycle_time = self.calculate_dynamic_cycle_time(intent);
        
        // Prevent division by zero and halt phase advancement if stopped
        if cycle_time > 0.0 {
            self.current_phase = (self.current_phase + (dt_sec / cycle_time)) % 1.0;
        } else if cycle_time == 0.0 && (self.current_phase < STOP_PHASE_THRESHOLD || self.current_phase > 1.0 - STOP_PHASE_THRESHOLD) {
            // Stop when foot has finished its cycle
            self.current_phase = 0.0;
        } else {
            // Wait for foot to finish cycle (finish fast with shortest path)
            if self.current_phase >= 0.5 {
                self.current_phase = (self.current_phase + (dt_sec / self.max_cycle_time_sec)) % 1.0;
            }
            else {
                // Lower foot to reach home position faster
                self.current_phase = (self.current_phase - (dt_sec / self.max_cycle_time_sec)).clamp(0.0, 1.0);
            }
        }  

    }

    /// Scales cycle time to reach ordered speed
    fn calculate_dynamic_cycle_time(&self, intent: &MotionIntent) -> f32 {
        let (vx, vy, omega) = match intent {
            MotionIntent::Walk { vx, vy, omega } => (*vx, *vy, *omega),
            MotionIntent::Stop => return 0.0,
        };

        // Combines rotational and translational speeds into a single value
        let speed_magnitude = (vx.powi(2) + vy.powi(2)).sqrt() + (omega.abs() * 50.0);

        let speed_factor = (speed_magnitude / self.max_expected_speed).clamp(0.0, 1.0);

        // Linearily interpolize to match slowest speed with maximum cycle time and vice versa
        self.max_cycle_time_sec - speed_factor * (self.max_cycle_time_sec - self.min_cycle_time_sec)
    }

    /// Calculate the XYZ offset for a specific leg based on omnidirectional body velocity
    pub fn calculate_leg_offset(
        &self,
        group: LegGroup,
        intent: &MotionIntent,
        leg_center_offset: Point3D,
        leg_mount_angle_deg: f32,
    ) -> Point3D {
        let leg_mount_angle_rad = leg_mount_angle_deg.to_radians();

        let (vx, vy, omega) = match intent {
            MotionIntent::Walk { vx, vy, omega } => (*vx, *vy, *omega),
            MotionIntent::Stop => ( 0.0, 0.0, 0.0 ),
        };

        // Calculates how this specific point on the chassis is moving based on translation + rotation in global body space
        let mut global_stride_x = vx - (omega * leg_center_offset.y);
        let mut global_stride_y = vy + (omega *leg_center_offset.x);

        // Prevent the generated XYZ from demanding angles outside the valid foot stride radius
        let stride_magnitude = (global_stride_x.powi(2) + global_stride_y.powi(2)).sqrt();
        if stride_magnitude > self.max_stride_radius {
            // Preserve the angle of the stride, but scale the length down to the safe limit
            let scale_factor = self.max_stride_radius / stride_magnitude;
            global_stride_x *= scale_factor;
            global_stride_y *= scale_factor;
        }

        // Use 2D rotation matrix with the legs physical mounting angle to rotate to local leg coordinate space
        let leg_stride_x = global_stride_x * leg_mount_angle_rad.cos() + global_stride_y * leg_mount_angle_rad.sin();
        let leg_stride_y = -global_stride_x * leg_mount_angle_rad.sin() + global_stride_y * leg_mount_angle_rad.cos();

        let leg_phase = match group {
            LegGroup::TripodLeft => self.current_phase,
            LegGroup::TripodRight => (self.current_phase + 0.5) % 1.0,
        };

        let result = if leg_phase < 0.5 {
            // Apply swing mode
            let swing_t = leg_phase * 2.0;
            
            // Limit Z height to ensure the femur doesn't exceed bounds
            let safe_step_height = self.step_height;
            let z = safe_step_height * (std::f32::consts::PI * swing_t).sin();

            // Ensure that leg is 90° to the ground when touching down
            let touchdown_threshold = 0.80;
            let xy_normalized = if swing_t < touchdown_threshold {
                let ease_t = swing_t / touchdown_threshold;
                0.5 - 0.5 * (std::f32::consts::PI * ease_t).cos()
            } else {
                1.0
            };

            let progress = xy_normalized - 0.5;

            Point3D {
                x: leg_stride_x * progress,
                y: leg_stride_y * progress,
                z,
            }
        } else {
            // Apply stance mode
            let stance_t = (leg_phase - 0.5) * 2.0;
            let progress = 0.5 - stance_t; 

            Point3D {
                x: leg_stride_x * progress,
                y: leg_stride_y * progress,
                z: 0.0, 
            }
        };

        result
    }
}