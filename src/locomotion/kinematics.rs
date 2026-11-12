use crate::types::{JointAngles, Point3D};

#[derive(Debug, Clone, Copy)]
pub struct LegDimensions {
    pub coxa_length: f32,  // S1
    pub femur_length: f32, // S2
    pub tibia_length: f32, // S3
}

#[derive(Debug)]
pub enum IkError {
    TargetOutOReach, // Clamped Values get returned instead
}

impl LegDimensions {
    /// Calculates the Inverse Kinematics for a hexapod leg with three servos.
    /// Returns joint angles in degrees. 
    pub fn calculate_ik(&self, target: &Point3D) -> Result<JointAngles, (JointAngles, IkError)> {
        // Calculated in the horizontal XY plane from the origin for the coxa angle
        let coxa_rad = target.y.atan2(target.x);
        
        // Horizontal distance from coxa joint to target
        let radial_dist = (target.x.powi(2) + target.y.powi(2)).sqrt();
        
        // Horizontal distance from the femur joint to the target
        let l_dist = radial_dist - self.coxa_length;
        
        // Direct distance from femur joint to foot target
        let direct_dist = (l_dist.powi(2) + target.z.powi(2)).sqrt();
        
        let is_reachable = direct_dist <= (self.femur_length + self.tibia_length);
        
        // Clamp the distance to prevent NaN panics in acos()
        let direct_dist = direct_dist.min(self.femur_length + self.tibia_length);

        // Using Law of Cosines: C^2 = A^2 + B^2 - 2AB*cos(c) for tibia angle
        let cos_tibia = (self.femur_length.powi(2) + self.tibia_length.powi(2) - direct_dist.powi(2)) 
            / (2.0 * self.femur_length * self.tibia_length);
        let cos_tibia_clamped = cos_tibia.clamp(-1.0, 1.0);
        let tibia_rad = cos_tibia_clamped.acos();

        // Angle from horizontal to the direct distance line for 
        let phi_1 = target.z.atan2(l_dist);
        
        // Internal angle between femur and the direct distance line
        let cos_phi_2 = (self.femur_length.powi(2) + direct_dist.powi(2) - self.tibia_length.powi(2)) 
            / (2.0 * self.femur_length * direct_dist);
        let cos_phi_2_clamped = cos_phi_2.clamp(-1.0, 1.0);
        let phi_2 = cos_phi_2_clamped.acos();
        
        // Total femur angle relative to horizontal
        let femur_rad = phi_1 + phi_2;

        let angles = JointAngles {
            coxa: coxa_rad.to_degrees(),
            femur: femur_rad.to_degrees(),
            tibia: tibia_rad.to_degrees() - 180.0, // Results for tibia must be mapped to current system
        };

        if is_reachable {
            Ok(angles)
        } else {
            // Returns the angles stretched directly towards the unreachable point
            Err((angles, IkError::TargetOutOReach))
        }
    }

    /// Calculates the required horizontal reach to ensure the Tibia is perfectly vertical for a given height
    /// target_z is measured from the coxa joint => reduce target_z to elevate the platform
    pub fn get_stance_for_vertical_tibia(&self, target_z: f32) -> Option<f32> {
        let knee_z = target_z + self.tibia_length;
        
        let femur_sq = self.femur_length.powi(2);
        let knee_z_sq = knee_z.powi(2);
        
        // Check if the femur is physically long enough to reach this height
        if knee_z_sq > femur_sq {
            return None; 
        }
        
        // Calculate the horizontal distance from the coxa pivot to the knee
        let knee_x = (femur_sq - knee_z_sq).sqrt();
        
        // Due to the tibia being vertical, the x component of the foot only comprises of the distance from the 
        // mounting point to the tibia joint
        Some(self.coxa_length + knee_x)
    }

    /// Calculates the maximum safe stride radius (in mm) for a given standing height
    /// => called when platforms height gets changed
    pub fn calculate_max_stride_radius(&self, nominal_r: f32, nominal_z: f32) -> f32 {
        let mut r_min_reach = nominal_r;
        let mut r_max_reach = nominal_r;
        
        // Sweep outward to find the max reach
        for r_test in (nominal_r as i32..=(self.femur_length + self.tibia_length) as i32).step_by(1) {
            let target = Point3D { x: r_test as f32, y: 0.0, z: nominal_z };
            if let Ok(angles) = self.calculate_ik(&target) {
                // Check against hardware limits
                if angles.femur <= crate::config::LEG_FEMUR_MAX 
                    && angles.femur >= crate::config::LEG_FEMUR_MIN 
                    && angles.tibia <= crate::config::LEG_TIBIA_MAX 
                    && angles.tibia >= crate::config::LEG_TIBIA_MIN 
                {
                    r_max_reach = r_test as f32;
                    continue;
                }
            }
            break; // Hit an IK or physical boundary
        }

        // Sweep inward to find the min reach
        for r_test in (0..=(nominal_r as i32)).rev().step_by(1) {
            let target = Point3D { x: r_test as f32, y: 0.0, z: nominal_z };
            if let Ok(angles) = self.calculate_ik(&target) {
                if angles.femur <= crate::config::LEG_FEMUR_MAX 
                    && angles.femur >= crate::config::LEG_FEMUR_MIN 
                    && angles.tibia <= crate::config::LEG_TIBIA_MAX 
                    && angles.tibia >= crate::config::LEG_TIBIA_MIN 
                {
                    r_min_reach = r_test as f32;
                    continue;
                }
            }
            break; 
        }

        // Keep 10% safety margin from the max reach border
        let forward_clearance = r_max_reach - nominal_r;
        let backward_clearance = nominal_r - r_min_reach;
        
        forward_clearance.min(backward_clearance) * 0.90
    }

    /// Converts a relative gait offset into an coordinate set relative to the mounting point
    /// required by the Inverse Kinematics solver.
    pub fn get_absolute_ik_target(
        &self,
        gait_offset: &Point3D,
        nominal_r: f32,
        nominal_z: f32,
    ) -> Point3D {
        let target = Point3D {
            // Outward sweep: Resting reach + lateral step movement
            x: nominal_r + gait_offset.x,
            
            // Forward sweep: Centered at 0 + forward/backward step movement
            y: gait_offset.y,
            
            // Vertical sweep: Resting height + vertical step height
            z: nominal_z + gait_offset.z,
        };
        target
    }
}