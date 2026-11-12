// The thread loop: receives parameters, generates step, calls hardware

use std::sync::mpsc::{Sender, Receiver};
use crate::config::{CLOCK_SPEED_SEC, LEG_BODY_OFFSETS, LEG_DIMENSIONS, LEG_GROUP_LEFT_MEMBERS, LEG_NOMINAL_Z_DEFAULT, LEG_Z_MAX, LEG_Z_MIN, MAX_GAIT_CYCLE_TIME, MAX_GAIT_EXPECTED_STEP_SIZE, MIN_GAIT_CYCLE_TIME, NUMBER_OF_LEGS, STEP_HEIGHT};
use crate::locomotion::gait::{LegGroup, TripodGait};
use crate::locomotion::kinematics::LegDimensions;
use crate::messages::{LegHardwareCommand, MotionIntent, PostureIntent, RollIntent};
use crate::task::TaskStep;
use crate::types::{JointAngles, Point3D};

pub mod gait;
pub mod kinematics;

pub struct LocomotionTask {
    motion_intent_receive: Receiver<MotionIntent>,
    servo_hardware_send: Sender<LegHardwareCommand>,
    posture_intent_rcv: Receiver<PostureIntent>,
    roll_intent_rcv: Receiver<RollIntent>,
    gait_module: TripodGait,
    ik_module: LegDimensions,
    current_intent: MotionIntent,
    current_nominal_r: f32,
    current_walking_height: f32,
    current_roll_intent: RollIntent,
    current_movement_lock: bool
}
impl LocomotionTask {
    pub fn new(intent_receive: Receiver<MotionIntent>, hardware_send: Sender<LegHardwareCommand>, post_rcv: Receiver<PostureIntent>, roll_rcv: Receiver<RollIntent>) -> Self {
        let leg_dim = LegDimensions { coxa_length: LEG_DIMENSIONS[0], femur_length: LEG_DIMENSIONS[1], tibia_length: LEG_DIMENSIONS[2] };
        let (nominal_r_def, max_stride_radius) = LocomotionTask::get_stance_for_new_z(LEG_NOMINAL_Z_DEFAULT, leg_dim);
        Self {
            motion_intent_receive: intent_receive,
            servo_hardware_send: hardware_send,
            posture_intent_rcv: post_rcv,
            roll_intent_rcv: roll_rcv,
            gait_module: TripodGait::new(MIN_GAIT_CYCLE_TIME, MAX_GAIT_CYCLE_TIME, MAX_GAIT_EXPECTED_STEP_SIZE, STEP_HEIGHT, max_stride_radius),
            ik_module: leg_dim,
            current_intent: MotionIntent::Stop,
            current_nominal_r: nominal_r_def,
            current_walking_height: LEG_NOMINAL_Z_DEFAULT,
            current_roll_intent: RollIntent::NoRoll,
            current_movement_lock: false
        }
    }
    fn get_stance_for_new_z(new_z: f32, ik_mod: LegDimensions) -> (f32 /* nominal_r */, f32 /* max_stride_radius */) {
        let nominal_r_def = ik_mod.get_stance_for_vertical_tibia(new_z).expect("Ride height is physically impossible for femur length!");
        let max_stride_radius = ik_mod.calculate_max_stride_radius(nominal_r_def, new_z);
        (nominal_r_def, max_stride_radius)
    }

    // Calculate the height difference required for a specific leg to perform the ordered rolling move 
    fn get_leg_height_diff(leg_pos: (f32 /*x*/, f32 /*y*/), roll_intent: RollIntent) -> f32 {
        let (x_roll, y_roll) = match roll_intent {
            RollIntent::NoRoll => (0.0, 0.0),
            RollIntent::Roll { x, y } => (x,y),
        };
        x_roll.tan() * leg_pos.0 + y_roll.tan() * leg_pos.1
    }
    
}
impl TaskStep for LocomotionTask {
    fn step(&mut self) {
        while let Ok(intent) = self.motion_intent_receive.try_recv() {
            self.current_intent = intent;
        }
        while let Ok(post_intent) = self.posture_intent_rcv.try_recv() {
            let (nominal_r_def, max_stride_radius) = LocomotionTask::get_stance_for_new_z(post_intent.standing_height, self.ik_module);
            self.current_nominal_r = nominal_r_def;
            self.gait_module.update_max_stride_radius(max_stride_radius);
            self.current_walking_height = post_intent.standing_height;
        }
        while let Ok(roll_intent) = self.roll_intent_rcv.try_recv() {
            self.current_roll_intent = roll_intent;
            if roll_intent == RollIntent::NoRoll {
                if self.current_movement_lock {self.current_movement_lock = false}
            }
        }
        let mut angle_array: [JointAngles; NUMBER_OF_LEGS] = [JointAngles::default(); NUMBER_OF_LEGS];
        for i in 0..NUMBER_OF_LEGS {
            let group: LegGroup = if LEG_GROUP_LEFT_MEMBERS[i] {LegGroup::TripodLeft} else {LegGroup::TripodRight};
            let offset = LEG_BODY_OFFSETS[i];
            let target_positions: Point3D = {
                if self.current_movement_lock { Point3D::default() } // No walking allowed in this mode to ensure stable rolling
                else { self.gait_module.calculate_leg_offset(group, &self.current_intent, Point3D {x: offset.0, y: offset.1, z: offset.2}, offset.3) }
            };
            let leg_height_diff: f32 = {
                // Ensure, rolling only starts when the vehicle is static and all legs are on the ground
                if self.current_roll_intent != RollIntent::NoRoll && self.current_intent == MotionIntent::Stop && self.gait_module.get_current_phase() == 0.0 {
                    // Lock movement while rolling to keep vehicle stable
                    if !self.current_movement_lock {self.current_movement_lock = true}
                    LocomotionTask::get_leg_height_diff((offset.0, offset.1), self.current_roll_intent)
                }
                else { 0.0 }
            };

            // "-" required to add diff to apply correctly to the orientation system
            let absolute_offset = self.ik_module.get_absolute_ik_target(&target_positions, self.current_nominal_r, (self.current_walking_height - leg_height_diff).clamp(LEG_Z_MAX, LEG_Z_MIN));
            let angles = self.ik_module.calculate_ik(&absolute_offset);

            if let Err(_) = &angles {
                log::error!("IK returned maxed out angles due to invalid position - Continuing with maxed out value");
            }
            let checked_angles = match angles {
                Ok(ang) => ang,
                Err(ang_with_err) => {
                    log::error!("IK returned maxed out angles due to invalid position - Continuing with maxed out value");
                    ang_with_err.0
                },
            };
            angle_array[i] = checked_angles;
        }
        _ = self.servo_hardware_send.send(LegHardwareCommand {angles: angle_array});
        self.gait_module.update_clock(CLOCK_SPEED_SEC, &self.current_intent);
    }  
}