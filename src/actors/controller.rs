// The Brain: reads from rx queues, evaluates state, sends to Locomotion
use std::sync::mpsc::{Sender, Receiver};
use crate::actors::leveling_controller::LevelingController;
use crate::messages::{ChangeAutomaticLeveling, ControllerInput, LegStates, MotionIntent, PostureIntent, RollIntent, TelemetryData};
use crate::task::TaskStep;
use crate::config::{BUTTON_DEBOUNCE_TIME, CHANGE_HEIGHT_SPEED, CLOCK_SPEED_SEC, D_TERM_LEVELING, DEFAULT_LED_COLOR, LEG_NOMINAL_Z_DEFAULT, LEG_Z_MAX, LEG_Z_MIN, MAX_GAIT_EXPECTED_STEP_SIZE, MAX_ROLL_RAD, MAX_ROTATION_SPEED, NUMBER_OF_LEDS, P_TERM_LEVELING, RGB_STEP_ADVANCE};
use smart_leds::{RGB8};
use crate::types::StatusColors;

const ALL_FEET_DOWN: u8 = 0b00111111;

pub struct ControllerTask {
    motion_intent_sender: Sender<MotionIntent>,
    controller_data_receiver: Receiver<ControllerInput>,
    current_controller_input: ControllerInput,
    led_command_sender: Sender<StatusColors>,
    posture_sender: Sender<PostureIntent>,
    roll_sender: Sender<RollIntent>,
    leg_states_receiver: Receiver<LegStates>,
    telemetry_data_receiver: Receiver<TelemetryData>,
    color_mode: u8,
    rgb_counter: u8,
    dt_last_button_a_press: f32,
    dt_last_button_x_press: f32,
    current_height: f32,
    current_leg_states: LegStates,
    current_telemetry_data: TelemetryData,
    current_automatic_leveling_mode: bool,
    leveling_controller_x: LevelingController,
    leveling_controller_y: LevelingController,
    automatic_leveling_sender: Sender<ChangeAutomaticLeveling>,
}
impl ControllerTask {
    pub fn new(intent_sender: Sender<MotionIntent>, controller_rcv: Receiver<ControllerInput>, led_send: Sender<StatusColors>, post_send: Sender<PostureIntent>, roll_send: Sender<RollIntent>, leg_states_rcv: Receiver<LegStates>, tel_data_rcv: Receiver<TelemetryData>, autom_lvl_send: Sender<ChangeAutomaticLeveling>) -> Self {
        Self {
            motion_intent_sender: intent_sender,
            controller_data_receiver: controller_rcv,
            current_controller_input: ControllerInput::default(),
            led_command_sender: led_send,
            posture_sender: post_send,
            roll_sender: roll_send,
            leg_states_receiver: leg_states_rcv,
            telemetry_data_receiver: tel_data_rcv,
            color_mode: 0,
            rgb_counter: 0,
            dt_last_button_a_press: 0.0,
            dt_last_button_x_press: 0.0,
            current_height: LEG_NOMINAL_Z_DEFAULT,
            current_leg_states: LegStates::default(),
            current_telemetry_data: TelemetryData::default(),
            current_automatic_leveling_mode: false,
            // Special PID controller versions (PD) with "external" integral term
            leveling_controller_x: LevelingController::new(P_TERM_LEVELING, D_TERM_LEVELING),
            leveling_controller_y: LevelingController::new(P_TERM_LEVELING, D_TERM_LEVELING),
            automatic_leveling_sender: autom_lvl_send,
        }
    }
    fn create_rgb_array_same_color(r: u8, g: u8, b: u8) -> StatusColors {
        let single = RGB8::new(r, g, b);
        [single; NUMBER_OF_LEDS] // For vehicles 3-Led Chain
    }
    // Phase progress value from 0 to 255 for rainbow effect
    fn color_wheel(mut pos: u8) -> RGB8 {
        // Reverse the position so the colors shift in the standard optical order
        pos = 255 - pos;

        if pos < 85 {
            // Red to Green
            RGB8::new(255 - pos * 3, 0, pos * 3)
        } else if pos < 170 {
            // Green to Blue
            pos -= 85;
            RGB8::new(0, pos * 3, 255 - pos * 3)
        } else {
            // Blue to Red
            pos -= 170;
            RGB8::new(pos * 3, 255 - pos * 3, 0)
        }
    }
}
impl TaskStep for ControllerTask {
    fn step(&mut self) {
        // Receive controller data
        while let Ok(input) = self.controller_data_receiver.try_recv() {
            self.current_controller_input = input;
        }

        let stop_ordered = self.current_controller_input.stick_lx == 0.0 && self.current_controller_input.stick_ly == 0.0 
                            && self.current_controller_input.trigger_l == 0.0 && self.current_controller_input.trigger_r == 0.0;
        
        while let Ok(input) = self.leg_states_receiver.try_recv() {
            self.current_leg_states = input;
        }

        while let Ok(input) = self.telemetry_data_receiver.try_recv() {
            self.current_telemetry_data = input;
        }

        if self.current_controller_input.btn_a() && self.dt_last_button_a_press > BUTTON_DEBOUNCE_TIME {
            self.dt_last_button_a_press = 0.0;
            self.color_mode += 1;
        }

        let leds: Option<StatusColors> = match self.color_mode {
            0 => Some(ControllerTask::create_rgb_array_same_color(DEFAULT_LED_COLOR[0], DEFAULT_LED_COLOR[1], DEFAULT_LED_COLOR[2])),
            // Green
            1 => Some(ControllerTask::create_rgb_array_same_color(0, 255, 0)),
            // Blue
            2 => Some(ControllerTask::create_rgb_array_same_color(0, 0, 255)),
            // White
            3 => Some(ControllerTask::create_rgb_array_same_color(255, 255,255)),
            // RGB mode
            4 => {
                self.rgb_counter = ((self.rgb_counter as u16 + RGB_STEP_ADVANCE) % 255) as u8;
                let rgb = ControllerTask::color_wheel(self.rgb_counter);
                Some(ControllerTask::create_rgb_array_same_color(rgb.r, rgb.g, rgb.b))
            },
            // Leg-State Led Mode
            5 => {
                // All feet in the air => Red (Error)
                if self.current_leg_states.leg_states == 0 { Some(ControllerTask::create_rgb_array_same_color(255, 0, 0)) }
                // All feet down => Yellow (Load Warning)
                else if self.current_leg_states.leg_states == ALL_FEET_DOWN { Some(ControllerTask::create_rgb_array_same_color(255, 255, 0)) }
                else {
                    if stop_ordered {
                        Some(ControllerTask::create_rgb_array_same_color(0, 0, 255))
                    }
                    else {
                        Some(ControllerTask::create_rgb_array_same_color(0, 255, 0))
                    }
                }
            },
            _ => {
                self.color_mode = 0;
                self.rgb_counter = 0;
                None
            }
        };

        if let Some(vals) = leds {
            self.led_command_sender.send(vals).unwrap();
        }

        // Every iteration, provide the most recent controller_input for the robots motion system

        let intent = { if self.current_controller_input.stick_lx == 0.0 && self.current_controller_input.stick_ly == 0.0 
            && self.current_controller_input.trigger_l == 0.0 && self.current_controller_input.trigger_r == 0.0 {
                MotionIntent::Stop
            }
            else {
                MotionIntent::Walk { 
                    vx: self.current_controller_input.stick_lx * MAX_GAIT_EXPECTED_STEP_SIZE, 
                    vy: self.current_controller_input.stick_ly * MAX_GAIT_EXPECTED_STEP_SIZE, 
                    omega: {
                        let total_rot_input = self.current_controller_input.trigger_l - self.current_controller_input.trigger_r;
                        total_rot_input * MAX_ROTATION_SPEED
                    }
                }
            }
        };
        
        _ = self.motion_intent_sender.send(intent);

        // Apply change height and roll commands to hardware

        if self.current_controller_input.bumper_r() && !self.current_controller_input.bumper_l() && (self.current_height - CHANGE_HEIGHT_SPEED) > LEG_Z_MAX {
            // Increase height
            self.current_height -= CHANGE_HEIGHT_SPEED;
        }
        else if !self.current_controller_input.bumper_r() && self.current_controller_input.bumper_l() && (self.current_height + CHANGE_HEIGHT_SPEED) < LEG_Z_MIN {
            // Decrease height
            self.current_height += CHANGE_HEIGHT_SPEED;
        }

        let posture_int = PostureIntent {
            standing_height: self.current_height
        };

        _ = self.posture_sender.send(posture_int);

        // Apply the selected roll mode => Automatic / Manual leveling

        if self.current_controller_input.btn_x() && self.dt_last_button_x_press > BUTTON_DEBOUNCE_TIME {
            self.dt_last_button_x_press = 0.0;
            self.current_automatic_leveling_mode = !self.current_automatic_leveling_mode;
            _ = self.automatic_leveling_sender.send(ChangeAutomaticLeveling { automatic_state: self.current_automatic_leveling_mode });
        }

        let roll_intent: RollIntent = {
            // Automatic controll => Sends permanent roll commands to prevent walking during this mode
            if self.current_automatic_leveling_mode {
                RollIntent::Roll { 
                    // Approximate the time difference with the sleep time of the task as it gets close enough to the actual difference
                    // X sensor axis is the Y axis of this system => switch
                    x: self.leveling_controller_x.perform(self.current_telemetry_data.angle_y_rad, CLOCK_SPEED_SEC),
                    y: -self.leveling_controller_y.perform(self.current_telemetry_data.angle_x_rad, CLOCK_SPEED_SEC)
                }
            }
            // Manual controll
            else {
                if self.current_controller_input.stick_rx == 0.0 && self.current_controller_input.stick_ry == 0.0 {
                    RollIntent::NoRoll
                }
                else {
                    RollIntent::Roll { 
                        x: -MAX_ROLL_RAD * self.current_controller_input.stick_rx, 
                        y: -MAX_ROLL_RAD * self.current_controller_input.stick_ry 
                    }
                }
            }
        };

        _ = self.roll_sender.send(roll_intent);

        self.dt_last_button_a_press += CLOCK_SPEED_SEC;
        self.dt_last_button_x_press += CLOCK_SPEED_SEC;
    }
}