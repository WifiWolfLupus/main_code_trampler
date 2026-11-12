mod actors;
mod config;
mod hardware;
mod locomotion;
mod messages;
mod types;
mod task;
use std::sync::{mpsc};
use std::thread;
use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;

use crate::actors::controller::ControllerTask;
use crate::config::{CLOCK_SPEED_SEC};
use crate::hardware::HardwareTask;
use crate::locomotion::LocomotionTask;
use crate::messages::{ChangeAutomaticLeveling, ControllerInput, LegHardwareCommand, LegStates, MotionIntent, PostureIntent, RollIntent, TelemetryData};
use crate::task::TaskStep;
use crate::types::StatusColors;

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Starting up");
    
    let peripherals = Peripherals::take().unwrap();

    // Channels used for intertask communication
    let (tx_hardware, rx_hardware) = mpsc::channel::<LegHardwareCommand>();
    let (tx_intent, rx_intent) = mpsc::channel::<MotionIntent>();
    let (tx_controller_input, rx_controller_input) = mpsc::channel::<ControllerInput>();
    let (tx_led_command, rx_led_command) = mpsc::channel::<StatusColors>();
    let (tx_posture_command, rx_posture_command) = mpsc::channel::<PostureIntent>();
    let (tx_roll_command, rx_roll_command) = mpsc::channel::<RollIntent>();
    let (tx_leg_states_command, rx_leg_states_command) = mpsc::channel::<LegStates>();
    let (tx_telemetry_data_command, rx_telemetry_data_command) = mpsc::channel::<TelemetryData>();
    let (tx_auto_leveling_command, rx_auto_leveling_command) = mpsc::channel::<ChangeAutomaticLeveling>();


    let core0_config = ThreadSpawnConfiguration {
        pin_to_core: Some(Core::Core0),
        ..Default::default()
    };

    let core1_config = ThreadSpawnConfiguration {
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    };

    core1_config.set().unwrap(); // Affects the tasks spawned after this call until sth. else is set

    let hardware_task = thread::Builder::new()
        .name("Hardware Task".into())
        .stack_size(8192)
        .spawn(move || {
            let mut hardware_task_struct = HardwareTask::new(rx_hardware, rx_auto_leveling_command, tx_controller_input, rx_led_command, tx_leg_states_command, peripherals, tx_telemetry_data_command);

            loop {
                hardware_task_struct.step();
                thread::sleep(Duration::from_secs_f32(CLOCK_SPEED_SEC));
            }
        }
    ).unwrap();

    let locomotion_task = thread::Builder::new()
        .name("Locomotion Task".into())
        .stack_size(16384)
        .spawn(move || {
            let mut lo_task_struct = LocomotionTask::new(rx_intent, tx_hardware, rx_posture_command, rx_roll_command);

            loop {
                lo_task_struct.step();
                thread::sleep(Duration::from_secs_f32(CLOCK_SPEED_SEC));
            }
        }
    ).unwrap();

    core0_config.set().unwrap();

    let controller_task = thread::Builder::new()
        .name("Controller Task".into())
        .stack_size(8192)
        .spawn(move || {
            let mut controller_task_struct = ControllerTask::new(tx_intent, rx_controller_input, tx_led_command, tx_posture_command, tx_roll_command, rx_leg_states_command, rx_telemetry_data_command, tx_auto_leveling_command);

            loop {
                controller_task_struct.step();
                thread::sleep(Duration::from_secs_f32(CLOCK_SPEED_SEC));
            }
        }
    ).unwrap();

    // Wait for these tasks to finish => prevent main thread from exiting
    hardware_task.join().unwrap();
    locomotion_task.join().unwrap();
    controller_task.join().unwrap();
}
