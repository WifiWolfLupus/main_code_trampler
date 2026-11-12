// The hardware task holds and manages all hardware objects and interactions
use std::cell::{RefCell};
use embedded_hal_bus::i2c::RefCellDevice;
use std::sync::mpsc::{Sender, Receiver};
use crate::config::{LEFT_PCA_ADDRESS, NUMBER_OF_LEGS, NUMBER_SERVOS_PER_LEG, PRESCALE_50HZ, RIGHT_PCA_ADDRESS, NUMBER_OF_LEDS, DEFAULT_LED_COLOR};
use crate::hardware::led_controller::LedController;
use crate::hardware::mpu6050_reader::MPU6050Reader;
use crate::hardware::pca9685::ServoDriver;
use crate::hardware::servo::Servo;
use crate::hardware::leg_switch_reader::LegReader;
use crate::hardware::usb_reader::USBReader;
use crate::messages::{ChangeAutomaticLeveling, ControllerInput, LegHardwareCommand, LegStates, TelemetryData};
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver, I2cError};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::units::FromValueType;
use pwm_pca9685::{Channel, Pca9685};
use crate::task::TaskStep;
use crate::types::{ArmTypes, StatusColors};
use std::iter::repeat;
pub use ws2812_esp32_rmt_driver::lib_smart_leds::{Ws2812Esp32Rmt};
use smart_leds::{RGB8, SmartLedsWrite};
use esp_idf_svc::hal::gpio::{PinDriver, Pull, Input};
use mpu6050::Mpu6050;
use embedded_hal_compat::Reverse;
use esp_idf_svc::hal::delay::FreeRtos;
use std::time::Duration;
pub mod pca9685;
pub mod servo;
pub mod usb_reader;
pub mod led_controller;
pub mod leg_switch_reader;
pub mod mpu6050_reader;
pub mod low_pass_filter;

pub struct HardwareTask<'a> {
    leg_command_receive: Receiver<LegHardwareCommand>,
    automatic_leveling_state_receive: Receiver<ChangeAutomaticLeveling>,
    current_legs_command: LegHardwareCommand,
    previous_legs_command: LegHardwareCommand,
    servo_driver: pca9685::ServoDriver<'a>,
    controller_input_rcv: USBReader,
    led_controller: LedController<'a>,
    leg_reader: LegReader<'a>,
    mpu_reader: MPU6050Reader<'a>,
    current_auto_leveling_state: bool
}
impl HardwareTask<'_> {
    pub fn new(command_receive: Receiver<LegHardwareCommand>, change_leveling_rcv: Receiver<ChangeAutomaticLeveling>, controller_input_send: Sender<ControllerInput>, led_cmd_receive: Receiver<StatusColors>, leg_reader_sender: Sender<LegStates>, peripheral: Peripherals, telemetry_sender: Sender<TelemetryData>) -> Self {
        let i2c_conf = I2cConfig::new().baudrate(400.kHz().into()).timeout(Duration::from_millis(10).into());
        let i2c_driv = I2cDriver::new(
            peripheral.i2c0,
            peripheral.pins.gpio21,
            peripheral.pins.gpio22,
            &i2c_conf,
        );
        let parsed_driv = match i2c_driv {
            Ok(parsed) => parsed,
            Err(_) => panic!("I2C could not be openened in Hardware Task"),
        };

        // Custom wrapper for I2C bus to use custom timeout as that is hardcoded in the generic Driver
        // Shorter timeout required to ignore corrupted I2C packets and continue operating without delay
        let fast_bus: FastI2c<'_> = FastI2c { driver: parsed_driv, timeout_ticks: 10 };

        // Required to share a bus between multiple devices
        let i2c_bus = Box::leak(Box::new(RefCell::new(fast_bus)));

        let uart = peripheral.uart1;
        let tx_pin = peripheral.pins.gpio5;
        let rx_pin = peripheral.pins.gpio4;

        let usb_reader = USBReader::new(uart, tx_pin, rx_pin, controller_input_send);

        let servo_list: [Servo; NUMBER_SERVOS_PER_LEG * NUMBER_OF_LEGS] = {
            let mut vect = Vec::with_capacity(NUMBER_SERVOS_PER_LEG * NUMBER_OF_LEGS);
            for i in 0..NUMBER_OF_LEGS {
                vect.push(Servo::new(Self::to_channel(0,i ), i, ArmTypes::Coxa));
                vect.push(Servo::new(Self::to_channel(1, i), i, ArmTypes::Femur));
                vect.push(Servo::new(Self::to_channel(2, i), i, ArmTypes::Tibia));
            }
            vect.try_into().unwrap_or_else(|_| panic!("Vector length did not match array size in Hardware Task"))
        };

        let servo_driver = ServoDriver::new(
            servo_list, 
            Self::create_servo(RefCellDevice::new(i2c_bus), RIGHT_PCA_ADDRESS),
            Self::create_servo(RefCellDevice::new(i2c_bus), LEFT_PCA_ADDRESS)
        );

        // Deprecated version required for stable operation with the current Ws2812Esp32Rmt crate
        #[allow(deprecated)]
        let rmt_channel = peripheral.rmt.channel0;

        let led_strip_pin = peripheral.pins.gpio19;
        let mut led_strip = Ws2812Esp32Rmt::new(rmt_channel, led_strip_pin).unwrap();

        let red = RGB8::new(DEFAULT_LED_COLOR[0], DEFAULT_LED_COLOR[1], DEFAULT_LED_COLOR[2]);
        let pixels = repeat(red).take(NUMBER_OF_LEDS);
        led_strip.write(pixels).unwrap();

        let led_ctr = LedController::new(led_strip, led_cmd_receive);

        let leg_switches_input: [PinDriver<Input>; NUMBER_OF_LEGS] = [
            PinDriver::input(peripheral.pins.gpio27, Pull::Up).unwrap(),
            PinDriver::input(peripheral.pins.gpio26, Pull::Up).unwrap(),
            PinDriver::input(peripheral.pins.gpio25, Pull::Up).unwrap(),
            PinDriver::input(peripheral.pins.gpio33, Pull::Up).unwrap(),
            PinDriver::input(peripheral.pins.gpio32, Pull::Up).unwrap(),
            PinDriver::input(peripheral.pins.gpio23, Pull::Up).unwrap()
        ];

        let leg_reader = LegReader::new(leg_reader_sender, leg_switches_input);

        // Reverse required to addapt the newer 1.0 embedded-hal traits from RefCellDevice to the older Mpu6050 crate
        let mut mpu = Mpu6050::new(Reverse::new(
            RefCellDevice::new(i2c_bus)
        ));

        // Mutable delay struct required for the initialization of the sensor crate
        let mut delay_temp = FreeRtos;
        mpu.init(&mut delay_temp).unwrap();

        let mpu_r = MPU6050Reader::new(telemetry_sender, mpu);

        Self { 
            leg_command_receive: command_receive,
            automatic_leveling_state_receive: change_leveling_rcv,
            current_legs_command: LegHardwareCommand::default(),
            previous_legs_command: LegHardwareCommand::default(),
            servo_driver: servo_driver,
            controller_input_rcv: usb_reader,
            led_controller: led_ctr,
            leg_reader: leg_reader,
            mpu_reader: mpu_r,
            current_auto_leveling_state: false,
        }
    }
    fn to_channel(servo_part: usize, leg_id: usize) -> Channel{
        // Channel numbers must be continues along one side of the robot => leg 0, 2, 4 have continous channels for their servos
        let mod_leg_id = if leg_id % 2 != 0 {leg_id-1} else {leg_id};
        let channel_id = mod_leg_id/2 * NUMBER_SERVOS_PER_LEG + servo_part;
        channel_id.try_into().unwrap_or_else(|_| panic!("Conversion from Servo Id to Channel failed in Hardware Task"))
    }
    fn create_servo<'a, 'b>(i2c_driver: RefCellDevice<'a, FastI2c<'b>>, address: u8) -> Pca9685<RefCellDevice<'a, FastI2c<'b>>>{
        let mut pwm = Pca9685::new(i2c_driver, address).unwrap_or_else(|_| panic!("Create for PCA driver failed in Hardware Task"));
        pwm.set_prescale(PRESCALE_50HZ).unwrap_or_else(|_| panic!("Set Prescale for PCA driver failed in Hardware Task with address: {address}"));
        pwm.enable().unwrap_or_else(|_| panic!("Enable for PCA driver failed in Hardware Task"));
        pwm
    }
}
impl TaskStep for HardwareTask<'_> {
    fn step(&mut self) {
        while let Ok(command) = self.leg_command_receive.try_recv() {   
            self.current_legs_command = command;
        }

        // Only write to servos if sth changed in the state => Additionally prevents default start state from being written
        if self.current_legs_command != self.previous_legs_command {
            self.previous_legs_command = self.current_legs_command.clone();
            self.servo_driver.apply_joint_angles(&self.current_legs_command.angles);
        }

        while let Ok(state) = self.automatic_leveling_state_receive.try_recv() {    
            self.current_auto_leveling_state = state.automatic_state;
        }

        self.controller_input_rcv.step();
        self.led_controller.step();
        self.leg_reader.step();

        // Only read current telemetry data if it is required (for Auto-Leveling)
        // Prevents instability during other modes (reading corrupted I2C packets causes instability)
        if self.current_auto_leveling_state {
            self.mpu_reader.step();
        }
        
    }
}

// Fix for system-hardcoded 1s timeout for I2C reading:
// Use a custom wrapper that implements all relevant I2C Traits and applies the custom timeout
use embedded_hal::i2c::{I2c, ErrorType, Operation};

pub struct FastI2c<'d> {
    pub driver: I2cDriver<'d>,
    pub timeout_ticks: u32,
}
// Error type declared seperatly due to older hal version
impl<'d> ErrorType for FastI2c<'d> {
    type Error = I2cError;
}

impl<'d> I2c for FastI2c<'d> {
    
    // Standard operations are overriden to inject custom timeout
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.driver
            .read(address, read, self.timeout_ticks)
            .map_err(Into::into)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.driver
            .write(address, write, self.timeout_ticks)
            .map_err(Into::into)
    }

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.driver
            .write_read(address, write, read, self.timeout_ticks)
            .map_err(Into::into)
    }

    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        embedded_hal::i2c::I2c::transaction(&mut self.driver, address, operations)
    }
}