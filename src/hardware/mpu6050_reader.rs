use core::result::Result::{Ok};
use std::sync::mpsc::{Sender};
use crate::{config::ALPHA_FILTER, hardware::low_pass_filter::LowPassFilter, messages::TelemetryData, task::TaskStep, types::MpuType};
use mpu6050::Mpu6050;

pub struct MPU6050Reader<'a> {
    telemetry_data_sender: Sender<TelemetryData>,
    mpu: Mpu6050<MpuType<'a>>,
    low_pass_filter_x: LowPassFilter,
    low_pass_filter_y: LowPassFilter,
}
impl<'a> MPU6050Reader<'a> {
    pub fn new(data_sender: Sender<TelemetryData>, mpu_mod: Mpu6050<MpuType<'a>>) -> Self {
        Self {
            telemetry_data_sender: data_sender,
            mpu: mpu_mod,
            low_pass_filter_x: LowPassFilter::new(ALPHA_FILTER, 0.0),
            low_pass_filter_y: LowPassFilter::new(ALPHA_FILTER, 0.0)
        }
    }
}
impl TaskStep for MPU6050Reader<'_>
{
    fn step(&mut self){
        if let Ok(angles) = self.mpu.get_acc_angles() {
            let telemetry = TelemetryData {
                angle_x_rad: self.low_pass_filter_x.update(angles.x),
                angle_y_rad: self.low_pass_filter_y.update(angles.y),
            };
            _ = self.telemetry_data_sender.send(telemetry);
        }
    }
} 