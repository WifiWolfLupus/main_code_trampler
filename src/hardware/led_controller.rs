pub use ws2812_esp32_rmt_driver::lib_smart_leds::{Ws2812Esp32Rmt};
use smart_leds::{SmartLedsWrite};
use std::sync::mpsc::{Receiver};
use crate::types::StatusColors;

use crate::{task::TaskStep};
pub struct LedController<'a> {
    led_strip_module: Ws2812Esp32Rmt<'a>,
    led_command_receiver: Receiver<StatusColors>
}

impl LedController<'_> {
    pub fn new(led_strp_mdl: Ws2812Esp32Rmt<'static>, rcv_led: Receiver<StatusColors>) -> Self {
        Self {
            led_strip_module: led_strp_mdl,
            led_command_receiver: rcv_led
        }
    }
}
impl TaskStep for LedController<'_> {
    fn step(&mut self){
        while let Ok(rcv) = self.led_command_receiver.try_recv() {
            self.led_strip_module.write(rcv).unwrap();
        }
    }
} 