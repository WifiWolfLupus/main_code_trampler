use std::sync::mpsc::{Sender};
use crate::{config::NUMBER_OF_LEGS, messages::{LegStates}, task::TaskStep};
use esp_idf_svc::hal::gpio::{PinDriver, Input};

pub struct LegReader<'a> {
    leg_state_sender: Sender<LegStates>,
    input_pins: [PinDriver<'a, Input>; NUMBER_OF_LEGS]
}
impl LegReader<'_> {
    pub fn new(st_sender: Sender<LegStates>, input_p: [PinDriver<'static, Input>; NUMBER_OF_LEGS]) -> Self {
        Self {
            leg_state_sender: st_sender,
            input_pins: input_p,
        }
    }
}
impl TaskStep for LegReader<'_> {
    fn step(&mut self){
        let mut leg_state: u8 = 0;
        for i in 0..NUMBER_OF_LEGS {
            if !self.input_pins[i].is_high() { leg_state |= 1 << i } // Negate to depict pressed / not pressed states with Pull-Up
        }
        let leg_st_send = LegStates { leg_states: leg_state };
        _ = self.leg_state_sender.send(leg_st_send);
    }
} 