use std::sync::mpsc::{Sender};
use esp_idf_svc::hal::delay::NON_BLOCK;
use esp_idf_svc::hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_svc::hal::uart::{config::Config, Uart, UartDriver};

use crate::messages::{ControllerInput};
use crate::task::TaskStep;

const DEADZONE_RAW: i16 = 80;
const MAX_AXIS_DEFLECTION: f32 = 512.0;
const ANALOG_SHOULDERS_MAX: f32 = 255.0;
const READ_BUFFER_SIZE: usize = 64;
const PAYLOAD_SIZE: usize = 12;

enum ParserState {
    WaitHeader1,
    WaitHeader2,
    ReadingPayload,
    WaitChecksum,
}

struct FrameParser {
    state: ParserState,
    payload: [u8; PAYLOAD_SIZE],
    payload_idx: usize,
}

impl FrameParser {
    fn new() -> Self {
        Self {
            state: ParserState::WaitHeader1,
            payload: [0u8; PAYLOAD_SIZE],
            payload_idx: 0,
        }
    }

    fn push_byte(&mut self, byte: u8) -> Option<[u8; PAYLOAD_SIZE]> {
        match self.state {
            ParserState::WaitHeader1 => {
                if byte == 0xAA {
                    self.state = ParserState::WaitHeader2;
                }
            }
            ParserState::WaitHeader2 => {
                if byte == 0x55 {
                    self.state = ParserState::ReadingPayload;
                    self.payload_idx = 0;
                } else {
                    self.state = ParserState::WaitHeader1;
                }
            }
            ParserState::ReadingPayload => {
                self.payload[self.payload_idx] = byte;
                self.payload_idx += 1;
                if self.payload_idx == self.payload.len() {
                    self.state = ParserState::WaitChecksum;
                }
            }
            ParserState::WaitChecksum => {
                self.state = ParserState::WaitHeader1;
                let calculated_checksum = self.payload.iter().fold(0u8, |acc, &b| acc ^ b);
                if byte == calculated_checksum {
                    return Some(self.payload);
                }
                log::warn!("Dropped corrupted UART packet: Checksum mismatch");
            }
        }
        None
    }
}

pub struct USBReader {
    pub controller_status_sender: Sender<ControllerInput>,
    uart_driver: UartDriver<'static>,
    parser: FrameParser,
    read_buf: [u8; READ_BUFFER_SIZE],
}

impl USBReader {
    pub fn new<U: Uart + 'static>(
        uart: U,
        tx_pin: impl OutputPin + 'static,
        rx_pin: impl InputPin + 'static,
        status_sender: Sender<ControllerInput>,
    ) -> Self {
        let config = Config::default().baudrate(115_200.into());
        let driver = UartDriver::new(
            uart,
            tx_pin,
            rx_pin,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &config,
        ).expect("Failed to initialize UART driver for controller receiver");

        Self {
            controller_status_sender: status_sender,
            uart_driver: driver,
            parser: FrameParser::new(),
            read_buf: [0u8; 64],
        }
    }

    
}

impl TaskStep for USBReader {
    fn step(&mut self) {
        // Read all available bytes from UART hardware buffer without blocking
        if let Ok(bytes_read) = self.uart_driver.read(&mut self.read_buf, NON_BLOCK) {
            if bytes_read > 0 {
                for i in 0..bytes_read {
                    let byte = self.read_buf[i];
                    if let Some(payload) = self.parser.push_byte(byte) {
                        let parse_axis = |low: u8, high: u8| -> f32 {
                                let raw = i16::from_le_bytes([low, high]);
                                if raw.abs() > DEADZONE_RAW { 
                                    (raw as f32) / MAX_AXIS_DEFLECTION 
                                } else { 
                                    0.0 
                                }
                            };

                        let rcv_input = ControllerInput {
                            stick_lx: parse_axis(payload[0], payload[1]),
                            stick_ly: -parse_axis(payload[2], payload[3]),
                            stick_rx: parse_axis(payload[4], payload[5]),
                            stick_ry: -parse_axis(payload[6], payload[7]),
                            trigger_l: (payload[8] as f32) / ANALOG_SHOULDERS_MAX,
                            trigger_r: (payload[9] as f32) / ANALOG_SHOULDERS_MAX,
                            buttons: u16::from_le_bytes([payload[10], payload[11]]),
                        };
                        let _ = self.controller_status_sender.send(rcv_input);
                    }
                }
            }
        }
    }
}