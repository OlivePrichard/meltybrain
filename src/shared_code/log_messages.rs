#![allow(unused)]

use core::time::Duration;
use postcard::{from_bytes_cobs, to_slice_cobs};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Log {
    Initializing,
    Initialized,
    WifiStarted,
    WifiError,
    WifiReceivedPacket { address: [u8; 4], port: u16 },
    LogMessageBufferFull,
    ReceivedPacketTooLarge,
    ReceivedLogData(u32),
    ReceivedForgotLogData(u32),
    WatchdogTimeout,
    ConnectionRestored,
    InitializingMotors,
    MotorsInitialized,
    MotorPowers { left: u8, right: u8 },
}

impl Log {
    #[cfg(not(target_os = "none"))]
    pub fn to_string(&self) -> String {
        match self {
            Log::Initializing => "Initializing".to_string(),
            Log::Initialized => "Initialized".to_string(),
            Log::WifiStarted => "Wifi started".to_string(),
            Log::WifiError => "Wifi error".to_string(),
            Log::WifiReceivedPacket { address, port } => format!(
                "Wifi received packet from {}.{}.{}.{}:{}",
                address[0], address[1], address[2], address[3], port
            ),
            Log::LogMessageBufferFull => "Log message buffer full".to_string(),
            Log::ReceivedPacketTooLarge => "Received packet too large".to_string(),
            Log::ReceivedLogData(id) => format!("Received log data {}", id),
            Log::ReceivedForgotLogData(id) => format!("Received forgot log data {}", id),
            Log::WatchdogTimeout => "Watchdog timeout".to_string(),
            Log::ConnectionRestored => "Connection restored".to_string(),
            Log::InitializingMotors => "Initializing motors".to_string(),
            Log::MotorsInitialized => "Motors initialized".to_string(),
            Log::MotorPowers { left, right } => {
                format!("Motor powers: left: {}, right: {}", left, right)
            }
        }
    }

    pub fn to_bytes(&self, time: Duration, buffer: &mut [u8]) -> Option<usize> {
        let res = to_slice_cobs(&LogWithTime { time, log: *self }, buffer);
        res.ok().map(|slice| slice.len())
    }

    pub fn from_bytes(buffer: &mut [u8]) -> Option<LogWithTime> {
        from_bytes_cobs(buffer).ok()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LogWithTime {
    pub time: Duration,
    pub log: Log,
}

pub struct LogIterator<'a> {
    buffer: &'a mut [u8],
    index: usize,
}

impl<'a> LogIterator<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        // println!("New Log Iterator: {:02X?}", buffer);
        Self { buffer, index: 0 }
    }
}

impl<'a> Iterator for LogIterator<'a> {
    type Item = LogWithTime;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.buffer.len() == self.index {
                return None;
            }
            let Some(boundary) = self
                .buffer
                .iter()
                .skip(self.index)
                .position(|&b| b == 0)
                .map(|i| i + 1 + self.index)
            else {
                self.index = self.buffer.len();
                return None;
            };
            let data = &mut self.buffer[self.index..boundary];
            self.index = boundary;
            if let Some(log_message) = Log::from_bytes(data) {
                // println!("Got good log message {:?} from bytes: {:02X?}", log_message, data);
                return Some(log_message);
            } else {
                #[cfg(not(target_os = "none"))]
                println!("Got nonsense log message: {:02X?}", data);
            }
        }
    }
}
