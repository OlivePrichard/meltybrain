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
    Testing(u8, f32),
}

impl Log {
    #[cfg(not(target_os = "none"))]
    pub fn to_string(&self) -> String {
        match self {
            Log::Initialized => "Initializing".to_string(),
            Log::Initialized => "Initialized".to_string(),
            Log::WifiStarted => "Wifi started".to_string(),
            Log::WifiError => "Wifi error".to_string(),
            Log::WifiReceivedPacket { address, port } => format!(
                "Wifi received packet from {}.{}.{}.{}:{}",
                address[0], address[1], address[2], address[3], port
            ),
            Log::LogMessageBufferFull => "Log message buffer full".to_string(),
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
            let Some(boundary) = self.buffer.iter().position(|&b| b == 0).map(|i| i + 1) else {
                self.index = self.buffer.len();
                return None;
            };
            let data = &mut self.buffer[self.index..boundary];
            self.index = boundary;
            if let Some(log_message) = Log::from_bytes(data) {
                return Some(log_message);
            } else {
                #[cfg(not(target_os = "none"))]
                println!("Got nonsense log message: {:02X?}", data);
            }
        }
    }
}
