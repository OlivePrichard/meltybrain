#![allow(unused)]

use core::time::Duration;
use postcard::{from_bytes, to_slice};
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

    pub fn to_bytes(&self, time_us: u32, buffer: &mut [u8]) -> Option<usize> {
        if buffer.len() < 6 {
            return None;
        }
        buffer[1..5].copy_from_slice(&time_us.to_le_bytes());
        match to_slice(self, &mut buffer[6..]) {
            Ok(slice) => {
                let size = slice.len();
                buffer[0] = size as u8;
                Some(size + 5)
            }
            Err(_) => None,
        }
    }

    pub fn from_bytes(buffer: &[u8]) -> Option<LogWithTime> {
        if buffer.len() < 6 {
            return None;
        }
        let micros = u32::from_le_bytes([buffer[1], buffer[2], buffer[3], buffer[4]]);
        let time = Duration::from_micros(micros as u64);
        from_bytes(&buffer[5..])
            .ok()
            .map(|log| LogWithTime { time, log })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LogWithTime {
    pub time: Duration,
    pub log: Log,
}
