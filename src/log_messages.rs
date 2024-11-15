use serde::{Serialize, Deserialize};
use postcard::{from_bytes_cobs, to_slice_cobs};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Log {
    Initialized,
    WifiStarted,
    WifiError,
    WifiReceivedPacket{address: [u8; 4], port: u16},
}

impl Log {
    #[cfg(not(target_os = "none"))]
    pub fn to_string(&self) -> String {
        match self {
            Log::Initialized => "Initialized".to_string(),
            Log::WifiStarted => "Wifi started".to_string(),
            Log::WifiError => "Wifi error".to_string(),
            Log::WifiReceivedPacket{address, port} => format!("Wifi received packet from {}.{}.{}.{}:{}", address[0], address[1], address[2], address[3], port),
        }
    }

    pub fn to_bytes(&self, buffer: &mut [u8]) -> Option<usize> {
        let res = to_slice_cobs(self, buffer);
        res.ok().map(|slice| slice.len())
    }

    pub fn from_bytes(buffer: &mut [u8]) -> Option<Log> {
        from_bytes_cobs(buffer).ok()
    }
}