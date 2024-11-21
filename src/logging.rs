use crate::shared_code::log_messages::Log;

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, once_lock::OnceLock,
};
use embassy_time::Instant;
use static_cell::StaticCell;

macro_rules! log {
    ($message:ident) => {{
        // use esp_println::println;
        let message = crate::shared_code::log_messages::Log::$message;
        crate::logging::log_message(message).await;
        // println!("{:?}", message);
    }};
    ($message:ident { $( $field:ident: $val:expr ),* } ) => {{
        // use esp_println::println;
        let message = crate::shared_code::log_messages::Log::$message { $( $field: $val ),* };
        crate::logging::log_message(message).await;
        // println!("{:?}", message);
    }};
    ($message:ident ( $( $val:expr ),* ) ) => {{
        // use esp_println::println;
        let message = crate::shared_code::log_messages::Log::$message ( $( $val ),* );
        crate::logging::log_message(message).await;
        // println!("{:?}", message);
    }};
}
pub(crate) use log;

pub fn get_telemetry() -> &'static Mutex<CriticalSectionRawMutex, Telemetry> {
    const CURRENT_LOGS_BUFFER_LENGTH: usize = 4096;

    static TELEMETRY_LOCK: OnceLock<Mutex<CriticalSectionRawMutex, Telemetry>> = OnceLock::new();

    TELEMETRY_LOCK.get_or_init(|| {
        static CURRENT_LOGS_BUFFER_CELL: StaticCell<[u8; CURRENT_LOGS_BUFFER_LENGTH]> =
            StaticCell::new();
            
        let current_logs_buffer =
            CURRENT_LOGS_BUFFER_CELL.init_with(|| [0; CURRENT_LOGS_BUFFER_LENGTH]);

        Mutex::new(Telemetry::new(
            current_logs_buffer
        ))
    })
}

pub async fn log_message(message: Log) {
    return;
    let mut telemetry = get_telemetry().lock().await;
    let time = Instant::now() - telemetry.creation_time;
    let time = time.as_micros() as u32;
    let index = telemetry.index;
    if let Some(size) = message.to_bytes(time, &mut telemetry.logs[index..]) {
        telemetry.index += size;
    }
}

pub struct Telemetry {
    pub logs: &'static mut [u8],
    pub index: usize,
    creation_time: Instant,
}

impl Telemetry {
    pub fn new(
        logs: &'static mut [u8],
    ) -> Self {
        Self {
            logs,
            index: 4, // leave room for the log packet id
            creation_time: Instant::now(),
        }
    }
}
