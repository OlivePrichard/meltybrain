use crate::shared_code::log_messages::{Log, LogWithTime};
use data_structures::CobsQueue;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, once_lock::OnceLock};
use embassy_time::Instant;
use static_cell::StaticCell;
use postcard::to_slice_cobs;

macro_rules! log {
    ($message:ident) => {
        crate::logging::log_message(crate::shared_code::log_messages::Log::$message).await
    };
    ($message:ident { $( $field:ident: $val:expr ),* } ) => {
        crate::logging::log_message(crate::shared_code::log_messages::Log::$message { $( $field: $val ),* }).await
    };
    ($message:ident ( $( $val:expr ),* ) ) => {
        crate::logging::log_message(crate::shared_code::log_messages::Log::$message ( $( $val ),* )).await
    };
}
pub(crate) use log;

fn get_telemetry() -> &'static Mutex<CriticalSectionRawMutex, Telemetry> {
    const PAST_LOGS_BUFFER_LENGTH: usize = 1024 * 8;
    const CURRENT_LOGS_BUFFER_LENGTH: usize = 1024 * 2;
    const ENCODING_BUFFER_LENGTH: usize = size_of::<LogWithTime>() * 2 * 255 / 254 + 2;

    static TELEMETRY_LOCK: OnceLock<Mutex<CriticalSectionRawMutex, Telemetry>> = OnceLock::new();

    TELEMETRY_LOCK.get_or_init(|| {
        static PAST_LOGS_BUFFER_CELL: StaticCell<[u8; PAST_LOGS_BUFFER_LENGTH]> = StaticCell::new();
        static CURRENT_LOGS_BUFFER_CELL: StaticCell<[u8; CURRENT_LOGS_BUFFER_LENGTH]> =
            StaticCell::new();
        static ENCODING_BUFFER_CELL: StaticCell<[u8; ENCODING_BUFFER_LENGTH]> = StaticCell::new();

        let past_logs_buffer = PAST_LOGS_BUFFER_CELL.init_with(|| [0; PAST_LOGS_BUFFER_LENGTH]);
        let current_logs_buffer =
            CURRENT_LOGS_BUFFER_CELL.init_with(|| [0; CURRENT_LOGS_BUFFER_LENGTH]);
        let encoding_buffer = ENCODING_BUFFER_CELL.init_with(|| [0; ENCODING_BUFFER_LENGTH]);

        Mutex::new(Telemetry::new(
            past_logs_buffer,
            current_logs_buffer,
            encoding_buffer,
        ))
    })
}

pub async fn log_message(message: Log) {
    let mut telemetry = get_telemetry().lock().await;
    let time = Instant::now() - telemetry.creation_time;
    telemetry.log(LogWithTime {
        time: time.into(),
        log: message,
    });
}

// returns Err(true) if the id is too old, Err(false) if there was a different issue, and Ok(size) if the log was successfully copied
pub async fn get_log(id: u32, buffer: &mut [u8]) -> Result<usize, bool> {
    let telemetry = get_telemetry().lock().await;
    telemetry.get_log(id, buffer)
}

pub async fn get_current_log(buffer: &mut [u8]) -> Result<usize, ()> {
    let mut telemetry = get_telemetry().lock().await;
    telemetry.get_current_log(buffer)
}

pub async fn get_current_log_length() -> usize {
    let telemetry = get_telemetry().lock().await;
    telemetry.current_logs.used_space() + 24 // extra space for possible message about buffer full
}

struct Telemetry {
    past_logs: CobsQueue<'static, (u8, u8)>,
    current_logs: CobsQueue<'static, (u8,)>,
    encoding_buffer: &'static mut [u8],
    id: u32,
    current_logs_full: bool,
    creation_time: Instant,
}

impl Telemetry {
    fn new(
        past_logs_buffer: &'static mut [u8],
        current_logs_buffer: &'static mut [u8],
        encoding_buffer: &'static mut [u8],
    ) -> Self {
        Self {
            past_logs: CobsQueue::new(past_logs_buffer),
            current_logs: CobsQueue::new(current_logs_buffer),
            encoding_buffer,
            id: 0,
            current_logs_full: false,
            creation_time: Instant::now(),
        }
    }

    fn log(&mut self, message: LogWithTime) {
        let Ok(encoded) = to_slice_cobs(&message, &mut self.encoding_buffer) else { return; };
        if self.current_logs.push(encoded) == Ok(true) {
            self.current_logs_full = true;
        }
    }

    fn get_log(&self, id: u32, buffer: &mut [u8]) -> Result<usize, bool> {
        if id >= self.id {
            return Err(false);
        }
        let oldest_id = self.id - self.past_logs.count() as u32;
        if id < oldest_id as u32 {
            return Err(true);
        }
        let index = id - oldest_id;
        self.past_logs.get(index as usize, buffer).map_err(|_| false)
    }

    fn get_current_log(&mut self, buffer: &mut [u8]) -> Result<usize, ()> {
        if self.current_logs_full {
            let time = Instant::now() - self.creation_time;
            self.log(LogWithTime {
                time: time.into(),
                log: Log::LogMessageBufferFull,
            });
        }

        if self.current_logs.used_space() > buffer.len() {
            return Err(());
        }
        for (i, byte) in self.current_logs.byte_iter().enumerate() {
            buffer[i] = byte;
        }
        let size = self.current_logs.used_space();

        _ = self.past_logs.push(&buffer[..size]);
        self.id += 1;
        self.current_logs.clear();

        Ok(size)
    }
}

mod data_structures {
    use core::{iter, marker::PhantomData};
    use itertools::{traits::HomogeneousTuple, Itertools};

    // D is a u8 tuple with length equal to the number of zeros acting as a delimiter
    pub struct CobsQueue<'a, D> {
        buffer: &'a mut [u8],
        start: usize,
        end: usize,
        count: usize,
        _type: PhantomData<D>,
    }

    impl<'a, D> CobsQueue<'a, D>
    where
        D: HomogeneousTuple<Item = u8> + Default + Copy + Eq,
    {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            Self {
                buffer,
                start: 0,
                end: 0,
                count: 0,
                _type: PhantomData,
            }
        }

        fn free_space(&self) -> usize {
            if self.end < self.start || self.count == 0 {
                self.start - self.end
            } else {
                self.buffer.len() + self.start - self.end
            }
        }

        pub fn used_space(&self) -> usize {
            self.buffer.len() - self.free_space()
        }

        pub fn push(&mut self, data: &[u8]) -> Result<bool, ()> {
            let zeros_to_add = D::num_items() - data.iter().rev().take_while(|&&b| b == 0).count();
            let size = data.len() + zeros_to_add;
            let mut popped = false;
            while self.free_space() < size {
                if self.count == 0 {
                    return Err(());
                }
                self.pop(None);
                popped = true;
            }
            for &byte in data.iter().chain(iter::repeat(&0).take(zeros_to_add)) {
                self.buffer[self.end] = byte;
                self.end += 1;
                if self.end == self.buffer.len() {
                    self.end = 0;
                }
            }
            self.count += 1;
            Ok(popped)
        }

        pub fn pop(&mut self, buffer: Option<&mut [u8]>) {
            if self.count == 0 {
                return;
            }
            let index = QueueIterator::new(self.buffer, self.start, self.used_space())
                .tuple_windows::<D>()
                .position(|d| d == D::default())
                .expect("Safe because we always keep a delimeter at the end of the buffer");
            let new_start = self.start + index + D::num_items();
            if let Some(buffer) = buffer {
                let iter = QueueIterator::new(self.buffer, new_start, index + D::num_items());
                for (i, item) in iter.take(buffer.len()).enumerate() {
                    buffer[i] = item;
                }
            }
            self.start = new_start;
            self.count -= 1;
        }

        pub fn count(&self) -> usize {
            self.count
        }

        pub fn get(&self, index: usize, buffer: &mut [u8]) -> Result<usize, ()> {
            if index >= self.count {
                return Err(());
            }
            let mut count = 0;
            let position = QueueIterator::new(self.buffer, self.start, self.used_space())
                .tuple_windows::<D>()
                .take_while(|&t| {
                    if t == D::default() {
                        count += 1;
                    }
                    count != index
                })
                .count();
            let mut start = self.start + position + D::num_items();
            if start >= self.buffer.len() {
                start -= self.buffer.len();
            }
            let size = QueueIterator::new(
                self.buffer,
                start,
                self.used_space() - position - D::num_items(),
            )
            .tuple_windows::<D>()
            .take_while(|&t| t != D::default())
            .count();
            for (i, item) in
                QueueIterator::new(self.buffer, start, size + D::num_items() - 1).enumerate()
            {
                if i == buffer.len() {
                    return Err(());
                }
                buffer[i] = item;
            }
            Ok(size + D::num_items() - 1)
        }

        pub fn byte_iter(&'a self) -> QueueIterator<'a> {
            QueueIterator::new(self.buffer, self.start, self.used_space())
        }

        pub fn clear(&mut self) {
            self.start = 0;
            self.end = 0;
            self.count = 0;
        }
    }

    pub struct QueueIterator<'a> {
        buffer: &'a [u8],
        index: usize,
        remaining: usize,
    }

    impl<'a> QueueIterator<'a> {
        pub fn new(buffer: &'a [u8], start: usize, remaining: usize) -> Self {
            Self {
                buffer,
                index: start,
                remaining,
            }
        }
    }

    impl<'a> Iterator for QueueIterator<'a> {
        type Item = u8;

        fn next(&mut self) -> Option<Self::Item> {
            if self.remaining == 0 {
                return None;
            }
            let item = self.buffer[self.index];
            self.index += 1;
            if self.index == self.buffer.len() {
                self.index = 0;
            }
            self.remaining -= 1;
            Some(item)
        }

        fn size_hint(&self) -> (usize, Option<usize>) {
            (self.remaining, Some(self.remaining))
        }
    }
}
