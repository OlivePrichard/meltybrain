#![no_std]
#![no_main]

use core::num::Wrapping;

use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_backtrace as _;
use esp_hal::{
    gpio::Io, i2c::{self, I2c}, peripherals::UART0, timer::timg::TimerGroup, uart::{
        config::{AtCmdConfig, Config},
        Uart, UartRx, UartTx,
    }, Async
};
use esp_println::println;
use static_cell::StaticCell;

const READ_BUF_SIZE: usize = 64;
const AT_CMD: u8 = 0x04;

#[embassy_executor::task]
async fn writer(
    mut tx: UartTx<'static, UART0, Async>,
    _signal: &'static Signal<NoopRawMutex, usize>,
) {
    // let frequency = 1000;
    // let low_byte = (frequency & 0xFF) as u8;
    // let high_byte = (frequency >> 8 & 0xFF) as u8;
    // let mut packet = [0x5A, 0x06, 0x03, low_byte, high_byte, 0x00];

    // let baud_rate = 230_400u32;
    // let bytes = baud_rate.to_le_bytes();
    // let mut packet = [0x5A, 0x08, 0x06, bytes[0], bytes[1], bytes[2], bytes[3], 0x00];

    // let mut packet = [0x5A, 0x05, 0x05, 0x06, 0x6A];

    // let mut check = Wrapping(0u8);
    // for byte in packet {
    //     check += byte;
    // }
    // packet[packet.len() - 1] = check.0;

    // embedded_io_async::Write::write(&mut tx, &packet)
    //     .await
    //     .unwrap();
    // embedded_io_async::Write::flush(&mut tx).await.unwrap();

    // embedded_io_async::Write::write(&mut tx, &[0x5A, 0x04, 0x11, 0x6F])
    //     .await
    //     .unwrap();
    // embedded_io_async::Write::flush(&mut tx).await.unwrap();

    loop {
        yield_now().await;
    }
}

fn check_packet(packet: &[u8]) -> bool {
    let mut check = Wrapping(0u8);
    for byte in &packet[..packet.len() - 1] {
        check += byte;
    }
    check.0 == packet[packet.len() - 1]
}

#[derive(PartialEq, Eq, Debug)]
enum PacketData {
    Measurement { value: i16 },
    Control { length: usize },
    Checksum { length: usize },
    Nonsense,
    Incomplete,
}

fn read_packet(packet: &[u8]) -> PacketData {
    if packet.len() < 4 {
        PacketData::Incomplete
    } else if packet[0] == 0x59 && packet[1] == 0x59 {
        if packet.len() < 9 {
            PacketData::Incomplete
        } else if check_packet(&packet[..9]) {
            PacketData::Measurement {
                value: i16::from_le_bytes([packet[2], packet[3]]),
            }
        } else {
            println!("Checksum failed: {packet:02X?}");
            PacketData::Checksum { length: 9 }
        }
    } else if packet[0] == 0x5A {
        let length = packet[1] as usize;
        if length < 4 {
            println!("Packet ill formatted: {packet:02X?}");
            PacketData::Nonsense
        } else if packet.len() < length {
            PacketData::Incomplete
        } else if check_packet(&packet[..length]) {
            println!("Control packet: {packet:02X?}");
            PacketData::Control { length }
        } else {
            println!("Checksum failed: {packet:02X?}");
            PacketData::Checksum { length }
        }
    } else {
        println!("Packet ill formatted: {packet:02X?}");
        PacketData::Nonsense
    }
}

struct PacketReader {
    previous: [u8; 9],
    length: usize,
}

impl PacketReader {
    fn new() -> Self {
        Self {
            previous: [0; 9],
            length: 0,
        }
    }

    fn parse_data(&mut self, mut buffer: &[u8], mut distances: &mut [i16]) -> usize {
        let mut count = 0;
        while self.length != 0 {
            if buffer.len() + self.length >= 9 {
                self.previous[self.length..].copy_from_slice(&buffer[..9 - self.length]);
            } else {
                self.previous[self.length..self.length + buffer.len()].copy_from_slice(buffer);
                self.length += buffer.len();
                return 0;
            }
            let result = read_packet(&self.previous);
            match result {
                PacketData::Measurement { value } => {
                    distances[0] = value;
                    distances = &mut distances[1..];
                    count += 1;
                    buffer = &buffer[9 - self.length..];
                    self.length = 0;
                }
                PacketData::Control { length } => {
                    buffer = &buffer[length - self.length..];
                    self.length = 0;
                }
                PacketData::Checksum { length } => {
                    buffer = &buffer[length - self.length..];
                    self.length = 0;
                }
                PacketData::Nonsense => {
                    for i in 0..8 {
                        self.previous[i] = self.previous[i + 1];
                    }
                    buffer = &buffer[9 - self.length..];
                    self.length = 8;
                }
                PacketData::Incomplete => {
                    self.length += buffer.len();
                    return 0;
                }
            }
        }

        while buffer.len() > 3 {
            let result = read_packet(buffer);
            match result {
                PacketData::Measurement { value } => {
                    distances[0] = value;
                    distances = &mut distances[1..];
                    count += 1;
                    buffer = &buffer[9..];
                }
                PacketData::Control { length } => {
                    buffer = &buffer[length..];
                }
                PacketData::Checksum { length } => {
                    buffer = &buffer[length..];
                }
                PacketData::Nonsense => {
                    buffer = &buffer[1..];
                }
                PacketData::Incomplete => {
                    break;
                }
            }
        }

        self.previous[..buffer.len()].copy_from_slice(buffer);
        self.length = buffer.len();
        count
    }
}

#[embassy_executor::task]
async fn reader(
    mut rx: UartRx<'static, UART0, Async>,
    _signal: &'static Signal<NoopRawMutex, usize>,
) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut buf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut decoder = PacketReader::new();
    let mut distances = [0i16; 64];
    let mut prev_time = esp_hal::time::now();
    let mut counter = 0;
    let mut loop_counter = 0;
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut buf).await;
        match r {
            Ok(len) => {
                loop_counter += 1;
                counter += decoder.parse_data(&buf[..len], &mut distances);
                if counter >= 1000 {
                    let time = esp_hal::time::now();
                    let difference = time - prev_time;
                    prev_time = time;
                    let hz = (1e6 * counter as f32) / difference.to_micros() as f32;
                    let hz = (hz * 10f32) as i32 as f32 / 10f32;
                    let loop_hz = (1e6 * loop_counter as f32) / difference.to_micros() as f32;
                    let loop_hz = (loop_hz * 10f32) as i32 as f32 / 10f32;
                    loop_counter = 0;
                    counter = 0;
                    println!(
                        "Last 1000 loops averaged {hz} Hz, {loop_hz} Hz     Distance: {}\n{:02X?}",
                        distances[0],
                        &buf[..len],
                    );
                }
            }
            Err(e) => esp_println::println!("RX Error: {:?}", e),
        }
    }
}

// #[esp_hal_embassy::main]
// async fn main(spawner: Spawner) {
    //! Embassy I2C
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This is an example of running the embassy executor with IC2. It uses an
//! LIS3DH to get accelerometer data.
//!
//! Following pins are used:
//! - SDA => GPIO4
//! - SCL => GPIO5

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy embassy-generic-timers

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    prelude::*,
    timer::timg::TimerGroup,
};


#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c0 = I2c::new(peripherals.I2C0, io.pins.gpio4, io.pins.gpio5, 400.kHz());


    loop {
        // esp_println::println!("X: {:+.5}  Y: {:+.5}  Z: {:+.5}", norm.x, norm.y, norm.z);

        Timer::after(Duration::from_millis(100)).await;
    }
}

