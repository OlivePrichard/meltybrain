#![no_std]
#![no_main]

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
};
use core::cell::RefCell;
use core::num::Wrapping;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp_alloc as _;
use esp_backtrace as _;
use esp_backtrace as _;
use esp_hal::{
    delay::{self, Delay}, gpio::{Input, Io, Pull}, peripherals::UART0, prelude::*, rng::Rng, time, timer::timg::TimerGroup, uart::{
        config::{AtCmdConfig, Config},
        Uart, UartRx, UartTx,
    }, Async
};
use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, init, EspWifiInitFor};
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

async fn serial_example(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let (tx_pin, rx_pin) = (io.pins.gpio21, io.pins.gpio20);

    let config = Config::default()
        .rx_fifo_full_threshold(READ_BUF_SIZE as u16)
        .baudrate(230_400);

    let mut uart0 = Uart::new_async_with_config(peripherals.UART0, config, rx_pin, tx_pin).unwrap();
    uart0.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));

    let (rx, tx) = uart0.split();

    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    spawner.spawn(reader(rx, &signal)).ok();
    spawner.spawn(writer(tx, &signal)).ok();
}

async fn bluetooth(spawner: Spawner) {
    // esp_println::logger::init_logger_from_env();
    // let peripherals = esp_hal::init({
    //     let mut config = esp_hal::Config::default();
    //     config.cpu_clock = CpuClock::max();
    //     config
    // });

    // esp_alloc::heap_allocator!(72 * 1024);

    // let timg0 = TimerGroup::new(peripherals.TIMG0);

    // let init = init(
    //     EspWifiInitFor::Ble,
    //     timg0.timer0,
    //     Rng::new(peripherals.RNG),
    //     peripherals.RADIO_CLK,
    // )
    // .unwrap();

    // let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    // let button = Input::new(io.pins.gpio9, Pull::Down);

    // use esp_hal::timer::systimer::{SystemTimer, Target};
    // let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    // esp_hal_embassy::init(systimer.alarm0);

    // let mut bluetooth = peripherals.BT;

    // let connector = BleConnector::new(&init, &mut bluetooth);

    // let now = || time::now().duration_since_epoch().to_millis();
    // let mut ble = Ble::new(connector, now);
    // println!("Connector created");

    // let pin_ref = RefCell::new(button);
    // let pin_ref = &pin_ref;

    // loop {
    //     println!("{:?}", ble.init().await);
    //     println!("{:?}", ble.cmd_set_le_advertising_parameters().await);
    //     println!(
    //         "{:?}",
    //         ble.cmd_set_le_advertising_data(
    //             create_advertising_data(&[
    //                 AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
    //                 AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
    //                 AdStructure::CompleteLocalName(esp_hal::chip!()),
    //             ])
    //             .unwrap()
    //         )
    //         .await
    //     );
    //     println!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

    //     println!("started advertising");

    //     let mut rf = |_offset: usize, data: &mut [u8]| {
    //         data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
    //         17
    //     };
    //     let mut wf = |offset: usize, data: &[u8]| {
    //         println!("RECEIVED: {} {:?}", offset, data);
    //     };

    //     let mut wf2 = |offset: usize, data: &[u8]| {
    //         println!("RECEIVED: {} {:?}", offset, data);
    //     };

    //     let mut rf3 = |_offset: usize, data: &mut [u8]| {
    //         data[..5].copy_from_slice(&b"Hola!"[..]);
    //         5
    //     };
    //     let mut wf3 = |offset: usize, data: &[u8]| {
    //         println!("RECEIVED: Offset {}, data {:?}", offset, data);
    //     };

    //     gatt!([service {
    //         uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
    //         characteristics: [
    //             characteristic {
    //                 uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
    //                 read: rf,
    //                 write: wf,
    //             },
    //             characteristic {
    //                 uuid: "957312e0-2354-11eb-9f10-fbc30a62cf38",
    //                 write: wf2,
    //             },
    //             characteristic {
    //                 name: "my_characteristic",
    //                 uuid: "987312e0-2354-11eb-9f10-fbc30a62cf38",
    //                 notify: true,
    //                 read: rf3,
    //                 write: wf3,
    //             },
    //         ],
    //     },]);

    //     let mut rng = bleps::no_rng::NoRng;
    //     let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

    //     let counter = RefCell::new(0u8);
    //     let counter = &counter;

    //     let mut notifier = || {
    //         // TODO how to check if notifications are enabled for the characteristic?
    //         // maybe pass something into the closure which just can query the characteristic
    //         // value probably passing in the attribute server won't work?

    //         async {
    //             pin_ref.borrow_mut().wait_for_rising_edge().await;
    //             let mut data = [0u8; 13];
    //             data.copy_from_slice(b"Notification0");
    //             {
    //                 let mut counter = counter.borrow_mut();
    //                 data[data.len() - 1] += *counter;
    //                 *counter = (*counter + 1) % 10;
    //             }
    //             NotificationData::new(my_characteristic_handle, &data)
    //         }
    //     };

    //     srv.run(&mut notifier).await.unwrap();
    // }
    println!("Running");
    let delay = Delay::new();
    let mut counter = 0;
    loop {
        delay.delay_millis(1000);
        println!("{counter}");
        counter += 1
    }
}



#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    serial_example(spawner).await;
}
