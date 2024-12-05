use crate::{
    shared_code::{
        controller::ControllerState,
        message_format::{Message, MessageIter},
    },
    watchdog::Watchdog,
};

use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    Stack,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use esp_wifi::wifi::{WifiApDevice, WifiDevice};

macro_rules! static_buffer {
    ($t:ty, [$val:expr; $size: expr]) => {{
        static STATIC_CELL: static_cell::StaticCell<[$t; $size]> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        STATIC_CELL.init_with(|| [$val; $size])
    }};
}

#[embassy_executor::task]
pub async fn handle_networking(
    stack: &'static Stack<WifiDevice<'static, WifiApDevice>>,
    controllers: &'static Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    watchdog: &'static Watchdog,
) -> ! {
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    const RX_BUFFER_SIZE: usize = 64; // Controller state messages should be 36 bytes
    const TX_BUFFER_SIZE: usize = 0;

    let rx_buffer_internal = static_buffer!(u8, [0; RX_BUFFER_SIZE]);
    let tx_buffer_internal = static_buffer!(u8, [0; TX_BUFFER_SIZE]);
    let rx_metadata = static_buffer!(PacketMetadata, [PacketMetadata::EMPTY; 1]);
    let tx_metadata = static_buffer!(PacketMetadata, [PacketMetadata::EMPTY; 0]);

    let rx_buffer = static_buffer!(u8, [0; RX_BUFFER_SIZE]);

    let mut previous_controller_id = 0;
    let mut socket = UdpSocket::new(
        stack,
        rx_metadata,
        rx_buffer_internal,
        tx_metadata,
        tx_buffer_internal,
    );
    socket.bind(55440).unwrap();

    let mut driver_station_address = None;

    loop {
        match  socket.recv_from(rx_buffer).await {
            Ok((size, addr)) => {
                if driver_station_address.is_none() {
                    watchdog.start().await;
                }
                driver_station_address = Some(addr);

                watchdog.feed().await;

                previous_controller_id =
                    receive_packet(&rx_buffer[..size], previous_controller_id, controllers).await;
            }
            Err(_) => () // shouldn't ever happen but if it does we just ignore it
        }
    }
}

async fn receive_packet(
    buffer: &[u8],
    previous_controller_id: u32,
    controllers: &Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
) -> u32 {
    for message in MessageIter::new(buffer) {
        match message {
            Message::ControllerData(id, primary, secondary) => {
                if id >= previous_controller_id {
                    let mut controllers = controllers.lock().await;
                    controllers.0 = primary;
                    controllers.1 = secondary;
                    return id;
                }
            }
            Message::LogData(..) => () // ignore this case, it shouldn't come up
        }
    }

    return previous_controller_id;
}
