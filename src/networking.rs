use crate::{
    logging::{get_current_log, get_current_log_length, get_log, log},
    shared_code::{
        controller::ControllerState,
        message_format::{Message, MessageIter},
    },
    watchdog::Watchdog,
};

use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    IpEndpoint, Stack,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{with_deadline, Duration, Instant, Timer};
// use esp_println::println;
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

    const RX_BUFFER_SIZE: usize = 276;
    const TX_BUFFER_SIZE: usize = 4096;

    let rx_buffer_internal = static_buffer!(u8, [0; RX_BUFFER_SIZE]); // this is honestly overkill, allows for 20 missed log messages at once
    let tx_buffer_internal = static_buffer!(u8, [0; TX_BUFFER_SIZE]);
    let rx_metadata = static_buffer!(PacketMetadata, [PacketMetadata::EMPTY; 8]);
    let tx_metadata = static_buffer!(PacketMetadata, [PacketMetadata::EMPTY; 8]);

    let rx_buffer = static_buffer!(u8, [0; RX_BUFFER_SIZE]);
    let tx_buffer = static_buffer!(u8, [0; TX_BUFFER_SIZE]);

    let working_buffer = static_buffer!(u8, [0; 2048]);

    let resend_requests = static_buffer!(u32, [0; 24]);
    let mut num_resend_requests = 0;

    let mut previous_controller_id = 0;
    let mut log_id = 0;

    let mut socket = UdpSocket::new(
        stack,
        rx_metadata,
        rx_buffer_internal,
        tx_metadata,
        tx_buffer_internal,
    );
    socket.bind(55440).unwrap();

    let mut driver_station_address = None;

    let transmission_period = Duration::from_hz(1);
    let mut timeout = Instant::now() + transmission_period;
    loop {
        match with_deadline(timeout, socket.recv_from(rx_buffer)).await {
            Ok(Ok((size, addr))) => {
                if driver_station_address.is_none() {
                    watchdog.start().await;
                }
                driver_station_address = Some(addr);
                if let Ok(ipv4) = addr.addr.as_bytes().try_into() {
                    log!(WifiReceivedPacket {
                        address: ipv4,
                        port: addr.port
                    });
                } else {
                    // println!("Received packet from address: {:?}", addr);
                }
                watchdog.feed().await;

                num_resend_requests = receive_packet(
                    &rx_buffer[..size],
                    resend_requests,
                    &mut previous_controller_id,
                    controllers,
                )
                .await;
            }
            Ok(Err(_)) => {
                log!(ReceivedPacketTooLarge);
            }
            Err(_) => {
                timeout += transmission_period;
                if let Some(addr) = driver_station_address {
                    // send_packet(
                    //     &mut socket,
                    //     addr,
                    //     tx_buffer,
                    //     working_buffer,
                    //     &resend_requests[..num_resend_requests],
                    //     log_id,
                    // )
                    // .await;
                    // log_id += 1;
                    // num_resend_requests = 0;
                }
            }
        }
    }
}

async fn send_packet(
    socket: &mut UdpSocket<'static>,
    address: IpEndpoint,
    data_buffer: &mut [u8],
    temp_buffer: &mut [u8],
    resend_requests: &[u32],
    log_id: u32,
) {
    let current_log_size = 12 + get_current_log_length().await;
    let end_index = data_buffer.len() - current_log_size;
    let other_logs_buffer = &mut data_buffer[..end_index];
    let mut index = 0;
    for &id in resend_requests {
        match get_log(id, temp_buffer).await {
            Ok(size) => {
                let message = Message::LogData(id, &temp_buffer[..size]);
                if other_logs_buffer.len() - index >= message.buffer_len() {
                    index += message.to_le_bytes(&mut other_logs_buffer[index..]) as usize;
                }
            }
            Err(true) => {}
            Err(false) => {
                // do nothing
            }
        }
    }
    let size = get_current_log(temp_buffer).await.unwrap();
    let message = Message::LogData(log_id, &temp_buffer[..size]);
    index += message.to_le_bytes(&mut data_buffer[index..]) as usize;

    _ = socket.send_to(&data_buffer[..index], address).await;
}

async fn receive_packet(
    buffer: &[u8],
    resend_requests: &mut [u32],
    previous_controller_id: &mut u32,
    controllers: &Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
) -> usize {
    let mut size = 0;
    for message in MessageIter::new(buffer) {
        match message {
            Message::ControllerData(id, primary, secondary) => {
                if id >= *previous_controller_id {
                    *previous_controller_id = id;
                    let mut controllers = controllers.lock().await;
                    controllers.0 = primary;
                    controllers.1 = secondary;
                }
            }
            Message::LogData(id, _) => {
                log!(ReceivedLogData(id));
            }
        }
    }

    size
}
