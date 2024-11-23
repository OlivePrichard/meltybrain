use crate::{
    logging::{get_telemetry, log},
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

    let transmission_period = Duration::from_hz(20);
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

                previous_controller_id =
                    receive_packet(&rx_buffer[..size], previous_controller_id, controllers).await;
            }
            Ok(Err(_)) => {
                log!(ReceivedPacketTooLarge);
            }
            Err(_) => {
                timeout += transmission_period;
                if let Some(addr) = driver_station_address {
                    // send_packet(&mut socket, addr, log_id).await;
                    // log_id += 1;
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

async fn send_packet(socket: &mut UdpSocket<'static>, address: IpEndpoint, log_id: u32) {
    let mut telemetry = get_telemetry().lock().await;
    let id_bytes = log_id.to_le_bytes();
    telemetry.logs[0..4].copy_from_slice(&id_bytes);

    _ = socket
        .send_to(&telemetry.logs[..telemetry.index], address)
        .await;
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
            Message::LogData(..) => {
                unreachable!();
                // log!(ReceivedLogData(id));
            }
        }
    }

    return previous_controller_id;
}
