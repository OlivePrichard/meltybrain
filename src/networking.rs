use embassy_net::Stack;
use esp_wifi::wifi::{WifiApDevice, WifiDevice};

macro_rules! static_buffer {
    ($t:ty, [$val:expr; $size: expr]) => {{
        static STATIC_CELL: static_cell::StaticCell<[$t; $size]> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        STATIC_CELL.init_with(|| [$val; $size])
    }};
}

pub async fn handle_networking(stack: &'static Stack<WifiDevice<'static, WifiApDevice>>) -> ! {
    let rx_buffer = static_buffer!(u8, [0; 276]); // this is honestly overkill, allows for 20 missed log messages at once
    let tx_buffer = static_buffer!(u8, [0; 1024 * 4]);

    unreachable!()
}