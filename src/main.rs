#![no_std]
#![no_main]

mod shared_code;

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    gpio::Io, timer::timg::TimerGroup
};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
}
