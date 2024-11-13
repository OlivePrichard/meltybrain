#![no_std]
#![no_main]


use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    gpio::Io, i2c::I2c, prelude::*, timer::timg::TimerGroup
};
use esp_println::println;



#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut i2c0 = I2c::new_async(peripherals.I2C0, io.pins.gpio6, io.pins.gpio7, 400.kHz());

    let mut buffer = [0u8; 1024];
    loop {
        buffer = [0u8; 1024];
        match i2c0.read(0x53, &mut buffer).await {
            Ok(_) => println!("{:02X?}", &buffer[..100]),
            Err(e) => println!("{:?}", e),
        }
    }
}
