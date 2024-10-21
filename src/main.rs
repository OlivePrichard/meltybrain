#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::{Io, Level, Output}, prelude::*};
use esp_println::println;

#[entry]
fn main() -> ! {
    #[allow(unused)]
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    esp_println::logger::init_logger_from_env();
    
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio7, Level::Low);

    led.set_high();

    println!("Hello World!");

    loop {
        delay.delay_millis(500);
        led.toggle();
    }
}
