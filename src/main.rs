#![no_std]
#![no_main]


use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    gpio::Io, i2c::I2c, ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed}, prelude::*, timer::timg::TimerGroup

};
use esp_println::println;



#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let led = io.pins.gpio21; // io.pins.gpio9

    let mut ledc = Ledc::new(peripherals.LEDC);

    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 8.kHz(),
        })
        .unwrap();

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, led);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // channel0.start_duty_fade(0, 100, 2000).expect_err(
    //     "Fading from 0% to 100%, at 24kHz and 5-bit resolution, over 2 seconds, should fail",
    // );
    channel0.start_duty_fade(0, 50, 500).unwrap();
    while channel0.is_duty_fade_running() {
    }
    channel0.start_duty_fade(50, 0, 500).unwrap();
    while channel0.is_duty_fade_running() {
    }
    channel0.start_duty_fade(0, 50, 500).unwrap();
    while channel0.is_duty_fade_running() {
    }
    channel0.start_duty_fade(50, 0, 1500).unwrap();
    while channel0.is_duty_fade_running() {
    }
    channel0.start_duty_fade(0, 70, 1000).unwrap();
    while channel0.is_duty_fade_running() {
    }
    loop {
        channel0.set_duty(70).unwrap();
    }
 

}