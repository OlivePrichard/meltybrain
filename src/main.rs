#![no_std]
#![no_main]


use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    gpio::Io, i2c::I2c, ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed}, prelude::*, riscv::asm::delay, timer::timg::TimerGroup

};
use esp_println::println;



#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let left_motor = io.pins.gpio21; // io.pins.gpio9
    let right_motor = io.pins.gpio5;

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

    let mut leftChannel = ledc.get_channel(channel::Number::Channel0, left_motor);
    leftChannel
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    let mut rightChannel = ledc.get_channel(channel::Number::Channel1, right_motor);
    rightChannel
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // channel0.start_duty_fade(0, 100, 2000).expect_err(
    //     "Fading from 0% to 100%, at 24kHz and 5-bit resolution, over 2 seconds, should fail",
    // );
    leftChannel.start_duty_fade(0, 50, 500).unwrap();
    rightChannel.start_duty_fade(0, 50, 500).unwrap();
    while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
    }
    leftChannel.start_duty_fade(50, 0, 500).unwrap();
    rightChannel.start_duty_fade(50, 0, 500).unwrap();
    while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
    }
    leftChannel.start_duty_fade(0, 50, 500).unwrap();
    rightChannel.start_duty_fade(0, 50, 500).unwrap();
    while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
    }
    leftChannel.start_duty_fade(50, 0, 1500).unwrap();
    rightChannel.start_duty_fade(50, 0, 1500).unwrap();
    while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
    }
    leftChannel.start_duty_fade(0, 70, 1000).unwrap();
    rightChannel.start_duty_fade(0, 70, 1000).unwrap();
    while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
    }



    loop {
        leftChannel.start_duty_fade(40, 70, 1000).unwrap();
        rightChannel.start_duty_fade(40, 70, 1000).unwrap();
        while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {}
        leftChannel.start_duty_fade(70, 40, 1000).unwrap();
        rightChannel.start_duty_fade(70, 40, 1000).unwrap(); 
        while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {}
    }
 

}