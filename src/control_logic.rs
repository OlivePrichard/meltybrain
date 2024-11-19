use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use esp_hal::{
    gpio::GpioPin,
    ledc::{channel::Channel, LowSpeed},
    prelude::_esp_hal_ledc_channel_ChannelIFace,
};
// use esp_println::println;

use crate::{logging::log, shared_code::controller::ControllerState};

type Motor<Pin> = Channel<'static, LowSpeed, Pin>;

#[embassy_executor::task]
pub async fn control_logic(
    controllers: &'static Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    armed: &'static Mutex<NoopRawMutex, bool>,
    left_motor: Motor<GpioPin<21>>,
    right_motor: Motor<GpioPin<5>>,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_hz(2000));

    log!(InitializingMotors);

    left_motor.start_duty_fade(0, 100, 1000).unwrap();
    right_motor.start_duty_fade(0, 100, 1000).unwrap();
    while left_motor.is_duty_fade_running() || right_motor.is_duty_fade_running() {}
    left_motor.start_duty_fade(100, 0, 1000).unwrap();
    right_motor.start_duty_fade(100, 0, 1000).unwrap();
    while left_motor.is_duty_fade_running() || right_motor.is_duty_fade_running() {}

    log!(MotorsInitialized);

    loop {
        ticker.next().await;

        let is_armed = { *armed.lock().await };

        if !is_armed {
            left_motor.set_duty(0).unwrap();
            right_motor.set_duty(0).unwrap();
            continue;
        }

        let (primary_controller, _secondary_controller) = { *controllers.lock().await };
        let power = primary_controller.left_stick.get_y().clamp(0., 1.);
        let duty_cycle = 50 + (power * 50. + 0.5) as u8;

        // log!(MotorPowers { left: duty_cycle, right: duty_cycle });
        // println!("Duty cycle: {}", duty_cycle);

        left_motor.set_duty(duty_cycle).unwrap();
        right_motor.set_duty(duty_cycle).unwrap();
    }
}

// leftChannel.start_duty_fade(0, 100, 1000).unwrap();
// rightChannel.start_duty_fade(0, 100, 1000).unwrap();
// while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
// }
// leftChannel.start_duty_fade(100, 0, 1000).unwrap();
// rightChannel.start_duty_fade(100, 0, 1000).unwrap();
// while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
// }

// loop {
//     leftChannel.start_duty_fade(60, 100, 1000).unwrap();
//     rightChannel.start_duty_fade(60, 100, 1000).unwrap();
//     while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
//     }
//     leftChannel.start_duty_fade(100, 51, 1000).unwrap();
//     rightChannel.start_duty_fade(100, 51, 1000).unwrap();
//     while leftChannel.is_duty_fade_running() || rightChannel.is_duty_fade_running() {
//     }

// }
