use as5600::asynch::As5600;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::{
    gpio::GpioPin,
    i2c::I2c,
    ledc::{channel::Channel, LowSpeed},
    peripherals::I2C0,
    prelude::_esp_hal_ledc_channel_ChannelIFace,
    Async,
};

use crate::{
    hardware::{Motor, WheelAngle},
    math,
    shared_code::controller::{Button, ControllerState},
};

pub async fn control_logic(
    _spawner: Spawner,
    controllers: &'static Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    armed: &'static Mutex<NoopRawMutex, bool>,
    mut left_motor: Motor<GpioPin<21>>,
    mut right_motor: Motor<GpioPin<5>>,
    // accelerometer: Accelerometer,
    encoder: As5600<I2c<'static, I2C0, Async>>,
    led: Channel<'static, LowSpeed, GpioPin<10>>,
) -> ! {
    left_motor.start_duty_fade(0, 100, 1000).unwrap();
    right_motor.start_duty_fade(0, 100, 1000).unwrap();
    while left_motor.is_fade_running() || right_motor.is_fade_running() {}
    left_motor.start_duty_fade(100, 0, 1000).unwrap();
    right_motor.start_duty_fade(100, 0, 1000).unwrap();
    while left_motor.is_fade_running() || right_motor.is_fade_running() {}

    motor_control(controllers, armed, left_motor, right_motor, encoder, led).await;
}

async fn motor_control(
    controllers: &Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    armed: &Mutex<NoopRawMutex, bool>,
    mut left_motor: Motor<GpioPin<21>>,
    mut right_motor: Motor<GpioPin<5>>,
    // state_vector: &Mutex<NoopRawMutex, StateVector>,
    mut encoder: As5600<I2c<'static, I2C0, Async>>,
    led: Channel<'static, LowSpeed, GpioPin<10>>,
) -> ! {
    let dt = Duration::from_hz(2000);
    let mut ticker = Ticker::every(dt);

    let mut k_spin_power = 30.; // needs to be above 22% as of 4:44 pm, Friday 22 Nov 2024, the night before comp
    // this is probably some sort of esc issue but if it's lower than 22% the left motor doesn't turn
    let mut k_move_power = 4.;
    let mut translation_power = 0.;
    let mut led_on_position = math::deg2rad(-90.); // led turns on when at the 6:00 position
    const LED_APPARENT_POSITION: f32 = math::deg2rad(133. - 90.); // where the led is physically located
    const LED_ADJUST_SPEED: f32 = math::deg2rad(240.);
    const LED_JUMP_ADJUST_VALUE: f32 = math::deg2rad(60.);
    let mut prev_left_bumper = false;
    let mut prev_right_bumper = false;

    let mut previous_time = Instant::now();
    let mut prev_unwrapped_encoder_angle = WheelAngle::default();

    let mut robot_theta = 0.;

    const ANGLE_CONVERSION: f32 = core::f32::consts::TAU / 4096.;
    const TRACK_WIDTH: f32 = 114. * 1e-3;
    const WHEEL_DIAMETER: f32 = 41. * 1e-3;

    const LED_ANGLE: f32 = math::deg2rad(10.);

    loop {
        ticker.next().await;

        // stop if watchdog has timed out or isn't yet armed
        if !*armed.lock().await {
            left_motor.set_duty(50).unwrap();
            right_motor.set_duty(50).unwrap();
            continue;
        }

        let (primary_controller, secondary_controller) = { *controllers.lock().await };

        // allow us to change speeds based on button inputs
        if primary_controller.get(Button::Down) {
            k_spin_power = 30.;
            k_move_power = 4.;
        } else if primary_controller.get(Button::Right) {
            k_spin_power = 60.;
            k_move_power = 4.;
        } else if primary_controller.get(Button::Up) {
            k_spin_power = 90.;
            k_move_power = 4.;
        } else if primary_controller.get(Button::Left) {
            k_spin_power = 30.;
            k_move_power = 8.;
        }

        // assume both motors are spinning at a speed proportional to their input powers
        // this assumption is entirely untrue but it's the best we've got for estimating the speed of the left wheel
        // encoder_correction is the ratio between the average wheel speeds and the right wheel speed
        let encoder_correction = k_spin_power / (k_spin_power + translation_power);

        let mut left_power = 0.;
        let mut right_power = 0.;

        // spin if trigger is more than 1/8 pressed down
        if primary_controller.left_trigger >= 32 {
            left_power += k_spin_power;
            right_power += k_spin_power;
        }

        let Ok(angle) = encoder.angle().await else {
            continue;
        };
        let current_time = Instant::now();

        let encoder_angle = math::deg2rad(360.) - angle as f32 * ANGLE_CONVERSION;
        let unwrapped_encoder_angle = prev_unwrapped_encoder_angle.new(encoder_angle);
        let delta_encoder_angle = unwrapped_encoder_angle - prev_unwrapped_encoder_angle;
        prev_unwrapped_encoder_angle = unwrapped_encoder_angle;

        let dt = f32_seconds(current_time - previous_time);
        previous_time = current_time;

        let robot_omega = encoder_correction * delta_encoder_angle / dt * WHEEL_DIAMETER / TRACK_WIDTH;
        robot_theta += robot_omega * dt;
        robot_theta = math::wrap_angle(robot_theta);

        let delta_robot_theta = dt * robot_omega;

        // fine adjustment for led position
        let led_stick_x = -secondary_controller.right_stick.get_x();
        led_on_position += led_stick_x * LED_ADJUST_SPEED * dt;

        // allow pressing our bumpers to step the led position
        let left_bumper = secondary_controller.get(Button::LeftBumper);
        let right_bumper = secondary_controller.get(Button::RightBumper);
        if left_bumper && !prev_left_bumper {
            led_on_position += LED_JUMP_ADJUST_VALUE;
        }
        if right_bumper && !prev_right_bumper {
            led_on_position -= LED_JUMP_ADJUST_VALUE;
        }
        prev_left_bumper = left_bumper;
        prev_right_bumper = right_bumper;

        led_on_position = math::wrap_angle(led_on_position);

        // control led
        let average_theta = math::wrap_angle(robot_theta + delta_robot_theta * 0.5);
        let led_difference = math::wrap_angle(average_theta - led_on_position);
        if math::abs(led_difference) < LED_ANGLE {
            led.set_duty(100).unwrap();
        } else {
            led.set_duty(0).unwrap();
        }

        // adjust our movement based on the driver controlling the led position
        let led_correction = math::wrap_angle(led_on_position - LED_APPARENT_POSITION);

        let stick_x = -primary_controller.left_stick.get_x();
        let stick_y = primary_controller.left_stick.get_y();
        let stick_magnitude = math::sqrt(stick_x * stick_x + stick_y * stick_y).clamp(0., 1.);
        if stick_magnitude > 0.2 {
            let stick_angle = math::atan2(stick_y, stick_x);
            let angle_error = robot_theta - stick_angle;
            translation_power =
                stick_magnitude * k_move_power * math::cos(angle_error + delta_robot_theta - led_correction);
            left_power -= translation_power;
            right_power += translation_power;
        } else {
            translation_power = 0.;
        }

        left_motor.set_power(left_power).unwrap();
        right_motor.set_power(right_power).unwrap();
    }
}

fn f32_seconds(duration: Duration) -> f32 {
    const SECONDS_PER_MICROSECOND: f32 = 1.0e-6;
    duration.as_micros() as f32 * SECONDS_PER_MICROSECOND
}
