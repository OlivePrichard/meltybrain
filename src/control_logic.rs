use as5600::asynch::As5600;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_hal::{
    gpio::GpioPin,
    i2c::I2c,
    ledc::{channel::Channel, LowSpeed},
    peripherals::I2C0,
    prelude::_esp_hal_ledc_channel_ChannelIFace,
    Async,
};
use esp_println::println;
// use esp_println::println;

use crate::{
    hardware::{Accelerometer, Motor, WheelAngle}, logging::log, math, mk_static, shared_code::controller::{Button, ControllerState}
};

pub async fn control_logic(
    spawner: Spawner,
    controllers: &'static Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    armed: &'static Mutex<NoopRawMutex, bool>,
    mut left_motor: Motor<GpioPin<21>>,
    mut right_motor: Motor<GpioPin<5>>,
    accelerometer: Accelerometer,
    // encoder: As5600<I2c<'static, I2C0, Async>>,
    led: Channel<'static, LowSpeed, GpioPin<10>>,
) -> ! {
    log!(InitializingMotors);

    left_motor.start_duty_fade(0, 100, 1000).unwrap();
    right_motor.start_duty_fade(0, 100, 1000).unwrap();
    while left_motor.is_fade_running() || right_motor.is_fade_running() {}
    left_motor.start_duty_fade(100, 0, 1000).unwrap();
    right_motor.start_duty_fade(100, 0, 1000).unwrap();
    while left_motor.is_fade_running() || right_motor.is_fade_running() {}

    log!(MotorsInitialized);

    let state_vector = &*mk_static!(
        Mutex<NoopRawMutex, StateVector>, Mutex::new(StateVector {
        time: Instant::now(),
        theta: 0.0,
        omega: 0.0,
    }));

    spawner
        .spawn(accelerometer_data(state_vector, accelerometer))
        .ok();
    motor_control(controllers, armed, left_motor, right_motor, state_vector, led).await;
}

#[embassy_executor::task]
async fn accelerometer_data(
    state_vector: &'static Mutex<NoopRawMutex, StateVector>,
    controllers: &'static Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    mut accelerometer: Accelerometer,
) -> ! {
    const ACCELEROMETER_POSITION: f32 = 5.0 * 1.0e-3; // 6mm

    let mut accelerometer_position_offset = 0.;

    let period = Duration::from_hz(800);

    let mut offset_calibration = 0.;
    const SAMPLES: usize = 800 * 5;

    for _ in 0..SAMPLES {
        Timer::after(period).await;

        let Ok(data) = accelerometer.read_all().await else {
            continue;
        };

        offset_calibration += data.y;
    }
    let offset = offset_calibration / SAMPLES as f32;

    let mut observer = ConstantVelocityObserver::new(0.09, 0.);
    let mut previous_up = false;
    let mut previous_down = false;
    // let mut omega_offset = 100;

    loop {
        Timer::after(period).await;

        let Ok(data) = accelerometer.read_all().await else {
            continue;
        };

        let inc_up = controllers.lock().await.0.get(Button::Left);
        let inc_down = controllers.lock().await.0.get(Button::Right);

        if inc_up && !previous_up {
            accelerometer_position_offset += 0.1;
        }
        if inc_down && !previous_down {
            accelerometer_position_offset -= 0.1;
        }

        previous_up = inc_up;
        previous_down = inc_down;

        let time = Instant::now();
        // println!("{}", data);

        // a = r * omega^2
        // omega = 1 / sqrt(r / a)
        // println!("{}", data.y - offset);
        let omega_raw = math::sqrt(math::abs(data.y - offset) / ACCELEROMETER_POSITION + accelerometer_position_offset);
        let omega = observer.observe(omega_raw);

        let mut state = state_vector.lock().await;
        // let average_omega = (state.omega + omega) * 0.5;
        let dt = time - state.time;
        let dtheta = omega * f32_seconds(dt);
        let theta = state.theta + dtheta;
        *state = StateVector { time, theta, omega };
    }
}

// fn encoder_data(encoder: As5600<I2C0>, state_vector: &'static Mutex<NoopRawMutex, StateVector>) -> ! {
//     let period = Duration::from_hz(2000);
// }

async fn motor_control(
    controllers: &Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    armed: &Mutex<NoopRawMutex, bool>,
    mut left_motor: Motor<GpioPin<21>>,
    mut right_motor: Motor<GpioPin<5>>,
    state_vector: &Mutex<NoopRawMutex, StateVector>,
    // mut encoder: As5600<I2c<'static, I2C0, Async>>,
    mut led: Channel<'static, LowSpeed, GpioPin<10>>,
) -> ! {
    let dt = Duration::from_hz(2000);
    let mut ticker = Ticker::every(dt);

    let mut spin_power = 30.; // needs to be above 22% as of 4:44 pm on friday
    let move_power = 8.;
    let mut movement_power = 0.;
    let mut led_position = math::deg2rad(-90.);
    const LED_APPARENT_POSITION: f32 = math::deg2rad(133. - 90.);
    const LED_ADJUST_SPEED: f32 = math::deg2rad(240.);
    const LED_JUMP_ADJUST_VALUE: f32 = math::deg2rad(60.);
    let mut prev_left_bumper = false;
    let mut prev_right_bumper = false;

    let mut prev_up = false;
    let mut prev_down = false;

    // let mut measurment_buffer = [(Instant::now(), WheelAngle::default()); 400];
    // let mut measurment_index = 0;

    let mut previous_time = Instant::now();
    let mut previous_angle = WheelAngle::default();
    let mut previous_omega = 0.;

    let mut theta = 0.;

    const ANGLE_CONVERSION: f32 = core::f32::consts::TAU / 4096.;
    const TRACK_WIDTH: f32 = 114. * 1e-3;
    const WHEEL_DIAMETER: f32 = 41. * 1e-3;

    const LED_ANGLE: f32 = math::deg2rad(10.);
    const BANG_ZONE: f32 = math::deg2rad(22.5);

    loop {
        ticker.next().await;

        if !*armed.lock().await {
            left_motor.set_duty(50).unwrap();
            right_motor.set_duty(50).unwrap();
            continue;
        }

        let (primary_controller, secondary_controller) = { *controllers.lock().await };
        let state = { state_vector.lock().await.predict(Instant::now()) };

        let up = primary_controller.get(Button::Up);
        let down = primary_controller.get(Button::Down);
        if up && !prev_up {
            spin_power += 2.;
        }
        if down && !prev_down {
            spin_power -= 2.;
        }
        prev_up = up;
        prev_down = down;

        let encoder_correction = spin_power / (spin_power + movement_power);

        let mut left_power = 0.;
        let mut right_power = 0.;

        if primary_controller.left_trigger >= 32 {
            left_power += spin_power;
            right_power += spin_power;
        }

        // let Ok(angle) = encoder.angle().await else {
        //     continue;
        // };

        let current_time = Instant::now();
        let radians = state.theta;
        // let mut earliest_measurement_index = measurment_index + 1;
        // if earliest_measurement_index >= measurment_buffer.len() {
        //     earliest_measurement_index = 0;
        // }
        // let (earliest_time, earliest_angle) = measurment_buffer[earliest_measurement_index];
        // let (latest_time, latest_angle) = measurment_buffer[measurment_index];
        let current_wheel_angle = previous_angle.new(radians);
        let wheel_delta_angle = current_wheel_angle - previous_angle;
        let average_delta_time = f32_seconds(current_time - previous_time);
        let dt = f32_seconds(current_time - previous_time);

        let stick_x = primary_controller.left_stick.get_x();
        let stick_y = primary_controller.left_stick.get_y();
        let magnitude = math::sqrt(stick_x * stick_x + stick_y * stick_y).clamp(0., 1.);
        let angle = if magnitude > 0.2 {
            Some(math::atan2(stick_y, stick_x))
        } else {
            None
        };
        let in_bang_zone = if let Some(angle) = angle {
            let angle_error = math::abs(math::wrap_angle(
                theta - angle - led_position + LED_APPARENT_POSITION + dt * previous_omega * 0.5,
            ));
            angle_error < BANG_ZONE || angle_error > core::f32::consts::PI - BANG_ZONE
        } else {
            false
        };

        // if !in_bang_zone {
        //     let omega_temp = encoder_correction * wheel_delta_angle / average_delta_time
        //         * WHEEL_DIAMETER
        //         / TRACK_WIDTH;
        //     previous_omega = omega_temp;
        // }
        // let omega = previous_omega;
        // theta += omega * dt;
        // theta = math::wrap_angle(theta);
        // measurment_buffer[earliest_measurement_index] = (current_time, current_angle);
        // measurment_index = earliest_measurement_index;
        previous_angle = current_wheel_angle;
        previous_time = current_time;
        let correction = dt * state.omega;

        let led_stick_x = secondary_controller.right_stick.get_x();
        led_position += led_stick_x * LED_ADJUST_SPEED * dt;

        let left_bumper = primary_controller.get(Button::LeftBumper);
        let right_bumper = primary_controller.get(Button::RightBumper);
        if left_bumper && !prev_left_bumper {
            led_position -= LED_JUMP_ADJUST_VALUE;
        }
        if right_bumper && !prev_right_bumper {
            led_position += LED_JUMP_ADJUST_VALUE;
        }
        prev_left_bumper = left_bumper;
        prev_right_bumper = right_bumper;

        led_position = math::wrap_angle(led_position);

        let average_theta = math::wrap_angle(theta + correction * 0.5);
        let led_difference = math::wrap_angle(average_theta - led_position);
        if math::abs(led_difference) < LED_ANGLE {
            led.set_duty(100).unwrap();
        } else {
            led.set_duty(0).unwrap();
        }

        let led_correction = math::wrap_angle(led_position - LED_APPARENT_POSITION);

        if in_bang_zone {
            let stick_angle = math::atan2(stick_y, stick_x);
            let angle_error = theta - stick_angle;
            let movement_power =
                if math::abs(math::wrap_angle(angle_error - led_correction + correction))
                    < BANG_ZONE
                {
                    move_power
                } else {
                    -move_power
                } * magnitude;
            left_power -= movement_power;
            right_power += movement_power;
        } else {
            movement_power = 0.;
        }

        // let angle_correction = f32_seconds(dt) * state.omega;
        // let stick_x = primary_controller.left_stick.get_x();
        // let stick_y = primary_controller.left_stick.get_y();
        // let magnitude = math::sqrt(stick_x * stick_x + stick_y * stick_y);
        // if magnitude > 0.1 {
        //     let controller_angle = math::atan2(
        //         primary_controller.left_stick.get_y(),
        //         primary_controller.left_stick.get_x(),
        //     );
        //     let angle_error = state.theta - controller_angle;
        //     let movement = move_power * math::cos(angle_error + angle_correction);
        //     left_power -= movement;
        //     right_power += movement;
        // }

        // println!("Angle: {}", math::rad2deg(state.theta));

        // println!("{:.8}\t{:.8}", left_power, right_power);

        left_motor.set_power(left_power).unwrap();
        right_motor.set_power(right_power).unwrap();
    }
}

/*
    poses = [Pose(0, 0, 0)]
    step = 0.01
    move_weight = 0.5
    angular_velocity = 10
    motor_speed = angular_velocity * 0.16 / 2
    angle_offset = angular_velocity * step
    print(np.rad2deg(angle_offset))
    ts = np.arange(0, 10, step)
    for t in ts:
        pose = poses[-1]
        theta = pose.theta
        movement = move_weight * np.cos(theta + angle_offset)
        robot_simulation_step(poses, -motor_speed + movement, motor_speed + movement, step)
    draw_robot_path(poses)
*/

fn f32_seconds(duration: Duration) -> f32 {
    const SECONDS_PER_MICROSECOND: f32 = 1.0e-6;
    duration.as_micros() as f32 * SECONDS_PER_MICROSECOND
}

#[derive(Debug, Clone, Copy)]
struct StateVector {
    time: Instant,
    theta: f32, // radians
    omega: f32, // radians per second
}

impl StateVector {
    fn predict(&self, current_time: Instant) -> Self {
        let dt = current_time - self.time;
        let theta = self.theta + self.omega * f32_seconds(dt);
        let omega = self.omega;
        Self {
            time: current_time,
            theta,
            omega,
        }
    }
}

struct LowPassFilter {
    _alpha: f32,
    _last_value: f32,
}

impl LowPassFilter {
    fn new(alpha: f32, initial_value: f32) -> Self {
        Self {
            _alpha: alpha,
            _last_value: initial_value,
        }
    }

    fn filter(&mut self, value: f32) -> f32 {
        self._last_value = self._last_value * self._alpha + value * (1. - self._alpha);
        self._last_value
    }
}

impl Default for LowPassFilter {
    fn default() -> Self {
        Self::new(0.2, 0.)
    }
}

struct ConstantVelocityObserver {
    k: f32,
    estimated_velocity: f32,
}

impl ConstantVelocityObserver {
    fn new(k: f32, initial_velocity: f32) -> Self {
        Self {
            k,
            estimated_velocity: initial_velocity,
        }
    }

    fn observe(&mut self, z: f32) -> f32 {
        self.estimated_velocity = self.estimated_velocity + self.k * (z - self.estimated_velocity);
        self.estimated_velocity
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
