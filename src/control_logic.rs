use as5600::asynch::As5600;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_hal::{gpio::GpioPin, i2c::I2c, peripherals::I2C0, Async};
// use esp_println::println;

use crate::{
    hardware::{Accelerometer, Motor, WheelAngle},
    logging::log,
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
) -> ! {
    log!(InitializingMotors);

    left_motor.start_duty_fade(0, 100, 1000).unwrap();
    right_motor.start_duty_fade(0, 100, 1000).unwrap();
    while left_motor.is_fade_running() || right_motor.is_fade_running() {}
    left_motor.start_duty_fade(100, 0, 1000).unwrap();
    right_motor.start_duty_fade(100, 0, 1000).unwrap();
    while left_motor.is_fade_running() || right_motor.is_fade_running() {}

    log!(MotorsInitialized);

    // let state_vector = &*mk_static!(
    //     Mutex<NoopRawMutex, StateVector>, Mutex::new(StateVector {
    //     time: Instant::now(),
    //     theta: 0.0,
    //     omega: 0.0,
    // }));

    // spawner
    //     .spawn(accelerometer_data(state_vector, accelerometer))
    //     .ok();
    motor_control(controllers, armed, left_motor, right_motor, encoder).await;
}

// #[embassy_executor::task]
async fn accelerometer_data(
    state_vector: &'static Mutex<NoopRawMutex, StateVector>,
    mut accelerometer: Accelerometer,
) -> ! {
    const ACCELEROMETER_POSITION: f32 = 5.0 * 1.0e-3; // 6mm

    let period = Duration::from_hz(800);

    let mut _offset_calibration = 0.;
    const SAMPLES: usize = 800 * 5;

    for _ in 0..SAMPLES {
        Timer::after(period).await;

        let Ok(data) = accelerometer.read_all().await else {
            continue;
        };

        _offset_calibration += data.y;
    }
    // let offset = offset_calibration / SAMPLES as f32;

    // let mut observer = ConstantVelocityObserver::new(0.09, 0.);
    // let mut omega_offset = 100;

    loop {
        Timer::after(period).await;

        let Ok(_data) = accelerometer.read_all().await else {
            continue;
        };
        let time = Instant::now();
        // println!("{}", data);

        // a = r * omega^2
        // omega = 1 / sqrt(r / a)
        // println!("{}", data.y - offset);
        // let omega_raw = math::sqrt(math::abs(data.y - offset) / ACCELEROMETER_POSITION);
        let omega = 100.0; // observer.observe(omega_raw);

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
    // state_vector: &Mutex<NoopRawMutex, StateVector>,
    mut encoder: As5600<I2c<'static, I2C0, Async>>,
) -> ! {
    let dt = Duration::from_hz(2000);
    let mut ticker = Ticker::every(dt);

    let spin_power = 10.;
    let move_power = 5.;

    let mut measurment_buffer = [(Instant::now(), WheelAngle::default()); 400];
    let mut measurment_index = 0;

    let mut theta = 0.;

    const ANGLE_CONVERSION: f32 = core::f32::consts::TAU / 4096.;
    const TRACK_WIDTH: f32 = 120. * 1e-3;
    const WHEEL_DIAMETER: f32 = 30. * 1e-3;

    loop {
        ticker.next().await;

        if !*armed.lock().await {
            left_motor.set_duty(50).unwrap();
            right_motor.set_duty(50).unwrap();
            continue;
        }

        let (primary_controller, _secondary_controller) = { *controllers.lock().await };
        // let state = { state_vector.lock().await.predict(Instant::now()) };

        let mut left_power = 0.;
        let mut right_power = 0.;

        if primary_controller.get(Button::LeftBumper) {
            left_power += spin_power;
            right_power += spin_power;
        }

        let Ok(angle) = encoder.angle().await else { continue; };
        let current_time = Instant::now();
        let radians = angle as f32 * ANGLE_CONVERSION;
        let mut earliest_measurement_index = measurment_index + 1;
        if earliest_measurement_index >= measurment_buffer.len() {
            earliest_measurement_index = 0;
        }
        let (earliest_time, earliest_angle) = measurment_buffer[earliest_measurement_index];
        let (latest_time, latest_angle) = measurment_buffer[measurment_index];
        let current_angle = latest_angle.new(radians);
        let average_delta_angle = current_angle - earliest_angle;
        let average_delta_time = f32_seconds(current_time - earliest_time);
        let omega = average_delta_angle / average_delta_time * WHEEL_DIAMETER / TRACK_WIDTH;
        let dt = f32_seconds(current_time - latest_time);
        theta += omega * dt;
        measurment_buffer[earliest_measurement_index] = (current_time, current_angle);
        measurment_index = earliest_measurement_index;

        let stick_x = primary_controller.left_stick.get_x();
        let stick_y = primary_controller.left_stick.get_y();
        let magnitude = math::sqrt(stick_x * stick_x + stick_y * stick_y);
        if magnitude > 0.2 {
            let correction = dt * omega;
            let stick_angle = math::atan2(stick_y, stick_x);
            let angle_error = theta - stick_angle;
            let movement_power = move_power * math::cos(angle_error + correction);
            left_power -= movement_power;
            right_power += movement_power;
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

        // left_motor.set_power(left_power).unwrap();
        // right_motor.set_power(right_power).unwrap();
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
    alpha: f32,
    last_value: f32,
}

impl LowPassFilter {
    fn new(alpha: f32, initial_value: f32) -> Self {
        Self {
            alpha,
            last_value: initial_value,
        }
    }

    fn filter(&mut self, value: f32) -> f32 {
        self.last_value = self.last_value * self.alpha + value * (1. - self.alpha);
        self.last_value
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
