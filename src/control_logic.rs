use core::f32::consts::{PI, TAU};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::{
    gpio::GpioPin,
    i2c::{Error, I2c},
    ledc::{channel::Channel, LowSpeed},
    peripherals::I2C0,
    Async, Blocking,
};
use esp_println::println;

use crate::{
    math,
    shared_code::controller::{Button, ControllerState},
};

const FEATHER_ADDR: u8 = 0x42;
const ACCEL_ADDR: u8 = 0x53;

pub async fn control_logic(
    _spawner: Spawner,
    controllers: &'static Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    armed: &'static Mutex<NoopRawMutex, bool>,
    // accelerometer: Accelerometer,
    i2c: I2c<'static, I2C0, Blocking>,
    led: Channel<'static, LowSpeed, GpioPin<10>>,
) -> ! {

    motor_control(controllers, armed, i2c, led).await;
}

async fn motor_control(
    controllers: &Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
    armed: &Mutex<NoopRawMutex, bool>,
    // mut left_motor: Motor<GpioPin<21>>,
    // mut right_motor: Motor<GpioPin<5>>,
    // state_vector: &Mutex<NoopRawMutex, StateVector>,
    // mut encoder: As5600<I2c<'static, I2C0, Async>>,
    mut i2c: I2c<'static, I2C0, Blocking>,
    led: Channel<'static, LowSpeed, GpioPin<10>>,
) -> ! {
    let dt = Duration::from_hz(3200);
    let mut ticker = Ticker::every(dt);

    let mut k_spin_velocity = 80.; // needs to be above 22% as of 4:44 pm, Friday 22 Nov 2024, the night before comp
    // this is probably some sort of esc issue but if it's lower than 22% the left motor doesn't turn
    let mut k_move_velocity = 8.;
    const LED_ON_POSITION: f32 = math::deg2rad(-90.); // led turns on when at the 6:00 position
    const LED_WINDOW: f32 = math::deg2rad(10.);
    let mut accelerometer_radius: f32 = 10e-3; // 10 mm

    let rate_command: [u8; 2] = [0x2C, 0x0D];
    // let res = i2c.write(0x53, &rate_command).await;
    // if let Err(e) = res {
    //     println!("A {:?}", e);
    // }

    let mut previous_time = Instant::now();

    let mut robot_theta: f32 = 0.;
    let mut robot_omega: f32 = 0.;
    let mut loop_counter = 0;

    loop {
        ticker.next().await;

        // stop if watchdog has timed out or isn't yet armed
        if !*armed.lock().await {
            let res = motor_command(&mut i2c, 0., 0.).await;
            if let Err(e) = res {
                // println!("B {:?}", e);
            }
            // left_motor.set_duty(50).unwrap();
            // right_motor.set_duty(50).unwrap();
            continue;
        }

        let (primary_controller, _secondary_controller) = { *controllers.lock().await };

        // allow us to change speeds based on button inputs
        if primary_controller.get(Button::Down) {
            k_spin_velocity = 40.;
            k_move_velocity = 8.;
        } else if primary_controller.get(Button::Right) {
            k_spin_velocity = 80.;
            k_move_velocity = 8.;
        } else if primary_controller.get(Button::Up) {
            k_spin_velocity = 120.;
            k_move_velocity = 8.;
        } else if primary_controller.get(Button::Left) {
            k_spin_velocity = 80.;
            k_move_velocity = 16.;
        }

        let mut left_power = 0.;
        let mut right_power = 0.;

        // spin if trigger is more than 1/8 pressed down
        if primary_controller.left_trigger >= 32 {
            left_power -= k_spin_velocity;
            right_power += k_spin_velocity;
        }

        let dt = dt.as_micros() as f32 / 1e6;
        robot_theta += dt * robot_omega;
        if robot_theta >= PI {
            robot_theta -= TAU;
        }
        else if robot_theta < -PI {
            robot_theta += TAU;
        }
        led_update(&led, robot_theta).await;

        if loop_counter % 4 == 0 {
            // accelerometer shit
        }

        if loop_counter % 2 == 0 {
            let res = motor_command(&mut i2c, left_power, right_power).await;
            if let Err(e) = res {
                // println!("C {:?}", e);
            }
        }

        loop_counter += 1
    }
}

async fn led_update(led: &Channel<'static, LowSpeed, GpioPin<10>>, position: f32) {

}

async fn motor_command(
    i2c: &mut I2c<'static, I2C0, Blocking>,
    left_vel: f32,
    right_vel: f32,
) -> Result<(), Error> {
    // println!("Motor command: {}, {}", left_vel, right_vel);
    let left_buffer = left_vel.to_bits().to_le_bytes();
    let right_buffer = right_vel.to_bits().to_le_bytes();
    let mut packet = [0; 8];
    packet[0..4].copy_from_slice(&left_buffer);
    packet[4..8].copy_from_slice(&right_buffer);
    i2c.write(0x42, &packet)?;
    Ok(())
}

async fn read_accelerometer(
    i2c: &mut I2c<'static, I2C0, Blocking>
) -> Result<[f32; 3], Error> {
    const G_PER_LSB: f32 = 0.049;
    const GRAV: f32 = 9.80665;
    let mut data_in = [0u8; 6];
    let register = [0x32u8];
    i2c.write_read(0x53, &register, &mut data_in)?;
    let mut axes = [0., 0., 0.];
    for i in 0..3 {
        let value = i16::from_le_bytes(data_in[(2 * i)..(2 * i + 2)].try_into().expect("This is always 2 bytes so the conversion never fails."));
        axes[i] = value as f32 * G_PER_LSB * GRAV;
    }
    Ok(axes)
}
