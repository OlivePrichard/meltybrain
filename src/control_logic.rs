use core::f32::consts::{PI, TAU};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::{
    gpio::GpioPin, i2c::{Error, I2c}, ledc::{channel::Channel, LowSpeed}, peripherals::I2C0, prelude::_esp_hal_ledc_channel_ChannelIFace, Async, Blocking
};
use esp_println::println;

use crate::{
    hardware::IrSensor, math::{self, wheelSpeedtoAngularVelocity}, shared_code::controller::{Button, ControllerState}
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
    ir_sensor: IrSensor,
) -> ! {

    motor_control(controllers, armed, i2c, led, ir_sensor).await;
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
    mut ir_sensor: IrSensor,
) -> ! {
    let dt = Duration::from_hz(3200);
    let mut ticker = Ticker::every(dt);

    let mut robot_omega_sp = 20.;
    let mut robot_v_sp = 15.;
    const TANK_FORWARD: f32 = 60.;
    const TANK_ROTATE: f32 = 15.;
    // let mut accelerometer_radius: f32 = 10e-3; // 10 mm
    const IR_SENSOR_POSITION: f32 = math::deg2rad(-45.);
    const IR_BEACON_POSITION: f32 = math::deg2rad(-90.);

    // let rate_command: [u8; 2] = [0x2C, 0x0D];
    // let res = i2c.write(0x53, &rate_command).await;
    // if let Err(e) = res {
    //     println!("A {:?}", e);
    // }

    let mut previous_time = Instant::now();

    let mut robot_inverted = false;
    let mut direction_reversed = false;

    let mut integrated_robot_theta: f32 = 0.;
    let mut feedforward_robot_omega: f32 = 0.; // this is raw estimated without any sort of beacon correction
    let mut loop_counter = 0;
    let mut beacon_offset: f32 = 0.;
    let mut previous_ir_reading = false;

    let mut previous_left = false;
    let mut previous_right = false;
    let mut trim = 0.;

    let mut previous_instant = Instant::now();

    loop {
        ticker.next().await;

        let (primary_controller, _secondary_controller) = if *armed.lock().await {
            *controllers.lock().await
        } else {
            (ControllerState::default(), ControllerState::default())
        };

        // allow us to change speeds based on button inputs
        if primary_controller.get(Button::Down) {
            robot_omega_sp = 20.;
            robot_v_sp = 15.;
        } else if primary_controller.get(Button::Right) {
            robot_omega_sp = 60.;
            robot_v_sp = 15.;
        } else if primary_controller.get(Button::Up) {
            robot_omega_sp = 70.;
            robot_v_sp = 5.;
        } else if primary_controller.get(Button::Left) {
            robot_omega_sp = 50.;
            robot_v_sp = 25.;
        }

        let mut left_motor_sp = 0.;
        let mut right_motor_sp = 0.;


        let now = Instant::now();
        let dt = (now - previous_instant).as_micros() as f32 * 1e-6;
        previous_instant = now;
        integrated_robot_theta += dt * feedforward_robot_omega;
        integrated_robot_theta = math::wrap_angle(integrated_robot_theta);

        // spin if trigger is more than 1/8 pressed down
        if primary_controller.left_trigger >= 32 {
            left_motor_sp -= robot_omega_sp;
            right_motor_sp += robot_omega_sp;
        
            led_update(&led, integrated_robot_theta + beacon_offset, robot_inverted).await;
        } else {
            _ = led.set_duty(50u8);
        }

        // if loop_counter % 4 == 0 {
        //     // accelerometer shit
        // }

        if loop_counter % 2 == 0 {
            let actual_theta = integrated_robot_theta + beacon_offset;
            let mut x = primary_controller.left_stick.get_x();
            let mut y = primary_controller.left_stick.get_y();
            let right_x = primary_controller.right_stick.get_x();
            let left = primary_controller.get(Button::LeftBumper);
            let right = primary_controller.get(Button::RightBumper);
            let tank_mode = primary_controller.right_trigger >= 32;
            let left_edge = left && !previous_left;
            previous_left = left;
            let right_edge = right && !previous_right;
            previous_right = right;
            if left_edge {
                trim -= 1.;
            }
            if right_edge {
                trim += 1.;
            }
            let desired_angle = math::atan2(y, x);// + math::deg2rad(90.);
            let mut power = math::sqrt(x * x + y * y);
            if power < 0.2 {
                power = 0.;
                x = 0.;
                y = 0.;
            }
            let (left, right) =
                calculate_motor_command(actual_theta, desired_angle, robot_v_sp * power);
            if tank_mode {
                left_motor_sp = TANK_FORWARD * y + TANK_ROTATE * right_x;
                right_motor_sp = TANK_FORWARD * y - TANK_ROTATE * right_x;
            }
            left_motor_sp += left;
            right_motor_sp += right;
            let res = motor_command(&mut i2c, left_motor_sp, right_motor_sp, trim).await;
            if let Ok(angular_velocity_from_motors) = res {
                feedforward_robot_omega = angular_velocity_from_motors;
            } 
        }

        let ir_reading = ir_sensor.read().await;
        if let Ok(reading) = ir_reading {
            let signal = reading < 0.6;
            let rising_edge = signal && !previous_ir_reading;
            previous_ir_reading = signal;
            if rising_edge {
                let current_position_est = IR_BEACON_POSITION - IR_SENSOR_POSITION;
                beacon_offset = current_position_est - integrated_robot_theta;
            }
        }

        loop_counter += 1
    }
}

async fn led_update(led: &Channel<'static, LowSpeed, GpioPin<10>>, position: f32, inverted: bool) {
    const LED_ROBOT_POSITION: f32 = math::deg2rad(-35.);
    const LED_ON_POSITION: f32 = math::deg2rad(-90.); // led turns on when at the 6:00 position
    const LED_WINDOW: f32 = math::deg2rad(10.);
    let led_position = math::wrap_angle(if inverted { position - LED_ROBOT_POSITION } else { position + LED_ROBOT_POSITION });
    let led_err = math::wrap_angle(led_position - LED_ON_POSITION);
    _ = if math::abs(led_err) < LED_WINDOW {
        led.set_duty(100u8)
    } else {
        led.set_duty(0u8)
    }
}

fn calculate_motor_command(angle: f32, heading_direction: f32, target_speed: f32) -> (f32, f32) {
    const MOVEMENT_ZONE_WIDTH: f32 = math::deg2rad(20.);
    let error = math::wrap_angle(angle - heading_direction);
    if math::abs(error) < MOVEMENT_ZONE_WIDTH {
        (target_speed, target_speed)
    } else if math::abs(math::wrap_angle(error + math::deg2rad(180.))) < MOVEMENT_ZONE_WIDTH {
        (-target_speed, -target_speed)
    } else {
        (0., 0.)
    }
}

async fn motor_command(
    i2c: &mut I2c<'static, I2C0, Blocking>,
    left_vel: f32,
    right_vel: f32,
    trim: f32,
) -> Result<f32, Error> {
    // println!("Motor command: {}, {}", left_vel, right_vel);
    let left_buffer = (-left_vel).to_bits().to_le_bytes();
    let right_buffer = right_vel.to_bits().to_le_bytes();
    let mut packet = [0; 8];
    packet[0..4].copy_from_slice(&left_buffer);
    packet[4..8].copy_from_slice(&right_buffer);
    i2c.write(0x42, &packet)?;
    Ok(wheelSpeedtoAngularVelocity(left_vel, right_vel, trim))
}

// async fn read_accelerometer(
//     i2c: &mut I2c<'static, I2C0, Blocking>
// ) -> Result<[f32; 3], Error> {
//     const G_PER_LSB: f32 = 0.049;
//     const GRAV: f32 = 9.80665;
//     let mut data_in = [0u8; 6];
//     let register = [0x32u8];
//     i2c.write_read(0x53, &register, &mut data_in)?;
//     let mut axes = [0., 0., 0.];
//     for i in 0..3 {
//         let value = i16::from_le_bytes(data_in[(2 * i)..(2 * i + 2)].try_into().expect("This is always 2 bytes so the conversion never fails."));
//         axes[i] = value as f32 * G_PER_LSB * GRAV;
//     }
//     Ok(axes)
// }
