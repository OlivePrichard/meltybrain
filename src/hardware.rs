#![allow(dead_code)]

use core::{
    fmt::{self, Display},
    ops::Sub,
};

use esp_hal::{
    gpio::OutputPin,
    i2c::{Error, I2c},
    ledc::{
        channel::{self, Channel},
        LowSpeed,
    },
    peripherals::I2C0,
    prelude::_esp_hal_ledc_channel_ChannelIFace,
    Async,
};
use esp_println::println;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccelerometerData {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

fn convert_to_mss(value: i16) -> f32 {
    // in milli-g's
    let mg = (value as f32) * 49.0;
    let g = mg / 1000.0;
    let mss = g * 9.80665;
    mss
}

fn convert_to_offset(value: f32) -> i8 {
    const G_PER_LSB: f32 = 0.196;
    let gs = value / 9.80665;
    let mut lsb = -gs / G_PER_LSB;
    if lsb > 0. {
        lsb += 0.5;
    } else {
        lsb -= 0.5;
    }
    lsb as i8
}

impl<'a> From<[u8; 6]> for AccelerometerData {
    fn from(bytes: [u8; 6]) -> Self {
        Self {
            x: convert_to_mss(i16::from_le_bytes([bytes[0], bytes[1]])) - X_OFFSET,
            y: convert_to_mss(i16::from_le_bytes([bytes[2], bytes[3]])) - Y_OFFSET,
            z: convert_to_mss(i16::from_le_bytes([bytes[4], bytes[5]])) - Z_OFFSET,
        }
    }
}

impl Display for AccelerometerData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "x: {:02.2},\ty: {:02.2},\tz: {:02.2}",
            self.x, self.y, self.z
        )
    }
}

const X_OFFSET: f32 = 9.460854;
const Y_OFFSET: f32 = 3.8646255;
const Z_OFFSET: f32 = 5.829481 - 9.80665;

pub struct Accelerometer {
    i2c: I2c<'static, I2C0, Async>,
}

impl<'a> Accelerometer {
    pub fn new(i2c: I2c<'static, I2C0, Async>) -> Self {
        Self { i2c }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        // writes to the register 0x2d with 0x08 to enable the appropriate regsiter.
        self.i2c.write(0x53, &[0x2d, 0x08]).await?;
        Ok(())
    }

    pub async fn find_calibration_offset(&mut self) -> Result<(), Error> {
        // self.i2c.write(0x53, &[0x1E, 0x00]).await?;
        // self.i2c.write(0x53, &[0x1F, 0x00]).await?;
        // self.i2c.write(0x53, &[0x20, 0x00]).await?;

        const SAMPLES: usize = 20_000;

        let mut x_sum = 0.0;
        let mut y_sum = 0.0;
        let mut z_sum = 0.0;

        for _ in 0..SAMPLES {
            let data = self.read_all().await?;
            x_sum += data.x;
            y_sum += data.y;
            z_sum += data.z;
        }
        let x_calibration_offset = x_sum / SAMPLES as f32;
        let y_calibration_offset = y_sum / SAMPLES as f32;
        let z_calibration_offset = z_sum / SAMPLES as f32;

        println!(
            "x: {}\ny: {}\nz: {}",
            x_calibration_offset, y_calibration_offset, z_calibration_offset
        );

        let x_offset = convert_to_offset(x_calibration_offset);
        let y_offset = convert_to_offset(y_calibration_offset);
        let z_offset = convert_to_offset(z_calibration_offset - 9.80665);

        let _x_offset_byte = u8::from_ne_bytes(x_offset.to_ne_bytes());
        let _y_offset_byte = u8::from_ne_bytes(y_offset.to_ne_bytes());
        let _z_offset_byte = u8::from_ne_bytes(z_offset.to_ne_bytes());

        // self.i2c.write(0x53, &[0x1E, x_offset_byte]).await?;
        // self.i2c.write(0x53, &[0x1F, y_offset_byte]).await?;
        // self.i2c.write(0x53, &[0x20, z_offset_byte]).await?;

        Ok(())
    }

    pub async fn read_all(&mut self) -> Result<AccelerometerData, Error> {
        let mut buffer = [0u8; 6];
        // recieve 6 bytes starting at 0x32 which should be everything.
        self.i2c.write_read(0x53, &[0x32], &mut buffer).await?;
        Ok(AccelerometerData::from(buffer))
    }
}

pub struct Motor<Pin: OutputPin + 'static> {
    pwm: Channel<'static, LowSpeed, Pin>,
}

impl<Pin: OutputPin + 'static> Motor<Pin> {
    pub fn new(pwm: Channel<'static, LowSpeed, Pin>) -> Self {
        Self { pwm }
    }

    // technically these references to &mut self could by &self, but I feel like this makes more sense
    pub fn start_power_fade(
        &mut self,
        start: f32,
        end: f32,
        duration_ms: u16,
    ) -> Result<(), channel::Error> {
        let start = power_to_duty_cycle(start);
        let end = power_to_duty_cycle(end);
        self.pwm.start_duty_fade(start, end, duration_ms)
    }

    pub fn start_duty_fade(
        &mut self,
        start: u8,
        end: u8,
        duration_ms: u16,
    ) -> Result<(), channel::Error> {
        self.pwm.start_duty_fade(start, end, duration_ms)
    }

    pub fn is_fade_running(&self) -> bool {
        self.pwm.is_duty_fade_running()
    }

    pub fn set_duty(&mut self, duty: u8) -> Result<(), channel::Error> {
        self.pwm.set_duty(duty)
    }

    pub fn set_power(&mut self, power: f32) -> Result<(), channel::Error> {
        let duty = power_to_duty_cycle(power);
        self.pwm.set_duty(duty)
    }
}

fn power_to_duty_cycle(power: f32) -> u8 {
    (power.clamp(0., 100.) * 0.5 + 0.5) as u8 + 50
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelAngle {
    // this is only gonna work for positive angles, im not gonna fix it
    turns: i32, // rotations
    angle: f32, // radians
}

impl WheelAngle {
    pub fn new(&self, angle: f32) -> Self {
        use core::f32::consts::PI;

        if angle < self.angle - PI {
            Self {
                turns: self.turns + 1,
                angle,
            }
        } else if angle > self.angle + PI {
            Self {
                turns: self.turns - 1,
                angle,
            }
        } else {
            Self { angle, ..*self }
        }
    }
}

impl From<f32> for WheelAngle {
    fn from(angle: f32) -> Self {
        use core::f32::consts::TAU;

        let turns = (angle / TAU) as i32;
        let angle = angle - turns as f32 * TAU;

        Self { turns, angle }
    }
}

impl From<WheelAngle> for f32 {
    fn from(angle: WheelAngle) -> f32 {
        use core::f32::consts::TAU;

        angle.angle + angle.turns as f32 * TAU
    }
}

impl Sub for WheelAngle {
    type Output = f32;

    fn sub(self, rhs: Self) -> Self::Output {
        use core::f32::consts::TAU;

        let turns = self.turns - rhs.turns;
        let angle = self.angle - rhs.angle;

        turns as f32 * TAU + angle
    }
}
