#![no_std]
#![no_main]

use core::fmt::{self, Display};
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    gpio::Io, i2c::{Error, I2c}, peripherals::I2C0, prelude::*, timer::timg::TimerGroup, Async
};
use esp_println::println;

#[derive(Debug)]
struct AccelerometerData {
    x: f32,
    y: f32,
    z: f32,
}

impl Display for AccelerometerData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "x: \t {:02.2}, y: \t {:02.2}, z: \t {:02.2}", self.x, self.y, self.z)
    }
}

struct Accelerometer<'a> { 
    i2c: I2c<'a, I2C0, Async>,
}

fn convert_to_mss(value: i16) -> f32 {
    // in milli-g's 
    let mg = (value as f32) * 49.0;
    let g = mg / 1000.0;
    let mss = g * 9.80665;
    mss
}

impl<'a> From<[u8; 6]> for AccelerometerData {
    fn from(bytes: [u8; 6]) -> Self {
        Self {
            x: convert_to_mss(i16::from_le_bytes([bytes[0], bytes[1]])),
            y: convert_to_mss(i16::from_le_bytes([bytes[2], bytes[3]])),
            z: convert_to_mss(i16::from_le_bytes([bytes[4], bytes[5]])),
        }
    }
}

impl<'a> Accelerometer<'a> {
    async fn init(&mut self) -> Result<(), Error> {
        // writes to the register 0x2d with 0x08 to enable the appropriate regsiter. 
        self.i2c.write(0x53, &[0x2d, 0x08]).await?;
        Ok(())
    }

    async fn read_all(&mut self) -> Result<AccelerometerData, Error> {
        let mut buffer = [0u8; 6];
        // recieve 6 bytes starting at 0x32 which should be everything. 
        self.i2c.write_read(0x53, &[0x32], &mut buffer).await?;
        Ok(AccelerometerData::from(buffer))
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c0 = I2c::new_async(peripherals.I2C0, io.pins.gpio6, io.pins.gpio7, 400.kHz());

    let mut accelerometer = Accelerometer { i2c: i2c0 };

    while accelerometer.init().await.is_err() {
        println!("Failed to initialize accelerometer, retrying...");
    }

    loop {
        let data = accelerometer.read_all().await;
        if let Ok(data) = data {
            println!("{}", data);
        }
    }
}

fn angle_wrap(angle: f32) -> f32 {
    let mut angle = angle;
    while angle > 3.14 { angle -= 2.0 * 3.14159265359 }
    while angle < -3.14 { angle += 2.0 * 3.14159265359 }
    angle
}       

fn factorial(n: u32) -> f32 {
    let mut result = 1.0;
    for i in 1..=n {
        result *= i as f32;
    }
    result
}

fn pow(x: f32, n: u32) -> f32 {
    let mut result = 1.0;
    for _ in 0..n {
        result *= x;
    }
    result
}

fn iterative_sin(mut x: f32) -> f32 {
    x = angle_wrap(x);  
    // Normalize x to be between -2π and 2π
    const TWO_PI: f32 = 2.0 * 3.14159265359;
    x = x % TWO_PI;
    
    let mut result = 0.0;
    // Use first 7 terms of Taylor series for reasonable accuracy
    // sin(x) = x - x³/3! + x⁵/5! - x⁷/7! + x⁹/9! - x¹¹/11! + x¹³/13!
    for i in 0..7 {
        let n = 2 * i + 1;
        let term = pow(x, n as u32) / factorial(n as u32);
        if i % 2 == 0 {
            result += term;
        } else {
            result -= term;
        }
    }
    result
}

fn iterative_cos(mut x: f32) -> f32 {
    x = angle_wrap(x);
    // Normalize x to be between -2π and 2π
    const TWO_PI: f32 = 2.0 * 3.14159265359;
    x = x % TWO_PI;
    
    let mut result = 0.0;
    // Use first 7 terms of Taylor series for reasonable accuracy
    // cos(x) = 1 - x²/2! + x⁴/4! - x⁶/6! + x⁸/8! - x¹⁰/10! + x¹²/12!
    for i in 0..7 {
        let n = 2 * i;
        let term = pow(x, n as u32) / factorial(n as u32);
        if i % 2 == 0 {
            result += term;
        } else {
            result -= term;
        }
    }
    result
}

fn fast_inverse_sqrt(x: f32) -> f32 {
    let i = x.to_bits();
    let i = 0x5f3759df - (i >> 1);
    let mut y = f32::from_bits(i);
    y *= (1.5 - 0.5 * x * y * y);
    y * (1.5 - 0.5 * x * y * y)
}   

