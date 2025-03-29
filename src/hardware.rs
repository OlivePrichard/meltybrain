#![allow(dead_code)]

use core::ops::Sub;

use esp_hal::{
    analog::adc::{Adc, AdcPin}, gpio::{GpioPin, OutputPin}, ledc::{
        channel::{self, Channel},
        LowSpeed,
    }, peripherals::ADC1, prelude::{_esp_hal_ledc_channel_ChannelIFace, nb}
};

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

pub struct IrSensor {
    adc: &'static mut Adc<'static, ADC1>,
    pin: AdcPin<GpioPin<3>, ADC1>,
}

impl IrSensor {
    pub fn new(adc: &'static mut Adc<'static, ADC1>, pin: AdcPin<GpioPin<3>, ADC1>) -> Self {
        Self {
            adc,
            pin,
        }
    }

    pub async fn read(&mut self) -> Result<f32, ()> {
        nb::block!(self.adc.read_oneshot(&mut self.pin)).map(|x| { x as f32 / 0x10_00 as f32 })
    }
}
