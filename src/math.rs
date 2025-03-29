#![allow(dead_code)]

use core::f32::consts::PI;
const WHEELBASE: f32 = 116.0 + 16.; // * 1.036888888; // mm wheel base distance 
const WHEEL_DIAMETER: f32 = 48.0; // mm wheel diameter

/**
 * wheel speeds in rotations per second to radians per second of the chassis 
 */
pub fn wheelSpeedtoAngularVelocity(l: f32, r: f32, trim: f32) -> f32 {
    let left_mm: f32 = (WHEEL_DIAMETER * PI * l); // (rotations per second) * mm * pi = mm / s
    let right_mm: f32 = (WHEEL_DIAMETER * PI * r);
    let angular_velocity_mm_s = (right_mm - left_mm) / (WHEELBASE + trim);
    return angular_velocity_mm_s
}


#[inline]
pub fn rad2deg(rad: f32) -> f32 {
    use core::f32::consts::PI;

    rad * 180. / PI
}

#[inline]
pub const fn deg2rad(rad: f32) -> f32 {
    use core::f32::consts::PI;

    rad * PI / 180.
}

/// wraps the input angle (rad) to the range [-pi, pi)
#[inline]
pub fn wrap_angle(mut angle: f32) -> f32 {
    use core::f32::consts::{PI, TAU};

    while angle < -PI {
        angle += TAU;
    }

    while angle >= PI {
        angle -= TAU;
    }

    angle
}

#[inline]
pub fn sin(mut x: f32) -> f32 {
    use core::f32::consts::{FRAC_PI_4, PI};

    if x < 0. {
        x = PI - x;
    }

    let (x, cycles) = reduce(x);

    let y = match cycles & 0b11 {
        0b000 => kernel_sin(x),
        0b001 => kernel_cos(FRAC_PI_4 - x),
        0b010 => kernel_cos(x),
        0b011 => kernel_sin(FRAC_PI_4 - x),
        _ => unreachable!(),
    };

    if cycles & 0b100 != 0 {
        -y
    } else {
        y
    }
}

#[inline]
pub fn cos(x: f32) -> f32 {
    use core::f32::consts::FRAC_PI_4;

    let (x, cycles) = reduce(x);

    let y = match cycles & 0b11 {
        0b000 => kernel_cos(x),
        0b001 => kernel_sin(FRAC_PI_4 - x),
        0b010 => -kernel_sin(x),
        0b011 => -kernel_cos(FRAC_PI_4 - x),
        _ => unreachable!(),
    };

    if cycles & 0b100 != 0 {
        -y
    } else {
        y
    }
}

#[inline]
pub fn tan(x: f32) -> f32 {
    let (s, c) = sin_cos(x);
    s / c
}

#[inline]
pub fn sin_cos(x: f32) -> (f32, f32) {
    (sin(x), cos(x))
}

#[inline]
pub fn asin(x: f32) -> f32 {
    atan(x * inv_sqrt(1. - x * x))
}

#[inline]
pub fn acos(x: f32) -> f32 {
    use core::f32::consts::FRAC_PI_2;

    FRAC_PI_2 + asin(-x)
}

#[inline]
pub fn atan(x: f32) -> f32 {
    use core::f32::consts::FRAC_PI_2;

    if x < -1. {
        kernel_atan(-1. / x) - FRAC_PI_2
    } else if x < 0. {
        -kernel_atan(-x)
    } else if x <= 1. {
        kernel_atan(x)
    } else {
        FRAC_PI_2 - kernel_atan(1. / x)
    }
}

#[inline]
pub fn atan2(y: f32, x: f32) -> f32 {
    use core::f32::{
        consts::{FRAC_PI_2, PI},
        NAN,
    };

    if x > 0. {
        atan(y / x)
    } else if x == 0. {
        if y > 0. {
            FRAC_PI_2
        } else if y < 0. {
            -FRAC_PI_2
        } else {
            NAN
        }
    } else if y > 0. {
        atan(y / x) + PI
    } else {
        atan(y / x) - PI
    }
}

#[inline]
pub fn inv_sqrt(x: f32) -> f32 {
    // fast inverse sqrt from Quake III Arena

    let x2 = x * 0.5;
    let i = x.to_bits();
    let i = 0x5F37_59DF - (i >> 1);
    let mut y = f32::from_bits(i);
    y *= 1.5 - x2 * y * y;
    y * 1.5 - x2 * y * y
}

#[inline]
pub fn sqrt(x: f32) -> f32 {
    x * inv_sqrt(x)
}

#[inline]
pub fn abs(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7F_FF_FF_FF)
}

#[inline]
pub fn sign(mag: f32, sign: f32) -> f32 {
    f32::from_bits((mag.to_bits() & 0x7F_FF_FF_FF) | (sign.to_bits() & 0x80_00_00_00))
}

#[inline]
fn reduce(mut x: f32) -> (f32, i32) {
    use core::f32::consts::FRAC_PI_4;

    let cycles = (x / FRAC_PI_4) as i32;
    x -= cycles as f32 * FRAC_PI_4;

    (x, cycles)
}

#[inline]
fn kernel_sin(x: f32) -> f32 {
    // 4 terms gives error bound of 3e-7
    // y = x - x^3/3! + x^5/5! - x^7/7!
    // y = x * (1 - x^2 * (1/3! - x^2 * (1/5! - x^2/7!)))

    const INV_3_FACT: f32 = 1. / 6.;
    const INV_5_FACT: f32 = 1. / 120.;
    const INV_7_FACT: f32 = 1. / 5040.;

    let x2 = x * x;
    x * (1. - x2 * (INV_3_FACT - x2 * (INV_5_FACT - x2 * INV_7_FACT)))
}

#[inline]
fn kernel_cos(x: f32) -> f32 {
    // 5 terms gives error bound of 2e-8
    // y = 1 - x^2/2! + x^4/4! - x^6/6! + x^8/8!
    // y = 1 - x^2 * (1/2! - x^2 * (1/4! - x^2 * (1/6! - x^2/8!)))

    const INV_2_FACT: f32 = 1. / 2.;
    const INV_4_FACT: f32 = 1. / 24.;
    const INV_6_FACT: f32 = 1. / 720.;
    const INV_8_FACT: f32 = 1. / 40320.;

    let x2 = x * x;
    1. - x2 * (INV_2_FACT - x2 * (INV_4_FACT - x2 * (INV_6_FACT - x2 * INV_8_FACT)))
}

#[inline]
fn kernel_atan(x: f32) -> f32 {
    // uses approximation: arctan(x) = pi/4 * x - x * (x - 1) * (0.2447 + 0.0663 * x)
    // error is < 0.086 degrees on [0, 1]
    // arctan(x) = x * (pi/4 - (x - 1) * (0.2447 + 0.0663 * x))
    // arctan(x) = (pi/4 + .2447 - ((.2447 - .0663) + .0663 * x) * x) * x

    use core::f32::consts::FRAC_PI_4;

    const A: f32 = FRAC_PI_4 + 0.2447;
    const B: f32 = 0.1784;
    const C: f32 = 0.0663;

    let y = (A - (B + C * x) * x) * x;

    // newton iteration
    // f(y) = tan(y) - x
    // f'(y) = sec^2(y)
    // y_new = y - (tan(y) - x) / sec^2(x)
    // y_new = y - cos(y) * sin(y) + cos^2(y) * x

    let (s, c) = sin_cos(y);
    y - s * c + c * c * x
}
