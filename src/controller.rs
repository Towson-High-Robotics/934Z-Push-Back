use serde::{ Serialize, Deserialize };
use vexide::devices::controller::ControllerState;
use vexide::prelude::Float;

use crate::conf::Config;

#[derive(Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
pub(crate) enum ControllerLayouts {
    Tank,
    Arcade,
    FlippedArcade,
    SingleArcadeLeft,
    SingleArcadeRight
}

impl Default for ControllerLayouts { fn default() -> Self { Self::Tank } }

#[derive(Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
pub(crate) enum JoystickCurves {
    Linear,
    Cubic,
    CubicInverse,
    Custom
}

impl Default for JoystickCurves { fn default() -> Self { Self::Linear } }

fn mag(v: (f64, f64)) -> f64 {
    (v.0*v.0 + v.1*v.1).sqrt()
}

fn norm(v: (f64, f64), s: f64) -> (f64, f64) {
    let l = mag(v);
    (v.0 / l * s, v.1 / l * s)
}

fn dot(v0: (f64, f64), v1: (f64, f64)) -> f64 {
    v0.0*v1.0 + v0.1*v1.1
}

pub fn apply_curve(conf: &Config, state: &ControllerState) -> ((f64, f64), (f64, f64)) {
    let mut left = (state.left_stick.x(), state.left_stick.y());
    let mut right = (state.right_stick.y(), state.right_stick.x());

    let mut left_mag = mag(left);
    let mut right_mag = mag(right);

    if conf.controller.left_deadzone_inner >= left_mag {
        left_mag = 0.0;
    } else if left_mag >= conf.controller.left_deadzone_outer {
        left_mag = conf.controller.left_deadzone_outer;
    }

    if conf.controller.right_deadzone_inner >= right_mag {
        right_mag = 0.0;
    } else if right_mag >= conf.controller.right_deadzone_outer {
        right_mag = conf.controller.right_deadzone_outer;
    }

    match conf.controller.curve {
        JoystickCurves::Linear => {
            left = norm(left, left_mag);
            right = norm(right, right_mag);
        },
        JoystickCurves::Cubic => {
            left = norm(left, left_mag * left_mag * left_mag);
            right = norm(right, right_mag * right_mag * right_mag);
        },
        JoystickCurves::CubicInverse => {
            left = norm(left, left_mag.powf(1.0/3.0));
            right = norm(right, right_mag.powf(1.0/3.0));
        },
        JoystickCurves::Custom => todo!(),
    }

    (left, right)
}

pub fn get_drive_volts(conf: &Config, left: (f64, f64), right: (f64, f64)) -> (f64, f64) {
    match conf.controller.layout {
        ControllerLayouts::Tank => {
            (11.0 * dot(left, (0.0, 1.0)), 11.0 * dot(right, (0.0, 1.0)))
        },
        ControllerLayouts::Arcade => {
            let left_dot = dot(left, (0.0, 1.0));
            let right_dot = dot(right, (1.0, 0.0));
            (11.0 * (left_dot - right_dot), 11.0 * (left_dot + right_dot))
        },
        ControllerLayouts::FlippedArcade => {
            let left_dot = dot(left, (1.0, 0.0));
            let right_dot = dot(right, (0.0, 1.0));
            (11.0 * (right_dot - left_dot), 11.0 * (right_dot + left_dot))
        },
        ControllerLayouts::SingleArcadeLeft => {
            (11.0 * (left.1 - left.0), 11.0 * (left.1 + left.0))
        },
        ControllerLayouts::SingleArcadeRight => {
            (11.0 * (right.1 - right.0), 11.0 * (right.1 + right.0))
        },
    }
}