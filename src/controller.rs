use serde::{ Serialize, Deserialize };
<<<<<<< HEAD
use vexide::float::Float;

use crate::conf::ControllerConfig;

#[derive(Serialize, Deserialize, Clone, Copy)]
pub(crate) enum ControlSchemes {
=======
use vexide::devices::controller::ControllerState;
use vexide::prelude::Float;

use crate::conf::Config;

#[derive(Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
pub(crate) enum ControllerLayouts {
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
    Tank,
    Arcade,
    FlippedArcade,
    SingleArcadeLeft,
    SingleArcadeRight
}

<<<<<<< HEAD
#[derive(Serialize, Deserialize, Clone, Copy)]
pub(crate) enum ControllerCurves {
    Linear,
    Cubic,
    Custom
}

fn mag(v: (f64, f64)) -> f64 {
    (v.0 * v.0 + v.1 * v.1).sqrt()
}

fn norm(v: (f64, f64), s: f64) -> (f64, f64) {
    let v_mag = mag(v);
    (v.0 * s / v_mag, v.1 * s / v_mag)
}

fn clamp_mag(v: (f64, f64), min: f64, max: f64) -> (f64, f64) {
    let v_mag = mag(v);
    if v_mag < min { norm(v, min) } else if v_mag > max { norm(v, max) } else { v }
}

pub(crate) fn apply_curve(cont_conf: ControllerConfig, left: (f64, f64), right: (f64, f64)) -> ((f64, f64), (f64, f64)) {
    let mut mod_left = left;
    let mut mod_right = right;
    match cont_conf.controller_curve_type {
        ControllerCurves::Linear => {},
        ControllerCurves::Cubic => {
            let mut left_m = mag(mod_left);
            if left_m < cont_conf.left_deadzone_inner { left_m = 0.; } else if left_m > cont_conf.left_deadzone_outer { left_m = cont_conf.left_deadzone_outer; }
            let mut right_m = mag(mod_right);
            if right_m < cont_conf.right_deadzone_inner { right_m = 0.; } else if right_m > cont_conf.right_deadzone_outer { right_m = cont_conf.right_deadzone_outer; }
            mod_left = clamp_mag(norm(mod_left, left_m * left_m * left_m), 0., 1.);
            mod_right = clamp_mag(norm(mod_right, right_m * right_m * right_m), 0., 1.);
        },
        ControllerCurves::Custom => todo!("Remind me to make the impl for this later"),
    }
    (mod_left, mod_right)
}

pub(crate) fn get_drive_volts(cont_conf: ControllerConfig, left: (f64, f64), right: (f64, f64)) -> (f64, f64) {
    match cont_conf.control_scheme {
        ControlSchemes::Tank => {
            (left.1.signum() * mag(left) * 12., right.1.signum() * mag(right) * 12.)
        },
        ControlSchemes::Arcade => {
            ((left.1 - right.0) * 12., (left.1 + right.0) * 12.)
        },
        ControlSchemes::FlippedArcade => {
            ((right.1 - left.0) * 12., (right.1 + left.0) * 12.)
        },
        ControlSchemes::SingleArcadeLeft => {
            ((left.1 - left.0) * 12., (left.1 + left.0) * 12.)
        },
        ControlSchemes::SingleArcadeRight => {
            ((right.1 - right.0) * 12., (right.1 + right.0) * 12.)
=======
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
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
        },
    }
}