use serde::{ Serialize, Deserialize };
use vexide::float::Float;

use crate::conf::ControllerConfig;

#[derive(Serialize, Deserialize, Clone, Copy)]
pub(crate) enum ControlSchemes {
    Tank,
    Arcade,
    FlippedArcade,
    SingleArcadeLeft,
    SingleArcadeRight
}

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
        },
    }
}