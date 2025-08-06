use serde::{ Serialize, Deserialize };
use vexide::devices::controller::ControllerState;
use vexide::prelude::Float;

use crate::conf::Config;

#[derive(Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
pub(crate) enum ControllerLayouts { Tank, Arcade, FlippedArcade, SingleArcadeLeft, SingleArcadeRight }
impl Default for ControllerLayouts { fn default() -> Self { Self::Tank } }

#[derive(Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
pub(crate) enum JoystickCurves { Linear, Cubic, CubicInverse, Custom }
impl Default for JoystickCurves { fn default() -> Self { Self::Linear } }

fn mag(v: (f64, f64)) -> f64 { (v.0*v.0 + v.1*v.1).sqrt() }

fn norm(v: (f64, f64), s: f64) -> (f64, f64) { let l = mag(v); (v.0 / l * s, v.1 / l * s) }

pub(crate) fn apply_curve(conf: &Config, state: &ControllerState) -> ((f64, f64), (f64, f64)) {
    let (left, right) = ((state.left_stick.x(), state.left_stick.y()), (state.right_stick.x(), state.right_stick.y()));

    let (mut left_mag, mut right_mag) = (mag(left), mag(right));

    if conf.controller.left_deadzone_inner >= left_mag { left_mag = 0.0; }
    else if left_mag >= conf.controller.left_deadzone_outer { left_mag = conf.controller.left_deadzone_outer; }

    if conf.controller.right_deadzone_inner >= right_mag { right_mag = 0.0; }
    else if right_mag >= conf.controller.right_deadzone_outer { right_mag = conf.controller.right_deadzone_outer; }

    match conf.controller.curve {
        JoystickCurves::Linear => { (norm(left, left_mag), norm(right, right_mag)) },
        JoystickCurves::Cubic => { (norm(left, left_mag * left_mag * left_mag), norm(right, right_mag * right_mag * right_mag)) },
        JoystickCurves::CubicInverse => { (norm(left, left_mag.powf(1.0/3.0)), norm(right, right_mag.powf(1.0/3.0))) },
        JoystickCurves::Custom => todo!(),
    }
}

pub(crate) fn get_drive_volts(conf: &Config, left: (f64, f64), right: (f64, f64)) -> (f64, f64) {
    match conf.controller.layout {
        ControllerLayouts::Tank => { (11.0 * left.1, -11.0 * right.1) },
        ControllerLayouts::Arcade => { (11.0 * (left.1 - right.0), 11.0 * (left.1 + right.0)) },
        ControllerLayouts::FlippedArcade => { (11.0 * (right.1 - left.0), 11.0 * (right.1 + left.0)) },
        ControllerLayouts::SingleArcadeLeft => { (11.0 * (left.1 - left.0), 11.0 * (left.1 + left.0)) },
        ControllerLayouts::SingleArcadeRight => { (11.0 * (right.1 - right.0), 11.0 * (right.1 + right.0)) },
    }
}