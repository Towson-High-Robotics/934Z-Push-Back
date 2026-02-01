use vexide::controller::ControllerState;

use crate::{autos::auto::desaturate, conf::Config, util::mag};

fn drive_curve(v: f64, s: f64, n: f64, a: f64, dl: f64, du: f64) -> f64 {
    if v == 0.0 {
        return 0.0;
    }
    let snms = (s - n) / s;
    if v.signum() == -1.0 {
        let sius = s / (a.powf(s.abs() - dl - s) * (s.abs() - dl));
        let iu = a.powf(v.abs() - dl - s) * (v.abs() - dl) * sius;
        -snms * iu - n
    } else {
        let sius = s / (a.powf(s.abs() - du - s) * (s.abs() - du));
        let iu = a.powf(v.abs() - du - s) * (v.abs() - du) * sius;
        snms * iu + n
    }
}

pub(crate) fn arcade(conf: &Config, state: &ControllerState) -> (f64, f64) {
    let (left, right) = ((state.left_stick.x(), state.left_stick.y()), (state.right_stick.x(), state.right_stick.y()));

    let (mut left_mag, mut right_mag) = (mag(left), mag(right));

    if conf.controller.left_deadzone_inner >= left_mag {
        left_mag = 0.0;
    }
    left_mag = left_mag.min(conf.controller.left_deadzone_outer);

    if conf.controller.right_deadzone_inner >= right_mag {
        right_mag = 0.0;
    }
    right_mag = right_mag.min(conf.controller.right_deadzone_outer);

    let arcade_vals = desaturate((
        drive_curve(left.1 / left_mag, 1.0, 0.05, conf.controller.curve_amt, 0.0, 0.0),
        drive_curve(right.0 / right_mag, 1.0, 0.05, conf.controller.curve_amt, 0.0, 0.0),
    ));
    (arcade_vals.0 - arcade_vals.1, arcade_vals.0 + arcade_vals.1)
}
