use nalgebra::{Matrix4, Vector4};

use crate::autos::path::{CubicPolyBezier};

pub(crate) fn cubic_regression(x_values: Vec<f64>, y_values: Vec<f64>) -> [f64; 4] {
    let x0 = x_values.len() as f64;
    let (mut x1, mut x2, mut x3, mut x4, mut x5, mut x6) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    let (mut y0, mut y1, mut y2, mut y3) = (0.0, 0.0, 0.0, 0.0);

    for i in 0..x_values.len() {
        let val = x_values[i];
        let val2 = val * val;
        let val3 = val2 * val;
        let val4 = val3 * val;
        let val5 = val4 * val;
        let val6 = val5 * val;
        x1 += val;
        x2 += val2;
        x3 += val3;
        x4 += val4;
        x5 += val5;
        x6 += val6;

        let y_val = y_values[i];
        y0 += y_val;
        y1 += y_val * val;
        y2 += y_val * val2;
        y3 += y_val * val3;
    }

    *Matrix4::new(x0, x1, x2, x3, x1, x2, x3, x4, x2, x3, x4, x5, x3, x4, x5, x6).lu().solve(&Vector4::new(y0, y1, y2, y3)).unwrap().data.0[0]
        .iter()
        .rev()
        .copied()
        .collect::<Vec<f64>>()
        .as_array()
        .unwrap_or(&[0., 0., 0., 0.])
}

pub(crate) fn curve_reg(x_values: Vec<f64>, y_values: Vec<f64>, t_values: Vec<f64>) -> CubicPolyBezier {
    let x = cubic_regression(t_values.clone(), x_values);
    let y = cubic_regression(t_values, y_values);
    CubicPolyBezier {
        a: (x[0], y[0]),
        b: (x[1], y[1]),
        c: (x[2], y[2]),
        d: (x[3], y[3]),
    }
}
