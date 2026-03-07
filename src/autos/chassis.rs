use std::{
    sync::{nonpoison::RwLock, Arc},
    time::Instant,
};

use vexide::{prelude::*, smart::motor::BrakeMode};

use crate::{tracking::Tracking, util::Drivetrain};

#[derive(Debug)]
pub(crate) struct Pid {
    last_err: f64,
    last_deriv: f64,
    sum_err: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub deriv_alpha: f64,
    pub slew: f64,
    pub small_error: f64,
    pub small_error_timeout: f64,
    small_timeout_start: Instant,
    pub large_error: f64,
    pub large_error_timeout: f64,
    large_timeout_start: Instant,
}

impl Default for Pid {
    fn default() -> Self {
        Self {
            last_err: 0.0,
            last_deriv: 0.0,
            sum_err: 0.0,
            kp: 4.0,
            ki: 0.0,
            kd: 20.0,
            deriv_alpha: 0.7,
            slew: 75.0,
            small_error: 1.0,
            small_error_timeout: 100.0,
            small_timeout_start: Instant::now(),
            large_error: 3.0,
            large_error_timeout: 500.0,
            large_timeout_start: Instant::now(),
        }
    }
}

impl Pid {
    #[allow(clippy::too_many_arguments)]
    pub fn new(kp: f64, ki: f64, kd: f64, deriv_alpha: f64, slew: f64, small_error: f64, small_error_timeout: f64, large_error: f64, large_error_timeout: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            deriv_alpha,
            slew,
            small_error,
            small_error_timeout,
            small_timeout_start: Instant::now(),
            large_error,
            large_error_timeout,
            large_timeout_start: Instant::now(),
            ..Default::default()
        }
    }

    pub(crate) fn update(&mut self, error: f64) -> f64 {
        self.sum_err = error + 0.95 * self.sum_err;
        let prop = self.kp * error;
        let raw_deriv = error - self.last_err;
        self.last_deriv = self.deriv_alpha * (self.last_deriv - raw_deriv) + raw_deriv;
        let deriv = self.kd * self.last_deriv;
        let int = self.ki * self.sum_err;
        self.last_err = error;
        prop + deriv + int
    }

    pub(crate) fn update_timeouts(&mut self, value: f64) -> bool {
        if (value < self.small_error && self.small_timeout_start.elapsed().as_secs_f64() * 1000.0 > self.small_error_timeout)
            || (value < self.large_error && self.large_timeout_start.elapsed().as_secs_f64() * 1000.0 > self.large_error_timeout)
        {
            return true;
        }
        if value > self.small_error {
            self.small_timeout_start = Instant::now();
        }
        if value > self.large_error {
            self.large_timeout_start = Instant::now();
        }
        false
    }

    pub(crate) fn reset(&mut self) {
        self.last_err = 0.0;
        self.sum_err = 0.0;
    }
}

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

pub fn desaturate(p: (f64, f64)) -> (f64, f64) {
    let left = p.0 - p.1;
    let right = p.0 + p.1;
    let max = left.abs().max(right.abs());
    if max <= 1.0 {
        (left, right)
    } else {
        (left / max, right / max)
    }
}

#[derive(Default, Debug)]
pub struct ControllerSettings {
    pub left_deadzone: (f64, f64) = (0.01, 0.01),
    pub right_deadzone: (f64, f64) = (0.01, 0.01),
    pub curve: (f64, f64) = (1.028, 1.028),
    pub scale: (f64, f64) = (1.0, 1.0),
    pub min_out: (f64, f64) = (0.025, 0.025),
}

#[derive(Debug)]
pub(crate) struct Chassis {
    pub linear: Pid,
    pub angular: Pid,
    pub k: f64 = 1.0,
    pub drive: Arc<RwLock<Drivetrain>>,
    pub pose: Arc<RwLock<Tracking>>,
    pub controller: ControllerSettings,
    pub last_linear_out: f64 = 0.0,
    pub last_angular_out: f64 = 0.0,
}

impl Chassis {
    pub fn new(drive: Arc<RwLock<Drivetrain>>, pose: Arc<RwLock<Tracking>>) -> Self {
        Self {
            drive, pose,
            linear: Pid::default(),
            angular: Pid::default(),
            k: 1.0,
            controller: ControllerSettings::default(),
            last_linear_out: 0.0,
            last_angular_out: 0.0,
        }
    }

    pub(crate) fn arcade(&mut self, left: f64, right: f64) {
        let motor_percents = desaturate((
            drive_curve(left, self.controller.scale.0, self.controller.min_out.0, self.controller.curve.0, self.controller.left_deadzone.0, self.controller.left_deadzone.1),
            drive_curve(right, self.controller.scale.1, self.controller.min_out.1, self.controller.curve.1, self.controller.right_deadzone.0, self.controller.right_deadzone.1)
        ));

        let mut dt = self.drive.write();
        let left_con = dt.left_motors.iter().filter(|m| m.is_connected()).count() as f64;
        let right_con = dt.left_motors.iter().filter(|m| m.is_connected()).count() as f64;
        if left_con.min(right_con) == 0.0 {
            return;
        }

        let enabled_ratio = left_con.min(right_con) / left_con.max(right_con);

        dt.left_motors.iter_mut().for_each(|m| {
            m.set_voltage(motor_percents.0 * m.max_voltage() * enabled_ratio).ok();
        });
        dt.right_motors.iter_mut().for_each(|m| {
            m.set_voltage(motor_percents.1 * m.max_voltage() * enabled_ratio).ok();
        });
    }

    pub(crate) fn tank(&mut self, left: f64, right: f64) {
        let motor_percents = (
            drive_curve(left, self.controller.scale.0, self.controller.min_out.0, self.controller.curve.0, self.controller.left_deadzone.0, self.controller.left_deadzone.1).clamp(0.0, 1.0),
            drive_curve(right, self.controller.scale.1, self.controller.min_out.1, self.controller.curve.1, self.controller.right_deadzone.0, self.controller.right_deadzone.1).clamp(0.0, 1.0)
        );

        let mut dt = self.drive.write();
        let left_con = dt.left_motors.iter().filter(|m| m.is_connected()).count() as f64;
        let right_con = dt.left_motors.iter().filter(|m| m.is_connected()).count() as f64;
        if left_con.min(right_con) == 0.0 {
            return;
        }

        let enabled_ratio = left_con.min(right_con) / left_con.max(right_con);

        dt.left_motors.iter_mut().for_each(|m| {
            m.set_voltage(motor_percents.0 * m.max_voltage() * enabled_ratio).ok();
        });
        dt.right_motors.iter_mut().for_each(|m| {
            m.set_voltage(motor_percents.1 * m.max_voltage() * enabled_ratio).ok();
        });
    }

    pub(crate) fn set_brake_mode(&mut self, mode: BrakeMode) {
        self.drive.write().left_motors.iter_mut().for_each(|m| { m.brake(mode).ok(); });
        self.drive.write().right_motors.iter_mut().for_each(|m| { m.brake(mode).ok(); });
    }

    pub async fn calibrate(&mut self, init_pose: (f64, f64, f64)) {
        self.reset();
        self.last_linear_out = 0.0;
        self.last_angular_out = 0.0;
        self.pose.write().calibrate(init_pose).await;
    }

    pub fn set_pose(&mut self, pose: (f64, f64, f64)) {
        self.pose.write().reset_pose(pose);
    }

    pub fn reset(&mut self) {
        self.linear.reset();
        self.angular.reset();
        self.last_linear_out = 0.0;
        self.last_angular_out = 0.0;
    }
}
