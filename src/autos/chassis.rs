use std::{
    sync::{Arc, nonpoison::RwLock},
    time::Instant,
};

use crate::tracking::Tracking;

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

#[derive(Debug)]
pub(crate) struct Chassis {
    pub linear: Pid,
    pub angular: Pid,
    pub k: f64 = 1.0,
    pub pose: Arc<RwLock<Tracking>>,
    pub last_linear_out: f64,
    pub last_angular_out: f64,
}

impl Chassis {
    pub fn new(linear: Pid, angular: Pid, k: f64, pose: Arc<RwLock<Tracking>>) -> Self {
        Self {
            linear,
            angular,
            k,
            pose,
            last_linear_out: 0.0,
            last_angular_out: 0.0,
        }
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
