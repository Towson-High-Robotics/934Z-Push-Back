use std::{
    sync::{nonpoison::RwLock, Arc},
    time::Instant,
};

use crate::tracking::Pose;

#[derive(Debug)]
pub(crate) struct Pid {
    prev_val: f64,
    last_err: f64,
    sum_err: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
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
            prev_val: 0.0,
            last_err: 0.0,
            sum_err: 0.0,
            kp: 4.0,
            ki: 0.0,
            kd: 20.0,
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
    pub fn new(kp: f64, ki: f64, kd: f64, slew: f64, small_error: f64, small_error_timeout: f64, large_error: f64, large_error_timeout: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
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

    pub(crate) fn update(&mut self, target: f64) -> f64 {
        let error: f64 = target - self.prev_val;
        self.prev_val = target;
        self.sum_err = error + 0.95 * self.sum_err;
        let prop: f64 = self.kp * error;
        let deriv: f64 = self.kd * (error - self.last_err);
        let int = self.ki * self.sum_err;
        self.last_err = error;
        prop + deriv + int
    }

    pub(crate) fn update_timeouts(&mut self, value: f64) -> bool {
        if value < self.small_error && self.small_timeout_start.elapsed().as_millis() > self.small_error_timeout as u128
            || value < self.large_error && self.large_timeout_start.elapsed().as_millis() > self.large_error_timeout as u128
        {
            return true;
        } else if value > self.small_error {
            self.small_timeout_start = Instant::now();
        } else if value > self.large_error {
            self.large_timeout_start = Instant::now();
        }
        false
    }

    pub(crate) fn reset(&mut self) {
        self.last_err = 0.0;
        self.sum_err = 0.0;
        self.prev_val = 0.0;
    }
}

#[derive(Default, Debug)]
pub(crate) struct Chassis {
    pub linear: Pid,
    pub heading: Pid,
    pub angular: Pid,
    pub k: f64 = 1.0,
    pub pose: Arc<RwLock<Pose>>,
    pub last_linear_out: f64,
    pub last_angular_out: f64,
}

impl Chassis {
    pub fn new(linear: Pid, heading: Pid, angular: Pid, k: f64, pose: Arc<RwLock<Pose>>) -> Self {
        Self {
            linear,
            heading,
            angular,
            k,
            pose,
            ..Default::default()
        }
    }

    pub fn calibrate(&mut self, init_pose: (f64, f64, f64)) {
        self.reset();
        let mut writer = self.pose.write();
        writer.reset_pos = init_pose;
        writer.calibrate = true;
    }

    pub fn set_pose(&mut self, pose: (f64, f64, f64)) {
        let mut writer = self.pose.write();
        writer.reset_pos = pose;
        writer.reset = true;
    }

    pub fn reset(&mut self) {
        self.linear.reset();
        self.heading.reset();
        self.angular.reset();
    }
}
