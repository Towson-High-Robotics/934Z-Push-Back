use std::sync::{Arc, nonpoison::RwLock};

use crate::tracking::Pose;

#[derive(Default, Debug)]
pub(crate) struct Pid {
    last_err: f64 = 0.0,
    sum_err: f64 = 0.0,
    pub kp: f64 = 1.0,
    pub ki: f64 = 1.0,
    pub kd: f64 = 0.0,
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self { Self { kp, ki, kd, ..Default::default() } }

    pub(crate) fn update(&mut self, value: f64, target: f64) -> f64 {
        let error: f64 = target - value;
        self.sum_err = error + 0.95 * self.sum_err;
        let prop: f64 = self.kp * error;
        let deriv: f64 = self.kd * (error - self.last_err);
        let int = self.ki * self.sum_err;
        self.last_err = error;
        prop + deriv + int
    }

    pub(crate) fn reset(&mut self) {
        self.last_err = 0.0;
        self.sum_err = 0.0;
    }
}

#[derive(Default, Debug)]
pub(crate) struct Chassis {
    pub linear: Pid,
    pub heading: Pid,
    pub angular: Pid,
    pub k: f64 = 1.0,
    pub pose: Arc<RwLock<Pose>>,
    pub last_path_vel: f64 = 0.0
}

impl Chassis {
    pub fn new(linear: Pid, heading: Pid, angular: Pid, k: f64, pose: Arc<RwLock<Pose>>) -> Self {
        Self { linear, heading, angular, k, pose, ..Default::default() }
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