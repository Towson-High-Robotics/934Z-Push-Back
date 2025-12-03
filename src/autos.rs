use std::{
    any::{Any, TypeId},
    sync::{nonpoison::RwLock, Arc},
    time::Instant,
    vec::Vec,
};

use vexide::prelude::Motor;

use crate::{
    tracking::Pose,
    util::{mag, norm},
};

#[allow(dead_code)]
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Autos {
    Left,
    Right,
    Solo,
    Skills,
    SkillsDriver,
    #[default]
    None,
    Recorded,
}

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

    fn update(&mut self, value: f64, target: f64) -> f64 {
        let error: f64 = target - value;
        self.sum_err = error + 0.95 * self.sum_err;
        let prop: f64 = self.kp * error;
        let deriv: f64 = self.kd * (error - self.last_err);
        let int = self.ki * self.sum_err;
        self.last_err = error;
        prop + deriv + int
    }

    fn reset(&mut self) {
        self.last_err = 0.0;
        self.sum_err = 0.0;
    }
}

#[derive(Debug)]
pub(crate) struct SpeedCurve {
    start_speed: f64,
    control_1: f64,
    end_speed: f64,
}

impl SpeedCurve {
    pub fn new_linear(start: f64, end: f64) -> Self {
        Self {
            start_speed: start,
            control_1: (start + end) / 2.0,
            end_speed: end,
        }
    }

    pub fn new(start_speed: f64, control_1: f64, end_speed: f64) -> Self { Self { start_speed, control_1, end_speed } }

    pub fn sample(&self, t: f64) -> f64 { t * t * (self.end_speed - 2.0 * self.control_1 + self.start_speed) + 2.0 * t * (self.control_1 - self.start_speed) + self.start_speed }
}

pub(crate) trait Followable {
    fn sample(&self, t: f64) -> (f64, f64);

    fn sample_heading(&self, t: f64) -> f64;
}

#[derive(Debug)]
pub(crate) struct LinearInterp {
    pub a: (f64, f64),
    pub b: (f64, f64),
}

impl Followable for LinearInterp {
    fn sample(&self, t: f64) -> (f64, f64) { (self.a.0 + t * (self.b.0 - self.a.0), self.a.1 + t * (self.b.1 - self.a.1)) }

    fn sample_heading(&self, _t: f64) -> f64 { (self.b.1 - self.a.1).atan2(self.b.0 - self.a.1) }
}

#[derive(Debug)]
pub(crate) struct CubicBezier {
    pub a: (f64, f64),
    pub b: (f64, f64),
    pub c: (f64, f64),
    pub d: (f64, f64),
}

impl Followable for CubicBezier {
    fn sample(&self, t: f64) -> (f64, f64) {
        let t2 = t * t;
        let t3 = t2 * t;
        (
            t3 * (-self.a.0 + 3.0 * self.b.0 - 3.0 * self.c.0 + self.d.0) + t2 * (3.0 * self.a.0 - 6.0 * self.b.0 + 3.0 * self.c.0) + t * (-3.0 * self.a.0 + 3.0 * self.b.0) + self.a.0,
            t3 * (-self.a.1 + 3.0 * self.b.1 - 3.0 * self.c.1 + self.d.1) + t2 * (3.0 * self.a.1 - 6.0 * self.b.1 + 3.0 * self.c.1) + t * (-3.0 * self.a.1 + 3.0 * self.b.1) + self.a.1,
        )
    }

    fn sample_heading(&self, t: f64) -> f64 {
        let t2 = t * t;
        (t2 * (-3.0 * self.a.1 + 9.0 * self.b.1 - 9.0 * self.c.1 + 3.0 * self.d.1) + t * (6.0 * self.a.1 - 12.0 * self.b.1 + 6.0 * self.c.1) - 3.0 * self.a.1 + 3.0 * self.b.1)
            .atan2(t2 * (-3.0 * self.a.0 + 9.0 * self.b.0 - 9.0 * self.c.0 + 3.0 * self.d.0) + t * (6.0 * self.a.0 - 12.0 * self.b.0 + 6.0 * self.c.0) - 3.0 * self.a.0 + 3.0 * self.b.0)
    }
}

pub(crate) struct PathSegment {
    pub curve: Box<dyn Followable>,
    pub speed: SpeedCurve,
    pub end_heading: f64,
    pub reversed_drive: bool,
    pub timeout: f64,
    pub wait_time: f64,
}

#[allow(unused)]
pub(crate) enum Action {
    ToggleMatchload,
    ToggleDescore,
    SpinIntake(f64),
    StopIntake,
    SpinIndexer(f64),
    StopIndexer,
    ResetPos(f64, f64, f64),
}

pub(crate) struct Auto {
    pub start_pose: (f64, f64, f64),
    pub spline: Vec<PathSegment> = vec![],
    pub spline_t: f64 = 0.0,
    pub current_curve: usize = 0,
    pub actions: Vec<(Action, f64)> = vec![],
    pub current_action: usize = 0,
    pub timeout_start: Instant
}

impl Auto {
    pub fn new() -> Self {
        Self {
            start_pose: (0.0, 0.0, 0.0),
            spline: vec![],
            spline_t: 0.0,
            current_curve: 0,
            actions: vec![],
            current_action: 0,
            timeout_start: Instant::now(),
        }
    }

    pub fn add_curves(&mut self, mut curves: Vec<PathSegment>) { self.spline.append(&mut curves); }

    pub fn add_actions(&mut self, mut actions: Vec<(Action, f64)>) { self.actions.append(&mut actions); }

    pub fn reset_state(&mut self) {
        self.spline_t = 0.0;
        self.current_curve = 0;
        self.current_action = 0;
        self.timeout_start = Instant::now();
    }

    fn sample(&self, t: f64) -> (f64, f64) {
        let segment: &PathSegment = &self.spline[(self.spline.len() - 1 - t.floor() as usize).min(self.spline.len() - 1)];
        segment.curve.sample(t.fract())
    }

    fn sample_heading(&self, t: f64) -> f64 {
        let segment: &PathSegment = &self.spline[(self.spline.len() - 1 - t.floor() as usize).min(self.spline.len() - 1)];
        segment.curve.sample_heading(t.fract())
    }

    fn sample_speed(&self, t: f64) -> f64 {
        let segment: &PathSegment = &self.spline[(self.spline.len() - 1 - t.floor() as usize).min(self.spline.len() - 1)];
        segment.speed.sample(t.fract()) * if segment.reversed_drive { -1.0 } else { 1.0 }
    }

    fn closest_point(&mut self, pos: &(f64, f64)) -> f64 {
        let mut closest = self.spline_t;
        let mut closest_dist = (self.sample(closest).0 - pos.0).hypot(self.sample(closest).1 - pos.1);
        for i in -5..=10 {
            let t_offset = i as f64 / 100.0;
            let sample_point = self.sample(self.spline_t + t_offset);
            let dist = (sample_point.0 - pos.0).hypot(sample_point.1 - pos.1);
            if dist <= closest_dist {
                closest = self.spline_t + t_offset;
                closest_dist = dist;
            }
        }
        self.spline_t = closest;
        closest_dist
    }

    pub fn get_timeout(&self) -> f64 {
        let segment: &PathSegment = &self.spline[(self.spline.len() - 1 - self.spline_t.floor() as usize).min(self.spline.len() - 1)];
        segment.timeout + segment.wait_time
    }
}

#[derive(Default, Debug)]
pub(crate) struct Chassis {
    pub linear: Pid,
    pub angular: Pid,
    pub k: f64 = 1.0,
    pub pose: Arc<RwLock<Pose>>,
    pub last_path_vel: f64 = 0.0
}

impl Chassis {
    pub fn new(linear: Pid, angular: Pid, k: f64, pose: Arc<RwLock<Pose>>) -> Self { Self { linear, angular, k, pose, ..Default::default() } }

    pub fn stanley(&mut self, pose: &(f64, f64, f64), auto: &mut Auto, efa: f64) -> (f64, f64) {
        let theta_e = pose.2 - auto.sample_heading(auto.spline_t);
        let path_vel = auto.sample_speed(auto.spline_t);
        let sigma = theta_e + (self.k * efa / path_vel).atan();
        (path_vel, sigma)
    }

    pub fn move_to_point(&mut self, pose: &(f64, f64, f64), auto: &mut Auto) -> (f64, f64) {
        if (pose.2 - auto.sample_heading(auto.spline_t)).abs() > 0.5 {
            (0.0, pose.2 - auto.sample_heading(auto.spline_t))
        } else {
            (auto.sample_speed(auto.spline_t), (auto.sample_heading(auto.spline_t) - pose.2))
        }
    }

    pub fn update(&mut self, auto: &mut Auto) -> (f64, f64) {
        let pose = self.pose.read().pose;
        let efa = auto.closest_point(&(pose.0, pose.1));

        if (auto.spline_t - auto.spline_t.floor()).abs() < 0.005 {
            let angular = self.angular.update(pose.2, auto.spline[auto.spline_t.floor() as usize].end_heading);
            let unorm_vel = (angular, -angular);
            return norm(unorm_vel, mag(unorm_vel));
        }

        let targets = if (*auto.spline[auto.spline_t.floor() as usize].curve).type_id() == TypeId::of::<LinearInterp>() {
            self.move_to_point(&pose, auto)
        } else {
            self.stanley(&pose, auto, efa)
        };
        let linear = self.linear.update(self.last_path_vel, targets.0);
        self.last_path_vel = targets.0;
        let angular = self.angular.update(pose.2, targets.1);
        let unorm_vel = (linear + angular, linear - angular);
        norm(unorm_vel, targets.0 * Motor::V5_MAX_VOLTAGE)
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
        self.angular.reset();
    }
}
