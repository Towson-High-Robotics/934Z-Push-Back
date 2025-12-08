use core::f64;
use std::{
    any::{Any, TypeId},
    fmt::Debug,
    time::Instant,
    vec::Vec,
};

use vexide::prelude::Motor;

use crate::{
    autos::{
        chassis::Chassis,
        path::{LinearInterp, PathSegment},
    },
    util::norm,
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

#[allow(unused)]
#[derive(Debug)]
pub(crate) enum Action {
    ToggleMatchload,
    ToggleDescore,
    SpinIntake(f64),
    StopIntake,
    SpinIndexer(f64),
    StopIndexer,
    ResetPose(f64, f64, f64),
}

#[derive(Debug)]
pub(crate) struct Auto {
    pub start_pose: (f64, f64, f64),
    pub spline: Vec<PathSegment> = vec![],
    pub spline_t: f64 = 0.0,
    pub current_curve: usize = 0,
    pub actions: Vec<(Action, f64)> = vec![],
    pub current_action: usize = 0,
    pub timeout_start: Instant,
    pub wait_start: Instant,
    pub waiting: bool = false
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
            wait_start: Instant::now(),
            waiting: false
        }
    }

    pub fn add_curves(&mut self, mut curves: Vec<PathSegment>) { self.spline.append(&mut curves); }

    pub fn add_actions(&mut self, mut actions: Vec<(Action, f64)>) { self.actions.append(&mut actions); }

    pub fn reset_state(&mut self) {
        self.spline_t = 0.0;
        self.current_curve = 0;
        self.current_action = 0;
        self.timeout_start = Instant::now();
        self.wait_start = Instant::now();
        self.waiting = false;
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
        segment.timeout
    }

    pub fn get_wait(&self) -> f64 {
        let segment: &PathSegment = &self.spline[(self.spline.len() - 1 - self.spline_t.floor() as usize).min(self.spline.len() - 1)];
        segment.wait_time
    }

    pub fn is_reversed(&self) -> bool {
        let segment: &PathSegment = &self.spline[(self.spline.len() - 1 - self.spline_t.floor() as usize).min(self.spline.len() - 1)];
        segment.reversed_drive
    }
}

impl Chassis {
    pub fn stanley(&mut self, pose: &(f64, f64, f64), auto: &mut Auto, efa: f64) -> (f64, f64) {
        let theta_e = pose.2 - auto.sample_heading(auto.spline_t);
        let path_vel = auto.sample_speed(auto.spline_t);
        let sigma = theta_e + (self.k * efa / path_vel).atan();
        (path_vel, sigma)
    }

    pub fn move_to_point(&mut self, pose: &(f64, f64, f64), auto: &mut Auto) -> (f64, f64) {
        if (pose.2 - auto.sample_heading(auto.spline_t)).abs() > 0.5 {
            (0.0, auto.sample_heading(auto.spline_t))
        } else {
            (auto.sample_speed(auto.spline_t), auto.sample_heading(auto.spline_t))
        }
    }

    pub fn update(&mut self, auto: &mut Auto) -> (f64, f64) {
        let pose = self.pose.read().pose;
        let efa = auto.closest_point(&(pose.0, pose.1));

        if (auto.spline_t - auto.spline_t.floor()).abs() < 0.1 {
            let angular = self.angular.update(pose.2, auto.spline[auto.spline_t.floor() as usize].end_heading.to_radians());
            let unorm_vel = (angular, -angular);
            return norm(unorm_vel, angular * Motor::V5_MAX_VOLTAGE);
        }

        let targets = if (*auto.spline[auto.spline_t.floor() as usize].curve).type_id() == TypeId::of::<LinearInterp>() {
            self.move_to_point(&pose, auto)
        } else {
            self.stanley(&pose, auto, efa)
        };
        let linear = self.linear.update(self.last_path_vel, targets.0);
        self.last_path_vel = targets.0;
        let angular = self.heading.update(pose.2, targets.1);
        let unorm_vel = (linear + angular, linear - angular);
        norm(unorm_vel, targets.0 * Motor::V5_MAX_VOLTAGE)
    }
}
