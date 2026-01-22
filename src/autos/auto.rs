use core::f64;
use std::{
    fmt::Debug,
    time::Instant,
    vec::Vec,
};

use crate::autos::{
        chassis::Chassis,
        path::{LinearInterp, PathSegment, SpeedCurve},
    };

#[allow(dead_code)]
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Autos {
    LeftQual,
    RightQual,
    Solo,
    LeftElims,
    RightElims,
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
    pub last_update: Instant,
    pub waiting: bool = false,
    pub close: bool = false,
    pub exit_state: u8 = 0,
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
            last_update: Instant::now(),
            waiting: false,
            close: false,
            exit_state: 0,
        }
    }

    pub fn add_curves(&mut self, mut curves: Vec<PathSegment>) { self.spline.append(&mut curves); }

    pub fn _add_actions(&mut self, mut actions: Vec<(Action, f64)>) { self.actions.append(&mut actions); }

    pub fn move_to_pose(&mut self, pose: (f64, f64, f64), speed: f64) {
        let start_pos = if self.spline.is_empty() {
            (self.start_pose.0, self.start_pose.1)
        } else {
            self.spline.last().unwrap().curve.sample(1.0)
        };
        let curve = PathSegment {
            curve: LinearInterp::new(start_pos, (pose.0, pose.1)),
            end_heading: pose.2,
            speed: SpeedCurve::new_linear(speed, speed),
            ..Default::default()
        };
        self.spline.push(curve);
    }

    pub fn move_to_pose_reverse(&mut self, pose: (f64, f64, f64), speed: f64) {
        let start_pos = if self.spline.is_empty() {
            (self.start_pose.0, self.start_pose.1)
        } else {
            self.spline.last().unwrap().curve.sample(1.0)
        };
        let curve = PathSegment {
            curve: LinearInterp::new(start_pos, (pose.0, pose.1)),
            end_heading: pose.2,
            speed: SpeedCurve::new_linear(speed, speed),
            reversed_drive: true,
            ..Default::default()
        };
        self.spline.push(curve);
    }

    pub fn add_action(&mut self, action: Action, time: f64) { self.actions.push((action, time)); }

    pub fn wait_for(&mut self, time: f64) {
        let pos = if self.spline.is_empty() {
            (self.start_pose.0, self.start_pose.1)
        } else {
            self.spline.last().unwrap().curve.sample(1.0)
        };
        let heading = if self.spline.is_empty() { self.start_pose.2 } else { self.spline.last().unwrap().end_heading };
        let curve = PathSegment {
            curve: LinearInterp::new(pos, pos),
            end_heading: heading,
            timeout: 0.0,
            wait_time: time,
            ..Default::default()
        };
        self.spline.push(curve);
    }

    pub fn reset_state(&mut self) {
        self.spline_t = 0.0;
        self.current_curve = 0;
        self.current_action = 0;
        self.timeout_start = Instant::now();
        self.wait_start = Instant::now();
        self.waiting = false;
    }

    fn get_curve(&self, t: f64) -> &PathSegment { &self.spline[(t.floor() as usize).min(self.spline.len() - 1)] }

    fn sample(&self, t: f64) -> (f64, f64) {
        self.get_curve(t).curve.sample(t.fract())
    }

    fn sample_heading(&self, t: f64) -> f64 {
        self.get_curve(t).curve.sample_heading(t.fract())
    }

    fn sample_speed(&self, t: f64) -> f64 {
        self.get_curve(t).speed.sample(t.fract()) * if self.get_curve(t).reversed_drive { -1.0 } else { 1.0 }
    }

    fn closest_point(&mut self, pos: &(f64, f64)) -> f64 {
        let mut closest = self.spline_t.max(self.current_curve as f64);
        let mut closest_dist = (self.sample(closest).0 - pos.0).hypot(self.sample(closest).1 - pos.1);
        for i in -6..=25 {
            let t_offset = i as f64 / 250.0;
            let sample_point = self.sample(self.spline_t.max(self.current_curve as f64) + t_offset);
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
        self.get_curve(self.spline_t).timeout
    }

    pub fn get_wait(&self) -> f64 {
        self.get_curve(self.spline_t).wait_time
    }
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 { (b.0 - a.0).hypot(b.1 - a.1) }

fn desaturate(p: (f64, f64)) -> (f64, f64) {
    let left = p.0 - p.1;
    let right = p.0 + p.1;
    let sum = left.abs() + right.abs();
    if sum <= 1.0 {
        (left, right)
    } else {
        (left / sum, right / sum)
    }
}

impl Chassis {
    pub fn update(&mut self, auto: &mut Auto) -> (f64, f64) {
        let pose = self.pose.read().pose;
        let efa = auto.closest_point(&(pose.0, pose.1));
        
        if auto.spline_t % 1.0 > 0.975 || auto.exit_state == 1 {
            let min_angular = if !auto.get_curve(auto.spline_t).chained { 0.0 } else { auto.get_curve(auto.spline_t).speed.min() };
            let max_angular = auto.get_curve(auto.spline_t).speed.sample(auto.spline_t);
            let angular_err = (pose.2 - auto.get_curve(auto.spline_t).end_heading.to_radians()) % f64::consts::TAU;
            let mut angular = self.angular.update(angular_err);
            if angular.abs() < 0.1 && angular.signum() != self.last_angular_out.signum() { auto.exit_state = 2; angular = 0.0 };
            if angular_err.abs() < auto.get_curve(auto.spline_t).end_heading_err.to_radians() && auto.get_curve(auto.spline_t).chained { auto.exit_state = 2; angular = 0.0 };
            if (angular - self.last_angular_out).abs() > (self.angular.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0).abs() {
                angular = self.last_angular_out + (self.angular.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0 * (angular - self.last_angular_out).signum());
            }
            if angular.abs() < min_angular { angular = angular.signum() * min_angular; }
            angular = angular.clamp(-max_angular, max_angular);
            self.last_linear_out = 0.0;
            self.last_angular_out = angular;
            return desaturate((angular, -angular));
        }
        
        if auto.get_curve(auto.spline_t).curve.curve_type() == 0 {
            let mut max_linear = auto.get_curve(auto.spline_t).speed.sample(auto.spline_t);
            let min_linear = if auto.get_curve(auto.spline_t).chained { auto.get_curve(auto.spline_t).speed.min() } else { 0.0 };
            let mut max_angular = auto.get_curve(auto.spline_t).speed.sample(auto.spline_t);
            
            if !auto.close && distance((pose.0, pose.1), auto.sample(auto.spline_t.ceil())) < 7.5 && !auto.get_curve(auto.spline_t).chained {
                auto.close = true;
                max_linear = self.last_linear_out.abs().max(4.7);
                max_angular = self.last_angular_out.abs().max(4.7);
            }
            
            let angular_err = (pose.2 - auto.sample_heading(auto.spline_t) + if auto.get_curve(auto.spline_t).reversed_drive { 180.0 } else { 0.0 }) % f64::consts::TAU;
            let linear_err = distance((pose.0, pose.1), auto.sample(auto.spline_t.floor() + 0.99)) * ((pose.2 - auto.sample_heading(auto.spline_t)) % f64::consts::TAU).cos();
            
            if self.linear.update_timeouts(linear_err) ||
            self.last_linear_out.signum() < linear_err.signum() * 0.01 { auto.exit_state = 1; return (0.0, 0.0); }
            
            let mut linear_out = self.linear.update(linear_err / 39.37);
            linear_out = linear_out.clamp(-max_linear, max_linear);
            linear_out = if auto.close {
                linear_out
            } else if (linear_out - self.last_linear_out).abs() > (self.linear.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0).abs() {
                self.last_linear_out + (self.linear.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0 * (linear_out - self.last_linear_out).signum())
            } else {
                linear_out
            };

            linear_out = if auto.close && !auto.get_curve(auto.spline_t).chained {
                linear_out
            } else if auto.get_curve(auto.spline_t).reversed_drive {
                linear_out.clamp(-min_linear, -max_linear)
            } else {
                linear_out.clamp(min_linear, max_linear)
            };
            
            let mut angular_out = if !auto.close { self.angular.update(angular_err / 39.37) } else { 0.0 };
            angular_out = angular_out.clamp(-max_angular, max_angular).to_radians() % f64::consts::TAU;
            angular_out = if auto.close {
                angular_out
            } else if (angular_out - self.last_angular_out).abs() > (self.angular.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0).abs() {
                self.last_angular_out + (self.angular.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0 * (angular_out - self.last_angular_out).signum())
            } else {
                angular_out
            };
            
            self.last_linear_out = linear_out;
            self.last_angular_out = angular_out;

            auto.last_update = Instant::now();
            
            desaturate((linear_out, angular_out))
        } else {
            let theta_e = pose.2 - auto.sample_heading(auto.spline_t);
            let path_vel = auto.sample_speed(auto.spline_t);
            let sigma = theta_e + (self.k * efa / path_vel).atan();

            let mut max_linear = auto.get_curve(auto.spline_t).speed.sample(auto.spline_t);
            let min_linear = if auto.get_curve(auto.spline_t).chained { auto.get_curve(auto.spline_t).speed.min() } else { 0.0 };
            let mut max_angular = auto.get_curve(auto.spline_t).speed.sample(auto.spline_t);
            
            if !auto.close && distance((pose.0, pose.1), auto.sample(auto.spline_t.ceil())) < 7.5 && !auto.get_curve(auto.spline_t).chained {
                auto.close = true;
                max_linear = self.last_linear_out.abs().max(4.7);
                max_angular = self.last_angular_out.abs().max(4.7);
            }

            let linear_err = path_vel * (sigma % f64::consts::TAU).cos();
            let angular_err = sigma % f64::consts::TAU;

            if self.linear.update_timeouts(linear_err) ||
            self.last_linear_out.signum() != linear_err.signum() { return (0.0, 0.0); }
            
            let mut linear_out = self.linear.update(linear_err / 39.37);
            linear_out = linear_out.clamp(-max_linear, max_linear);
            linear_out = if auto.close {
                linear_out
            } else if (linear_out - self.last_linear_out).abs() > (self.linear.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0).abs() {
                self.last_linear_out + (self.linear.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0 * (linear_out - self.last_linear_out).signum())
            } else {
                linear_out
            };

            linear_out = if auto.close && !auto.get_curve(auto.spline_t).chained {
                linear_out
            } else if auto.get_curve(auto.spline_t).reversed_drive {
                linear_out.clamp(-min_linear, -max_linear)
            } else {
                linear_out.clamp(min_linear, max_linear)
            };
            
            let mut angular_out = if !auto.close { self.linear.update(angular_err / 39.37) } else { 0.0 };
            angular_out = angular_out.clamp(-max_angular, max_angular).to_radians() % f64::consts::TAU;
            angular_out = if auto.close {
                angular_out
            } else if (angular_out - self.last_angular_out).abs() > (self.linear.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0).abs() {
                self.last_angular_out + (self.linear.slew * (auto.last_update.elapsed().as_millis() as f64) / 1000.0 * (angular_out - self.last_angular_out).signum())
            } else {
                angular_out
            };
            
            self.last_linear_out = linear_out;
            self.last_angular_out = angular_out;

            auto.last_update = Instant::now();

            desaturate((linear_out, angular_out))
        }
    }
}
