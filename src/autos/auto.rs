use core::f64;
use std::{fmt::Debug, time::Instant, vec::Vec};

use crate::{
    autos::{
        chassis::Chassis,
        path::{LinearInterp, PathSegment},
    }, log_debug, util::dot
};

/// Types of Autos that can be created/used
/// Can be a variant of Left, Right, Solo AWP, Skills or a misc Auto
#[allow(dead_code)]
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Autos {
    LeftQual,         // Goal: 9 Block Half AWP + wing
    LeftElims,        // Goal: 9 Block Long Goal + Wing
    LeftCounterQual,  // Goal: 7 Block Half AWP + Wing Rush
    LeftCounterElims, // Goal: 7 Block Long Goal + Wing Rush
    RightQual,        // Same as Left Side but on Right
    RightElims,
    RightCounterQual,
    RightCounterElims,
    Solo,         // Goal: 13 Block SAWP + Wing
    CounterSolo,  // Goal: 10 Block SAWP + Wing
    Skills,       // Goal 98-117 Point Skills
    SkillsDriver, // Either does nothing or runs the skills auto
    #[default]
    None,     // Drive forwards 12 inches and stop
    Recorded, // Use the most recently recorded auto
}

/// A miscellaneous action seperate from motion control
/// Includes motors, solenoids and position tracking
#[allow(unused)]
#[derive(Clone, Copy, Debug)]
pub(crate) enum Action {
    ToggleMatchload,
    ToggleDescore,
    SpinIntake(f64),
    StopIntake,
    SpinIndexer(f64),
    StopIndexer,
    ResetPose(f64, f64, f64),
    DistanceReset(u8),
}

/// The main Auto struct - holds all the information relevant to the
/// current state of the auto and the path and actions associated with the
/// auto \
/// Fields: \
///  `start_pose: (f64, f64, f64)` - holds the starting pose in x, y, theta \
///  `spline: Vec<PathSegment>` - holds the path the robot takes \
///  `spline_t: f64` (internal) - how far along has the robot traveled along the path \
///  `curve_t: f64` (internal) - how long along has the robot traveled along the current curve \
///  `current_curve` (internal) - what `PathSegment` is the robot on \
///  `actions: Vec<(Action, f64)>` - list of all the actions in the auto, at what times (in ms) \
///  `current_action: usize` (internal) - what action is the auto on \
///  `timeout_start: Instant` (internal) - when did the last motion start \
///  `wait_start: Instant` (internal) - when did the wait period for the last motion start \
///  `waiting: bool` (internal) - is the robot waiting in place or not \
///  `close: bool` (internal) - are we close to the end of the motion \
///  `exit_state: u8` (internal) - have we exited a curve or a heading correction motion or not
#[derive(Debug)]
pub(crate) struct Auto {
    pub start_pose: (f64, f64, f64) = (0.0, 0.0, 0.0),
    pub spline: Vec<PathSegment> = vec![],
    pub(crate) curve_t: f64 = 0.0,
    pub(crate) current_curve: usize = 0,
    pub actions: Vec<(Action, f64)> = vec![],
    pub current_action: usize = 0,
    pub motion_start: Instant,
    pub last_update: Instant,
    pub close: bool = false,
    pub exit_state: u8 = 0,
}

impl Auto {
    pub fn new() -> Self {
        Self {
            start_pose: (0.0, 0.0, 0.0),
            spline: vec![],
            curve_t: 0.0,
            current_curve: 0,
            actions: vec![],
            current_action: 0,
            motion_start: Instant::now(),
            last_update: Instant::now(),
            close: false,
            exit_state: 0,
        }
    }

    pub fn add_curves(&mut self, mut curves: Vec<PathSegment>) { self.spline.append(&mut curves); }

    pub fn add_actions(&mut self, mut actions: Vec<(Action, f64)>) { self.actions.append(&mut actions); }

    pub fn move_to_pose(&mut self, x: f64, y: f64, theta: f64) -> &mut PathSegment {
        let start_pos = if self.spline.is_empty() {
            (self.start_pose.0, self.start_pose.1)
        } else {
            self.spline.last().unwrap().curve.sample(1.0)
        };
        let curve = PathSegment {
            curve: LinearInterp::new(start_pos, (x, y)),
            end_heading: theta,
            force_stanley: false,
            ..Default::default()
        };
        self.spline.push(curve);
        self.spline.last_mut().unwrap()
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
        self.curve_t = 0.0;
        self.current_curve = 0;
        self.current_action = 0;
        self.motion_start = Instant::now();
    }

    fn cross_track_err(&mut self, pos: (f64, f64)) -> f64 {
        // Running t value as we try and find the closest point
        let mut t = self.curve_t;
        for _ in 0..5 {
            // Calculate the point on the curve at the current value of t
            let p = self.spline[self.current_curve].curve.sample(t);
            // Then on the derivative of the curve
            let dp = self.spline[self.current_curve].curve.sample_derivative(t);
            // And finally on the second derivative of the curve
            let ddp = self.spline[self.current_curve].curve.sample_derivative2(t);

            // Now calculate the error between the point and the robot
            let path_to_robot = (p.0 - pos.0, p.1 - pos.1);
            // And get the dot product between that and the derivative of the curve
            let f = dot(path_to_robot, dp);
            // Now get the derivative of that
            let f_prime = dot(dp, dp) + dot(path_to_robot, ddp);

            // If the derivative of our (kinda) error function is close to zero exit the
            // loop as the benefits of continuing diminish rapidly
            if f_prime.abs() < 1E-4 {
                break;
            }
            // Otherwise continue iterating
            t -= f / f_prime;
        }
        // Clamp our new t value to not overshoot
        t = t.clamp(self.curve_t - 0.1, self.curve_t + 0.1);
        // Store our new t value
        self.curve_t = t;

        // Get the closest point and derivative at closest point again
        let p = self.spline[self.current_curve].curve.sample(t);
        let dp = self.spline[self.current_curve].curve.sample_derivative(t);
        // Get the length of the tangent so we can normalize our values later on
        let len_dp = dp.0.hypot(dp.1).max(1E-6);
        // Get the error between the robot and our closest point
        let err = (pos.0 - p.0, pos.1 - p.1);
        // Return a signed cross track error based on the normal vector of the path
        err.0.hypot(err.1) * if dot(err, (-dp.1, dp.0)) / len_dp < 0.0 { -1.0 } else { 1.0 }
    }

    pub fn get_timeout(&self) -> f64 { self.spline[self.current_curve].timeout }

    pub fn get_wait(&self) -> f64 { self.spline[self.current_curve].wait_time }
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 { (b.0 - a.0).hypot(b.1 - a.1) }

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

impl Chassis {
    pub fn update(&mut self, auto: &mut Auto) -> (f64, f64) {
        let pose = self.pose.read().pose;
        let efa = auto.cross_track_err((pose.0, pose.1));
        let target_pos = auto.spline[auto.current_curve].curve.sample(1.0);
        let target_dist = distance((pose.0, pose.1), target_pos);

        if auto.exit_state >= 2 { return (0.0, 0.0); }

        if target_dist < 0.05 || auto.exit_state == 1 {
            // Minimum anglar velocity for chained motions, maximum angular velocity from
            // parameters
            let min_angular = if auto.spline[auto.current_curve].chained {
                auto.spline[auto.current_curve].min_speed.sample(1.0)
            } else {
                0.0
            };
            let mut max_angular = auto.spline[auto.current_curve].max_speed.sample(1.0);

            // Target heading based on the unit circle instead of path.jerryio's units
            let target_heading = (auto.spline[auto.current_curve].end_heading).to_radians().rem_euclid(f64::consts::TAU);
            // Angular error normalized between [-pi, pi]
            let mut angular_err = (pose.2 - target_heading).rem_euclid(f64::consts::TAU);
            if angular_err > f64::consts::PI {
                angular_err -= f64::consts::TAU;
            }
            // Force a lower maximum angular PID value if we are 20 degrees from the target
            // and not chaining motions
            if angular_err.abs() <= (90.0_f64).to_radians() && !auto.spline[auto.current_curve].chained {
                max_angular = self.last_angular_out.abs().max(0.05);
            }
            
            // Get the angular PID value based on our normalized error
            let mut angular = self.angular.update(angular_err);
            // Early exit if our angular error is small enough
            if angular_err.abs() <= (0.25_f64).to_radians() && !auto.spline[auto.current_curve].chained {
                auto.exit_state = 2;
                angular = 0.0;
            };
            // Use a larger (user defined) early exit parameter if we are chaining motions
            if angular_err.abs() <= auto.spline[auto.current_curve].end_heading_err.to_radians() && auto.spline[auto.current_curve].chained {
                auto.exit_state = 2;
                angular = 0.0;
            };

            // Calculate delta time for slew, limit it to a minimum of 100 microseconds if
            // it's too small
            let dt = auto.last_update.elapsed().as_secs_f64().max(1E-4);
            // Make sure that our angular PID respects our defined slew value
            if (angular - self.last_angular_out).abs() > (self.angular.slew * dt).abs() {
                angular = self.last_angular_out + (self.angular.slew * dt * (angular - self.last_angular_out).signum());
            }

            // Clamp our angular PID within our maximum angular PID value
            angular = angular.clamp(-max_angular, max_angular);
            // If we aren't at our target and we are chaining motions then force our angular
            // PID to the minimum angular PID value
            if angular.abs() < min_angular && auto.spline[auto.current_curve].chained {
                angular = angular_err.signum() * min_angular;
            }

            // Update the last angular PID value and force the last linear PID value to 0.0
            // since we aren't moving linearly
            self.last_linear_out = 0.0;
            self.last_angular_out = angular;

            // Update delta time for next time
            auto.last_update = Instant::now();

            // Output a combination of linear and angular PID that respects the maxmimum
            // motor voltage
            return desaturate((0.0, angular));
        }

        if auto.spline[auto.current_curve].curve.curve_type() == 0 && !auto.spline[auto.current_curve].force_stanley {
            // Maximum linear PID value; Based on arguments and settling distance
            let mut max_linear = auto.spline[auto.current_curve].max_speed.sample(auto.curve_t);
            // Minimum linear PID value; Only used during a motion chain
            let min_linear = if auto.spline[auto.current_curve].chained {
                auto.spline[auto.current_curve].min_speed.sample(auto.curve_t)
            } else {
                0.0
            };
            // Maximum angular PID value; Based on arguments and settling distance once
            // again
            let mut max_angular = auto.spline[auto.current_curve].max_speed.sample(auto.curve_t);

            // If the robot is close to the target point, lower the maximum linear and
            // angular PID values Don't settle if we are chaining the motion
            if target_dist < 4.0 && !auto.spline[auto.current_curve].chained {
                auto.close = true;
                max_linear = self.last_linear_out.abs().max(4.7);
                max_angular = self.last_angular_out.abs().max(4.7);
            }

            // Angular error in radians, if we are going in reverse flip it by 180 degrees
            // (PI radians), then normalize between [-pi, pi]
            let target_heading = (-(target_pos.1 - pose.1).atan2(target_pos.0 - pose.0) + f64::consts::FRAC_PI_2).rem_euclid(f64::consts::TAU);
            let mut angular_err =
                (pose.2 - target_heading + if auto.spline[auto.current_curve].reversed_drive { f64::consts::PI } else { 0.0 }).rem_euclid(f64::consts::TAU);
            if angular_err > f64::consts::PI {
                angular_err -= f64::consts::TAU
            };
            // Linear error in inches, distance between the robot's position and the target
            // position
            let mut linear_err = target_dist;

            // Get the forward vector of the robot and the vector between the robot's
            // position and the target position
            let forward_vector = (pose.2.cos(), pose.2.sin());
            let target_to_robot_vector = (pose.0 - target_pos.0, pose.1 - target_pos.1);
            // Use a dot product to compare the forwards vector and the target to robot
            // vector If we cross the target, the dot product should be positive
            // and we can exit If we are reversed, flip the value to represent
            // the 180 deg flip in rotation of the robot
            let side = dot(forward_vector, target_to_robot_vector) * if auto.spline[auto.current_curve].reversed_drive { -1.0 } else { 1.0 };
            // Exit the loop if either the timeouts expire or we go past the target point
            if self.linear.update_timeouts(linear_err) || (side > 0.0 && target_dist < 0.2) {
                auto.exit_state = 1;
                return (0.0, 0.0);
            };

            // Scale the linear error by the cosine of the angular error so that we can get
            // to our target heading without spiraling into our target point
            // Use the forward angular error so that we can get reverse motion for free
            let cos_err = ((pose.2 - target_heading).rem_euclid(f64::consts::TAU)).cos();
            linear_err *= cos_err.abs().max(0.01) * cos_err.signum();

            // Calculate delta time for slew calculations, clamp it to a minimum of 100
            // microseconds if it gets too small Should be around 25 ms (main
            // loop update speed)
            let dt = auto.last_update.elapsed().as_secs_f64().max(1E-4); // 1E-4 is 100 microseconds

            // Get the linear PID value using the linear error converted to meters
            let mut linear_out = self.linear.update(linear_err / 39.37);
            // Clamp linear PID value to the max error
            linear_out = linear_out.clamp(-max_linear, max_linear);
            // If we are accelerating or deccelerating too fast clamp the change in linear
            // PID to the slew Ignore slew if we are settling to the target
            linear_out = if auto.close {
                linear_out
            } else if (linear_out - self.last_linear_out).abs() > (self.linear.slew * dt).abs() {
                self.last_linear_out + (self.linear.slew * dt * (linear_out - self.last_linear_out).signum())
            } else {
                linear_out
            };

            // If the motion is chained, then clamp the lower bound of the linear PID to the
            // minimum linear PID value we set earlier
            linear_out = if auto.close && !auto.spline[auto.current_curve].chained {
                linear_out
            } else if auto.spline[auto.current_curve].reversed_drive {
                linear_out.clamp(-max_linear, -min_linear)
            } else {
                linear_out.clamp(min_linear, max_linear)
            };

            // Same deal as the linear PID but for angular, ignore angular PID if we are
            // setting so that the robot can settle faster
            let mut angular_out = if !auto.close { self.angular.update(angular_err) } else { 0.0 };
            angular_out = angular_out.clamp(-max_angular, max_angular);
            angular_out = if auto.close {
                angular_out
            } else if (angular_out - self.last_angular_out).abs() > (self.angular.slew * dt).abs() {
                self.last_angular_out + (self.angular.slew * dt * (angular_out - self.last_angular_out).signum())
            } else {
                angular_out
            };

            // Update the last linear and angular PID values
            self.last_linear_out = linear_out;
            self.last_angular_out = angular_out;

            // Update delta time
            auto.last_update = Instant::now();

            // Return a combination of the linear and angular PID that respect maximum motor
            // voltage
            desaturate((linear_out, angular_out))
        } else {
            let mut theta_e =
                (pose.2 - (auto.spline[auto.current_curve].curve.sample_heading(auto.curve_t) + if auto.spline[auto.current_curve].reversed_drive { f64::consts::PI } else { 0.0 }).rem_euclid(f64::consts::TAU)).rem_euclid(f64::consts::TAU);
            if theta_e > f64::consts::PI {
                theta_e -= f64::consts::TAU;
            }
            let mut path_vel = auto.spline[auto.current_curve].min_speed.sample(auto.curve_t).midpoint(auto.spline[auto.current_curve].max_speed.sample(auto.curve_t));
            let mut sigma = if path_vel == 0.0 { theta_e } else { theta_e - (self.k * efa / path_vel).atan() };
            sigma = sigma.rem_euclid(f64::consts::TAU);
            if sigma > f64::consts::PI {
                sigma -= f64::consts::TAU;
            }

            let mut max_linear = auto.spline[auto.current_curve].max_speed.sample(auto.curve_t);
            let min_linear = if auto.spline[auto.current_curve].chained { auto.spline[auto.current_curve].min_speed.sample(auto.curve_t) } else { 0.0 };
            let mut max_angular = auto.spline[auto.current_curve].max_speed.sample(auto.curve_t);

            if target_dist < 7.5 && !auto.spline[auto.current_curve].chained {
                auto.close = true;
                max_linear = self.last_linear_out.abs().max(4.7);
                max_angular = self.last_angular_out.abs().max(4.7);
            }

            let forward_vector = (pose.2.cos(), pose.2.sin());
            let target_to_robot_vector = (pose.0 - target_pos.0, pose.1 - target_pos.1);
            let side = dot(forward_vector, target_to_robot_vector) * if auto.spline[auto.current_curve].reversed_drive { -1.0 } else { 1.0 };

            if self.linear.update_timeouts(target_dist) || (side > 0.0 && target_dist < 0.2) {
                auto.exit_state = 1;
                return (0.0, 0.0);
            }

            let cos_err = ((pose.2 - auto.spline[auto.current_curve].curve.sample_heading(auto.curve_t)).rem_euclid(f64::consts::TAU)).cos();
            path_vel *= cos_err.abs().max(0.01) * cos_err.signum();

            let dt = auto.last_update.elapsed().as_secs_f64().max(1E-4);

            let mut linear_out = self.linear.update(path_vel);
            linear_out = linear_out.clamp(-max_linear, max_linear);
            linear_out = if auto.close {
                linear_out
            } else if (linear_out - self.last_linear_out).abs() > (self.linear.slew * dt).abs() {
                self.last_linear_out + (self.linear.slew * dt * (linear_out - self.last_linear_out).signum())
            } else {
                linear_out
            };

            linear_out = if auto.close && !auto.spline[auto.current_curve].chained {
                linear_out
            } else if auto.spline[auto.current_curve].reversed_drive {
                linear_out.clamp(-max_linear, -min_linear)
            } else {
                linear_out.clamp(min_linear, max_linear)
            };

            let mut angular_out = self.angular.update(sigma) * if auto.close { 0.25 } else { 1.0 };
            angular_out = angular_out.clamp(-max_angular, max_angular);
            angular_out = if auto.close {
                angular_out
            } else if (angular_out - self.last_angular_out).abs() > (self.angular.slew * dt).abs() {
                self.last_angular_out + (self.angular.slew * dt * (angular_out - self.last_angular_out).signum())
            } else {
                angular_out
            };

            log_debug!("{linear_out:.2}, {angular_out:.2}, {}", auto.close);

            self.last_linear_out = linear_out;
            self.last_angular_out = angular_out;

            auto.last_update = Instant::now();

            desaturate((linear_out, angular_out))
        }
    }
}
