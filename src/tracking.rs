use core::f64;
use std::{
    ops::Rem,
    sync::{Arc, nonpoison::RwLock},
    time::{Duration, Instant},
};

use vexide::{competition::{CompetitionStatus, status}, math::Angle, peripherals::DynamicPeripherals, prelude::*};

use crate::{
    log_error, log_info, log_warn, telemetry::Telem, util::{Drivetrain, TrackingWheel}
};

#[derive(Debug)]
pub(crate) struct TrackingSensors {
    horizontal_track: TrackingWheel,
    vertical_track: TrackingWheel,
    imu: InertialSensor,
    distance_left: (DistanceSensor, f64, f64),
    distance_right: (DistanceSensor, f64, f64),
    distance_front: (DistanceSensor, f64, f64),
}

impl TrackingSensors {
    pub fn new(per: &mut DynamicPeripherals, ports: [u8; 6], offsets: [f64; 5], angles: [f64; 3], reversed: [bool; 2]) -> Self {
        // Create objects for the sensors
        let imu = InertialSensor::new(per.take_smart_port(ports[0]).expect("IMU port not set"));
        
        let hor_rot_sens = RotationSensor::new(
            per.take_smart_port(ports[1]).expect("Horizontal tracking wheel sensor port not set"),
            if reversed[0] { Direction::Reverse } else { Direction::Forward },
        );

        let vert_rot_sens = RotationSensor::new(
            per.take_smart_port(ports[2]).expect("Vertical tracking wheel sensor port not set"),
            if reversed[1] { Direction::Reverse } else { Direction::Forward },
        );

        let dist_left = DistanceSensor::new(per.take_smart_port(ports[3]).unwrap());
        let dist_right = DistanceSensor::new(per.take_smart_port(ports[4]).unwrap());
        let dist_front = DistanceSensor::new(per.take_smart_port(ports[5]).unwrap());
        
        TrackingSensors {
            imu,
            horizontal_track: TrackingWheel { sens: hor_rot_sens, offset: offsets[0] },
            vertical_track: TrackingWheel { sens: vert_rot_sens, offset: offsets[1] },
            distance_left: (dist_left, offsets[2], angles[0]),
            distance_right: (dist_right, offsets[3], angles[1]),
            distance_front: (dist_front, offsets[4], angles[2])
        }
    }
}

#[derive(Debug)]
pub(crate) struct Tracking {
    last_tick: Instant,
    drive: Arc<RwLock<Drivetrain>>,
    sensors: TrackingSensors,
    imu_calibrated: bool,
    telem: Arc<RwLock<Telem>>,
    pub(crate) pose: (f64, f64, f64),
    start_heading: f64,
    delta_pose: (f64, f64),
    h0: f64,
    v0: f64,
    l0: f64,
    r0: f64,
    dist_vals: (f64, f64, f64)
}

impl Tracking {
    pub fn new(sensors: TrackingSensors, telem: Arc<RwLock<Telem>>, drive: Arc<RwLock<Drivetrain>>) -> Tracking {
        // And return the struct
        Tracking {
            telem,
            drive,
            sensors,
            imu_calibrated: false,
            last_tick: Instant::now(),
            pose: (0.0, 0.0, 0.0),
            start_heading: 0.0,
            delta_pose: (0.0, 0.0),
            h0: 0.0,
            v0: 0.0,
            l0: 0.0,
            r0: 0.0,
            dist_vals: (0.0, 0.0, 0.0)
        }
    }

    pub async fn calibrate_imu(&mut self) {
        // Exit if the IMU isn't plugged in
        if !self.sensors.imu.is_connected() {
            log_warn!("IMU isn't connected, couldn't calibrate");
            self.imu_calibrated = false;
            return;
        }

        // Attempt to calibrate the IMU twice
        match self.sensors.imu.calibrate().await {
            Ok(_) => log_info!("IMU successfully calibrated :D"),
            Err(e) => {
                log_warn!("IMU failed to calibrate :(\nError:\n{e:?}");
                if self.sensors.imu.calibrate().await.is_err() {
                    log_error!("IMU failed to calibrate (again) >:(");
                    self.imu_calibrated = false;
                }
            }
        }
        self.imu_calibrated = true;
    }

    pub fn reset_pose(&mut self, reset_pose: (f64, f64, f64)) {
        // Set the new pose
        self.pose = reset_pose;
        self.start_heading = reset_pose.2;

        // Reset sensors
        self.sensors.horizontal_track.sens.reset_position().ok();
        self.sensors.vertical_track.sens.reset_position().ok();
        self.sensors.imu.set_heading(Angle::from_degrees((-reset_pose.2 + 90.0).rem(360.0))).ok();

        // Reset IMEs for odometry
        self.drive.write().left_motors.iter_mut().for_each(|m| {
            m.reset_position().ok();
        });
        self.drive.write().right_motors.iter_mut().for_each(|m| {
            m.reset_position().ok();
        });

        // Reset some odom-specific values
        self.delta_pose = (0.0, 0.0);
        self.h0 = 0.0;
        self.v0 = 0.0;
        self.l0 = 0.0; self.r0 = 0.0;
    }

    pub async fn calibrate(&mut self, reset_pose: (f64, f64, f64)) {
        self.reset_pose(reset_pose);
        self.calibrate_imu().await;
    }

    pub fn odom_tick(&mut self, l1: f64, r1: f64) {
        // Fall back to IME heading if the IMU is dc'ed / uncalibrated
        let heading = if self.imu_calibrated {
            self.sensors.imu.heading().unwrap().as_radians().rem_euclid(f64::consts::TAU)
        } else {
            ((((l1 - self.l0) * 1.21875) - ((r1 - self.r0) * 1.21875)) / 10.37 + self.pose.2).rem_euclid(f64::consts::TAU)
        };
        // Get delta theta and LAO
        let mut delta_theta = (heading - self.pose.2).rem_euclid(f64::consts::TAU);
        if delta_theta > f64::consts::PI { delta_theta -= f64::consts::TAU };
        let lao = self.pose.2 + delta_theta / 2.0;

        // Vertical displacement, fall back to IMEs if no vert wheel
        let delta_dly = if self.sensors.vertical_track.sens.is_connected() {
            // Vertical wheel travel
            let v1 = self.sensors.vertical_track.sens.angle().unwrap_or_default().as_radians();
            let delta_v = v1 - self.v0;
            self.v0 = v1;

            // Mapped to an arc if the change in angle is 0
            if delta_theta != 0.0 {
                2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta + self.sensors.vertical_track.offset)
            } else {
                delta_v
            }
        } else {
            // Emulate vertical wheel using averages
            // 0.609375 = 0.5 (for averaging) * 1.625 (wheel radius) * 36/48 (gear ratio)
            let v0 = (self.l0 + self.r0) * 0.609375;
            let v1 = (l1 + r1) * 0.609375;
            let delta_v = v1 - v0;

            // Now map it to an arc if angle change is 0
            if delta_theta != 0.0 {
                2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta)
            } else {
                delta_v
            }
        };

        // Horizontal displacement, return 0 if no horizontal wheel
        let delta_dlx = if self.sensors.horizontal_track.sens.is_connected() {
            // Horizontal wheel displacement
            let h1 = self.sensors.horizontal_track.sens.angle().unwrap_or_default().as_radians() * 2.00;
            let delta_h = h1 - self.h0;
            self.h0 = h1;

            // Map it to an arc if there's an angle change
            if delta_theta != 0.0 {
                2.0 * (delta_theta / 2.0).sin() * (delta_h / delta_theta + self.sensors.horizontal_track.offset)
            } else {
                delta_h
            }
        } else {
            0.0
        };

        // Rotate local displacement by -LAO
        let delta_dx = (-lao).cos() * delta_dlx - (-lao).sin() * delta_dly;
        let delta_dy = (-lao).sin() * delta_dlx + (-lao).cos() * delta_dly;

        // Update pose and state vars
        let new_pose = (self.pose.0 + delta_dx, self.pose.1 + delta_dy, heading + self.start_heading);

        self.l0 = l1;
        self.r0 = r1;
        self.pose = new_pose;
        self.delta_pose = (delta_dx, delta_dx);
    }

    fn set_pos_dist(&mut self, dist: f64, offset: f64, angle: f64) {
        let sensor_heading = self.pose.2 - angle.to_radians() + f64::consts::FRAC_PI_2;
        match sensor_heading.rem_euclid(f64::consts::PI) {
            0.0..f64::consts::FRAC_PI_2 => {
                // Reset Positive X
                let scaled_dist = sensor_heading.cos() * (dist + offset);
                self.pose.0 = 72.0 + scaled_dist;
            },
            f64::consts::FRAC_PI_2..f64::consts::PI => {
                // Reset Positive Y
                let scaled_dist = sensor_heading.sin() * (dist + offset);
                self.pose.1 = 72.0 + scaled_dist;
            }
            _ => {}
        }
    }

    pub fn distance_reset(&mut self, sensor: u8) {
        match sensor {
            0 => self.set_pos_dist(self.dist_vals.0, self.sensors.distance_left.1, self.sensors.distance_left.2),
            1 => self.set_pos_dist(self.dist_vals.1, self.sensors.distance_right.1, self.sensors.distance_right.2),
            2 => self.set_pos_dist(self.dist_vals.2, self.sensors.distance_front.1, self.sensors.distance_front.2),
            _ => {}
        }
    }

    pub fn update_dist_sensors(&mut self) {
        if let Ok(Some(obj)) = self.sensors.distance_left.0.object() {
            self.dist_vals.0 = obj.distance as f64 * (0.5 * obj.confidence + 0.5) + (self.dist_vals.0 * (1.0 - (0.5 * obj.confidence + 0.5)));
        }

        if let Ok(Some(obj)) = self.sensors.distance_right.0.object() {
            self.dist_vals.1 = obj.distance as f64 * (0.5 * obj.confidence + 0.5) + (self.dist_vals.1 * (1.0 - (0.5 * obj.confidence + 0.5)));
        }

        if let Ok(Some(obj)) = self.sensors.distance_front.0.object() {
            self.dist_vals.2 = obj.distance as f64 * (0.5 * obj.confidence + 0.5) + (self.dist_vals.2 * (1.0 - (0.5 * obj.confidence + 0.5)));
        }
    }

    pub async fn tracking_loop(tracking: Arc<RwLock<Tracking>>) {
        loop {
            let mut track = tracking.write();

            track.last_tick = Instant::now();

            // Only update odom & GUI if the robot can move
            if !status().contains(CompetitionStatus::DISABLED) {
                // Get average IME values
                let dt = track.drive.read();
                let l1 = dt.left_motors.iter().fold(0.0, |a, m| a + m.position().unwrap_or_default().as_radians()) / dt.left_motors.iter().filter(|m| m.is_connected()).count() as f64;
                let r1 = dt.right_motors.iter().fold(0.0, |a, m| a + m.position().unwrap_or_default().as_radians()) / dt.right_motors.iter().filter(|m| m.is_connected()).count() as f64;
                drop(dt);
                // Odom Update
                track.odom_tick(l1, r1);
                track.update_dist_sensors();
                // GUI Update
                if let Ok(mut t) = track.telem.try_write() {
                    t.sensor_values = vec![track.sensors.imu.heading().unwrap_or_default().as_degrees(), track.h0, track.v0];
                    t.sensor_status = vec![track.imu_calibrated, track.sensors.horizontal_track.sens.is_connected(), track.sensors.vertical_track.sens.is_connected()];
                    t.pose = track.pose;
                }
            }
            // Get runtime to sleep the loop
            let dt = track.last_tick.elapsed();
            drop(track);
            sleep(Duration::from_secs_f64((0.007 - dt.as_secs_f64()).min(0.0))).await;
        }
    }
}
