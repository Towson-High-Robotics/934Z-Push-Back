use std::{
    f64,
    ops::Rem,
    sync::{Arc, nonpoison::RwLock},
    time::{Duration, Instant},
};

use vexide::{competition::{CompetitionStatus, status}, math::Angle, peripherals::DynamicPeripherals, prelude::*};

use crate::{
    conf::Config, log_error, log_info, log_warn, telemetry::Telem, util::{Drivetrain, TrackingWheel}
};

#[derive(Debug)]
pub(crate) struct Tracking {
    last_tick: Instant,
    horizontal_track: TrackingWheel,
    vertical_track: TrackingWheel,
    drive: Arc<RwLock<Drivetrain>>,
    imu: InertialSensor,
    imu_calibrated: bool,
    telem: Arc<RwLock<Telem>>,
    pub(crate) pose: (f64, f64, f64),
    start_heading: f64,
    delta_pose: (f64, f64),
    h0: f64,
    v0: f64,
    l0: f64,
    r0: f64,
}

impl Tracking {
    pub fn new(peripherals: &mut DynamicPeripherals, telem: Arc<RwLock<Telem>>, drive: Arc<RwLock<Drivetrain>>, conf: &Config) -> Tracking {
        
        // Create objects for the sensors
        let hor_rot_sens = RotationSensor::new(
            peripherals.take_smart_port(conf.ports[9]).expect("Horizontal tracking wheel sensor port not set"),
            if conf.reversed[9] { Direction::Reverse } else { Direction::Forward },
        );

        let vert_rot_sens = RotationSensor::new(
            peripherals.take_smart_port(conf.ports[10]).expect("Vertical tracking wheel sensor port not set"),
            if conf.reversed[10] { Direction::Reverse } else { Direction::Forward },
        );

        let imu = InertialSensor::new(peripherals.take_smart_port(conf.ports[11]).expect("IMU port not set"));

        // And return the struct
        Tracking {
            last_tick: Instant::now(),
            horizontal_track: TrackingWheel { sens: hor_rot_sens, offset: conf.offsets[0] },
            vertical_track: TrackingWheel { sens: vert_rot_sens, offset: conf.offsets[1] },
            imu,
            imu_calibrated: false,
            telem,
            drive,
            pose: (0.0, 0.0, 0.0),
            start_heading: 0.0,
            delta_pose: (0.0, 0.0),
            h0: 0.0,
            v0: 0.0,
            l0: 0.0,
            r0: 0.0,
        }
    }

    pub async fn calibrate_imu(&mut self) {
        // Exit if the IMU isn't plugged in
        if !self.imu.is_connected() {
            log_warn!("IMU isn't connected, couldn't calibrate");
            self.imu_calibrated = false;
            return;
        }

        // Attempt to calibrate the IMU twice
        match self.imu.calibrate().await {
            Ok(_) => log_info!("IMU successfully calibrated :D"),
            Err(e) => {
                log_warn!("IMU failed to calibrate :(\nError:\n{e:?}");
                if self.imu.calibrate().await.is_err() {
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
        self.horizontal_track.sens.reset_position().ok();
        self.vertical_track.sens.reset_position().ok();
        self.imu.set_heading(Angle::from_degrees((-reset_pose.2 + 90.0).rem(360.0))).ok();

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
            self.imu.heading().unwrap().as_radians().rem_euclid(f64::consts::TAU)
        } else {
            ((((l1 - self.l0) * 1.21875) - ((r1 - self.r0) * 1.21875)) / 10.37 + self.pose.2).rem_euclid(f64::consts::TAU)
        };
        // Get delta theta and LAO
        let mut delta_theta = (heading - self.pose.2).rem_euclid(f64::consts::TAU);
        if delta_theta > f64::consts::PI { delta_theta -= f64::consts::TAU };
        let lao = self.pose.2 + delta_theta / 2.0;

        // Vertical displacement, fall back to IMEs if no vert wheel
        let delta_dly = if self.vertical_track.sens.is_connected() {
            // Vertical wheel travel
            let v1 = self.vertical_track.sens.angle().unwrap_or_default().as_radians();
            let delta_v = v1 - self.v0;
            self.v0 = v1;

            // Mapped to an arc if the change in angle is 0
            if delta_theta != 0.0 {
                2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta + self.vertical_track.offset)
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
        let delta_dlx = if self.horizontal_track.sens.is_connected() {
            // Horizontal wheel displacement
            let h1 = self.horizontal_track.sens.angle().unwrap_or_default().as_radians() * 2.00;
            let delta_h = h1 - self.h0;
            self.h0 = h1;

            // Map it to an arc if there's an angle change
            if delta_theta != 0.0 {
                2.0 * (delta_theta / 2.0).sin() * (delta_h / delta_theta + self.horizontal_track.offset)
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
                // GUI Update
                if let Ok(mut t) = track.telem.try_write() {
                    t.sensor_values = vec![track.imu.heading().unwrap_or_default().as_degrees(), track.h0, track.v0];
                    t.sensor_status = vec![track.imu_calibrated, track.horizontal_track.sens.is_connected(), track.vertical_track.sens.is_connected()];
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
