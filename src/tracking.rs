use std::{
    f64,
    ops::Rem,
    sync::{nonpoison::RwLock, Arc},
    time::Duration,
};

use vexide::{math::Angle, peripherals::DynamicPeripherals, prelude::*};

use crate::{
    conf::Config,
    util::{Drivetrain, Telem, TrackingWheel},
};

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct Pose {
    pub pose: (f64, f64, f64) = (0.0, 0.0, 0.0),
    pub reset_pos: (f64, f64, f64) = (0.0, 0.0, 0.0),
    pub reset: bool = false,
    pub calibrate: bool = false,
}

#[derive(Debug)]
pub(crate) struct Tracking {
    horizontal_track: TrackingWheel,
    vertical_track: TrackingWheel,
    drive: Arc<RwLock<Drivetrain>>,
    imu: InertialSensor,
    telem: Arc<RwLock<Telem>>,
    pose: Arc<RwLock<Pose>>,
    delta_pose: (f64, f64),
    h0: f64,
    v0: f64,
    l0: f64,
    r0: f64,
}

impl Tracking {
    pub fn new(peripherals: &mut DynamicPeripherals, telem: Arc<RwLock<Telem>>, drive: Arc<RwLock<Drivetrain>>, conf: &Config) -> (Tracking, Arc<RwLock<Pose>>) {
        let hor_rot_sens = RotationSensor::new(
            peripherals.take_smart_port(conf.ports[9]).expect("Horizontal tracking wheel sensor port not set"),
            if conf.reversed[9] { Direction::Reverse } else { Direction::Forward },
        );

        let vert_rot_sens = RotationSensor::new(
            peripherals.take_smart_port(conf.ports[10]).expect("Vertical tracking wheel sensor port not set"),
            if conf.reversed[10] { Direction::Reverse } else { Direction::Forward },
        );

        let imu = InertialSensor::new(peripherals.take_smart_port(conf.ports[11]).expect("IMU port not set"));

        let pose = Arc::new(RwLock::new(Pose::default()));
        (
            Tracking {
                horizontal_track: TrackingWheel { sens: hor_rot_sens, offset: conf.offsets[0] },
                vertical_track: TrackingWheel { sens: vert_rot_sens, offset: conf.offsets[1] },
                imu,
                telem,
                drive,
                pose: pose.clone(),
                delta_pose: (0.0, 0.0),
                h0: 0.0,
                v0: 0.0,
                l0: 0.0,
                r0: 0.0,
            },
            pose,
        )
    }

    pub async fn calibrate_imu(&mut self) {
        if !self.imu.is_connected() {
            println!("IMU isn't connected, couldn't calibrate");
            return;
        }

        match self.imu.calibrate().await {
            Ok(_) => println!("IMU successfully calibrated :D"),
            Err(e) => {
                println!("IMU failed to calibrate :(\nError:\n{e:?}");
                if self.imu.calibrate().await.is_err() {
                    println!("IMU failed to calibrate (again) >:(");
                }
            }
        }
    }

    fn odom_tick(&mut self) {
        if !self.imu.is_connected() {
            return;
        }

        let pose = self.pose.read();
        let imu_heading = self.imu.heading().unwrap().as_radians();
        let delta_theta = imu_heading - pose.pose.2;
        let lao = pose.pose.2 + delta_theta / 2.0;

        let dt = self.drive.read();
        let l1 = dt.left_motors.iter().fold(0.0, |a, m| a + m.position().unwrap_or_default().as_radians()) / dt.left_motors.iter().filter(|m| m.is_connected()).count() as f64;
        let r1 = dt.right_motors.iter().fold(0.0, |a, m| a + m.position().unwrap_or_default().as_radians()) / dt.right_motors.iter().filter(|m| m.is_connected()).count() as f64;

        let delta_dly = /* if self.vertical_track.sens.is_connected() {
            let v1 = self.vertical_track.sens.angle().unwrap_or_default().as_radians();
            let delta_v = v1 - self.v0;
            self.v0 = v1;

            if delta_theta != 0.0 {
                2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta + self.vertical_track.offset)
            } else {
                delta_v
            }
        } else  */{
            // 0.609375 = 0.5 (for averaging) * 1.625 (wheel radius) * 36/48 (gear ratio)
            let v0 = (self.l0 + self.r0) * 0.609375;
            let v1 = (l1 + r1) * 0.609375;
            let delta_v = v1 - v0;
            if delta_theta != 0.0 {
                2.0 * (delta_theta / 2.0).sin() * delta_v / delta_theta
            } else {
                delta_v
            }
        };

        let delta_dlx = /* if self.horizontal_track.sens.is_connected() {
            let h1 = self.horizontal_track.sens.angle().unwrap_or_default().as_radians() * 2.00;
            let delta_h = h1 - self.h0;
            if delta_theta != 0.0 {
                2.0 * (delta_theta / 2.0).sin() * (delta_h / delta_theta + self.horizontal_track.offset)
            } else {
                delta_h
            }
        } else  */{
            0.0
        };

        let delta_dx = lao.cos() * delta_dlx - lao.sin() * delta_dly;
        let delta_dy = lao.cos() * delta_dly + lao.sin() * delta_dlx;

        let new_pose = (pose.pose.0 + delta_dx, pose.pose.1 + delta_dy, imu_heading + pose.reset_pos.2);
        drop(pose);

        self.l0 = l1;
        self.r0 = r1;
        self.pose.write().pose = new_pose;
        self.delta_pose = (delta_dx, delta_dx);
    }

    pub async fn tracking_loop(&mut self) {
        loop {
            if self.pose.read().reset || self.pose.read().calibrate {
                if self.pose.read().calibrate {
                    self.pose.write().calibrate = false;
                    self.calibrate_imu().await;
                }
                let mut pose = self.pose.write();
                pose.reset = false;
                pose.pose = pose.reset_pos;
                self.horizontal_track.sens.reset_position().ok();
                self.vertical_track.sens.reset_position().ok();
                self.imu.set_heading(Angle::from_degrees((-pose.reset_pos.2 - 90.0).rem(360.0))).ok();
                drop(pose);
                self.drive.write().left_motors.iter_mut().for_each(|m| {
                    m.reset_position().ok();
                });
                self.drive.write().right_motors.iter_mut().for_each(|m| {
                    m.reset_position().ok();
                });
                self.delta_pose = (0.0, 0.0);
                self.h0 = 0.0;
                self.v0 = 0.0;
            }
            self.odom_tick();
            if let Ok(mut t) = self.telem.try_write() {
                t.sensor_values = vec![self.imu.heading().unwrap_or_default().as_degrees(), self.h0, self.v0];
                t.sensor_status = vec![self.imu.is_connected(), self.horizontal_track.sens.is_connected(), self.vertical_track.sens.is_connected()];
                t.pose = self.pose.read().pose;
            }
            sleep(Duration::from_millis(10)).await;
        }
    }
}
