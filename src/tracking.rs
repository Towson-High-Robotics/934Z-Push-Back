use std::{
    sync::{nonpoison::RwLock, Arc},
    time::Duration,
};

use vexide::{peripherals::DynamicPeripherals, prelude::*};

use crate::{
    conf::Config,
    gui::MotorType,
    util::{Telem, TrackingWheel},
};

#[derive(Default, Debug)]
pub(crate) struct TrackingState {
    pose: Arc<RwLock<Pose>>,
    delta_pose: (f64, f64),
    h0: f64,
    v0: f64,
    l0: f64,
    r0: f64,
}

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct Pose {
    pub pose: (f64, f64, f64) = (0.0, 0.0, 0.0),
    pub reset_pos: (f64, f64, f64) = (0.0, 0.0, 0.0),
    pub reset: bool = false,
}

impl TrackingState {
    fn new() -> (TrackingState, Arc<RwLock<Pose>>) {
        let pose = Arc::new(RwLock::new(Pose::default()));
        (
            TrackingState {
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
}

#[derive(Debug)]
pub(crate) struct Tracking {
    state: TrackingState,
    horizontal_track: TrackingWheel,
    vertical_track: TrackingWheel,
    imu: InertialSensor,
    telem: Arc<RwLock<Telem>>,
}

impl Tracking {
    pub fn new(peripherals: &mut DynamicPeripherals, telem: Arc<RwLock<Telem>>, conf: &Config) -> (Tracking, Arc<RwLock<Pose>>) {
        let hor_rot_sens = RotationSensor::new(
            peripherals.take_smart_port(conf.ports[9]).expect("Horizontal tracking wheel sensor port not set"),
            if conf.reversed[9] { Direction::Reverse } else { Direction::Forward },
        );

        let vert_rot_sens = RotationSensor::new(
            peripherals.take_smart_port(conf.ports[10]).expect("Vertical tracking wheel sensor port not set"),
            if conf.reversed[10] { Direction::Reverse } else { Direction::Forward },
        );

        let imu = InertialSensor::new(peripherals.take_smart_port(conf.ports[11]).expect("IMU port not set"));

        let (state, pose) = TrackingState::new();
        (
            Tracking {
                state,
                horizontal_track: TrackingWheel { sens: hor_rot_sens, offset: conf.offsets[0] },
                vertical_track: TrackingWheel { sens: vert_rot_sens, offset: conf.offsets[1] },
                imu,
                telem,
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

        let mut pose = self.state.pose.write();
        let imu_heading = self.imu.heading().unwrap().as_radians();
        let delta_theta = (imu_heading + pose.reset_pos.2) - pose.pose.2;
        let lao = pose.pose.2 + delta_theta / 2.0;

        if self.vertical_track.sens.is_connected() {
            let v1 = self.vertical_track.sens.angle().unwrap_or_default().as_radians() * 2.00;
            let delta_v = v1 - self.state.v0;
            let delta_dly = 2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta + self.vertical_track.offset);

            if self.horizontal_track.sens.is_connected() {
                let h1 = self.horizontal_track.sens.angle().unwrap_or_default().as_radians() * 2.00;
                let delta_h = h1 - self.state.h0;
                let delta_dlx = 2.0 * (delta_theta / 2.0).sin() * (delta_h / delta_theta + self.horizontal_track.offset);

                let delta_dx = lao.cos() * delta_dlx - lao.sin() * delta_dly;
                let delta_dy = lao.cos() * delta_dly + lao.sin() * delta_dlx;

                pose.pose.0 += delta_dx;
                pose.pose.1 += delta_dy;
                pose.pose.2 = imu_heading + pose.reset_pos.1;
                self.state.delta_pose = (delta_dx, delta_dx);
                self.state.h0 = h1;
            } else {
                let delta_dx = -lao.sin() * delta_dly;
                let delta_dy = lao.cos() * delta_dly;

                pose.pose.0 += delta_dx;
                pose.pose.1 += delta_dy;
                pose.pose.2 = imu_heading + pose.reset_pos.1;
                self.state.delta_pose = (delta_dx, delta_dx);
            }
            self.state.v0 = v1;
            if let Ok(t) = self.telem.try_read() {
                let left_c = t.motor_types[0..3].iter().filter(|x| **x != MotorType::Disconnected).fold(0.0, |a, _| a + 1.0);
                self.state.l0 = t.motor_headings[0..3].iter().fold(0.0, |a, h| a + h) / left_c;
                let right_c = t.motor_types[3..6].iter().filter(|x| **x != MotorType::Disconnected).fold(0.0, |a, _| a + 1.0);
                self.state.r0 = t.motor_headings[3..6].iter().fold(0.0, |a, h| a + h) / right_c;
            }
        } else if let Ok(t) = self.telem.try_read() {
            let left_c = t.motor_types[0..3].iter().filter(|x| **x != MotorType::Disconnected).fold(0.0, |a, _| a + 1.0);
            let l1 = t.motor_headings[0..3].iter().fold(0.0, |a, h| a + h) / left_c;
            let right_c = t.motor_types[3..6].iter().filter(|x| **x != MotorType::Disconnected).fold(0.0, |a, _| a + 1.0);
            let r1 = t.motor_headings[3..6].iter().fold(0.0, |a, h| a + h) / right_c;

            let v1 = l1 + r1;
            let delta_v = v1 - self.state.v0;
            let delta_dly = 2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta);

            if self.horizontal_track.sens.is_connected() {
                let h1 = self.horizontal_track.sens.angle().unwrap_or_default().as_radians() * 2.00;
                let delta_h = h1 - self.state.h0;
                let delta_dlx = 2.0 * (delta_theta / 2.0).sin() * (delta_h / delta_theta + self.horizontal_track.offset);

                let delta_dx = lao.cos() * delta_dlx - lao.sin() * delta_dly;
                let delta_dy = lao.cos() * delta_dly + lao.sin() * delta_dlx;

                pose.pose.0 += delta_dx;
                pose.pose.1 += delta_dy;
                pose.pose.2 = imu_heading + pose.reset_pos.1;
                self.state.delta_pose = (delta_dx, delta_dx);
                self.state.h0 = h1;
            } else {
                let delta_dx = -lao.sin() * delta_dly;
                let delta_dy = lao.cos() * delta_dly;

                pose.pose.0 += delta_dx;
                pose.pose.1 += delta_dy;
                pose.pose.2 = imu_heading + pose.reset_pos.1;
                self.state.delta_pose = (delta_dx, delta_dx);
            }
            self.state.l0 = l1;
            self.state.r0 = r1;
        }

        drop(pose);
    }

    pub async fn tracking_loop(&mut self) {
        loop {
            if self.state.pose.read().reset {
                self.state.pose.write().reset = false;
                self.calibrate_imu().await;
                self.horizontal_track.sens.reset_position().ok();
                self.vertical_track.sens.reset_position().ok();
                self.state.delta_pose = (0.0, 0.0);
                self.state.h0 = 0.0;
                self.state.v0 = 0.0;
            }
            self.odom_tick();
            if let Ok(mut t) = self.telem.try_write() {
                t.sensor_values = vec![self.imu.heading().unwrap_or_default().as_degrees(), self.state.h0, self.state.v0];
                t.sensor_status = vec![self.imu.is_connected(), self.horizontal_track.sens.is_connected(), self.vertical_track.sens.is_connected()];
            }
            sleep(Duration::from_millis(10)).await;
        }
    }
}
