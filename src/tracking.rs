use std::{cell::RefCell, rc::Rc, sync::{Arc, nonpoison::RwLock}, time::Duration};

use vexide::{peripherals::DynamicPeripherals, prelude::*};

use crate::{
    conf::Config,
    util::{Telem, TrackingWheel},
};

#[derive(Default, Debug)]
pub(crate) struct TrackingState {
    pose: Rc<RefCell<Pose>>,
    delta_pose: (f64, f64),
    h0: f64,
    v0: f64,
}

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct Pose {
    pub pose: (f64, f64, f64) = (0.0, 0.0, 0.0),
    pub reset_pos: (f64, f64, f64) = (0.0, 0.0, 0.0),
    pub reset: bool = false,
}

impl TrackingState {
    fn new() -> (TrackingState, Rc<RefCell<Pose>>) {
        let pose = Rc::new(RefCell::new(Pose::default()));
        (
            TrackingState {
                pose: pose.clone(),
                delta_pose: (0.0, 0.0),
                h0: 0.0,
                v0: 0.0,
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
    pub fn new(peripherals: &mut DynamicPeripherals, telem: Arc<RwLock<Telem>>, conf: &Config) -> (Tracking, Rc<RefCell<Pose>>) {
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

    async fn odom_tick(&mut self) {
        if !(self.horizontal_track.sens.is_connected() && self.vertical_track.sens.is_connected() && self.imu.is_connected()) {
            return;
        }

        let imu_heading = self.imu.heading().unwrap().as_radians();
        let h1 = self.horizontal_track.sens.angle().unwrap_or_default().as_radians() * 2.00;
        let v1 = self.vertical_track.sens.angle().unwrap_or_default().as_radians() * 2.00;

        let delta_h = h1 - self.state.h0;
        let delta_v = v1 - self.state.v0;

        let mut pose = self.state.pose.borrow_mut();
        let delta_theta = (imu_heading + pose.reset_pos.2) - pose.pose.2;

        let delta_dlx = 2.0 * (delta_theta / 2.0).sin() * (delta_h / delta_theta + self.horizontal_track.offset);
        let delta_dly = 2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta + self.vertical_track.offset);
        let lao = pose.pose.2 + delta_theta / 2.0;
        let delta_dx = lao.cos() * delta_dlx - lao.sin() * delta_dly;
        let delta_dy = lao.cos() * delta_dly + lao.sin() * delta_dlx;

        pose.pose.0 += delta_dx;
        pose.pose.1 += delta_dy;
        pose.pose.2 = imu_heading + pose.reset_pos.1;
        drop(pose);
        self.state.delta_pose = (delta_dx, delta_dx);
        self.state.h0 = h1;
        self.state.v0 = v1;
    }

    pub async fn tracking_loop(&mut self) {
        let tracking_pause = Duration::from_millis(10);
        loop {
            if self.state.pose.borrow_mut().reset {
                self.state.pose.borrow_mut().reset = false;
                self.calibrate_imu().await;
                self.horizontal_track.sens.reset_position().ok();
                self.vertical_track.sens.reset_position().ok();
                self.state.delta_pose = (0.0, 0.0);
                self.state.h0 = 0.0;
                self.state.v0 = 0.0;
            }
            self.odom_tick().await;
            if let Ok(mut t) = self.telem.try_write() {
                t.sensor_values = vec![
                    self.imu.heading().unwrap_or_default().as_degrees(),
                    self.state.h0,
                    self.state.v0
                ];
                t.sensor_status = vec![
                    self.imu.is_connected(), 
                    self.horizontal_track.sens.is_connected(), 
                    self.vertical_track.sens.is_connected()
                ];
            }
            sleep(tracking_pause).await;
        }
    }
}
