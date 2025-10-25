use std::{cell::RefCell, rc::Rc, time::Duration};

use vexide::{peripherals::DynamicPeripherals, prelude::*};

use crate::{conf::Config, util::TrackingWheel};

#[derive(Default, Debug)]
pub(crate) struct TrackingState {
    pose: Rc<RefCell<Pose>>,
    delta_pose: (f64, f64),
    h0: f64,
    v0: f64,
}

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct Pose {
    pub pose: ((f64, f64), f64) = ((0.0, 0.0), 0.0),
    pub reset_pos: ((f64, f64), f64) = ((0.0, 0.0), 0.0),
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
    _dst_1: (DistanceSensor, f64, f64),
    _dst_2: (DistanceSensor, f64, f64),
    _dst_3: (DistanceSensor, f64, f64),
    imu: InertialSensor,
    enabled: bool,
}

impl Tracking {
    pub fn new(peripherals: &mut DynamicPeripherals, conf: &Config) -> (Tracking, Rc<RefCell<Pose>>) {
        let hor_rot_sens = RotationSensor::new(
            peripherals.take_smart_port(conf.ports[9]).expect("Horizontal tracking wheel sensor port not set"),
            if conf.reversed[9] { Direction::Reverse } else { Direction::Forward },
        );

        let vert_rot_sens = RotationSensor::new(
            peripherals.take_smart_port(conf.ports[10]).expect("Vertical tracking wheel sensor port not set"),
            if conf.reversed[10] { Direction::Reverse } else { Direction::Forward },
        );

        let imu = InertialSensor::new(peripherals.take_smart_port(conf.ports[11]).expect("IMU port not set"));

        let dst_1 = DistanceSensor::new(peripherals.take_smart_port(conf.ports[12]).expect("Distance Sensor 1 port not set"));
        let dst_2 = DistanceSensor::new(peripherals.take_smart_port(conf.ports[13]).expect("Distance Sensor 2 port not set"));
        let dst_3 = DistanceSensor::new(peripherals.take_smart_port(conf.ports[14]).expect("Distance Sensor 3 port not set"));

        let (state, pose) = TrackingState::new();
        (
            Tracking {
                state,
                horizontal_track: TrackingWheel { sens: hor_rot_sens, offset: conf.offsets[0] },
                vertical_track: TrackingWheel { sens: vert_rot_sens, offset: conf.offsets[1] },
                _dst_1: (dst_1, conf.offsets[2], conf.offsets[3]),
                _dst_2: (dst_2, conf.offsets[4], conf.offsets[5]),
                _dst_3: (dst_3, conf.offsets[6], conf.offsets[7]),
                imu,
                enabled: true,
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

        let mut pose = self.state.pose.borrow().clone();
        let delta_theta = (imu_heading + pose.reset_pos.1) - pose.pose.1;

        let delta_dlx = 2.0 * (delta_theta / 2.0).sin() * (delta_h / delta_theta + self.horizontal_track.offset);
        let delta_dly = 2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta + self.vertical_track.offset);
        let lao = pose.pose.1 + delta_theta / 2.0;
        let delta_dx = lao.cos() * delta_dlx - lao.sin() * delta_dly;
        let delta_dy = lao.cos() * delta_dly + lao.sin() * delta_dlx;

        pose.pose.1 = imu_heading + pose.reset_pos.1;
        pose.pose.0 .0 += delta_dx;
        pose.pose.0 .1 += delta_dy;
        self.state.pose.replace(pose);
        self.state.delta_pose = (delta_dx, delta_dx);
        self.state.h0 = h1;
        self.state.v0 = v1;
    }

    fn mcl_tick(&mut self) {}

    pub async fn tracking_loop(&mut self) {
        let tracking_pause = Duration::from_millis(10);
        loop {
            if self.enabled {
                if self.state.pose.borrow_mut().reset {
                    self.state.pose.borrow_mut().reset = false;
                    self.calibrate_imu().await;
                    let _ = self.horizontal_track.sens.reset_position();
                    let _ = self.vertical_track.sens.reset_position();
                    self.state.delta_pose = (0.0, 0.0);
                    self.state.h0 = 0.0;
                    self.state.v0 = 0.0;
                }
                self.odom_tick().await;
                self.mcl_tick();
            }
            sleep(tracking_pause).await;
        }
    }
}
