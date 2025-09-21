use alloc::rc::Rc;
use core::{cell::RefCell, f64, time::Duration};

use vexide::{
    io::println,
    prelude::{Direction, DynamicPeripherals, Float, InertialSensor, RotationSensor, SmartDevice},
    time::sleep,
};

use crate::{conf::Config, util::TrackingWheel};

#[derive(Default, Debug, Clone)]
pub(crate) struct TrackingState {
    pose: Rc<RefCell<((f64, f64), f64)>>,
    h0: f64,
    v0: f64,
}

impl TrackingState {
    fn new() -> (TrackingState, Rc<RefCell<((f64, f64), f64)>>) {
        let pose = Rc::new(RefCell::new(((0., 0.), 0.)));
        (
            TrackingState {
                pose: pose.clone(),
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
    enabled: bool,
}

impl Tracking {
    pub fn new(peripherals: &mut DynamicPeripherals, conf: Config) -> (Tracking, Rc<RefCell<((f64, f64), f64)>>) {
        println!("Attempting to initialize Horizontal Tracking sensor!");
        let hor_rot_sens = RotationSensor::new(
            peripherals
                .take_smart_port(conf.tracking.horizontal_track_port)
                .expect("Horizontal tracking wheel sensor port not set"),
            Direction::Forward,
        );

        println!("Attempting to initialize Vertical Tracking sensor!");
        let vert_rot_sens = RotationSensor::new(
            peripherals
                .take_smart_port(conf.tracking.vertical_track_port)
                .expect("Horizontal tracking wheel sensor port not set"),
            Direction::Forward,
        );

        println!("Attempting to initialize IMU!");
        let imu = InertialSensor::new(peripherals.take_smart_port(conf.tracking.imu_port).expect("IMU port not set"));
        let (state, pose) = TrackingState::new();
        (
            Tracking {
                state,
                horizontal_track: TrackingWheel {
                    sens: hor_rot_sens,
                    offset: conf.tracking.horizontal_track_offset,
                },
                vertical_track: TrackingWheel {
                    sens: vert_rot_sens,
                    offset: conf.tracking.vertical_track_offset,
                },
                imu,
                enabled: true,
            },
            pose,
        )
    }

    pub async fn calibrate_imu(&mut self) {
        if !self.imu.is_connected() {
            println!("IMU isn't connected! Aborting!");
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

    fn _odom_reset(&mut self, d0: (f64, f64), theta_r: f64) {
        *self.state.pose.borrow_mut() = (d0, theta_r);
        self.state.h0 = 0.0;
        self.state.v0 = 0.0;
    }

    fn odom_tick(&mut self) {
        if !(self.horizontal_track.sens.is_connected() && self.vertical_track.sens.is_connected() && self.imu.is_connected()) {
            return;
        }

        let imu_heading = self.imu.heading().unwrap();
        let h1 = self.horizontal_track.sens.angle().unwrap_or_default().as_degrees();
        let v1 = self.vertical_track.sens.angle().unwrap_or_default().as_degrees();

        let delta_h = h1 - self.state.h0;
        let delta_v = v1 - self.state.v0;

        let pose = self.state.pose.borrow();
        let delta_theta = imu_heading - pose.1;
        drop(pose);

        let delta_dlx = 2.0 * (delta_theta / 2.0).sin() * (delta_h / delta_theta + self.horizontal_track.offset);
        let delta_dly = 2.0 * (delta_theta / 2.0).sin() * (delta_v / delta_theta + self.vertical_track.offset);
        let lao = self.state.pose.borrow().1 + delta_theta / 2.0;
        let delta_dx = lao.cos() * delta_dlx - lao.sin() * delta_dly;
        let delta_dy = lao.cos() * delta_dly + lao.sin() * delta_dlx;

        let mut pose = self.state.pose.borrow_mut();
        pose.1 = imu_heading;
        pose.0 .0 += delta_dx;
        pose.0 .1 += delta_dy;
        drop(pose);
        self.state.h0 = h1;
        self.state.v0 = v1;
    }

    fn mcl_tick(&mut self) {}

    pub async fn tracking_loop(&mut self) {
        let tracking_pause = Duration::from_millis(10);
        loop {
            if self.enabled {
                self.odom_tick();
                self.mcl_tick();
            }
            sleep(tracking_pause).await;
        }
    }
}
