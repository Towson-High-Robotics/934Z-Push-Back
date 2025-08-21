use alloc::rc::Rc;
use core::{cell::RefCell, f64, time::Duration};

use vexide::{
    io::println,
    prelude::{Direction, DynamicPeripherals, Float, InertialSensor, RotationSensor, SmartDevice},
    time::sleep,
};

use crate::{
    conf::Config,
    util::{Drivetrain, TrackingWheel},
};

#[derive(Default, Debug, Clone)]
pub(crate) struct TrackingState {
    theta_r: f64,
    pose: Rc<RefCell<((f64, f64), f64)>>,
    l0: f64,
    r0: f64,
    s0: f64,
}

impl TrackingState {
    fn new() -> (TrackingState, Rc<RefCell<((f64, f64), f64)>>) {
        let pose = Rc::new(RefCell::new(((0., 0.), 0.)));
        (
            TrackingState {
                theta_r: 0.0,
                pose: pose.clone(),
                l0: 0.0,
                r0: 0.0,
                s0: 0.0,
            },
            pose,
        )
    }
}

#[derive(Debug)]
pub(crate) struct Tracking {
    state: TrackingState,
    conf: Config,
    drive: Drivetrain,
    horizontal_track: TrackingWheel,
    imu: InertialSensor,
    enabled: bool,
}

impl Tracking {
    pub fn new(drive: Drivetrain, peripherals: &mut DynamicPeripherals, conf: Config) -> (Tracking, Rc<RefCell<((f64, f64), f64)>>) {
        println!("Attempting to initialize Horizontal Tracking sensor!");
        let rot_sens = RotationSensor::new(
            peripherals
                .take_smart_port(conf.tracking.horizontal_track_port)
                .expect("Horizontal tracking wheel sensor port not set"),
            Direction::Forward,
        );
        println!("Attempting to initialize IMU!");
        let imu = InertialSensor::new(peripherals.take_smart_port(conf.tracking.imu_port).expect("IMU port not set"));
        let (state, pose) = TrackingState::new();
        (
            Tracking {
                state,
                conf,
                drive,
                horizontal_track: TrackingWheel {
                    sens: rot_sens,
                    offset: conf.tracking.horizontal_track_offset,
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
        self.drive.left_motors.borrow_mut().iter_mut().for_each(|m| {
            let _ = m.motor.reset_position();
        });
        self.drive.right_motors.borrow_mut().iter_mut().for_each(|m| {
            let _ = m.motor.reset_position();
        });
        *self.state.pose.borrow_mut() = (d0, theta_r);
        self.state.theta_r = theta_r;
        self.state.l0 = 0.0;
        self.state.r0 = 0.0;
        self.state.s0 = 0.0;
    }

    fn odom_tick(&mut self) {
        let (mut l1, connected) = self
            .drive
            .left_motors
            .borrow_mut()
            .iter_mut()
            .fold((0.0, 0.0), |acc, m| (acc.0 + m.get_pos_degrees().unwrap_or(0.0), m.connected() as i32 as f64 + acc.1));
        if connected > 0.0 {
            l1 /= connected;
        } else {
            return;
        }

        let (mut r1, connected) = self
            .drive
            .right_motors
            .borrow_mut()
            .iter_mut()
            .fold((0.0, 0.0), |acc, m| (acc.0 + m.get_pos_degrees().unwrap_or(0.0), m.connected() as i32 as f64 + acc.1));
        if connected > 0.0 {
            r1 /= connected;
        } else {
            return;
        }

        let imu_heading = self.imu.heading().unwrap_or(1000.0);
        if !self.horizontal_track.sens.is_connected() {
            return;
        }
        let s1 = self.horizontal_track.sens.angle().unwrap_or_default().as_degrees();

        let delta_l = l1 - self.state.l0;
        let delta_r = r1 - self.state.r0;
        let delta_s = s1 - self.state.s0;

        let mut theta_1 = (delta_l - delta_r) / (self.conf.tracking.left_wheel_offset + self.conf.tracking.right_wheel_offset) * 180.0
            / f64::consts::PI
            + self.state.theta_r;
        let pose = self.state.pose.borrow();
        if ((theta_1 - pose.1) - (imu_heading - pose.1)).abs() <= 10.0 {
            theta_1 = imu_heading;
        }
        let delta_theta = theta_1 - pose.1;
        drop(pose);

        let delta_dlx = 2.0 * (delta_theta / 2.0).sin() * (delta_s / delta_theta + self.horizontal_track.offset);
        let delta_dly = 2.0 * (delta_theta / 2.0).sin() * (delta_r / delta_theta + self.conf.tracking.right_wheel_offset);
        let lao = self.state.pose.borrow().1 + delta_theta / 2.0;
        let delta_dx = lao.cos() * delta_dlx - lao.sin() * delta_dly;
        let delta_dy = lao.cos() * delta_dly + lao.cos() * delta_dlx;

        {
            let mut pose = self.state.pose.borrow_mut();
            pose.1 = theta_1;
            pose.0 .0 += delta_dx;
            pose.0 .1 += delta_dy;
        }
        self.state.l0 = l1;
        self.state.r0 = r1;
        self.state.s0 = s1;
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
