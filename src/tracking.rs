use core::{cell::RefCell, f64, time::Duration};

use alloc::rc::Rc;
use vexide::{io::println, prelude::{Direction, Float, InertialSensor, RotationSensor, SmartDevice}, time::sleep};

use crate::util::{Robot, TrackingWheel};

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct TrackingState {
    theta_r: f64,
    theta_0: f64,
    d0: (f64, f64),
    l0: f64,
    r0: f64,
    s0: f64
}

impl TrackingState {
    fn new() -> TrackingState {
        TrackingState {
            theta_r: 0.0, theta_0: 0.0,
            d0: (0.0, 0.0),
            l0: 0.0, r0: 0.0,
            s0: 0.0
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct TrackingDevices {
    robot: Rc<RefCell<Robot>>,
    horizontal_track: Rc<RefCell<TrackingWheel>>,
    imu: Rc<RefCell<InertialSensor>>
}

impl TrackingDevices {
    fn new(robot: Rc<RefCell<Robot>>, horizontal_track: TrackingWheel, imu: InertialSensor) -> TrackingDevices {
        TrackingDevices {
            robot,
            horizontal_track: Rc::new(RefCell::new(horizontal_track)),
            imu: Rc::new(RefCell::new(imu))
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct Tracking {
    state: TrackingState,
    devices: TrackingDevices,
    enabled: bool,
}

impl Tracking {
    pub fn new(robot: Rc<RefCell<Robot>>) -> Tracking {
        let mut borrowed_robot = robot.borrow_mut();
        let conf = borrowed_robot.conf;
        println!("Attempting to initialize Horizontal Tracking sensor!");
        let rot_sens = RotationSensor::new(
            borrowed_robot.take_smart(conf.tracking.horizontal_track_port).expect("Horizontal tracking wheel sensor port not set"),
            Direction::Forward
        );
        println!("Attempting to initialize IMU!");
        let imu = InertialSensor::new(
            borrowed_robot.take_smart(conf.tracking.imu_port).expect("IMU port not set")
        );
        drop(borrowed_robot);
        Tracking {
            state: TrackingState::new(),
            devices: TrackingDevices::new(
                robot,
                TrackingWheel {
                    sens: rot_sens,
                    offset: conf.tracking.horizontal_track_offset
                },
                imu
            ),
            enabled: true
        }
    }

    pub async fn calibrate_imu(&mut self) {
        let mut imu = self.devices.imu.borrow_mut();
        if !imu.is_connected() {
            println!("IMU isn't connected! Aborting!");
            return;
        }
        
        match imu.calibrate().await {
            Ok(_) => println!("IMU successfully calibrated :D"),
            Err(e) => {
                println!("IMU failed to calibrate :(\nError:\n{e:?}");
                if imu.calibrate().await.is_err() {
                    println!("IMU failed to calibrate (again) >:(");
                }
            }
        }
        drop(imu);
    }

    fn odom_reset(&mut self, d0: (f64, f64), theta_r: f64) {
        let mut robot = self.devices.robot.borrow_mut();
        let drive = robot.drive.as_mut().expect("Tracking requires an Initialized Drivetrain");
        drive.left_motors.borrow_mut().iter_mut().for_each(|m| { let _ = m.motor.reset_position(); });
        drive.right_motors.borrow_mut().iter_mut().for_each(|m| { let _ = m.motor.reset_position(); });
        robot.pose = ((0.0, 0.0), 0.0);
        drop(robot);
        self.state.theta_r = theta_r; self.state.theta_0 = theta_r;
        self.state.d0 = d0;
        self.state.l0 = 0.0; self.state.r0 = 0.0;
        self.state.s0 = 0.0;
    }

    fn odom_tick(&mut self) {
        let mut robot = self.devices.robot.borrow_mut();
        let drive = robot.drive.as_mut().expect("Tracking requires an Initialized Drivetrain");

        let imu_heading = self.devices.imu.borrow().heading().unwrap_or(1000.0);
        let (mut l1, connected) = drive.left_motors.borrow_mut().iter_mut().fold((0.0, 0.0), |acc, m| (acc.0 + m.get_pos_degrees().unwrap_or_default(), m.motor.is_connected() as i32 as f64 + acc.1));
        if connected > 0.0 { l1 /= connected; } else { return; }
        let (mut r1, connected) = drive.right_motors.borrow_mut().iter_mut().fold((0.0, 0.0), |acc, m| (acc.0 + m.get_pos_degrees().unwrap_or_default(), m.motor.is_connected() as i32 as f64 + acc.1));
        if connected > 0.0 { r1 /= connected; } else { return; }
        if !self.devices.horizontal_track.borrow().sens.is_connected() { return; }
        let s1 = self.devices.horizontal_track.borrow_mut().sens.angle().unwrap_or_default().as_degrees();
        
        let delta_l = l1 - self.state.l0; let delta_r = r1 - self.state.r0;
        let delta_s = s1 - self.state.s0;

        let mut theta_1 = (delta_l - delta_r) / (robot.conf.tracking.left_wheel_offset + robot.conf.tracking.right_wheel_offset) * 180.0 / f64::consts::PI + self.state.theta_r;
        if ((theta_1 - self.state.theta_0) - (imu_heading - self.state.theta_0)).abs() <= 10.0 { theta_1 = imu_heading; }
        let delta_theta = theta_1 - self.state.theta_0;

        let delta_dlx = 2.0 * (delta_theta / 2.0).sin() * (delta_s / delta_theta + robot.conf.tracking.horizontal_track_offset);
        let delta_dly = 2.0 * (delta_theta / 2.0).sin() * (delta_r / delta_theta + robot.conf.tracking.right_wheel_offset);
        let lao = self.state.theta_0 + delta_theta / 2.0;
        let delta_dx = lao.cos() * delta_dlx - lao.sin() * delta_dly;
        let delta_dy = lao.cos() * delta_dly + lao.cos() * delta_dlx;

        self.state.theta_0 = theta_1;
        self.state.d0.0 += delta_dx; self.state.d0.1 += delta_dy;
        self.state.l0 = l1; self.state.r0 = r1;
        self.state.s0 = s1;

        robot.pose = (self.state.d0, self.state.theta_0);

        drop(robot);
    }

    fn mcl_tick(&mut self) {

    }

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