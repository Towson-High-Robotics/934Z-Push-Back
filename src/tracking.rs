use core::{cell::{Ref, RefCell}, time::Duration};

use alloc::{borrow, rc::Rc};
use vexide::{io::println, prelude::{Direction, InertialSensor, RotationSensor}, time::sleep};

use crate::util::{Drivetrain, Robot, TrackingWheel};

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct TrackingState {
    theta_r: f64,
    theta_0: f64,
    d_0: (f64, f64),
    l_0: f64,
    r_0: f64,
    s_0: f64
}

impl TrackingState {
    fn new() -> TrackingState {
        TrackingState {
            theta_r: 0.0, theta_0: 0.0,
            d_0: (0.0, 0.0),
            l_0: 0.0, r_0: 0.0,
            s_0: 0.0
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
            devices: TrackingDevices::new(
                robot,
                TrackingWheel {
                    sens: rot_sens,
                    offset: conf.tracking.horizontal_track_offset
                },
                imu
            )
        }
    }

    pub async fn calibrate_imu(&mut self) {
        let mut imu = self.devices.imu.borrow_mut();
        match imu.calibrate().await {
            Ok(_) => println!("IMU successfully calibrated :D"),
            Err(e) => {
                println!("IMU failed to calibrate :(\nError:\n{e:?}");
                if imu.calibrate().await.is_err() {
                    panic!("IMU failed to calibrate (again) >:(");
                }
            }
        }
        drop(imu);
    }

    fn odom_reset(&mut self, d_0: (f64, f64), theta_r: f64) {
        let mut robot = self.devices.robot.borrow_mut();
        let drive = robot.drive.as_mut().expect("Tracking requires an Initialized Drivetrain");
        drive.left_motors.borrow_mut().iter_mut().for_each(|m| { let _ = m.motor.reset_position(); });
        drive.right_motors.borrow_mut().iter_mut().for_each(|m| { let _ = m.motor.reset_position(); });
        drop(robot);
        self.state.theta_r = theta_r; self.state.theta_0 = theta_r;
        self.state.d_0 = d_0;
        self.state.l_0 = 0.0; self.state.r_0 = 0.0;
        self.state.s_0 = 0.0;
    }

    fn odom_tick(&mut self) {
        
    }

    fn kalman_tick(&mut self) {

    }

    pub async fn tracking_loop(&mut self) {
        let tracking_pause = Duration::from_millis(10);
        loop {
            self.odom_tick();
            self.kalman_tick();
            sleep(tracking_pause).await;
        }
    }
}