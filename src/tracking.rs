extern crate alloc;

use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::prelude::*;

use crate::util::{Drivetrain, Robot};

pub(crate) struct TrackingWheel {
    pub sens: RotationSensor,
    pub offset: f64
}

pub(crate) struct OdomSensors {
    pub left_motors: Rc<RefCell<[crate::LabeledMotor; 3]>>,
    pub right_motors: Rc<RefCell<[crate::LabeledMotor; 3]>>,
    pub hor_track: TrackingWheel,
    pub inertial: InertialSensor,
    pub last_reset_angle: f64,
    pub last_angle: f64,
    pub last_position: [f64; 2]
}

impl OdomSensors {
    pub fn new(drive: Drivetrain, inertial: InertialSensor, h_track: TrackingWheel) -> OdomSensors {
        OdomSensors {
            left_motors: drive.left_motors.clone(),
            right_motors: drive.right_motors.clone(),
            hor_track: h_track,
            inertial,
            last_reset_angle: 0.0,
            last_angle: 0.0,
            last_position: [0.0, 0.0] 
        }
    }
}

pub(crate) struct DistSensors {
    dist_sensors: [DistanceSensor; 3],
    dist_angles: [f64; 3],
    dist_pos: [[f64; 2]; 3]
}

impl DistSensors {
    pub fn new(robot: &mut Robot, dist_ports: [u8; 3], dist_angles: [f64; 3], dist_pos: [[f64; 2]; 3]) -> DistSensors {
        DistSensors {
            dist_sensors: [
                DistanceSensor::new(robot.take_port(dist_ports[0]).expect("guh")),
                DistanceSensor::new(robot.take_port(dist_ports[1]).expect("guh")),
                DistanceSensor::new(robot.take_port(dist_ports[2]).expect("guh"))
            ],
            dist_angles, dist_pos
        }
    }
}