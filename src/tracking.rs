extern crate alloc;

use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::prelude::*;

pub(crate) struct TrackingWheel {
    pub sens: RotationSensor,
    pub offset: f64
}

pub(crate) struct OdomSensors {
    pub left_motors: Rc<RefCell<[crate::LabeledMotor; 3]>>,
    pub right_motors: Rc<RefCell<[crate::LabeledMotor; 3]>>,
    pub hor_track: Option<TrackingWheel>,
    pub inertial: InertialSensor,
    pub last_reset_angle: f64,
    pub last_angle: f64,
    pub last_position: [f64; 2]
}

pub(crate) struct MCLSensors {
    pub left_dist: DistanceSensor,
    pub right_dist: DistanceSensor,
    pub back_dist: DistanceSensor,
}
