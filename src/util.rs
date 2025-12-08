use std::sync::{nonpoison::RwLock, Arc};

use vexide::{peripherals::DynamicPeripherals, prelude::*};

use crate::{
    autos::{
        auto::Autos,
        chassis::Chassis,
    },
    comp::AutoHandler,
    conf::Config,
    gui::MotorType,
};

#[derive(Debug)]
pub(crate) struct TrackingWheel {
    pub sens: RotationSensor,
    pub _offset: f64,
}

#[derive(Debug)]
pub(crate) struct Intake {
    pub motor_1: Motor,
    pub motor_2: Motor,
}

impl Intake {
    pub fn new(conf: &Config, peripherals: &mut DynamicPeripherals) -> Self {
        Self {
            motor_1: Motor::new(peripherals.take_smart_port(conf.ports[6]).unwrap(), Gearset::Blue, if conf.reversed[6] { Direction::Reverse } else { Direction::Forward }),
            motor_2: Motor::new_exp(peripherals.take_smart_port(conf.ports[7]).unwrap(), if conf.reversed[7] { Direction::Reverse } else { Direction::Forward }),
        }
    }
}

#[derive(Debug)]
pub(crate) struct Drivetrain {
    pub left_motors: [Motor; 3],
    pub right_motors: [Motor; 3],
}

impl Drivetrain {
    pub fn new(conf: &Config, peripherals: &mut DynamicPeripherals) -> Self {
        let ports = &conf.ports;
        let dirs = &conf.reversed;
        Self {
            left_motors: [
                Motor::new(peripherals.take_smart_port(ports[0]).unwrap(), Gearset::Blue, if dirs[0] { Direction::Reverse } else { Direction::Forward }),
                Motor::new(peripherals.take_smart_port(ports[1]).unwrap(), Gearset::Blue, if dirs[1] { Direction::Reverse } else { Direction::Forward }),
                Motor::new(peripherals.take_smart_port(ports[2]).unwrap(), Gearset::Blue, if dirs[2] { Direction::Reverse } else { Direction::Forward }),
            ],
            right_motors: [
                Motor::new(peripherals.take_smart_port(ports[3]).unwrap(), Gearset::Blue, if dirs[3] { Direction::Reverse } else { Direction::Forward }),
                Motor::new(peripherals.take_smart_port(ports[4]).unwrap(), Gearset::Blue, if dirs[4] { Direction::Reverse } else { Direction::Forward }),
                Motor::new(peripherals.take_smart_port(ports[5]).unwrap(), Gearset::Blue, if dirs[5] { Direction::Reverse } else { Direction::Forward }),
            ],
        }
    }
}

#[derive(Default, Debug, Clone)]
pub(crate) struct Telem {
    pub pose: (f64, f64, f64) = (0.0, 0.0, 0.0),
    pub motor_names: Vec<&'static str> = vec![],
    pub motor_temperatures: Vec<f64> = vec![],
    pub motor_headings: Vec<f64> = vec![],
    pub motor_types: Vec<MotorType> = vec![],
    pub sensor_names: Vec<&'static str> = vec![],
    pub sensor_values: Vec<f64> = vec![],
    pub sensor_status: Vec<bool> = vec![],
    pub offsets: (f64, f64) = (0.0, 0.0),
    pub auto: Autos = Autos::None,
    pub selector_active: bool = false,
    pub update_requested: bool = false
}

#[allow(dead_code)]
pub(crate) struct Robot {
    pub cont: Controller,
    pub conf: Config,
    pub drive: Arc<RwLock<Drivetrain>>,
    pub intake: Intake,
    pub indexer: Motor,
    pub matchload: AdiDigitalOut,
    pub descore: AdiDigitalOut,
    pub chassis: Chassis,
    pub comp: AutoHandler,
    pub telem: Arc<RwLock<Telem>>,
}

pub fn mag(v: (f64, f64)) -> f64 { v.0.hypot(v.1) }

pub fn norm(v: (f64, f64), s: f64) -> (f64, f64) {
    let l = mag(v);
    (v.0 / l * s, v.1 / l * s)
}

pub fn dot(v1: (f64, f64), v2: (f64, f64)) -> f64 {
    v1.0 * v2.0 + v1.1 * v2.1
}