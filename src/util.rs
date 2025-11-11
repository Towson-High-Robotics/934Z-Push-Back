use std::{
    string::String,
    sync::{nonpoison::RwLock, Arc},
};

use vexide::{
    peripherals::DynamicPeripherals,
    prelude::*,
    smart::{PortError, SmartPort},
};

use crate::{
    autos::{Autos, Chassis},
    comp::CompHandler,
    conf::Config,
    gui::MotorType,
};

#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct NamedMotor {
    pub motor: Motor,
    pub name: String,
}

impl NamedMotor {
    pub fn new_v5(port: SmartPort, gear: Gearset, reverse: bool, name: String) -> Self {
        Self {
            motor: Motor::new(port, gear, if reverse { Direction::Reverse } else { Direction::Forward }),
            name,
        }
    }

    pub fn new_exp(port: SmartPort, reverse: bool, name: String) -> Self {
        Self {
            motor: Motor::new_exp(port, if reverse { Direction::Reverse } else { Direction::Forward }),
            name,
        }
    }

    pub fn get_pos_degrees(&self) -> f64 {
        match self.motor.position() {
            Ok(p) => p.as_degrees(),
            Err(_) => f64::NAN,
        }
    }

    pub fn get_temp(&self) -> f64 { self.motor.temperature().unwrap_or_default() }

    pub fn connected(&self) -> bool { self.motor.is_connected() }

    pub fn set_voltage(&mut self, volt_per: f64) -> Result<(), PortError> {
        self.motor.set_voltage(volt_per * self.motor.max_voltage())?;
        Ok(())
    }
}

#[derive(Debug)]
pub(crate) struct TrackingWheel {
    pub sens: RotationSensor,
    pub offset: f64,
}

#[derive(Debug)]
pub(crate) struct Intake {
    pub motor_1: NamedMotor,
    pub motor_2: NamedMotor,
}

impl Intake {
    pub fn new(conf: &Config, peripherals: &mut DynamicPeripherals) -> Self {
        Self {
            motor_1: NamedMotor::new_v5(peripherals.take_smart_port(conf.ports[6]).unwrap(), Gearset::Blue, conf.reversed[6], conf.names[6].clone()),
            motor_2: NamedMotor::new_exp(peripherals.take_smart_port(conf.ports[7]).unwrap(), conf.reversed[7], conf.names[7].clone()),
        }
    }
}

#[derive(Debug)]
pub(crate) struct Drivetrain {
    pub left_motors: [NamedMotor; 3],
    pub right_motors: [NamedMotor; 3],
}

impl Drivetrain {
    pub fn new(conf: &Config, peripherals: &mut DynamicPeripherals) -> Self {
        let ports = &conf.ports;
        let names = &conf.names;
        let dirs = &conf.reversed;
        Self {
            left_motors: [
                NamedMotor::new_v5(peripherals.take_smart_port(ports[0]).unwrap(), Gearset::Blue, dirs[0], names[0].clone()),
                NamedMotor::new_v5(peripherals.take_smart_port(ports[1]).unwrap(), Gearset::Blue, dirs[1], names[1].clone()),
                NamedMotor::new_v5(peripherals.take_smart_port(ports[2]).unwrap(), Gearset::Blue, dirs[2], names[2].clone()),
            ],
            right_motors: [
                NamedMotor::new_v5(peripherals.take_smart_port(ports[3]).unwrap(), Gearset::Blue, dirs[3], names[3].clone()),
                NamedMotor::new_v5(peripherals.take_smart_port(ports[4]).unwrap(), Gearset::Blue, dirs[4], names[4].clone()),
                NamedMotor::new_v5(peripherals.take_smart_port(ports[5]).unwrap(), Gearset::Blue, dirs[5], names[5].clone()),
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
}

#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct Robot {
    pub cont: Controller,
    pub conf: Config,
    pub drive: Drivetrain,
    pub intake: Intake,
    pub indexer: NamedMotor,
    pub matchload: AdiDigitalOut,
    pub descore: AdiDigitalOut,
    pub chassis: Chassis,
    pub comp: CompHandler,
    pub telem: Arc<RwLock<Telem>>,
}
