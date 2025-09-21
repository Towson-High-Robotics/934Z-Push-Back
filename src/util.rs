use alloc::rc::Rc;
use core::cell::RefCell;

use vexide::{devices::smart::motor::MotorError, prelude::*};

use crate::conf::Config;

#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct NamedMotor {
    pub motor: Motor,
    pub name_short: &'static str,
}

impl NamedMotor {
    pub fn new_v5(port: SmartPort, gear: Gearset, dir: Direction, name_short: &'static str) -> Self {
        Self {
            motor: Motor::new(port, gear, dir),
            name_short,
        }
    }

    pub fn new_exp(port: SmartPort, dir: Direction, name_short: &'static str) -> Self {
        Self {
            motor: Motor::new_exp(port, dir),
            name_short,
        }
    }

    pub fn get_pos_degrees(&mut self) -> f64 {
        match self.motor.position() {
            Ok(p) => p.as_degrees(),
            Err(_) => 0.0,
        }
    }

    pub fn get_temp(&mut self) -> Option<f64> { self.motor.temperature().ok() }

    pub fn connected(&mut self) -> bool { self.motor.is_connected() }

    pub fn set_voltage(&mut self, volt_per: f64) -> Result<(), MotorError> {
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
    pub full_speed: bool,
}

impl Intake {
    pub fn new(conf: Config, peripherals: &mut DynamicPeripherals) -> Self {
        println!("Attempting to Initialize the Intake!");
        Self {
            motor_1: NamedMotor::new_v5(
                peripherals.take_smart_port(conf.general.intake_ports[0]).unwrap(),
                Gearset::Blue,
                if conf.general.intake_dir[0] {
                    Direction::Reverse
                } else {
                    Direction::Forward
                },
                "IF",
            ),
            motor_2: NamedMotor::new_exp(
                peripherals.take_smart_port(conf.general.intake_ports[1]).unwrap(),
                if conf.general.intake_dir[0] {
                    Direction::Reverse
                } else {
                    Direction::Forward
                },
                "IF",
            ),
            full_speed: false,
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct Drivetrain {
    pub left_motors: Rc<RefCell<[NamedMotor; 3]>>,
    pub right_motors: Rc<RefCell<[NamedMotor; 3]>>,
}

impl Drivetrain {
    pub fn new(conf: Config, peripherals: &mut DynamicPeripherals) -> Self {
        println!("Attempting to Initialize the Drivetrain!");
        let left = conf.general.left_dt_ports;
        let right = conf.general.right_dt_ports;
        Self {
            left_motors: Rc::new(RefCell::new([
                NamedMotor::new_v5(peripherals.take_smart_port(left[0]).unwrap(), Gearset::Blue, Direction::Reverse, "LF"),
                NamedMotor::new_v5(peripherals.take_smart_port(left[1]).unwrap(), Gearset::Blue, Direction::Reverse, "LM"),
                NamedMotor::new_v5(peripherals.take_smart_port(left[2]).unwrap(), Gearset::Blue, Direction::Forward, "LB"),
            ])),
            right_motors: Rc::new(RefCell::new([
                NamedMotor::new_v5(peripherals.take_smart_port(right[0]).unwrap(), Gearset::Blue, Direction::Forward, "RF"),
                NamedMotor::new_v5(peripherals.take_smart_port(right[1]).unwrap(), Gearset::Blue, Direction::Forward, "RM"),
                NamedMotor::new_v5(peripherals.take_smart_port(right[2]).unwrap(), Gearset::Blue, Direction::Reverse, "RB"),
            ])),
        }
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct Robot {
    pub cont: Controller,
    pub conf: Config,
    pub drive: Drivetrain,
    pub intake: Intake,
    pub indexer: NamedMotor,
    pub scraper: AdiDigitalOut,
    pub pose: Rc<RefCell<((f64, f64), f64)>>,
}
