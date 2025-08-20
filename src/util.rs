use alloc::rc::Rc;
use core::cell::RefCell;

use vexide::{devices::smart::motor::MotorError, prelude::*};

use crate::conf::Config;

#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct NamedMotor {
    pub motor: Motor,
    pub name_short: &'static str,
    pub name_long: &'static str,
}

impl NamedMotor {
    pub fn new_v5(port: SmartPort, gear: Gearset, dir: Direction, name_short: &'static str, name_long: &'static str) -> Self {
        Self {
            motor: Motor::new(port, gear, dir),
            name_short,
            name_long,
        }
    }

    pub fn new_exp(port: SmartPort, dir: Direction, name_short: &'static str, name_long: &'static str) -> Self {
        Self {
            motor: Motor::new_exp(port, dir),
            name_short,
            name_long,
        }
    }

    pub fn get_pos_degrees(&mut self) -> Option<f64> {
        match self.motor.position() {
            Ok(p) => Some(p.as_degrees()),
            Err(_) => None,
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
                "Intake Full",
            ),
            motor_2: NamedMotor::new_exp(
                peripherals.take_smart_port(conf.general.intake_ports[1]).unwrap(),
                if conf.general.intake_dir[0] {
                    Direction::Reverse
                } else {
                    Direction::Forward
                },
                "IF",
                "Intake Full",
            ),
        }
    }

    pub fn set_voltage(&mut self, volt_per: f64) -> Result<(), MotorError> {
        self.motor_1.set_voltage(volt_per)?;
        self.motor_2.set_voltage(volt_per)?;
        Ok(())
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
        Self {
            left_motors: Rc::new(RefCell::new([
                NamedMotor::new_v5(
                    peripherals.take_smart_port(conf.general.left_dt_ports[0]).unwrap(),
                    Gearset::Blue,
                    Direction::Reverse,
                    "LF",
                    "Left Forward",
                ),
                NamedMotor::new_v5(
                    peripherals.take_smart_port(conf.general.left_dt_ports[1]).unwrap(),
                    Gearset::Blue,
                    Direction::Reverse,
                    "LM",
                    "Left Middle",
                ),
                NamedMotor::new_v5(
                    peripherals.take_smart_port(conf.general.left_dt_ports[2]).unwrap(),
                    Gearset::Blue,
                    Direction::Forward,
                    "LB",
                    "Left Back",
                ),
            ])),
            right_motors: Rc::new(RefCell::new([
                NamedMotor::new_v5(
                    peripherals.take_smart_port(conf.general.right_dt_ports[0]).unwrap(),
                    Gearset::Blue,
                    Direction::Forward,
                    "RF",
                    "Right Forward",
                ),
                NamedMotor::new_v5(
                    peripherals.take_smart_port(conf.general.right_dt_ports[1]).unwrap(),
                    Gearset::Blue,
                    Direction::Forward,
                    "RM",
                    "Right Middle",
                ),
                NamedMotor::new_v5(
                    peripherals.take_smart_port(conf.general.right_dt_ports[2]).unwrap(),
                    Gearset::Blue,
                    Direction::Reverse,
                    "RB",
                    "Right Back",
                ),
            ])),
        }
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct Robot {
    pub cont: Rc<RefCell<Controller>>,
    pub conf: Config,
    pub drive: Drivetrain,
    pub intake: Intake,
    pub indexer: NamedMotor,
    pub scraper: AdiDigitalOut,
    pub pose: Rc<RefCell<((f64, f64), f64)>>,
    pub connected: bool,
}
