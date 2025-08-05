use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::{devices::smart::motor::MotorError, prelude::*};

use crate::conf::Config;

#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct NamedMotor {
    pub motor: Motor,
    pub name_short: &'static str,
    pub name_long: &'static str
}

impl NamedMotor {
    pub fn new_v5(port: SmartPort, gear: Gearset, dir: Direction, name_short: &'static str, name_long: &'static str) -> Self {
        Self { 
            motor: Motor::new(port, gear, dir), 
            name_short, name_long
        }
    }

    pub fn new_exp(port: SmartPort, dir: Direction, name_short: &'static str, name_long: &'static str) -> Self {
        Self { motor: Motor::new_exp(port, dir), name_short, name_long }
    }

    pub fn get_pos_degrees(&mut self) -> Option<f64> {
        match self.motor.position() {
            Ok(p) => Some(p.as_degrees()),
            Err(_) => None,
        }
    }

    pub fn get_temp(&mut self) -> Option<f64> { self.motor.temperature().ok() }

    pub fn connected(&mut self) -> bool { self.motor.is_connected() }
}

#[derive(Debug)]
pub(crate) struct TrackingWheel { pub sens: RotationSensor, pub offset: f64 }

#[derive(Debug)]
pub(crate) struct Intake { pub motor_1: NamedMotor, pub motor_2: NamedMotor }

impl Intake {
    pub fn new(robot: &mut Robot) -> Self {
        println!("Attempting to Initialize the Intake!");
        Self {
            motor_1: NamedMotor::new_v5(robot.take_smart(robot.conf.general.intake_ports[0]).unwrap(), Gearset::Blue, if robot.conf.general.intake_dir[0] { Direction::Reverse } else { Direction::Forward }, "IF", "Intake Full"),
            motor_2: NamedMotor::new_exp(robot.take_smart(robot.conf.general.intake_ports[1]).unwrap(), if robot.conf.general.intake_dir[0] { Direction::Reverse } else { Direction::Forward }, "IF", "Intake Full"),
        }
    }

    pub fn set_voltage(&mut self, volts: f64) -> Result<(), MotorError> {
        self.motor_1.motor.set_voltage(volts)?;
        self.motor_2.motor.set_voltage(volts / 2.0)?;
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub(crate) struct Drivetrain {
    pub left_motors: Rc<RefCell<[NamedMotor; 3]>>,
    pub right_motors: Rc<RefCell<[NamedMotor; 3]>>
}

impl Drivetrain {
    pub fn new(robot: &mut Robot) -> Self {
        println!("Attempting to Initialize the Drivetrain!");
        Self {
            left_motors: Rc::new(RefCell::new([
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.left_dt_ports[0]).unwrap(), Gearset::Blue, Direction::Forward, "LF", "Left Forward"),
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.left_dt_ports[1]).unwrap(), Gearset::Blue, Direction::Reverse, "LM", "Left Middle"),
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.left_dt_ports[2]).unwrap(), Gearset::Blue, Direction::Forward, "LB", "Left Back")
            ])),
            right_motors: Rc::new(RefCell::new([
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.right_dt_ports[0]).unwrap(), Gearset::Blue, Direction::Forward, "RF", "Right Forward"),
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.right_dt_ports[1]).unwrap(), Gearset::Blue, Direction::Reverse, "RM", "Right Middle"),
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.right_dt_ports[2]).unwrap(), Gearset::Blue, Direction::Forward, "RB", "Right Back")
            ]))
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct Robot {
    peripherals: Rc<RefCell<DynamicPeripherals>>,
    pub conf: Config,
    pub drive: Option<Drivetrain>,
    pub intake: Option<Rc<RefCell<Intake>>>,
    pub indexer: Option<Rc<RefCell<NamedMotor>>>,
    pub scraper: Option<Rc<RefCell<AdiDigitalOut>>>,
    pub pose: ((f64, f64), f64),
    pub connected: bool,
}

impl Robot {
    pub fn new(peripherals: DynamicPeripherals) -> Self {
        Self { peripherals: Rc::new(RefCell::new(peripherals)), conf: Config::load(), drive: None, intake: None, indexer: None, scraper: None, pose: ((0.0, 0.0), 0.0), connected: false }
    }

    pub fn take_controller(&mut self) -> Option<Controller> { self.peripherals.borrow_mut().take_primary_controller() }

    pub fn return_controller(&mut self, cont: Controller) { self.peripherals.borrow_mut().return_primary_controller(cont); }

    pub fn take_display(&mut self) -> Option<Display> { self.peripherals.borrow_mut().take_display() }

    pub fn _return_display(&mut self, disp: Display) { self.peripherals.borrow_mut().return_display(disp); }

    pub fn take_smart(&mut self, port: u8) -> Option<SmartPort> { if (1..=21).contains(&port) { self.peripherals.borrow_mut().take_smart_port(port) } else { None } }

    pub fn take_adi(&mut self, port: u8) -> Option<AdiPort> { if (1..=8).contains(&port) { self.peripherals.borrow_mut().take_adi_port(port) } else { None } }
}