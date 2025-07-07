extern crate alloc;

use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::prelude::*;

pub(crate) struct LabeledMotor {
    pub motor: Motor,
    pub name_long: &'static str,
    pub name_short: &'static str,
    pub is_connected: bool,
    pub temperature: Option<f64>,
    pub position: Option<f64>,
}

impl LabeledMotor {
    fn from_vexide(motor: Motor, name_long: &'static str, name_short: &'static str) -> LabeledMotor {
        let mut out =  LabeledMotor {
            motor, name_long, name_short,
            is_connected: false,
            temperature: None,
            position: None
        };
        let connected = out.motor.is_connected();
        out.is_connected = connected;
        if connected{
            out.temperature = Some(out.motor.temperature().unwrap());
            out.position = Some(out.motor.position().unwrap().as_degrees());
        }
        out
    }

    pub fn new(robot: &mut Robot, port: u8, gear: Gearset, dir: Direction, name_long: &'static str, name_short: &'static str) -> LabeledMotor {
        let port = robot.take_port(port);
        if let Some(port) = port {
            LabeledMotor::from_vexide(Motor::new(port, gear, dir), name_long, name_short)
        } else {
            panic!("how the hell did you mess ports up again")
        }
    }
}

pub(crate) struct Drivetrain {
    pub left_motors: Rc<RefCell<[LabeledMotor; 3]>>,
    pub right_motors: Rc<RefCell<[LabeledMotor; 3]>>
}

impl Drivetrain {
    pub fn new(robot: &mut Robot, left_ports: [u8; 3], right_ports: [u8; 3]) -> Drivetrain {
        Drivetrain {
            left_motors: Rc::new( RefCell::new([
                LabeledMotor::new(robot, left_ports[0], Gearset::Blue, Direction::Forward, "Left Front", "LF"),
                LabeledMotor::new(robot, left_ports[1], Gearset::Blue, Direction::Reverse, "Left Middle", "LM"),
                LabeledMotor::new(robot, left_ports[2], Gearset::Blue, Direction::Forward, "Left Back", "LB"),
            ])),
            right_motors: Rc::new( RefCell::new([
                LabeledMotor::new(robot, right_ports[0], Gearset::Blue, Direction::Forward, "Right Front", "RF"),
                LabeledMotor::new(robot, right_ports[1], Gearset::Blue, Direction::Reverse, "Right Middle", "RM"),
                LabeledMotor::new(robot, right_ports[2], Gearset::Blue, Direction::Forward, "Right Back", "RB"),
            ]))
        }
    }
}

pub(crate) struct Robot {
    pub connected: bool,
    peripherals: DynamicPeripherals,
    pub drive: Option<Drivetrain>,
    pub odom: Option<crate::OdomSensors>,
    pub mcl: Option<crate::MCLSensors>,
    port_available: [bool; 21],
    adi_port_available: [bool; 8],
}

impl Robot {
    pub fn new(per: DynamicPeripherals) -> Robot {
        Robot {
            connected: false,
            peripherals: per,
            drive: None,
            odom: None,
            mcl: None,
            port_available: [true; 21],
            adi_port_available: [true; 8]
        }
    }

    pub fn get_display(&mut self) -> Option<Display> { self.peripherals.take_display() }

    pub fn return_display(&mut self, disp: Display) { self.peripherals.return_display(disp); }
    
    pub fn get_cont(&mut self) -> Option<Controller> { self.peripherals.take_primary_controller() }

    pub fn return_cont(&mut self, cont: Controller) { self.peripherals.return_primary_controller(cont); }

    fn take_port(&mut self, port: u8) -> Option<SmartPort> {
        if self.port_available[usize::from(port)] {
            self.port_available[usize::from(port)] = false;
            self.peripherals.take_smart_port(port)
        } else {
            None
        }
    }

    fn take_adi(&mut self, port: u8) -> Option<AdiPort> {
        if self.adi_port_available[usize::from(port)] {
            self.adi_port_available[usize::from(port)] = false;
            self.peripherals.take_adi_port(port)
        } else {
            None
        }
    }
}
