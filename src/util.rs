<<<<<<< HEAD
extern crate alloc;

use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::prelude::*;

use crate::{comp_controller::VirtCompContState, conf::ConfigWrapper, tracking::{DistSensors, OdomSensors, TrackingWheel}};

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

#[derive(Clone)]
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
=======
use core::cell::RefCell;

use alloc::rc::Rc;
use vexide::prelude::{AdiPort, Controller, Direction, Display, DynamicPeripherals, Gearset, Motor, RotationSensor, SmartPort};

use crate::conf::Config;

#[derive(Debug)]
pub(crate) struct NamedMotor {
    pub motor: Motor,
    pub name_short: &'static str,
    pub name_long: &'static str
}

impl NamedMotor {
    pub fn new_v5(port: SmartPort, gear: Gearset, dir: Direction, name_short: &'static str, name_long: &'static str) -> NamedMotor {
        NamedMotor { 
            motor: Motor::new(port, gear, dir), 
            name_short, name_long
        }
    }

    pub fn new_exp(port: SmartPort, dir: Direction, name_short: &'static str, name_long: &'static str) -> NamedMotor {
        NamedMotor { motor: Motor::new_exp(port, dir), name_short, name_long }
    }

    pub fn get_pos_degrees(&mut self) -> Option<f64> {
        match self.motor.position() {
            Ok(p) => Some(p.as_degrees()),
            Err(_) => None,
        }
    }

    pub fn get_temp(&mut self) -> Option<f64> {
        self.motor.temperature().ok()
    }
}

#[derive(Debug)]
pub(crate) struct TrackingWheel {
    pub sens: RotationSensor,
    pub offset: f64
}

#[derive(Debug, Clone)]
pub(crate) struct Drivetrain {
    pub left_motors: Rc<RefCell<[NamedMotor; 3]>>,
    pub right_motors: Rc<RefCell<[NamedMotor; 3]>>
}

impl Drivetrain {
    pub fn new(robot: &mut Robot) -> Drivetrain {
        Drivetrain {
            left_motors: Rc::new(RefCell::new([
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.left_dt_ports[0]).unwrap(), Gearset::Blue, Direction::Forward, "LF", "Left Forward"),
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.left_dt_ports[1]).unwrap(), Gearset::Blue, Direction::Reverse, "LM", "Left Middle"),
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.left_dt_ports[2]).unwrap(), Gearset::Blue, Direction::Forward, "LB", "Left Back")
            ])),
            right_motors: Rc::new(RefCell::new([
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.right_dt_ports[0]).unwrap(), Gearset::Blue, Direction::Forward, "RF", "Right Forward"),
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.right_dt_ports[1]).unwrap(), Gearset::Blue, Direction::Reverse, "RM", "Right Middle"),
                NamedMotor::new_v5(robot.take_smart(robot.conf.general.right_dt_ports[2]).unwrap(), Gearset::Blue, Direction::Forward, "RB", "Right Back")
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
            ]))
        }
    }
}

<<<<<<< HEAD
pub(crate) struct Robot {
    pub connected: bool,
    peripherals: DynamicPeripherals,
    pub drive: Option<Drivetrain>,
    pub odom: Option<crate::OdomSensors>,
    pub mcl: Option<crate::DistSensors>,
    port_available: [bool; 21],
    adi_port_available: [bool; 8],
    pub conf: ConfigWrapper,
    pub cont: VirtCompContState
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
            adi_port_available: [true; 8],
            conf: ConfigWrapper::new(),
            cont: VirtCompContState::new()
        }
    }

    pub fn get_display(&mut self) -> Option<Display> { self.peripherals.take_display() }

    pub fn return_display(&mut self, disp: Display) { self.peripherals.return_display(disp); }
    
    pub fn get_cont(&mut self) -> Option<Controller> { self.peripherals.take_primary_controller() }

    pub fn return_cont(&mut self, cont: Controller) { self.peripherals.return_primary_controller(cont); }

    pub fn take_port(&mut self, port: u8) -> Option<SmartPort> {
        if self.port_available[usize::from(port)] {
            self.port_available[usize::from(port)] = false;
            self.peripherals.take_smart_port(port)
        } else {
            None
        }
    }

    pub fn take_adi(&mut self, port: u8) -> Option<AdiPort> {
        if self.adi_port_available[usize::from(port)] {
            self.adi_port_available[usize::from(port)] = false;
            self.peripherals.take_adi_port(port)
        } else {
            None
        }
    }

    pub fn update_from_conf(&mut self) {
        let conf = self.conf.conf.as_mut().cloned().unwrap();
        self.drive = Some(Drivetrain::new(self, conf.drive_conf.left_ports, conf.drive_conf.right_ports));
        self.odom = Some(OdomSensors::new(
            self.drive.clone().unwrap(), 
            InertialSensor::new(
                self.take_port(conf.odom_conf.inertial_port).expect("Inertial port number does not exist")
            ), TrackingWheel {
                sens: RotationSensor::new(
                    self.take_port(conf.odom_conf.hor_track_port).expect("The port for the horizontal tracking wheel's rotation sensor doesn't exist"),
                    Direction::Forward
                ),
                offset: conf.odom_conf.hor_track_offset
            })
        );
        self.mcl = Some(DistSensors::new(self, conf.mcl_conf.dist_ports, conf.mcl_conf.dist_angles, conf.mcl_conf.dist_pos));
    }
=======
#[derive(Debug, Clone)]
pub(crate) struct Robot {
    peripherals: Rc<RefCell<DynamicPeripherals>>,
    pub conf: Config,
    pub drive: Option<Drivetrain>,
    pub connected: bool,
}

impl Robot {
    pub fn new(peripherals: DynamicPeripherals) -> Robot {
        Robot { peripherals: Rc::new(RefCell::new(peripherals)), drive: None, conf: Config::load(), connected: false }
    }

    pub fn take_controller(&mut self) -> Option<Controller> { self.peripherals.borrow_mut().take_primary_controller() }

    pub fn return_controller(&mut self, cont: Controller) { self.peripherals.borrow_mut().return_primary_controller(cont); }

    pub fn take_display(&mut self) -> Option<Display> { self.peripherals.borrow_mut().take_display() }

    pub fn return_display(&mut self, disp: Display) { self.peripherals.borrow_mut().return_display(disp); }

    pub fn take_smart(&mut self, port: u8) -> Option<SmartPort> { if (1..=21).contains(&port) { self.peripherals.borrow_mut().take_smart_port(port) } else { None } }

    pub fn take_adi(&mut self, port: u8) -> Option<AdiPort> { if (1..=8).contains(&port) { self.peripherals.borrow_mut().take_adi_port(port) } else { None } }
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
}