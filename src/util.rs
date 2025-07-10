extern crate alloc;

use core::cell::RefCell;

use alloc::{borrow::ToOwned, rc::Rc, vec::Vec};
use vexide::prelude::*;

use crate::{conf::{ConfObj, ConfigData}, tracking::{DistSensors, OdomSensors, TrackingWheel}};

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
            ]))
        }
    }
}

pub(crate) struct Robot {
    pub connected: bool,
    peripherals: DynamicPeripherals,
    pub drive: Option<Drivetrain>,
    pub odom: Option<crate::OdomSensors>,
    pub mcl: Option<crate::DistSensors>,
    port_available: [bool; 21],
    adi_port_available: [bool; 8],
    pub conf: ConfigData<9>
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
            conf: ConfigData::new([
                ConfObj::new("drive_left_ports"),
                ConfObj::new("drive_right_ports"),
                ConfObj::new("inertial_port"),
                ConfObj::new("h_track_port"),
                ConfObj::new("h_track_offset"),
                ConfObj::new("dist_ports"),
                ConfObj::new("dist_angles"),
                ConfObj::new("dist_pos_x"),
                ConfObj::new("dist_pos_y"),
            ])
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

    fn all_present<T: core::default::Default + core::marker::Copy>(l: &[Option<T>; 3]) -> ([T; 3], bool) {
        let mut out: ([T; _], bool) = ([Default::default(); 3], true);
        for i in l.iter().enumerate() {
            match i.1 {
                Some(v) => { out.0[i.0] = *v; },
                None => { out.1 = false; break; }
            }
        }
        out
    }

    pub fn update_from_conf(&mut self) {
        let drive_left = Robot::all_present::<u8>(self.conf.data[0].get_vec::<u8>().as_array().expect("guh"));
        let drive_right = Robot::all_present::<u8>(self.conf.data[1].get_vec::<u8>().as_array().expect("guh"));

        if drive_left.1 && drive_right.1 {
            self.drive = Some(Drivetrain::new(self, drive_left.0, drive_right.0));
        }

        let inertial_port = self.conf.data[2].get_val::<u8>();
        let h_track_port = self.conf.data[3].get_val::<u8>();
        let h_track_offset = self.conf.data[4].get_val::<f64>();

        if inertial_port.is_some() && h_track_port.is_some() && h_track_offset.is_some() && self.drive.is_some() {
            self.odom = Some(OdomSensors::new(
                self.drive.clone().expect("guh"), 
                InertialSensor::new(
                    self.take_port(inertial_port.unwrap_or_default()).expect("guh")
                ), 
                TrackingWheel {
                    sens: RotationSensor::new(
                        self.take_port(h_track_port.unwrap_or_default()).expect("guh"),
                        Direction::Forward
                    ),
                    offset: h_track_offset.unwrap_or_default()
                }
            ));
        }

        let distance_ports = Robot::all_present::<u8>(self.conf.data[5].get_vec::<u8>().as_array().expect("guh"));
        let distance_angles = Robot::all_present::<f64>(self.conf.data[6].get_vec::<f64>().as_array().expect("guh"));
        let distance_pos_x = Robot::all_present::<f64>(self.conf.data[7].get_vec::<f64>().as_array().expect("guh"));
        let distance_pos_y = Robot::all_present::<f64>(self.conf.data[8].get_vec::<f64>().as_array().expect("guh"));

        if distance_ports.1 && distance_angles.1 && distance_pos_x.1 && distance_pos_y.1 {
            self.mcl = Some(DistSensors::new(self, distance_ports.0, distance_angles.0, distance_pos_x.0, distance_pos_y.0));
        }
    }
}
