use std::sync::{nonpoison::RwLock, Arc};

use vexide::{peripherals::DynamicPeripherals, prelude::*, smart::PortError};

use crate::{autos::chassis::Chassis, comp::AutoHandler, conf::Config, telemetry::Telem};

#[derive(Debug)]
pub(crate) struct TrackingWheel {
    pub sens: RotationSensor,
    pub offset: f64,
}

#[derive(Debug)]
pub(crate) struct Intake {
    pub motor_1: Motor,
    pub motor_2: Motor,
    pub last_voltage: f64,
    pub m1_stalled: bool,
    pub m2_stalled: bool,
}

impl Intake {
    pub fn new(conf: &Config, peripherals: &mut DynamicPeripherals) -> Self {
        Self {
            motor_1: Motor::new(peripherals.take_smart_port(conf.ports[6]).unwrap(), Gearset::Blue, if conf.reversed[6] { Direction::Reverse } else { Direction::Forward }),
            motor_2: Motor::new_exp(peripherals.take_smart_port(conf.ports[7]).unwrap(), if conf.reversed[7] { Direction::Reverse } else { Direction::Forward }),
            last_voltage: 0.0,
            m1_stalled: false,
            m2_stalled: false,
        }
    }

    pub fn set_voltage(&mut self, volts_per: f64) -> Result<(), PortError> {
        self.last_voltage = volts_per;
        if volts_per == 0.0 || self.last_voltage == 0.0 || self.last_voltage.signum() != volts_per.signum() {
            self.reset();
        }

        if self.m1_stalled || (self.motor_1.efficiency().unwrap_or(0.0) <= 0.05 && self.motor_1.velocity().unwrap_or_default() >= 478.5) {
            self.m1_stalled = true;
            self.motor_1.set_voltage(0.0)?
        } else {
            self.motor_1.set_voltage(volts_per * self.motor_1.max_voltage())?
        }

        if self.m2_stalled || (self.motor_2.efficiency().unwrap_or(0.0) <= 0.05 && self.motor_2.velocity().unwrap_or_default() >= 162.5) {
            self.m2_stalled = true;
            self.motor_2.set_voltage(0.0)
        } else {
            self.motor_2.set_voltage(volts_per * self.motor_2.max_voltage())
        }
    }

    pub fn reset(&mut self) { self.m1_stalled = false; self.m2_stalled = false; }
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

pub fn dot(v1: (f64, f64), v2: (f64, f64)) -> f64 { v1.0 * v2.0 + v1.1 * v2.1 }
