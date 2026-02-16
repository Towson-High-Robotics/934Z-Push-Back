use vexide::{
    prelude::{Gearset, Motor},
    smart::SmartDevice,
};

use crate::autos::auto::Autos;

#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MotorType {
    Disconnected,
    Exp,
    Red,
    Green,
    Blue,
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

impl Telem {
    pub(crate) fn new(motor_names: Vec<&'static str>, sensor_names: Vec<&'static str>) -> Self {
        let motors = motor_names.len();
        let sensors = sensor_names.len();
        Self {
            motor_names,
            motor_temperatures: vec![25.0; motors],
            motor_headings: vec![0.0; motors],
            motor_types: vec![MotorType::Disconnected; motors],
            sensor_names,
            sensor_values: vec![0.0; sensors],
            sensor_status: vec![false; sensors],
            ..Default::default()
        }
    }

    pub(crate) fn update_motor(&mut self, motor: &Motor, index: usize) {
        self.motor_temperatures[index] = motor.temperature().unwrap_or(f64::NAN);
        self.motor_headings[index] = motor.position().unwrap_or_default().as_degrees();
        self.motor_types[index] = if !motor.is_connected() {
            MotorType::Disconnected
        } else if motor.is_exp() {
            MotorType::Exp
        } else {
            match motor.gearset().unwrap() {
                Gearset::Red => MotorType::Red,
                Gearset::Green => MotorType::Green,
                Gearset::Blue => MotorType::Blue,
            }
        }
    }
}
