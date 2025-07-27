use alloc::string::{String, ToString};
<<<<<<< HEAD

use serde::{Serialize, Deserialize};
use vexide::{fs, io::ErrorKind, path::Path};

use crate::controller::{ControlSchemes, ControllerCurves};

extern crate alloc;

#[derive(Serialize, Deserialize, Clone, Copy)]
pub(crate) struct DriveConfig {
    pub left_ports: [u8; 3],
    pub right_ports: [u8; 3]
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub(crate) struct OdometryConfig {
    pub inertial_port: u8,
    pub hor_track_port: u8,
    pub hor_track_offset: f64
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub(crate) struct MCLConfig {
    pub dist_ports: [u8; 3],
    pub dist_angles: [f64; 3],
    pub dist_pos: [[f64; 2]; 3]
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub(crate) struct StanleyConfig {

}

#[derive(Serialize, Deserialize, Clone, Copy)]
=======
use serde::{Deserialize, Serialize};
use serde_json::{from_str, to_string};
use vexide::{fs::{read, write}, io::ErrorKind, path::Path};

use crate::controller::{ControllerLayouts, JoystickCurves};

#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
pub(crate) struct GeneralConfig {
    pub left_dt_ports: [u8; 3],
    pub right_dt_ports: [u8; 3]
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub(crate) struct TrackingConfig {
    pub left_wheel_offset: [f64; 3],
    pub right_wheel_offset: [f64; 3],
    pub horizontal_track_port: u8,
    pub horizontal_track_offset: f64,
    pub imu_port: u8,
    pub distance_ports: [u8; 3],
    pub distance_angles: [f64; 3],
    pub distance_offsets: [f64; 3]
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
pub(crate) struct ControllerConfig {
    pub left_deadzone_inner: f64,
    pub left_deadzone_outer: f64,
    pub right_deadzone_inner: f64,
    pub right_deadzone_outer: f64,
<<<<<<< HEAD
    pub control_scheme: ControlSchemes,
    pub controller_curve_type: ControllerCurves
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub(crate) struct Config {
    pub drive_conf: DriveConfig,
    pub odom_conf: OdometryConfig,
    pub mcl_conf: MCLConfig,
    pub stanley_conf: StanleyConfig,
    pub controller_conf: ControllerConfig,
}

pub(crate) struct ConfigWrapper {
    conf_str: String,
    pub conf: Option<Config>
}

impl ConfigWrapper {
    const DEFAULT_JSON: &str = "";

    pub fn new() -> ConfigWrapper {
        let mut this = ConfigWrapper { conf_str: String::new(), conf: None };
        this.parse();
        this
    }

    pub fn parse(&mut self) {
        self.read_config();
        self.conf = match serde_json::from_str::<Config>(&self.conf_str) {
            Ok(a) => Some(a),
            Err(_) => panic!("failed to parse json config")
        }
    }

    pub fn export(&mut self) {
        self.conf_str = serde_json::to_string(&self.conf.as_mut().unwrap()).unwrap_or_default();
        self.write_config();
    }

    fn read_config(&mut self) {
        match fs::read_to_string(Path::new("conf.json")) {
            Ok(str) => { self.conf_str = str; },
            Err(e) => match e.kind() {
                ErrorKind::NotFound => { self.conf_str = Self::DEFAULT_JSON.to_string(); }
                ErrorKind::InvalidInput => panic!("Wrong parameters fool"),
                _ => panic!("Challenge completed: How did we get here?")
            },
        };
    }

    fn write_config(&mut self) {
        match fs::write(Path::new("conf.json"), &self.conf_str) {
            Ok(_) => {},
            Err(e) => match e.kind() {
                ErrorKind::InvalidInput => panic!("Wrong parameters fool"),
                _ => panic!("Challenge completed: How did we get here?")
            }
=======
    pub layout: ControllerLayouts,
    pub curve: JoystickCurves
}

#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
pub(crate) struct GuiConfig {

}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub(crate) struct Config {
    pub general: GeneralConfig,
    pub tracking: TrackingConfig,
    pub controller: ControllerConfig,
    pub gui: GuiConfig
}

const DEFAULT_JSON: &str = "";

impl Config {
    pub fn load() -> Config {
        let file = match read(Path::new("conf.json")) {
            Ok(v) => String::from_utf8(v).unwrap_or(DEFAULT_JSON.to_string()),
            Err(e) => match e.kind() {
                ErrorKind::NotFound => DEFAULT_JSON.to_string(),
                ErrorKind::InvalidInput => panic!(),
                _ => DEFAULT_JSON.to_string(),
            }
        };
        from_str::<Config>(file.as_str()).unwrap_or(from_str::<Config>(DEFAULT_JSON).expect("Incorrect Default JSON"))
    }

    pub fn save(&mut self) {
        match write(Path::new("conf.json"), to_string(&self).unwrap()) {
            Ok(_) => (),
            Err(e) => if e.kind() == ErrorKind::InvalidInput { panic!() },
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
        }
    }
}