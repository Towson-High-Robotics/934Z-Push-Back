use alloc::string::{String, ToString};

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
pub(crate) struct ControllerConfig {
    pub left_deadzone_inner: f64,
    pub left_deadzone_outer: f64,
    pub right_deadzone_inner: f64,
    pub right_deadzone_outer: f64,
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
        }
    }
}