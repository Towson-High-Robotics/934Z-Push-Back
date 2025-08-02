use alloc::string::{String, ToString};
use serde::{Deserialize, Serialize};
use serde_json::{from_str, to_string};
use vexide::{fs::{read, write}, io::{println, ErrorKind}, path::Path};

use crate::controller::{ControllerLayouts, JoystickCurves};

#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
pub(crate) struct GeneralConfig {
    pub left_dt_ports: [u8; 3],
    pub right_dt_ports: [u8; 3]
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub(crate) struct TrackingConfig {
    pub left_wheel_offset: f64,
    pub right_wheel_offset: f64,
    pub horizontal_track_port: u8,
    pub horizontal_track_offset: f64,
    pub imu_port: u8,
    pub distance_ports: [u8; 3],
    pub distance_angles: [f64; 3],
    pub distance_offsets: [f64; 3]
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub(crate) struct ControllerConfig {
    pub left_deadzone_inner: f64,
    pub left_deadzone_outer: f64,
    pub right_deadzone_inner: f64,
    pub right_deadzone_outer: f64,
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

const DEFAULT_JSON: &str = "{
    \"general\": {
        \"left_dt_ports\": [ 1, 2, 3 ],
        \"right_dt_ports\": [ 11, 12, 13 ]
    },
    \"tracking\": {
        \"left_wheel_offset\": 0.0,
        \"right_wheel_offset\": 0.0,
        \"horizontal_track_port\": 4,
        \"horizontal_track_offset\": 0.0,
        \"imu_port\": 5,
        \"distance_ports\": [ 6, 7, 8 ],
        \"distance_angles\": [ 0.0, 0.0, 0.0 ],
        \"distance_offsets\": [ 0.0, 0.0, 0.0 ]
    },
    \"controller\": {
        \"left_deadzone_inner\": 0.0,
        \"left_deadzone_outer\": 1.0,
        \"right_deadzone_inner\": 0.0,
        \"right_deadzone_outer\": 1.0,
        \"layout\": \"Tank\",
        \"curve\": \"Linear\"
    },
    \"gui\": {}
}";

impl Config {
    pub fn load() -> Config {
        println!("Attempting to load Config!");
        let file = match read(Path::new("conf.json")) {
            Ok(v) => String::from_utf8(v).unwrap_or(DEFAULT_JSON.to_string()),
            Err(e) => match e.kind() {
                ErrorKind::NotFound => {
                    println!("Config file not found, loading defaults. Is the SD Card inserted?");
                    DEFAULT_JSON.to_string()
                },
                ErrorKind::InvalidInput => panic!(),
                _ => DEFAULT_JSON.to_string(),
            }
        };
        println!("Parsing JSON!");
        from_str::<Config>(file.as_str()).unwrap_or(from_str::<Config>(DEFAULT_JSON).expect("Incorrect Default JSON"))
    }

    pub fn save(&mut self) {
        match write(Path::new("conf.json"), to_string(&self).unwrap()) {
            Ok(_) => (),
            Err(e) => if e.kind() == ErrorKind::InvalidInput { panic!() },
        }
    }
}