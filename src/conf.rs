use std::string::{String, ToString};

use serde::{Deserialize, Serialize};
use serde_json::{from_str, to_string};
use std::{println, 
    fs::{read, write},
    io::{ErrorKind},
    path::Path,
};

use crate::controller::JoystickCurves;

#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub(crate) struct ControllerConfig {
    pub left_deadzone_inner: f64,
    pub left_deadzone_outer: f64,
    pub right_deadzone_inner: f64,
    pub right_deadzone_outer: f64,
    pub curve: JoystickCurves,
}

#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub(crate) struct Config {
    pub ports: [u8; 15],
    pub names: [String; 15],
    pub reversed: [bool; 11],
    pub offsets: [f64; 8],
    pub controller: ControllerConfig,
}

const DEFAULT_JSON: &str = "{
    \"ports\": [ 1, 2, 3, 11, 12, 13, 4, 5, 6, 14, 15, 16, 7, 8, 9 ],
    \"names\": [ \"LF\", \"LM\", \"LB\", \"RF\", \"RM\", \"RB\", \"IF\", \"IH\", \"IND\", \"HT\", \"VT\", \"IMU\", \"DS1\", \"DS2\", \"DS3\" ],
    \"reversed\": [ true, true, false, false, false, true, false, false, false, false, false ],
    \"offsets\": [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
    \"controller\": {
        \"left_deadzone_inner\": 0.0,
        \"left_deadzone_outer\": 1.0,
        \"right_deadzone_inner\": 0.0,
        \"right_deadzone_outer\": 1.0,
        \"curve\": \"Linear\"
    }
}";

impl Config {
    pub fn load() -> Config {
        println!("Attempting to load Config!");
        let file = match read(Path::new("conf.json")) {
            Ok(v) => String::from_utf8(v).unwrap_or(DEFAULT_JSON.to_string()),
            Err(e) => match e.kind() {
                ErrorKind::NotFound => {
                    println!("Config file not found!");
                    DEFAULT_JSON.to_string()
                }
                ErrorKind::InvalidInput => panic!(),
                _ => DEFAULT_JSON.to_string(),
            },
        };
        println!("Parsing JSON!");
        from_str::<Config>(file.as_str()).unwrap_or(from_str::<Config>(DEFAULT_JSON).expect("Incorrect Default JSON"))
    }

    pub fn _save(&mut self) {
        match write(Path::new("conf.json"), to_string(&self).unwrap()) {
            Ok(_) => (),
            Err(e) => {
                if e.kind() == ErrorKind::InvalidInput {
                    panic!()
                }
            }
        }
    }
}
