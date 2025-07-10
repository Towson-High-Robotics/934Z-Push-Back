extern crate alloc;

use core::{ cell::RefCell, str::FromStr};

use alloc::{borrow::ToOwned, fmt::format, format, rc::Rc, string::{String, ToString}, vec::Vec};
use vexide::{fs, path::Path, prelude::*};

use crate::{tracking::OdomSensors, util::{Drivetrain, Robot}};

pub(crate) struct ConfObj {
    name: &'static str,
    prop_str: String
}

impl ConfObj {
    fn set_prop_str(&mut self, str: String) { self.prop_str = str; }

    pub fn get_val<T: core::str::FromStr>(&mut self) -> Option<T> { if self.prop_str.contains(self.name) { self.prop_str.split_at(self.name.len()).1.trim().parse::<T>().ok() } else { None } }

    pub fn get_str_val(&mut self) -> Option<String> { if self.prop_str.contains(self.name) { Some(self.prop_str.split_at(self.name.len()).1.trim().to_owned()) } else { None } }

    pub fn get_vec<T: core::str::FromStr>(&mut self) -> Vec<Option<T>> {
        if self.prop_str.contains(self.name) {
            self.prop_str.split_at(self.name.len() - 1).1.split_whitespace().map(|x| x.parse::<T>().ok()).collect()
        } else { Vec::new() }
    }

    pub fn set_value<T: core::fmt::Display>(&mut self, value: T) {
        self.set_prop_str(format!("{} {}", self.name, value));
    }

    pub fn set_vec_value<T: core::fmt::Display>(&mut self, value: Vec<T>) {
        value.iter().map(|x| format!("{x}")).fold(format!("{} ", self.name), |acc, x| format!("{acc} {x}"));
    }

    pub fn new(name: &'static str) -> ConfObj {
        ConfObj { name, prop_str: format!("{name} ") }
    }
}

pub(crate) struct ConfigData<const VALUES: usize> {
    conf_str: String,
    pub data: [ConfObj; VALUES]
}

impl<const VALUES: usize> ConfigData<VALUES> {
    fn update_props(&mut self) {
        for i in self.conf_str.lines() {
            for j in self.data.iter_mut() {
                if i.contains(j.name) {
                    j.set_prop_str(i.to_string());
                    break;
                }
            }
        }
    }

    pub fn read_conf(&mut self) {
        let conf = fs::read(Path::new("conf.txt"));
        let conf_unwrap = match conf {
            Ok(_) => {
                String::from_utf8(conf.unwrap_or_default()).unwrap_or_default()
            },
            Err(_) => match conf.unwrap_err().kind() {
                vexide::io::ErrorKind::NotFound => {
                    let _ = fs::write(Path::new("conf.txt"), "");
                    String::new()
                },
                vexide::io::ErrorKind::InvalidInput => panic!(),
                _ => panic!(),
            },
        };
        self.conf_str = conf_unwrap;
        self.update_props();
    }

    pub fn new(props: [ConfObj; VALUES]) -> ConfigData<VALUES> {
        ConfigData { conf_str: String::new(), data: props }
    }
}