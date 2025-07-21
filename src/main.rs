#![no_main]
#![no_std]
#![feature(slice_as_array)]
#![feature(const_type_id)]
#![feature(duration_millis_float)]

extern crate alloc;

use vexide::prelude::*;

pub mod util;
use util::*;

pub mod tracking;
use tracking::*;

use crate::comp_controller::{Auto, CompContState};

pub mod conf;

pub mod controller;

pub mod comp_controller;

impl Robot {
    pub fn driver_tick(&mut self) {
        let cont = self.get_cont().unwrap();
        let cont_conf = self.conf.conf.as_mut().cloned().unwrap().controller_conf;
        let cont_state = match cont.state() {
            Ok(state) => state,
            Err(_) => { self.return_cont(cont); return }
        };
        let (left, right) = controller::apply_curve(
            cont_conf, 
            (cont_state.left_stick.x(), cont_state.left_stick.y()),
            (cont_state.right_stick.x(), cont_state.right_stick.y())
        );
        let (left_volt, right_volt) = controller::get_drive_volts(cont_conf, left, right);
        self.drive.as_ref().unwrap().left_motors.borrow_mut().iter_mut().for_each(|x: &mut LabeledMotor| match x.motor.set_voltage(left_volt) { Ok(_) => {}, Err(_) => panic!() });
        self.drive.as_ref().unwrap().right_motors.borrow_mut().iter_mut().for_each(|x: &mut LabeledMotor| match x.motor.set_voltage(right_volt) { Ok(_) => {}, Err(_) => panic!() });
        self.return_cont(cont);
    }

    pub fn auto_tick(&mut self) {

    }
}

impl Compete for Robot {
    async fn connected(&mut self) { self.connected = true; }
    async fn disconnected(&mut self) { self.connected = false; }
    async fn disabled(&mut self) { self.cont.state = CompContState::Disabled; }
    async fn driver(&mut self) {
        loop {
            if self.connected {
                if self.cont.state != CompContState::Driver || self.cont.state != CompContState::SkillsDriver {
                    if self.cont.selected_auto == Auto::SkillsDriver { self.cont.state = CompContState::SkillsDriver; self.cont.skills_counter.start(); }
                    else { self.cont.state = CompContState::Driver; self.cont.driver_counter.start(); }
                }

                self.driver_tick();

                if self.cont.state == CompContState::Driver { self.cont.driver_counter.update(); }
                else if self.cont.state == CompContState::SkillsDriver { self.cont.skills_counter.update(); }
            } else {
                let cont = self.get_cont().unwrap();
                let cont_state = match cont.state() {
                    Ok(state) => state,
                    Err(_) => { self.return_cont(cont); return }
                };
                self.cont.cont_keybinds(cont_state);
                self.return_cont(cont);
                self.comp_cont_handle();
            }
        }
    }
    async fn autonomous(&mut self) { 
        loop {
            if self.cont.state != CompContState::Auto || self.cont.state != CompContState::SkillsAuto {
                if self.cont.selected_auto == Auto::Skills { self.cont.state = CompContState::SkillsAuto; self.cont.skills_counter.start(); }
                else { self.cont.state = CompContState::Auto; self.cont.auto_counter.start(); }
            }

            self.auto_tick();

            if self.cont.state == CompContState::Auto { self.cont.auto_counter.update(); }
            else if self.cont.state == CompContState::SkillsAuto { self.cont.skills_counter.update(); }
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let dyn_peripherals = DynamicPeripherals::new(peripherals);
    let mut robot = Robot::new(dyn_peripherals);
    robot.update_from_conf();
    robot.compete().await;
}