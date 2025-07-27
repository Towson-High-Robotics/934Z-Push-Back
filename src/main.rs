#![no_main]
#![no_std]
<<<<<<< HEAD
#![feature(slice_as_array)]
#![feature(const_type_id)]
=======

>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
#![feature(duration_millis_float)]

extern crate alloc;

<<<<<<< HEAD
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
=======
use core::{cell:: RefCell, time::Duration};

use alloc::rc::Rc;
use futures::future::join;
use vexide::prelude::*;

pub mod autos;
pub mod conf;
pub mod comp_controller;
pub mod controller;
pub mod gui;
pub mod tracking;
pub mod util;

use autos::*;
use comp_controller::*;
use controller::*;
use tracking::*;
use util::*;

struct CompeteHandler {
    robot: Rc<RefCell<Robot>>,
    comp_cont: CompController
}

impl Robot {
    pub fn auto_tick(&mut self) {

    }

    pub fn driver_tick(&mut self) {
        let cont = self.take_controller().unwrap();
        let state = match cont.state() {
            Ok(s) => s,
            Err(_) => {
                self.return_controller(cont);
                return
            }
        };

        let curved_joysticks = apply_curve(&self.conf, &state);
        let motor_volts = get_drive_volts(&self.conf, curved_joysticks.0, curved_joysticks.1);

        if let Some(drive) = &self.drive {
            drive.left_motors.borrow_mut().iter_mut().for_each(|m| { let _ = m.motor.set_voltage(motor_volts.0); });
            drive.right_motors.borrow_mut().iter_mut().for_each(|m| { let _ = m.motor.set_voltage(motor_volts.1); });
        }

        self.return_controller(cont);
    }
}

impl Compete for CompeteHandler {
    async fn connected(&mut self) {
        self.robot.borrow_mut().connected = true;
    }

    async fn disconnected(&mut self) {
        self.robot.borrow_mut().connected = false;
    }

    async fn disabled(&mut self) {
        
    }

    async fn autonomous(&mut self) {
        loop {
            let mut robot = self.robot.borrow_mut();
            robot.auto_tick();
            drop(robot);
            sleep(Duration::from_millis(10)).await;
        }
    }

    async fn driver(&mut self) {
        loop {
            let mut robot = self.robot.borrow_mut();
            if robot.connected {
                if self.comp_cont.state != CompContState::Driver || self.comp_cont.state != CompContState::SkillsDriver {
                    if self.comp_cont.auto == SelectedAuto::SkillsDriver { self.comp_cont.state = CompContState::SkillsDriver; self.comp_cont.skills_timer.reset(); self.comp_cont.skills_timer.start(); }
                    else { self.comp_cont.state = CompContState::Driver; self.comp_cont.driver_timer.reset(); self.comp_cont.driver_timer.start(); }
                }
                robot.driver_tick();
                if self.comp_cont.state == CompContState::Driver { self.comp_cont.driver_timer.unchecked_update(); }
                else if self.comp_cont.state == CompContState::SkillsDriver { self.comp_cont.skills_timer.unchecked_update(); }
            } else {
                let mut cont = robot.take_controller().unwrap();
                self.comp_cont.controller_handle(&mut cont);
                robot.return_controller(cont);
                self.comp_cont.comp_controller_update(&mut robot);
            }
            drop(robot);
            sleep(Duration::from_millis(25)).await;
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
        }
    }
}

<<<<<<< HEAD
#[vexide::main]
async fn main(peripherals: Peripherals) {
    let dyn_peripherals = DynamicPeripherals::new(peripherals);
    let mut robot = Robot::new(dyn_peripherals);
    robot.update_from_conf();
    robot.compete().await;
=======
use vexide::startup::banner::themes::THEME_TRANS;

#[vexide::main(banner(enabled = true, theme = THEME_TRANS))]
async fn main(peripherals: Peripherals) {
    let mut robot = Robot::new(DynamicPeripherals::new(peripherals));
    robot.drive = Some(Drivetrain::new(&mut robot));

    let robot_cell = Rc::new(RefCell::new(robot));
    let compete = CompeteHandler { robot: robot_cell.clone(), comp_cont: CompController::new() };
    let mut tracking = Tracking::new(robot_cell.clone());
    tracking.calibrate_imu().await;

    join(compete.compete(), tracking.tracking_loop()).await;
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
}