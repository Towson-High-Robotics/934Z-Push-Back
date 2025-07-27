#![no_main]
#![no_std]
#![feature(duration_millis_float)]

extern crate alloc;

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
        }
    }
}

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
}