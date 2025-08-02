#![no_main]
#![no_std]
#![feature(duration_millis_float)]

extern crate alloc;

use core::{cell:: RefCell, time::Duration};

use alloc::rc::Rc;
use futures::future::join;
use vexide::{prelude::*, startup::banner::themes::THEME_TRANS};

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

use crate::gui::Gui;

struct CompeteHandler {
    robot: Rc<RefCell<Robot>>,
    comp_cont: Rc<RefCell<CompController>>
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
        println!("Connected to Competition Controller!");
        self.robot.borrow_mut().connected = true;
    }

    async fn disconnected(&mut self) {
        println!("Disconnected from Competition Controller!");
        self.robot.borrow_mut().connected = false;
    }

    async fn disabled(&mut self) {
        println!("Robot Disabled by Competition Controller!");
    }

    async fn autonomous(&mut self) {
        println!("Running the Autonomous Loop!");
        loop {
            let mut robot = self.robot.borrow_mut();
            robot.auto_tick();
            drop(robot);
            sleep(Duration::from_millis(10)).await;
        }
    }

    async fn driver(&mut self) {
        println!("Running Drive Loop!");
        loop {
            let mut robot = self.robot.borrow_mut();
            let mut comp_cont = self.comp_cont.borrow_mut();
            if robot.connected {
                if comp_cont.state != CompContState::Driver || comp_cont.state != CompContState::SkillsDriver {
                    if comp_cont.auto == SelectedAuto::SkillsDriver { comp_cont.state = CompContState::SkillsDriver; comp_cont.skills_timer.reset(); comp_cont.skills_timer.start(); }
                    else { comp_cont.state = CompContState::Driver; comp_cont.driver_timer.reset(); comp_cont.driver_timer.start(); }
                }
                robot.driver_tick();
                if comp_cont.state == CompContState::Driver { comp_cont.driver_timer.unchecked_update(); }
                else if comp_cont.state == CompContState::SkillsDriver { comp_cont.skills_timer.unchecked_update(); }
            } else {
                let mut cont = robot.take_controller().unwrap();
                comp_cont.controller_handle(&mut cont);
                robot.return_controller(cont);
                comp_cont.comp_controller_update(&mut robot);
            }
            drop(comp_cont);
            drop(robot);
            sleep(Duration::from_millis(25)).await;
        }
    }
}

#[vexide::main/* (banner(theme=THEME_TRANS)) */]
async fn main(peripherals: Peripherals) {
    println!("Creating Robot");
    let mut robot = Robot::new(DynamicPeripherals::new(peripherals));
    println!("Initializing Drivetrain");
    robot.drive = Drivetrain::new(&mut robot);

    let robot_cell = Rc::new(RefCell::new(robot));
    println!("Creating Virtual Competition Controller");
    let comp_cont_cell = Rc::new(RefCell::new(CompController::new()));
    let compete = CompeteHandler { robot: robot_cell.clone(), comp_cont: comp_cont_cell.clone() };
    println!("Creating Tracking Devices");
    let mut tracking = Tracking::new(robot_cell.clone());
    println!("Creating GUI Object");
    let mut gui = Gui::new(robot_cell.clone(), comp_cont_cell.clone());

    println!("Calibrating IMU");
    tracking.calibrate_imu().await;
    println!("Running Competition, Tracking, and GUI threads");
    join(join(compete.compete(), tracking.tracking_loop()), gui.render_loop()).await;
}