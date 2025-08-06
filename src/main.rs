#![no_main]
#![no_std]
#![feature(duration_millis_float)]

extern crate alloc;

use core::{cell::RefCell, time::Duration};

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
use gui::*;
use tracking::*;
use util::*;

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
            drive.left_motors.borrow_mut().iter_mut().for_each(|m| { let _ = m.motor.set_voltage(motor_volts.0 * m.motor.max_voltage()); });
            drive.right_motors.borrow_mut().iter_mut().for_each(|m| { let _ = m.motor.set_voltage(motor_volts.1 * m.motor.max_voltage()); });
        }

        if let Some(intake) = &self.intake {
            let _ = intake.borrow_mut().set_voltage(
                if state.button_r1.is_pressed() { 0.25 }
                else if state.button_r2.is_pressed() { -0.25 }
                else { 0.0 });
        }

        if let Some(indexer) = &self.indexer {
            let _ = indexer.borrow_mut().motor.set_voltage(
                if state.button_l1.is_pressed() { 0.25 }
                else if state.button_l2.is_pressed() { -0.25 }
                else { 0.0 });
        }

        if let Some(scraper) = &self.scraper {
            if state.button_b.is_now_pressed() {
                let _ = scraper.borrow_mut().toggle();
            }
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
            let mut comp_cont = self.comp_cont.borrow_mut();
            if comp_cont.auto != Autos::Skills { comp_cont.auto_timer.unchecked_update(); }
            else { comp_cont.skills_timer.unchecked_update(); }
            drop(comp_cont);
        }
    }

    async fn driver(&mut self) {
        println!("Running Drive Loop!");
        loop {
            if self.robot.borrow().connected {
                self.robot.borrow_mut().driver_tick();
                let mut comp_cont = self.comp_cont.borrow_mut();
                if comp_cont.auto != Autos::SkillsDriver { comp_cont.driver_timer.unchecked_update(); }
                else { comp_cont.skills_timer.unchecked_update(); }
                drop(comp_cont);
            } else {
                let mut cont = self.robot.borrow_mut().take_controller().unwrap();
                self.comp_cont.borrow_mut().controller_handle(&mut cont).await;
                self.robot.borrow_mut().return_controller(cont);
                self.comp_cont.borrow_mut().comp_controller_update(&mut self.robot.borrow_mut());
            }
            sleep(Duration::from_millis(25)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    println!("Creating Robot");
    let mut robot = Robot::new(DynamicPeripherals::new(peripherals));
    robot.drive = Some(Drivetrain::new(&mut robot));
    robot.intake = Some(Rc::new(RefCell::new(Intake::new(&mut robot))));
    robot.indexer = Some(Rc::new(RefCell::new(NamedMotor::new_exp(robot.take_smart(robot.conf.general.indexer_port).unwrap(), if robot.conf.general.indexer_dir { Direction::Forward } else { Direction::Reverse }, "IND", "Indexer"))));
    robot.scraper = Some(Rc::new(RefCell::new(AdiDigitalOut::new(robot.take_adi(1).unwrap()))));

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