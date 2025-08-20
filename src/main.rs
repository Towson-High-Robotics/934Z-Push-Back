#![no_main]
#![no_std]
#![feature(duration_millis_float)]

extern crate alloc;

use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use futures::future::join;
use vexide::prelude::*;

pub mod autos;
pub mod comp_controller;
pub mod conf;
pub mod controller;
pub mod gui;
pub mod tracking;
pub mod util;

use autos::*;
use comp_controller::*;
use conf::*;
use controller::*;
use gui::*;
use tracking::*;
use util::*;

struct CompeteHandler {
    robot: Rc<RefCell<Robot>>,
    comp_cont: Rc<RefCell<CompController>>,
}

impl Robot {
    pub fn auto_tick(&mut self) {}

    pub fn driver_tick(&mut self) {
        let cont = match self.cont.try_borrow_mut() {
            Ok(c) => c,
            Err(_) => return,
        };
        let state = match cont.state() {
            Ok(s) => s,
            Err(_) => {
                drop(cont);
                return;
            }
        };

        let curved_joysticks = apply_curve(&self.conf, &state);
        let motor_volts = get_drive_volts(&self.conf, curved_joysticks.0, curved_joysticks.1);

        self.drive.left_motors.borrow_mut().iter_mut().for_each(|m| {
            let _ = m.motor.set_voltage(motor_volts.0 * m.motor.max_voltage());
        });
        self.drive.right_motors.borrow_mut().iter_mut().for_each(|m| {
            let _ = m.motor.set_voltage(motor_volts.1 * m.motor.max_voltage());
        });

        let _ = self.intake.set_voltage(if state.button_r1.is_pressed() {
            1.0
        } else if state.button_r2.is_pressed() {
            -1.0
        } else {
            0.0
        });

        let _ = self.indexer.set_voltage(if state.button_l1.is_pressed() {
            1.0
        } else if state.button_l2.is_pressed() {
            -1.0
        } else {
            0.0
        });

        let _ = self.scraper.toggle();

        drop(cont);
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
            self.robot.borrow_mut().auto_tick();
            sleep(Duration::from_millis(10)).await;
            let mut comp_cont = self.comp_cont.borrow_mut();
            if comp_cont.auto != Autos::Skills {
                comp_cont.auto_timer.unchecked_update();
            } else {
                comp_cont.skills_timer.unchecked_update();
            }
            drop(comp_cont);
        }
    }

    async fn driver(&mut self) {
        println!("Running Drive Loop!");
        loop {
            if self.robot.borrow().connected {
                self.robot.borrow_mut().driver_tick();
                let mut comp_cont = self.comp_cont.borrow_mut();
                if comp_cont.auto != Autos::SkillsDriver {
                    comp_cont.driver_timer.unchecked_update();
                } else {
                    comp_cont.skills_timer.unchecked_update();
                }
                drop(comp_cont);
            } else {
                self.comp_cont
                    .borrow_mut()
                    .controller_handle(&mut self.robot.borrow_mut().cont.borrow_mut())
                    .await;
                self.comp_cont.borrow_mut().comp_controller_update(&mut self.robot.borrow_mut());
            }
            sleep(Duration::from_millis(25)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    println!("Creating Robot");
    let conf = Config::load();
    let mut dyn_peripherals = DynamicPeripherals::new(peripherals);
    let drive = Drivetrain::new(conf, &mut dyn_peripherals);
    let intake = Intake::new(conf, &mut dyn_peripherals);
    let indexer = NamedMotor::new_exp(
        dyn_peripherals.take_smart_port(conf.general.indexer_port).unwrap(),
        if conf.general.indexer_dir {
            Direction::Forward
        } else {
            Direction::Reverse
        },
        "IND",
        "Indexer",
    );
    let scraper = AdiDigitalOut::new(dyn_peripherals.take_adi_port(1).unwrap());

    println!("Creating Tracking Devices");
    let (mut tracking, pose) = Tracking::new(drive.clone(), &mut dyn_peripherals, conf);

    let cont = Rc::new(RefCell::new(dyn_peripherals.take_primary_controller().unwrap()));

    let robot = Robot {
        cont,
        conf,
        drive,
        intake,
        indexer,
        scraper,
        pose,
        connected: false,
    };
    let robot_cell = Rc::new(RefCell::new(robot));
    println!("Creating Virtual Competition Controller");
    let comp_cont_cell = Rc::new(RefCell::new(CompController::new()));
    let compete = CompeteHandler {
        robot: robot_cell.clone(),
        comp_cont: comp_cont_cell.clone(),
    };

    println!("Creating GUI Object");
    let mut gui = Gui::new(robot_cell.clone(), comp_cont_cell.clone(), dyn_peripherals.take_display().unwrap());

    println!("Calibrating IMU");
    tracking.calibrate_imu().await;
    println!("Running Competition, Tracking, and GUI threads");
    join(join(compete.compete(), tracking.tracking_loop()), gui.render_loop()).await;
}
