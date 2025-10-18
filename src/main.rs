#![feature(duration_millis_float)]

use std::{cell::RefCell, rc::Rc};

use futures::future::join;
use vexide::{controller::ControllerState, peripherals::DynamicPeripherals, prelude::*};

pub mod autos;
pub mod conf;
pub mod controller;
pub mod gui;
pub mod tracking;
pub mod util;

use conf::*;
use controller::*;
use gui::*;
use tracking::*;
use util::*;

// Robot Wrapper to seperate Competiton handling from the GUI loop
struct CompeteHandler {
    robot: Rc<RefCell<Robot>>,
}

// Functions to handle the Autonomous Period and Driver Control
impl Robot {
    // Update the robot input during the Autonomous Period
    pub fn auto_tick(&mut self) {}

    // Let the driver control the robot during Driver Control
    pub fn driver_tick(&mut self, state: ControllerState) {
        // Apply a curve to the joystick input and convert it to voltages for the
        // Drivetrain
        let joystick_vals = apply_curve(&self.conf, &state);

        // Apply the voltage to each side of the Drivetrain
        self.drive.left_motors.iter_mut().for_each(|m| {
            let _ = m.motor.set_voltage(joystick_vals.0 .1 * m.motor.max_voltage());
        });
        self.drive.right_motors.iter_mut().for_each(|m| {
            let _ = m.motor.set_voltage(joystick_vals.1 .1 * m.motor.max_voltage());
        });

        // Apply a constant voltage to the intake if R1 or R2 is pressed
        if state.button_down.is_now_pressed() {
            self.intake.full_speed = !self.intake.full_speed;
        }

        if state.button_r1.is_pressed() {
            let _ = self.intake.motor_2.set_voltage(1.0);
            let _ = self.intake.motor_1.set_voltage(if self.intake.full_speed { 1.0 } else { 0.5 });
        } else if state.button_r2.is_pressed() {
            let _ = self.intake.motor_2.set_voltage(-1.0);
            let _ = self.intake.motor_1.set_voltage(-if self.intake.full_speed { 1.0 } else { 0.5 });
        } else {
            let _ = self.intake.motor_1.set_voltage(0.0);
            let _ = self.intake.motor_2.set_voltage(0.0);
        }

        // Apply a constnat voltage to the indexer if L1 or L2 is pressed
        let _ = self.indexer.set_voltage(if state.button_l1.is_pressed() {
            1.0
        } else if state.button_l2.is_pressed() {
            -1.0
        } else {
            0.0
        });

        // Toggle the Solenoid for the Scraper if B is pressed
        if state.button_b.is_now_pressed() {
            let _ = self.scraper.toggle();
        }
    }
}

impl Compete for CompeteHandler {
    // Autonomous Loop when the Competition Switch is connected
    async fn autonomous(&mut self) {
        println!("Running the Autonomous Loop!");
        loop {
            // Update the Auto/Skills timer for the Controller Display
            // Run auto tick
            self.robot.borrow_mut().auto_tick();
            // Wait for 10 ms (0.01 seconds)
            sleep(Motor::UPDATE_INTERVAL).await;
        }
    }

    // Driver Loop when the Competition Switch is connected, main loop for
    // CompController when the Competition Switch is disconnected
    async fn driver(&mut self) {
        println!("Running Drive Loop!");
        loop {
            // Get the Controller's current State
            let robot = self.robot.borrow_mut();
            let state = robot.cont.state();
            drop(robot);
            let state = match state {
                Ok(s) => s,
                Err(_) => {
                    sleep(Controller::UPDATE_INTERVAL).await;
                    continue;
                }
            };
            // Run the driver tick
            self.robot.borrow_mut().driver_tick(state);
            // 25 ms (0.025 second) wait (Controller update time)
            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // Load the Robot settings from conf.json or the defaults
    let conf = Config::load();
    // Create the DynamicPeripherals
    let mut dyn_peripherals = DynamicPeripherals::new(peripherals);

    // Create the Drivetrain, Intake and Indexer Motors
    let drive = Drivetrain::new(&conf, &mut dyn_peripherals);
    let intake = Intake::new(&conf, &mut dyn_peripherals);
    let indexer = NamedMotor::new_exp(dyn_peripherals.take_smart_port(conf.ports[8]).unwrap(), conf.reversed[8], conf.names[8].clone());

    // Create the Solenoid for the Scraper
    let scraper = AdiDigitalOut::new(dyn_peripherals.take_adi_port(1).unwrap());

    // Create the Devices needed for Tracking
    println!("Creating Tracking Devices");
    let (mut tracking, pose) = Tracking::new(&mut dyn_peripherals, &conf);

    // Borrow the primary controller for the Competition loop
    let cont = dyn_peripherals.take_primary_controller().unwrap();

    // Create the main Robot struct
    println!("Creating Robot");
    let robot = Robot { cont, conf, drive, intake, indexer, scraper, pose };
    let robot_cell = Rc::new(RefCell::new(robot));

    // Create an instance of the CompController and the wrapper for Competition
    // switch handling
    println!("Creating Virtual Competition Controller");
    let compete = CompeteHandler { robot: robot_cell.clone() };

    // Initialize the GUI loop
    println!("Creating GUI Object");
    let mut gui = Gui::new(robot_cell.clone(), dyn_peripherals.take_display().unwrap());

    // Calibrate the IMU
    println!("Calibrating IMU");
    tracking.calibrate_imu().await;

    // Run the Competition, Tracking and GUI loops
    println!("Running Competition, Tracking, and GUI threads");
    join(join(compete.compete(), tracking.tracking_loop()), gui.render_loop()).await;
}
