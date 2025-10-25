#![feature(duration_millis_float)]
#![feature(default_field_values)]
#![feature(nonpoison_mutex, sync_nonpoison, lock_value_accessors)]

use std::{cell::RefCell, rc::Rc, time::Instant};

use futures::future::join;
use vexide::{controller::ControllerState, peripherals::DynamicPeripherals, prelude::*};

pub mod autos;
pub mod comp;
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

use crate::{
    autos::{Action, Auto, Autos, Chassis, CubicBezier, PathSegment, Pid, SpeedCurve},
    comp::CompHandler,
};

// Robot Wrapper to seperate Competiton handling from the GUI loop
struct CompeteHandler {
    robot: Rc<RefCell<Robot>>,
}

// Functions to handle the Autonomous Period and Driver Control
impl Robot {
    pub fn update_telemetry(&mut self) {}

    // Update the robot input during the Autonomous Period
    pub fn auto_tick(&mut self) {
        let time = self.comp.time.get_cloned();
        let auto = self.comp.get_auto();
        let checkpoint_time = auto.get_checkpoint() - time;

        if auto.spline_t.fract() >= 0.99 && checkpoint_time.abs() < 0.1 {
            auto.current_curve += 1;
        } else if auto.spline_t >= 0.99 && checkpoint_time < -0.1 {
            auto.delay += checkpoint_time.abs();
        }

        let (left, right) = self.chassis.update(auto);

        self.drive.left_motors.iter_mut().for_each(|m| {
            let _ = m.motor.set_voltage(left);
        });
        self.drive.right_motors.iter_mut().for_each(|m| {
            let _ = m.motor.set_voltage(right);
        });

        for i in auto.current_action..(auto.actions.len() - 1) {
            let action = &auto.actions[i];
            if (action.1 - auto.spline_t).abs() < 0.05 {
                match action.0 {
                    autos::Action::ToggleMatchload => {
                        let _ = self.scraper.toggle();
                    }
                    autos::Action::SpinIntake(v) => {
                        let _ = self.intake.motor_1.set_voltage(v);
                        let _ = self.intake.motor_2.set_voltage(v);
                    }
                    autos::Action::StopIntake => {
                        let _ = self.intake.motor_1.set_voltage(0.0);
                        let _ = self.intake.motor_2.set_voltage(0.0);
                    }
                    autos::Action::SpinIndexer(v) => {
                        let _ = self.indexer.set_voltage(v);
                    }
                    autos::Action::StopIndexer => {
                        let _ = self.indexer.set_voltage(0.0);
                    }
                };
            } else {
                break;
            }
        }
    }

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
        println!("Running the Autonomous Loop");
        let mut robot = self.robot.borrow_mut();
        *robot.comp.time.lock() = 0.0;
        robot.comp.get_auto().reset_state();
        let start_pose = robot.comp.get_auto().start_pose;
        robot.chassis.calibrate(start_pose);
        robot.chassis.reset();
        drop(robot);
        let mut last_update = Instant::now();
        loop {
            let now = Instant::now();
            self.robot.borrow_mut().comp.update(now.duration_since(last_update));
            last_update = now;
            // Run auto tick
            self.robot.borrow_mut().auto_tick();
            // Wait for 25 ms (0.04 seconds)
            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }

    // Driver Loop when the Competition Switch is connected, main loop for
    // CompController when the Competition Switch is disconnected
    async fn driver(&mut self) {
        println!("Running the Drive Loop");
        *self.robot.borrow_mut().comp.time.lock() = 0.0;
        let mut last_update = Instant::now();
        loop {
            let now = Instant::now();
            self.robot.borrow_mut().comp.update(now.duration_since(last_update));
            last_update = now;
            // Get the Controller's current State
            let state = self.robot.borrow_mut().cont.state();
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

fn setup_autos(mut comp: CompHandler) -> CompHandler {
    let mut red_left = Auto::new();

    red_left.add_curves(
        vec![
            PathSegment {
                curve: CubicBezier {
                    a: (-60.0, 15.0),
                    b: (-60.0, 35.0),
                    c: (-41.0, 41.0),
                    d: (-28.0, 28.0),
                },
                speed: SpeedCurve::new_linear(0.75, 0.5),
                end_heading: (135.0f64).to_radians(),
                reversed_drive: false,
            },
            PathSegment {
                curve: CubicBezier {
                    a: (-28.0, 28.0),
                    b: (-20.0, 20.0),
                    c: (-16.0, 16.0),
                    d: (-11.0, 11.0),
                },
                speed: SpeedCurve::new_linear(0.5, 0.0),
                end_heading: (135.0f64).to_radians(),
                reversed_drive: false,
            },
        ],
        vec![2.0, 3.0],
    );

    red_left.add_actions(vec![
        (Action::ToggleMatchload, 0.85),
        (Action::SpinIntake(1.0), 0.85),
        (Action::StopIntake, 1.125),
        (Action::ToggleMatchload, 1.25),
        (Action::SpinIntake(-1.0), 1.85),
    ]);

    comp.autos.push((Autos::RedLeft, red_left));

    comp.autos.push((Autos::RedRight, Auto::new()));

    comp
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
    let (mut tracking, pose) = Tracking::new(&mut dyn_peripherals, &conf);
    let chassis = Chassis::new(Pid::new(4.0, 20.0, 0.0), Pid::new(1.0, 0.0, 0.0), 1.0, pose);

    // Borrow the primary controller for the Competition loop
    let cont = dyn_peripherals.take_primary_controller().unwrap();

    println!("Creating Autos");
    let comp = setup_autos(CompHandler::new());

    // Create the main Robot struct
    println!("Creating Robot");
    let robot = Robot {
        cont,
        conf,
        drive,
        intake,
        indexer,
        scraper,
        chassis,
        comp,
    };
    let robot_cell = Rc::new(RefCell::new(robot));

    // Create an instance of the CompController and the wrapper for Competition
    // switch handling
    let compete = CompeteHandler { robot: robot_cell.clone() };

    // Initialize the GUI loop
    let mut gui = Gui::new(robot_cell.clone(), dyn_peripherals.take_display().unwrap());

    // Calibrate the IMU
    tracking.calibrate_imu().await;

    // Run the Competition, Tracking and GUI loops
    println!("Running Competition, Tracking, and GUI threads");
    join(join(compete.compete(), tracking.tracking_loop()), gui.render_loop()).await;
}
