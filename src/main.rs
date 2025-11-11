#![feature(duration_millis_float)]
#![feature(default_field_values)]
#![feature(nonpoison_mutex, nonpoison_rwlock, sync_nonpoison, lock_value_accessors)]

use std::{
    sync::{Arc, nonpoison::RwLock},
    time::{Duration, Instant},
};

use futures::join;
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

// Functions to handle the Autonomous Period and Driver Control
impl Robot {
    pub fn update_telemetry(&mut self) {
        if let Ok(mut t) = self.telem.try_write() {
            t.motor_temperatures = vec![
                self.drive.left_motors[0].get_temp(),
                self.drive.left_motors[1].get_temp(),
                self.drive.left_motors[2].get_temp(),
                self.drive.right_motors[0].get_temp(),
                self.drive.right_motors[1].get_temp(),
                self.drive.right_motors[2].get_temp(),
                self.intake.motor_1.get_temp(),
                self.intake.motor_2.get_temp(),
                self.indexer.get_temp(),
            ];
            t.motor_headings = vec![
                self.drive.left_motors[0].get_pos_degrees(),
                self.drive.left_motors[1].get_pos_degrees(),
                self.drive.left_motors[2].get_pos_degrees(),
                self.drive.right_motors[0].get_pos_degrees(),
                self.drive.right_motors[1].get_pos_degrees(),
                self.drive.right_motors[2].get_pos_degrees(),
                self.intake.motor_1.get_pos_degrees(),
                self.intake.motor_2.get_pos_degrees(),
                self.indexer.get_pos_degrees(),
            ];
            t.motor_types = vec![
                if self.drive.left_motors[0].motor.is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.drive.left_motors[1].motor.is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.drive.left_motors[2].motor.is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.drive.right_motors[0].motor.is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.drive.right_motors[1].motor.is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.drive.right_motors[2].motor.is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.intake.motor_1.motor.is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.intake.motor_2.motor.is_connected() { MotorType::Exp } else { MotorType::Disconnected },
                if self.indexer.motor.is_connected() { MotorType::Exp } else { MotorType::Disconnected },
            ];
            t.offsets = self.conf.offsets.into();
        }
    }

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
            m.motor.set_voltage(left).ok();
        });
        self.drive.right_motors.iter_mut().for_each(|m| {
            m.motor.set_voltage(right).ok();
        });

        if !auto.actions.is_empty() {
            for i in auto.current_action..(auto.actions.len() - 1) {
                let action = &auto.actions[i];
                if (action.1 - auto.spline_t).abs() < 0.05 {
                    match action.0 {
                        autos::Action::ToggleMatchload => {
                            self.matchload.toggle().ok();
                        }
                        autos::Action::ToggleDescore => {
                            self.descore.toggle().ok();
                        }
                        autos::Action::SpinIntake(v) => {
                            self.intake.motor_1.set_voltage(v).ok();
                            self.intake.motor_2.set_voltage(v).ok();
                        }
                        autos::Action::StopIntake => {
                            self.intake.motor_1.set_voltage(0.0).ok();
                            self.intake.motor_2.set_voltage(0.0).ok();
                        }
                        autos::Action::SpinIndexer(v) => {
                            self.indexer.set_voltage(v).ok();
                        }
                        autos::Action::StopIndexer => {
                            self.indexer.set_voltage(0.0).ok();
                        }
                    };
                } else {
                    break;
                }
            }
        }
    }

    // Let the driver control the robot during Driver Control
    pub fn driver_tick(&mut self, state: Option<ControllerState>) {
        match state {
            Some(state) => {
                // Apply a curve to the joystick input and convert it to voltages for the
                // Drivetrain
                let joystick_vals = apply_curve(&self.conf, &state);

                // Apply the voltage to each side of the Drivetrain
                self.drive.left_motors.iter_mut().for_each(|m| {
                    m.motor.set_voltage(joystick_vals.0 .1 * m.motor.max_voltage()).ok();
                });
                self.drive.right_motors.iter_mut().for_each(|m| {
                    m.motor.set_voltage(joystick_vals.1 .1 * m.motor.max_voltage()).ok();
                });

                if state.button_r1.is_pressed() {
                    self.intake.motor_1.set_voltage(1.0).ok();
                    self.intake.motor_2.set_voltage(1.0).ok();
                } else if state.button_r2.is_pressed() {
                    self.intake.motor_1.set_voltage(-1.0).ok();
                    self.intake.motor_2.set_voltage(-1.0).ok();
                } else {
                    self.intake.motor_1.set_voltage(0.0).ok();
                    self.intake.motor_2.set_voltage(0.0).ok();
                }

                // Apply a constnat voltage to the indexer if L1 or L2 is pressed
                self.indexer
                    .set_voltage(if state.button_l1.is_pressed() {
                        1.0
                    } else if state.button_l2.is_pressed() {
                        -1.0
                    } else {
                        0.0
                    })
                    .ok();

                // Toggle the Solenoid for the Scraper if B is pressed
                if state.button_b.is_now_pressed() {
                    self.matchload.toggle().ok();
                }
            }
            None => {
                // Apply the voltage to each side of the Drivetrain
                self.drive.left_motors.iter_mut().for_each(|m| {
                    m.motor.set_voltage(0.0).ok();
                });
                self.drive.right_motors.iter_mut().for_each(|m| {
                    m.motor.set_voltage(0.0).ok();
                });
            }
        }
    }
}

impl Compete for Robot {
    // Autonomous Loop when the Competition Switch is connected
    async fn autonomous(&mut self) {
        println!("Running the Autonomous Loop");
        *self.comp.time.lock() = 0.0;
        self.comp.get_auto().reset_state();
        let start_pose = self.comp.get_auto().start_pose;
        self.chassis.calibrate(start_pose);
        self.chassis.reset();
        let mut last_update = Instant::now();
        let mut now;
        loop {
            now = Instant::now();
            self.comp.update(now.duration_since(last_update));
            last_update = now;
            // Run auto tick
            self.auto_tick();
            self.update_telemetry();
            // Wait for 25 ms (0.04 seconds)
            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }

    // Driver Loop when the Competition Switch is connected, main loop for
    // CompController when the Competition Switch is disconnected
    async fn driver(&mut self) {
        println!("Running the Drive Loop");
        *self.comp.time.lock() = 0.0;
        let mut last_update = Instant::now();
        loop {
            self.comp.update(last_update.elapsed());
            last_update = Instant::now();
            // Get the Controller's current State
            self.driver_tick(self.cont.state().ok());
            self.update_telemetry();
            // 25 ms (0.04 second) wait (Controller update time)
            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

fn setup_autos(mut comp: CompHandler) -> CompHandler {
    let mut no = Auto::new();
    no.start_pose = (-60.0, 16.0, 0.0);
    no.add_curves(
        vec![PathSegment {
            curve: CubicBezier {
                a: (-60.0, 16.0),
                b: (-60.0, 18.0),
                c: (-60.0, 20.0),
                d: (-60.0, 22.0),
            },
            speed: SpeedCurve::new_linear(1.0, 0.0),
            end_heading: 0.0,
            reversed_drive: false,
        }],
        vec![15.0],
    );

    comp.autos.push((Autos::None, no));

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

    // Create the Solenoid for the matchload
    let matchload = AdiDigitalOut::new(dyn_peripherals.take_adi_port(1).unwrap());
    let descore = AdiDigitalOut::new(dyn_peripherals.take_adi_port(2).unwrap());

    let telem = Arc::new(RwLock::new(Telem {
        motor_names: vec!["LF", "LM", "LB", "RF", "RM", "RB", "IF", "IT", "IB"],
        sensor_names: vec!["IMU", "HT", "VT"],
        ..Default::default()
    }));

    // Create the Devices needed for Tracking
    let (mut tracking, pose) = Tracking::new(&mut dyn_peripherals, telem.clone(), &conf);
    let chassis = Chassis::new(Pid::new(4.0, 20.0, 0.0), Pid::new(1.0, 0.0, 0.0), 1.0, pose);

    // Borrow the primary controller for the Competition loop
    let cont = dyn_peripherals.take_primary_controller().unwrap();

    println!("Creating Autos");
    let comp = setup_autos(CompHandler::new());

    // Initialize the GUI loop
    let mut gui = Gui::new(dyn_peripherals.take_display().unwrap(), telem.clone());

    // Create the main Robot struct
    println!("Creating Robot");
    let robot = Robot {
        cont,
        conf,
        drive,
        intake,
        indexer,
        matchload,
        descore,
        chassis,
        comp,
        telem,
    };

    // Calibrate the IMU
    tracking.calibrate_imu().await;

    // Run the Competition, Tracking and GUI loops
    println!("Running Competition, Tracking, and GUI threads");
    join!(robot.compete(), tracking.tracking_loop(), gui.render_loop());
}
