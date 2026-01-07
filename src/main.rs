#![feature(default_field_values)]
#![feature(slice_as_array)]
#![feature(nonpoison_rwlock, sync_nonpoison, lock_value_accessors)]

use std::{
    sync::{nonpoison::RwLock, Arc},
    time::Instant,
};

use vexide::{controller::ControllerState, peripherals::DynamicPeripherals, prelude::*, smart::motor::BrakeMode};

pub mod autos;
pub mod comp;
pub mod conf;
pub mod controller;
pub mod cubreg;
pub mod gui;
pub mod tracking;
pub mod util;

use conf::*;
use controller::*;
use gui::*;
use tracking::*;
use util::*;

use crate::{
    autos::{
        auto::{Action, Auto, Autos},
        chassis::{Chassis, Pid},
        path::{LinearInterp, PathSegment, SpeedCurve},
    },
    comp::AutoHandler,
};

// Functions to handle the Autonomous Period and Driver Control
impl Robot {
    pub fn update_telemetry(&mut self) {
        if let Ok(mut t) = self.telem.try_write() {
            let drive = self.drive.read();
            t.motor_temperatures = vec![
                drive.left_motors[0].temperature().unwrap_or(f64::NAN),
                drive.left_motors[1].temperature().unwrap_or(f64::NAN),
                drive.left_motors[2].temperature().unwrap_or(f64::NAN),
                drive.right_motors[0].temperature().unwrap_or(f64::NAN),
                drive.right_motors[1].temperature().unwrap_or(f64::NAN),
                drive.right_motors[2].temperature().unwrap_or(f64::NAN),
                self.intake.motor_1.temperature().unwrap_or(f64::NAN),
                self.intake.motor_2.temperature().unwrap_or(f64::NAN),
                self.indexer.temperature().unwrap_or(f64::NAN),
            ];
            t.motor_headings = vec![
                drive.left_motors[0].position().unwrap_or_default().as_degrees(),
                drive.left_motors[1].position().unwrap_or_default().as_degrees(),
                drive.left_motors[2].position().unwrap_or_default().as_degrees(),
                drive.right_motors[0].position().unwrap_or_default().as_degrees(),
                drive.right_motors[1].position().unwrap_or_default().as_degrees(),
                drive.right_motors[2].position().unwrap_or_default().as_degrees(),
                self.intake.motor_1.position().unwrap_or_default().as_degrees(),
                self.intake.motor_2.position().unwrap_or_default().as_degrees(),
                self.indexer.position().unwrap_or_default().as_degrees(),
            ];
            t.motor_types = vec![
                if drive.left_motors[0].is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if drive.left_motors[1].is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if drive.left_motors[2].is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if drive.right_motors[0].is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if drive.right_motors[1].is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if drive.right_motors[2].is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.intake.motor_1.is_connected() { MotorType::Blue } else { MotorType::Disconnected },
                if self.intake.motor_2.is_connected() { MotorType::Exp } else { MotorType::Disconnected },
                if self.indexer.is_connected() { MotorType::Exp } else { MotorType::Disconnected },
            ];
            t.offsets = self.conf.offsets.into();
            t.update_requested = false;
        }
    }

    // Update the robot input during the Autonomous Period
    pub fn auto_tick(&mut self) {
        let auto = self.comp.get_auto();

        if (auto.timeout_start.elapsed().as_millis() as f64 >= auto.get_timeout() && auto.spline_t <= 0.95) && !auto.waiting {
            auto.wait_start = Instant::now();
            auto.waiting = true;
        } else if auto.wait_start.elapsed().as_millis() as f64 >= auto.get_wait() && auto.waiting {
            if auto.current_curve != auto.spline.len() - 1 {
                auto.current_curve += 1
            } else {
                return;
            };
            auto.timeout_start = Instant::now();
            auto.waiting = false;
            println!("next");
        } else if auto.waiting {
            return;
        }

        let (left, right) = self.chassis.update(auto);

        self.drive.write().left_motors.iter_mut().for_each(|m| {
            m.set_voltage(-left).ok();
        });
        self.drive.write().right_motors.iter_mut().for_each(|m| {
            m.set_voltage(right).ok();
        });

        if auto.actions.is_empty() {
            return;
        }
        for i in auto.current_action..(auto.actions.len() - 1) {
            let action = &auto.actions[i];
            if (action.1 - auto.spline_t).abs() < 0.025 {
                println!("{:?}", action.0);
                match action.0 {
                    Action::ToggleMatchload => {
                        self.matchload.toggle().ok();
                    }
                    Action::ToggleDescore => {
                        self.descore.toggle().ok();
                    }
                    Action::SpinIntake(v) => {
                        self.intake.motor_1.set_voltage(v * self.intake.motor_1.max_voltage()).ok();
                        self.intake.motor_2.set_voltage(v * self.intake.motor_2.max_voltage()).ok();
                    }
                    Action::StopIntake => {
                        self.intake.motor_1.set_voltage(0.0).ok();
                        self.intake.motor_2.set_voltage(0.0).ok();
                    }
                    Action::SpinIndexer(v) => {
                        self.indexer.set_voltage(v * self.indexer.max_voltage()).ok();
                    }
                    Action::StopIndexer => {
                        self.indexer.set_voltage(0.0).ok();
                    }
                    Action::ResetPose(x, y, theta) => {
                        self.chassis.set_pose((x, y, theta));
                    }
                };
            } else {
                break;
            }
        }
    }

    // Let the driver control the robot during Driver Control
    pub fn driver_tick(&mut self, state: Option<ControllerState>) {
        match state {
            Some(state) => {
                if state.button_up.is_now_pressed() && state.button_left.is_now_pressed() {
                    self.comp.start_recording = true;
                }

                if state.button_right.is_now_pressed() {
                    let mut telem = self.telem.write();
                    telem.selector_active = true;
                    drop(telem);
                    println!("hi");
                }

                // Apply a curve to the joystick input and convert it to voltages for the
                // Drivetrain
                let joystick_vals = apply_curve(&self.conf, &state);

                // Apply the voltage to each side of the Drivetrain
                self.drive.write().left_motors.iter_mut().for_each(|m| {
                    m.set_voltage(joystick_vals.0 .1 * m.max_voltage()).ok();
                });
                self.drive.write().right_motors.iter_mut().for_each(|m| {
                    m.set_voltage(joystick_vals.1 .1 * m.max_voltage()).ok();
                });

                self.comp.recorded_poses.push((self.telem.read().pose, self.comp.start_time.elapsed().as_millis() as f64));

                if state.button_r1.is_pressed() {
                    self.comp.recorded_actions.push((Action::SpinIntake(1.00), self.comp.start_time.elapsed().as_millis() as f64));
                    self.intake.motor_1.set_voltage(self.intake.motor_1.max_voltage()).ok();
                    self.intake.motor_2.set_voltage(self.intake.motor_1.max_voltage()).ok();
                } else if state.button_r2.is_pressed() {
                    self.comp.recorded_actions.push((Action::SpinIntake(-1.00), self.comp.start_time.elapsed().as_millis() as f64));
                    self.intake.motor_1.set_voltage(-self.intake.motor_1.max_voltage()).ok();
                    self.intake.motor_2.set_voltage(-self.intake.motor_2.max_voltage()).ok();
                } else {
                    self.comp.recorded_actions.push((Action::StopIntake, self.comp.start_time.elapsed().as_millis() as f64));
                    self.intake.motor_1.set_voltage(0.0).ok();
                    self.intake.motor_2.set_voltage(0.0).ok();
                }

                // Apply a constnat voltage to the indexer if L1 or L2 is pressed
                self.indexer
                    .set_voltage(
                        if state.button_l1.is_pressed() {
                            self.comp.recorded_actions.push((Action::SpinIndexer(1.00), self.comp.start_time.elapsed().as_millis() as f64));
                            1.0
                        } else if state.button_l2.is_pressed() {
                            self.comp.recorded_actions.push((Action::SpinIndexer(-1.00), self.comp.start_time.elapsed().as_millis() as f64));
                            -1.0
                        } else {
                            self.comp.recorded_actions.push((Action::StopIndexer, self.comp.start_time.elapsed().as_millis() as f64));
                            0.0
                        } * self.indexer.max_voltage(),
                    )
                    .ok();

                // Toggle the Solenoid for the Scraper if B is pressed
                if state.button_b.is_now_pressed() {
                    self.comp.recorded_actions.push((Action::ToggleMatchload, self.comp.start_time.elapsed().as_millis() as f64));
                    self.matchload.toggle().ok();
                }

                if state.button_x.is_now_pressed() {
                    self.comp.recorded_actions.push((Action::ToggleDescore, self.comp.start_time.elapsed().as_millis() as f64));
                    self.descore.toggle().ok();
                }
            }
            None => {
                // Apply the voltage to each side of the Drivetrain
                self.drive.write().left_motors.iter_mut().for_each(|m| {
                    m.set_voltage(0.0).ok();
                });
                self.drive.write().right_motors.iter_mut().for_each(|m| {
                    m.set_voltage(0.0).ok();
                });
            }
        }
    }
}

impl Compete for Robot {
    // Autonomous Loop when the Competition Switch is connected
    async fn connected(&mut self) {
        let mut telem = self.telem.write();
        telem.selector_active = true;
        drop(telem);
    }

    async fn disabled(&mut self) {
        self.chassis.calibrate((0.0, 0.0, 0.0));
        self.chassis.reset();
    }

    async fn autonomous(&mut self) {
        println!("Running the Autonomous Loop");
        self.comp.start_time = Instant::now();
        self.chassis.set_pose(self.comp.get_auto().start_pose);
        self.drive.write().left_motors.iter_mut().for_each(|m| {
            m.brake(BrakeMode::Brake).ok();
        });
        self.drive.write().right_motors.iter_mut().for_each(|m| {
            m.brake(BrakeMode::Brake).ok();
        });
        let mut last_update = Instant::now();
        let mut now;
        self.comp.get_auto().reset_state();
        loop {
            now = Instant::now();
            self.comp.update(now.duration_since(last_update));
            last_update = now;
            // Run auto tick
            self.auto_tick();
            if self.telem.read().update_requested {
                self.update_telemetry();
            }
            // Wait for 25 ms (0.04 seconds)
            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }

    // Driver Loop when the Competition Switch is connected, main loop for
    // CompController when the Competition Switch is disconnected
    async fn driver(&mut self) {
        println!("Running the Drive Loop");
        self.comp.start_time = Instant::now();
        self.drive.write().left_motors.iter_mut().for_each(|m| {
            m.brake(BrakeMode::Coast).ok();
        });
        self.drive.write().right_motors.iter_mut().for_each(|m| {
            m.brake(BrakeMode::Coast).ok();
        });
        let mut last_update = Instant::now();
        let mut countdown_start = Instant::now();
        loop {
            self.comp.update(last_update.elapsed());
            last_update = Instant::now();
            if self.comp.start_recording && countdown_start.elapsed().as_millis() > 3500 {
                countdown_start = Instant::now();
            } else if self.comp.start_recording && countdown_start.elapsed().as_millis() > 2990 {
                self.comp.is_recording = true;
                self.comp.start_recording = false;
            }
            // Get the Controller's current State
            self.driver_tick(self.cont.state().ok());
            if self.telem.read().update_requested {
                self.update_telemetry();
            }
            // 25 ms (0.04 second) wait (Controller update time)
            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

fn setup_autos(mut comp: AutoHandler) -> AutoHandler {
    let mut no = Auto::new();
    no.start_pose = (0.0, 0.0, 0.0);
    no.add_curves(vec![PathSegment {
        curve: LinearInterp::new((0.0, 0.0), (0.0, 10.0)),
        ..Default::default()
    }]);

    comp.autos.push((Autos::None, no));

    // left_elims
    let mut left_elims = Auto::new();
    left_elims.start_pose = (0.0, 0.0, 0.0);
    left_elims.add_action(Action::SpinIntake(1.0), 0.0);
    left_elims.move_to_pose((-48.0,  47.0, 270.0), 1.0);
    left_elims.add_action(Action::ToggleMatchload, 1.0);
    left_elims.move_to_pose((-55.0,  47.0, 270.0), 1.0);
    left_elims.wait_for(1.0);
    left_elims.move_to_pose_reverse((-30.0,  47.0, 270.0), 1.0);
    left_elims.add_action(Action::ToggleDescore, 4.0);
    left_elims.add_action(Action::SpinIndexer(1.0), 4.0);
    left_elims.add_action(Action::ToggleMatchload, 4.0);
    left_elims.wait_for(1.0);
    left_elims.add_action(Action::StopIndexer, 5.0);
    left_elims.add_action(Action::ToggleDescore, 5.0);
    left_elims.move_to_pose((-36.0,  36.0, 135.0), 1.0);
    left_elims.move_to_pose((-28.5,  28.5, 135.0), 1.0);
    left_elims.add_action(Action::ToggleMatchload, 7.0);
    left_elims.move_to_pose((-19.0,  19.0, 135.0), 0.75);
    left_elims.add_action(Action::ToggleMatchload, 8.0);
    left_elims.move_to_pose((-13.0,  13.0, 135.0), 1.0);
    left_elims.add_action(Action::SpinIntake(-0.5), 9.0);
    left_elims.wait_for(1.0);
    left_elims.add_action(Action::StopIntake, 10.0);
    left_elims.move_to_pose((-25.0,  39.0,  90.0), 1.0);
    left_elims.add_action(Action::ToggleDescore, 11.0);
    left_elims.move_to_pose((-12.5,  39.0,  90.0), 1.0);
    comp.autos.push((Autos::LeftElims, left_elims));
    
    // right_elims
    let mut right_elims = Auto::new();
    right_elims.start_pose = (0.0, 0.0, 0.0);
    right_elims.add_action(Action::SpinIntake(1.0), 0.0);
    right_elims.move_to_pose((-48.0, -47.0, 270.0), 1.0);
    right_elims.add_action(Action::ToggleMatchload, 1.0);
    right_elims.move_to_pose((-55.0, -47.0, 270.0), 1.0);
    right_elims.wait_for(1.0);
    right_elims.move_to_pose_reverse((-30.0, -47.0, 270.0), 1.0);
    right_elims.add_action(Action::ToggleDescore, 4.0);
    right_elims.add_action(Action::SpinIndexer(1.0), 4.0);
    right_elims.add_action(Action::ToggleMatchload, 4.0);
    right_elims.wait_for(1.0);
    right_elims.add_action(Action::StopIndexer, 5.0);
    right_elims.add_action(Action::ToggleDescore, 5.0);
    right_elims.move_to_pose((-36.0, -36.0,  45.0), 1.0);
    right_elims.move_to_pose((-28.5, -28.5,  45.0), 1.0);
    right_elims.add_action(Action::ToggleMatchload, 7.0);
    right_elims.move_to_pose((-19.0, -19.0,  45.0), 0.75);
    right_elims.add_action(Action::ToggleMatchload, 8.0);
    right_elims.move_to_pose((-13.0, -13.0,  45.0), 1.0);
    right_elims.add_action(Action::SpinIntake(-0.5), 9.0);
    right_elims.wait_for(1.0);
    right_elims.add_action(Action::StopIntake, 10.0);
    right_elims.move_to_pose((-25.0, -39.0,  90.0), 1.0);
    right_elims.add_action(Action::ToggleDescore, 11.0);
    right_elims.move_to_pose((-12.5, -39.0,  90.0), 1.0);
    comp.autos.push((Autos::RightElims, right_elims));
    
    // left_qual
    let mut left_qual = Auto::new();
    left_qual.start_pose = (0.0, 0.0, 0.0);
    left_qual.add_action(Action::SpinIntake(1.0), 0.0);
    left_qual.move_to_pose((-28.0,  16.0,  45.0), 1.0);
    left_qual.add_action(Action::ToggleMatchload, 1.0);
    left_qual.move_to_pose((-16.0,  28.0, 300.0), 0.75);
    left_qual.add_action(Action::ToggleMatchload, 2.0);
    left_qual.move_to_pose((-47.0,  47.0, 270.0), 1.0);
    left_qual.add_action(Action::ToggleMatchload, 3.0);
    left_qual.move_to_pose((-56.0,  47.0, 270.0), 1.0);
    left_qual.wait_for(1.0);
    left_qual.move_to_pose_reverse((-30.0,  47.0, 270.0), 1.0);
    left_qual.add_action(Action::ToggleDescore, 6.0);
    left_qual.add_action(Action::SpinIndexer(1.0), 6.0);
    left_qual.add_action(Action::ToggleMatchload, 6.0);
    left_qual.wait_for(1.0);
    left_qual.move_to_pose((-35.0,  39.0,  90.0), 1.0);
    left_qual.add_action(Action::ToggleDescore, 8.0);
    left_qual.move_to_pose((-12.0,  39.0,  90.0), 1.0);
    comp.autos.push((Autos::LeftQual, left_qual));
    
    // right_qual
    let mut right_qual = Auto::new();
    right_qual.start_pose = (0.0, 0.0, 0.0);
    right_qual.add_action(Action::SpinIntake(1.0), 0.0);
    right_qual.move_to_pose((-28.0, -16.0, 135.0), 1.0);
    right_qual.add_action(Action::ToggleMatchload, 1.0);
    right_qual.move_to_pose((-16.0, -28.0, 240.0), 0.75);
    right_qual.add_action(Action::ToggleMatchload, 2.0);
    right_qual.move_to_pose((-47.0, -47.0, 270.0), 1.0);
    right_qual.add_action(Action::ToggleMatchload, 3.0);
    right_qual.move_to_pose((-56.0, -47.0, 270.0), 1.0);
    right_qual.wait_for(1.0);
    right_qual.move_to_pose_reverse((-30.0, -47.0, 270.0), 1.0);
    right_qual.add_action(Action::ToggleDescore, 6.0);
    right_qual.add_action(Action::SpinIndexer(1.0), 6.0);
    right_qual.add_action(Action::ToggleMatchload, 6.0);
    right_qual.wait_for(1.0);
    right_qual.move_to_pose((-35.0, -39.0,  90.0), 1.0);
    right_qual.add_action(Action::ToggleDescore, 8.0);
    right_qual.move_to_pose((-12.0, -39.0,  90.0), 1.0);
    comp.autos.push((Autos::RightQual, right_qual));
    
    // sawp
    let mut sawp = Auto::new();
    sawp.start_pose = (0.0, 0.0, 0.0);
    sawp.add_action(Action::SpinIntake(1.0), 0.0);
    sawp.move_to_pose((-48.0, -47.0, 270.0), 1.0);
    sawp.add_action(Action::ToggleMatchload, 1.0);
    sawp.move_to_pose((-56.0, -47.0, 270.0), 1.0);
    sawp.wait_for(1000.0);
    sawp.move_to_pose_reverse((-30.0, -47.0, 270.0), 1.0);
    sawp.add_action(Action::ToggleDescore, 4.0);
    sawp.add_action(Action::SpinIndexer(1.0), 4.0);
    sawp.wait_for(2.0);
    sawp.add_action(Action::ToggleDescore, 5.0);
    sawp.add_action(Action::StopIndexer, 5.0);
    sawp.add_action(Action::ToggleMatchload, 5.0);
    sawp.move_to_pose((-24.0, -30.0,   0.0), 1.0);
    sawp.add_action(Action::ToggleMatchload, 6.0);
    sawp.move_to_pose((-24.0, -15.0,   0.0), 0.75);
    sawp.add_action(Action::ToggleMatchload, 7.0);
    sawp.move_to_pose((-24.0,  12.0,   0.0), 1.0);
    sawp.add_action(Action::ToggleMatchload, 8.0);
    sawp.move_to_pose((-24.0,  30.0, 330.0), 0.75);
    sawp.move_to_pose((-11.0,  11.0, 315.0), 1.0);
    sawp.add_action(Action::SpinIndexer(-0.5), 10.0);
    sawp.wait_for(0.5);
    sawp.add_action(Action::StopIndexer, 11.0);
    sawp.move_to_pose((-47.0,  47.0, 270.0), 1.0);
    sawp.move_to_pose_reverse((-30.0,  47.0, 270.0), 1.0);
    sawp.add_action(Action::ToggleDescore, 13.0);
    sawp.add_action(Action::SpinIndexer(1.0), 13.0);
    comp.autos.push((Autos::Solo, sawp));
    
    comp
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // Load the Robot settings from conf.json or the defaults
    let conf = Config::load();
    // Create the DynamicPeripherals
    let mut dyn_peripherals = DynamicPeripherals::new(peripherals);

    // Create the Drivetrain, Intake and Indexer Motors
    let drive = Arc::new(RwLock::new(Drivetrain::new(&conf, &mut dyn_peripherals)));
    let intake = Intake::new(&conf, &mut dyn_peripherals);
    let indexer = Motor::new_exp(dyn_peripherals.take_smart_port(conf.ports[8]).unwrap(), if conf.reversed[8] { Direction::Reverse } else { Direction::Forward });

    // Create the Solenoid for the matchload
    let matchload = AdiDigitalOut::new(dyn_peripherals.take_adi_port(1).unwrap());
    let descore = AdiDigitalOut::new(dyn_peripherals.take_adi_port(2).unwrap());

    let telem = Arc::new(RwLock::new(Telem {
        motor_names: vec!["LF", "LM", "LB", "RF", "RM", "RB", "IF", "IT", "IB"],
        sensor_names: vec!["IMU", "HT", "VT"],
        ..Default::default()
    }));

    // Create the Devices needed for Tracking
    let (mut tracking, pose) = Tracking::new(&mut dyn_peripherals, telem.clone(), drive.clone(), &conf);
    let chassis = Chassis::new(Pid::new(7.0, 0.0, 105.0), Pid::new(2.0, 0.0, 10.0), Pid::new(8.0, 0.0, 120.0), 2.3, pose);

    // Borrow the primary controller for the Competition loop
    let cont = dyn_peripherals.take_primary_controller().unwrap();

    println!("Creating Autos");
    let comp = setup_autos(AutoHandler::new());

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

    let compete = spawn(robot.compete());
    let track = spawn(async move { tracking.tracking_loop().await });
    let gui = spawn(async move { gui.render_loop().await });
    gui.await;
    track.await;
    compete.await;
}
