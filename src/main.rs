#![feature(duration_millis_float)]
#![feature(default_field_values)]
#![feature(nonpoison_mutex, nonpoison_rwlock, sync_nonpoison, lock_value_accessors)]

use std::{
    sync::{nonpoison::RwLock, Arc},
    time::Instant,
};

use vexide::{controller::ControllerState, peripherals::DynamicPeripherals, prelude::*};

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
    autos::{Auto, Autos, Chassis, LinearInterp, PathSegment, Pid, SpeedCurve},
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
        }
    }

    // Update the robot input during the Autonomous Period
    pub fn auto_tick(&mut self) {
        let auto = self.comp.get_auto();

        if auto.spline_t.fract() >= 0.995 && auto.timeout_start.elapsed().as_millis_f64() <= auto.get_timeout() {
            auto.current_curve += 1;
            auto.timeout_start = Instant::now();
        } else if auto.spline_t.fract() >= 0.995 {
            return;
        }

        let (left, right) = self.chassis.update(auto);

        self.drive.write().left_motors.iter_mut().for_each(|m| m.set_voltage(left).unwrap());
        self.drive.write().right_motors.iter_mut().for_each(|m| m.set_voltage(right).unwrap());

        if !auto.actions.is_empty() {
            for i in auto.current_action..(auto.actions.len() - 1) {
                let action = &auto.actions[i];
                if (action.1 - auto.spline_t).abs() < 0.01 {
                    match action.0 {
                        autos::Action::ToggleMatchload => {
                            self.matchload.toggle().ok();
                        }
                        autos::Action::ToggleDescore => {
                            self.descore.toggle().ok();
                        }
                        autos::Action::SpinIntake(v) => {
                            self.intake.motor_1.set_voltage(v * self.intake.motor_1.max_voltage()).ok();
                            self.intake.motor_2.set_voltage(v * self.intake.motor_2.max_voltage()).ok();
                        }
                        autos::Action::StopIntake => {
                            self.intake.motor_1.set_voltage(0.0).ok();
                            self.intake.motor_2.set_voltage(0.0).ok();
                        }
                        autos::Action::SpinIndexer(v) => {
                            self.indexer.set_voltage(v * self.indexer.max_voltage()).ok();
                        }
                        autos::Action::StopIndexer => {
                            self.indexer.set_voltage(0.0).ok();
                        }
                        autos::Action::ResetPos(x, y, theta) => {
                            self.chassis.set_pose((x, y, theta));
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
                if state.button_up.is_now_pressed() && state.button_left.is_now_pressed() {
                    self.comp.is_recording = true;
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

                if state.button_r1.is_pressed() {
                    self.intake.motor_1.set_voltage(self.intake.motor_1.max_voltage()).ok();
                    self.intake.motor_2.set_voltage(self.intake.motor_1.max_voltage()).ok();
                } else if state.button_r2.is_pressed() {
                    self.intake.motor_1.set_voltage(-self.intake.motor_1.max_voltage()).ok();
                    self.intake.motor_2.set_voltage(-self.intake.motor_2.max_voltage()).ok();
                } else {
                    self.intake.motor_1.set_voltage(0.0).ok();
                    self.intake.motor_2.set_voltage(0.0).ok();
                }

                // Apply a constnat voltage to the indexer if L1 or L2 is pressed
                self.indexer
                    .set_voltage(
                        if state.button_l1.is_pressed() {
                            1.0
                        } else if state.button_l2.is_pressed() {
                            -1.0
                        } else {
                            0.0
                        } * self.indexer.max_voltage(),
                    )
                    .ok();

                // Toggle the Solenoid for the Scraper if B is pressed
                if state.button_b.is_now_pressed() {
                    self.matchload.toggle().ok();
                }

                if state.button_x.is_now_pressed() {
                    self.descore.toggle().ok();
                }
            }
            None => {
                // Apply the voltage to each side of the Drivetrain
                self.drive.write().left_motors.iter_mut().for_each(|m| m.set_voltage(0.0).unwrap());
                self.drive.write().right_motors.iter_mut().for_each(|m| m.set_voltage(0.0).unwrap());
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
        println!("hi");
    }

    async fn disabled(&mut self) {
        self.chassis.calibrate((0.0, 0.0, 0.0));
        self.chassis.reset();
    }

    async fn autonomous(&mut self) {
        println!("Running the Autonomous Loop");
        *self.comp.time.write() = 0.0;
        self.comp.get_auto().reset_state();
        self.chassis.set_pose(self.comp.get_auto().start_pose);
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
        *self.comp.time.write() = 0.0;
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

fn setup_autos(mut comp: AutoHandler) -> AutoHandler {
    let mut no = Auto::new();
    no.start_pose = (0.0, 0.0, 0.0);
    no.add_curves(vec![PathSegment {
        curve: Box::new(LinearInterp { a: (0.0, 0.0), b: (0.0, 0.0) }),
        speed: SpeedCurve::new_linear(1.0, 0.0),
        end_heading: 0.0,
        reversed_drive: false,
        timeout: 4000.0,
        wait_time: 0.0,
    }]);

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
    let chassis = Chassis::new(Pid::new(0.1, 0.0, 105.0), Pid::new(0.1, 0.0, 120.0), 2.3, pose);

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
