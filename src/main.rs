#![no_main]
#![no_std]

use core::time::Duration;

use alloc::{borrow::ToOwned, format};
use vexide::{devices::{display::Text, math::Point2}, prelude::*};

extern crate alloc;

use alloc::string::ToString;

struct ControllerInfo {
    cont: Controller,
    deadZoneInnerX: f64,
    deadZoneInnerY: f64,
    deadZoneOuterX: f64,
    deadZoneOuterY: f64
}

struct Drivetrain {
    left_motors: [Motor; 3],
    right_motors: [Motor; 3],
    imu: InertialSensor
}

struct Robot {
    drive: Drivetrain,
    solenoid_a: AdiDigitalOut,
    solenoid_b: AdiDigitalOut,
    //radio: RadioLink,
    prim_cont: ControllerInfo,
    part_cont: ControllerInfo,
    screen: Display
}

fn tank_drive(robot: &mut Robot) -> &mut Robot {
    let cont_state = robot.prim_cont.cont.state().unwrap_or_default();
    let left_stick: f64 = cont_state.left_stick.y();
    let right_stick: f64 = cont_state.right_stick.y();



    if Float::abs(left_stick) > robot.prim_cont.deadZoneInnerY && Float::abs(left_stick) < robot.prim_cont.deadZoneOuterY {
        for i in 0..(robot.drive.left_motors.iter().count() - 1) {
            let _ = robot.drive.left_motors[i].set_voltage(left_stick * robot.drive.left_motors[i].max_voltage());
        }
    }

    if Float::abs(right_stick) > robot.prim_cont.deadZoneInnerY && Float::abs(right_stick) < robot.prim_cont.deadZoneOuterY {
        for i in 0..(robot.drive.right_motors.iter().count() - 1) {
            let _ = robot.drive.right_motors[i].set_voltage(right_stick * robot.drive.right_motors[i].max_voltage());
        }
    }

    robot
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        //TODO: Innovate Submission
    }

    async fn driver(&mut self) {
        println!("Driver!");
        loop {
            //tank_drive(self);
            self.screen.erase(Rgb {r: 0, g: 0, b: 0});
            let text = vexide::devices::display::Text::new(format!("{}", self.prim_cont.cont.state().unwrap().right_stick.y()).as_str(), vexide::devices::display::Font::new(vexide::devices::display::FontSize { numerator: 1, denominator: 4}, vexide::devices::display::FontFamily::Monospace), Point2::from([0, 0]));
            self.screen.draw_text(&text, Rgb {r: 255, g: 255, b:255},core::prelude::v1::Some(Rgb {r: 0, g: 0, b: 0}));
            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    println!("Hello from vexide::main!");
    
    let robot = Robot {
        drive: Drivetrain {
            left_motors: [
                Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward)
            ],
            right_motors: [
                Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward)
            ],
            imu: InertialSensor::new(peripherals.port_4)
        },
        solenoid_a: AdiDigitalOut::new(peripherals.adi_a),
        solenoid_b: AdiDigitalOut::new(peripherals.adi_b),
        //radio: RadioLink::open(peripherals.port_21, "934Z", LinkType::Manager),
        prim_cont: ControllerInfo {
            cont: peripherals.primary_controller,
            deadZoneInnerX: 0., deadZoneInnerY: 0.,
            deadZoneOuterX: 127., deadZoneOuterY: 127.
        },
        part_cont: ControllerInfo {
            cont: peripherals.partner_controller,
            deadZoneInnerX: 0., deadZoneInnerY: 0.,
            deadZoneOuterX: 127., deadZoneOuterY: 127.
        },
        screen: peripherals.display
    };

    robot.compete().await;
}