#![no_main]
#![no_std]

use core::time::Duration;

use vexide::prelude::*;

struct ControllerInfo {
    cont: Controller,
    deadZoneInnerX: i32,
    deadZoneInnerY: i32,
    deadZoneOuterX: i32,
    deadZoneOuterY: i32
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
    radio: RadioLink,
    prim_cont: ControllerInfo,
    part_cont: ControllerInfo,
    screen: Display
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        loop {
            
            sleep(Duration::from_millis(10)).await;
        }
    }

    async fn driver(&mut self) {
        println!("Driver!");
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
        radio: RadioLink::open(peripherals.port_21, "934Z", LinkType::Manager),
        prim_cont: ControllerInfo {
            cont: peripherals.primary_controller,
            deadZoneInnerX: 0, deadZoneInnerY: 0,
            deadZoneOuterX: 127, deadZoneOuterY: 127
        },
        part_cont: ControllerInfo {
            cont: peripherals.partner_controller,
            deadZoneInnerX: 0, deadZoneInnerY: 0,
            deadZoneOuterX: 127, deadZoneOuterY: 127
        },
        screen: peripherals.display
    };

    robot.compete().await;
}