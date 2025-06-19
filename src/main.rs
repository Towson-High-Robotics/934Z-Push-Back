#![no_main]
#![no_std]

use core::time::Duration;

use alloc::{format, string::{String, ToString}};

use vexide:: {
    devices::{display::{self, Text}, math::Point2},
    prelude::*
};

extern crate alloc;

#[non_exhaustive]
struct Colors;

impl Colors {
    pub const Red: Rgb<u8> = Rgb::new(175, 20, 0);
    pub const Orange: Rgb<u8> = Rgb::new(175, 100, 0);
    pub const Yellow: Rgb<u8> = Rgb::new(175, 175, 0);
    pub const Green: Rgb<u8> = Rgb::new(0, 175, 0);
    pub const LightBlue: Rgb<u8> = Rgb::new(0, 75, 175);
    pub const Blue: Rgb<u8> = Rgb::new(0, 0, 175);
    pub const Purple: Rgb<u8> = Rgb::new(75, 0, 175);
    pub const Magenta: Rgb<u8> = Rgb::new(175, 0, 175);
    pub const White: Rgb<u8> = Rgb::new(255, 255, 255);
    pub const LightGrey: Rgb<u8> = Rgb::new(225, 230, 235);
    pub const Black: Rgb<u8> = Rgb::new(0, 0, 0);
}

struct ControllerInfo {
    cont: Controller,
    dead_zone_inner_x: f64,
    dead_zone_inner_y: f64,
    dead_zone_outer_x: f64,
    dead_zone_outer_y: f64
}

struct TrackingWheel {
    sens: RotationSensor,
    offset: f64
}

struct MCLDist {
    sens: DistanceSensor,
    offset: f64,
    angle: f64,
    tilt: f64
}

struct TrackingSensors {
    h_wheel: Option<TrackingWheel>,
    v_wheel: Option<TrackingWheel>,
    dist1: Option<MCLDist>,
    dist2: Option<MCLDist>,
    dist3: Option<MCLDist>,
    imu: InertialSensor
}

struct LabeledMotor {
    name: &'static str,
    motor: Motor
}

struct Drivetrain {
    left_motors: [LabeledMotor; 3],
    right_motors: [LabeledMotor; 3],
    sensors: TrackingSensors
}

struct Robot {
    drive: Drivetrain,
    solenoid_a: AdiDigitalOut,
    solenoid_b: AdiDigitalOut,
    cont: ControllerInfo,
    screen: Display
}

#[derive(Clone, Copy)]
struct MotorStatus {
    name: &'static str,
    connected: bool,
    max_volt: f64,
    temperature: f64,
    position: f64
}

fn tank_drive(robot: &mut Robot) {
    let cont_state = robot.cont.cont.state().unwrap_or_default();
    let left_stick: f64 = cont_state.left_stick.y();
    let right_stick: f64 = cont_state.right_stick.y();

    if Float::abs(left_stick) > robot.cont.dead_zone_inner_y && Float::abs(left_stick) < robot.cont.dead_zone_outer_y {
        for i in 0..(robot.drive.left_motors.iter().count() - 1) {
            let _ = robot.drive.left_motors[i].motor.set_voltage(left_stick * robot.drive.left_motors[i].motor.max_voltage());
        }
    }

    if Float::abs(right_stick) > robot.cont.dead_zone_inner_y && Float::abs(right_stick) < robot.cont.dead_zone_outer_y {
        for i in 0..(robot.drive.right_motors.iter().count() - 1) {
            let _ = robot.drive.right_motors[i].motor.set_voltage(right_stick * robot.drive.right_motors[i].motor.max_voltage());
        }
    }
}

fn draw_text(robot: &mut Robot, text_str: &str, pos: [i16; 2], size: u32, color: Rgb<u8>, bg_color: Rgb<u8>) {
    robot.screen.draw_text(
        &display::Text::new(
            text_str,
            display::Font::new(
                display::FontSize { numerator: 1, denominator: size },
                display::FontFamily::Monospace
            ),
            Point2::from(pos)
        ),
        color,
        Some(bg_color)
    );
}

fn draw_rounded_rect(robot: &mut Robot, startX: i16, startY: i16, endX: i16, endY: i16, radius: i16, color: Rgb<u8>) {
    let u_radius = (radius as u16);
    robot.screen.fill(
        &display::Circle::new(
            Point2::from([startX + radius, startY + radius]),
            u_radius
        ), color
    );
    robot.screen.fill(
        &display::Circle::new(
            Point2::from([endX - radius, startY + radius]),
            u_radius
        ), color
    );
    robot.screen.fill(
        &display::Circle::new(
            Point2::from([startX + radius, endY - radius]),
            u_radius
        ), color
    );
    robot.screen.fill(
        &display::Circle::new(
            Point2::from([endX - radius, endY - radius]),
            u_radius
        ), color
    );
    robot.screen.fill(
        &display::Rect::new(
            Point2::from([startX, startY + radius]), 
            Point2::from([endX, endY - radius])
        ), color
    );
    robot.screen.fill(
        &display::Rect::new(
            Point2::from([startX + radius, startY]), 
            Point2::from([endX - radius, endY])
        ), color
    );
}

fn get_motor_status(robot: &mut Robot) -> [MotorStatus; 6] {
    let mut motors = [MotorStatus { name: "", connected: false, max_volt: 11.0, temperature: 0.0, position: 0.0 }; 6];
    for i in 0..robot.drive.left_motors.iter().count() {
        motors[i].name = robot.drive.left_motors[i].name;
        motors[i].connected = robot.drive.left_motors[i].motor.is_connected();
        if motors[i].connected {
            motors[i].temperature = robot.drive.left_motors[i].motor.temperature().unwrap();
            motors[i].max_volt = robot.drive.left_motors[i].motor.max_voltage();
            motors[i].position = robot.drive.left_motors[i].motor.position().unwrap().as_degrees();
        }
    }
    for i in 0..robot.drive.right_motors.iter().count() {
        motors[i + 3].name = robot.drive.right_motors[i].name;
        motors[i + 3].connected = robot.drive.right_motors[i].motor.is_connected();
        if motors[i + 3].connected {
            motors[i + 3].temperature = robot.drive.right_motors[i].motor.temperature().unwrap();
            motors[i + 3].max_volt = robot.drive.right_motors[i].motor.max_voltage();
            motors[i + 3].position = robot.drive.right_motors[i].motor.position().unwrap().as_degrees();
        }
    }
    motors
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        //TODO: Innovate Submission
    }

    async fn driver(&mut self) {
        println!("Driver!");
        let mut display_cycle_time: i32 = 0;
        loop {
            //tank_drive(self);
            self.screen.set_render_mode(display::RenderMode::DoubleBuffered);
            if display_cycle_time == 0 {
                self.screen.fill(&display::Rect::new(Point2::from([0, 0]), Point2::from([480, 240])), Colors::White);
                let motor_status = get_motor_status(self);
                draw_rounded_rect(self, 4, 4, 236, (16 * motor_status.iter().count() + 20).try_into().unwrap(), 12, Colors::LightGrey);
                for i in 0..(motor_status.iter().count()) {
                    let mut connected_col = Colors::Red;
                    if motor_status[i].connected { connected_col = Colors::Green; }
                    draw_text(self, motor_status[i].name, [16, (12 + (16 * i)).try_into().unwrap()], 4, connected_col, Colors::LightGrey);
                    draw_text(self, format!("{}w", motor_status[i].max_volt).as_str(), [64, (12 + (16 * i)).try_into().unwrap()], 4, Colors::Black, Colors::LightGrey);
                    draw_text(self, format!("{}deg", motor_status[i].position).as_str(), [108, (12 + (16 * i)).try_into().unwrap()], 4, Colors::Black, Colors::LightGrey);
                    let mut temp_col = Colors::Green;
                    if motor_status[i].temperature >= 70.0 { temp_col = Colors::Black }
                    else if motor_status[i].temperature >= 65.0 { temp_col = Colors::Red }
                    else if motor_status[i].temperature >= 60.0 { temp_col = Colors::Orange; }
                    else if motor_status[i].temperature >= 55.0 { temp_col = Colors::Yellow; }
                    draw_text(self, format!("{}°C", motor_status[i].temperature).as_str(), [184, (12 + (16 * i)).try_into().unwrap()], 4, temp_col, Colors::LightGrey);
                }
                self.screen.render();
            } else if display_cycle_time == 3 {
                display_cycle_time = 0;
            } else {
                display_cycle_time += 1;
            }
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
                LabeledMotor { name: "LF", motor: Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward) },
                LabeledMotor { name: "LM", motor: Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward) },
                LabeledMotor { name: "LB", motor: Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward) }
            ],
            right_motors: [
                LabeledMotor { name: "RF", motor: Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward) },
                LabeledMotor { name: "RM", motor: Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward) },
                LabeledMotor { name: "RB", motor: Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward) }
            ],
            sensors: TrackingSensors {
                h_wheel: Some(TrackingWheel { sens: RotationSensor::new(peripherals.port_4, Direction::Forward), offset: 0.0 }),
                v_wheel: None,
                imu: InertialSensor::new(peripherals.port_5),
                dist1: Some(MCLDist { sens: DistanceSensor::new(peripherals.port_6), offset: 0.0, angle: 0.0, tilt: 0.0 }),
                dist2: Some(MCLDist { sens: DistanceSensor::new(peripherals.port_7), offset: 0.0, angle: 0.0, tilt: 0.0 }),
                dist3: Some(MCLDist { sens: DistanceSensor::new(peripherals.port_8), offset: 0.0, angle: 0.0, tilt: 0.0 }),
            }
        },
        solenoid_a: AdiDigitalOut::new(peripherals.adi_a),
        solenoid_b: AdiDigitalOut::new(peripherals.adi_b),
        cont: ControllerInfo {
            cont: peripherals.primary_controller,
            dead_zone_inner_x: 0., dead_zone_inner_y: 0.,
            dead_zone_outer_x: 127., dead_zone_outer_y: 127.
        },
        screen: peripherals.display
    };

    robot.compete().await;
}