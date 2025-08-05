use core::{cell::RefCell, f64, time::Duration};

use alloc::{format, rc::Rc};
use vexide::{devices::{display::*, math::Point2}, prelude::*};

use crate::{comp_controller::{CompContState, CompController}, util::{NamedMotor, Robot}};

#[non_exhaustive]
struct Colors {}

// #[allow(dead_code)]
// impl Colors {
//     const BG_1: Rgb<u8> = Rgb::new(230, 235, 255);
//     const BG_2: Rgb<u8> = Rgb::new(200, 207, 220);
//     const BG_3: Rgb<u8> = Rgb::new(160, 170, 190);
//     const TEXT_1: Rgb<u8> = Rgb::new(10, 15, 30);
//     const TEXT_2: Rgb<u8> = Rgb::new(30, 35, 50);
//     const TEXT_3: Rgb<u8> = Rgb::new(40, 50, 60);
//     const MAROON: Rgb<u8> = Rgb::new(105, 0, 30);
//     const RED: Rgb<u8> = Rgb::new(205, 20, 0);
//     const ORANGE: Rgb<u8> = Rgb::new(205, 107, 0);
//     const YELLOW: Rgb<u8> = Rgb::new(205, 205, 0);
//     const GREEN: Rgb<u8> = Rgb::new(0, 205, 0);
//     const LIGHT_BLUE: Rgb<u8> = Rgb::new(0, 147, 255);
//     const BLUE: Rgb<u8> = Rgb::new(0, 20, 205);
//     const PURPLE: Rgb<u8> = Rgb::new(147, 0, 255);
//     const BLACK: Rgb<u8> = Rgb::new(0, 0, 0);
//     const WHITE: Rgb<u8> = Rgb::new(255, 255, 255);
// }
   
#[allow(dead_code)]
impl Colors {
    const BG_1: Rgb<u8> = Rgb::new(10, 15, 30);
    const BG_2: Rgb<u8> = Rgb::new(30, 35, 50);
    const BG_3: Rgb<u8> = Rgb::new(40, 50, 60);
    const TEXT_1: Rgb<u8> = Rgb::new(230, 235, 255);
    const TEXT_2: Rgb<u8> = Rgb::new(200, 207, 220);
    const TEXT_3: Rgb<u8> = Rgb::new(160, 170, 190);
    const MAROON: Rgb<u8> = Rgb::new(205, 0, 0);
    const RED: Rgb<u8> = Rgb::new(255, 20, 0);
    const ORANGE: Rgb<u8> = Rgb::new(255, 187, 0);
    const YELLOW: Rgb<u8> = Rgb::new(255, 255, 0);
    const GREEN: Rgb<u8> = Rgb::new(0, 255, 0);
    const LIGHT_BLUE: Rgb<u8> = Rgb::new(0, 187, 255);
    const BLUE: Rgb<u8> = Rgb::new(0, 40, 255);
    const PURPLE: Rgb<u8> = Rgb::new(187, 0, 255);
}

fn erase(display: &mut Display, color: Rgb<u8>) {
    display.fill(&Rect::new([0, 0], [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION]), color)
}

fn draw_rounded_rect(display: &mut Display, start: (i16, i16), end: (i16, i16), rad: u8, color: Rgb<u8>) {
    let irad = i16::from(rad);
    let smr = (start.0 + irad, start.1 + irad);
    let epr = (end.0 - irad, end.1 - irad);
    display.fill(&Circle::new(Point2::from([smr.0, smr.1]), u16::from(rad)), color);
    display.fill(&Circle::new(Point2::from([epr.0, smr.1]), u16::from(rad)), color);
    display.fill(&Circle::new(Point2::from([epr.0, epr.1]), u16::from(rad)), color);
    display.fill(&Circle::new(Point2::from([smr.0, epr.1]), u16::from(rad)), color);
    display.fill(&Rect::new([smr.0, start.1], [epr.0, end.1]), color);
    display.fill(&Rect::new([start.0, smr.1], [end.0, epr.1]), color);
}

fn draw_text(disp: &mut Display, text: &str, pos: [i16; 2], size: f32, color: Rgb<u8>, bg_col: Rgb<u8>) {
    disp.draw_text(&Text::new(text, 
            Font::new(FontSize::from_float(size / 65.33).unwrap(), FontFamily::Monospace),
            pos
        ),
        color, Some(bg_col)
    );
}

fn draw_motor_status(display: &mut Display, m: (usize, &mut NamedMotor)) {
    draw_text(display, m.1.name_short, 
        [12, (m.0 * 24 + 12).try_into().unwrap_or_default()], 18.0, 
        if m.1.motor.is_connected() && m.1.motor.is_exp() { Colors::PURPLE }
        else if !m.1.motor.is_connected() { Colors::MAROON } else { match m.1.motor.gearset().unwrap() {
            Gearset::Red => Colors::RED, Gearset::Green => Colors::GREEN, Gearset::Blue => Colors::LIGHT_BLUE }
        }, Colors::BG_2
    );
    draw_text(display, &format!("{:.2}°", m.1.get_pos_degrees().unwrap_or(f64::NAN)),
        [60, (m.0 * 24 + 12).try_into().unwrap_or_default()], 18.0, Colors::TEXT_1, Colors::BG_2
    );
    draw_text(display, &format!("{:.2}°C", m.1.get_temp().unwrap_or(f64::NAN)),
        [162, (m.0 * 24 + 12).try_into().unwrap_or_default()], 18.0, match m.1.get_temp() {
            Some(..=55.0) => Colors::GREEN, Some(55.0..=60.0) => Colors::YELLOW, Some(60.0..=65.0) => Colors::ORANGE, Some(65.0..) => Colors::RED,
            Some(_) => Colors::MAROON, None => Colors::MAROON,
        }, Colors::BG_2
    );
}

#[derive(Debug, Clone)]
pub(crate) struct Gui {
    robot: Rc<RefCell<Robot>>,
    comp_cont: Rc<RefCell<CompController>>,
}

impl Gui {
    pub fn new(robot: Rc<RefCell<Robot>>, comp_cont: Rc<RefCell<CompController>>) -> Self {
        Self { robot, comp_cont }
    }

    pub async fn render_loop(&mut self) {
        let mut robot = self.robot.borrow_mut();
        let mut disp = robot.take_display().unwrap();
        drop(robot);
        disp.set_render_mode(RenderMode::DoubleBuffered);
        let comp_gui_frametime = Duration::from_secs_f64(1.0);
        let gui_frametime = Duration::from_secs_f64(0.1);
        loop {
            let comp_cont = *self.comp_cont.borrow();
            if comp_cont.match_start_timer.running && !comp_cont.match_start_timer.finished {
                erase(&mut disp, Colors::BG_1);
                draw_rounded_rect(&mut disp, (6, 6), (474, 36), 6, Colors::BG_2);
                draw_text(&mut disp, &format!("Starting {:?}!", comp_cont.state), [12, 12], 18.0, Colors::TEXT_2, Colors::BG_2);
                draw_text(&mut disp, &format!("{:1.0}", comp_cont.match_start_timer.time / 1000.0), [198, 42], 168.0, Colors::TEXT_1, Colors::BG_1);
                disp.render();
                sleep(comp_gui_frametime).await;
            } else if comp_cont.state == CompContState::Disabled {
                erase(&mut disp, Colors::BG_1);

                disp.render();
                sleep(gui_frametime).await;
            } else {
                erase(&mut disp, Colors::BG_1);

                draw_rounded_rect(&mut disp, (6, 6), (237, 234), 12, Colors::BG_2);
                self.robot.borrow_mut().drive.as_mut().unwrap()
                    .left_motors.borrow_mut().iter_mut().enumerate()
                    .for_each(|m| draw_motor_status(&mut disp, m));
                self.robot.borrow_mut().drive.as_mut().unwrap()
                    .right_motors.borrow_mut().iter_mut().enumerate()
                    .for_each(|m| draw_motor_status(&mut disp, (m.0 + 3, m.1)));
                draw_motor_status(&mut disp, (6, &mut self.robot.borrow_mut().intake.as_mut().unwrap().borrow_mut().motor_1));
                draw_motor_status(&mut disp, (7, &mut self.robot.borrow_mut().intake.as_mut().unwrap().borrow_mut().motor_2));
                draw_motor_status(&mut disp, (8, &mut self.robot.borrow_mut().indexer.as_mut().unwrap().borrow_mut()));
                
                draw_rounded_rect(&mut disp, (243, 6), (474, 234), 12, Colors::BG_2);
                draw_text(&mut disp, "Controls:", [249, 12], 16.0, Colors::TEXT_1, Colors::BG_2);
                disp.fill(&Line::new([255, 38], [468, 38]), Colors::TEXT_3);
                draw_text(&mut disp, "Drive: Tank", [249, 48], 16.0, Colors::TEXT_1, Colors::BG_2);
                draw_text(&mut disp, "Intake: R1 Forward, R2 Back", [249, 72], 16.0, Colors::TEXT_1, Colors::BG_2);
                draw_text(&mut disp, "Indexer: R1 Forward, R2 Back", [249, 96], 16.0, Colors::TEXT_1, Colors::BG_2);
                draw_text(&mut disp, "Scraper: B", [249, 120], 16.0, Colors::TEXT_1, Colors::BG_2);
                disp.fill(&Line::new([255, 144], [468, 144]), Colors::TEXT_3);
                draw_text(&mut disp, &format!("Battery: {:.0}%", battery::capacity() * 100.0), [249, 156], 16.0, 
                    match battery::capacity() {
                        0.25..=1.0 => Colors::GREEN,
                        0.125..=0.25 => Colors::YELLOW,
                        ..=0.125 => Colors::RED,
                        _ => Colors::TEXT_2
                    }, Colors::BG_2);
                draw_text(&mut disp, &format!("Battery Temp: {:.0}°C", battery::temperature()), [249, 180], 16.0,
                    match battery::temperature() as f64 {
                        ..=35.0 => Colors::GREEN,
                        35.0..=40.0 => Colors::YELLOW,
                        40.0.. => Colors::RED,
                        _ => Colors::MAROON
                    }, Colors::BG_2);

                disp.render();
                sleep(comp_gui_frametime).await;
            }
        }
    }
}