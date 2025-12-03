use core::time::Duration;
use std::{
    ffi::CString,
    format,
    sync::{nonpoison::RwLock, Arc},
};

use vexide::{battery, color::Color, display::*, math::Point2, time::sleep};

use crate::util::Telem;

#[allow(unused)]
mod colors {
    use vexide::color::Color;

    pub const BG_1: Color = Color::new(10, 15, 30);
    pub const BG_2: Color = Color::new(30, 35, 50);
    pub const BG_3: Color = Color::new(40, 50, 60);
    pub const TEXT_1: Color = Color::new(230, 235, 255);
    pub const TEXT_2: Color = Color::new(200, 207, 220);
    pub const TEXT_3: Color = Color::new(160, 170, 190);
    pub const MAROON: Color = Color::new(205, 0, 0);
    pub const RED: Color = Color::new(255, 20, 0);
    pub const ORANGE: Color = Color::new(255, 187, 0);
    pub const YELLOW: Color = Color::new(255, 255, 0);
    pub const GREEN: Color = Color::new(0, 255, 0);
    pub const LIGHT_BLUE: Color = Color::new(0, 187, 255);
    pub const BLUE: Color = Color::new(0, 40, 255);
    pub const PURPLE: Color = Color::new(187, 0, 255);
}

#[allow(unused)]
mod sizes {
    use vexide::display::FontSize;
    pub const SMALL: FontSize = FontSize::new(12, 65);
    pub const MEDIUM: FontSize = FontSize::new(16, 65);
    pub const LARGE: FontSize = FontSize::new(24, 65);
}

#[allow(unused)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
enum GuiState {
    // Left Side Views
    MotorView,
    SensorView,
    AutoSelectorOverview,
    AutoSelectorMatch,
    // Right Side Views
    ControlsView,
    OdomCalibrateView,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum MotorType {
    Disconnected,
    Exp,
    Red,
    Green,
    Blue,
}

fn erase(display: &mut Display, color: Color) { display.fill(&Rect::new([0, 0], [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION]), color) }

fn draw_rounded_rect(display: &mut Display, start: (i16, i16), end: (i16, i16), rad: u8, color: Color) {
    let irad = i16::from(rad);
    let (smr, epr) = ((start.0 + irad, start.1 + irad), (end.0 - irad, end.1 - irad));
    display.fill(&Circle::new([smr.0, smr.1], u16::from(rad)), color);
    display.fill(&Circle::new([epr.0, smr.1], u16::from(rad)), color);
    display.fill(&Circle::new([epr.0, epr.1], u16::from(rad)), color);
    display.fill(&Circle::new([smr.0, epr.1], u16::from(rad)), color);
    display.fill(&Rect::new([smr.0, start.1], [epr.0, end.1]), color);
    display.fill(&Rect::new([start.0, smr.1], [end.0, epr.1]), color);
}

fn draw_text(disp: &mut Display, text: &str, pos: [i16; 2], size: FontSize, color: Color, bg_col: Color) {
    let c_str = CString::new(text.to_string()).unwrap();
    disp.draw_text(&Text::new(c_str.as_c_str(), Font::new(size, FontFamily::Monospace), pos), color, Some(bg_col));
}

fn normal_text(disp: &mut Display, text: &str, pos: [i16; 2]) { draw_text(disp, text, pos, sizes::MEDIUM, colors::TEXT_1, colors::BG_2); }

fn normal_bg_text(disp: &mut Display, text: &str, pos: [i16; 2], color: Color) { draw_text(disp, text, pos, sizes::MEDIUM, color, colors::BG_2); }

fn draw_motor_status(display: &mut Display, motor_state: (&'static str, MotorType, f64, f64), pos: [i16; 2]) {
    let (name, motor_type, heading, temperature) = motor_state;
    normal_bg_text(display, name, pos, match motor_type {
        MotorType::Disconnected => colors::MAROON,
        MotorType::Exp => colors::PURPLE,
        MotorType::Red => colors::RED,
        MotorType::Green => colors::GREEN,
        MotorType::Blue => colors::LIGHT_BLUE,
    });

    normal_bg_text(display, &format!("{:.0}°", heading), [pos[0] + 52, pos[1]], colors::TEXT_1);

    normal_bg_text(display, &format!("{:.2}°C", temperature), [pos[0] + 116, pos[1]], match temperature {
        ..=55.0 => colors::GREEN,
        55.0..=60.0 => colors::YELLOW,
        60.0..=65.0 => colors::ORANGE,
        _ => colors::RED,
    });
}

fn draw_motor_satus_panel(disp: &mut Display, telem: &Telem) {
    draw_rounded_rect(disp, (6, 6), (237, 234), 6, colors::BG_2);
    if telem.motor_names.is_empty() || telem.motor_types.is_empty() {
        return;
    }
    for i in 0..(telem.motor_names.len()) {
        draw_motor_status(disp, (telem.motor_names[i], telem.motor_types[i], telem.motor_headings[i], telem.motor_temperatures[i]), [12, i as i16 * 18 + 12]);
    }
}

fn draw_sensor_panel(disp: &mut Display, telem: &Telem) {
    draw_rounded_rect(disp, (6, 6), (237, 234), 6, colors::BG_2);
    if telem.sensor_names.is_empty() || telem.sensor_status.is_empty() {
        return;
    }
    for i in 0..(telem.sensor_names.len()) {
        normal_bg_text(
            disp,
            &format!("{}: {:.1}", telem.sensor_names[i], telem.sensor_values[i]),
            [12, i as i16 * 18 + 12],
            if telem.sensor_status[i] { colors::TEXT_1 } else { colors::MAROON },
        );
    }
    draw_text(disp, &format!("Pose: {:.1}, {:.1}, {:.1}", telem.pose.0, telem.pose.1, telem.pose.2), [12, 198], sizes::MEDIUM, colors::TEXT_1, colors::BG_2);
    draw_text(disp, &format!("Offsets: {:.2},{:.2}", telem.offsets.0, telem.offsets.1), [12, 216], sizes::MEDIUM, colors::TEXT_1, colors::BG_2);
}

fn draw_auto_overview(disp: &mut Display) {
    draw_rounded_rect(disp, (6, 6), (237, 234), 6, colors::BG_2);
    draw_rounded_rect(disp, (9, 8), (234, 119), 6, colors::RED);
    draw_rounded_rect(disp, (9, 121), (120, 232), 6, colors::GREEN);
    draw_rounded_rect(disp, (123, 121), (234, 232), 6, colors::BLUE);
    draw_text(disp, "Match", [52, 43], sizes::MEDIUM, colors::TEXT_2, colors::RED);
    draw_text(disp, "Skills", [39, 138], sizes::MEDIUM, colors::TEXT_2, colors::GREEN);
    draw_text(disp, "None", [164, 150], sizes::MEDIUM, colors::TEXT_2, colors::BLUE);
}

fn draw_auto_selector(disp: &mut Display) {
    draw_rounded_rect(disp, (6, 6), (237, 234), 6, colors::BG_2);
    draw_rounded_rect(disp, (9, 8), (237, 80), 6, colors::RED);
    draw_rounded_rect(disp, (9, 82), (237, 154), 6, colors::GREEN);
    draw_rounded_rect(disp, (9, 156), (237, 228), 6, colors::BLUE);
    draw_text(disp, "Left", [91, 36], sizes::MEDIUM, colors::TEXT_1, colors::RED);
    draw_text(disp, "Solo", [91, 110], sizes::MEDIUM, colors::TEXT_1, colors::GREEN);
    draw_text(disp, "Right", [83, 184], sizes::MEDIUM, colors::TEXT_1, colors::BLUE);
}

#[derive(Debug)]
pub(crate) struct Gui {
    disp: Display,
    left_split: GuiState,
    _right_split: GuiState,
    telem: Arc<RwLock<Telem>>,
    prev_press: TouchState,
}

impl Gui {
    pub fn new(disp: Display, telem: Arc<RwLock<Telem>>) -> Self {
        Self {
            disp,
            left_split: GuiState::MotorView,
            _right_split: GuiState::ControlsView,
            telem,
            prev_press: TouchState::Released,
        }
    }

    fn in_range(pos: Point2<i16>, x: (i16, i16), y: (i16, i16)) -> bool { x.0 <= pos.x && pos.x <= x.1 && y.0 <= pos.y && pos.y <= y.1 }

    pub async fn render_loop(&mut self) {
        self.disp.set_render_mode(RenderMode::DoubleBuffered);
        loop {
            erase(&mut self.disp, colors::BG_1);

            if let Ok(mut t) = self.telem.try_write() {
                if t.selector_active {
                    self.left_split = GuiState::AutoSelectorOverview;
                    t.selector_active = false;
                }
            }

            let touch = self.disp.touch_status();
            match self.left_split {
                GuiState::MotorView => {
                    if let Ok(t) = self.telem.try_read() {
                        draw_motor_satus_panel(&mut self.disp, &t);
                        if Self::in_range(touch.point, (6, 237), (6, 234)) && self.prev_press == TouchState::Released && touch.state != TouchState::Released {
                            self.left_split = GuiState::SensorView;
                        }
                    }
                }
                GuiState::SensorView => {
                    if let Ok(t) = self.telem.try_read() {
                        draw_sensor_panel(&mut self.disp, &t);
                        if self.prev_press == TouchState::Released && Self::in_range(touch.point, (6, 237), (6, 234)) && touch.state != TouchState::Released {
                            self.left_split = GuiState::MotorView;
                        }
                    };
                }
                GuiState::AutoSelectorOverview => {
                    draw_auto_overview(&mut self.disp);
                    if self.prev_press == TouchState::Released && touch.state != TouchState::Released {
                        if Self::in_range(touch.point, (9, 234), (8, 119)) {
                            self.left_split = GuiState::AutoSelectorMatch;
                        } else if Self::in_range(touch.point, (9, 120), (82, 232)) {
                            self.telem.write().auto = crate::autos::Autos::Skills;
                            self.left_split = GuiState::MotorView;
                        } else if Self::in_range(touch.point, (123, 234), (156, 232)) {
                            self.telem.write().auto = crate::autos::Autos::None;
                            self.left_split = GuiState::MotorView;
                        }
                    }
                }
                GuiState::AutoSelectorMatch => {
                    draw_auto_selector(&mut self.disp);
                    if self.prev_press == TouchState::Released && touch.state != TouchState::Released {
                        if Self::in_range(touch.point, (9, 237), (8, 80)) {
                            self.telem.write().auto = crate::autos::Autos::Left;
                        } else if Self::in_range(touch.point, (9, 237), (121, 154)) {
                            self.telem.write().auto = crate::autos::Autos::Solo;
                        } else if Self::in_range(touch.point, (9, 237), (121, 228)) {
                            self.telem.write().auto = crate::autos::Autos::Right;
                        }
                        self.left_split = GuiState::MotorView;
                    }
                }
                _ => {
                    self.left_split = GuiState::MotorView;
                }
            }
            self.prev_press = touch.state;

            draw_rounded_rect(&mut self.disp, (243, 6), (474, 234), 12, colors::BG_2);
            normal_text(&mut self.disp, "Controls:", [249, 12]);
            self.disp.fill(&Line::new([255, 38], [468, 38]), colors::TEXT_3);
            normal_text(&mut self.disp, "Drive: Tank", [249, 48]);
            normal_text(&mut self.disp, "Intake: R1 Forward, R2 Back", [249, 72]);
            normal_text(&mut self.disp, "Indexer: R1 Forward, R2 Back", [249, 96]);
            normal_text(&mut self.disp, "Scraper: B", [249, 120]);
            self.disp.fill(&Line::new([255, 144], [468, 144]), colors::TEXT_3);

            normal_bg_text(&mut self.disp, &format!("Battery: {:.0}%", battery::capacity() * 100.0), [249, 156], match battery::capacity() {
                ..=0.25 => colors::RED,
                0.25..=0.5 => colors::YELLOW,
                0.5..=1.0 => colors::GREEN,
                _ => colors::TEXT_2,
            });

            normal_bg_text(&mut self.disp, &format!("Battery Temp: {:.0}°C", battery::temperature()), [249, 180], match battery::temperature() {
                ..=35 => colors::GREEN,
                36..=40 => colors::YELLOW,
                41.. => colors::RED,
            });

            self.disp.render();
            sleep(Duration::from_millis(250)).await;
        }
    }
}
