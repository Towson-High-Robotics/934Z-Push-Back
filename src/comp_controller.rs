use vexide::{devices::controller::ControllerState, time::Instant};

use crate::util::Robot;


#[derive(PartialEq)]
pub(crate) enum CompContState {
    Driver,
    Auto,
    SkillsDriver,
    SkillsAuto,
    Disabled,
    Off
}

#[derive(PartialEq)]
pub(crate) enum Auto {
    LeftRed,
    RightRed,
    LeftBlue,
    RightBlue,
    RedSoloAWP,
    BlueSoloAWP,
    Disabled,
    Skills,
    SkillsDriver
}

pub struct CountdownTimer {
    pub running: bool,
    pub finished: bool,
    time: f64,
    max_time: f64,
    last_update: Instant
}

impl CountdownTimer {
    pub fn new(max_time: f64) -> CountdownTimer {
        CountdownTimer {
            running: false,
            finished: false,
            time: 0.,
            max_time,
            last_update: Instant::now()
        }
    }

    pub fn start(&mut self) {
        self.running = true;
        self.last_update = Instant::now();
    }

    pub fn reset(&mut self) {
        self.running = false;
        self.finished = false;
        self.time = 0.;
    }

    pub fn update(&mut self) {
        if !self.running { return; }
        let now = Instant::now();
        self.time += now.clone().checked_duration_since(self.last_update).unwrap_or_default().as_millis_f64();
        self.last_update = now;
        if self.time >= self.max_time {
            self.running = false;
            self.finished = true;
        }
    }

    pub fn unchecked_update(&mut self) {
        let now = Instant::now();
        self.time += now.clone().checked_duration_since(self.last_update).unwrap_or_default().as_millis_f64();
        self.last_update = now;
    }
}

pub(crate) struct VirtCompContState {
    pub state: CompContState,
    pub selected_auto: Auto,
    pub sim_match: bool,
    pub awaiting_start: bool,
    match_start: CountdownTimer,
    pub auto_counter: CountdownTimer,
    pub driver_counter: CountdownTimer,
    pub skills_counter: CountdownTimer
}

impl VirtCompContState {
    pub fn new() -> VirtCompContState {
        VirtCompContState {
            state: CompContState::Driver,
            selected_auto: Auto::Disabled,
            sim_match: false,
            awaiting_start: true,
            match_start: CountdownTimer::new(3000.),
            auto_counter: CountdownTimer::new(15000.),
            driver_counter: CountdownTimer::new(105000.),
            skills_counter: CountdownTimer::new(60000.)
        }
    }

    pub fn cont_keybinds(&mut self, cont: ControllerState) {
        if self.state == CompContState::Disabled {
            if cont.button_x.is_pressed() { self.state = CompContState::Off; }
            else if cont.button_up.is_pressed() { self.awaiting_start = true;
                self.state = CompContState::Auto;
            } else if cont.button_down.is_pressed() { self.awaiting_start = true;
                self.state = CompContState::SkillsDriver;
                self.selected_auto = Auto::SkillsDriver;
            } else if cont.button_right.is_pressed() { self.awaiting_start = true;
                self.state = CompContState::SkillsAuto;
                self.selected_auto = Auto::Skills;
            } else if cont.button_a.is_pressed() { self.awaiting_start = false; }
            else if cont.button_b.is_pressed() { self.awaiting_start = true; 
                self.state = CompContState::Disabled;
                self.selected_auto = Auto::Disabled;
            }
        } else if cont.button_x.is_pressed() && cont.button_r1.is_pressed() && cont.button_r2.is_pressed() {
            self.awaiting_start = true; 
            self.state = CompContState::Disabled;
            self.selected_auto = Auto::Disabled;
        }
    }
}

impl Robot {
    pub fn comp_cont_handle(&mut self) {
        if self.cont.awaiting_start { return; }
        match self.cont.state {
            CompContState::Driver => {
                self.driver_tick();
                self.cont.driver_counter.update();
                if self.cont.driver_counter.finished {
                    self.cont.state = CompContState::Disabled;
                    self.cont.awaiting_start = true;
                }
            },
            CompContState::Auto => {
                self.auto_tick();
                self.cont.auto_counter.update();
                if self.cont.auto_counter.finished {
                    self.cont.state = CompContState::Driver;
                    self.cont.awaiting_start = true;
                }
            },
            CompContState::SkillsDriver => {
                self.driver_tick();
                self.cont.skills_counter.update();
                if self.cont.skills_counter.finished {
                    self.cont.state = CompContState::Disabled;
                    self.cont.awaiting_start = true;
                }
            },
            CompContState::SkillsAuto => {
                self.auto_tick();
                self.cont.skills_counter.update();
                if self.cont.skills_counter.finished {
                    self.cont.state = CompContState::Disabled;
                    self.cont.awaiting_start = true;
                }
            },
            CompContState::Disabled => {},
            CompContState::Off => {
                self.driver_tick();
                self.cont.driver_counter.unchecked_update();
            },
        }
    }
}