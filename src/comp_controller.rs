<<<<<<< HEAD
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
=======
use vexide::time::Instant;
use vexide::prelude::Controller;

use crate::autos::SelectedAuto;
use crate::util::Robot;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum CompContState {
    Disabled,
    Auto,
    Driver,
    SkillsAuto,
    SkillsDriver,
    Off
}

impl Default for CompContState { fn default() -> Self { Self::Disabled } }

#[derive(Debug, Clone, Copy)]
pub(crate) struct Timer {
    pub finished: bool,
    running: bool,
    pub max_time: f64,
    pub time: f64,
    last_update: Instant
}

impl Timer {
    pub fn new(max_time: f64) -> Self {
        Self { finished: false, running: false, max_time, time: 0.0, last_update: Instant::now() }
    }

    pub fn reset(&mut self) {
        self.finished = false; self.running = false; self.time = 0.0;
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
    }

    pub fn start(&mut self) {
        self.running = true;
        self.last_update = Instant::now();
    }

<<<<<<< HEAD
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
=======
    pub fn update(&mut self) {
        if self.running {
            let now = Instant::now();
            self.time += now.clone().duration_since(self.last_update).as_millis_f64();
            self.last_update = now;
            if self.time >= self.max_time {
                self.finished = true; self.running = false;
            }
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
        }
    }

    pub fn unchecked_update(&mut self) {
        let now = Instant::now();
<<<<<<< HEAD
        self.time += now.clone().checked_duration_since(self.last_update).unwrap_or_default().as_millis_f64();
=======
        self.time += now.clone().duration_since(self.last_update).as_millis_f64();
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
        self.last_update = now;
    }
}

<<<<<<< HEAD
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
=======
impl Default for Timer { fn default() -> Self { Self::new(0.0) } }

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct CompController {
    pub state: CompContState,
    pub auto: SelectedAuto,
    pub auto_timer: Timer,
    pub driver_timer: Timer,
    pub skills_timer: Timer,
    sim_match: bool,
    awaiting_start: bool,
    match_start_timer: Timer
}

impl CompController {
    pub fn new() -> CompController {
        CompController {
            state: CompContState::Disabled,
            auto: SelectedAuto::None,
            auto_timer: Timer::new(15000.0),
            driver_timer: Timer::new(105000.0),
            skills_timer: Timer::new(60000.0),
            sim_match: false,
            awaiting_start: true,
            match_start_timer: Timer::new(3000.0)
        }
    }

    pub fn controller_handle(&mut self, cont: &mut Controller) {
        let state = match cont.state() {
            Ok(s) => s,
            Err(_) => return
        };
        self.match_start_timer.update();
        if self.sim_match && self.match_start_timer.finished {
            self.awaiting_start = false; self.match_start_timer.reset();
        }

        if state.button_x.is_pressed() && state.button_r1.is_pressed() && state.button_r2.is_pressed() && !self.awaiting_start {
            self.awaiting_start = true; self.state = CompContState::Disabled; self.sim_match = false;
        }

        if state.button_a.is_pressed() { self.match_start_timer.start(); }
        else if state.button_b.is_pressed() { self.state = CompContState::Disabled; self.sim_match = false; }
        else if state.button_x.is_pressed() { self.state = CompContState::Off; }
        else if state.button_up.is_pressed() { self.state = CompContState::Auto; self.sim_match = true; }
        else if state.button_down.is_pressed() { self.state = CompContState::SkillsAuto; self.sim_match = true; }
        else if state.button_right.is_pressed() { self.state = CompContState::Driver; self.sim_match = true; }
        else if state.button_left.is_pressed() { self.state = CompContState::SkillsDriver; self.sim_match = true; }
    }

    pub fn update_state(&mut self) {
        match self.state {
            CompContState::Disabled => {},
            CompContState::Auto => if self.auto_timer.finished {
                self.state = CompContState::Driver; self.awaiting_start = true; self.match_start_timer.start();
            },
            CompContState::Driver => if self.driver_timer.finished {
                self.state = CompContState::Disabled; self.sim_match = false; self.awaiting_start = true;
            },
            CompContState::SkillsAuto => if self.skills_timer.finished {
                self.state = CompContState::Disabled; self.sim_match = false; self.awaiting_start = true;
            },
            CompContState::SkillsDriver => if self.skills_timer.finished {
                self.state = CompContState::Disabled; self.sim_match = false; self.awaiting_start = true;
            },
            CompContState::Off => {},
        }
    }

    pub fn comp_controller_update(&mut self, robot: &mut Robot) {
        if self.awaiting_start && self.state != CompContState::Off { return; }
        match self.state {
            CompContState::Disabled => {},
            CompContState::Auto => {
                self.driver_timer.update();
                robot.auto_tick();
            },
            CompContState::Driver => {
                self.driver_timer.update();
                robot.driver_tick();
            },
            CompContState::SkillsAuto => {
                self.skills_timer.update();
                robot.auto_tick();
            },
            CompContState::SkillsDriver => {
                self.skills_timer.update();
                robot.driver_tick();
            },
            CompContState::Off => {
                self.driver_timer.unchecked_update();
                robot.driver_tick();
            },
        }
        self.update_state();
>>>>>>> 608ab7c (Refactor #2 + Setup for Odometry)
    }
}