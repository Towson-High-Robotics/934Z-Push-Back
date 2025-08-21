use vexide::{devices::controller::ControllerState, prelude::*, time::Instant};

use crate::{autos::Autos, util::Robot};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum CompContState {
    Disabled,
    Auto,
    Driver,
    Skills,
    Off,
}

impl Default for CompContState {
    fn default() -> Self { Self::Disabled }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Timer {
    pub finished: bool,
    pub running: bool,
    pub max_time: f64,
    pub time: f64,
    last_update: Instant,
}

impl Timer {
    pub fn new(max_time: f64) -> Self {
        Self {
            finished: false,
            running: false,
            max_time,
            time: 0.0,
            last_update: Instant::now(),
        }
    }

    pub fn reset(&mut self) {
        self.finished = false;
        self.running = false;
        self.time = 0.0;
    }

    pub fn start(&mut self) {
        self.running = true;
        self.last_update = Instant::now();
    }

    pub fn update(&mut self) {
        if self.running {
            let now = Instant::now();
            self.time += now.duration_since(self.last_update).as_millis_f64();
            self.last_update = now;
            if self.time >= self.max_time {
                self.finished = true;
                self.running = false;
            }
        }
    }

    pub fn unchecked_update(&mut self) {
        let now = Instant::now();
        self.time += now.duration_since(self.last_update).as_millis_f64();
        self.last_update = now;
    }
}

impl Default for Timer {
    fn default() -> Self { Self::new(0.0) }
}

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct CompController {
    pub state: CompContState,
    pub auto: Autos,
    pub auto_timer: Timer,
    pub driver_timer: Timer,
    pub skills_timer: Timer,
    pub sim_match: bool,
    pub awaiting_start: bool,
    pub match_start_timer: Timer,
}

impl CompController {
    pub fn new() -> CompController {
        CompController {
            state: CompContState::Off,
            auto: Autos::None,
            auto_timer: Timer::new(15000.0),
            driver_timer: Timer::new(105000.0),
            skills_timer: Timer::new(60000.0),
            sim_match: false,
            awaiting_start: true,
            match_start_timer: Timer::new(3000.0),
        }
    }

    pub async fn controller_handle(&mut self, state: ControllerState, cont: &mut Controller) {
        self.match_start_timer.update();
        if self.sim_match && self.match_start_timer.finished {
            println!("Starting Match!");
            self.awaiting_start = false;
            self.match_start_timer.reset();
            match self.state {
                CompContState::Auto => {
                    self.auto_timer.start();
                }
                CompContState::Driver => {
                    self.driver_timer.start();
                }
                CompContState::Skills => {
                    self.skills_timer.start();
                }
                _ => (),
            };
        }

        if state.button_x.is_pressed()
            && state.button_r1.is_pressed()
            && state.button_r2.is_pressed()
            && (self.state == CompContState::Off || !self.awaiting_start)
        {
            println!("Controller Reset!");
            self.awaiting_start = true;
            self.state = CompContState::Disabled;
            self.sim_match = false;
        }

        if !self.awaiting_start || self.state == CompContState::Off {
            return;
        }

        if state.button_a.is_now_pressed() && self.state != CompContState::Disabled {
            println!("Starting in 3 seconds!");
            self.match_start_timer.start();
            cont.rumble("- - -").await.unwrap();
        } else if state.button_b.is_now_pressed() {
            println!("Cancelled!");
            self.state = CompContState::Disabled;
            self.sim_match = false;
        } else if state.button_x.is_now_pressed() && !(state.button_r1.is_pressed() && state.button_r2.is_pressed()) {
            println!("Disabled Controller!");
            self.state = CompContState::Off;
        } else if state.button_up.is_now_pressed() {
            println!("Tournament Match Selected!");
            self.state = CompContState::Auto;
            self.sim_match = true;
        } else if state.button_down.is_now_pressed() {
            println!("Skills Autonomous Selected!");
            self.state = CompContState::Skills;
            self.auto = Autos::Skills;
            self.sim_match = true;
        } else if state.button_right.is_now_pressed() {
            println!("Tournament Driver Control Selected!");
            self.state = CompContState::Driver;
            self.sim_match = true;
        } else if state.button_left.is_now_pressed() {
            println!("Driver Skills Selected!");
            self.state = CompContState::Skills;
            self.auto = Autos::SkillsDriver;
            self.sim_match = true;
        }
    }

    pub fn update_state(&mut self) {
        self.awaiting_start = true;
        match self.state {
            CompContState::Disabled => {}
            CompContState::Auto => {
                if self.auto_timer.finished {
                    println!("Auto period ended! Press A to go to Driver!");
                    self.state = CompContState::Driver;
                }
            }
            CompContState::Driver => {
                if self.driver_timer.finished {
                    println!("Match Ended!");
                    self.state = CompContState::Disabled;
                    self.sim_match = false;
                }
            }
            CompContState::Skills => {
                if self.skills_timer.finished {
                    println!("Match Ended!");
                    self.state = CompContState::Disabled;
                    self.sim_match = false;
                }
            }
            CompContState::Off => {}
        }
    }

    pub fn comp_controller_update(&mut self, robot: &mut Robot, state: ControllerState) {
        if self.awaiting_start && self.state != CompContState::Off {
            return;
        }
        match self.state {
            CompContState::Disabled => {}
            CompContState::Auto => {
                self.auto_timer.update();
                robot.auto_tick();
            }
            CompContState::Driver => {
                self.driver_timer.update();
                robot.driver_tick(state);
            }
            CompContState::Skills => {
                self.skills_timer.update();
                if self.auto == Autos::SkillsDriver {
                    robot.driver_tick(state);
                } else {
                    robot.auto_tick();
                }
            }
            CompContState::Off => {
                self.driver_timer.unchecked_update();
                robot.driver_tick(state);
            }
        }
        self.update_state();
    }
}
