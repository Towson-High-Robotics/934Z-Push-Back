use std::{
    sync::{nonpoison::RwLock, Arc},
    time::Duration,
};

use crate::{
    autos::{
        auto::{Action, Auto, Autos},
        path::{Curve, PathSegment, SpeedCurve}
    },
    cubreg::curve_reg,
    util::dot
};

static MATCH_AUTO_TIME: f64 = Duration::from_secs(15).as_millis() as f64;
static SKILLS_TIME: f64 = Duration::from_secs(60).as_millis() as f64;

pub(crate) struct AutoHandler {
    pub autos: Vec<(Autos, Auto)>,
    pub start_time: Instant,
    pub selected_auto: Arc<RwLock<Autos>>,
    pub is_recording: bool,
    pub start_recording: bool,
    pub recorded_poses: Vec<((f64, f64, f64), f64)>,
    pub recorded_actions: Vec<(Action, f64)>,
}

impl AutoHandler {
    pub fn new() -> Self {
        Self {
            autos: vec![],
            start_time: Instant::now(),
            selected_auto: Arc::new(RwLock::new(Autos::None)),
            is_recording: false,
            start_recording: false,
            recorded_poses: vec![],
            recorded_actions: vec![]
        }
    }

    pub fn get_auto(&mut self) -> &mut Auto {
        &mut self.autos.iter_mut().find(|a| a.0 == *self.selected_auto.read()).unwrap().1
    }

    pub fn update(&mut self, time_elapsed: Duration) {
        let time = self.start_time.elapsed().as_millis() as f64;
        let auto = *self.selected_auto.read();
        if (auto == Autos::Skills && time > SKILLS_TIME) && (auto != Autos::None && auto != Autos::Skills && time > MATCH_AUTO_TIME) {
            self.is_recording = false;
            self.process_recording();
        }
    }

    fn process_recording(&mut self) {
        let path_len = self.recorded_poses.len();
        if path_len == 0 {
            return;
        }
        if path_len <= 25 {
            let curve = curve_reg(self.recorded_poses.iter().map(|v| v.0.0).collect(), self.recorded_poses.iter().map(|v| v.0.1).collect(), self.recorded_poses.iter().map(|v| v.1).collect());
            let mut auto = Auto::new();
            auto.add_curves(vec![PathSegment {
                curve: Box::new(curve.clone()), speed: SpeedCurve::new_linear(1.0, 1.0),
                end_heading: self.recorded_poses.last().unwrap().1,
                reversed_drive: dot((curve.sample_heading(0.5).cos(), curve.sample_heading(0.5).sin()), (self.recorded_poses.first().unwrap().0.2.cos(), self.recorded_poses.first().unwrap().0.2.sin())) < 0.0,
                timeout: self.recorded_poses.last().unwrap().1 - self.recorded_poses.first().unwrap().1,
                wait_time: 0.0
            }]);
            println!("{:?}", auto);
        }
    }
}
