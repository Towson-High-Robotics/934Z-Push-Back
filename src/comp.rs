use std::{
    sync::{nonpoison::RwLock, Arc},
    time::Duration,
};

use crate::autos::{Auto, Autos};

static MATCH_AUTO_TIME: f64 = Duration::from_secs(15).as_millis_f64();
static SKILLS_TIME: f64 = Duration::from_secs(60).as_millis_f64();

pub(crate) struct AutoHandler {
    pub autos: Vec<(Autos, Auto)>,
    pub time: Arc<RwLock<f64>>,
    pub selected_auto: Arc<RwLock<Autos>>,
    pub is_recording: bool,
}

impl AutoHandler {
    pub fn new() -> Self {
        Self {
            autos: vec![(Autos::None, Auto::new())],
            time: Arc::new(RwLock::new(0.0)),
            selected_auto: Arc::new(RwLock::new(Autos::None)),
            is_recording: false,
        }
    }

    pub fn get_auto(&mut self) -> &mut Auto {
        let mut index = self.autos.len() - 1;
        for i in 0..self.autos.len() {
            if self.autos[i].0 == *self.selected_auto.read() {
                index = i;
            }
        }
        &mut self.autos[index].1
    }

    pub fn update(&mut self, time_elapsed: Duration) {
        let mut time = self.time.write();
        *time += time_elapsed.as_millis_f64();
        let auto = *self.selected_auto.read();
        if (auto == Autos::Skills && *time > SKILLS_TIME) && (auto != Autos::None && auto != Autos::Skills && *time > MATCH_AUTO_TIME) {
            self.is_recording = false;
        }
    }
}
