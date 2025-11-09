use std::{
    sync::{nonpoison::Mutex, Arc},
    time::Duration,
};

use crate::autos::{Auto, Autos};

// static MATCH_AUTO_TIME: Duration = Duration::from_secs(15);
// static MATCH_DRIVER_TIME: Duration = Duration::from_secs(105);
// static SKILLS_TIME: Duration = Duration::from_secs(60);

#[derive(Debug)]
pub(crate) struct CompHandler {
    pub autos: Vec<(Autos, Auto)>,
    pub time: Arc<Mutex<f64>>,
    pub selected_auto: Arc<Mutex<Autos>>,
}

impl CompHandler {
    pub fn new() -> Self {
        Self {
            autos: vec![(Autos::None, Auto::default())],
            time: Arc::new(Mutex::new(0.0)),
            selected_auto: Arc::new(Mutex::new(Autos::None)),
        }
    }

    pub fn get_auto(&mut self) -> &mut Auto {
        let mut index = self.autos.len() - 1;
        for i in 0..self.autos.len() {
            if self.autos[i].0 == *self.selected_auto.lock() {
                index = i;
            }
        }
        &mut self.autos[index].1
    }

    pub fn update(&mut self, time_elapsed: Duration) { *self.time.lock() += time_elapsed.as_millis_f64(); }
}
