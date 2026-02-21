use core::f64;
use std::{
    sync::{nonpoison::RwLock, Arc},
    time::{Duration, Instant},
};

use crate::{
    autos::{
        auto::{Action, Auto, Autos},
        path::{CubicPolyBezier, Curve, LinearInterp, PathSegment},
    },
    cubreg::curve_reg,
    util::{dot, mag},
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
            recorded_actions: vec![],
        }
    }

    pub fn get_auto(&mut self) -> &mut Auto { &mut self.autos.iter_mut().find(|a| a.0 == *self.selected_auto.read()).unwrap().1 }

    pub fn update(&mut self, _time_elapsed: Duration) {
        let time = self.start_time.elapsed().as_millis() as f64;
        let auto = *self.selected_auto.read();
        if (auto == Autos::Skills && time > SKILLS_TIME) && (auto != Autos::None && auto != Autos::Skills && time > MATCH_AUTO_TIME) {
            self.is_recording = false;
            self.process_recording();
        }
    }

    fn process_recording(&mut self) {
        let path_len = self.recorded_poses.len();
        if path_len != 0 {
            let mut search_ind = (0_usize, 1_usize);
            let mut curve_out: Vec<PathSegment> = vec![];
            let mut actions_out: Vec<(Action, f64)> = vec![];
            let mut current_action = 0_usize;
            loop {
                let mut working_curve = CubicPolyBezier::default();
                let mut last_heading_delta = 0.0;
                let mut last_pose_delta = (0.0, 0.0);
                loop {
                    let mut heading_delta = (self.recorded_poses[search_ind.1].0.2 - self.recorded_poses[search_ind.1 - 1].0.2)
                        / (self.recorded_poses[search_ind.1].1 - self.recorded_poses[search_ind.1 - 1].1);
                    let pose_delta = ((self.recorded_poses[search_ind.1].0.0 - self.recorded_poses[search_ind.1 - 1].0.0)
                        / (self.recorded_poses[search_ind.1].1 - self.recorded_poses[search_ind.1 - 1].1),
                        (self.recorded_poses[search_ind.1].0.1 - self.recorded_poses[search_ind.1 - 1].0.1)
                        / (self.recorded_poses[search_ind.1].1 - self.recorded_poses[search_ind.1 - 1].1));
                    let pose_delta_2 = if search_ind.1 < 2 { (0.0, 0.0) } else {
                        ((self.recorded_poses[search_ind.1 - 2].0.0 - self.recorded_poses[search_ind.1 - 1].0.0),
                         (self.recorded_poses[search_ind.1 - 2].0.1 - self.recorded_poses[search_ind.1 - 1].0.1))
                    };
                    if dot(pose_delta, pose_delta_2) >= 0.0 { search_ind.1 -= 1; break; }
                    if heading_delta > f64::consts::PI { heading_delta -= f64::consts::TAU; }
                    last_heading_delta = heading_delta; last_pose_delta = pose_delta;
                    if (self.recorded_poses[search_ind.1].1 - self.recorded_poses[search_ind.1 - 1].1) > 0.1 && mag(pose_delta) < 0.1 { search_ind.1 -= 1; break; }
                    if mag(pose_delta) < 0.2 && heading_delta.abs() > 10.0_f64.to_radians() { search_ind.1 -= 1; break; }
                    if heading_delta.abs() > 20.0_f64.to_radians() && search_ind.1 != 1 { search_ind.1 -= 1; break; }
                    let x_vals: Vec<f64> = (search_ind.0..search_ind.1).map(|i| self.recorded_poses[i].0.0).collect();
                    let y_vals: Vec<f64> = (search_ind.0..search_ind.1).map(|i| self.recorded_poses[i].0.1).collect();
                    let t_vals: Vec<f64> = (search_ind.0..search_ind.1).map(|i| self.recorded_poses[i].1 / (self.recorded_poses[search_ind.1].1 - self.recorded_poses[search_ind.0].1)).collect();
                    let curve = curve_reg(
                        x_vals.clone(),
                        y_vals.clone(),
                        t_vals.clone()
                    );
                    let mad = t_vals.iter().enumerate().map(|t| {
                        let curve_sample = curve.sample(*t.1);
                        mag((curve_sample.0 - x_vals[t.0], curve_sample.1 - y_vals[t.0]))
                    }).fold(0.0, |a, i| a + i);
                    if mad > 0.25 { search_ind.1 -= 1; break; }
                    working_curve = curve;
                    search_ind.1 += 1;
                    if search_ind.1 == path_len { search_ind.1 -= 1; break; }
                }
                loop {
                    if self.recorded_actions[current_action].1 < self.recorded_poses[search_ind.1].1 {
                        actions_out.push((self.recorded_actions[current_action].0, self.recorded_actions[current_action].1 / (self.recorded_poses[search_ind.1].1 - self.recorded_poses[search_ind.0].1) + curve_out.len() as f64));
                        current_action += 1;
                    } else { break; }
                }
                curve_out.push(PathSegment {
                    curve: Box::new(working_curve),
                    end_heading: self.recorded_poses[search_ind.1].0.2,
                    end_heading_err: 5.0,
                    reversed_drive: dot(((self.recorded_poses[search_ind.1].0.0 - self.recorded_poses[search_ind.0].0.0),
                                             (self.recorded_poses[search_ind.1].0.1 - self.recorded_poses[search_ind.0].0.1)),
                                        (self.recorded_poses[search_ind.1].0.2.cos(), self.recorded_poses[search_ind.1].0.2.sin())) <= 0.0,
                    timeout: self.recorded_poses[search_ind.1].1 - self.recorded_poses[search_ind.0].1,
                    wait_time: if search_ind.1 == path_len - 1 { 0.0 } else { 1000.0 * (self.recorded_poses[search_ind.1 + 1].1 - self.recorded_poses[search_ind.1].1) },
                    chained: true,
                    force_stanley: true,
                    ..Default::default()
                });
                if mag(last_pose_delta) < 0.2 && last_heading_delta.abs() > 10.0_f64.to_radians() {
                    curve_out.push(PathSegment {
                        curve: LinearInterp::new(
                            (self.recorded_poses[search_ind.1 + 1].0.0, self.recorded_poses[search_ind.1 + 1].0.1),
                            (self.recorded_poses[search_ind.1 + 1].0.0, self.recorded_poses[search_ind.1 + 1].0.1)),
                        end_heading: self.recorded_poses[search_ind.1 + 1].0.2,
                        end_heading_err: 5.0,
                        chained: true,
                        ..Default::default()
                    });
                }
                search_ind.0 = search_ind.1;
                search_ind.1 = search_ind.0 + 1;
                if search_ind.0 == path_len - 1 {
                    let mut export = Auto::new();
                    export.add_curves(curve_out);
                    export.add_actions(actions_out);
                    break;
                }
            }
        }
    }
}
