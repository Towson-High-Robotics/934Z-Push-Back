use alloc::vec::Vec;

#[allow(dead_code)]
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq)]
pub(crate) enum AutoTypes {
    RedLeft,
    RedRight,
    RedAWP,
    BlueLeft,
    BlueRight,
    BlueAWP,
    Skills,
    SkillsDriver,
    #[default]
    None,
}

pub(crate) struct PID {
    last_err: f64,
    sum_err: f64,
    pub kP: f64,
    pub kD: f64,
    pub kI: f64,
}

impl PID {
    fn new(kP: f64, kD: f64, kI: f64) -> Self { Self { last_err: 0.0, sum_err: 0.0, kP, kD, kI } }

    fn update(&mut self, value: f64, target: f64) -> f64 {
        let error: f64 = target - value;
        self.sum_err = error + 0.95 * self.sum_err;
        let prop: f64 = self.kP * error;
        let deriv: f64 = self.kD * (error - self.last_err);
        let int = self.kI * self.sum_err;
        self.last_err = error;
        prop + deriv + int
    }

    fn reset(&mut self) {
        self.last_err = 0.0;
        self.sum_err = 0.0;
    }
}

pub(crate) struct SpeedCurve {
    start_speed: f64,
    control_1: f64,
    end_speed: f64,
}

impl SpeedCurve {
    pub fn new_linear(start: f64, end: f64) -> Self {
        Self {
            start_speed: start,
            control_1: (start + end) / 2.0,
            end_speed: end,
        }
    }

    pub fn new(start_speed: f64, control_1: f64, end_speed: f64) -> Self { Self { start_speed, control_1, end_speed } }

    pub fn sample(&self, t: f64) -> f64 { t * t * (self.end_speed - 2.0 * self.control_1 + self.start_speed) + 2.0 * t * (self.control_1 - self.start_speed) + self.start_speed }
}

pub(crate) struct QuinticHermite {
    p0: (f64, f64),
    p1: (f64, f64),
    v0: (f64, f64),
    v1: (f64, f64),
    a0: (f64, f64),
    a1: (f64, f64),
}

impl QuinticHermite {
    pub fn sample(&self, t: f64) -> (f64, f64) {
        (
            (((((6.0 * self.p1.0 - 3.0 * self.v0.0 - 6.0 * self.p0.0 - 3.0 * self.v1.0 + 0.5 * self.a1.0 - 0.5 * self.a0.0) * t
                + (-15.0 * self.p1.0 + 1.5 * self.a0.0 + 8.0 * self.v0.0 + 15.0 * self.p0.0 + 7.0 * self.v1.0 - self.a1.0))
                * t
                + (10.0 * self.p1.0 - 1.5 * self.a0.0 - 6.0 * self.v0.0 - 10.0 * self.p0.0 - 4.0 * self.v1.0 + 0.5 * self.a1.0))
                * t
                + (0.5 * self.a0.0))
                * t
                + self.v0.0)
                * t
                + self.p0.0,
            (((((6.0 * self.p1.1 - 3.0 * self.v0.1 - 6.0 * self.p0.1 - 3.0 * self.v1.1 + 0.5 * self.a1.1 - 0.5 * self.a0.1) * t
                + (-15.0 * self.p1.1 + 1.5 * self.a0.1 + 8.0 * self.v0.1 + 15.0 * self.p0.1 + 7.0 * self.v1.1 - self.a1.1))
                * t
                + (10.0 * self.p1.1 - 1.5 * self.a0.1 - 6.0 * self.v0.1 - 10.0 * self.p0.1 - 4.0 * self.v1.1 + 0.5 * self.a1.1))
                * t
                + (0.5 * self.a0.1))
                * t
                + self.v0.1)
                * t
                + self.p0.1,
        )
    }

    pub fn sample_derivative(&self, t: f64) -> (f64, f64) {
        (
            (((((6.0 * self.p1.0 - 3.0 * self.v0.0 - 6.0 * self.p0.0 - 3.0 * self.v1.0 + 0.5 * self.a1.0 - 0.5 * self.a0.0) * t
                + (-15.0 * self.p1.0 + 1.5 * self.a0.0 + 8.0 * self.v0.0 + 15.0 * self.p0.0 + 7.0 * self.v1.0 - self.a1.0))
                * t
                + (10.0 * self.p1.0 - 1.5 * self.a0.0 - 6.0 * self.v0.0 - 10.0 * self.p0.0 - 4.0 * self.v1.0 + 0.5 * self.a1.0))
                * t
                + (0.5 * self.a0.0))
                * t
                + self.v0.0)
                * t
                + self.p0.0,
            (((((6.0 * self.p1.1 - 3.0 * self.v0.1 - 6.0 * self.p0.1 - 3.0 * self.v1.1 + 0.5 * self.a1.1 - 0.5 * self.a0.1) * t
                + (-15.0 * self.p1.1 + 1.5 * self.a0.1 + 8.0 * self.v0.1 + 15.0 * self.p0.1 + 7.0 * self.v1.1 - self.a1.1))
                * t
                + (10.0 * self.p1.1 - 1.5 * self.a0.1 - 6.0 * self.v0.1 - 10.0 * self.p0.1 - 4.0 * self.v1.1 + 0.5 * self.a1.1))
                * t
                + (0.5 * self.a0.1))
                * t
                + self.v0.1)
                * t
                + self.p0.1,
        )
    }
}

pub(crate) struct PathSegment {
    curve: QuinticHermite,
    speed: SpeedCurve,
    start_heading: f64,
    end_heading: f64,
    reversed_drive: bool,
}

pub(crate) enum Action {
    ToggleMatchload,
    SpinIntake,
    StopIntake,
    SpinIndexer,
    StopIndexer,
    SetPose(f64, f64, f64),
}

pub(crate) struct Auto {
    spline: Vec<PathSegment>,
    time_checkpoints: Vec<f64>,
    spline_t: f64,
    delay: f64,
    actions: Vec<(Action, f64)>,
    current_action: usize,
}
