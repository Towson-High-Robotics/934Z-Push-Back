#[allow(dead_code)]
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Autos {
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
