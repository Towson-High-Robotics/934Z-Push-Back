use std::fmt::Debug;

#[derive(Debug)]
pub(crate) struct SpeedCurve {
    start_speed: f64,
    end_speed: f64,
}

impl SpeedCurve {
    pub const fn new(start: f64, end: f64) -> Self { Self { start_speed: start, end_speed: end } }

    pub fn sample(&self, t: f64) -> f64 { t * (self.end_speed - self.start_speed) + self.start_speed }
}

pub(crate) trait Curve {
    fn sample(&self, t: f64) -> (f64, f64);
    fn sample_derivative(&self, t: f64) -> (f64, f64);
    fn sample_derivative2(&self, t: f64) -> (f64, f64);

    fn sample_heading(&self, t: f64) -> f64;

    fn curve_type(&self) -> u8;

    fn data_str(&self) -> String;
}

impl Debug for dyn Curve {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result { write!(f, "Followable{{{}}}", self.data_str()) }
}

#[derive(Debug, Clone)]
pub(crate) struct LinearInterp {
    pub a: (f64, f64),
    pub b: (f64, f64),
}

impl LinearInterp {
    pub(crate) fn new(start: (f64, f64), end: (f64, f64)) -> Box<Self> { Box::new(Self { a: start, b: end }) }
}

impl Curve for LinearInterp {
    fn sample(&self, t: f64) -> (f64, f64) { (self.a.0 + t * (self.b.0 - self.a.0), self.a.1 + t * (self.b.1 - self.a.1)) }
    fn sample_derivative(&self, _t: f64) -> (f64, f64) { (self.b.0 - self.a.1, self.b.1 - self.a.1) }
    fn sample_derivative2(&self, _t: f64) -> (f64, f64) { (0.0, 0.0) }

    fn sample_heading(&self, _t: f64) -> f64 { (self.b.1 - self.a.1).atan2(self.b.0 - self.a.1) }

    fn curve_type(&self) -> u8 { 0 }

    fn data_str(&self) -> String { format!("a: {:?}, b: {:?}", self.a, self.b) }
}

#[derive(Debug, Clone)]
pub(crate) struct CubicBezier {
    pub a: (f64, f64),
    pub b: (f64, f64),
    pub c: (f64, f64),
    pub d: (f64, f64),
}

impl CubicBezier {
    pub(crate) fn new(start: (f64, f64), c1: (f64, f64), c2: (f64, f64), end: (f64, f64)) -> Box<Self> { Box::new(Self { a: start, b: c1, c: c2, d: end }) }
}

impl Curve for CubicBezier {
    fn sample(&self, t: f64) -> (f64, f64) {
        let t2 = t * t;
        let t3 = t2 * t;
        (
            t3 * (-self.a.0 + 3.0 * self.b.0 - 3.0 * self.c.0 + self.d.0) + t2 * (3.0 * self.a.0 - 6.0 * self.b.0 + 3.0 * self.c.0) + t * (-3.0 * self.a.0 + 3.0 * self.b.0) + self.a.0,
            t3 * (-self.a.1 + 3.0 * self.b.1 - 3.0 * self.c.1 + self.d.1) + t2 * (3.0 * self.a.1 - 6.0 * self.b.1 + 3.0 * self.c.1) + t * (-3.0 * self.a.1 + 3.0 * self.b.1) + self.a.1,
        )
    }

    fn sample_derivative(&self, t: f64) -> (f64, f64) {
        let t2 = t * t;
        (
            t2 * (-3.0 * self.a.0 + 9.0 * self.b.0 - 9.0 * self.c.0 + 3.0 * self.d.0) + t * (6.0 * self.a.0 - 12.0 * self.b.0 + 6.0 * self.c.0) - 3.0 * self.a.0 + 3.0 * self.b.0,
            t2 * (-3.0 * self.a.1 + 9.0 * self.b.1 - 9.0 * self.c.1 + 3.0 * self.d.1) + t * (6.0 * self.a.1 - 12.0 * self.b.1 + 6.0 * self.c.1) - 3.0 * self.a.1 + 3.0 * self.b.1,
        )
    }

    fn sample_derivative2(&self, t: f64) -> (f64, f64) {
        (
            t * (-6.0 * self.a.0 + 18.0 * self.b.0 - 18.0 * self.c.0 + 6.0 * self.d.0) + (6.0 * self.a.0 - 12.0 * self.b.0 + 6.0 * self.c.0),
            t * (-6.0 * self.a.1 + 18.0 * self.b.1 - 18.0 * self.c.1 + 6.0 * self.d.1) + (6.0 * self.a.1 - 12.0 * self.b.1 + 6.0 * self.c.1),
        )
    }

    fn sample_heading(&self, t: f64) -> f64 {
        let t2 = t * t;
        (t2 * (-3.0 * self.a.1 + 9.0 * self.b.1 - 9.0 * self.c.1 + 3.0 * self.d.1) + t * (6.0 * self.a.1 - 12.0 * self.b.1 + 6.0 * self.c.1) - 3.0 * self.a.1 + 3.0 * self.b.1)
            .atan2(t2 * (-3.0 * self.a.0 + 9.0 * self.b.0 - 9.0 * self.c.0 + 3.0 * self.d.0) + t * (6.0 * self.a.0 - 12.0 * self.b.0 + 6.0 * self.c.0) - 3.0 * self.a.0 + 3.0 * self.b.0)
    }

    fn curve_type(&self) -> u8 { 1 }

    fn data_str(&self) -> String { format!("a: {:?}, b: {:?}, c: {:?}, d: {:?}", self.a, self.b, self.c, self.d) }
}

#[derive(Clone, Debug)]
pub(crate) struct CubicPolyBezier {
    pub a: (f64, f64),
    pub b: (f64, f64),
    pub c: (f64, f64),
    pub d: (f64, f64),
}

impl Curve for CubicPolyBezier {
    fn sample(&self, t: f64) -> (f64, f64) {
        let t2 = t * t;
        let t3 = t2 * t;
        (self.a.0 * t3 + self.b.0 * t2 + self.c.0 * t + self.d.0, self.a.1 * t3 + self.b.1 * t2 + self.c.1 * t + self.d.1)
    }

    fn sample_derivative(&self, t: f64) -> (f64, f64) {
        let t2 = t * t;
        (3.0 * self.a.0 * t2 + 2.0 * self.b.0 * t + self.c.0, 3.0 * self.a.1 * t2 + 2.0 * self.b.1 * t + self.c.1)
    }

    fn sample_derivative2(&self, t: f64) -> (f64, f64) { (6.0 * self.a.0 * t + 2.0 * self.b.0, 6.0 * self.a.1 * t + 2.0 * self.b.1) }

    fn sample_heading(&self, t: f64) -> f64 {
        let t2 = t * t;
        (3.0 * self.a.1 * t2 + 2.0 * self.b.1 * t + self.c.1).atan2(3.0 * self.a.0 * t2 + 2.0 * self.b.0 * t + self.c.0)
    }

    fn curve_type(&self) -> u8 { 2 }

    fn data_str(&self) -> String { format!("a: {:?}, b: {:?}, c: {:?}, d: {:?}", self.a, self.b, self.c, self.d) }
}

#[derive(Debug)]
pub(crate) struct PathSegment {
    pub curve: Box<dyn Curve>,
    pub min_speed: SpeedCurve,
    pub max_speed: SpeedCurve,
    pub end_heading: f64,
    pub end_heading_err: f64,
    pub reversed_drive: bool,
    pub timeout: f64,
    pub wait_time: f64,
    pub chained: bool,
}

impl Default for PathSegment {
    fn default() -> Self {
        Self {
            curve: Box::new(LinearInterp { a: (0.0, 0.0), b: (0.0, 0.0) }),
            min_speed: SpeedCurve::new(0.0, 0.0),
            max_speed: SpeedCurve::new(1.0, 1.0),
            end_heading: 0.0,
            end_heading_err: 1.0,
            reversed_drive: false,
            timeout: 5000.0,
            wait_time: 0.0,
            chained: false,
        }
    }
}

#[allow(unused)]
impl PathSegment {
    /// Set the curve that this `PathSegment` represents
    pub fn curve(&mut self, curve: Box<dyn Curve>) -> &mut PathSegment {
        self.curve = curve;
        self
    }

    /// Set a constant minimum speed, valid for motion chains only
    pub fn min_speed(&mut self, speed: f64) -> &mut PathSegment {
        self.min_speed = SpeedCurve::new(speed, speed);
        self
    }

    /// Set a dynamic minimum speed, following a linear interpolation, valid for
    /// motion chains only
    pub fn min_speed_lin(&mut self, start: f64, end: f64) -> &mut PathSegment {
        self.min_speed = SpeedCurve::new(start, end);
        self
    }

    /// Set a constant maximum speed
    pub fn max_speed(&mut self, speed: f64) -> &mut PathSegment {
        self.max_speed = SpeedCurve::new(speed, speed);
        self
    }

    /// Set a dynamic maximum speed, following a linear interpolation
    pub fn max_speed_lin(&mut self, start: f64, end: f64) -> &mut PathSegment {
        self.max_speed = SpeedCurve::new(start, end);
        self
    }

    /// Set the heading target for when the motion finishes
    pub fn heading(&mut self, heading: f64) -> &mut PathSegment {
        self.end_heading = heading;
        self
    }

    /// Set the acceptable error for the heading target, valid for motion chains
    /// only
    pub fn heading_err(&mut self, err: f64) -> &mut PathSegment {
        self.end_heading_err = err;
        self
    }

    /// Reverse the direction this `PathSegment` is followed in
    pub fn reverse(&mut self) -> &mut PathSegment {
        self.reversed_drive = true;
        self
    }

    /// Set a timeout for how long the `PathSegment` is allowed to take to reach
    /// it's ending point
    pub fn timeout(&mut self, timeout: f64) -> &mut PathSegment {
        self.timeout = timeout;
        self
    }

    /// Set a wait time for after the `PathSegment` is finished being followed
    pub fn wait(&mut self, wait: f64) -> &mut PathSegment {
        self.wait_time = wait;
        self
    }

    /// Enable motion chaining on this `PathSegment`
    pub fn chain_motion(&mut self) -> &mut PathSegment {
        self.chained = true;
        self
    }
}
