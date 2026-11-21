//! `b2Vec2`, `b2Mat22`, `b2XForm` and helpers, transcribed with identical operation order.
use std::ops::{Add, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    pub fn negative(self) -> Self {
        Self::new(-self.x, -self.y)
    }
    pub fn length(self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    pub fn length_squared(self) -> f64 {
        self.x * self.x + self.y * self.y
    }
    /// Returns the previous length; leaves the vector untouched if it is shorter than `f64::MIN_POSITIVE`
    /// (`Number.MIN_VALUE` in JS is 5e-324, i.e. the smallest subnormal).
    pub fn normalize(&mut self) -> f64 {
        let l = (self.x * self.x + self.y * self.y).sqrt();
        if l < 5e-324 {
            return 0.0;
        }
        let inv = 1.0 / l;
        self.x *= inv;
        self.y *= inv;
        l
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}
impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat22 {
    pub col1: Vec2,
    pub col2: Vec2,
}

impl Default for Mat22 {
    fn default() -> Self {
        Self::identity()
    }
}

impl Mat22 {
    pub fn from_angle(a: f64) -> Self {
        let (s, c) = a.sin_cos();
        Self { col1: Vec2::new(c, s), col2: Vec2::new(-s, c) }
    }
    pub fn identity() -> Self {
        Self { col1: Vec2::new(1.0, 0.0), col2: Vec2::new(0.0, 1.0) }
    }
    pub fn set_angle(&mut self, a: f64) {
        *self = Self::from_angle(a);
    }
    pub fn angle(&self) -> f64 {
        self.col1.y.atan2(self.col1.x)
    }
    pub fn invert(&self) -> Option<Mat22> {
        let (a, b, c, d) = (self.col1.x, self.col2.x, self.col1.y, self.col2.y);
        let mut det = a * d - b * c;
        if det == 0.0 {
            return None;
        }
        det = 1.0 / det;
        Some(Mat22 { col1: Vec2::new(det * d, -det * c), col2: Vec2::new(-det * b, det * a) })
    }
    /// Solves `A x = (bx, by)` like `b2Mat22.Solve`.
    pub fn solve(&self, bx: f64, by: f64) -> Vec2 {
        let (a, b, c, d) = (self.col1.x, self.col2.x, self.col1.y, self.col2.y);
        let det = 1.0 / (a * d - b * c);
        Vec2::new(det * (d * bx - b * by), det * (a * by - c * bx))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct XForm {
    pub position: Vec2,
    pub r: Mat22,
}

pub fn dot(a: Vec2, b: Vec2) -> f64 {
    a.x * b.x + a.y * b.y
}
pub fn cross_vv(a: Vec2, b: Vec2) -> f64 {
    a.x * b.y - a.y * b.x
}
/// `b2CrossVF(v, s)` = (s·v.y, -s·v.x)
pub fn cross_vf(v: Vec2, s: f64) -> Vec2 {
    Vec2::new(s * v.y, -s * v.x)
}
/// `b2CrossFV(s, v)` = (-s·v.y, s·v.x)
pub fn cross_fv(s: f64, v: Vec2) -> Vec2 {
    Vec2::new(-s * v.y, s * v.x)
}
pub fn mul_mv(m: &Mat22, v: Vec2) -> Vec2 {
    Vec2::new(m.col1.x * v.x + m.col2.x * v.y, m.col1.y * v.x + m.col2.y * v.y)
}
pub fn mul_tmv(m: &Mat22, v: Vec2) -> Vec2 {
    Vec2::new(dot(v, m.col1), dot(v, m.col2))
}
pub fn mul_x(t: &XForm, v: Vec2) -> Vec2 {
    let mut r = mul_mv(&t.r, v);
    r.x += t.position.x;
    r.y += t.position.y;
    r
}
pub fn mul_xt(t: &XForm, v: Vec2) -> Vec2 {
    let r = v - t.position;
    Vec2::new(r.x * t.r.col1.x + r.y * t.r.col1.y, r.x * t.r.col2.x + r.y * t.r.col2.y)
}
pub fn clamp(a: f64, lo: f64, hi: f64) -> f64 {
    let m = if a < hi { a } else { hi };
    if lo > m {
        lo
    } else {
        m
    }
}
