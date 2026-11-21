//! Joints (force-based Box2D 2.0.1 formulation, `dt`/`inv_dt` scaled): revolute and prismatic,
//! transcribed from box2d-pretty.js (~6218-6573 and ~5699-6217).
use crate::math::*;
use crate::settings::*;
use crate::world::{Body, BodyId, StepParams};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitState {
    Inactive,
    AtLower,
    AtUpper,
    Equal,
}

#[derive(Debug, Clone)]
pub struct RevoluteJointDef {
    pub body1: BodyId,
    pub body2: BodyId,
    pub local_anchor1: Vec2,
    pub local_anchor2: Vec2,
    pub reference_angle: f64,
    pub enable_limit: bool,
    pub lower_angle: f64,
    pub upper_angle: f64,
    pub enable_motor: bool,
    pub motor_speed: f64,
    pub max_motor_torque: f64,
    pub collide_connected: bool,
    pub user_data: u64,
}

impl RevoluteJointDef {
    /// `b2RevoluteJointDef.Initialize(body1, body2, anchor)` (anchor in world space).
    pub fn initialize(b1: &Body, id1: BodyId, b2: &Body, id2: BodyId, anchor: Vec2) -> Self {
        Self {
            body1: id1,
            body2: id2,
            local_anchor1: b1.local_point(anchor),
            local_anchor2: b2.local_point(anchor),
            reference_angle: b2.sweep.a - b1.sweep.a,
            enable_limit: false,
            lower_angle: 0.0,
            upper_angle: 0.0,
            enable_motor: false,
            motor_speed: 0.0,
            max_motor_torque: 0.0,
            collide_connected: false,
            user_data: 0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PrismaticJointDef {
    pub body1: BodyId,
    pub body2: BodyId,
    pub local_anchor1: Vec2,
    pub local_anchor2: Vec2,
    pub local_axis1: Vec2,
    pub reference_angle: f64,
    pub enable_limit: bool,
    pub lower_translation: f64,
    pub upper_translation: f64,
    pub enable_motor: bool,
    pub max_motor_force: f64,
    pub motor_speed: f64,
    pub collide_connected: bool,
    pub user_data: u64,
}

impl PrismaticJointDef {
    /// `Initialize(body1, body2, anchor, axis)` (both in world space).
    pub fn initialize(b1: &Body, id1: BodyId, b2: &Body, id2: BodyId, anchor: Vec2, axis: Vec2) -> Self {
        Self {
            body1: id1,
            body2: id2,
            local_anchor1: b1.local_point(anchor),
            local_anchor2: b2.local_point(anchor),
            local_axis1: mul_tmv(&b1.xf.r, axis),
            reference_angle: b2.sweep.a - b1.sweep.a,
            enable_limit: false,
            lower_translation: 0.0,
            upper_translation: 0.0,
            enable_motor: false,
            max_motor_force: 0.0,
            motor_speed: 0.0,
            collide_connected: false,
            user_data: 0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Revolute {
    pub local_anchor1: Vec2,
    pub local_anchor2: Vec2,
    pub reference_angle: f64,
    pub pivot_force: Vec2,
    pivot_mass: Mat22,
    motor_mass: f64,
    pub motor_force: f64,
    limit_force: f64,
    limit_position_impulse: f64,
    limit_state: LimitState,
    pub lower_angle: f64,
    pub upper_angle: f64,
    pub max_motor_torque: f64,
    pub motor_speed: f64,
    pub enable_limit: bool,
    pub enable_motor: bool,
}

#[derive(Debug, Clone, Copy, Default)]
struct Jacobian {
    linear1: Vec2,
    angular1: f64,
    linear2: Vec2,
    angular2: f64,
}

impl Jacobian {
    fn compute(&self, v1: Vec2, w1: f64, v2: Vec2, w2: f64) -> f64 {
        self.linear1.x * v1.x + self.linear1.y * v1.y + self.angular1 * w1 + (self.linear2.x * v2.x + self.linear2.y * v2.y) + self.angular2 * w2
    }
}

#[derive(Debug, Clone)]
pub struct Prismatic {
    pub local_anchor1: Vec2,
    pub local_anchor2: Vec2,
    local_x_axis1: Vec2,
    local_y_axis1: Vec2,
    ref_angle: f64,
    linear_jacobian: Jacobian,
    linear_mass: f64,
    force: f64,
    angular_mass: f64,
    torque: f64,
    motor_jacobian: Jacobian,
    motor_mass: f64,
    pub motor_force: f64,
    limit_force: f64,
    limit_position_impulse: f64,
    limit_state: LimitState,
    pub lower_translation: f64,
    pub upper_translation: f64,
    pub max_motor_force: f64,
    pub motor_speed: f64,
    pub enable_limit: bool,
    pub enable_motor: bool,
}

#[derive(Debug, Clone)]
pub enum JointKind {
    Revolute(Revolute),
    Prismatic(Prismatic),
    Distance(Distance),
    Gear(Gear),
    Rope(Rope),
}

#[derive(Debug, Clone)]
pub struct Joint {
    pub body1: BodyId,
    pub body2: BodyId,
    pub collide_connected: bool,
    pub island_flag: bool,
    pub user_data: u64,
    pub kind: JointKind,
}

fn rot(r: &Mat22, x: f64, y: f64) -> (f64, f64) {
    (r.col1.x * x + r.col2.x * y, r.col1.y * x + r.col2.y * y)
}

/// Splits two distinct bodies out of the slice.
fn two(bodies: &mut [Body], a: usize, b: usize) -> (&mut Body, &mut Body) {
    assert_ne!(a, b);
    if a < b {
        let (l, r) = bodies.split_at_mut(b);
        (&mut l[a], &mut r[0])
    } else {
        let (l, r) = bodies.split_at_mut(a);
        (&mut r[0], &mut l[b])
    }
}

impl Joint {
    pub fn new_revolute(d: &RevoluteJointDef) -> Self {
        Joint {
            body1: d.body1,
            body2: d.body2,
            collide_connected: d.collide_connected,
            island_flag: false,
            user_data: d.user_data,
            kind: JointKind::Revolute(Revolute {
                local_anchor1: d.local_anchor1,
                local_anchor2: d.local_anchor2,
                reference_angle: d.reference_angle,
                pivot_force: Vec2::default(),
                pivot_mass: Mat22::identity(),
                motor_mass: 0.0,
                motor_force: 0.0,
                limit_force: 0.0,
                limit_position_impulse: 0.0,
                limit_state: LimitState::Inactive,
                lower_angle: d.lower_angle,
                upper_angle: d.upper_angle,
                max_motor_torque: d.max_motor_torque,
                motor_speed: d.motor_speed,
                enable_limit: d.enable_limit,
                enable_motor: d.enable_motor,
            }),
        }
    }

    pub fn new_prismatic(d: &PrismaticJointDef) -> Self {
        Joint {
            body1: d.body1,
            body2: d.body2,
            collide_connected: d.collide_connected,
            island_flag: false,
            user_data: d.user_data,
            kind: JointKind::Prismatic(Prismatic {
                local_anchor1: d.local_anchor1,
                local_anchor2: d.local_anchor2,
                local_x_axis1: d.local_axis1,
                local_y_axis1: Vec2::new(-d.local_axis1.y, d.local_axis1.x),
                ref_angle: d.reference_angle,
                linear_jacobian: Jacobian::default(),
                linear_mass: 0.0,
                force: 0.0,
                angular_mass: 0.0,
                torque: 0.0,
                motor_jacobian: Jacobian::default(),
                motor_mass: 0.0,
                motor_force: 0.0,
                limit_force: 0.0,
                limit_position_impulse: 0.0,
                limit_state: LimitState::Inactive,
                lower_translation: d.lower_translation,
                upper_translation: d.upper_translation,
                max_motor_force: d.max_motor_force,
                motor_speed: d.motor_speed,
                enable_limit: d.enable_limit,
                enable_motor: d.enable_motor,
            }),
        }
    }

    pub fn init_velocity_constraints(&mut self, bodies: &mut [Body], step: &StepParams) {
        let (i1, i2) = (self.body1, self.body2);
        let (b1, b2) = two(bodies, i1, i2);
        match &mut self.kind {
            JointKind::Revolute(j) => j.init_velocity(b1, b2, step),
            JointKind::Prismatic(j) => j.init_velocity(b1, b2, step),
            JointKind::Distance(j) => j.init_velocity(b1, b2, step),
            JointKind::Gear(j) => j.init_velocity(b1, b2, step),
            JointKind::Rope(j) => j.init_velocity(b1, b2, step),
        }
    }

    pub fn solve_velocity_constraints(&mut self, bodies: &mut [Body], step: &StepParams) {
        let (i1, i2) = (self.body1, self.body2);
        let (b1, b2) = two(bodies, i1, i2);
        match &mut self.kind {
            JointKind::Revolute(j) => j.solve_velocity(b1, b2, step),
            JointKind::Prismatic(j) => j.solve_velocity(b1, b2, step),
            JointKind::Distance(j) => j.solve_velocity(b1, b2),
            JointKind::Gear(j) => j.solve_velocity(b1, b2, step),
            JointKind::Rope(j) => j.solve_velocity(b1, b2, step),
        }
    }

    pub fn solve_position_constraints(&mut self, bodies: &mut [Body]) -> bool {
        let (i1, i2) = (self.body1, self.body2);
        if let JointKind::Gear(g) = &mut self.kind {
            return g.solve_position(bodies, i1, i2);
        }
        let (b1, b2) = two(bodies, i1, i2);
        match &mut self.kind {
            JointKind::Revolute(j) => j.solve_position(b1, b2),
            JointKind::Prismatic(j) => j.solve_position(b1, b2),
            JointKind::Distance(j) => j.solve_position(b1, b2),
            JointKind::Rope(j) => j.solve_position(b1, b2),
            JointKind::Gear(_) => unreachable!(),
        }
    }
}

impl Revolute {
    fn init_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        let (mut a, mut o) = rot(&b1.xf.r, self.local_anchor1.x - b1.sweep.local_center.x, self.local_anchor1.y - b1.sweep.local_center.y);
        let (l, d) = rot(&b2.xf.r, self.local_anchor2.x - b2.sweep.local_center.x, self.local_anchor2.y - b2.sweep.local_center.y);
        let (u, p, m, f) = (b1.inv_mass, b2.inv_mass, b1.inv_i, b2.inv_i);
        let k1 = Mat22 { col1: Vec2::new(u + p, 0.0), col2: Vec2::new(0.0, u + p) };
        let k2 = Mat22 { col1: Vec2::new(m * o * o, -m * a * o), col2: Vec2::new(-m * a * o, m * a * a) };
        let k3 = Mat22 { col1: Vec2::new(f * d * d, -f * l * d), col2: Vec2::new(-f * l * d, f * l * l) };
        let mut k = k1;
        k.col1.x += k2.col1.x;
        k.col1.y += k2.col1.y;
        k.col2.x += k2.col2.x;
        k.col2.y += k2.col2.y;
        k.col1.x += k3.col1.x;
        k.col1.y += k3.col1.y;
        k.col2.x += k3.col2.x;
        k.col2.y += k3.col2.y;
        if let Some(inv) = k.invert() {
            self.pivot_mass = inv;
        }
        self.motor_mass = 1.0 / (m + f);
        if !self.enable_motor {
            self.motor_force = 0.0;
        }
        if self.enable_limit {
            let ang = b2.sweep.a - b1.sweep.a - self.reference_angle;
            if (self.upper_angle - self.lower_angle).abs() < 2.0 * ANGULAR_SLOP {
                self.limit_state = LimitState::Equal;
            } else if ang <= self.lower_angle {
                if self.limit_state != LimitState::AtLower {
                    self.limit_force = 0.0;
                }
                self.limit_state = LimitState::AtLower;
            } else if ang >= self.upper_angle {
                if self.limit_state != LimitState::AtUpper {
                    self.limit_force = 0.0;
                }
                self.limit_state = LimitState::AtUpper;
            } else {
                self.limit_state = LimitState::Inactive;
                self.limit_force = 0.0;
            }
        } else {
            self.limit_force = 0.0;
        }
        if step.warm_starting {
            b1.linear_velocity.x -= step.dt * u * self.pivot_force.x;
            b1.linear_velocity.y -= step.dt * u * self.pivot_force.y;
            b1.angular_velocity -= step.dt * m * (a * self.pivot_force.y - o * self.pivot_force.x + self.motor_force + self.limit_force);
            b2.linear_velocity.x += step.dt * p * self.pivot_force.x;
            b2.linear_velocity.y += step.dt * p * self.pivot_force.y;
            b2.angular_velocity += step.dt * f * (l * self.pivot_force.y - d * self.pivot_force.x + self.motor_force + self.limit_force);
        } else {
            self.pivot_force = Vec2::default();
            self.motor_force = 0.0;
            self.limit_force = 0.0;
        }
        self.limit_position_impulse = 0.0;
        let _ = (&mut a, &mut o);
    }

    fn solve_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        let (a, o) = rot(&b1.xf.r, self.local_anchor1.x - b1.sweep.local_center.x, self.local_anchor1.y - b1.sweep.local_center.y);
        let (d, c) = rot(&b2.xf.r, self.local_anchor2.x - b2.sweep.local_center.x, self.local_anchor2.y - b2.sweep.local_center.y);
        let u = b2.linear_velocity.x + -b2.angular_velocity * c - b1.linear_velocity.x - -b1.angular_velocity * o;
        let p = b2.linear_velocity.y + b2.angular_velocity * d - b1.linear_velocity.y - b1.angular_velocity * a;
        let m = -step.inv_dt * (self.pivot_mass.col1.x * u + self.pivot_mass.col2.x * p);
        let f = -step.inv_dt * (self.pivot_mass.col1.y * u + self.pivot_mass.col2.y * p);
        self.pivot_force.x += m;
        self.pivot_force.y += f;
        let ix = step.dt * m;
        let iy = step.dt * f;
        b1.linear_velocity.x -= b1.inv_mass * ix;
        b1.linear_velocity.y -= b1.inv_mass * iy;
        b1.angular_velocity -= b1.inv_i * (a * iy - o * ix);
        b2.linear_velocity.x += b2.inv_mass * ix;
        b2.linear_velocity.y += b2.inv_mass * iy;
        b2.angular_velocity += b2.inv_i * (d * iy - c * ix);
        if self.enable_motor && self.limit_state != LimitState::Equal {
            let y = b2.angular_velocity - b1.angular_velocity - self.motor_speed;
            let mut g = -step.inv_dt * self.motor_mass * y;
            let w = self.motor_force;
            self.motor_force = clamp(self.motor_force + g, -self.max_motor_torque, self.max_motor_torque);
            g = self.motor_force - w;
            b1.angular_velocity -= b1.inv_i * step.dt * g;
            b2.angular_velocity += b2.inv_i * step.dt * g;
        }
        if self.enable_limit && self.limit_state != LimitState::Inactive {
            let x = b2.angular_velocity - b1.angular_velocity;
            let mut b = -step.inv_dt * self.motor_mass * x;
            match self.limit_state {
                LimitState::Equal => self.limit_force += b,
                LimitState::AtLower => {
                    let old = self.limit_force;
                    self.limit_force = (self.limit_force + b).max(0.0);
                    b = self.limit_force - old;
                }
                LimitState::AtUpper => {
                    let old = self.limit_force;
                    self.limit_force = (self.limit_force + b).min(0.0);
                    b = self.limit_force - old;
                }
                LimitState::Inactive => {}
            }
            b1.angular_velocity -= b1.inv_i * step.dt * b;
            b2.angular_velocity += b2.inv_i * step.dt * b;
        }
    }

    fn solve_position(&mut self, b1: &mut Body, b2: &mut Body) -> bool {
        let (o, l) = rot(&b1.xf.r, self.local_anchor1.x - b1.sweep.local_center.x, self.local_anchor1.y - b1.sweep.local_center.y);
        let (u, p) = rot(&b2.xf.r, self.local_anchor2.x - b2.sweep.local_center.x, self.local_anchor2.y - b2.sweep.local_center.y);
        let m = b1.sweep.c.x + o;
        let f = b1.sweep.c.y + l;
        let ex = b2.sweep.c.x + u - m;
        let ey = b2.sweep.c.y + p - f;
        let r = (ex * ex + ey * ey).sqrt();
        let (y, g, w, x) = (b1.inv_mass, b2.inv_mass, b1.inv_i, b2.inv_i);
        let k1 = Mat22 { col1: Vec2::new(y + g, 0.0), col2: Vec2::new(0.0, y + g) };
        let k2 = Mat22 { col1: Vec2::new(w * l * l, -w * o * l), col2: Vec2::new(-w * o * l, w * o * o) };
        let k3 = Mat22 { col1: Vec2::new(x * p * p, -x * u * p), col2: Vec2::new(-x * u * p, x * u * u) };
        let mut k = k1;
        k.col1.x += k2.col1.x;
        k.col1.y += k2.col1.y;
        k.col2.x += k2.col2.x;
        k.col2.y += k2.col2.y;
        k.col1.x += k3.col1.x;
        k.col1.y += k3.col1.y;
        k.col2.x += k3.col2.x;
        k.col2.y += k3.col2.y;
        let imp = k.solve(-ex, -ey);
        let (bx, by) = (imp.x, imp.y);
        b1.sweep.c.x -= b1.inv_mass * bx;
        b1.sweep.c.y -= b1.inv_mass * by;
        b1.sweep.a -= b1.inv_i * (o * by - l * bx);
        b2.sweep.c.x += b2.inv_mass * bx;
        b2.sweep.c.y += b2.inv_mass * by;
        b2.sweep.a += b2.inv_i * (u * by - p * bx);
        b1.synchronize_transform();
        b2.synchronize_transform();
        let mut a_err = 0.0f64;
        if self.enable_limit && self.limit_state != LimitState::Inactive {
            let c = b2.sweep.a - b1.sweep.a - self.reference_angle;
            let mut s = 0.0;
            match self.limit_state {
                LimitState::Equal => {
                    let n = clamp(c, -MAX_ANGULAR_CORRECTION, MAX_ANGULAR_CORRECTION);
                    s = -self.motor_mass * n;
                    a_err = n.abs();
                }
                LimitState::AtLower => {
                    let mut n = c - self.lower_angle;
                    a_err = (0.0f64).max(-n);
                    n = clamp(n + ANGULAR_SLOP, -MAX_ANGULAR_CORRECTION, 0.0);
                    s = -self.motor_mass * n;
                    let e = self.limit_position_impulse;
                    self.limit_position_impulse = (self.limit_position_impulse + s).max(0.0);
                    s = self.limit_position_impulse - e;
                }
                LimitState::AtUpper => {
                    let mut n = c - self.upper_angle;
                    a_err = (0.0f64).max(n);
                    n = clamp(n - ANGULAR_SLOP, 0.0, MAX_ANGULAR_CORRECTION);
                    s = -self.motor_mass * n;
                    let e = self.limit_position_impulse;
                    self.limit_position_impulse = (self.limit_position_impulse + s).min(0.0);
                    s = self.limit_position_impulse - e;
                }
                LimitState::Inactive => {}
            }
            b1.sweep.a -= b1.inv_i * s;
            b2.sweep.a += b2.inv_i * s;
            b1.synchronize_transform();
            b2.synchronize_transform();
        }
        r <= LINEAR_SLOP && a_err <= ANGULAR_SLOP
    }
}

impl Prismatic {
    pub fn local_x_axis1(&self) -> Vec2 {
        self.local_x_axis1
    }

    fn init_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        let (a, o) = rot(&b1.xf.r, self.local_anchor1.x - b1.sweep.local_center.x, self.local_anchor1.y - b1.sweep.local_center.y);
        let (l, d) = rot(&b2.xf.r, self.local_anchor2.x - b2.sweep.local_center.x, self.local_anchor2.y - b2.sweep.local_center.y);
        let (u, p, m, f) = (b1.inv_mass, b2.inv_mass, b1.inv_i, b2.inv_i);
        let r = &b1.xf.r;
        let yx = r.col1.x * self.local_y_axis1.x + r.col2.x * self.local_y_axis1.y;
        let yy = r.col1.y * self.local_y_axis1.x + r.col2.y * self.local_y_axis1.y;
        let dx = b2.sweep.c.x + l - b1.sweep.c.x;
        let dy = b2.sweep.c.y + d - b1.sweep.c.y;
        let lj = &mut self.linear_jacobian;
        lj.linear1 = Vec2::new(-yx, -yy);
        lj.linear2 = Vec2::new(yx, yy);
        lj.angular1 = -(dx * yy - dy * yx);
        lj.angular2 = l * yy - d * yx;
        self.linear_mass = u + m * lj.angular1 * lj.angular1 + p + f * lj.angular2 * lj.angular2;
        self.linear_mass = 1.0 / self.linear_mass;
        self.angular_mass = m + f;
        if self.angular_mass > 5e-324 {
            self.angular_mass = 1.0 / self.angular_mass;
        }
        if self.enable_limit || self.enable_motor {
            let r = &b1.xf.r;
            let w = r.col1.x * self.local_x_axis1.x + r.col2.x * self.local_x_axis1.y;
            let x = r.col1.y * self.local_x_axis1.x + r.col2.y * self.local_x_axis1.y;
            let mj = &mut self.motor_jacobian;
            mj.linear1 = Vec2::new(-w, -x);
            mj.linear2 = Vec2::new(w, x);
            mj.angular1 = -(dx * x - dy * w);
            mj.angular2 = l * x - d * w;
            self.motor_mass = u + m * mj.angular1 * mj.angular1 + p + f * mj.angular2 * mj.angular2;
            self.motor_mass = 1.0 / self.motor_mass;
            if self.enable_limit {
                let t = w * (dx - a) + x * (dy - o);
                if (self.upper_translation - self.lower_translation).abs() < 2.0 * LINEAR_SLOP {
                    self.limit_state = LimitState::Equal;
                } else if t <= self.lower_translation {
                    if self.limit_state != LimitState::AtLower {
                        self.limit_force = 0.0;
                    }
                    self.limit_state = LimitState::AtLower;
                } else if t >= self.upper_translation {
                    if self.limit_state != LimitState::AtUpper {
                        self.limit_force = 0.0;
                    }
                    self.limit_state = LimitState::AtUpper;
                } else {
                    self.limit_state = LimitState::Inactive;
                    self.limit_force = 0.0;
                }
            }
        }
        if !self.enable_motor {
            self.motor_force = 0.0;
        }
        if !self.enable_limit {
            self.limit_force = 0.0;
        }
        if step.warm_starting {
            let (lj, mj) = (&self.linear_jacobian, &self.motor_jacobian);
            let ml = self.motor_force + self.limit_force;
            let t = step.dt * (self.force * lj.linear1.x + ml * mj.linear1.x);
            let a_ = step.dt * (self.force * lj.linear1.y + ml * mj.linear1.y);
            let c = step.dt * (self.force * lj.linear2.x + ml * mj.linear2.x);
            let s = step.dt * (self.force * lj.linear2.y + ml * mj.linear2.y);
            let e = step.dt * (self.force * lj.angular1 - self.torque + ml * mj.angular1);
            let o_ = step.dt * (self.force * lj.angular2 + self.torque + ml * mj.angular2);
            b1.linear_velocity.x += u * t;
            b1.linear_velocity.y += u * a_;
            b1.angular_velocity += m * e;
            b2.linear_velocity.x += p * c;
            b2.linear_velocity.y += p * s;
            b2.angular_velocity += f * o_;
        } else {
            self.force = 0.0;
            self.torque = 0.0;
            self.limit_force = 0.0;
            self.motor_force = 0.0;
        }
        self.limit_position_impulse = 0.0;
    }

    fn apply(b1: &mut Body, b2: &mut Body, j: &Jacobian, u: f64) {
        b1.linear_velocity.x += b1.inv_mass * u * j.linear1.x;
        b1.linear_velocity.y += b1.inv_mass * u * j.linear1.y;
        b1.angular_velocity += b1.inv_i * u * j.angular1;
        b2.linear_velocity.x += b2.inv_mass * u * j.linear2.x;
        b2.linear_velocity.y += b2.inv_mass * u * j.linear2.y;
        b2.angular_velocity += b2.inv_i * u * j.angular2;
    }

    fn solve_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        let d = self.linear_jacobian.compute(b1.linear_velocity, b1.angular_velocity, b2.linear_velocity, b2.angular_velocity);
        let c = -step.inv_dt * self.linear_mass * d;
        self.force += c;
        let u = step.dt * c;
        let lj = self.linear_jacobian;
        Self::apply(b1, b2, &lj, u);
        let p = b2.angular_velocity - b1.angular_velocity;
        let m = -step.inv_dt * self.angular_mass * p;
        self.torque += m;
        let f = step.dt * m;
        b1.angular_velocity -= b1.inv_i * f;
        b2.angular_velocity += b2.inv_i * f;
        if self.enable_motor && self.limit_state != LimitState::Equal {
            let mj = self.motor_jacobian;
            let e = mj.compute(b1.linear_velocity, b1.angular_velocity, b2.linear_velocity, b2.angular_velocity) - self.motor_speed;
            let mut v = -step.inv_dt * self.motor_mass * e;
            let old = self.motor_force;
            self.motor_force = clamp(self.motor_force + v, -self.max_motor_force, self.max_motor_force);
            v = self.motor_force - old;
            Self::apply(b1, b2, &mj, step.dt * v);
        }
        if self.enable_limit && self.limit_state != LimitState::Inactive {
            let mj = self.motor_jacobian;
            let g = mj.compute(b1.linear_velocity, b1.angular_velocity, b2.linear_velocity, b2.angular_velocity);
            let mut w = -step.inv_dt * self.motor_mass * g;
            match self.limit_state {
                LimitState::Equal => self.limit_force += w,
                LimitState::AtLower => {
                    let old = self.limit_force;
                    self.limit_force = (self.limit_force + w).max(0.0);
                    w = self.limit_force - old;
                }
                LimitState::AtUpper => {
                    let old = self.limit_force;
                    self.limit_force = (self.limit_force + w).min(0.0);
                    w = self.limit_force - old;
                }
                LimitState::Inactive => {}
            }
            Self::apply(b1, b2, &mj, step.dt * w);
        }
    }

    fn solve_position(&mut self, b1: &mut Body, b2: &mut Body) -> bool {
        let (o_, l_, d_, u_) = (b1.inv_mass, b2.inv_mass, b1.inv_i, b2.inv_i);
        let (p, m) = rot(&b1.xf.r, self.local_anchor1.x - b1.sweep.local_center.x, self.local_anchor1.y - b1.sweep.local_center.y);
        let (f, _y) = rot(&b2.xf.r, self.local_anchor2.x - b2.sweep.local_center.x, self.local_anchor2.y - b2.sweep.local_center.y);
        let v = b1.sweep.c.x + p;
        let y = b1.sweep.c.y + m;
        let g = b2.sweep.c.x + f;
        let w = b2.sweep.c.y + _y;
        let x = g - v;
        let b = w - y;
        let r = &b1.xf.r;
        let mut t = (r.col1.x * self.local_y_axis1.x + r.col2.x * self.local_y_axis1.y) * x + (r.col1.y * self.local_y_axis1.x + r.col2.y * self.local_y_axis1.y) * b;
        t = clamp(t, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION);
        let a = -self.linear_mass * t;
        let lj = self.linear_jacobian;
        b1.sweep.c.x += o_ * a * lj.linear1.x;
        b1.sweep.c.y += o_ * a * lj.linear1.y;
        b1.sweep.a += d_ * a * lj.angular1;
        b2.sweep.c.x += l_ * a * lj.linear2.x;
        b2.sweep.c.y += l_ * a * lj.linear2.y;
        b2.sweep.a += u_ * a * lj.angular2;
        let mut c_err = t.abs();
        let mut s = b2.sweep.a - b1.sweep.a - self.ref_angle;
        s = clamp(s, -MAX_ANGULAR_CORRECTION, MAX_ANGULAR_CORRECTION);
        let e = -self.angular_mass * s;
        b1.sweep.a -= b1.inv_i * e;
        b2.sweep.a += b2.inv_i * e;
        b1.synchronize_transform();
        b2.synchronize_transform();
        let o_err = s.abs();
        if self.enable_limit && self.limit_state != LimitState::Inactive {
            let (p, m) = rot(&b1.xf.r, self.local_anchor1.x - b1.sweep.local_center.x, self.local_anchor1.y - b1.sweep.local_center.y);
            let (f, yy) = rot(&b2.xf.r, self.local_anchor2.x - b2.sweep.local_center.x, self.local_anchor2.y - b2.sweep.local_center.y);
            let v = b1.sweep.c.x + p;
            let y = b1.sweep.c.y + m;
            let x = (b2.sweep.c.x + f) - v;
            let b = (b2.sweep.c.y + yy) - y;
            let r = &b1.xf.r;
            let k = (r.col1.x * self.local_x_axis1.x + r.col2.x * self.local_x_axis1.y) * x + (r.col1.y * self.local_x_axis1.x + r.col2.y * self.local_x_axis1.y) * b;
            let mut pp = 0.0;
            match self.limit_state {
                LimitState::Equal => {
                    let e = clamp(k, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION);
                    pp = -self.motor_mass * e;
                    c_err = c_err.max(s.abs());
                }
                LimitState::AtLower => {
                    let mut e = k - self.lower_translation;
                    c_err = c_err.max(-e);
                    e = clamp(e + LINEAR_SLOP, -MAX_LINEAR_CORRECTION, 0.0);
                    pp = -self.motor_mass * e;
                    let n = self.limit_position_impulse;
                    self.limit_position_impulse = (self.limit_position_impulse + pp).max(0.0);
                    pp = self.limit_position_impulse - n;
                }
                LimitState::AtUpper => {
                    let mut e = k - self.upper_translation;
                    c_err = c_err.max(e);
                    e = clamp(e - LINEAR_SLOP, 0.0, MAX_LINEAR_CORRECTION);
                    pp = -self.motor_mass * e;
                    let n = self.limit_position_impulse;
                    self.limit_position_impulse = (self.limit_position_impulse + pp).min(0.0);
                    pp = self.limit_position_impulse - n;
                }
                LimitState::Inactive => {}
            }
            let mj = self.motor_jacobian;
            b1.sweep.c.x += o_ * pp * mj.linear1.x;
            b1.sweep.c.y += o_ * pp * mj.linear1.y;
            b1.sweep.a += d_ * pp * mj.angular1;
            b2.sweep.c.x += l_ * pp * mj.linear2.x;
            b2.sweep.c.y += l_ * pp * mj.linear2.y;
            b2.sweep.a += u_ * pp * mj.angular2;
            b1.synchronize_transform();
            b2.synchronize_transform();
        }
        c_err <= LINEAR_SLOP && o_err <= ANGULAR_SLOP
    }
}

/// `b2DistanceJointDef`.
#[derive(Debug, Clone)]
pub struct DistanceJointDef {
    pub body1: BodyId,
    pub body2: BodyId,
    pub local_anchor1: Vec2,
    pub local_anchor2: Vec2,
    pub length: f64,
    pub frequency_hz: f64,
    pub damping_ratio: f64,
    pub collide_connected: bool,
    pub user_data: u64,
}

impl DistanceJointDef {
    /// `Initialize(body1, body2, anchor1, anchor2)` (world-space anchors; the length is their distance).
    pub fn initialize(b1: &Body, id1: BodyId, b2: &Body, id2: BodyId, anchor1: Vec2, anchor2: Vec2) -> Self {
        let (dx, dy) = (anchor2.x - anchor1.x, anchor2.y - anchor1.y);
        Self {
            body1: id1,
            body2: id2,
            local_anchor1: b1.local_point(anchor1),
            local_anchor2: b2.local_point(anchor2),
            length: (dx * dx + dy * dy).sqrt(),
            frequency_hz: 0.0,
            damping_ratio: 0.0,
            collide_connected: false,
            user_data: 0,
        }
    }
}

/// `b2DistanceJoint` (box2d-pretty.js 6698): keeps two anchors a fixed distance apart, optionally as a
/// soft spring (`frequency_hz > 0`).
#[derive(Debug, Clone)]
pub struct Distance {
    pub local_anchor1: Vec2,
    pub local_anchor2: Vec2,
    u: Vec2,
    pub length: f64,
    pub frequency_hz: f64,
    pub damping_ratio: f64,
    impulse: f64,
    mass: f64,
    gamma: f64,
    bias: f64,
}

impl Joint {
    pub fn new_distance(d: &DistanceJointDef) -> Self {
        Joint {
            body1: d.body1,
            body2: d.body2,
            collide_connected: d.collide_connected,
            island_flag: false,
            user_data: d.user_data,
            kind: JointKind::Distance(Distance {
                local_anchor1: d.local_anchor1,
                local_anchor2: d.local_anchor2,
                u: Vec2::default(),
                length: d.length,
                frequency_hz: d.frequency_hz,
                damping_ratio: d.damping_ratio,
                impulse: 0.0,
                mass: 0.0,
                gamma: 0.0,
                bias: 0.0,
            }),
        }
    }
}

impl Distance {
    pub fn impulse(&self) -> f64 {
        self.impulse
    }
    fn anchors(&self, b1: &Body, b2: &Body) -> ((f64, f64), (f64, f64)) {
        (
            rot(&b1.xf.r, self.local_anchor1.x - b1.sweep.local_center.x, self.local_anchor1.y - b1.sweep.local_center.y),
            rot(&b2.xf.r, self.local_anchor2.x - b2.sweep.local_center.x, self.local_anchor2.y - b2.sweep.local_center.y),
        )
    }

    fn init_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        let ((s, a), (o, h)) = self.anchors(b1, b2);
        self.u.x = b2.sweep.c.x + o - b1.sweep.c.x - s;
        self.u.y = b2.sweep.c.y + h - b1.sweep.c.y - a;
        let l = (self.u.x * self.u.x + self.u.y * self.u.y).sqrt();
        if l > crate::settings::LINEAR_SLOP {
            self.u.x *= 1.0 / l;
            self.u.y *= 1.0 / l;
        } else {
            self.u = Vec2::default();
        }
        let d = s * self.u.y - a * self.u.x;
        let u = o * self.u.y - h * self.u.x;
        let p = b1.inv_mass + b1.inv_i * d * d + b2.inv_mass + b2.inv_i * u * u;
        self.mass = 1.0 / p;
        if self.frequency_hz > 0.0 {
            let m = l - self.length;
            let f = 2.0 * std::f64::consts::PI * self.frequency_hz;
            let c = 2.0 * self.mass * self.damping_ratio * f;
            let v = self.mass * f * f;
            self.gamma = 1.0 / (step.dt * (c + step.dt * v));
            self.bias = m * step.dt * v * self.gamma;
            self.mass = 1.0 / (p + self.gamma);
        }
        if step.warm_starting {
            self.impulse *= step.dt_ratio;
            let (x, y) = (self.impulse * self.u.x, self.impulse * self.u.y);
            b1.linear_velocity.x -= b1.inv_mass * x;
            b1.linear_velocity.y -= b1.inv_mass * y;
            b1.angular_velocity -= b1.inv_i * (s * y - a * x);
            b2.linear_velocity.x += b2.inv_mass * x;
            b2.linear_velocity.y += b2.inv_mass * y;
            b2.angular_velocity += b2.inv_i * (o * y - h * x);
        } else {
            self.impulse = 0.0;
        }
    }

    fn solve_velocity(&mut self, b1: &mut Body, b2: &mut Body) {
        let ((i, s), (o, h)) = self.anchors(b1, b2);
        let l = b1.linear_velocity.x + -b1.angular_velocity * s;
        let d = b1.linear_velocity.y + b1.angular_velocity * i;
        let c = b2.linear_velocity.x + -b2.angular_velocity * h;
        let u = b2.linear_velocity.y + b2.angular_velocity * o;
        let p = self.u.x * (c - l) + self.u.y * (u - d);
        let m = -self.mass * (p + self.bias + self.gamma * self.impulse);
        self.impulse += m;
        let (f, g) = (m * self.u.x, m * self.u.y);
        b1.linear_velocity.x -= b1.inv_mass * f;
        b1.linear_velocity.y -= b1.inv_mass * g;
        b1.angular_velocity -= b1.inv_i * (i * g - s * f);
        b2.linear_velocity.x += b2.inv_mass * f;
        b2.linear_velocity.y += b2.inv_mass * g;
        b2.angular_velocity += b2.inv_i * (o * g - h * f);
    }

    fn solve_position(&mut self, b1: &mut Body, b2: &mut Body) -> bool {
        if self.frequency_hz > 0.0 {
            return true;
        }
        let ((r, i), (a, o)) = self.anchors(b1, b2);
        let mut l = b2.sweep.c.x + a - b1.sweep.c.x - r;
        let mut d = b2.sweep.c.y + o - b1.sweep.c.y - i;
        let u = (l * l + d * d).sqrt();
        l /= u;
        d /= u;
        let p = clamp(u - self.length, -crate::settings::MAX_LINEAR_CORRECTION, crate::settings::MAX_LINEAR_CORRECTION);
        let m = -self.mass * p;
        self.u = Vec2::new(l, d);
        let (f, g) = (m * self.u.x, m * self.u.y);
        b1.sweep.c.x -= b1.inv_mass * f;
        b1.sweep.c.y -= b1.inv_mass * g;
        b1.sweep.a -= b1.inv_i * (r * g - i * f);
        b2.sweep.c.x += b2.inv_mass * f;
        b2.sweep.c.y += b2.inv_mass * g;
        b2.sweep.a += b2.inv_i * (a * g - o * f);
        b1.synchronize_transform();
        b2.synchronize_transform();
        p.abs() < crate::settings::LINEAR_SLOP
    }
}

/// Rope joint (joint type 7, class at box2d-pretty.js 5540, built from the same def as a distance joint): the
/// anchors may not be further apart than `max_length` (one-sided, no softness).
#[derive(Debug, Clone)]
pub struct Rope {
    pub local_anchor1: Vec2,
    pub local_anchor2: Vec2,
    u: Vec2,
    length: f64,
    pub max_length: f64,
    impulse: f64,
    mass: f64,
    inv_dt: f64,
}

impl Joint {
    pub fn new_rope(d: &DistanceJointDef) -> Self {
        Joint {
            body1: d.body1,
            body2: d.body2,
            collide_connected: d.collide_connected,
            island_flag: false,
            user_data: d.user_data,
            kind: JointKind::Rope(Rope {
                local_anchor1: d.local_anchor1,
                local_anchor2: d.local_anchor2,
                u: Vec2::default(),
                length: 0.0,
                max_length: d.length,
                impulse: 0.0,
                mass: 0.0,
                inv_dt: 0.0,
            }),
        }
    }
}

impl Rope {
    fn anchors(&self, b1: &Body, b2: &Body) -> ((f64, f64), (f64, f64)) {
        (
            rot(&b1.xf.r, self.local_anchor1.x - b1.sweep.local_center.x, self.local_anchor1.y - b1.sweep.local_center.y),
            rot(&b2.xf.r, self.local_anchor2.x - b2.sweep.local_center.x, self.local_anchor2.y - b2.sweep.local_center.y),
        )
    }

    fn init_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        self.inv_dt = step.inv_dt;
        let ((s, a), (o, h)) = self.anchors(b1, b2);
        self.u.x = b2.sweep.c.x + o - b1.sweep.c.x - s;
        self.u.y = b2.sweep.c.y + h - b1.sweep.c.y - a;
        self.length = (self.u.x * self.u.x + self.u.y * self.u.y).sqrt();
        if !(self.length > crate::settings::LINEAR_SLOP) {
            self.u = Vec2::default();
            self.mass = 0.0;
            self.impulse = 0.0;
            return;
        }
        self.u.x *= 1.0 / self.length;
        self.u.y *= 1.0 / self.length;
        let l = s * self.u.y - a * self.u.x;
        let d = o * self.u.y - h * self.u.x;
        let u = b1.inv_mass + b1.inv_i * l * l + b2.inv_mass + b2.inv_i * d * d;
        self.mass = 1.0 / u;
        if step.warm_starting {
            self.impulse *= step.dt_ratio;
            let (p, m) = (self.impulse * self.u.x, self.impulse * self.u.y);
            b1.linear_velocity.x -= b1.inv_mass * p;
            b1.linear_velocity.y -= b1.inv_mass * m;
            b1.angular_velocity -= b1.inv_i * (s * m - a * p);
            b2.linear_velocity.x += b2.inv_mass * p;
            b2.linear_velocity.y += b2.inv_mass * m;
            b2.angular_velocity += b2.inv_i * (o * m - h * p);
        } else {
            self.impulse = 0.0;
        }
    }

    fn solve_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        let ((i, s), (o, h)) = self.anchors(b1, b2);
        let l = b1.linear_velocity.x + -b1.angular_velocity * s;
        let d = b1.linear_velocity.y + b1.angular_velocity * i;
        let c = b2.linear_velocity.x + -b2.angular_velocity * h;
        let u = b2.linear_velocity.y + b2.angular_velocity * o;
        let mut p = self.u.x * (c - l) + self.u.y * (u - d);
        let m = self.length - self.max_length;
        if m < 0.0 {
            p += step.inv_dt * m;
        }
        let f = -self.mass * p;
        let old = self.impulse;
        self.impulse = (self.impulse + f).min(0.0);
        let f = self.impulse - old;
        let (v, y) = (f * self.u.x, f * self.u.y);
        b1.linear_velocity.x -= b1.inv_mass * v;
        b1.linear_velocity.y -= b1.inv_mass * y;
        b1.angular_velocity -= b1.inv_i * (i * y - s * v);
        b2.linear_velocity.x += b2.inv_mass * v;
        b2.linear_velocity.y += b2.inv_mass * y;
        b2.angular_velocity += b2.inv_i * (o * y - h * v);
    }

    fn solve_position(&mut self, b1: &mut Body, b2: &mut Body) -> bool {
        let ((r, i), (a, o)) = self.anchors(b1, b2);
        let mut l = b2.sweep.c.x + a - b1.sweep.c.x - r;
        let mut d = b2.sweep.c.y + o - b1.sweep.c.y - i;
        let u = (l * l + d * d).sqrt();
        l /= u;
        d /= u;
        let p = clamp(u - self.max_length, 0.0, crate::settings::MAX_LINEAR_CORRECTION);
        let m = -self.mass * p;
        self.u = Vec2::new(l, d);
        let (f, g) = (m * self.u.x, m * self.u.y);
        b1.sweep.c.x -= b1.inv_mass * f;
        b1.sweep.c.y -= b1.inv_mass * g;
        b1.sweep.a -= b1.inv_i * (r * g - i * f);
        b2.sweep.c.x += b2.inv_mass * f;
        b2.sweep.c.y += b2.inv_mass * g;
        b2.sweep.a += b2.inv_i * (a * g - o * f);
        b1.synchronize_transform();
        b2.synchronize_transform();
        u - self.max_length < crate::settings::LINEAR_SLOP
    }
}

/// `b2GearJoint` (box2d-pretty.js 5352), revolute–revolute only: couples the angles of two revolute joints,
/// `angle1 + ratio * angle2 = constant`. `body1`/`body2` are the *second* bodies of the two joints
/// (the originals' `m_body1`/`m_body2`); `ground1`/`ground2` are their first bodies.
#[derive(Debug, Clone)]
pub struct Gear {
    ground1: BodyId,
    ground2: BodyId,
    ref1: f64,
    ref2: f64,
    pub ratio: f64,
    constant: f64,
    pub force: f64,
    mass: f64,
}

impl Joint {
    /// `GearJointDef{joint1, joint2, ratio}` for two revolute joints (their ids are looked up by the caller).
    pub fn new_gear(j1: &Joint, j2: &Joint, bodies: &[Body], ratio: f64) -> Option<Self> {
        let (JointKind::Revolute(r1), JointKind::Revolute(r2)) = (&j1.kind, &j2.kind) else { return None };
        let angle = |j: &Joint, r: &Revolute| bodies[j.body2].sweep.a - bodies[j.body1].sweep.a - r.reference_angle;
        let (a1, a2) = (angle(j1, r1), angle(j2, r2));
        Some(Joint {
            body1: j1.body2,
            body2: j2.body2,
            collide_connected: false,
            island_flag: false,
            user_data: 0,
            kind: JointKind::Gear(Gear {
                ground1: j1.body1,
                ground2: j2.body1,
                ref1: r1.reference_angle,
                ref2: r2.reference_angle,
                ratio,
                constant: a1 + ratio * a2,
                force: 0.0,
                mass: 0.0,
            }),
        })
    }
}

impl Gear {
    fn init_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        self.mass = 1.0 / (b1.inv_i + self.ratio * self.ratio * b2.inv_i);
        if step.warm_starting {
            let m = step.dt * self.force;
            b1.angular_velocity -= b1.inv_i * m;
            b2.angular_velocity += b2.inv_i * m * -self.ratio;
        } else {
            self.force = 0.0;
        }
    }

    fn solve_velocity(&mut self, b1: &mut Body, b2: &mut Body, step: &StepParams) {
        let cdot = -b1.angular_velocity + -self.ratio * b2.angular_velocity;
        let i = -step.inv_dt * self.mass * cdot;
        self.force += i;
        let s = step.dt * i;
        b1.angular_velocity -= b1.inv_i * s;
        b2.angular_velocity += b2.inv_i * s * -self.ratio;
    }

    fn solve_position(&mut self, bodies: &mut [Body], i1: BodyId, i2: BodyId) -> bool {
        let e = bodies[i1].sweep.a - bodies[self.ground1].sweep.a - self.ref1;
        let t = bodies[i2].sweep.a - bodies[self.ground2].sweep.a - self.ref2;
        let c = self.constant - (e + self.ratio * t);
        let s = -self.mass * c;
        let (a, b) = two(bodies, i1, i2);
        a.sweep.a -= a.inv_i * s;
        b.sweep.a += b.inv_i * s * -self.ratio;
        a.synchronize_transform();
        b.synchronize_transform();
        true
    }
}
