//! `b2World` / `b2Body` / contacts / `b2ContactSolver` / `b2Island` (no joints, no TOI yet).
//! Transcribed from box2d-pretty.js: bodies ~2990-3530, contacts ~3596-4040, solver ~4338-4780,
//! world ~6930-7176. Linked lists with head insertion are kept as `Vec`s iterated newest-first, which
//! preserves the original iteration order.
use crate::broadphase::{BroadPhase, PairEvent, NULL_PROXY};
use crate::collision::*;
use crate::math::mul_mv as box2d_mul;
use crate::joints::*;
use crate::math::*;
use crate::settings::*;
use crate::shape::*;

pub type BodyId = usize;
pub type ShapeId = usize;
pub type ContactId = usize;
const NULL_CONTACT: usize = usize::MAX;

const FROZEN: u32 = 2;
const ISLAND: u32 = 4;
const SLEEP: u32 = 8;
const ALLOW_SLEEP: u32 = 16;
const BULLET: u32 = 32;
const FIXED_ROTATION: u32 = 64;

#[derive(Debug, Clone)]
pub struct BodyDef {
    pub position: Vec2,
    pub angle: f64,
    pub mass_data: MassData,
    pub linear_damping: f64,
    pub angular_damping: f64,
    pub allow_sleep: bool,
    pub is_sleeping: bool,
    pub fixed_rotation: bool,
    pub is_bullet: bool,
    pub user_data: u64,
}

impl Default for BodyDef {
    fn default() -> Self {
        Self {
            position: Vec2::default(),
            angle: 0.0,
            mass_data: MassData::default(),
            linear_damping: 0.0,
            angular_damping: 0.0,
            allow_sleep: true,
            is_sleeping: false,
            fixed_rotation: false,
            is_bullet: false,
            user_data: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Sweep {
    pub local_center: Vec2,
    pub c0: Vec2,
    pub c: Vec2,
    pub a0: f64,
    pub a: f64,
    pub t0: f64,
}

impl Sweep {
    pub fn advance(&mut self, t: f64) {
        if self.t0 < t && 1.0 - self.t0 > 5e-324 {
            let a = (t - self.t0) / (1.0 - self.t0);
            self.c0.x = (1.0 - a) * self.c0.x + a * self.c.x;
            self.c0.y = (1.0 - a) * self.c0.y + a * self.c.y;
            self.a0 = (1.0 - a) * self.a0 + a * self.a;
            self.t0 = t;
        }
    }
}

#[derive(Debug, Clone)]
pub struct Body {
    pub xf: XForm,
    pub sweep: Sweep,
    pub linear_velocity: Vec2,
    pub angular_velocity: f64,
    pub force: Vec2,
    pub torque: f64,
    pub flags: u32,
    pub mass: f64,
    pub inv_mass: f64,
    pub i: f64,
    pub inv_i: f64,
    pub linear_damping: f64,
    pub angular_damping: f64,
    pub sleep_time: f64,
    pub is_static: bool,
    pub user_data: u64,
    /// Shapes in creation order (the original iterates its list newest-first).
    pub shapes: Vec<ShapeId>,
    /// Contacts touching this body in creation order (iterated newest-first).
    pub contacts: Vec<ContactId>,
    /// Joints attached (creation order): `(joint id, other body, collide_connected)`.
    pub joints: Vec<(usize, BodyId, bool)>,
    pub alive: bool,
}

impl Body {
    pub fn is_sleeping(&self) -> bool {
        self.flags & SLEEP != 0
    }
    pub fn is_bullet(&self) -> bool {
        self.flags & BULLET != 0
    }
    pub fn set_bullet(&mut self, on: bool) {
        if on {
            self.flags |= BULLET;
        } else {
            self.flags &= !BULLET;
        }
    }
    pub fn wake_up(&mut self) {
        self.flags &= !SLEEP;
        self.sleep_time = 0.0;
    }
    pub fn put_to_sleep(&mut self) {
        self.flags |= SLEEP;
        self.sleep_time = 0.0;
        self.linear_velocity = Vec2::default();
        self.angular_velocity = 0.0;
        self.force = Vec2::default();
        self.torque = 0.0;
    }
    pub fn world_point(&self, p: Vec2) -> Vec2 {
        let r = &self.xf.r;
        Vec2::new(r.col1.x * p.x + r.col2.x * p.y + self.xf.position.x, r.col1.y * p.x + r.col2.y * p.y + self.xf.position.y)
    }
    pub fn local_point(&self, p: Vec2) -> Vec2 {
        mul_xt(&self.xf, p)
    }
    pub fn linear_velocity_from_local_point(&self, p: Vec2) -> Vec2 {
        let w = self.world_point(p);
        Vec2::new(
            self.linear_velocity.x + self.angular_velocity * (w.y - self.sweep.c.y),
            self.linear_velocity.y - self.angular_velocity * (w.x - self.sweep.c.x),
        )
    }
    pub fn linear_velocity_from_world_point(&self, w: Vec2) -> Vec2 {
        Vec2::new(
            self.linear_velocity.x + self.angular_velocity * (w.y - self.sweep.c.y),
            self.linear_velocity.y - self.angular_velocity * (w.x - self.sweep.c.x),
        )
    }
    pub fn synchronize_transform(&mut self) {
        self.xf.r.set_angle(self.sweep.a);
        let r = self.xf.r;
        let lc = self.sweep.local_center;
        self.xf.position.x = self.sweep.c.x - (r.col1.x * lc.x + r.col2.x * lc.y);
        self.xf.position.y = self.sweep.c.y - (r.col1.y * lc.x + r.col2.y * lc.y);
    }
    pub fn apply_force(&mut self, f: Vec2, p: Vec2) {
        if self.is_sleeping() {
            self.wake_up();
        }
        self.force.x += f.x;
        self.force.y += f.y;
        self.torque += (p.x - self.sweep.c.x) * f.y - (p.y - self.sweep.c.y) * f.x;
    }
    pub fn apply_torque(&mut self, t: f64) {
        if self.is_sleeping() {
            self.wake_up();
        }
        self.torque += t;
    }
    pub fn apply_impulse(&mut self, f: Vec2, p: Vec2) {
        if self.is_sleeping() {
            self.wake_up();
        }
        self.linear_velocity.x += self.inv_mass * f.x;
        self.linear_velocity.y += self.inv_mass * f.y;
        self.angular_velocity += self.inv_i * ((p.x - self.sweep.c.x) * f.y - (p.y - self.sweep.c.y) * f.x);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactKind {
    CircleCircle,
    PolygonCircle,
    PolygonPolygon,
}

#[derive(Debug, Clone)]
pub struct Contact {
    pub shape1: ShapeId,
    pub shape2: ShapeId,
    kind: ContactKind,
    pub manifold: Manifold,
    pub manifold_count: usize,
    pub flags: u32,
    pub friction: f64,
    pub restitution: f64,
    pub toi: f64,
}

const C_NONSOLID: u32 = 1;
const C_SLOW: u32 = 2;
const C_ISLAND: u32 = 4;
const C_TOI: u32 = 8;

pub struct ShapeRec {
    pub shape: Shape,
    pub body: BodyId,
    pub proxy: u16,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ContactEventKind {
    Add,
    Persist,
    Remove,
    /// Post-solve report (`b2ContactListener.Result`): `normal_impulse` is the point's accumulated impulse.
    Result,
}

/// Mirrors `b2ContactPoint` handed to the original's contact listener.
#[derive(Debug, Clone, Copy)]
pub struct ContactEvent {
    pub kind: ContactEventKind,
    pub shape1: ShapeId,
    pub shape2: ShapeId,
    pub position: Vec2,
    pub velocity: Vec2,
    pub normal: Vec2,
    pub separation: f64,
    pub friction: f64,
    pub restitution: f64,
    pub id: ContactId2,
    /// Index of the owning contact (`Core` slot).
    pub contact: usize,
    /// Only set for `Result` events.
    pub normal_impulse: f64,
}

pub type ContactId2 = crate::collision::ContactId;

pub struct StepParams {
    pub dt: f64,
    pub inv_dt: f64,
    pub dt_ratio: f64,
    pub max_iterations: usize,
    pub warm_starting: bool,
    pub position_correction: bool,
}

pub struct Core {
    pub bodies: Vec<Body>,
    /// Body ids in creation order (iterated newest-first).
    body_order: Vec<BodyId>,
    pub shapes: Vec<ShapeRec>,
    contacts: Vec<Option<Contact>>,
    contact_order: Vec<ContactId>,
    pub joints: Vec<Joint>,
    joint_order: Vec<usize>,
    pub events: Vec<ContactEvent>,
    /// Emit `Result` events after each island solve (only needed when something listens).
    pub report_results: bool,
    pub gravity: Vec2,
    pub allow_sleep: bool,
    inv_dt0: f64,
    locked: bool,
}

pub struct World {
    pub bp: BroadPhase,
    pub c: Core,
}

fn should_collide(a: &Shape, b: &Shape) -> bool {
    let (fa, fb) = (a.filter, b.filter);
    if fa.group_index == fb.group_index && fa.group_index != 0 {
        fa.group_index > 0
    } else {
        fa.mask_bits & fb.category_bits != 0 && fa.category_bits & fb.mask_bits != 0
    }
}

impl Core {
    fn pair_event(&mut self, ev: PairEvent) -> Option<usize> {
        match ev {
            PairEvent::Added(s1, s2) => Some(self.pair_added(s1, s2)),
            PairEvent::Removed(_, _, ud) => {
                if let Some(c) = ud {
                    if c != NULL_CONTACT {
                        self.destroy_contact(c);
                    }
                }
                None
            }
        }
    }

    fn pair_added(&mut self, s1: ShapeId, s2: ShapeId) -> usize {
        let (b1, b2) = (self.shapes[s1].body, self.shapes[s2].body);
        if self.bodies[b1].is_static && self.bodies[b2].is_static {
            return NULL_CONTACT;
        }
        if b1 == b2 {
            return NULL_CONTACT;
        }
        // body2.IsConnected(body1): a joint exists with collideConnected == false
        if self.bodies[b2].joints.iter().any(|&(_, other, collide)| other == b1 && !collide) {
            return NULL_CONTACT;
        }
        if !should_collide(&self.shapes[s1].shape, &self.shapes[s2].shape) {
            return NULL_CONTACT;
        }
        let (t1, t2) = (&self.shapes[s1].shape.kind, &self.shapes[s2].shape.kind);
        let (sa, sb, kind) = match (t1, t2) {
            (ShapeKind::Circle(_), ShapeKind::Circle(_)) => (s1, s2, ContactKind::CircleCircle),
            (ShapeKind::Polygon { .. }, ShapeKind::Circle(_)) => (s1, s2, ContactKind::PolygonCircle),
            (ShapeKind::Circle(_), ShapeKind::Polygon { .. }) => (s2, s1, ContactKind::PolygonCircle),
            (ShapeKind::Polygon { .. }, ShapeKind::Polygon { .. }) => (s1, s2, ContactKind::PolygonPolygon),
        };
        let (a, b) = (&self.shapes[sa].shape, &self.shapes[sb].shape);
        let mut flags = 0;
        if a.is_sensor || b.is_sensor {
            flags |= C_NONSOLID;
        }
        let contact = Contact {
            shape1: sa,
            shape2: sb,
            kind,
            manifold: Manifold::default(),
            manifold_count: 0,
            flags,
            friction: (a.friction * b.friction).sqrt(),
            restitution: if a.restitution > b.restitution { a.restitution } else { b.restitution },
            toi: 0.0,
        };
        let id = self.contacts.len();
        self.contacts.push(Some(contact));
        self.contact_order.push(id);
        let (ba, bb) = (self.shapes[sa].body, self.shapes[sb].body);
        self.bodies[ba].contacts.push(id);
        self.bodies[bb].contacts.push(id);
        id
    }

    fn destroy_contact(&mut self, id: ContactId) {
        let c = self.contacts[id].take().unwrap();
        let (b1, b2) = (self.shapes[c.shape1].body, self.shapes[c.shape2].body);
        if c.manifold_count > 0 {
            self.bodies[b1].wake_up();
            self.bodies[b2].wake_up();
        }
        self.contact_order.retain(|&x| x != id);
        self.bodies[b1].contacts.retain(|&x| x != id);
        self.bodies[b2].contacts.retain(|&x| x != id);
    }

    fn contact(&self, id: ContactId) -> &Contact {
        self.contacts[id].as_ref().unwrap()
    }

    fn emit(&mut self, kind: ContactEventKind, cid: ContactId, m: &Manifold, p: &ManifoldPoint) {
        let c = self.contact(cid);
        let (s1, s2, fr, re) = (c.shape1, c.shape2, c.friction, c.restitution);
        let (b1, b2) = (&self.bodies[self.shapes[s1].body], &self.bodies[self.shapes[s2].body]);
        let v1 = b1.linear_velocity_from_local_point(p.local_point1);
        let v2 = b2.linear_velocity_from_local_point(p.local_point2);
        let ev = ContactEvent {
            kind,
            shape1: s1,
            shape2: s2,
            position: b1.world_point(p.local_point1),
            velocity: Vec2::new(v2.x - v1.x, v2.y - v1.y),
            normal: m.normal,
            separation: p.separation,
            friction: fr,
            restitution: re,
            id: p.id,
            contact: cid,
            normal_impulse: 0.0,
        };
        self.events.push(ev);
    }

    /// `b2Contact.Update` (+ the type-specific `Evaluate`).
    fn update_contact(&mut self, cid: ContactId) {
        let (s1, s2, kind) = {
            let c = self.contact(cid);
            (c.shape1, c.shape2, c.kind)
        };
        let (b1, b2) = (self.shapes[s1].body, self.shapes[s2].body);
        let (xf1, xf2) = (self.bodies[b1].xf, self.bodies[b2].xf);
        let old_count = self.contact(cid).manifold_count;
        let m0 = self.contact(cid).manifold;
        let mut m = m0;
        match kind {
            ContactKind::CircleCircle => {
                let (ShapeKind::Circle(c1), ShapeKind::Circle(c2)) = (&self.shapes[s1].shape.kind, &self.shapes[s2].shape.kind) else { unreachable!() };
                collide_circles(&mut m, c1, &xf1, c2, &xf2);
            }
            ContactKind::PolygonCircle => {
                let (ShapeKind::Polygon { poly, .. }, ShapeKind::Circle(c)) = (&self.shapes[s1].shape.kind, &self.shapes[s2].shape.kind) else { unreachable!() };
                collide_polygon_and_circle(&mut m, poly, &xf1, c, &xf2);
            }
            ContactKind::PolygonPolygon => {
                let (ShapeKind::Polygon { poly: p1, .. }, ShapeKind::Polygon { poly: p2, .. }) = (&self.shapes[s1].shape.kind, &self.shapes[s2].shape.kind) else { unreachable!() };
                collide_polygons(&mut m, p1, &xf1, p2, &xf2);
            }
        }
        let new_count;
        if kind == ContactKind::CircleCircle {
            if m.point_count > 0 {
                new_count = 1;
                if m0.point_count == 0 {
                    m.points[0].normal_impulse = 0.0;
                    m.points[0].tangent_impulse = 0.0;
                    let p = m.points[0];
                    self.emit(ContactEventKind::Add, cid, &m, &p);
                } else {
                    m.points[0].normal_impulse = m0.points[0].normal_impulse;
                    m.points[0].tangent_impulse = m0.points[0].tangent_impulse;
                    let p = m.points[0];
                    self.emit(ContactEventKind::Persist, cid, &m, &p);
                }
            } else {
                new_count = 0;
                if m0.point_count > 0 {
                    let p = m0.points[0];
                    self.emit(ContactEventKind::Remove, cid, &m0, &p);
                }
            }
        } else {
            let mut matched = [false; 2];
            if m.point_count > 0 {
                for i in 0..m.point_count {
                    m.points[i].normal_impulse = 0.0;
                    m.points[i].tangent_impulse = 0.0;
                    let key = m.points[i].id.key();
                    let mut found = false;
                    for j in 0..m0.point_count {
                        if !matched[j] && m0.points[j].id.key() == key {
                            matched[j] = true;
                            m.points[i].normal_impulse = m0.points[j].normal_impulse;
                            m.points[i].tangent_impulse = m0.points[j].tangent_impulse;
                            found = true;
                            let p = m.points[i];
                            self.emit(ContactEventKind::Persist, cid, &m, &p);
                            break;
                        }
                    }
                    if !found {
                        let p = m.points[i];
                        self.emit(ContactEventKind::Add, cid, &m, &p);
                    }
                }
                new_count = 1;
            } else {
                new_count = 0;
            }
            for j in 0..m0.point_count {
                if !matched[j] {
                    let p = m0.points[j];
                    self.emit(ContactEventKind::Remove, cid, &m0, &p);
                }
            }
        }
        {
            let c = self.contacts[cid].as_mut().unwrap();
            c.manifold = m;
            c.manifold_count = new_count;
        }
        if new_count == 0 && old_count > 0 {
            self.bodies[b1].wake_up();
            self.bodies[b2].wake_up();
        }
        let (a, b) = (&self.bodies[b1], &self.bodies[b2]);
        let fast = a.is_static || a.is_bullet() || b.is_static || b.is_bullet();
        let c = self.contacts[cid].as_mut().unwrap();
        if fast {
            c.flags &= !C_SLOW;
        } else {
            c.flags |= C_SLOW;
        }
    }

    fn collide(&mut self) {
        let order: Vec<ContactId> = self.contact_order.iter().rev().copied().collect();
        for cid in order {
            let c = self.contact(cid);
            let (b1, b2) = (self.shapes[c.shape1].body, self.shapes[c.shape2].body);
            if self.bodies[b1].is_sleeping() && self.bodies[b2].is_sleeping() {
                continue;
            }
            self.update_contact(cid);
        }
    }

    pub fn contact_ids(&self) -> impl Iterator<Item = ContactId> + '_ {
        self.contact_order.iter().rev().copied()
    }
    pub fn contact_info(&self, id: ContactId) -> &Contact {
        self.contact(id)
    }

    /// Marks every existing contact of `shape` as non-solid, as if the shape had already been a sensor when the contact was created
    /// (`b2Contact` decides `e_nonSolidFlag` in its constructor, so flipping `m_isSensor` afterwards leaves old contacts solid).
    pub fn make_contacts_non_solid(&mut self, shape: ShapeId) {
        let ids: Vec<ContactId> = self.contact_order.to_vec();
        for id in ids {
            let c = self.contact(id);
            if c.shape1 == shape || c.shape2 == shape {
                if let Some(c) = self.contacts[id].as_mut() {
                    c.flags |= C_NONSOLID;
                }
            }
        }
    }
}

struct ContactConstraintPoint {
    local_anchor1: Vec2,
    local_anchor2: Vec2,
    r1: Vec2,
    r2: Vec2,
    normal_impulse: f64,
    tangent_impulse: f64,
    position_impulse: f64,
    normal_mass: f64,
    tangent_mass: f64,
    equalized_mass: f64,
    separation: f64,
    velocity_bias: f64,
}

struct ContactConstraint {
    points: Vec<ContactConstraintPoint>,
    normal: Vec2,
    body1: BodyId,
    body2: BodyId,
    friction: f64,
    restitution: f64,
    contact: ContactId,
}

fn rot(r: &Mat22, x: f64, y: f64) -> (f64, f64) {
    (r.col1.x * x + r.col2.x * y, r.col1.y * x + r.col2.y * y)
}

fn build_constraints(core: &Core, contacts: &[ContactId]) -> Vec<ContactConstraint> {
    let mut out = Vec::new();
    for &cid in contacts {
        let c = core.contact(cid);
        if c.manifold_count == 0 {
            continue;
        }
        let (b1, b2) = (core.shapes[c.shape1].body, core.shapes[c.shape2].body);
        let (h, l) = (&core.bodies[b1], &core.bodies[b2]);
        let m = &c.manifold;
        let (tx, ty) = (m.normal.x, m.normal.y);
        let mut cc = ContactConstraint { points: Vec::new(), normal: m.normal, body1: b1, body2: b2, friction: c.friction, restitution: c.restitution, contact: cid };
        for s in 0..m.point_count {
            let e = &m.points[s];
            let (k, p) = rot(&h.xf.r, e.local_point1.x - h.sweep.local_center.x, e.local_point1.y - h.sweep.local_center.y);
            let (r2x, r2y) = rot(&l.xf.r, e.local_point2.x - l.sweep.local_center.x, e.local_point2.y - l.sweep.local_center.y);
            let (r1x, r1y) = (k, p);
            let ll = r1x * r1x + r1y * r1y;
            let jj = r2x * r2x + r2y * r2y;
            let bb = r1x * tx + r1y * ty;
            let dd = r2x * tx + r2y * ty;
            let mut f = h.inv_mass + l.inv_mass;
            f += h.inv_i * (ll - bb * bb) + l.inv_i * (jj - dd * dd);
            let normal_mass = 1.0 / f;
            let mut n = h.mass * h.inv_mass + l.mass * l.inv_mass;
            n += h.mass * h.inv_i * (ll - bb * bb) + l.mass * l.inv_i * (jj - dd * dd);
            let equalized_mass = 1.0 / n;
            let g = -tx;
            let u = r1x * ty + r1y * g;
            let v = r2x * ty + r2y * g;
            let mut hh = h.inv_mass + l.inv_mass;
            hh += h.inv_i * (ll - u * u) + l.inv_i * (jj - v * v);
            let tangent_mass = 1.0 / hh;
            let mut velocity_bias = 0.0;
            if e.separation > 0.0 {
                velocity_bias = -60.0 * e.separation;
            }
            let dvx = l.linear_velocity.x + -l.angular_velocity * r2y - h.linear_velocity.x - -h.angular_velocity * r1y;
            let dvy = l.linear_velocity.y + l.angular_velocity * r2x - h.linear_velocity.y - h.angular_velocity * r1x;
            let w = cc.normal.x * dvx + cc.normal.y * dvy;
            if w < -VELOCITY_THRESHOLD {
                velocity_bias += -cc.restitution * w;
            }
            cc.points.push(ContactConstraintPoint {
                local_anchor1: e.local_point1,
                local_anchor2: e.local_point2,
                r1: Vec2::new(r1x, r1y),
                r2: Vec2::new(r2x, r2y),
                normal_impulse: e.normal_impulse,
                tangent_impulse: e.tangent_impulse,
                position_impulse: 0.0,
                normal_mass,
                tangent_mass,
                equalized_mass,
                separation: e.separation,
                velocity_bias,
            });
        }
        out.push(cc);
    }
    out
}

fn init_velocity_constraints(cs: &mut [ContactConstraint], bodies: &mut [Body], step: &StepParams) {
    for c in cs.iter_mut() {
        let (i1, i2) = (c.body1, c.body2);
        let (m1, ii1, m2, ii2) = (bodies[i1].inv_mass, bodies[i1].inv_i, bodies[i2].inv_mass, bodies[i2].inv_i);
        let (nx, ny) = (c.normal.x, c.normal.y);
        let (tx, ty) = (ny, -nx);
        if step.warm_starting {
            for p in c.points.iter_mut() {
                p.normal_impulse *= step.dt_ratio;
                p.tangent_impulse *= step.dt_ratio;
                let px = p.normal_impulse * nx + p.tangent_impulse * tx;
                let py = p.normal_impulse * ny + p.tangent_impulse * ty;
                bodies[i1].angular_velocity -= ii1 * (p.r1.x * py - p.r1.y * px);
                bodies[i1].linear_velocity.x -= m1 * px;
                bodies[i1].linear_velocity.y -= m1 * py;
                bodies[i2].angular_velocity += ii2 * (p.r2.x * py - p.r2.y * px);
                bodies[i2].linear_velocity.x += m2 * px;
                bodies[i2].linear_velocity.y += m2 * py;
            }
        } else {
            for p in c.points.iter_mut() {
                p.normal_impulse = 0.0;
                p.tangent_impulse = 0.0;
            }
        }
    }
}

fn solve_velocity_constraints(cs: &mut [ContactConstraint], bodies: &mut [Body]) {
    for c in cs.iter_mut() {
        let (i1, i2) = (c.body1, c.body2);
        let mut w1 = bodies[i1].angular_velocity;
        let mut w2 = bodies[i2].angular_velocity;
        let (m1, ii1, m2, ii2) = (bodies[i1].inv_mass, bodies[i1].inv_i, bodies[i2].inv_mass, bodies[i2].inv_i);
        let (nx, ny) = (c.normal.x, c.normal.y);
        let (tx, ty) = (ny, -nx);
        let friction = c.friction;
        for p in c.points.iter_mut() {
            let v1 = bodies[i1].linear_velocity;
            let v2 = bodies[i2].linear_velocity;
            let dvx = v2.x + -w2 * p.r2.y - v1.x - -w1 * p.r1.y;
            let dvy = v2.y + w2 * p.r2.x - v1.y - w1 * p.r1.x;
            let vn = dvx * nx + dvy * ny;
            let mut lambda = -p.normal_mass * (vn - p.velocity_bias);
            let vt = dvx * tx + dvy * ty;
            let mut lt = p.tangent_mass * -vt;
            let new_n = (p.normal_impulse + lambda).max(0.0);
            lambda = new_n - p.normal_impulse;
            let max_f = friction * p.normal_impulse;
            let new_t = clamp(p.tangent_impulse + lt, -max_f, max_f);
            lt = new_t - p.tangent_impulse;
            let px = lambda * nx + lt * tx;
            let py = lambda * ny + lt * ty;
            bodies[i1].linear_velocity.x -= m1 * px;
            bodies[i1].linear_velocity.y -= m1 * py;
            w1 -= ii1 * (p.r1.x * py - p.r1.y * px);
            bodies[i2].linear_velocity.x += m2 * px;
            bodies[i2].linear_velocity.y += m2 * py;
            w2 += ii2 * (p.r2.x * py - p.r2.y * px);
            p.normal_impulse = new_n;
            p.tangent_impulse = new_t;
        }
        bodies[i1].angular_velocity = w1;
        bodies[i2].angular_velocity = w2;
    }
}

fn solve_position_constraints(cs: &mut [ContactConstraint], bodies: &mut [Body], baumgarte: f64) -> bool {
    let mut min_sep = 0.0f64;
    for c in cs.iter_mut() {
        let (i1, i2) = (c.body1, c.body2);
        let (b1, b2) = (&bodies[i1], &bodies[i2]);
        let m1 = b1.mass * b1.inv_mass;
        let ii1 = b1.mass * b1.inv_i;
        let m2 = b2.mass * b2.inv_mass;
        let ii2 = b2.mass * b2.inv_i;
        let mut a1 = b1.sweep.a;
        let mut a2 = b2.sweep.a;
        let (nx, ny) = (c.normal.x, c.normal.y);
        for p in c.points.iter_mut() {
            let (b1, b2) = (&bodies[i1], &bodies[i2]);
            let (r1x, r1y) = rot(&b1.xf.r, p.local_anchor1.x - b1.sweep.local_center.x, p.local_anchor1.y - b1.sweep.local_center.y);
            let (r2x, r2y) = rot(&b2.xf.r, p.local_anchor2.x - b2.sweep.local_center.x, p.local_anchor2.y - b2.sweep.local_center.y);
            let o = b1.sweep.c.x + r1x;
            let k = b1.sweep.c.y + r1y;
            let sep = (b2.sweep.c.x + r2x - o) * nx + (b2.sweep.c.y + r2y - k) * ny + p.separation;
            min_sep = min_sep.min(sep);
            let corr = baumgarte * clamp(sep + LINEAR_SLOP, -MAX_LINEAR_CORRECTION, 0.0);
            let mut imp = -p.equalized_mass * corr;
            let old = p.position_impulse;
            p.position_impulse = (old + imp).max(0.0);
            imp = p.position_impulse - old;
            let lx = imp * nx;
            let ly = imp * ny;
            bodies[i1].sweep.c.x -= m1 * lx;
            bodies[i1].sweep.c.y -= m1 * ly;
            a1 -= ii1 * (r1x * ly - r1y * lx);
            bodies[i1].sweep.a = a1;
            bodies[i1].synchronize_transform();
            bodies[i2].sweep.c.x += m2 * lx;
            bodies[i2].sweep.c.y += m2 * ly;
            a2 += ii2 * (r2x * ly - r2y * lx);
            bodies[i2].sweep.a = a2;
            bodies[i2].synchronize_transform();
        }
    }
    min_sep >= -1.5 * LINEAR_SLOP
}

/// Render pose of a body (`m_xf`, `m_sweep.c/a`), for display interpolation.
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub position: Vec2,
    pub r: Mat22,
    pub c: Vec2,
    pub a: f64,
    pub v: Vec2,
}

impl World {
    /// Poses of all bodies in id order (destroyed ones included, so indices stay stable).
    pub fn pose_snapshot(&self) -> Vec<Pose> {
        self.c.bodies.iter().map(|b| Pose { position: b.xf.position, r: b.xf.r, c: b.sweep.c, a: b.sweep.a, v: b.linear_velocity }).collect()
    }

    /// Display interpolation of the original's 60 fps mode (`b2Body.UpdateInterpolation`): every living non-static body that existed in `from`
    /// is moved to `from + (now − from)·t` (matrix columns, position, centre of mass, linear velocity and angle). Returns the true poses for `restore_poses`.
    pub fn blend_poses(&mut self, from: &[Pose], t: f64) -> Vec<Pose> {
        let saved = self.pose_snapshot();
        let lerp = |a: f64, b: f64| a + (b - a) * t;
        for (b, f) in self.c.bodies.iter_mut().zip(from) {
            if !b.alive || b.is_static {
                continue;
            }
            b.xf.position = Vec2::new(lerp(f.position.x, b.xf.position.x), lerp(f.position.y, b.xf.position.y));
            b.xf.r.col1 = Vec2::new(lerp(f.r.col1.x, b.xf.r.col1.x), lerp(f.r.col1.y, b.xf.r.col1.y));
            b.xf.r.col2 = Vec2::new(lerp(f.r.col2.x, b.xf.r.col2.x), lerp(f.r.col2.y, b.xf.r.col2.y));
            b.sweep.c = Vec2::new(lerp(f.c.x, b.sweep.c.x), lerp(f.c.y, b.sweep.c.y));
            b.sweep.a = lerp(f.a, b.sweep.a);
            b.linear_velocity = Vec2::new(lerp(f.v.x, b.linear_velocity.x), lerp(f.v.y, b.linear_velocity.y));
        }
        saved
    }

    pub fn restore_poses(&mut self, saved: &[Pose]) {
        for (b, s) in self.c.bodies.iter_mut().zip(saved) {
            b.xf.position = s.position;
            b.xf.r = s.r;
            b.sweep.c = s.c;
            b.sweep.a = s.a;
            b.linear_velocity = s.v;
        }
    }

    pub fn new(lower: Vec2, upper: Vec2, gravity: Vec2, allow_sleep: bool) -> Self {
        let mut w = World {
            bp: BroadPhase::new(lower, upper),
            c: Core {
                bodies: Vec::new(),
                body_order: Vec::new(),
                shapes: Vec::new(),
                contacts: Vec::new(),
                contact_order: Vec::new(),
                joints: Vec::new(),
                joint_order: Vec::new(),
                events: Vec::new(),
                report_results: false,
                gravity,
                allow_sleep,
                inv_dt0: 0.0,
                locked: false,
            },
        };
        w.create_body(&BodyDef::default()); // ground body
        w
    }

    /// Debug helper: contacts touching `body` as text.
    pub fn debug_contacts(&self, body: BodyId) -> Vec<String> {
        let mut out = Vec::new();
        for &cid in self.c.bodies[body].contacts.iter().rev() {
            let c = self.c.contact(cid);
            let m = &c.manifold;
            let pts: Vec<String> = m.points[..m.point_count]
                .iter()
                .map(|p| format!("lp1=({:.6},{:.6}) sep={:.6} ni={:.6} ti={:.6} key={}", p.local_point1.x, p.local_point1.y, p.separation, p.normal_impulse, p.tangent_impulse, p.id.key()))
                .collect();
            out.push(format!("cnt={} n=({:.6},{:.6}) {}", c.manifold_count, m.normal.x, m.normal.y, pts.join(" | ")));
        }
        out
    }

    pub fn ground(&self) -> BodyId {
        0
    }

    pub fn create_body(&mut self, def: &BodyDef) -> BodyId {
        let mut flags = 0;
        if def.is_bullet {
            flags |= BULLET;
        }
        if def.fixed_rotation {
            flags |= FIXED_ROTATION;
        }
        if def.allow_sleep {
            flags |= ALLOW_SLEEP;
        }
        if def.is_sleeping {
            flags |= SLEEP;
        }
        let xf = XForm { position: def.position, r: Mat22::from_angle(def.angle) };
        let lc = def.mass_data.center;
        let mut c = Vec2::new(xf.r.col1.x * lc.x + xf.r.col2.x * lc.y, xf.r.col1.y * lc.x + xf.r.col2.y * lc.y);
        c.x += xf.position.x;
        c.y += xf.position.y;
        let mass = def.mass_data.mass;
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        let mut i = 0.0;
        if flags & FIXED_ROTATION == 0 {
            i = def.mass_data.i;
        }
        let inv_i = if i > 0.0 { 1.0 / i } else { 0.0 };
        let body = Body {
            xf,
            sweep: Sweep { local_center: lc, c0: c, c, a0: def.angle, a: def.angle, t0: 1.0 },
            linear_velocity: Vec2::default(),
            angular_velocity: 0.0,
            force: Vec2::default(),
            torque: 0.0,
            flags,
            mass,
            inv_mass,
            i,
            inv_i,
            linear_damping: def.linear_damping,
            angular_damping: def.angular_damping,
            sleep_time: 0.0,
            is_static: inv_mass == 0.0 && inv_i == 0.0,
            user_data: def.user_data,
            shapes: Vec::new(),
            contacts: Vec::new(),
            joints: Vec::new(),
            alive: true,
        };
        let id = self.c.bodies.len();
        self.c.bodies.push(body);
        self.c.body_order.push(id);
        id
    }

    pub fn create_shape(&mut self, body: BodyId, def: &ShapeDef) -> ShapeId {
        let mut shape = Shape::new(def);
        shape.update_sweep_radius(self.c.bodies[body].sweep.local_center);
        let id = self.c.shapes.len();
        let aabb = shape.compute_aabb(&self.c.bodies[body].xf);
        self.c.shapes.push(ShapeRec { shape, body, proxy: NULL_PROXY });
        self.c.bodies[body].shapes.push(id);
        if self.bp.in_range(&aabb) {
            let proxy = self.bp.create_proxy(&aabb, id, &mut |e| self.c.pair_event(e));
            self.c.shapes[id].proxy = proxy;
        }
        id
    }

    /// `b2World.CreateJoint`: links the joint into both bodies and refilters the lighter body's shapes
    /// when the bodies must not collide (this destroys/recreates proxies like the original).
    pub fn create_joint(&mut self, joint: Joint) -> usize {
        let (b1, b2, collide) = (joint.body1, joint.body2, joint.collide_connected);
        let id = self.c.joints.len();
        self.c.joints.push(joint);
        self.c.joint_order.push(id);
        self.c.bodies[b1].joints.push((id, b2, collide));
        self.c.bodies[b2].joints.push((id, b1, collide));
        if !collide {
            let n = if self.c.bodies[b1].shapes.len() < self.c.bodies[b2].shapes.len() { b1 } else { b2 };
            self.refilter_shapes(n);
        }
        id
    }

    /// `b2World.Query`: shapes whose broad-phase box overlaps `aabb` (at most `max`).
    pub fn query(&self, aabb: &Aabb, max: usize) -> Vec<ShapeId> {
        let mut v = self.bp.query_aabb(aabb);
        v.truncate(max);
        v
    }

    /// `b2World.DestroyShape`: removes the proxy (which destroys the shape's contacts) and unlinks the shape.
    /// Like the original it does not recompute the body's mass.
    pub fn destroy_shape(&mut self, shape: ShapeId) {
        let body = self.c.shapes[shape].body;
        if !self.c.bodies[body].shapes.contains(&shape) {
            return;
        }
        let proxy = self.c.shapes[shape].proxy;
        if proxy != NULL_PROXY {
            self.bp.destroy_proxy(proxy, &mut |e| self.c.pair_event(e));
            self.c.shapes[shape].proxy = NULL_PROXY;
        }
        self.c.bodies[body].shapes.retain(|&s| s != shape);
    }

    /// `b2World.DestroyJoint`.
    pub fn destroy_joint(&mut self, id: usize) {
        let (b1, b2, collide) = (self.c.joints[id].body1, self.c.joints[id].body2, self.c.joints[id].collide_connected);
        if !self.c.bodies[b1].joints.iter().any(|j| j.0 == id) {
            return;
        }
        self.c.bodies[b1].joints.retain(|j| j.0 != id);
        self.c.bodies[b2].joints.retain(|j| j.0 != id);
        self.c.joint_order.retain(|&j| j != id);
        self.c.bodies[b1].wake_up();
        self.c.bodies[b2].wake_up();
        if !collide {
            let n = if self.c.bodies[b1].shapes.len() < self.c.bodies[b2].shapes.len() { b1 } else { b2 };
            self.refilter_shapes(n);
        }
    }

    /// `b2World.DestroyBody`: destroys the body's joints and shapes and removes it from the world.
    pub fn destroy_body(&mut self, body: BodyId) {
        if !self.c.bodies[body].alive {
            return;
        }
        let joints: Vec<usize> = self.c.bodies[body].joints.iter().map(|j| j.0).collect();
        for j in joints {
            self.destroy_joint(j);
        }
        let shapes: Vec<ShapeId> = self.c.bodies[body].shapes.iter().rev().copied().collect();
        for s in shapes {
            self.destroy_shape(s);
        }
        self.c.bodies[body].alive = false;
        self.c.body_order.retain(|&b| b != body);
    }

    pub fn create_rope_joint(&mut self, def: &DistanceJointDef) -> usize {
        self.create_joint(Joint::new_rope(def))
    }

    pub fn create_distance_joint(&mut self, def: &DistanceJointDef) -> usize {
        self.create_joint(Joint::new_distance(def))
    }

    /// Gear joint coupling two revolute joints (ids from `create_revolute_joint`); `ratio` as in the def.
    pub fn create_gear_joint(&mut self, joint1: usize, joint2: usize, ratio: f64) -> Option<usize> {
        let g = Joint::new_gear(&self.c.joints[joint1], &self.c.joints[joint2], &self.c.bodies, ratio)?;
        Some(self.create_joint(g))
    }

    /// Flips the ratio of a gear joint (`gearJoint.m_ratio *= -1`; the constant stays as built).
    pub fn flip_gear_ratio(&mut self, id: usize) {
        if let JointKind::Gear(g) = &mut self.c.joints[id].kind {
            g.ratio = -g.ratio;
        }
    }

    pub fn create_revolute_joint(&mut self, def: &RevoluteJointDef) -> usize {
        self.create_joint(Joint::new_revolute(def))
    }

    pub fn create_prismatic_joint(&mut self, def: &PrismaticJointDef) -> usize {
        self.create_joint(Joint::new_prismatic(def))
    }

    pub fn shape_mut(&mut self, id: ShapeId) -> &mut Shape {
        &mut self.c.shapes[id].shape
    }

    fn refilter_shapes(&mut self, body: BodyId) {
        let shapes: Vec<ShapeId> = self.c.bodies[body].shapes.iter().rev().copied().collect();
        for s in shapes {
            self.refilter(s);
        }
    }

    /// `b2World.Refilter`: re-creates the shape's broad-phase proxy so its contacts are re-evaluated.
    pub fn refilter(&mut self, s: ShapeId) {
        if self.c.shapes[s].proxy != NULL_PROXY {
            let body = self.c.shapes[s].body;
            self.bp.destroy_proxy(self.c.shapes[s].proxy, &mut |e| self.c.pair_event(e));
            let aabb = self.c.shapes[s].shape.compute_aabb(&self.c.bodies[body].xf);
            self.c.shapes[s].proxy = if self.bp.in_range(&aabb) {
                self.bp.create_proxy(&aabb, s, &mut |e| self.c.pair_event(e))
            } else {
                NULL_PROXY
            };
        }
    }

    /// `SetFilterData` + `Refilter`.
    pub fn set_shape_filter(&mut self, s: ShapeId, f: crate::shape::Filter) {
        self.c.shapes[s].shape.filter = f;
        self.refilter(s);
    }

    /// Contacts of `body` with a non-empty manifold whose other shape is not a sensor.
    pub fn touching_non_sensor_contacts(&self, body: BodyId) -> usize {
        self.c.bodies[body]
            .contacts
            .iter()
            .filter(|&&cid| {
                let c = self.c.contact_info(cid);
                c.manifold_count > 0 && !self.c.shapes[c.shape1].shape.is_sensor && !self.c.shapes[c.shape2].shape.is_sensor
            })
            .count()
    }

    pub fn set_mass_from_shapes(&mut self, body: BodyId) {
        let b = &mut self.c.bodies[body];
        b.mass = 0.0;
        b.inv_mass = 0.0;
        b.i = 0.0;
        b.inv_i = 0.0;
        let (mut cx, mut cy) = (0.0, 0.0);
        let shapes: Vec<ShapeId> = b.shapes.iter().rev().copied().collect();
        for &s in &shapes {
            let md = self.c.shapes[s].shape.compute_mass();
            let b = &mut self.c.bodies[body];
            b.mass += md.mass;
            cx += md.mass * md.center.x;
            cy += md.mass * md.center.y;
            b.i += md.i;
        }
        let b = &mut self.c.bodies[body];
        if b.mass > 0.0 {
            b.inv_mass = 1.0 / b.mass;
            cx *= b.inv_mass;
            cy *= b.inv_mass;
        }
        if b.i > 0.0 && b.flags & FIXED_ROTATION == 0 {
            b.i -= b.mass * (cx * cx + cy * cy);
            b.inv_i = 1.0 / b.i;
        } else {
            b.i = 0.0;
            b.inv_i = 0.0;
        }
        b.sweep.local_center = Vec2::new(cx, cy);
        let r = b.xf.r;
        let lc = b.sweep.local_center;
        b.sweep.c.x = r.col1.x * lc.x + r.col2.x * lc.y;
        b.sweep.c.y = r.col1.y * lc.x + r.col2.y * lc.y;
        b.sweep.c.x += b.xf.position.x;
        b.sweep.c.y += b.xf.position.y;
        b.sweep.c0 = b.sweep.c;
        let was_static = b.is_static;
        b.is_static = b.inv_mass == 0.0 && b.inv_i == 0.0;
        let changed = was_static != b.is_static;
        for &s in &shapes {
            self.c.shapes[s].shape.update_sweep_radius(lc);
        }
        if changed {
            self.refilter_shapes(body);
        }
    }

    /// `b2Body.SetMass`: overrides the mass data (e.g. zero mass turns the body static).
    pub fn set_mass(&mut self, body: BodyId, md: &MassData) {
        let b = &mut self.c.bodies[body];
        b.mass = md.mass;
        b.inv_mass = if md.mass > 0.0 { 1.0 / md.mass } else { 0.0 };
        b.i = 0.0;
        b.inv_i = 0.0;
        if md.i > 0.0 && b.flags & FIXED_ROTATION == 0 {
            b.i = md.i - md.mass * (md.center.x * md.center.x + md.center.y * md.center.y);
            b.inv_i = 1.0 / b.i;
        }
        b.sweep.local_center = md.center;
        let r = b.xf.r;
        let lc = md.center;
        b.sweep.c.x = r.col1.x * lc.x + r.col2.x * lc.y + b.xf.position.x;
        b.sweep.c.y = r.col1.y * lc.x + r.col2.y * lc.y + b.xf.position.y;
        b.sweep.c0 = b.sweep.c;
        let was_static = b.is_static;
        b.is_static = b.inv_mass == 0.0 && b.inv_i == 0.0;
        let changed = was_static != b.is_static;
        let shapes: Vec<ShapeId> = b.shapes.clone();
        for s in shapes {
            self.c.shapes[s].shape.update_sweep_radius(lc);
        }
        if changed {
            self.refilter_shapes(body);
        }
    }

    /// `GetReactionForce` of a revolute joint (the pivot force) and the distance between its two anchors.
    pub fn revolute_reaction(&self, id: usize) -> Option<(f64, f64)> {
        let j = &self.c.joints[id];
        match &j.kind {
            JointKind::Revolute(r) => {
                let (b1, b2) = (&self.c.bodies[j.body1], &self.c.bodies[j.body2]);
                let (p1, p2) = (b1.world_point(r.local_anchor1), b2.world_point(r.local_anchor2));
                Some((r.pivot_force.x.hypot(r.pivot_force.y), (p2.x - p1.x).hypot(p2.y - p1.y)))
            }
            _ => None,
        }
    }

    /// `GetJointTranslation` of a prismatic joint.
    pub fn prismatic_translation(&self, id: usize) -> Option<f64> {
        let j = &self.c.joints[id];
        match &j.kind {
            JointKind::Prismatic(p) => {
                let (b1, b2) = (&self.c.bodies[j.body1], &self.c.bodies[j.body2]);
                let (a, c) = (b1.world_point(p.local_anchor1), b2.world_point(p.local_anchor2));
                let axis = box2d_mul(&b1.xf.r, p.local_x_axis1());
                Some(axis.x * (c.x - a.x) + axis.y * (c.y - a.y))
            }
            _ => None,
        }
    }

    /// `m_impulse` of a distance joint (the strap break test reads it).
    pub fn distance_impulse(&self, id: usize) -> Option<f64> {
        match &self.c.joints[id].kind {
            JointKind::Distance(d) => Some(d.impulse()),
            _ => None,
        }
    }

    /// `GetAnchor1` of a distance joint and its rest length.
    pub fn distance_anchor1(&self, id: usize) -> Option<(Vec2, f64)> {
        let j = &self.c.joints[id];
        match &j.kind {
            JointKind::Distance(d) => Some((self.c.bodies[j.body1].world_point(d.local_anchor1), d.length)),
            _ => None,
        }
    }

    /// Owning body of a shape.
    pub fn shape_body(&self, s: ShapeId) -> BodyId {
        self.c.shapes[s].body
    }

    /// `shape.ComputeAABB(body.m_xf)`.
    pub fn shape_aabb(&self, s: ShapeId) -> crate::collision::Aabb {
        let sh = &self.c.shapes[s];
        sh.shape.compute_aabb(&self.c.bodies[sh.body].xf)
    }

    /// `GetJointSpeed` of a prismatic joint.
    pub fn prismatic_speed(&self, id: usize) -> Option<f64> {
        let j = &self.c.joints[id];
        let JointKind::Prismatic(p) = &j.kind else { return None };
        let (t, n) = (&self.c.bodies[j.body1], &self.c.bodies[j.body2]);
        let e = &t.xf.r;
        let (r, i) = (p.local_anchor1.x - t.sweep.local_center.x, p.local_anchor1.y - t.sweep.local_center.y);
        let (r, i) = (e.col1.x * r + e.col2.x * i, e.col1.y * r + e.col2.y * i);
        let e = &n.xf.r;
        let (a, o) = (p.local_anchor2.x - n.sweep.local_center.x, p.local_anchor2.y - n.sweep.local_center.y);
        let (a, o) = (e.col1.x * a + e.col2.x * o, e.col1.y * a + e.col2.y * o);
        let (h, l) = (t.sweep.c.x + r, t.sweep.c.y + i);
        let (d, c) = (n.sweep.c.x + a - h, n.sweep.c.y + o - l);
        let u = box2d_mul(&t.xf.r, p.local_x_axis1());
        let (pv, m, f, w) = (t.linear_velocity, n.linear_velocity, t.angular_velocity, n.angular_velocity);
        Some(d * (-f * u.y) + c * (f * u.x) + (u.x * (m.x + -w * o - pv.x - -f * i) + u.y * (m.y + w * a - pv.y - f * r)))
    }

    /// `GetVertices()[0]` of a polygon shape (body-local).
    pub fn shape_first_vertex(&self, s: ShapeId) -> Option<Vec2> {
        match &self.c.shapes[s].shape.kind {
            crate::shape::ShapeKind::Polygon { poly, .. } => poly.vertices.first().copied(),
            _ => None,
        }
    }

    /// `m_radius` of a circle shape (0 for polygons).
    pub fn shape_radius(&self, s: ShapeId) -> f64 {
        match &self.c.shapes[s].shape.kind {
            crate::shape::ShapeKind::Circle(c) => c.radius,
            _ => 0.0,
        }
    }

    /// `GetMaterial()` bit mask of a shape.
    pub fn shape_material(&self, s: ShapeId) -> i32 {
        self.c.shapes[s].shape.material
    }

    pub fn joint(&self, id: usize) -> &Joint {
        &self.c.joints[id]
    }

    pub fn joint_mut(&mut self, id: usize) -> &mut Joint {
        &mut self.c.joints[id]
    }

    /// `GetJointAngle` of a revolute joint (`angle2 - angle1 - referenceAngle`).
    pub fn revolute_angle(&self, id: usize) -> Option<f64> {
        let j = &self.c.joints[id];
        match &j.kind {
            JointKind::Revolute(r) => Some(self.c.bodies[j.body2].sweep.a - self.c.bodies[j.body1].sweep.a - r.reference_angle),
            _ => None,
        }
    }

    pub fn body(&self, id: BodyId) -> &Body {
        &self.c.bodies[id]
    }
    pub fn body_mut(&mut self, id: BodyId) -> &mut Body {
        &mut self.c.bodies[id]
    }

    /// `b2Body.SetXForm`: teleports the body (velocities kept); false if it is frozen or leaves the world.
    pub fn set_xform(&mut self, id: BodyId, position: Vec2, angle: f64) -> bool {
        if self.c.locked {
            return true;
        }
        if self.c.bodies[id].flags & FROZEN != 0 {
            return false;
        }
        {
            let b = &mut self.c.bodies[id];
            b.xf.r.set_angle(angle);
            b.xf.position = position;
            let (r, lc) = (b.xf.r, b.sweep.local_center);
            b.sweep.c.x = r.col1.x * lc.x + r.col2.x * lc.y + position.x;
            b.sweep.c.y = r.col1.y * lc.x + r.col2.y * lc.y + position.y;
            b.sweep.c0 = b.sweep.c;
            b.sweep.a0 = angle;
            b.sweep.a = angle;
        }
        if !self.synchronize_shapes(id) {
            return false;
        }
        self.bp.commit(&mut |e| self.c.pair_event(e));
        true
    }

    /// `b2Body.SynchronizeShapes`; returns false if the body left the world (frozen).
    fn synchronize_shapes(&mut self, id: BodyId) -> bool {
        let b = &self.c.bodies[id];
        let mut xf1 = XForm::default();
        xf1.r.set_angle(b.sweep.a0);
        let (r, lc) = (xf1.r, b.sweep.local_center);
        xf1.position.x = b.sweep.c0.x - (r.col1.x * lc.x + r.col2.x * lc.y);
        xf1.position.y = b.sweep.c0.y - (r.col1.y * lc.x + r.col2.y * lc.y);
        let xf2 = b.xf;
        let shapes: Vec<ShapeId> = b.shapes.iter().rev().copied().collect();
        let mut ok = true;
        for s in shapes {
            let rec = &self.c.shapes[s];
            if rec.proxy == NULL_PROXY {
                ok = false;
                break;
            }
            let aabb = rec.shape.compute_swept_aabb(&xf1, &xf2);
            if !self.bp.in_range(&aabb) {
                ok = false;
                break;
            }
            self.bp.move_proxy(rec.proxy, &aabb);
        }
        if !ok {
            let b = &mut self.c.bodies[id];
            b.flags |= FROZEN;
            b.linear_velocity = Vec2::default();
            b.angular_velocity = 0.0;
            let shapes: Vec<ShapeId> = b.shapes.iter().rev().copied().collect();
            for s in shapes {
                if self.c.shapes[s].proxy != NULL_PROXY {
                    self.bp.destroy_proxy(self.c.shapes[s].proxy, &mut |e| self.c.pair_event(e));
                    self.c.shapes[s].proxy = NULL_PROXY;
                }
            }
        }
        ok
    }

    pub fn step(&mut self, dt: f64, iterations: usize) {
        self.c.locked = true;
        let step = StepParams {
            dt,
            inv_dt: if dt > 0.0 { 1.0 / dt } else { 0.0 },
            dt_ratio: self.c.inv_dt0 * dt,
            max_iterations: iterations,
            warm_starting: true,
            position_correction: true,
        };
        self.c.collide();
        if step.dt > 0.0 {
            self.solve(&step);
        }
        if step.dt > 0.0 {
            self.solve_toi(&step);
        }
        self.c.inv_dt0 = step.inv_dt;
        self.c.locked = false;
    }

    fn solve(&mut self, step: &StepParams) {
        let order: Vec<BodyId> = self.c.body_order.iter().rev().copied().collect();
        for &b in &order {
            self.c.bodies[b].flags &= !ISLAND;
        }
        let corder: Vec<ContactId> = self.c.contact_order.iter().rev().copied().collect();
        for &c in &corder {
            self.c.contacts[c].as_mut().unwrap().flags &= !C_ISLAND;
        }
        for j in self.c.joints.iter_mut() {
            j.island_flag = false;
        }
        for &seed in &order {
            let sb = &self.c.bodies[seed];
            if sb.flags & (ISLAND | SLEEP | FROZEN) != 0 || sb.is_static {
                continue;
            }
            let mut island_bodies: Vec<BodyId> = Vec::new();
            let mut island_contacts: Vec<ContactId> = Vec::new();
            let mut island_joints: Vec<usize> = Vec::new();
            let mut stack = vec![seed];
            self.c.bodies[seed].flags |= ISLAND;
            while let Some(bi) = stack.pop() {
                island_bodies.push(bi);
                self.c.bodies[bi].flags &= !SLEEP;
                if self.c.bodies[bi].is_static {
                    continue;
                }
                let cl: Vec<ContactId> = self.c.bodies[bi].contacts.iter().rev().copied().collect();
                for cid in cl {
                    let c = self.c.contact(cid);
                    if c.flags & (C_ISLAND | C_NONSOLID) != 0 || c.manifold_count == 0 {
                        continue;
                    }
                    island_contacts.push(cid);
                    self.c.contacts[cid].as_mut().unwrap().flags |= C_ISLAND;
                    let (b1, b2) = (self.c.shapes[self.c.contact(cid).shape1].body, self.c.shapes[self.c.contact(cid).shape2].body);
                    let other = if b1 == bi { b2 } else { b1 };
                    if self.c.bodies[other].flags & ISLAND == 0 {
                        stack.push(other);
                        self.c.bodies[other].flags |= ISLAND;
                    }
                }
                let jl: Vec<(usize, BodyId, bool)> = self.c.bodies[bi].joints.iter().rev().copied().collect();
                for (jid, other, _) in jl {
                    if self.c.joints[jid].island_flag {
                        continue;
                    }
                    island_joints.push(jid);
                    self.c.joints[jid].island_flag = true;
                    if self.c.bodies[other].flags & ISLAND == 0 {
                        stack.push(other);
                        self.c.bodies[other].flags |= ISLAND;
                    }
                }
            }
            self.solve_island(&island_bodies, &island_contacts, &island_joints, step);
            for &b in &island_bodies {
                if self.c.bodies[b].is_static {
                    self.c.bodies[b].flags &= !ISLAND;
                }
            }
        }
        for &b in &order {
            let body = &self.c.bodies[b];
            if body.flags & (SLEEP | FROZEN) != 0 || body.is_static {
                continue;
            }
            self.synchronize_shapes(b);
        }
        self.bp.commit(&mut |e| self.c.pair_event(e));
    }

    fn advance_body(&mut self, id: BodyId, t: f64) {
        let b = &mut self.c.bodies[id];
        b.sweep.advance(t);
        b.sweep.c = b.sweep.c0;
        b.sweep.a = b.sweep.a0;
        b.synchronize_transform();
    }

    /// `b2World.SolveTOI`: continuous collision (sub-stepping to the earliest time of impact).
    fn solve_toi(&mut self, step: &StepParams) {
        use crate::distance::time_of_impact;
        let order: Vec<BodyId> = self.c.body_order.iter().rev().copied().collect();
        for &b in &order {
            self.c.bodies[b].flags &= !ISLAND;
            self.c.bodies[b].sweep.t0 = 0.0;
        }
        let corder: Vec<ContactId> = self.c.contact_order.iter().rev().copied().collect();
        for &c in &corder {
            self.c.contacts[c].as_mut().unwrap().flags &= !(C_TOI | C_ISLAND);
        }
        let mut iteration = 0;
        loop {
            let mut min_contact: Option<ContactId> = None;
            let mut min_toi = 1.0;
            let ids: Vec<ContactId> = self.c.contact_order.iter().rev().copied().collect();
            for cid in ids {
                let c = self.c.contact(cid);
                if c.flags & (C_SLOW | C_NONSOLID) != 0 {
                    continue;
                }
                #[allow(unused_assignments)]
                let mut f = 1.0;
                if c.flags & C_TOI != 0 {
                    f = c.toi;
                } else {
                    let (s1, s2) = (c.shape1, c.shape2);
                    let (i, s) = (self.c.shapes[s1].body, self.c.shapes[s2].body);
                    let (bi, bs) = (&self.c.bodies[i], &self.c.bodies[s]);
                    if (bi.is_static || bi.is_sleeping()) && (bs.is_static || bs.is_sleeping()) {
                        continue;
                    }
                    let mut t0 = bi.sweep.t0;
                    if bi.sweep.t0 < bs.sweep.t0 {
                        t0 = bs.sweep.t0;
                        self.c.bodies[i].sweep.advance(t0);
                    } else if bs.sweep.t0 < bi.sweep.t0 {
                        t0 = bi.sweep.t0;
                        self.c.bodies[s].sweep.advance(t0);
                    }
                    f = time_of_impact(&self.c.shapes[s1].shape, &self.c.bodies[i].sweep, &self.c.shapes[s2].shape, &self.c.bodies[s].sweep);
                    if f > 0.0 && f < 1.0 {
                        f = (1.0 - f) * t0 + f;
                        if f > 1.0 {
                            f = 1.0;
                        }
                    }
                    let c = self.c.contacts[cid].as_mut().unwrap();
                    c.toi = f;
                    c.flags |= C_TOI;
                }
                if 5e-324 < f && f < min_toi {
                    min_contact = Some(cid);
                    min_toi = f;
                }
            }
            let Some(pc) = min_contact else { break };
            if 1.0 - 100.0 * 5e-324 < min_toi {
                break;
            }
            if iteration == MAX_TOI_ITERATIONS {
                break;
            }
            iteration += 1;
            let (s1, s2) = (self.c.contact(pc).shape1, self.c.contact(pc).shape2);
            let (i, s) = (self.c.shapes[s1].body, self.c.shapes[s2].body);
            self.advance_body(i, min_toi);
            self.advance_body(s, min_toi);
            self.c.update_contact(pc);
            self.c.contacts[pc].as_mut().unwrap().flags &= !C_TOI;
            if self.c.contact(pc).manifold_count == 0 {
                continue;
            }
            let mut seed = i;
            if self.c.bodies[seed].is_static {
                seed = s;
            }
            let mut island_bodies: Vec<BodyId> = Vec::new();
            let mut island_contacts: Vec<ContactId> = Vec::new();
            let mut stack = vec![seed];
            self.c.bodies[seed].flags |= ISLAND;
            while let Some(bi) = stack.pop() {
                island_bodies.push(bi);
                self.c.bodies[bi].flags &= !SLEEP;
                if self.c.bodies[bi].is_static {
                    continue;
                }
                let cl: Vec<ContactId> = self.c.bodies[bi].contacts.iter().rev().copied().collect();
                for cid in cl {
                    let c = self.c.contact(cid);
                    if island_contacts.len() != MAX_TOI_CONTACTS_PER_ISLAND && c.flags & (C_ISLAND | C_SLOW | C_NONSOLID) == 0 && c.manifold_count != 0 {
                        island_contacts.push(cid);
                        self.c.contacts[cid].as_mut().unwrap().flags |= C_ISLAND;
                        let (b1, b2) = (self.c.shapes[self.c.contact(cid).shape1].body, self.c.shapes[self.c.contact(cid).shape2].body);
                        let other = if b1 == bi { b2 } else { b1 };
                        if self.c.bodies[other].flags & ISLAND == 0 {
                            if !self.c.bodies[other].is_static {
                                self.advance_body(other, min_toi);
                                self.c.bodies[other].wake_up();
                            }
                            stack.push(other);
                            self.c.bodies[other].flags |= ISLAND;
                        }
                    }
                }
            }
            let dt = (1.0 - min_toi) * step.dt;
            let sub = StepParams { dt, inv_dt: 1.0 / dt, dt_ratio: 0.0, max_iterations: step.max_iterations, warm_starting: false, position_correction: true };
            self.solve_island_toi(&island_bodies, &island_contacts, &sub);
            for &bi in &island_bodies {
                self.c.bodies[bi].flags &= !ISLAND;
                let b = &self.c.bodies[bi];
                if b.flags & (SLEEP | FROZEN) != 0 || b.is_static {
                    continue;
                }
                self.synchronize_shapes(bi);
                let cl: Vec<ContactId> = self.c.bodies[bi].contacts.clone();
                for cid in cl {
                    self.c.contacts[cid].as_mut().unwrap().flags &= !C_TOI;
                }
            }
            for &cid in &island_contacts {
                self.c.contacts[cid].as_mut().unwrap().flags &= !(C_TOI | C_ISLAND);
            }
            self.bp.commit(&mut |e| self.c.pair_event(e));
        }
    }

    fn solve_island_toi(&mut self, bodies: &[BodyId], contacts: &[ContactId], step: &StepParams) {
        let mut cs = build_constraints(&self.c, contacts);
        for _ in 0..step.max_iterations {
            solve_velocity_constraints(&mut cs, &mut self.c.bodies);
        }
        for &bi in bodies {
            let s = &mut self.c.bodies[bi];
            if s.is_static {
                continue;
            }
            s.sweep.c0 = s.sweep.c;
            s.sweep.a0 = s.sweep.a;
            s.sweep.c.x += step.dt * s.linear_velocity.x;
            s.sweep.c.y += step.dt * s.linear_velocity.y;
            s.sweep.a += step.dt * s.angular_velocity;
            s.synchronize_transform();
        }
        for _ in 0..step.max_iterations {
            if solve_position_constraints(&mut cs, &mut self.c.bodies, 0.75) {
                break;
            }
        }
        self.report(&cs);
    }

    /// `b2Island.Report`: post-solve per-point impulses to the listener (as `Result` events).
    fn report(&mut self, cs: &[ContactConstraint]) {
        if !self.c.report_results {
            return;
        }
        for cc in cs {
            let (m, cid) = {
                let c = self.c.contacts[cc.contact].as_ref().unwrap();
                (c.manifold, cc.contact)
            };
            for (i, p) in m.points.iter().enumerate().take(m.point_count) {
                self.c.emit(ContactEventKind::Result, cid, &m, p);
                if let Some(e) = self.c.events.last_mut() {
                    e.normal_impulse = cc.points[i].normal_impulse;
                }
            }
        }
    }

    fn solve_island(&mut self, bodies: &[BodyId], contacts: &[ContactId], joints: &[usize], step: &StepParams) {
        let gravity = self.c.gravity;
        for &bi in bodies {
            let s = &mut self.c.bodies[bi];
            if s.is_static {
                continue;
            }
            s.linear_velocity.x += step.dt * (gravity.x + s.inv_mass * s.force.x);
            s.linear_velocity.y += step.dt * (gravity.y + s.inv_mass * s.force.y);
            s.angular_velocity += step.dt * s.inv_i * s.torque;
            s.force = Vec2::default();
            s.torque = 0.0;
            let lin = clamp(1.0 - step.dt * s.linear_damping, 0.0, 1.0);
            s.linear_velocity.x *= lin;
            s.linear_velocity.y *= lin;
            s.angular_velocity *= clamp(1.0 - step.dt * s.angular_damping, 0.0, 1.0);
            if s.linear_velocity.length_squared() > MAX_LINEAR_VELOCITY_SQUARED {
                s.linear_velocity.normalize();
                s.linear_velocity.x *= MAX_LINEAR_VELOCITY;
                s.linear_velocity.y *= MAX_LINEAR_VELOCITY;
            }
            if s.angular_velocity * s.angular_velocity > MAX_ANGULAR_VELOCITY_SQUARED {
                s.angular_velocity = if s.angular_velocity < 0.0 { -MAX_ANGULAR_VELOCITY } else { MAX_ANGULAR_VELOCITY };
            }
        }
        let mut cs = build_constraints(&self.c, contacts);
        init_velocity_constraints(&mut cs, &mut self.c.bodies, step);
        for &j in joints {
            self.c.joints[j].init_velocity_constraints(&mut self.c.bodies, step);
        }
        for _ in 0..step.max_iterations {
            solve_velocity_constraints(&mut cs, &mut self.c.bodies);
            for &j in joints {
                self.c.joints[j].solve_velocity_constraints(&mut self.c.bodies, step);
            }
        }
        // FinalizeVelocityConstraints: store accumulated impulses back in the manifolds
        for cc in &cs {
            let c = self.c.contacts[cc.contact].as_mut().unwrap();
            for (i, p) in cc.points.iter().enumerate() {
                c.manifold.points[i].normal_impulse = p.normal_impulse;
                c.manifold.points[i].tangent_impulse = p.tangent_impulse;
            }
        }
        for &bi in bodies {
            let s = &mut self.c.bodies[bi];
            if s.is_static {
                continue;
            }
            s.sweep.c0 = s.sweep.c;
            s.sweep.a0 = s.sweep.a;
            s.sweep.c.x += step.dt * s.linear_velocity.x;
            s.sweep.c.y += step.dt * s.linear_velocity.y;
            s.sweep.a += step.dt * s.angular_velocity;
            s.synchronize_transform();
        }
        if step.position_correction {
            for _ in 0..step.max_iterations {
                let contacts_ok = solve_position_constraints(&mut cs, &mut self.c.bodies, CONTACT_BAUMGARTE);
                let mut joints_ok = true;
                for &j in joints {
                    let ok = self.c.joints[j].solve_position_constraints(&mut self.c.bodies);
                    joints_ok = joints_ok && ok;
                }
                if contacts_ok && joints_ok {
                    break;
                }
            }
        }
        self.report(&cs);
        if self.c.allow_sleep {
            let mut min_sleep = f64::MAX;
            let lin_tol = LINEAR_SLEEP_TOLERANCE * LINEAR_SLEEP_TOLERANCE;
            let ang_tol = ANGULAR_SLEEP_TOLERANCE * ANGULAR_SLEEP_TOLERANCE;
            for &bi in bodies {
                let s = &mut self.c.bodies[bi];
                if s.inv_mass == 0.0 {
                    continue;
                }
                if s.flags & ALLOW_SLEEP == 0 {
                    s.sleep_time = 0.0;
                    min_sleep = 0.0;
                }
                if s.flags & ALLOW_SLEEP == 0 || s.angular_velocity * s.angular_velocity > ang_tol || dot(s.linear_velocity, s.linear_velocity) > lin_tol {
                    s.sleep_time = 0.0;
                    min_sleep = 0.0;
                } else {
                    s.sleep_time += step.dt;
                    min_sleep = min_sleep.min(s.sleep_time);
                }
            }
            if min_sleep >= TIME_TO_SLEEP {
                for &bi in bodies {
                    let s = &mut self.c.bodies[bi];
                    s.flags |= SLEEP;
                    s.linear_velocity = Vec2::default();
                    s.angular_velocity = 0.0;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> World {
        World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true)
    }

    #[test]
    fn free_fall_matches_semi_implicit_euler() {
        let mut w = world();
        let b = w.create_body(&BodyDef { position: Vec2::new(0.0, 0.0), ..Default::default() });
        let mut d = ShapeDef::circle(0.5);
        d.density = 1.0;
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        let dt = 1.0 / 30.0;
        for _ in 0..10 {
            w.step(dt, 10);
        }
        // v_n = n*g*dt, y_n = sum_{k=1..n} v_k * dt = g*dt²*n(n+1)/2
        let expect = 10.0 * dt * dt * 55.0;
        assert!((w.body(b).xf.position.y - expect).abs() < 1e-12, "{} vs {}", w.body(b).xf.position.y, expect);
    }

    #[test]
    fn box_rests_on_static_floor() {
        let mut w = world();
        let ground = w.ground();
        let mut f = ShapeDef::oriented_box(20.0, 1.0, Vec2::new(0.0, 5.0), 0.0);
        f.friction = 1.0;
        w.create_shape(ground, &f);
        let b = w.create_body(&BodyDef { position: Vec2::new(0.0, 2.0), ..Default::default() });
        let mut d = ShapeDef::boxed(0.5, 0.5);
        d.density = 1.0;
        d.friction = 0.3;
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        for _ in 0..180 {
            w.step(1.0 / 30.0, 10);
        }
        let y = w.body(b).xf.position.y;
        assert!((y - 3.5).abs() < 0.02, "box centre y = {y} (expected ~3.5)");
        assert!(w.body(b).angular_velocity.abs() < 1e-6);
    }

    #[test]
    fn ball_rests_on_floor_and_falls_asleep() {
        let mut w = world();
        let ground = w.ground();
        w.create_shape(ground, &ShapeDef::oriented_box(20.0, 1.0, Vec2::new(0.0, 5.0), 0.0));
        let b = w.create_body(&BodyDef { position: Vec2::new(0.0, 0.0), ..Default::default() });
        let mut d = ShapeDef::circle(0.5);
        d.density = 1.0;
        d.restitution = 0.1;
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        for _ in 0..300 {
            w.step(1.0 / 30.0, 10);
        }
        assert!((w.body(b).xf.position.y - 3.5).abs() < 0.02);
        assert!(w.body(b).is_sleeping());
    }
}

#[cfg(test)]
mod pose_tests {
    use super::*;

    #[test]
    fn blend_moves_dynamic_bodies_and_restore_is_exact() {
        let mut w = World::new(Vec2::new(-100.0, -100.0), Vec2::new(100.0, 100.0), Vec2::new(0.0, 10.0), true);
        let b = w.create_body(&BodyDef { position: Vec2::new(1.0, 2.0), ..Default::default() });
        let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 0.5 });
        d.density = 1.0;
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        let before = w.pose_snapshot();
        w.step(1.0 / 30.0, 10);
        let now = w.pose_snapshot();
        assert!(now[b].position.y > before[b].position.y);
        let saved = w.blend_poses(&before, 0.5);
        let mid = w.body(b).xf.position.y;
        assert!((mid - (before[b].position.y + now[b].position.y) / 2.0).abs() < 1e-12);
        w.restore_poses(&saved);
        assert_eq!(w.body(b).xf.position.y, now[b].position.y);
        let same = w.blend_poses(&now, 0.3);
        assert_eq!(w.body(b).xf.position.y, now[b].position.y);
        w.restore_poses(&same);
    }
}
