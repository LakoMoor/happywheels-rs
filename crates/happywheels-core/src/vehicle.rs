//! Bike vehicle character (class `iQ` of the original, e.g. guide `c3`): frame (3 polygons), two wheels and a
//! pedal gear on top of the base ragdoll, joined by free revolutes and a gear joint; keys drive the wheel
//! motors and apply lean impulses (`leftPressedActions` …). Not modelled: smash/break (`frameSmash`), sounds,
//! sprites.
use box2d::{
    joints::{DistanceJointDef, PrismaticJointDef, RevoluteJointDef},
    math::Vec2,
    shape::{Filter, Geometry, ShapeDef},
    world::{BodyDef, BodyId},
};

use crate::character::{Guide, Limb, Part, PoseSet, Ragdoll};
use crate::physics::World;

const CHARACTER_SCALE: f64 = 62.5 * 2.0;

/// Keys held this frame (`z` = eject).
#[derive(Debug, Clone, Copy, Default)]
pub struct Keys {
    pub shift: bool,
    pub ctrl: bool,
    pub left: bool,
    pub right: bool,
    pub up: bool,
    pub down: bool,
    pub space: bool,
    pub z: bool,
}

pub struct Bike {
    /// c10: a shoulder or hip break also lets go of that handlebar or pedal.
    pub roots_let_go: bool,
    pub frame: BodyId,
    pub back_wheel: BodyId,
    pub front_wheel: BodyId,
    pub gear: BodyId,
    back_joint: usize,
    front_joint: usize,
    frame_pelvis: Option<usize>,
    frame_hands: [Option<usize>; 2],
    gear_feet: [Option<usize>; 2],
    pub gear_joint: Option<usize>,
    pub ejected: bool,
    pub wheel_max_speed: f64,
    pub accel_step: f64,
    pub impulse_magnitude: f64,
    pub impulse_offset: f64,
    pub max_spin_av: f64,
    /// Pieces left by `frameSmash` (fork, broken frame, seat), and their world-space polygons from the guide.
    pub pieces: Vec<BodyId>,
    pieces_geo: Vec<Vec<Vec2>>,
    pub frame_smashed: bool,
    /// The c3 dad's child on the seat behind him (class `MY`); None for the bare bike (c10's dad has two other kids instead).
    pub kid: Option<Box<Kid>>,
}

impl Bike {
    pub const FRAME_SMASH_LIMIT: f64 = 200.0;
    /// Builds the bike around `doll` (already spawned from the same `guide` at `start_px`), in the original's
    /// creation order: frame (seat, frame, fork), back wheel, front wheel, gear; then joints.
    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let poly = |names: &[&str]| -> Option<Vec<Vec2>> { names.iter().map(|n| at(n)).collect() };
        let b = world.b2_mut();
        let deg = 180.0 / std::f64::consts::PI;
        let shape = |geometry: Geometry, density: f64, friction: f64, restitution: f64, filter: Filter| {
            let mut d = ShapeDef::new(geometry);
            d.density = density;
            d.friction = friction;
            d.restitution = restitution;
            d.filter = filter;
            d
        };
        let seat_filter = Filter { category_bits: 513, mask_bits: 0xFFFF, group_index: 0 };
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let frame = b.create_body(&BodyDef::default());
        for (verts, filter) in [
            (poly(&["seatVert1", "seatVert2", "seatVert3"])?, seat_filter),
            (poly(&["frameVert1", "frameVert2", "frameVert3", "frameVert4"])?, zero),
            (poly(&["forkVert1", "forkVert2", "forkVert3"])?, zero),
        ] {
            b.create_shape(frame, &shape(Geometry::Polygon(verts), 2.0, 0.3, 0.1, filter));
        }
        b.set_mass_from_shapes(frame);
        let mut wheel = |name: &str, rotation_of: &str| -> Option<BodyId> {
            let g = guide.get(rotation_of)?;
            let pos = at(name)?;
            let body = b.create_body(&BodyDef { position: pos, angle: g[2] / deg, ..Default::default() });
            b.create_shape(body, &shape(Geometry::Circle { local_position: Vec2::default(), radius: g[5] / 2.0 / CHARACTER_SCALE }, 5.0, 1.0, 0.3, seat_filter));
            b.set_mass_from_shapes(body);
            Some(body)
        };
        let back_wheel = wheel("backWheelShape", "backWheelShape")?;
        let front_wheel = wheel("frontWheelShape", "frontWheelShape")?;
        let gear = wheel("gearShape", "gearShape")?;
        for part in [Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2] {
            let body = doll.bodies[&part];
            let s = b.body(body).shapes[0];
            b.shape_mut(s).is_sensor = true;
            // VERIFIED against the game's real session (analysis/golden-session-sounds.js): all four leg/frame contacts are non-solid there
            b.c.make_contacts_non_solid(s);
        }
        doll.finish_joints(world);
        let b = world.b2_mut();
        // Joints: lean-friendly limits first (`SetLimits` after the base joints), then the vehicle joints.
        let limits = [
            (Limb::Hip1, -110.0, 10.0),
            (Limb::Hip2, -110.0, 10.0),
            (Limb::Elbow1, -60.0, 0.0),
            (Limb::Elbow2, -60.0, 0.0),
            (Limb::Neck, -20.0, 0.0),
        ];
        for (limb, lo, hi) in limits {
            let id = doll.joints[&limb];
            let reference = match &b.joint(id).kind {
                box2d::joints::JointKind::Revolute(r) => r.reference_angle,
                _ => 0.0,
            };
            if let box2d::joints::JointKind::Revolute(r) = &mut b.joint_mut(id).kind {
                r.lower_angle = lo / deg - reference;
                r.upper_angle = hi / deg - reference;
            }
        }
        let revolute = |b1: BodyId, b2: BodyId, p: Vec2, b: &mut box2d::world::World| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 20.0;
            b.create_revolute_joint(&d)
        };
        let back_joint = revolute(frame, back_wheel, at("backWheelShape")?, b);
        let front_joint = revolute(frame, front_wheel, at("frontWheelShape")?, b);
        let pelvis = doll.bodies[&Part::Pelvis];
        let pelvis_pos = b.body(pelvis).xf.position;
        let frame_pelvis = revolute(frame, pelvis, pelvis_pos, b);
        let hand = at("handleAnchor")?;
        let frame_hand1 = revolute(frame, doll.bodies[&Part::LowerArm1], hand, b);
        let frame_hand2 = revolute(frame, doll.bodies[&Part::LowerArm2], hand, b);
        let frame_gear = revolute(frame, gear, at("gearShape")?, b);
        let gear_joint = b.create_gear_joint(back_joint, frame_gear, 1.0)?;
        b.flip_gear_ratio(gear_joint);
        let foot1 = revolute(gear, doll.bodies[&Part::LowerLeg1], at("gearAnchor1")?, b);
        let foot2 = revolute(gear, doll.bodies[&Part::LowerLeg2], at("gearAnchor2")?, b);
        Some(Self {
            frame,
            back_wheel,
            front_wheel,
            gear,
            back_joint,
            front_joint,
            frame_pelvis: Some(frame_pelvis),
            frame_hands: [Some(frame_hand1), Some(frame_hand2)],
            gear_feet: [Some(foot1), Some(foot2)],
            gear_joint: Some(gear_joint),
            ejected: false,
            wheel_max_speed: 20.0,
            accel_step: 1.0,
            impulse_magnitude: 3.0,
            impulse_offset: 1.0,
            max_spin_av: 5.0,
            pieces: Vec::new(),
            pieces_geo: vec![poly(&["forkVert1", "forkVert2", "forkVert3"])?, poly(&["brokenVert1", "brokenVert2", "brokenVert3", "brokenVert4"])?, poly(&["seatVert1", "seatVert2", "seatVert3"])?],
            roots_let_go: false,
            frame_smashed: false,
            kid: None,
        })
    }

    /// Number of non-sensor contacts the two wheels currently have (`wheelContacts`).
    pub fn wheel_contacts(&self, world: &World) -> usize {
        let b = world.b2();
        [self.back_wheel, self.front_wheel].iter().map(|&w| b.touching_non_sensor_contacts(w)).sum()
    }

    fn set_motors(&self, world: &mut World, enabled: bool, speed: Option<f64>) {
        for id in [self.back_joint, self.front_joint] {
            if let box2d::joints::JointKind::Revolute(r) = &mut world.b2_mut().joint_mut(id).kind {
                r.enable_motor = enabled;
                if let Some(s) = speed {
                    r.motor_speed = s;
                }
            }
        }
    }

    fn motor_enabled(&self, world: &World) -> bool {
        matches!(&world.b2().joint(self.back_joint).kind, box2d::joints::JointKind::Revolute(r) if r.enable_motor)
    }

    /// `GetJointSpeed` of the back wheel (w2 - w1).
    fn wheel_speed(&self, world: &World) -> f64 {
        let b = world.b2();
        b.body(self.back_wheel).angular_velocity - b.body(self.frame).angular_velocity
    }

    /// One frame of key handling for the seated rider (`checkKeyStates`, not ejected):
    /// left/right lean impulses + poses, up/down wheel motors, space brake, `z` eject.
    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        if let Some(mut kid) = self.kid.take() {
            // the dad's `shiftPressedActions` ejects him (original 21559); every key is forwarded to the kid afterwards
            if k.shift && !doll.dead {
                self.eject(world, doll);
            }
            self.keys_inner(world, doll, Keys { shift: false, ..k });
            kid.keys(world, k);
            self.kid = Some(kid);
        } else {
            self.keys_inner(world, doll, k);
        }
    }

    fn keys_inner(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        // `checkKeyStates` of a dead character runs none of the key actions
        if doll.dead {
            doll.apply_pose(world);
            return;
        }
        let Keys { left, right, up, down, space, z, .. } = k;
        if self.ejected {
            doll.keys(left, right, up, down);
            // `upAndDownActions` switches the wheel motors off even when ejected (it is the "neither up nor down" branch)
            if up == down && self.motor_enabled(world) {
                self.set_motors(world, false, None);
            }
            doll.apply_pose(world);
            doll.set_grab(world, space);
            return;
        }
        match (left, right) {
            (true, false) => self.lean(world, doll, true),
            (false, true) => self.lean(world, doll, false),
            _ => {}
        }
        match (up, down) {
            (true, false) => self.accelerate(world, 1.0),
            (false, true) => self.accelerate(world, -1.0),
            _ => {
                if self.motor_enabled(world) {
                    self.set_motors(world, false, None);
                }
            }
        }
        if space {
            if !self.motor_enabled(world) {
                self.set_motors(world, true, None);
            }
            self.set_motors(world, true, Some(0.0));
        }
        // `zPressedActions` is the last key handler of `checkKeyStates`: the actions above still ran this frame
        if z {
            self.eject(world, doll);
        }
    }

    fn accelerate(&self, world: &mut World, dir: f64) {
        if !self.motor_enabled(world) {
            self.set_motors(world, true, None);
        }
        let cur = self.wheel_speed(world);
        let new = if cur * dir < 0.0 {
            0.0
        } else if cur * dir < self.wheel_max_speed {
            cur + dir * self.accel_step
        } else {
            cur
        };
        self.set_motors(world, true, Some(new));
    }

    fn lean(&self, world: &mut World, doll: &Ragdoll, back: bool) {
        let b = world.b2_mut();
        let (angle, av) = (b.body(self.frame).sweep.a, b.body(self.frame).angular_velocity);
        let s = self.max_spin_av;
        let q = if back { (av + s) / s } else { (av - s) / -s }.clamp(0.0, 1.0);
        let (a, w) = (box2d::fdlibm::cos(angle) * self.impulse_magnitude * q, box2d::fdlibm::sin(angle) * self.impulse_magnitude * q);
        let c = b.body(self.frame).sweep.local_center;
        let off = if back { self.impulse_offset } else { -self.impulse_offset };
        let p = b.body(self.frame).world_point(Vec2::new(c.x + off, c.y));
        b.body_mut(self.frame).apply_impulse(Vec2::new(w, -a), p);
        let table: [(Limb, f64, f64); 3] = if back {
            [(Limb::Neck, 0.0, 2.0), (Limb::Elbow1, 1.04, 15.0), (Limb::Elbow2, 1.04, 15.0)]
        } else {
            [(Limb::Neck, 1.0, 1.0), (Limb::Elbow1, 0.0, 15.0), (Limb::Elbow2, 0.0, 15.0)]
        };
        doll.set_joints(world, &table);
    }

    /// The bike's overrides of the rider's smash/break callbacks: head/chest/pelvis smash and neck/waist breaks eject;
    /// an elbow or knee break drops that hand/foot joint and ejects once all four are gone (`checkEject`).
    pub fn rider_hurt(&mut self, world: &mut World, doll: &mut Ragdoll, smashed: bool, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        if smashed || broken.iter().any(|l| matches!(l, Limb::Neck | Limb::Waist)) {
            self.eject(world, doll);
            return;
        }
        let b = world.b2_mut();
        for l in broken {
            let slot = match l {
                Limb::Elbow1 => &mut self.frame_hands[0],
                Limb::Elbow2 => &mut self.frame_hands[1],
                Limb::Knee1 => &mut self.gear_feet[0],
                Limb::Knee2 => &mut self.gear_feet[1],
                // only the mom's class (22139) overrides these
                Limb::Shoulder1 if self.roots_let_go => &mut self.frame_hands[0],
                Limb::Shoulder2 if self.roots_let_go => &mut self.frame_hands[1],
                Limb::Hip1 if self.roots_let_go => &mut self.gear_feet[0],
                Limb::Hip2 if self.roots_let_go => &mut self.gear_feet[1],
                _ => continue,
            };
            if let Some(j) = slot.take() {
                b.destroy_joint(j);
            }
        }
        if !broken.is_empty() && self.frame_hands.iter().chain(&self.gear_feet).all(Option::is_none) {
            self.eject(world, doll);
        }
    }

    /// `frontWheelSmash` / `backWheelSmash`: the wheel's circle is replaced by a 25×50 px box turned to the contact normal
    /// (density 2; group-free filter if ejected, else category 513). The body's mass is NOT recomputed (as in the original).
    pub fn wheel_smash(&mut self, world: &mut World, front: bool, normal: Vec2) {
        {
            let at = world.b2().body(if front { self.front_wheel } else { self.back_wheel }).sweep.c;
            world.play("BikeTireSmash1", 1, at);
        }
        let wheel = if front { self.front_wheel } else { self.back_wheel };
        let b = world.b2_mut();
        let Some(&old) = b.body(wheel).shapes.first() else { return };
        b.destroy_shape(old);
        let angle = normal.y.atan2(normal.x) - b.body(wheel).sweep.a;
        let mut d = ShapeDef::new(Geometry::Polygon(oriented_box(25.0 / CHARACTER_SCALE, 50.0 / CHARACTER_SCALE, Vec2::default(), angle)));
        d.density = 2.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = if self.ejected { Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 } } else { Filter { category_bits: 513, ..Filter::default() } };
        b.create_shape(wheel, &d);
    }

    /// `frameSmash`: eject, drop wheel and gear joints, and replace the frame by three loose bodies (fork, broken frame,
    /// seat; density 2, zeroFilter) carrying its velocity.
    pub fn frame_smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.frame_smashed {
            return;
        }
        self.frame_smashed = true;
        {
            let at = world.b2().body(self.frame).sweep.c;
            world.play("BikeSmash1", 1, at);
        }
        self.eject(world, doll);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        for j in [Some(self.front_joint), Some(self.back_joint), self.gear_joint.take()].into_iter().flatten() {
            b.destroy_joint(j);
        }
        let (pos, ang) = (b.body(self.frame).xf.position, b.body(self.frame).sweep.a);
        let (v, w) = (b.body(self.frame).linear_velocity, b.body(self.frame).angular_velocity);
        for geo in self.pieces_geo.clone() {
            let body = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Polygon(geo));
            d.density = 2.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            b.body_mut(body).linear_velocity = v;
            b.body_mut(body).angular_velocity = w;
            self.pieces.push(body);
        }
        b.destroy_body(self.frame);
    }

    /// `eject`: free the rider (destroy frame–pelvis/hands and gear–feet joints, reset limits, make wheels and
    /// frame shapes collide with the rider group-free, un-sensor the legs).
    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        doll.reset_joint_limits(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        for j in [self.frame_pelvis.take(), self.frame_hands[0].take(), self.frame_hands[1].take(), self.gear_feet[0].take(), self.gear_feet[1].take()].into_iter().flatten() {
            b.destroy_joint(j);
        }
        let wheels = [self.front_wheel, self.back_wheel].map(|w| (b.body(w).shapes[0], zero));
        b.set_shape_filters(&wheels);
        let shapes: Vec<_> = b.body(self.frame).shapes.iter().rev().copied().collect(); // shape list is newest-first
        for s in shapes {
            b.set_shape_filter(s, zero);
        }
        for part in [Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2] {
            let s = b.body(doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = false;
            b.refilter(s);
        }
    }
}

/// Which vehicle (if any) the rider sits on.
#[allow(clippy::large_enum_variant)]
pub enum Vehicle {
    Bike(Bike),
    Segway(Segway),
    Chair(Chair),
    Dirt(DirtBike),
    Mower(Mower),
    Pogo(Pogo),
    Cart(Cart),
    Heli(Heli),
    Grinder(Grinder),
    Family(Box<Family>),
    Santa(Box<Santa>),
}

/// Both coordinates of `a` within `reach` of `b`.
fn near(a: Vec2, b: Vec2, reach: f64) -> bool {
    (a.x - b.x).abs() < reach && (a.y - b.y).abs() < reach
}

/// A polyline the original strokes with `graphics` (ropes, shocks, harness): world px, drawn in the layer of the clip `layer`
/// (below it, or above it when `above`).
#[derive(Debug, Clone)]
pub struct Stroke {
    pub points: Vec<[f64; 2]>,
    pub width: f64,
    pub color: u32,
    pub layer: std::borrow::Cow<'static, str>,
    pub above: bool,
}

const PX_PER_M: f64 = 62.5;

fn px(p: Vec2) -> [f64; 2] {
    [p.x * PX_PER_M, p.y * PX_PER_M]
}

/// `GetAnchor1` / `GetAnchor2` of a joint (also of a destroyed one, which the original keeps drawing).
pub(crate) fn joint_anchor(world: &World, id: usize, second: bool) -> Option<Vec2> {
    use box2d::joints::JointKind;
    let j = world.b2().joint(id);
    let (a, b) = match &j.kind {
        JointKind::Revolute(k) => (k.local_anchor1, k.local_anchor2),
        JointKind::Prismatic(k) => (k.local_anchor1, k.local_anchor2),
        JointKind::Distance(k) => (k.local_anchor1, k.local_anchor2),
        JointKind::Rope(k) => (k.local_anchor1, k.local_anchor2),
        JointKind::Gear(_) => return None,
    };
    Some(world.b2().body(if second { j.body2 } else { j.body1 }).world_point(if second { b } else { a }))
}

/// A shock absorber: from the joint's first anchor (or a body centre) to the centre of the sliding body.
fn shock_line(world: &World, from: Option<Vec2>, to: BodyId, width: f64, color: u32, layer: &'static str) -> Option<Stroke> {
    let end = world.b2().body(to);
    if !end.alive {
        return None;
    }
    Some(Stroke { points: vec![px(from?), px(end.sweep.c)], width, color, layer: layer.into(), above: false })
}

impl Vehicle {
    /// What `paint` strokes besides the clips.
    pub fn strokes(&self, world: &World) -> Vec<Stroke> {
        let center = |id: BodyId| Some(world.b2().body(id)).filter(|b| b.alive).map(|b| b.sweep.c);
        let anchor = |id: usize| joint_anchor(world, id, false);
        match self {
            Vehicle::Segway(s) if !s.frame_smashed => shock_line(world, center(s.stand), s.shock, 3.0, 0xd1d8e0, "wheel").into_iter().collect(),
            Vehicle::Cart(c) if !c.main_smashed => [(c.back_shock_joint, c.back_shock), (c.front_shock_joint, c.front_shock)]
                .into_iter()
                .filter_map(|(j, body)| shock_line(world, anchor(j), body, 3.0, 0x0c160d, "backWheel"))
                .collect(),
            Vehicle::Mower(m) => [(m.front_shock_joint, m.front_shock), (m.back_shock_joint, m.back_shock)]
                .into_iter()
                .filter_map(|(j, body)| shock_line(world, anchor(j), body, 3.0, 0x171717, "mower"))
                .collect(),
            Vehicle::Heli(h) => vec![Stroke { points: h.rope_points.iter().map(|p| px(p.0)).collect(), width: 1.0, color: 0x333333, layer: "copter".into(), above: false }],
            Vehicle::Santa(s) => [(&s.strap_head, 1.0), (&s.strap_chest1, 1.5), (&s.strap_chest2, 1.5)]
                .into_iter()
                .filter_map(|(chain, width)| {
                    let mut points = vec![px(joint_anchor(world, *chain.first()?, false)?)];
                    points.extend(chain.iter().filter_map(|&j| joint_anchor(world, j, true)).map(px));
                    Some(Stroke { points, width, color: 0x333333, layer: "sleigh".into(), above: true })
                })
                .collect(),
            _ => Vec::new(),
        }
    }

    pub fn ejected(&self) -> bool {
        match self {
            Vehicle::Bike(b) => b.ejected,
            Vehicle::Segway(s) => s.ejected,
            Vehicle::Chair(c) => c.ejected,
            Vehicle::Dirt(d) => d.ejected,
            Vehicle::Mower(m) => m.ejected,
            Vehicle::Pogo(m) => m.ejected,
            Vehicle::Cart(m) => m.ejected,
            Vehicle::Heli(m) => m.ejected,
            Vehicle::Grinder(m) => m.ejected,
            Vehicle::Family(m) => m.dad.ejected,
            Vehicle::Santa(m) => m.ejected,
        }
    }

    /// Whether `headSmash1` is overridden to call `eject()` (all but the chair and the helicopter; `chestSmash` and
    /// `pelvisSmash` eject everywhere).
    pub fn head_smash_ejects(&self) -> bool {
        !matches!(self, Vehicle::Chair(_) | Vehicle::Heli(_))
    }

    /// Breaks a passenger's joint as an overload would and runs its reaction; `who` is the original's field name
    /// (`kid`, `girl`, `daughter`, `son`, `elf1`, `elf2`). Returns whether they are off the vehicle now.
    pub fn break_passenger_limb(&mut self, world: &mut World, who: &str, limb: Limb) -> Option<bool> {
        macro_rules! hurt {
            ($p:expr) => {{
                let p = $p;
                let force = p.doll.break_limit(world, limb) + 1.0;
                p.doll.break_limb(world, limb, force);
                p.limbs_broken(world, &[limb]);
                p.ejected
            }};
        }
        Some(match (self, who) {
            (Vehicle::Bike(b), "kid") => hurt!(b.kid.as_mut()?),
            (Vehicle::Dirt(d), "girl") => hurt!(d.girl.as_mut()?),
            (Vehicle::Family(f), "daughter") => hurt!(&mut f.daughter),
            (Vehicle::Family(f), "son") => hurt!(&mut f.son),
            (Vehicle::Santa(s), "elf1") => hurt!(&mut s.elf1),
            (Vehicle::Santa(s), "elf2") => hurt!(&mut s.elf2),
            _ => return None,
        })
    }

    /// `shapeImpale` on a passenger, who has the rider's own: a lethal head impale throws them off and kills them. Returns
    /// whether `body` was a passenger's.
    pub fn impale_passenger(&mut self, world: &mut World, body: BodyId, fatal: bool, point: Option<Vec2>, radius2: f64) -> bool {
        macro_rules! impale {
            ($p:expr) => {{
                let p = $p;
                if p.doll.bodies.values().any(|&b| b == body) {
                    if p.doll.shape_impale(world, body, fatal, point, radius2) {
                        p.eject(world);
                    }
                    return true;
                }
            }};
        }
        match self {
            Vehicle::Bike(b) => {
                if let Some(kid) = b.kid.as_mut() {
                    impale!(kid);
                }
            }
            Vehicle::Dirt(d) => {
                if let Some(girl) = d.girl.as_mut() {
                    impale!(girl);
                }
            }
            Vehicle::Family(f) => {
                impale!(&mut f.daughter);
                impale!(&mut f.son);
            }
            Vehicle::Santa(s) => {
                impale!(&mut s.elf1);
                impale!(&mut s.elf2);
            }
            _ => {}
        }
        false
    }

    /// A smash (`smashed`) or the joints broken this frame: eject, or the vehicle's limb-break reaction.
    pub fn rider_hurt(&mut self, world: &mut World, doll: &mut Ragdoll, smashed: bool, broken: &[Limb]) {
        // `neckBreak` / `torsoBreak` overrides that call `eject()`: the chair, the helicopter and the moped keep the rider on a broken neck
        let neck = !matches!(self, Vehicle::Chair(_) | Vehicle::Heli(_) | Vehicle::Dirt(_));
        let torso = broken.iter().any(|&l| l == Limb::Waist || l == Limb::Neck && neck);
        match self {
            Vehicle::Bike(b) => b.rider_hurt(world, doll, smashed, broken),
            Vehicle::Family(f) => f.dad.rider_hurt(world, doll, smashed, broken),
            Vehicle::Mower(c) if !smashed && !torso => c.limbs_broken(world, doll, broken),
            Vehicle::Heli(c) if !smashed && !torso => c.limbs_broken(world, doll, broken),
            Vehicle::Pogo(c) if !smashed && !torso => c.limbs_broken(world, doll, broken),
            Vehicle::Cart(c) if !smashed && !torso => c.limbs_broken(world, doll, broken),
            Vehicle::Santa(c) if !smashed && !torso => c.limbs_broken(world, doll, broken),
            Vehicle::Segway(c) if !smashed && !torso => c.limbs_broken(world, doll, broken),
            Vehicle::Dirt(c) if !smashed && !torso => c.limbs_broken(world, doll, broken),
            Vehicle::Chair(c) if !smashed && !torso => c.limbs_broken(world, broken),
            Vehicle::Grinder(c) if !smashed && !torso => c.limbs_broken(world, doll, broken),
            _ => {
                if smashed || torso {
                    match self {
                        Vehicle::Segway(v) => v.eject(world, doll),
                        Vehicle::Chair(v) => v.eject(world, doll),
                        Vehicle::Dirt(v) => v.eject(world, doll),
                        Vehicle::Mower(v) => v.eject(world, doll),
                        Vehicle::Pogo(v) => v.eject(world, doll),
                        Vehicle::Cart(v) => v.eject(world, doll),
                        Vehicle::Heli(v) => v.eject(world, doll),
                        Vehicle::Grinder(v) => v.eject(world, doll),
                        Vehicle::Santa(v) => v.eject(world, doll),
                        _ => {}
                    }
                }
            }
        }
    }

    /// The vehicle part of `grabAction`: hand `k` of an ejected rider, its tip at `tip`, takes hold of the vehicle again when it is within
    /// 0.25 of the handle (the Segway, the shopping cart, the moped and the pogo stick; the others cannot be boarded again).
    pub fn board(&mut self, world: &mut World, doll: &mut Ragdoll, k: usize, tip: Vec2) -> bool {
        match self {
            Vehicle::Segway(v) => v.board(world, doll, k, tip),
            Vehicle::Cart(v) => v.board(world, doll, k, tip),
            Vehicle::Dirt(v) => v.board(world, doll, k, tip),
            Vehicle::Pogo(v) => v.board(world, doll, k, tip),
            _ => false,
        }
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        match self {
            Vehicle::Bike(b) => b.keys(world, doll, k),
            Vehicle::Segway(s) => s.keys(world, doll, k),
            Vehicle::Chair(c) => c.keys(world, doll, k),
            Vehicle::Dirt(d) => d.keys(world, doll, k),
            Vehicle::Mower(m) => m.keys(world, doll, k),
            Vehicle::Pogo(m) => m.keys(world, doll, k),
            Vehicle::Cart(m) => m.keys(world, doll, k),
            Vehicle::Heli(m) => m.keys(world, doll, k),
            Vehicle::Grinder(m) => m.keys(world, doll, k),
            Vehicle::Family(m) => m.keys(world, doll, k),
            Vehicle::Santa(m) => m.keys(world, doll, k),
        }
    }

    /// Bodies to draw.
    pub fn bodies(&self) -> Vec<BodyId> {
        match self {
            Vehicle::Bike(b) => {
                let mut v = [vec![b.frame, b.back_wheel, b.front_wheel, b.gear], b.pieces.clone()].concat();
                if let Some(k) = &b.kid {
                    v.extend(k.doll.bodies.values());
                    v.push(k.seat);
                }
                v
            }
            Vehicle::Segway(s) => vec![s.frame, s.stand, s.shock, s.wheel],
            Vehicle::Chair(c) => [vec![c.chair, c.big_wheel, c.small_wheel], c.debris.clone()].concat(),
            Vehicle::Dirt(d) => {
                let mut v = [vec![d.frame, d.back_wheel, d.front_wheel], d.pieces.clone()].concat();
                if let Some(g) = &d.girl {
                    v.extend(g.doll.bodies.values());
                }
                v
            }
            Vehicle::Pogo(m) => if m.frame_smashed { [vec![m.rod], m.pieces.clone()].concat() } else { vec![m.frame, m.rod] },
            Vehicle::Heli(m) => if m.copter_smashed { [vec![m.magnet], m.debris.clone()].concat() } else { [vec![m.copter, m.magnet], m.debris.clone()].concat() },
            Vehicle::Grinder(m) => vec![m.frame, m.back_wheel, m.front_wheel],
            Vehicle::Family(m) => m.bodies(),
            Vehicle::Santa(m) => m.bodies(&[]),
            Vehicle::Cart(m) => {
                let mut v = if m.main_smashed { vec![m.shaft, m.back_wheel, m.front_wheel] } else { vec![m.body, m.shaft, m.back_wheel, m.front_wheel, m.back_shock, m.front_shock] };
                v.extend(&m.pieces);
                v.extend(&m.groceries);
                v
            }
            Vehicle::Mower(m) => match m.halves {
                Some(h) => vec![m.back_wheel, m.front_wheel, h[0], h[1]],
                None => vec![m.body, m.back_wheel, m.front_wheel, m.back_shock, m.front_shock],
            },
        }
    }

    pub fn as_bike(&self) -> Option<&Bike> {
        match self {
            Vehicle::Bike(b) => Some(b),
            Vehicle::Family(f) => Some(&f.dad),
            _ => None,
        }
    }
}

/// Segway (class `i2`, guide `c2`): one wheel on a telescoping (prismatic) shock under a stand body, a frame
/// with the handlebar leaning on the stand (±15°), hands/feet on the frame. Space jumps (shock motor), shift
/// stretches the legs / squats, ctrl makes fists; up/down drive the wheel, left/right lean the frame.
pub struct Segway {
    pub frame: BodyId,
    pub stand: BodyId,
    pub shock: BodyId,
    pub wheel: BodyId,
    shock_joint: usize,
    wheel_joint: usize,
    stand_frame: usize,
    pub frame_smashed: bool,
    frame_hands: [Option<usize>; 2],
    stand_feet: [Option<usize>; 2],
    pub ejected: bool,
    pub wheel_max_speed: f64,
    pub accel_step: f64,
    jump_translation: f64,
    /// Joint definitions / frame-local anchors used by `reAttach` (hands then feet) and the `reAttaching` action flag.
    hand_defs: [RevoluteJointDef; 2],
    foot_defs: [RevoluteJointDef; 2],
    hand_anchor: Vec2,
    foot_anchor: Vec2,
    reattaching: bool,
}

impl Segway {
    pub const FRAME_SMASH_LIMIT: f64 = 20.0;

    /// `frameSmash` (original 30878): the stand–frame joint is destroyed, the rider ejected, and the frame and wheel shapes lose
    /// their group (zero filter).
    pub fn frame_smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.frame_smashed {
            return;
        }
        self.frame_smashed = true;
        {
            let at = world.b2().body(self.frame).sweep.c;
            world.play("BikeSmash1", 1, at);
        }
        world.b2_mut().destroy_joint(self.stand_frame);
        self.eject(world, doll);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let mut shapes: Vec<_> = b.body(self.frame).shapes.iter().rev().copied().collect();
        shapes.extend(b.body(self.wheel).shapes.iter().rev().copied());
        for s in shapes {
            b.set_shape_filter(s, zero);
        }
    }

    /// Limb-break reactions (class `i2`, original 30945, plus the 31116 subclass): shoulder and elbow breaks drop that hand's joint and
    /// eject once both hands are gone (`checkEject`); a hip or knee break drops that foot's joint (no eject).
    pub fn limbs_broken(&mut self, world: &mut World, doll: &mut Ragdoll, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        let b = world.b2_mut();
        for l in broken {
            let (hand, slot) = match l {
                Limb::Shoulder1 | Limb::Elbow1 => (true, &mut self.frame_hands[0]),
                Limb::Shoulder2 | Limb::Elbow2 => (true, &mut self.frame_hands[1]),
                Limb::Hip1 | Limb::Knee1 => (false, &mut self.stand_feet[0]),
                Limb::Hip2 | Limb::Knee2 => (false, &mut self.stand_feet[1]),
                _ => continue,
            };
            if let Some(j) = slot.take() {
                b.destroy_joint(j);
                if hand && self.frame_hands.iter().all(Option::is_none) {
                    self.eject(world, doll);
                    return;
                }
            }
        }
    }

    /// Tip of a lower arm / leg: `GetWorldPoint(0, GetVertices()[2].y)` of its shape.
    pub fn is_reattached(&self) -> bool {
        !self.ejected && self.frame_hands.iter().any(Option::is_some)
    }

    fn limb_tip(world: &World, doll: &Ragdoll, part: Part) -> Vec2 {
        let b = world.b2();
        let body = doll.bodies[&part];
        let y = match &b.c.shapes[b.body(body).shapes[0]].shape.kind {
            box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices.get(2).map_or(0.0, |v| v.y),
            _ => 0.0,
        };
        b.body(body).world_point(Vec2::new(0.0, y))
    }

    fn board(&mut self, world: &mut World, doll: &mut Ragdoll, k: usize, tip: Vec2) -> bool {
        if !self.ejected || self.frame_smashed || doll.dead || !near(tip, world.b2().body(self.frame).world_point(self.hand_anchor), 0.25) {
            return false;
        }
        self.re_attach(world, doll, k);
        true
    }

    /// `reAttach` (original 31255, triggered from `grabAction` when an ejected rider's hand is within 0.25 of the handle): the rider
    /// is seated again — limits as in `attach`, frame category 514, wheel filter, that hand's frame joint re-created — and the
    /// `reAttaching` action (see `re_attaching`) then re-grabs the other hand / feet as they come near.
    fn re_attach(&mut self, world: &mut World, doll: &mut Ragdoll, hand: usize) {
        let deg = 180.0 / std::f64::consts::PI;
        self.ejected = false;
        doll.pose = 0;
        doll.set_grab(world, false);
        for (limb, lo, hi) in [(Limb::Hip1, -50.0, 10.0), (Limb::Hip2, -50.0, 10.0), (Limb::Elbow1, -60.0, 0.0), (Limb::Elbow2, -60.0, 0.0), (Limb::Neck, 0.0, 20.0)] {
            doll.set_limits_now(world, limb, lo / deg, hi / deg);
        }
        let b = world.b2_mut();
        let f = Filter { category_bits: 514, mask_bits: 0xFFFF, group_index: 0 };
        let shapes: Vec<_> = b.body(self.frame).shapes.iter().rev().copied().collect();
        for s in shapes {
            b.set_shape_filter(s, f);
        }
        if let Some(&s) = b.body(self.wheel).shapes.first() {
            b.set_shape_filter(s, Filter { category_bits: 260, mask_bits: 268, group_index: 0 });
        }
        self.frame_hands[hand] = Some(b.create_revolute_joint(&self.hand_defs[hand].clone()));
        self.reattaching = true;
    }

    /// The `reAttaching` action (original 31121): every frame, a hand within 0.25 (feet 0.2) of its anchor whose limb is intact
    /// gets its joint back; once all four are attached the action stops.
    fn re_attaching(&mut self, world: &mut World, doll: &Ragdoll) {
        let mut n = 0;
        for k in 0..2 {
            let (arm, shoulder, elbow) = if k == 0 { (Part::LowerArm1, Limb::Shoulder1, Limb::Elbow1) } else { (Part::LowerArm2, Limb::Shoulder2, Limb::Elbow2) };
            if self.frame_hands[k].is_some() || doll.broken.contains(&elbow) || doll.broken.contains(&shoulder) {
                n += 1;
            } else {
                let (t, h) = (Self::limb_tip(world, doll, arm), world.b2().body(self.frame).world_point(self.hand_anchor));
                if (t.x - h.x).abs() < 0.25 && (t.y - h.y).abs() < 0.25 {
                    self.frame_hands[k] = Some(world.b2_mut().create_revolute_joint(&self.hand_defs[k].clone()));
                    n += 1;
                }
            }
        }
        for k in 0..2 {
            let (leg, knee, hip) = if k == 0 { (Part::LowerLeg1, Limb::Knee1, Limb::Hip1) } else { (Part::LowerLeg2, Limb::Knee2, Limb::Hip2) };
            if self.stand_feet[k].is_some() || doll.broken.contains(&knee) || doll.broken.contains(&hip) {
                n += 1;
            } else {
                let (t, h) = (Self::limb_tip(world, doll, leg), world.b2().body(self.frame).world_point(self.foot_anchor));
                if (t.x - h.x).abs() < 0.2 && (t.y - h.y).abs() < 0.2 {
                    self.stand_feet[k] = Some(world.b2_mut().create_revolute_joint(&self.foot_defs[k].clone()));
                    n += 1;
                }
            }
        }
        if n >= 4 {
            self.reattaching = false;
        }
    }

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        doll.poses = PoseSet::Segway;
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        let frame_filter = Filter { category_bits: 514, mask_bits: 0xFFFF, group_index: 0 };
        let box_def = |geometry: Geometry| {
            let mut d = ShapeDef::new(geometry);
            d.density = 5.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = frame_filter;
            d
        };
        let frame = b.create_body(&BodyDef::default());
        for name in ["frameShape", "handleShape"] {
            let g = guide.get(name)?;
            let d = box_def(Geometry::Polygon(oriented_box(g[3] * 5.0 / CHARACTER_SCALE, g[4] * 5.0 / CHARACTER_SCALE, at(name)?, g[2] / deg)));
            b.create_shape(frame, &d);
        }
        b.set_mass_from_shapes(frame);
        let stand_g = guide.get("standShape")?;
        let stand_pos = at("standShape")?;
        let stand_box = || Geometry::Polygon(oriented_box(stand_g[3] * 5.0 / CHARACTER_SCALE, stand_g[4] * 5.0 / CHARACTER_SCALE, Vec2::default(), 0.0));
        let mut sensor_def = box_def(stand_box());
        sensor_def.is_sensor = true;
        let stand = b.create_body(&BodyDef { position: stand_pos, fixed_rotation: true, ..Default::default() });
        b.create_shape(stand, &sensor_def);
        b.set_mass_from_shapes(stand);
        let shock = b.create_body(&BodyDef { position: stand_pos, fixed_rotation: true, ..Default::default() });
        b.create_shape(shock, &sensor_def);
        b.set_mass_from_shapes(shock);
        let wheel_g = guide.get("wheelShape")?;
        let wheel = b.create_body(&BodyDef { position: at("wheelShape")?, angle: wheel_g[2] / deg, ..Default::default() });
        let mut wd = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: wheel_g[5] / 2.0 / CHARACTER_SCALE });
        wd.density = 5.0;
        wd.friction = 1.0;
        wd.restitution = 0.3;
        wd.filter = Filter { category_bits: 260, mask_bits: 268, group_index: -1 };
        b.create_shape(wheel, &wd);
        b.set_mass_from_shapes(wheel);
        // Rider limits for this vehicle (SetLimits after the base joints).
        doll.finish_joints(world);
        for (limb, lo, hi) in [(Limb::Hip1, -50.0, 10.0), (Limb::Hip2, -50.0, 10.0), (Limb::Elbow1, -60.0, 0.0), (Limb::Elbow2, -60.0, 0.0), (Limb::Neck, 0.0, 20.0)] {
            doll.set_limits(world, limb, lo / deg, hi / deg);
        }
        let b = world.b2_mut();
        let shock_anchor = at("wheelShape")?;
        let mut pd = box2d::joints::PrismaticJointDef::initialize(b.body(stand), stand, b.body(shock), shock, shock_anchor, Vec2::new(0.0, 1.0));
        pd.max_motor_force = 1000.0;
        pd.enable_limit = true;
        pd.upper_translation = 0.0;
        pd.lower_translation = 0.0;
        let shock_joint = b.create_prismatic_joint(&pd);
        let mut rd = RevoluteJointDef::initialize(b.body(shock), shock, b.body(wheel), wheel, shock_anchor);
        rd.max_motor_torque = 20.0;
        let wheel_joint = b.create_revolute_joint(&rd);
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2, lo: f64, hi: f64| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 20.0;
            d.enable_limit = true;
            d.lower_angle = lo / deg;
            d.upper_angle = hi / deg;
            b.create_revolute_joint(&d)
        };
        let stand_frame = rev(b, stand, frame, at("frameAnchor")?, -15.0, 15.0);
        let hand = at("handleAnchor")?;
        let foot = at("footAnchor")?;
        let mut defs = Vec::new();
        let mut revd = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2, lo: f64, hi: f64| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 20.0;
            d.enable_limit = true;
            d.lower_angle = lo / deg;
            d.upper_angle = hi / deg;
            defs.push(d.clone());
            b.create_revolute_joint(&d)
        };
        let hand1 = revd(b, frame, doll.bodies[&Part::LowerArm1], hand, -100.0, 10.0);
        let hand2 = revd(b, frame, doll.bodies[&Part::LowerArm2], hand, -100.0, 10.0);
        let foot1 = revd(b, frame, doll.bodies[&Part::LowerLeg1], foot, -10.0, 10.0);
        let foot2 = revd(b, frame, doll.bodies[&Part::LowerLeg2], foot, -10.0, 10.0);
        let hand_anchor = b.body(frame).local_point(hand);
        let foot_anchor = b.body(frame).local_point(foot);
        let mut it = defs.into_iter();
        let (hd1, hd2, fd1, fd2) = (it.next()?, it.next()?, it.next()?, it.next()?);
        Some(Self {
            frame,
            stand,
            shock,
            wheel,
            shock_joint,
            wheel_joint,
            stand_frame,
            frame_smashed: false,
            frame_hands: [Some(hand1), Some(hand2)],
            stand_feet: [Some(foot1), Some(foot2)],
            ejected: false,
            wheel_max_speed: 40.0,
            accel_step: 1.0,
            jump_translation: 30.0 / 62.5,
            hand_defs: [hd1, hd2],
            foot_defs: [fd1, fd2],
            hand_anchor,
            foot_anchor,
            reattaching: false,
        })
    }

    fn wheel_motor(&self, world: &mut World, enabled: Option<bool>, speed: Option<f64>) {
        if let box2d::joints::JointKind::Revolute(r) = &mut world.b2_mut().joint_mut(self.wheel_joint).kind {
            if let Some(e) = enabled {
                r.enable_motor = e;
            }
            if let Some(s) = speed {
                r.motor_speed = s;
            }
        }
    }

    fn wheel_motor_enabled(&self, world: &World) -> bool {
        matches!(&world.b2().joint(self.wheel_joint).kind, box2d::joints::JointKind::Revolute(r) if r.enable_motor)
    }

    /// The shock (jump) logic shared by `spacePressedActions`/`spaceNullActions`.
    fn shock_step(&self, world: &mut World, pressed: bool) {
        use box2d::joints::JointKind::Prismatic;
        let translation = world.b2().prismatic_translation(self.shock_joint).unwrap_or(0.0);
        let jump = self.jump_translation;
        let mut started = false;
        let Prismatic(p) = &mut world.b2_mut().joint_mut(self.shock_joint).kind else { return };
        if p.enable_motor {
            let speed = p.motor_speed;
            if speed > 0.0 {
                if translation > jump {
                    p.motor_speed = -1.0;
                }
            } else if speed < 0.0 && translation < 0.0 {
                p.enable_motor = false;
                p.lower_translation = 0.0;
                p.upper_translation = 0.0;
                p.motor_speed = 0.0;
            }
        } else if pressed {
            p.motor_speed = 7.0;
            p.lower_translation = 0.0;
            p.upper_translation = jump;
            p.enable_motor = true;
            started = true;
        }
        if started {
            // `playAreaSoundInstance("SegwayJump", wheelBody)`
            let at = world.b2().body(self.wheel).sweep.c;
            world.play("SegwayJump", 1, at);
        }
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        let Keys { left, right, up, down, space, z, .. } = k;
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            // left / right
            if ejected {
                doll.keys(left, right, false, false);
            } else {
                let (lo, gain) = match (left, right) {
                    (true, false) => (0.0, 20.0),
                    (false, true) => (0.52, 20.0),
                    _ => (0.26, 20.0),
                };
                let stand_frame = self.stand_frame;
                doll.set_joint_raw(world, stand_frame, lo, gain);
            }
            // up / down
            match (up, down) {
                (true, false) | (false, true) if ejected => {
                    doll.keys(false, false, up, down);
                }
                (true, false) => self.drive(world, 1.0),
                (false, true) => self.drive(world, -1.0),
                _ => {
                    if self.wheel_motor_enabled(world) {
                        self.wheel_motor(world, Some(false), None);
                    }
                    if doll.pose == 3 || doll.pose == 4 {
                        doll.set_pose(0);
                    }
                }
            }
            // space
            if space {
                if self.ejected {
                    doll.set_grab(world, true);
                } else {
                    self.shock_step(world, true);
                }
            } else if ejected {
                doll.set_grab(world, false);
            } else {
                self.shock_step(world, false);
            }
            // shift / ctrl
            if k.shift {
                doll.set_pose(if ejected { 7 } else { 5 });
            } else if doll.pose == 5 || doll.pose == 7 {
                doll.set_pose(0);
            }
            if k.ctrl {
                doll.set_pose(if ejected { 8 } else { 6 });
            } else if doll.pose == 6 || doll.pose == 8 {
                doll.set_pose(0);
            }
            if z {
                self.eject(world, doll);
            }
        }
        if self.reattaching {
            self.re_attaching(world, doll);
        }
        doll.apply_pose(world);
    }

    fn drive(&self, world: &mut World, dir: f64) {
        if self.ejected {
            return;
        }
        if !self.wheel_motor_enabled(world) {
            self.wheel_motor(world, Some(true), None);
        }
        let b = world.b2();
        let cur = b.body(self.wheel).angular_velocity - b.body(self.shock).angular_velocity;
        let new = if cur * dir < self.wheel_max_speed { cur + dir * self.accel_step } else { cur };
        self.wheel_motor(world, None, Some(new));
    }

    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        self.reattaching = false;
        doll.reset_joint_limits(world);
        let b = world.b2_mut();
        for j in [self.frame_hands[0].take(), self.frame_hands[1].take(), self.stand_feet[0].take(), self.stand_feet[1].take()].into_iter().flatten() {
            b.destroy_joint(j);
        }
        if let box2d::joints::JointKind::Prismatic(p) = &mut b.joint_mut(self.shock_joint).kind {
            p.enable_motor = false;
            p.lower_translation = 0.0;
            p.upper_translation = 0.0;
            p.motor_speed = 0.0;
        }
        let f = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: -2 };
        let mut shapes: Vec<_> = b.body(self.frame).shapes.iter().rev().copied().collect();
        shapes.extend(b.body(self.wheel).shapes.iter().rev().copied());
        for s in shapes {
            b.set_shape_filter(s, f);
        }
    }
}

/// Polygon vertices of an oriented box (Box2D 2.0.1 `SetAsOrientedBox`).
fn oriented_box(hx: f64, hy: f64, center: Vec2, angle: f64) -> Vec<Vec2> {
    // `b2PolygonDef.SetAsOrientedBox`: fdlibm sin/cos (V8's Math) and center + (col1.x * x + col2.x * y)
    let m = box2d::math::Mat22::from_angle(angle);
    [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        .iter()
        .map(|&(x, y)| Vec2::new(center.x + (m.col1.x * x + m.col2.x * y), center.y + (m.col1.y * x + m.col2.y * y)))
        .collect()
}

/// Jet wheelchair (guide `c1`): a chair body (three boxes) on a big drive wheel and a small wheel, the rider
/// strapped in at chest/pelvis/thighs; space fires a jet that pushes the big wheel along the chair's angle
/// (+ `jet_angle`, tuned with shift/ctrl), up/down drive the big wheel, left/right tilt by impulse.
pub struct Chair {
    pub chair: BodyId,
    pub big_wheel: BodyId,
    pub small_wheel: BodyId,
    big_joint: usize,
    small_joint: usize,
    /// After `chair_smash`: loose debris (rod, fuel tank, jet).
    pub chair_smashed: bool,
    pub debris: Vec<BodyId>,
    pub tank: Option<BodyId>,
    pub jet: Option<BodyId>,
    straps: [Option<usize>; 4],
    pub ejected: bool,
    pub firing: bool,
    pub jet_angle: f64,
    pub wheel_max_speed: f64,
    pub accel_step: f64,
    pub impulse_magnitude: f64,
    pub impulse_offset: f64,
    pub max_spin_av: f64,
    pub jet_impulse: f64,
}

impl Chair {
    pub const JET_SMASH_LIMIT: f64 = 30.0;
    pub const FUELTANK_SMASH_LIMIT: f64 = 30.0;

    /// `jetSmash` (original 36345, limit 30): the loose jet body is destroyed.
    pub fn jet_smash(&mut self, world: &mut World) {
        if let Some(jet) = self.jet.take() {
            let c = world.b2().body(jet).sweep.c;
            world.shard_burst(crate::blood::ShardKind::Metal, c, 5.0, 50.0, 50);
            world.b2_mut().destroy_body(jet);
            self.debris.retain(|&d| d != jet);
        }
    }

    /// `fueltankSmash` (original 36307, limit 30): explosion — every non-static shape whose box touches a 4x4 m square around the
    /// tank gets an impulse of 10 * (1 - dist/2) away from the tank (one per shape), then the tank body is destroyed.
    pub fn fueltank_smash(&mut self, world: &mut World) {
        let Some(tank) = self.tank.take() else { return };
        let b = world.b2_mut();
        let c = b.body(tank).sweep.c;
        let aabb = box2d::collision::Aabb { lower: Vec2::new(c.x - 2.0, c.y - 2.0), upper: Vec2::new(c.x + 2.0, c.y + 2.0) };
        for sh in b.query(&aabb, 30) {
            let body = b.c.shapes[sh].body;
            if b.body(body).is_static {
                continue;
            }
            let wc = b.body(body).sweep.c;
            let d = Vec2::new(wc.x - c.x, wc.y - c.y);
            let t = 1.0 - d.length().min(2.0) / 2.0;
            let a = d.y.atan2(d.x);
            b.body_mut(body).apply_impulse(Vec2::new(box2d::fdlibm::cos(a) * t * 10.0, box2d::fdlibm::sin(a) * t * 10.0), wc);
        }
        b.destroy_body(tank);
        world.shard_burst(crate::blood::ShardKind::Metal, c, 5.0, 50.0, 50);
        self.debris.retain(|&d| d != tank);
    }

    pub const CHAIR_SMASH_LIMIT: f64 = 200.0;

    /// `chairSmash` (original 36215, limit 200 on the first chair shape): both wheel joints and all chair shapes go; the chair
    /// keeps one 80x90 px zero-filter box (mass not recomputed) and sheds three loose bodies carrying its velocity — a rod
    /// (10x70 px, polygon in world coordinates on an origin body), the fuel tank (60x18 px at the chair) and the jet (42x33 px at
    /// the big wheel, turned by the jet angle). Firing stops and the rider is ejected.
    pub fn chair_smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.chair_smashed {
            return;
        }
        self.chair_smashed = true;
        {
            let at = world.b2().body(self.chair).sweep.c;
            world.play("MetalSmashMedium", 1, at);
        }
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let cs = CHARACTER_SCALE;
        let b = world.b2_mut();
        b.destroy_joint(self.big_joint);
        b.destroy_joint(self.small_joint);
        let old: Vec<_> = b.body(self.chair).shapes.iter().rev().copied().collect();
        for sh in old {
            b.destroy_shape(sh);
        }
        let mut d = ShapeDef::new(Geometry::Polygon(Vec::new()));
        d.density = 2.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = zero;
        let lc = b.body(self.chair).sweep.local_center;
        d.geometry = Geometry::Polygon(oriented_box(40.0 / cs, 45.0 / cs, Vec2::new(lc.x + 8.0 / cs, lc.y + 2.0 / cs), 0.0));
        b.create_shape(self.chair, &d);
        let angle = b.body(self.chair).sweep.a;
        let (lv, av) = (b.body(self.chair).linear_velocity, b.body(self.chair).angular_velocity);
        // rod: origin body, polygon already in world coordinates
        let c = b.body(self.chair).world_point(Vec2::new(lc.x - 30.0 / cs, lc.y - 55.0 / cs));
        let rod = b.create_body(&BodyDef::default());
        d.geometry = Geometry::Polygon(oriented_box(5.0 / cs, 35.0 / cs, c, angle));
        b.create_shape(rod, &d);
        b.set_mass_from_shapes(rod);
        b.body_mut(rod).angular_velocity = av;
        b.body_mut(rod).linear_velocity = lv;
        // fuel tank
        let pos = b.body(self.chair).world_point(Vec2::new(lc.x + 8.0 / cs, lc.y + 14.0 / cs));
        let tank = b.create_body(&BodyDef { position: pos, angle, ..Default::default() });
        d.geometry = Geometry::Polygon(oriented_box(30.0 / cs, 9.0 / cs, Vec2::default(), 0.0));
        b.create_shape(tank, &d);
        b.set_mass_from_shapes(tank);
        b.body_mut(tank).angular_velocity = av;
        b.body_mut(tank).linear_velocity = lv;
        // jet
        let jet = b.create_body(&BodyDef::default());
        let wc = b.body(self.big_wheel).sweep.c;
        d.geometry = Geometry::Polygon(oriented_box(21.0 / cs, 16.5 / cs, wc, angle + self.jet_angle));
        b.create_shape(jet, &d);
        b.set_mass_from_shapes(jet);
        b.body_mut(jet).angular_velocity = av;
        b.body_mut(jet).linear_velocity = lv;
        self.debris = vec![rod, tank, jet];
        self.tank = Some(tank);
        self.jet = Some(jet);
        self.firing = false;
        self.eject(world, doll);
    }

    pub const WHEEL_SMASH_LIMIT: f64 = 200.0;

    /// `wheelSmash` (original 36297, limit 200): the big wheel's circle becomes a 30x100 px box turned to the contact normal
    /// (density 2; zero filter if ejected, else category 1025).
    pub fn wheel_smash(&mut self, world: &mut World, normal: Vec2) {
        let b = world.b2_mut();
        let Some(&old) = b.body(self.big_wheel).shapes.first() else { return };
        b.destroy_shape(old);
        let angle = normal.y.atan2(normal.x) - b.body(self.big_wheel).sweep.a;
        let mut d = ShapeDef::new(Geometry::Polygon(oriented_box(15.0 / CHARACTER_SCALE, 50.0 / CHARACTER_SCALE, Vec2::default(), angle)));
        d.density = 2.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = if self.ejected { Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 } } else { Filter { category_bits: 1025, ..Filter::default() } };
        b.create_shape(self.big_wheel, &d);
    }

    /// Limb-break reactions (original 36381): a hip break destroys that leg's strap joint (no eject).
    pub fn limbs_broken(&mut self, world: &mut World, broken: &[Limb]) {
        for l in broken {
            let slot = match l {
                Limb::Hip1 => 2,
                Limb::Hip2 => 3,
                _ => continue,
            };
            if let Some(j) = self.straps[slot].take() {
                world.b2_mut().destroy_joint(j);
            }
        }
    }

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        doll.poses = PoseSet::Chair;
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        let cat = Filter { category_bits: 1025, mask_bits: 0xFFFF, group_index: 0 };
        let chair = b.create_body(&BodyDef::default());
        for name in ["chair1Shape", "chair2Shape", "chair3Shape"] {
            let g = guide.get(name)?;
            let mut d = ShapeDef::new(Geometry::Polygon(oriented_box(g[5] / 2.0 / CHARACTER_SCALE, g[6] / 2.0 / CHARACTER_SCALE, at(name)?, 0.0)));
            d.density = 2.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = cat;
            b.create_shape(chair, &d);
        }
        b.set_mass_from_shapes(chair);
        let mut wheel = |name: &str| -> Option<BodyId> {
            let g = guide.get(name)?;
            let body = b.create_body(&BodyDef { position: at(name)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: g[5] / 2.0 / CHARACTER_SCALE });
            d.density = 5.0;
            d.friction = 1.0;
            d.restitution = 0.1;
            d.filter = cat;
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            Some(body)
        };
        let big_wheel = wheel("bigWheelShape")?;
        let small_wheel = wheel("smallWheelShape")?;
        for part in [Part::LowerArm1, Part::LowerArm2] {
            let s = b.body(doll.bodies[&part]).shapes[0];
            b.set_shape_filter(s, Filter { category_bits: 516, mask_bits: 520, group_index: -1 });
        }
        doll.finish_joints(world);
        for limb in [Limb::Hip1, Limb::Hip2] {
            doll.set_limits(world, limb, -110.0 / deg, -90.0 / deg);
        }
        let b = world.b2_mut();
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 20.0;
            b.create_revolute_joint(&d)
        };
        let big_joint = rev(b, chair, big_wheel, at("bigWheelShape")?);
        let small_joint = rev(b, chair, small_wheel, at("smallWheelShape")?);
        let pos = |b: &box2d::world::World, id: BodyId| b.body(id).xf.position;
        let chest = doll.bodies[&Part::Chest];
        let pelvis = doll.bodies[&Part::Pelvis];
        let (leg1, leg2) = (doll.bodies[&Part::UpperLeg1], doll.bodies[&Part::UpperLeg2]);
        let p = pos(b, chest);
        let chair_chest = rev(b, chair, chest, p);
        let p = pos(b, pelvis);
        let chair_pelvis = rev(b, chair, pelvis, p);
        let p = pos(b, leg1);
        let chair_leg1 = rev(b, chair, leg1, p);
        let p = pos(b, leg2);
        let chair_leg2 = rev(b, chair, leg2, p);
        Some(Self {
            chair,
            big_wheel,
            small_wheel,
            big_joint,
            small_joint,
            chair_smashed: false,
            debris: Vec::new(),
            tank: None,
            jet: None,
            straps: [Some(chair_pelvis), Some(chair_chest), Some(chair_leg1), Some(chair_leg2)],
            ejected: false,
            firing: false,
            jet_angle: 0.0,
            wheel_max_speed: 20.0,
            accel_step: 0.2,
            impulse_magnitude: 0.5,
            impulse_offset: 1.0,
            max_spin_av: 5.0,
            jet_impulse: 2.0,
        })
    }

    fn motor(&self, world: &mut World, enabled: Option<bool>, speed: Option<f64>) {
        if let box2d::joints::JointKind::Revolute(r) = &mut world.b2_mut().joint_mut(self.big_joint).kind {
            if let Some(e) = enabled {
                r.enable_motor = e;
            }
            if let Some(s) = speed {
                r.motor_speed = s;
            }
        }
    }

    fn motor_enabled(&self, world: &World) -> bool {
        matches!(&world.b2().joint(self.big_joint).kind, box2d::joints::JointKind::Revolute(r) if r.enable_motor)
    }

    fn tilt(&self, world: &mut World, left: bool) {
        let b = world.b2_mut();
        let (angle, av) = (b.body(self.chair).sweep.a, b.body(self.chair).angular_velocity);
        let s = self.max_spin_av;
        let q = if left { (av + s) / s } else { (av - s) / -s }.clamp(0.0, 1.0);
        let (a, w) = (box2d::fdlibm::cos(angle) * self.impulse_magnitude * q, box2d::fdlibm::sin(angle) * self.impulse_magnitude * q);
        let c = b.body(self.chair).sweep.local_center;
        let off = if left { self.impulse_offset } else { -self.impulse_offset };
        let p = b.body(self.chair).world_point(Vec2::new(c.x + off, c.y));
        b.body_mut(self.chair).apply_impulse(Vec2::new(w, -a), p);
    }

    fn drive(&self, world: &mut World, doll: &mut Ragdoll, dir: f64) {
        if !self.motor_enabled(world) {
            self.motor(world, Some(true), None);
        }
        let b = world.b2();
        let cur = b.body(self.big_wheel).angular_velocity - b.body(self.chair).angular_velocity;
        let new = if cur * dir < 0.0 {
            0.0
        } else if cur * dir < self.wheel_max_speed {
            cur + dir * self.accel_step
        } else {
            cur
        };
        self.motor(world, None, Some(new));
        doll.set_pose(5);
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            match (k.left, k.right) {
                (true, false) if ejected => doll.set_pose(1),
                (false, true) if ejected => doll.set_pose(2),
                (true, false) => self.tilt(world, true),
                (false, true) => self.tilt(world, false),
                _ if ejected && (doll.pose == 1 || doll.pose == 2) => doll.set_pose(0),
                _ => {}
            }
            match (k.up, k.down) {
                (true, false) if ejected => doll.set_pose(3),
                (false, true) if ejected => doll.set_pose(4),
                (true, false) => self.drive(world, doll, 1.0),
                (false, true) => self.drive(world, doll, -1.0),
                _ => {
                    if self.motor_enabled(world) {
                        self.motor(world, Some(false), None);
                    }
                    if ejected {
                        if doll.pose == 3 || doll.pose == 4 {
                            doll.set_pose(0);
                        }
                    } else if doll.pose == 5 {
                        doll.set_pose(0);
                    }
                }
            }
            if k.space {
                if ejected {
                    doll.set_grab(world, true);
                } else {
                    self.firing = true;
                }
            } else if ejected {
                doll.set_grab(world, false);
            } else {
                self.firing = false;
            }
            if k.shift {
                if ejected {
                    doll.set_pose(6);
                } else {
                    self.jet_angle += 0.1;
                }
            } else if doll.pose == 6 {
                doll.set_pose(0);
            }
            if k.ctrl {
                if ejected {
                    doll.set_pose(7);
                } else {
                    self.jet_angle -= 0.1;
                }
            } else if doll.pose == 7 {
                doll.set_pose(0);
            }
            if k.z {
                self.eject(world, doll);
            }
        }
        // `actions`: jet thrust, then the pose motors.
        if self.firing {
            let b = world.b2_mut();
            let a = b.body(self.chair).sweep.a + self.jet_angle;
            let p = b.body(self.chair).sweep.c;
            b.body_mut(self.big_wheel).apply_impulse(Vec2::new(box2d::fdlibm::cos(a) * self.jet_impulse, box2d::fdlibm::sin(a) * self.jet_impulse), p);
        }
        doll.apply_pose(world);
    }

    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        doll.reset_joint_limits(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        for j in self.straps.iter_mut().filter_map(Option::take) {
            b.destroy_joint(j);
        }
        let wheels = [self.small_wheel, self.big_wheel].map(|w| (b.body(w).shapes[0], zero));
        b.set_shape_filters(&wheels);
        let shapes: Vec<_> = b.body(self.chair).shapes.iter().rev().copied().collect();
        for s in shapes {
            b.set_shape_filter(s, zero);
        }
        for part in [Part::LowerArm1, Part::LowerArm2] {
            let s = b.body(doll.bodies[&part]).shapes[0];
            if b.shape_mut(s).filter.mask_bits == 520 {
                b.set_shape_filter(s, Filter { category_bits: 260, mask_bits: 270, group_index: -1 });
            }
        }
    }
}

/// Dirt bike with boost (guide `c5`, class with `boostVal`): frame of six polygons (rear/middle/engine with
/// category 513, seat/tank/fork with the zero filter), two wheels, rider joined to the frame by free
/// revolutes. Only the back wheel is driven; space = boost impulse (until the meter fills), ctrl = brake.
pub struct DirtBike {
    pub frame: BodyId,
    pub back_wheel: BodyId,
    pub front_wheel: BodyId,
    back_joint: usize,
    front_joint: usize,
    joints: [Option<usize>; 5],
    pub ejected: bool,
    /// Pieces left by `frameSmash` (fork, tank, engine, middle, rear, seat — body creation order).
    pub pieces: Vec<BodyId>,
    pub frame_smashed: bool,
    pub boost: f64,
    pub wheel_max_speed: f64,
    pub accel_step: f64,
    pub impulse_magnitude: f64,
    pub impulse_offset: f64,
    /// The moped girl behind the rider (class `MD`); None only for the bare bike.
    pub girl: Option<Box<Girl>>,
    /// `reAttach` data: creation defs of the pelvis / hand / foot joints and the frame-local anchors they pin to.
    pelvis_def: RevoluteJointDef,
    hand_defs: [RevoluteJointDef; 2],
    foot_defs: [RevoluteJointDef; 2],
    anchors: [Vec2; 3],
    reattaching: bool,
}

impl DirtBike {
    /// The handle anchor in the frame's local space (for tests / re-boarding).
    pub fn handle_anchor(&self) -> Vec2 {
        self.anchors[1]
    }

    /// `frontWheelSmash` / `backWheelSmash` (original 25683): like the bike's, but the box is 25×45 px. The original also zeroes
    /// its contact counter (the port derives contacts from the world, so nothing to do).
    pub fn wheel_smash(&mut self, world: &mut World, front: bool, normal: Vec2) {
        {
            let at = world.b2().body(if front { self.front_wheel } else { self.back_wheel }).sweep.c;
            world.play("BikeTireSmash1", 1, at);
        }
        let wheel = if front { self.front_wheel } else { self.back_wheel };
        let b = world.b2_mut();
        let Some(&old) = b.body(wheel).shapes.first() else { return };
        b.destroy_shape(old);
        let angle = normal.y.atan2(normal.x) - b.body(wheel).sweep.a;
        let mut d = ShapeDef::new(Geometry::Polygon(oriented_box(25.0 / CHARACTER_SCALE, 45.0 / CHARACTER_SCALE, Vec2::default(), angle)));
        d.density = 2.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = if self.ejected { Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 } } else { Filter { category_bits: 513, ..Filter::default() } };
        b.create_shape(wheel, &d);
    }

    pub const FRAME_SMASH_LIMIT: f64 = 200.0;

    /// `frameSmash` (original 25586): eject, then the frame's six polygons become six loose bodies (density 3, zero filter, the
    /// frame's pose and velocities). Bodies are created fork, tank, engine, middle, rear, seat; shapes are added rear, middle,
    /// engine, seat, tank, fork (as in the original); the frame is destroyed.
    pub fn frame_smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.frame_smashed {
            return;
        }
        self.frame_smashed = true;
        {
            let at = world.b2().body(self.frame).sweep.c;
            world.play("MetalSmashHeavy", 1, at);
            world.shard_burst(crate::blood::ShardKind::Metal, at, 5.0, 50.0, 50);
        }
        self.eject(world, doll);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        // frame shapes in creation order: rear, middle, engine, seat, tank, fork
        let polys: Vec<Vec<Vec2>> = b
            .body(self.frame)
            .shapes
            .iter()
            
            .map(|&sh| match &b.c.shapes[sh].shape.kind {
                box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices.clone(),
                _ => Vec::new(),
            })
            .collect();
        let (pos, ang) = (b.body(self.frame).xf.position, b.body(self.frame).sweep.a);
        let (v, w) = (b.body(self.frame).linear_velocity, b.body(self.frame).angular_velocity);
        let mut bodies = Vec::new();
        for _ in 0..6 {
            let body = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
            b.body_mut(body).linear_velocity = v;
            b.body_mut(body).angular_velocity = w;
            bodies.push(body); // fork, tank, engine, middle, rear, seat
        }
        // shape order rear(4), middle(3), engine(2), seat(5), tank(1), fork(0)
        for (poly, target) in polys.into_iter().zip([4usize, 3, 2, 5, 1, 0]) {
            let mut d = ShapeDef::new(Geometry::Polygon(poly));
            d.density = 3.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(bodies[target], &d);
            b.set_mass_from_shapes(bodies[target]);
        }
        // paint order: rear, middle, engine, seat, tank, fork
        self.pieces = [4usize, 3, 2, 5, 1, 0].iter().map(|&k| bodies[k]).collect();
        b.destroy_body(self.frame);
        // the passenger is thrown off once the frame is gone, which lets her legs collide at once
        if let Some(g) = self.girl.as_mut() {
            g.frame_gone = true;
            g.eject(world);
        }
    }

    /// Limb-break reactions (class with `boostVal`, original 25774): shoulder/elbow → that hand's joint, hip/knee → that foot's;
    /// eject once all four are gone (the pelvis joint, slot 0, does not count).
    pub fn limbs_broken(&mut self, world: &mut World, doll: &mut Ragdoll, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        let b = world.b2_mut();
        for l in broken {
            // a broken neck keeps the rider on the moped but takes both hands off the bars (25762)
            if *l == Limb::Neck {
                for j in [self.joints[1].take(), self.joints[2].take()].into_iter().flatten() {
                    b.destroy_joint(j);
                }
            }
            let slot = match l {
                Limb::Shoulder1 | Limb::Elbow1 => 1,
                Limb::Shoulder2 | Limb::Elbow2 => 2,
                Limb::Hip1 | Limb::Knee1 => 3,
                Limb::Hip2 | Limb::Knee2 => 4,
                _ => continue,
            };
            if let Some(j) = self.joints[slot].take() {
                b.destroy_joint(j);
            }
        }
        if !broken.is_empty() && self.joints[1..].iter().all(Option::is_none) {
            self.eject(world, doll);
        }
    }

    fn limb_tip(world: &World, doll: &Ragdoll, part: Part) -> Vec2 {
        let b = world.b2();
        let body = doll.bodies[&part];
        let y = match &b.c.shapes[b.body(body).shapes[0]].shape.kind {
            box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices.get(2).map_or(0.0, |v| v.y),
            _ => 0.0,
        };
        b.body(body).world_point(Vec2::new(0.0, y))
    }

    /// `reAttach` (original 25865/25944): neck −20..0°, elbows −90..0°, that hand's joint re-created, `reAttaching` started.
    fn board(&mut self, world: &mut World, doll: &mut Ragdoll, k: usize, tip: Vec2) -> bool {
        if !self.ejected || self.frame_smashed || doll.dead || !near(tip, world.b2().body(self.frame).world_point(self.anchors[1]), 0.25) {
            return false;
        }
        let deg = 180.0 / std::f64::consts::PI;
        self.ejected = false;
        doll.set_pose(0);
        doll.set_grab(world, false);
        for (limb, lo, hi) in [(Limb::Neck, -20.0, 0.0), (Limb::Elbow1, -90.0, 0.0), (Limb::Elbow2, -90.0, 0.0)] {
            doll.set_limits_now(world, limb, lo / deg, hi / deg);
        }
        self.joints[1 + k] = Some(world.b2_mut().create_revolute_joint(&self.hand_defs[k].clone()));
        self.reattaching = true;
        true
    }

    /// The `reAttaching` action (original 24609/24983, five joints): hands near the handle, the pelvis near its seat anchor with both hips bent
    /// past −0.87 rad (or broken), then (pelvis on) the feet near the foot anchor; every success re-creates the joint from its creation def.
    /// Re-seating the pelvis makes the thighs sensors again and starts the girl's own `reAttaching`.
    fn re_attaching(&mut self, world: &mut World, doll: &Ragdoll) {
        let mut n = 0;
        for k in 0..2 {
            let (arm, shoulder, elbow) = if k == 0 { (Part::LowerArm1, Limb::Shoulder1, Limb::Elbow1) } else { (Part::LowerArm2, Limb::Shoulder2, Limb::Elbow2) };
            if self.joints[1 + k].is_some() || doll.broken.contains(&elbow) || doll.broken.contains(&shoulder) {
                n += 1;
            } else if near(Self::limb_tip(world, doll, arm), world.b2().body(self.frame).world_point(self.anchors[1]), 0.25) {
                self.joints[1 + k] = Some(world.b2_mut().create_revolute_joint(&self.hand_defs[k].clone()));
                n += 1;
            }
        }
        if self.joints[0].is_some() {
            n += 1;
        } else {
            let pelvis = world.b2().body(doll.bodies[&Part::Pelvis]).xf.position;
            let seat = world.b2().body(self.frame).world_point(self.anchors[0]);
            let bent = |limb: Limb, w: &World| doll.broken.contains(&limb) || doll.joints.get(&limb).and_then(|&j| w.b2().revolute_angle(j)).is_some_and(|a| a < -0.87);
            if near(pelvis, seat, 0.25) && bent(Limb::Hip1, world) && bent(Limb::Hip2, world) {
                self.joints[0] = Some(world.b2_mut().create_revolute_joint(&self.pelvis_def.clone()));
                for part in [Part::UpperLeg1, Part::UpperLeg2] {
                    let b = world.b2_mut();
                    let s = b.body(doll.bodies[&part]).shapes[0];
                    b.shape_mut(s).is_sensor = true;
                    b.refilter(s);
                }
                n += 1;
                if let Some(g) = self.girl.as_mut() {
                    g.start_re_attach();
                }
            }
        }
        if self.joints[0].is_some() {
            for k in 0..2 {
                let (leg, knee, hip) = if k == 0 { (Part::LowerLeg1, Limb::Knee1, Limb::Hip1) } else { (Part::LowerLeg2, Limb::Knee2, Limb::Hip2) };
                if self.joints[3 + k].is_some() || doll.broken.contains(&knee) || doll.broken.contains(&hip) {
                    n += 1;
                } else if near(Self::limb_tip(world, doll, leg), world.b2().body(self.frame).world_point(self.anchors[2]), 0.25) {
                    self.joints[3 + k] = Some(world.b2_mut().create_revolute_joint(&self.foot_defs[k].clone()));
                    n += 1;
                }
            }
        }
        if n >= 5 {
            self.reattaching = false;
        }
    }

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let poly = |prefix: &str, n: usize| -> Option<Vec<Vec2>> { (1..=n).map(|i| at(&format!("{prefix}{i}"))).collect() };
        let deg = 180.0 / std::f64::consts::PI;
        let main = Filter { category_bits: 513, mask_bits: 0xFFFF, group_index: 0 };
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let frame = b.create_body(&BodyDef::default());
        for (prefix, n, filter) in [("rearVert", 3, main), ("middleVert", 4, main), ("engineVert", 5, main), ("seatVert", 4, zero), ("tankVert", 5, zero), ("forkVert", 3, zero)] {
            let mut d = ShapeDef::new(Geometry::Polygon(poly(prefix, n)?));
            d.density = 3.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = filter;
            b.create_shape(frame, &d);
        }
        b.set_mass_from_shapes(frame);
        let mut wheel = |name: &str| -> Option<BodyId> {
            let g = guide.get(name)?;
            let body = b.create_body(&BodyDef { position: at(name)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: g[3] * 5.0 / CHARACTER_SCALE });
            d.density = 5.0;
            d.friction = 1.0;
            d.restitution = 0.3;
            d.filter = zero;
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            Some(body)
        };
        let back_wheel = wheel("backWheelShape")?;
        let front_wheel = wheel("frontWheelShape")?;
        for part in [Part::UpperLeg1, Part::UpperLeg2] {
            let s = b.body(doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = true;
        }
        doll.finish_joints(world);
        for (limb, lo, hi) in [(Limb::Elbow1, -90.0, 0.0), (Limb::Elbow2, -90.0, 0.0), (Limb::Neck, -20.0, 0.0)] {
            doll.set_limits(world, limb, lo / deg, hi / deg);
        }
        let b = world.b2_mut();
        let def = |b: &box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 30.0;
            d
        };
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2| {
            let d = def(b, b1, b2, p);
            b.create_revolute_joint(&d)
        };
        let back_joint = rev(b, frame, back_wheel, at("backWheelShape")?);
        let front_joint = rev(b, frame, front_wheel, at("frontWheelShape")?);
        let pelvis = doll.bodies[&Part::Pelvis];
        let p = b.body(pelvis).xf.position;
        let pelvis_def = def(b, frame, pelvis, p);
        let frame_pelvis = b.create_revolute_joint(&pelvis_def);
        let hand = at("handleAnchor")?;
        let hand_defs = [def(b, frame, doll.bodies[&Part::LowerArm1], hand), def(b, frame, doll.bodies[&Part::LowerArm2], hand)];
        let hand1 = b.create_revolute_joint(&hand_defs[0]);
        let hand2 = b.create_revolute_joint(&hand_defs[1]);
        let foot = at("footAnchor")?;
        let foot_defs = [def(b, frame, doll.bodies[&Part::LowerLeg1], foot), def(b, frame, doll.bodies[&Part::LowerLeg2], foot)];
        let foot1 = b.create_revolute_joint(&foot_defs[0]);
        let foot2 = b.create_revolute_joint(&foot_defs[1]);
        let anchors = [b.body(frame).local_point(p), b.body(frame).local_point(hand), b.body(frame).local_point(foot)];
        Some(Self {
            frame,
            back_wheel,
            front_wheel,
            back_joint,
            front_joint,
            joints: [Some(frame_pelvis), Some(hand1), Some(hand2), Some(foot1), Some(foot2)],
            ejected: false,
            girl: None,
            pelvis_def,
            hand_defs,
            foot_defs,
            anchors,
            reattaching: false,
            pieces: Vec::new(),
            frame_smashed: false,
            boost: 0.0,
            wheel_max_speed: 30.0,
            accel_step: 0.5,
            impulse_magnitude: 1.0,
            impulse_offset: 1.0,
        })
    }

    fn motors(&self, world: &mut World, back: Option<bool>, front: Option<bool>, speed: Option<f64>) {
        for (id, enable) in [(self.back_joint, back), (self.front_joint, front)] {
            if let box2d::joints::JointKind::Revolute(r) = &mut world.b2_mut().joint_mut(id).kind {
                if let Some(e) = enable {
                    r.enable_motor = e;
                }
                if let (Some(s), Some(_)) = (speed, enable) {
                    r.motor_speed = s;
                }
            }
        }
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        if let Some(mut girl) = self.girl.take() {
            self.keys_inner(world, doll, k);
            // forwarded after the rider's own handler; the rider's shift ejects the girl (original `shiftPressedActions`)
            girl.keys(world, k);
            girl.re_attaching(world, doll.bodies[&Part::Chest]);
            self.girl = Some(girl);
        } else {
            self.keys_inner(world, doll, k);
        }
    }

    fn keys_inner(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            match (k.left, k.right) {
                (true, false) if ejected => doll.set_pose(1),
                (false, true) if ejected => doll.set_pose(2),
                (true, false) | (false, true) => {
                    let left = k.left;
                    let b = world.b2_mut();
                    let angle = b.body(self.frame).sweep.a;
                    let (a, w) = (box2d::fdlibm::cos(angle) * self.impulse_magnitude, box2d::fdlibm::sin(angle) * self.impulse_magnitude);
                    let c = b.body(self.frame).sweep.local_center;
                    let off = if left { self.impulse_offset } else { -self.impulse_offset };
                    let p = b.body(self.frame).world_point(Vec2::new(c.x + off, c.y));
                    b.body_mut(self.frame).apply_impulse(Vec2::new(w, -a), p);
                }
                _ if ejected && (doll.pose == 1 || doll.pose == 2) => doll.set_pose(0),
                _ => {}
            }
            match (k.up, k.down) {
                (true, false) if ejected => doll.set_pose(3),
                (false, true) if ejected => doll.set_pose(4),
                (true, false) | (false, true) => {
                    let dir = if k.up { 1.0 } else { -1.0 };
                    self.motors(world, Some(true), Some(false), None);
                    let b = world.b2();
                    let cur = b.body(self.back_wheel).angular_velocity - b.body(self.frame).angular_velocity;
                    let new = if cur * dir < 0.0 {
                        0.0
                    } else if cur * dir < self.wheel_max_speed {
                        cur + dir * self.accel_step
                    } else {
                        cur
                    };
                    if let box2d::joints::JointKind::Revolute(r) = &mut world.b2_mut().joint_mut(self.back_joint).kind {
                        r.motor_speed = new;
                    }
                }
                _ => {
                    self.motors(world, Some(false), Some(false), None);
                    if ejected && (doll.pose == 3 || doll.pose == 4) {
                        doll.set_pose(0);
                    }
                }
            }
            if k.space {
                if ejected {
                    doll.set_grab(world, true);
                } else {
                    self.boost = (self.boost + 2.0).min(50.0);
                    if self.boost < 50.0 {
                        let b = world.b2_mut();
                        let a = b.body(self.frame).sweep.a;
                        let p = b.body(self.frame).sweep.c;
                        b.body_mut(self.frame).apply_impulse(Vec2::new(box2d::fdlibm::cos(a) * 4.0, box2d::fdlibm::sin(a) * 4.0), p);
                    }
                }
            } else {
                if ejected {
                    doll.set_grab(world, false);
                }
                self.boost = (self.boost - 0.5).max(0.0);
            }
            if k.ctrl && !ejected {
                self.motors(world, Some(true), Some(true), Some(0.0));
            }
            if k.z {
                self.eject(world, doll);
            }
        }
        doll.apply_pose(world);
        if self.reattaching {
            self.re_attaching(world, doll);
        }
    }

    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            // original 24840: a second eject of the rider lets the girl let go of his torso
            if let Some(g) = self.girl.as_mut() {
                g.release_guy(world);
            }
            return;
        }
        self.ejected = true;
        self.reattaching = false;
        doll.reset_joint_limits(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        for j in self.joints.iter_mut().filter_map(Option::take) {
            b.destroy_joint(j);
        }
        let shapes: Vec<_> = b.body(self.frame).shapes.iter().rev().copied().collect();
        for s in shapes {
            b.set_shape_filter(s, zero);
        }
        for part in [Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2] {
            let s = b.body(doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = false;
            b.refilter(s);
        }
    }
}

/// Lawnmower (guide `c6`, tag Char11): one body of many world-coordinate polygons, two wheels hung on
/// vertical (prismatic) shocks, the rider holding the handle. Up/down drive both wheels (front ×1.76),
/// left/right spin impulses scaled by the current angular velocity, space = shock hop. The blade
/// grinding of bodies (joints created on contact) and `mowerSmash` are NOT ported yet.
pub struct Mower {
    pub body: BodyId,
    pub back_wheel: BodyId,
    pub front_wheel: BodyId,
    back_shock: BodyId,
    front_shock: BodyId,
    back_shock_joint: usize,
    front_shock_joint: usize,
    back_joint: usize,
    front_joint: usize,
    joints: [Option<usize>; 5],
    pub ejected: bool,
    /// `mowerSmash` result: the two halves and the stored polygons (guide, world space) the halves are rebuilt from.
    pub halves: Option<[BodyId; 2]>,
    smash_geo: Vec<Vec<Vec2>>,
    blade_shape: box2d::world::ShapeId,
    clearance_shape: box2d::world::ShapeId,
    /// `bladeCenter`, `bladeX` (half width) and (the body starts at the origin, so world = local at creation).
    blade_center: Vec2,
    blade_x: f64,
    /// `bladeShape.GetVertices()[2].y`: the blade's lower edge in body-local metres (sparks fly from there).
    blade_bottom: f64,
    /// Frame until which the previous `MowerImpact` sound is still playing (`mowerImpactSound`).
    impact_until: u32,
    mower_mass: f64,
    /// Blood sprays of the bodies being ground (`createBloodSpray`): (target body, spray start local, spray end local, particles per step).
    sprays: Vec<(BodyId, Vec2, Vec2, u32)>,
    /// Grinding in progress: (target body, rising body) pairs, new pairs queued by `handleBladeContacts`, and the clearance-sensor contact counts per body.
    pub targets: Vec<(BodyId, BodyId)>,
    added: Vec<usize>,
    contact_count: std::collections::HashMap<BodyId, i32>,
}

impl Mower {
    pub const SMASH_LIMIT: f64 = 200.0;

    /// `mowerLoop` volume: 1 while the wheel motors are on (up/down held), 0.5 otherwise.
    pub fn loop_volume(&self, world: &World) -> f32 {
        let on = matches!(&world.b2().joint(self.back_joint).kind, box2d::joints::JointKind::Revolute(r) if r.enable_motor);
        if on { 1.0 } else { 0.5 }
    }

    /// Blade grinding (original `contactBladeResultHandler` / `handleBladeContacts` / `actions`, deobfuscated.js 31550–31700, 31907–31945). Needs the step's
    /// contact `events`. A Result on the blade shape with a dynamic shape of material bits 1–7 whose contact lies in the middle of the blade (|x| < ½ blade width)
    /// starts grinding it: the shape gets filter (1, 9, −3) and material 0, a sensor "rising" body slides it down the blade along the mower's local y axis
    /// (prismatic, motor −0.5, limit −10…0) and is pinned to it (revolute, torque 5); the pair is removed once nothing touches the clearance sensor any
    /// more. Other blade hits only shove the mower (and the hit body) like the original. NOT ported: `grindShape` side effects on characters/level shapes,
    /// blood/particle sprays, sparks, target masks, stats.
    pub fn grind(&mut self, world: &mut World, events: &[box2d::world::ContactEvent]) {
        use box2d::world::ContactEventKind as K;
        if self.halves.is_some() {
            return;
        }
        // blood sprays of the bodies still being ground: `perFrame` particles per step from the blade edge, thrown over a 180° fan
        for &(target, a, b, per_frame) in &self.sprays {
            if !world.b2().c.bodies[target].alive {
                continue;
            }
            for _ in 0..per_frame {
                let (u, speed, spread) = (world.blood.rand(), world.blood.rand() * 5.0, world.blood.rand() * std::f64::consts::PI);
                let p = Vec2::new(a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u);
                let m = world.b2().body(self.body);
                let rot = (b.y - a.y).atan2(b.x - a.x);
                let ang = m.sweep.a + rot + spread;
                let v = m.linear_velocity_from_local_point(p);
                let w = m.world_point(p);
                world.blood.spawn([w.x * 62.5, w.y * 62.5], [box2d::fdlibm::cos(ang) * speed + v.x, box2d::fdlibm::sin(ang) * speed + v.y]);
            }
        }
        for e in events {
            if !matches!(e.kind, K::Add | K::Remove) || (e.shape1 != self.clearance_shape && e.shape2 != self.clearance_shape) {
                continue;
            }
            let other = if e.shape1 == self.clearance_shape { e.shape2 } else { e.shape1 };
            let body = world.b2().c.shapes[other].body;
            if body == self.body {
                continue;
            }
            *self.contact_count.entry(body).or_insert(0) += if e.kind == K::Add { 1 } else { -1 };
        }
        // strongest Result per other shape
        let mut hits: Vec<(box2d::world::ShapeId, Vec2, f64)> = Vec::new();
        for e in events.iter().filter(|e| e.kind == K::Result && (e.shape1 == self.blade_shape || e.shape2 == self.blade_shape)) {
            let other = if e.shape1 == self.blade_shape { e.shape2 } else { e.shape1 };
            if world.b2().c.shapes[other].shape.density == 0.0 {
                continue;
            }
            match hits.iter_mut().find(|h| h.0 == other) {
                Some(h) if e.normal_impulse > h.2 => *h = (other, e.position, e.normal_impulse),
                Some(_) => {}
                None => hits.push((other, e.position, e.normal_impulse)),
            }
        }
        for (shape, at, _) in hits {
            let (body, mass, material) = {
                let b = world.b2();
                let body = b.c.shapes[shape].body;
                (body, b.body(body).mass, b.c.shapes[shape].shape.material)
            };
            if !world.b2().body(body).alive {
                continue;
            }
            let local = {
                let b = world.b2();
                let l = b.body(self.body).local_point(at);
                Vec2::new(l.x - self.blade_center.x, l.y - self.blade_center.y)
            };
            let o = -local.x / self.blade_x;
            if material & 7 != 0 && o.abs() < 0.5 {
                let _ = self.start_grind(world, shape, body, at);
            } else {
                let angle = world.b2().body(self.body).sweep.a;
                // the original's literals (1.5708, not π/2)
                #[allow(clippy::approx_constant)]
                let a1 = o * 1.2217 - 1.5708 + angle;
                let a2 = a1 - std::f64::consts::PI;
                let b = world.b2_mut();
                // spark burst vector: the second impulse (on the other body) × 5, or the negated mower impulse × 5 against static shapes
                let spark_vel = if mass > 0.0 {
                    let share = mass / (mass + self.mower_mass);
                    b.body_mut(self.body).apply_impulse(Vec2::new(box2d::fdlibm::cos(a1) * 3.0 * share, box2d::fdlibm::sin(a1) * 3.0 * share), at);
                    let (yy, yf) = (box2d::fdlibm::cos(a2) * 3.0 * (1.0 - share), box2d::fdlibm::sin(a2) * 3.0 * (1.0 - share));
                    b.body_mut(body).apply_impulse(Vec2::new(yy, yf), at);
                    Vec2::new(yy * 5.0, yf * 5.0)
                } else {
                    let (yy, yf) = (box2d::fdlibm::cos(a1) * 3.0, box2d::fdlibm::sin(a1) * 3.0);
                    b.body_mut(self.body).apply_impulse(Vec2::new(yy, yf), at);
                    Vec2::new(-yy * 5.0, -yf * 5.0)
                };
                if self.targets.is_empty() && world.frame() >= self.impact_until {
                    // `MowerImpact1..3` plus a 20-spark burst at the blade edge under the hit
                    let name = format!("MowerImpact{}", 1 + world.frame() % 3);
                    self.impact_until = world.frame() + crate::physics::sound_frames(&name);
                    let lx = world.b2().body(self.body).local_point(at).x;
                    let spark_at = world.b2().body(self.body).world_point(Vec2::new(lx, self.blade_bottom));
                    world.play("MowerImpact", 3, at);
                    world.blood.spark_burst(spark_at, spark_vel, 5.0, 50.0, 20);
                }
            }
        }
        // `actions`: drop finished pairs, adopt the new ones
        for i in (0..self.targets.len()).rev() {
            let (target, rising) = self.targets[i];
            if self.contact_count.get(&target).copied().unwrap_or(0) <= 0 {
                if let Some(mut doll) = world.character.take() {
                    self.remove_from_rider(world, &mut doll, target);
                    world.character = Some(doll);
                }
                let b = world.b2_mut();
                b.destroy_body(target);
                b.destroy_body(rising);
                self.targets.remove(i);
                self.sprays.retain(|s| s.0 != target);
            }
        }
        for j in std::mem::take(&mut self.added) {
            let (rising, target) = {
                let jt = world.b2().joint(j);
                (jt.body1, jt.body2)
            };
            self.targets.push((target, rising));
        }
    }

    /// `removeBody` of the rider for a ground body, with the mower's reaction to the joint it breaks (`elbowBreak1` and so on).
    fn remove_from_rider(&mut self, world: &mut World, doll: &mut Ragdoll, target: BodyId) {
        if let Some(limb) = doll.remove_body(world, target) {
            self.limbs_broken(world, doll, &[limb]);
        }
    }

    /// `targetMaskHolder` mask of a body being ground (`contactBladeResultHandler`: a target heavier than 0.1 gets a rect `drawRect(-100, y, 200, 100)` with `y` = the
    /// clearance shape's first vertex minus the mower's local centre, in px, in the mower's frame): (mower centre px, mower angle, `y` px).
    pub fn blade_mask(&self, world: &World, target: BodyId) -> Option<([f64; 2], f64, f64)> {
        if !self.targets.iter().any(|t| t.0 == target) || world.b2().body(target).mass <= 0.1 {
            return None;
        }
        let b = world.b2().body(self.body);
        let v0 = match &world.b2().c.shapes[self.clearance_shape].shape.kind {
            box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices[0],
            _ => return None,
        };
        let y = (v0.y - b.sweep.local_center.y) * 62.5;
        Some(([b.sweep.c.x * 62.5, b.sweep.c.y * 62.5], b.sweep.a, y))
    }

    /// The `GrindLoop2` sound plays while something is being ground.
    pub fn grinding(&self) -> bool {
        !self.targets.is_empty() || !self.added.is_empty()
    }

    fn start_grind(&mut self, world: &mut World, shape: box2d::world::ShapeId, body: BodyId, at: Vec2) -> Option<()> {
        let b = world.b2_mut();
        b.set_shape_filter(shape, Filter { category_bits: 1, mask_bits: 9, group_index: -3 });
        b.shape_mut(shape).material = 0;
        // the refilter's `Remove`s reach the original's counter before it is reset, so they must not arrive afterwards
        let clearance = self.clearance_shape;
        b.c.events.retain(|e| !(e.kind == box2d::world::ContactEventKind::Remove && [e.shape1, e.shape2].contains(&clearance) && [e.shape1, e.shape2].contains(&shape)));
        self.contact_count.insert(body, 0);
        // `grindShape` of the main rider (other ragdolls are not tracked)
        if let Some(mut doll) = world.character.take() {
            doll.grind_shape(world, body);
            world.character = Some(doll);
        }
        world.npc_grind(body);
        let b = world.b2_mut();
        let angle = b.body(self.body).sweep.a;
        let rising = b.create_body(&BodyDef { position: at, angle, ..Default::default() });
        let mut d = ShapeDef::boxed(10.0 / 62.5, 10.0 / 62.5);
        d.density = 5.0;
        d.restitution = 0.1;
        d.friction = 0.1;
        d.is_sensor = true;
        b.create_shape(rising, &d);
        b.set_mass_from_shapes(rising);
        let mut pd = PrismaticJointDef::initialize(b.body(self.body), self.body, b.body(rising), rising, at, Vec2::new(-box2d::fdlibm::sin(angle), box2d::fdlibm::cos(angle)));
        pd.enable_limit = true;
        pd.upper_translation = 0.0;
        pd.lower_translation = -10.0;
        pd.max_motor_force = 100000.0;
        pd.enable_motor = true;
        pd.motor_speed = -0.5;
        b.create_prismatic_joint(&pd);
        let mut rd = RevoluteJointDef::initialize(b.body(rising), rising, b.body(body), body, at);
        rd.enable_motor = true;
        rd.motor_speed = 0.0;
        rd.max_motor_torque = 5.0;
        let rj = b.create_revolute_joint(&rd);
        self.added.push(rj);
        let mass = b.body(body).mass;
        if mass > 0.1 && !world.is_food(body) {
            // `createBloodSpray(mowerBody, (max(bladeLeftX, x − r), bottom), (min(bladeRightX, x + r), bottom), 0, 5, 180°, round(20·D), …)`
            let d = (mass / 0.4).min(1.0);
            let r = d * 1.12 * 0.5;
            let x = world.b2().body(self.body).local_point(at).x;
            let (lo, hi) = (self.blade_center.x - self.blade_x, self.blade_center.x + self.blade_x);
            self.sprays.push((body, Vec2::new(lo.max(x - r), self.blade_bottom), Vec2::new(hi.min(x + r), self.blade_bottom), (20.0 * d).round() as u32));
        }
        Some(())
    }

    /// `mowerSmash` (original 32513, mowing targets not modelled): eject, then the mower body becomes two loose halves — front
    /// (handle, shaft, front) and rear (base, blade, rear, top, back, seat), density 4, zero filter, each launched with the old
    /// body's velocity at the half's local centre (the original passes the half's local centre to the OLD body, reproduced).
    /// The old body and both shock bodies are destroyed and the wheels are re-pinned to the halves (unlimited, torque 100000)
    /// using the shock joints' local anchors. Afterwards `keys` does nothing.
    pub fn smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.halves.is_some() {
            return;
        }
        {
            let at = world.b2().body(self.body).sweep.c;
            world.play("MetalSmashHeavy3", 1, at);
            world.shard_burst(crate::blood::ShardKind::Mower, at, 30.0, 30.0, 70);
        }
        // original start of `mowerSmash`: everything being ground is destroyed with its rising body, sprays stop, the grind loop ends
        for (target, rising) in std::mem::take(&mut self.targets) {
            world.npc_remove_body(target);
            self.remove_from_rider(world, doll, target);
            let b = world.b2_mut();
            b.destroy_body(target);
            b.destroy_body(rising);
        }
        self.sprays.clear();
        self.added.clear();
        self.contact_count.clear();
        self.eject(world, doll);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let (pos, ang, av) = (b.body(self.body).xf.position, b.body(self.body).sweep.a, b.body(self.body).angular_velocity);
        let make = |b: &mut box2d::world::World| {
            let h = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
            b.body_mut(h).angular_velocity = av;
            h
        };
        let front = make(b);
        let rear = make(b);
        let add = |b: &mut box2d::world::World, h: BodyId, verts: Vec<Vec2>| {
            let mut d = ShapeDef::new(Geometry::Polygon(verts));
            d.density = 4.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(h, &d);
        };
        let geo = self.smash_geo.clone();
        for v in &geo[0..3] {
            add(b, front, v.clone());
        }
        b.set_mass_from_shapes(front);
        let lc = b.body(front).sweep.local_center;
        let wp = b.body(self.body).world_point(lc);
        let v = b.body(self.body).linear_velocity_from_world_point(wp);
        b.body_mut(front).linear_velocity = v;
        for v in &geo[3..9] {
            add(b, rear, v.clone());
        }
        b.set_mass_from_shapes(rear);
        let lc = b.body(rear).sweep.local_center;
        let wp = b.body(self.body).world_point(lc);
        let v = b.body(self.body).linear_velocity_from_world_point(wp);
        b.body_mut(rear).linear_velocity = v;
        let anchors = |b: &box2d::world::World, shock: usize, wheel: usize| match (&b.joint(shock).kind, &b.joint(wheel).kind) {
            (box2d::joints::JointKind::Prismatic(p), box2d::joints::JointKind::Revolute(r)) => (p.local_anchor1, r.local_anchor2),
            _ => (Vec2::default(), Vec2::default()),
        };
        let (fa1, fa2) = anchors(b, self.front_shock_joint, self.front_joint);
        let (ba1, ba2) = anchors(b, self.back_shock_joint, self.back_joint);
        b.destroy_body(self.body);
        b.destroy_body(self.front_shock);
        b.destroy_body(self.back_shock);
        let pin = |b: &mut box2d::world::World, half: BodyId, wheel: BodyId, a1: Vec2, a2: Vec2| {
            b.create_revolute_joint(&RevoluteJointDef {
                body1: half,
                body2: wheel,
                local_anchor1: a1,
                local_anchor2: a2,
                reference_angle: 0.0,
                enable_limit: false,
                lower_angle: 0.0,
                upper_angle: 0.0,
                enable_motor: false,
                motor_speed: 0.0,
                max_motor_torque: 100000.0,
                collide_connected: false,
                user_data: 0,
            })
        };
        self.front_joint = pin(b, front, self.front_wheel, fa1, fa2);
        self.back_joint = pin(b, rear, self.back_wheel, ba1, ba2);
        self.halves = Some([front, rear]);
    }

    /// Limb-break reactions (original ~32852): shoulder/elbow breaks drop that hand's joint, hip/knee breaks that foot's;
    /// `checkEject` fires once all four are gone (the pelvis joint, slot 0, does not count).
    pub fn limbs_broken(&mut self, world: &mut World, doll: &mut Ragdoll, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        let b = world.b2_mut();
        for l in broken {
            let slot = match l {
                Limb::Shoulder1 | Limb::Elbow1 => 1,
                Limb::Shoulder2 | Limb::Elbow2 => 2,
                Limb::Hip1 | Limb::Knee1 => 3,
                Limb::Hip2 | Limb::Knee2 => 4,
                _ => continue,
            };
            if let Some(j) = self.joints[slot].take() {
                b.destroy_joint(j);
            }
        }
        if !broken.is_empty() && self.joints[1..].iter().all(Option::is_none) {
            self.eject(world, doll);
        }
    }

    pub fn back_shock(&self) -> BodyId {
        self.back_shock
    }

    pub fn front_shock(&self) -> BodyId {
        self.front_shock
    }

    const MAX_SPEED: f64 = 15.0;
    const ACCEL: f64 = 3.0;
    const MAX_SPIN: f64 = 3.5;
    const VERTICAL: f64 = 20.0 / 62.5;

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let poly = |prefix: &str, n: usize| -> Option<Vec<Vec2>> { (1..=n).map(|i| at(&format!("{prefix}{i}"))).collect() };
        let boxed = |name: &str| -> Option<Vec<Vec2>> {
            let g = guide.get(name)?;
            Some(oriented_box(g[3] * 5.0 / CHARACTER_SCALE, g[4] * 5.0 / CHARACTER_SCALE, at(name)?, g[2] / (180.0 / std::f64::consts::PI)))
        };
        let deg = 180.0 / std::f64::consts::PI;
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: -2 };
        for part in [Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2] {
            let s = world.b2().body(doll.bodies[&part]).shapes[0];
            world.b2_mut().shape_mut(s).is_sensor = true;
            world.b2_mut().refilter(s);
        }
        let b = world.b2_mut();
        let body = b.create_body(&BodyDef::default());
        let add = |b: &mut box2d::world::World, verts: Vec<Vec2>, density: f64, friction: f64, filter: Filter, sensor: bool| {
            let mut d = ShapeDef::new(Geometry::Polygon(verts));
            d.density = density;
            d.friction = friction;
            d.restitution = 0.1;
            d.filter = filter;
            d.is_sensor = sensor;
            b.create_shape(body, &d)
        };
        add(b, poly("handleVert", 4)?, 4.0, 0.3, zero, false);
        add(b, boxed("shaftShape")?, 4.0, 0.3, zero, false);
        add(b, poly("frontVert", 6)?, 4.0, 0.3, zero, false);
        add(b, boxed("baseShape")?, 4.0, 0.3, zero, false);
        let blade_shape = add(b, boxed("bladeShape")?, 4.0, 0.3, zero, false);
        add(b, poly("rearVert", 5)?, 4.0, 0.3, zero, false);
        add(b, poly("topVert", 5)?, 4.0, 0.3, zero, false);
        add(b, poly("backVert", 4)?, 4.0, 0.3, zero, false);
        // `seatShape2` is created and destroyed again (it still takes and frees a broad-phase proxy, which changes later pair order)
        {
            let mut d = ShapeDef::new(Geometry::Polygon(poly("seatVert", 4)?));
            d.density = 4.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            let sh = b.create_shape(body, &d);
            b.destroy_shape(sh);
        }
        let align = Filter { category_bits: 1, mask_bits: 1, group_index: 0 };
        add(b, boxed("alignShape1")?, 0.01, 0.01, align, false);
        add(b, boxed("alignShape2")?, 0.01, 0.01, align, false);
        let clearance_shape = add(b, boxed("clearanceShape")?, 0.01, 0.01, align, true);
        b.set_mass_from_shapes(body);
        let mower_mass = b.body(body).mass;
        let gb = guide.get("bladeShape")?;
        let blade_center = at("bladeShape")?;
        let blade_x = gb[3] * 5.0 / CHARACTER_SCALE;
        let blade_bottom = blade_center.y + gb[4] * 5.0 / CHARACTER_SCALE;
        let wheel = |b: &mut box2d::world::World, name: &str| -> Option<BodyId> {
            let g = guide.get(name)?;
            let w = b.create_body(&BodyDef { position: at(name)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: g[5] / 2.0 / CHARACTER_SCALE });
            d.density = 5.0;
            d.friction = 1.0;
            d.restitution = 0.3;
            d.filter = zero;
            b.create_shape(w, &d);
            b.set_mass_from_shapes(w);
            Some(w)
        };
        let back_wheel = wheel(b, "backWheelShape")?;
        let front_wheel = wheel(b, "frontWheelShape")?;
        let shock = |b: &mut box2d::world::World, name: &str| -> Option<BodyId> {
            let s = b.create_body(&BodyDef::default());
            let mut d = ShapeDef::new(Geometry::Polygon(oriented_box(12.0 / CHARACTER_SCALE, 12.0 / CHARACTER_SCALE, at(name)?, 0.0)));
            d.density = 4.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(s, &d);
            b.set_mass_from_shapes(s);
            Some(s)
        };
        let front_shock = shock(b, "frontWheelShape")?;
        let back_shock = shock(b, "backWheelShape")?;
        doll.finish_joints(world);
        doll.set_limits(world, Limb::Neck, -10.0 / deg, 10.0 / deg);
        let b = world.b2_mut();
        let prismatic = |b: &mut box2d::world::World, shock: BodyId, name: &str| -> Option<usize> {
            let mut d = box2d::joints::PrismaticJointDef::initialize(b.body(body), body, b.body(shock), shock, at(name)?, Vec2::new(0.0, 1.0));
            d.max_motor_force = 1000.0;
            d.enable_limit = true;
            d.lower_translation = 0.0;
            d.upper_translation = 0.0;
            Some(b.create_prismatic_joint(&d))
        };
        let front_shock_joint = prismatic(b, front_shock, "frontWheelShape")?;
        let back_shock_joint = prismatic(b, back_shock, "backWheelShape")?;
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 100000.0;
            b.create_revolute_joint(&d)
        };
        let pelvis = doll.bodies[&Part::Pelvis];
        let pc = b.body(pelvis).sweep.c;
        let frame_pelvis = rev(b, body, pelvis, pc);
        let back_joint = rev(b, back_shock, back_wheel, at("backWheelShape")?);
        let front_joint = rev(b, front_shock, front_wheel, at("frontWheelShape")?);
        let hand = at("handleAnchor")?;
        let hand1 = rev(b, body, doll.bodies[&Part::LowerArm1], hand);
        let hand2 = rev(b, body, doll.bodies[&Part::LowerArm2], hand);
        let foot = at("footAnchor")?;
        let foot1 = rev(b, body, doll.bodies[&Part::LowerLeg1], foot);
        let foot2 = rev(b, body, doll.bodies[&Part::LowerLeg2], foot);
        doll.poses = PoseSet::Mower;
        // polygons used by `mowerSmash`: handle, shaft, front | base, blade, rear, top, back (seatShape1), seat (seatShape2)
        let smash_geo: Option<Vec<Vec<Vec2>>> = (|| {
            Some(vec![poly("handleVert", 4)?, boxed("shaftShape")?, poly("frontVert", 6)?, boxed("baseShape")?, boxed("bladeShape")?, poly("rearVert", 5)?, poly("topVert", 5)?, poly("backVert", 4)?, poly("seatVert", 4)?])
        })();
        Some(Self {
            body,
            back_wheel,
            front_wheel,
            back_shock,
            front_shock,
            back_shock_joint,
            front_shock_joint,
            back_joint,
            front_joint,
            joints: [Some(frame_pelvis), Some(hand1), Some(hand2), Some(foot1), Some(foot2)],
            ejected: false,
            halves: None,
            smash_geo: smash_geo?,
            blade_shape,
            clearance_shape,
            blade_center,
            blade_x,
            blade_bottom,
            sprays: Vec::new(),
            impact_until: 0,
            mower_mass,
            targets: Vec::new(),
            added: Vec::new(),
            contact_count: std::collections::HashMap::new(),
        })
    }

    fn rev_mut(world: &mut World, id: usize) -> Option<&mut box2d::joints::Revolute> {
        match &mut world.b2_mut().joint_mut(id).kind {
            box2d::joints::JointKind::Revolute(r) => Some(r),
            _ => None,
        }
    }

    fn shock_mut(world: &mut World, id: usize) -> Option<&mut box2d::joints::Prismatic> {
        match &mut world.b2_mut().joint_mut(id).kind {
            box2d::joints::JointKind::Prismatic(p) => Some(p),
            _ => None,
        }
    }

    fn both_shocks(&self, world: &mut World, motor: Option<bool>, limits: Option<(f64, f64)>, speed: f64) {
        for id in [self.front_shock_joint, self.back_shock_joint] {
            if let Some(p) = Self::shock_mut(world, id) {
                if let Some(m) = motor {
                    p.enable_motor = m;
                }
                if let Some((lo, hi)) = limits {
                    p.lower_translation = lo;
                    p.upper_translation = hi;
                }
                p.motor_speed = speed;
            }
        }
    }

    fn hop(&self, world: &mut World, pressed: bool) {
        let back = world.b2().prismatic_translation(self.back_shock_joint).unwrap_or(0.0);
        let front = world.b2().prismatic_translation(self.front_shock_joint).unwrap_or(0.0);
        let v = Self::VERTICAL;
        let Some(p) = Self::shock_mut(world, self.back_shock_joint) else { return };
        let (enabled, speed, upper) = (p.enable_motor, p.motor_speed, p.upper_translation);
        if enabled {
            if speed > 0.0 {
                if pressed && back > v {
                    self.both_shocks(world, Some(false), Some((v - 0.01, v)), 0.0);
                } else if !pressed && front > v {
                    self.both_shocks(world, None, None, -1.0);
                }
            } else if speed < 0.0 && back < 0.0 {
                self.both_shocks(world, Some(false), Some((0.0, 0.0)), 0.0);
            }
        } else if pressed && upper != v {
            self.both_shocks(world, Some(true), Some((0.0, v)), 2.5);
            let at = world.b2().body(self.back_wheel).sweep.c;
            world.play("SegwayJump", 1, at);
        } else if !pressed && upper != 0.0 {
            self.both_shocks(world, Some(true), Some((0.0, v)), -1.0);
        }
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        if self.halves.is_some() {
            // the vehicle ignores keys, but the rider's pose motors (`checkPose`) keep running
            doll.apply_pose(world);
            return;
        }
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            match (k.left, k.right) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.left { 1 } else { 2 }),
                (true, false) | (false, true) => {
                    let left = k.left;
                    let b = world.b2_mut();
                    let (angle, av) = (b.body(self.body).sweep.a, b.body(self.body).angular_velocity);
                    let scale = if left { (av + Self::MAX_SPIN) / Self::MAX_SPIN } else { (av - Self::MAX_SPIN) / -Self::MAX_SPIN };
                    let scale = scale.clamp(0.0, 1.0);
                    let imp = if left { 2.4 } else { 2.8 };
                    let (c, s) = (box2d::fdlibm::cos(angle) * imp * scale, box2d::fdlibm::sin(angle) * imp * scale);
                    let lc = b.body(self.body).sweep.local_center;
                    let near = b.body(self.body).world_point(Vec2::new(lc.x + 1.0, lc.y));
                    let far = b.body(self.body).world_point(Vec2::new(lc.x - 1.0, lc.y));
                    let (p1, p2) = if left { (Vec2::new(s, -c), Vec2::new(-s, c)) } else { (Vec2::new(-s, c), Vec2::new(s, -c)) };
                    b.body_mut(self.body).apply_impulse(p1, near);
                    b.body_mut(self.body).apply_impulse(p2, far);
                    doll.set_pose(if left { 7 } else { 8 });
                }
                _ if ejected && (doll.pose == 1 || doll.pose == 2) => doll.set_pose(0),
                _ => {}
            }
            match (k.up, k.down) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.up { 3 } else { 4 }),
                (true, false) | (false, true) => {
                    let dir = if k.up { 1.0 } else { -1.0 };
                    if !Self::rev_mut(world, self.back_joint).is_some_and(|r| r.enable_motor) {
                        for id in [self.back_joint, self.front_joint] {
                            if let Some(r) = Self::rev_mut(world, id) {
                                r.enable_motor = true;
                            }
                        }
                    }
                    let b = world.b2();
                    let cur = b.body(self.back_wheel).angular_velocity - b.body(self.back_shock).angular_velocity;
                    let new = if cur * dir < 0.0 {
                        0.0
                    } else if cur * dir < Self::MAX_SPEED {
                        cur + dir * Self::ACCEL
                    } else {
                        cur
                    };
                    if let Some(r) = Self::rev_mut(world, self.back_joint) {
                        r.motor_speed = new;
                    }
                    if let Some(r) = Self::rev_mut(world, self.front_joint) {
                        r.motor_speed = new * 1.76;
                    }
                }
                _ => {
                    if Self::rev_mut(world, self.back_joint).is_some_and(|r| r.enable_motor) {
                        for id in [self.back_joint, self.front_joint] {
                            if let Some(r) = Self::rev_mut(world, id) {
                                r.enable_motor = false;
                            }
                        }
                    }
                    if ejected && (doll.pose == 3 || doll.pose == 4) {
                        doll.set_pose(0);
                    }
                }
            }
            if ejected {
                doll.set_grab(world, k.space);
                if k.shift {
                    doll.set_pose(6);
                } else if doll.pose == 6 {
                    doll.set_pose(0);
                }
                if k.ctrl {
                    doll.set_pose(7);
                } else if doll.pose == 7 {
                    doll.set_pose(0);
                }
            } else {
                self.hop(world, k.space);
                if !k.shift && doll.pose == 6 {
                    doll.set_pose(0);
                }
                if !k.ctrl && doll.pose == 7 {
                    doll.set_pose(0);
                }
            }
            if k.z {
                self.eject(world, doll);
            }
        }
        doll.apply_pose(world);
    }

    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        doll.reset_joint_limits(world);
        let b = world.b2_mut();
        for j in self.joints.iter_mut().filter_map(Option::take) {
            b.destroy_joint(j);
        }
        for part in [Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2] {
            let s = b.body(doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = false;
            b.refilter(s);
        }
        self.both_shocks(world, Some(false), Some((0.0, 0.0)), 0.0);
        let b = world.b2_mut();
        let a = b.body(self.body).sweep.a - std::f64::consts::FRAC_PI_2;
        let imp = Vec2::new(box2d::fdlibm::cos(a) * 5.0, box2d::fdlibm::sin(a) * 5.0);
        for part in [Part::Chest, Part::Pelvis] {
            let id = doll.bodies[&part];
            let c = b.body(id).sweep.c;
            b.body_mut(id).apply_impulse(imp, c);
        }
    }
}

/// Pogo stick (guide `c9`, tag Char12): a frame and a rod on a prismatic spring (`pogoJoint`), rider holding the
/// handle. Up/down charge and release the spring towards a lean angle, space = charged high jump, left/right
/// lean impulses (air) or angular-velocity steering (neither key, uses the centre-of-mass velocity).
pub struct Pogo {
    pub frame: BodyId,
    pub rod: BodyId,
    pogo: usize,
    joints: [Option<usize>; 4],
    /// Creation defs of the hand and foot joints and their points on the frame, to climb on again.
    defs: [RevoluteJointDef; 4],
    anchors: [Vec2; 2],
    reattaching: bool,
    com: Vec<BodyId>,
    prev_com: Vec2,
    pivot_angle: f64,
    target_angle: f64,
    charging: bool,
    jump_frames: u32,
    relaxing: bool,
    pub ejected: bool,
    /// Pieces left by `frameSmash` (upper half, lower half, spring) and the flag.
    pub pieces: Vec<BodyId>,
    pub frame_smashed: bool,
}

impl Pogo {
    pub const FRAME_SMASH_LIMIT: f64 = 40.0;

    /// `frameSmash` (original 35126, limit 40): the prismatic joint is destroyed, the rider ejected, and the frame becomes three
    /// loose boxes — upper and lower half (10×31.25 px, density 5, friction 1) and a thin spring (4 px wide, density 0.5) — each
    /// moving with the frame's velocity at its own point; the frame body is destroyed. `keys` does nothing afterwards.
    pub fn frame_smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.frame_smashed {
            return;
        }
        self.frame_smashed = true;
        {
            let at = world.b2().body(self.frame).sweep.c;
            world.play("PogoFrameSmash", 1, at);
        }
        world.b2_mut().destroy_joint(self.pogo);
        self.eject(world, doll);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let (pos, ang, av) = (b.body(self.frame).xf.position, b.body(self.frame).sweep.a, b.body(self.frame).angular_velocity);
        let ir = 31.25 / CHARACTER_SCALE;
        for (hx, cy, density, sample) in [(10.0, -ir, 5.0, Some(-ir)), (10.0, ir, 5.0, Some(ir)), (4.0, 0.0, 0.5, None)] {
            let h = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Polygon(oriented_box(hx / CHARACTER_SCALE, ir, Vec2::new(0.0, cy), 0.0)));
            d.density = density;
            d.friction = 1.0;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(h, &d);
            b.set_mass_from_shapes(h);
            b.body_mut(h).angular_velocity = av;
            let v = match sample {
                Some(y) => {
                    let wp = b.body(self.frame).world_point(Vec2::new(0.0, y));
                    b.body(self.frame).linear_velocity_from_world_point(wp)
                }
                None => b.body(self.frame).linear_velocity,
            };
            b.body_mut(h).linear_velocity = v;
            self.pieces.push(h);
        }
        b.destroy_body(self.frame);
    }

    /// Limb-break reactions (original 35269..35380): arms let go of the handle (both gone → eject), legs come off the foot
    /// pegs; with a leg broken on each side the frame and the tip start colliding with the rider. The broken parts leave
    /// the balance's centre of mass, torn-off upper limbs join it.
    pub fn limbs_broken(&mut self, world: &mut World, doll: &mut Ragdoll, broken: &[Limb]) {
        for &limb in broken {
            if self.ejected {
                return;
            }
            let (slot, parts, torn): (usize, &[Part], bool) = match limb {
                Limb::Shoulder1 => (0, &[Part::UpperArm1, Part::LowerArm1], true),
                Limb::Shoulder2 => (1, &[Part::UpperArm2, Part::LowerArm2], true),
                Limb::Elbow1 => (0, &[Part::LowerArm1], false),
                Limb::Elbow2 => (1, &[Part::LowerArm2], false),
                Limb::Hip1 => (2, &[Part::UpperLeg1, Part::LowerLeg1], true),
                Limb::Hip2 => (3, &[Part::UpperLeg2, Part::LowerLeg2], true),
                Limb::Knee1 => (2, &[Part::LowerLeg1], false),
                Limb::Knee2 => (3, &[Part::LowerLeg2], false),
                _ => continue,
            };
            let gone: Vec<BodyId> = parts.iter().map(|p| doll.bodies[p]).collect();
            self.com.retain(|id| !gone.contains(id));
            if torn {
                self.com.extend(doll.sev.iter().zip(&doll.sev_limbs).filter(|&(_, &l)| l == limb).map(|(&b, _)| b));
            }
            let Some(j) = self.joints[slot].take() else { continue };
            world.b2_mut().destroy_joint(j);
            if slot < 2 {
                if self.joints[0].is_none() && self.joints[1].is_none() {
                    self.eject(world, doll);
                }
            } else {
                self.free_if_legless(world, doll);
            }
        }
    }

    /// While riding: elbows break at 90 (ligament below 110), knees at 120 (below 140), whatever the mass.
    const BREAK_LIMITS: [f64; 4] = [90.0, 110.0, 120.0, 140.0];
    const SCALE: f64 = CHARACTER_SCALE;
    const MAX_SPIN: f64 = 5.0;
    const UPRIGHT: f64 = -std::f64::consts::PI * 0.5;
    const FORWARD: f64 = -std::f64::consts::PI * 0.45;
    const REVERSE: f64 = -std::f64::consts::PI * 0.6;
    const BOUNCE: f64 = -30.0 / Self::SCALE;
    const JUMP: f64 = -40.0 / Self::SCALE;
    const RETRACT: f64 = -0.5;

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let deg = 180.0 / std::f64::consts::PI;
        let default_filter = Filter { category_bits: 260, mask_bits: 270, group_index: -1 };
        let b = world.b2_mut();
        let make = |b: &mut box2d::world::World, name: &str| -> Option<(BodyId, f64)> {
            let g = guide.get(name)?;
            let body = b.create_body(&BodyDef { position: at(name)?, angle: g[2] / deg, ..Default::default() });
            let (hx, hy) = (g[3] * 50.0 / Self::SCALE, g[4] * 50.0 / Self::SCALE);
            let mut d = ShapeDef::boxed(hx, hy);
            d.density = 5.0;
            d.friction = 1.0;
            d.restitution = 0.1;
            d.filter = default_filter;
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            Some((body, hy))
        };
        let (frame, _) = make(b, "frameShape")?;
        let (rod, rod_hy) = make(b, "rodShape")?;
        // The nub is added AFTER the mass was computed and the mass is not recomputed (as in the original).
        let mut nub = ShapeDef::new(Geometry::Circle { local_position: Vec2::new(0.0, rod_hy), radius: 7.0 / Self::SCALE });
        nub.density = 5.0;
        nub.friction = 1.0;
        nub.restitution = 0.3;
        nub.filter = default_filter;
        let nub_shape = b.create_shape(rod, &nub);
        doll.finish_joints(world);
        for (limb, lo, hi) in [
            (Limb::Hip1, -100.0, 0.0),
            (Limb::Hip2, -100.0, 0.0),
            (Limb::Shoulder1, -120.0, 0.0),
            (Limb::Shoulder2, -120.0, 0.0),
            (Limb::Elbow1, -60.0, 0.0),
            (Limb::Elbow2, -60.0, 0.0),
            (Limb::Neck, 0.0, 5.0),
        ] {
            doll.set_limits(world, limb, lo / deg, hi / deg);
        }
        let b = world.b2_mut();
        let frame_angle = b.body(frame).sweep.a;
        let mut pd = box2d::joints::PrismaticJointDef::initialize(b.body(frame), frame, b.body(rod), rod, at("frameShape")?, Vec2::new(-box2d::fdlibm::sin(frame_angle), box2d::fdlibm::cos(frame_angle)));
        pd.max_motor_force = 10000.0;
        pd.enable_limit = true;
        pd.lower_translation = 0.0;
        pd.upper_translation = 0.0;
        pd.enable_motor = true;
        pd.motor_speed = 0.0;
        let pogo = b.create_prismatic_joint(&pd);
        let def = |b: &mut box2d::world::World, b2: BodyId, p: Vec2, lo: f64, hi: f64| {
            let mut d = RevoluteJointDef::initialize(b.body(frame), frame, b.body(b2), b2, p);
            d.enable_limit = true;
            d.lower_angle = lo / deg;
            d.upper_angle = hi / deg;
            d
        };
        let hand = at("handleAnchor")?;
        let foot = at("footAnchor")?;
        let defs = [
            def(b, doll.bodies[&Part::LowerArm1], hand, -100.0, 10.0),
            def(b, doll.bodies[&Part::LowerArm2], hand, -100.0, 10.0),
            def(b, doll.bodies[&Part::LowerLeg1], foot, -10.0, 10.0),
            def(b, doll.bodies[&Part::LowerLeg2], foot, -10.0, 10.0),
        ];
        let [hand1, hand2, foot1, foot2] = [0, 1, 2, 3].map(|k| b.create_revolute_joint(&defs[k]));
        let anchors = [hand, foot].map(|p| b.body(frame).local_point(p));
        doll.limit_override = Some(Self::BREAK_LIMITS);
        let com: Vec<BodyId> = [Part::Head, Part::Chest, Part::Pelvis, Part::UpperArm1, Part::LowerArm1, Part::UpperArm2, Part::LowerArm2, Part::UpperLeg1, Part::LowerLeg1, Part::UpperLeg2, Part::LowerLeg2]
            .iter()
            .map(|p| doll.bodies[p])
            .collect();
        let mut s = Self {
            frame,
            rod,
            pogo,
            joints: [Some(hand1), Some(hand2), Some(foot1), Some(foot2)],
            defs,
            anchors,
            reattaching: false,
            com: [vec![frame, rod], com].concat(),
            prev_com: Vec2::default(),
            pivot_angle: f64::NAN,
            target_angle: Self::UPRIGHT,
            charging: false,
            jump_frames: 0,
            relaxing: false,
            ejected: false,
            pieces: Vec::new(),
            frame_smashed: false,
        };
        s.prev_com = s.center_of_mass(world);
        doll.poses = PoseSet::Pogo;
        doll.pose = 9;
        world.nub = crate::physics::NubState { shape: Some(nub_shape), body: Some(rod), contacts: 0, air_lean: false };
        Some(s)
    }

    fn center_of_mass(&self, world: &World) -> Vec2 {
        let b = world.b2();
        let (mut x, mut y, mut m) = (0.0, 0.0, 0.0);
        for &id in &self.com {
            let body = b.body(id);
            x += body.sweep.c.x * body.mass;
            y += body.sweep.c.y * body.mass;
            m += body.mass;
        }
        Vec2::new(x * (1.0 / m), y * (1.0 / m))
    }

    /// `paint`: the frame clip's spring is squeezed to 0.45 of its length as the stick compresses by 20 px.
    pub fn spring_scale(&self, world: &World) -> f64 {
        let squeeze = (world.b2().prismatic_translation(self.pogo).unwrap_or(0.0) * 62.5 / -20.0).clamp(0.0, 1.0);
        (1.0 - squeeze) * 0.55 + 0.45
    }

    fn pogo_mut(world: &mut World, id: usize) -> Option<&mut box2d::joints::Prismatic> {
        match &mut world.b2_mut().joint_mut(id).kind {
            box2d::joints::JointKind::Prismatic(p) => Some(p),
            _ => None,
        }
    }

    /// `upPressedActions` / `downPressedActions`: charge (retract) then release (bounce) the spring.
    fn bounce(&mut self, world: &mut World, up: bool) {
        self.target_angle = if up { Self::FORWARD } else { Self::REVERSE };
        world.nub.air_lean = false;
        if self.charging {
            return;
        }
        let translation = world.b2().prismatic_translation(self.pogo).unwrap_or(0.0);
        let contacts = world.nub.contacts;
        let (jump_frames, pivot, target) = (self.jump_frames, self.pivot_angle, self.target_angle);
        let mut released = false;
        let Some(p) = Self::pogo_mut(world, self.pogo) else { return };
        let speed = p.motor_speed;
        let in_window = if up { pivot > target && pivot < target + std::f64::consts::PI * 0.15 } else { pivot < target && pivot > target - std::f64::consts::PI * 0.15 };
        if contacts > 0 && jump_frames == 0 {
            if p.lower_translation == 0.0 {
                p.lower_translation = Self::BOUNCE;
                p.motor_speed = Self::RETRACT;
            } else if speed == Self::RETRACT && translation <= Self::BOUNCE && in_window {
                p.motor_speed = 6.0;
                self.jump_frames = 5;
                released = true;
            }
        }
        if speed > 0.0 && translation >= 0.0 {
            p.lower_translation = 0.0;
            p.motor_speed = 0.0;
        }
        if released {
            // `playAreaSoundInstance("PogoRelease", frameBody, min(1, max(-translation·scale / 40, 0)))`
            let at = world.b2().body(self.frame).sweep.c;
            world.play_scaled("PogoRelease", at, (-translation * 62.5 / 40.0).clamp(0.0, 1.0));
        }
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            match (k.left, k.right) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.left { 1 } else { 2 }),
                (true, false) | (false, true) => {
                    if world.nub.contacts == 0 {
                        world.nub.air_lean = true;
                    }
                    let left = k.left;
                    let b = world.b2_mut();
                    let (angle, av) = (b.body(self.frame).sweep.a, b.body(self.frame).angular_velocity);
                    let f = if left { (av + Self::MAX_SPIN) / Self::MAX_SPIN } else { (av - Self::MAX_SPIN) / -Self::MAX_SPIN };
                    let f = f.clamp(0.0, 1.0);
                    let (c, s) = (box2d::fdlibm::cos(angle) * 0.5 * f, box2d::fdlibm::sin(angle) * 0.5 * f);
                    let lc = b.body(self.frame).sweep.local_center;
                    let top = b.body(self.frame).world_point(Vec2::new(lc.x, lc.y - 1.0));
                    let bottom = b.body(self.frame).world_point(Vec2::new(lc.x, lc.y + 1.0));
                    let (it, ib) = if left { (Vec2::new(-c, -s), Vec2::new(c, s)) } else { (Vec2::new(c, s), Vec2::new(-c, -s)) };
                    b.body_mut(self.frame).apply_impulse(it, top);
                    b.body_mut(self.frame).apply_impulse(ib, bottom);
                }
                _ if ejected => {
                    if doll.pose == 1 || doll.pose == 2 {
                        doll.set_pose(0);
                    }
                }
                _ => self.steer(world),
            }
            match (k.up, k.down) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.up { 3 } else { 4 }),
                (true, false) | (false, true) => self.bounce(world, k.up),
                _ if ejected => {
                    if doll.pose == 3 || doll.pose == 4 {
                        doll.set_pose(0);
                    }
                }
                _ => {
                    self.target_angle = Self::UPRIGHT;
                    if !self.charging {
                        let translation = world.b2().prismatic_translation(self.pogo).unwrap_or(0.0);
                        if let Some(p) = Self::pogo_mut(world, self.pogo) {
                            if p.lower_translation != 0.0 {
                                if translation < 0.0 {
                                    p.motor_speed = 0.5;
                                } else {
                                    p.motor_speed = 0.0;
                                    p.lower_translation = 0.0;
                                }
                            }
                        }
                    }
                }
            }
            if k.space {
                if ejected {
                    doll.set_grab(world, true);
                } else {
                    self.charging = true;
                    let translation = world.b2().prismatic_translation(self.pogo).unwrap_or(0.0);
                    if let Some(p) = Self::pogo_mut(world, self.pogo) {
                        if p.lower_translation != Self::JUMP {
                            p.lower_translation = Self::JUMP;
                        }
                        p.motor_speed = if translation > p.lower_translation { Self::RETRACT } else { 0.0 };
                    }
                }
            } else if ejected {
                doll.set_grab(world, false);
            } else if self.charging {
                let translation = world.b2().prismatic_translation(self.pogo).unwrap_or(0.0);
                let mut done = false;
                let mut release_volume = None;
                if let Some(p) = Self::pogo_mut(world, self.pogo) {
                    if translation < 0.0 {
                        if translation * -70.0 > p.motor_speed {
                            release_volume = Some((-translation * 62.5 / 20.0).clamp(0.0, 1.0));
                        }
                        p.motor_speed = translation * -70.0;
                    } else {
                        p.motor_speed = 0.0;
                        p.lower_translation = 0.0;
                        done = true;
                    }
                }
                if let Some(volume) = release_volume {
                    let at = world.b2().body(self.frame).sweep.c;
                    world.play_scaled("PogoRelease", at, volume);
                }
                if done {
                    self.charging = false;
                    self.jump_frames = 5;
                }
            }
            if k.shift {
                doll.set_pose(if ejected { 7 } else { 5 });
            } else if ejected {
                if doll.pose == 7 {
                    doll.set_pose(0);
                }
            } else if doll.pose == 5 {
                doll.set_pose(9);
            }
            if k.ctrl {
                doll.set_pose(if ejected { 8 } else { 6 });
            } else if ejected {
                if doll.pose == 8 {
                    doll.set_pose(0);
                }
            } else if doll.pose == 6 {
                doll.set_pose(9);
            }
            if k.z {
                self.eject(world, doll);
            }
        }
        // actions: relaxPogo (after eject), checkPose, jump frame countdown
        if self.relaxing {
            let translation = world.b2().prismatic_translation(self.pogo).unwrap_or(0.0);
            let mut done = false;
            if let Some(p) = Self::pogo_mut(world, self.pogo) {
                if translation < 0.0 {
                    p.motor_speed = translation * -70.0;
                } else {
                    p.motor_speed = 0.0;
                    p.lower_translation = 0.0;
                    done = true;
                }
            }
            if done {
                self.jump_frames = 5;
                self.relaxing = false;
            }
        }
        doll.apply_pose(world);
        // `actions` runs after `checkPose`: the riding pose comes back on the next frame
        if self.reattaching {
            self.re_attaching(world, doll);
        }
        if self.jump_frames > 0 {
            self.jump_frames -= 1;
        }
    }

    /// `leftAndRightActions` while seated: steer the frame's angular velocity towards the target angle.
    fn steer(&mut self, world: &mut World) {
        let curr = self.center_of_mass(world);
        let vel = Vec2::new(curr.x - self.prev_com.x, curr.y - self.prev_com.y);
        self.prev_com = curr;
        let vel_angle = vel.y.atan2(vel.x);
        let tip = world.b2().body(self.rod).world_point(Vec2::new(0.0, 62.5 / Self::SCALE));
        self.pivot_angle = (curr.y - tip.y).atan2(curr.x - tip.x);
        if world.nub.air_lean {
            return;
        }
        let pi = std::f64::consts::PI;
        let q = if vel.y > 0.0 {
            let t = (vel.y / 0.25).min(1.0);
            self.target_angle * (1.0 - t) + (vel_angle - pi) * t
        } else {
            let t = (-vel.y / 0.25).min(1.0);
            self.target_angle * (1.0 - t) + vel_angle * t
        };
        let mut d = q - self.pivot_angle;
        if d > pi {
            d -= pi * 2.0;
        }
        if d < -pi {
            d += pi * 2.0;
        }
        let w = (d * 30.0).clamp(-Self::MAX_SPIN * 2.0, Self::MAX_SPIN * 2.0);
        world.b2_mut().body_mut(self.frame).angular_velocity = w;
    }

    /// `reAttach` (original 35609): the rider's riding limits and break limits again, frame and rod collide with the rider, that hand's joint,
    /// `reAttaching` started.
    fn board(&mut self, world: &mut World, doll: &mut Ragdoll, k: usize, tip: Vec2) -> bool {
        if !self.ejected || self.frame_smashed || doll.dead || !near(tip, world.b2().body(self.frame).world_point(self.anchors[0]), 0.25) {
            return false;
        }
        let deg = 180.0 / std::f64::consts::PI;
        self.relaxing = false;
        self.ejected = false;
        doll.set_pose(0);
        doll.set_grab(world, false);
        doll.limit_override = Some(Self::BREAK_LIMITS);
        for (limb, lo, hi) in [
            (Limb::Hip1, -100.0, 0.0),
            (Limb::Hip2, -100.0, 0.0),
            (Limb::Shoulder1, -120.0, 0.0),
            (Limb::Shoulder2, -120.0, 0.0),
            (Limb::Elbow1, -60.0, 0.0),
            (Limb::Elbow2, -60.0, 0.0),
            (Limb::Neck, 0.0, 5.0),
        ] {
            if !doll.broken.contains(&limb) {
                doll.set_limits_now(world, limb, lo / deg, hi / deg);
            }
        }
        let mounted = Filter { category_bits: 260, mask_bits: 270, group_index: -1 };
        let b = world.b2_mut();
        let shapes = [b.body(self.frame).shapes[0], b.body(self.rod).shapes[0]].map(|s| (s, mounted));
        b.set_shape_filters(&shapes);
        self.free_if_legless(world, doll);
        self.joints[k] = Some(world.b2_mut().create_revolute_joint(&self.defs[k].clone()));
        self.reattaching = true;
        true
    }

    /// `checkLegsBroken`: with both legs broken the frame and the nub collide with the rider's parts again.
    fn free_if_legless(&self, world: &mut World, doll: &Ragdoll) {
        let off = |a, b| doll.broken.contains(&a) || doll.broken.contains(&b);
        if off(Limb::Hip1, Limb::Knee1) && off(Limb::Hip2, Limb::Knee2) {
            let free = Filter { category_bits: 260, mask_bits: 270, group_index: 0 };
            let b = world.b2_mut();
            let shapes = [b.body(self.frame).shapes[0], b.body(self.rod).shapes[1]].map(|s| (s, free));
            b.set_shape_filters(&shapes);
        }
    }

    /// The `reAttaching` action (original `reAttaching` of the pogo rider): hands within 0.25 of the handle, feet within 0.2 of the foot
    /// rest; once all four hold or cannot, the action ends and the riding pose (9) returns.
    fn re_attaching(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected || doll.dead {
            self.reattaching = false;
            return;
        }
        let limbs = [(Part::LowerArm1, Limb::Shoulder1, Limb::Elbow1), (Part::LowerArm2, Limb::Shoulder2, Limb::Elbow2), (Part::LowerLeg1, Limb::Hip1, Limb::Knee1), (Part::LowerLeg2, Limb::Hip2, Limb::Knee2)];
        let mut n = 0;
        for (slot, (part, root, middle)) in limbs.into_iter().enumerate() {
            if self.joints[slot].is_some() || doll.broken.contains(&middle) || doll.broken.contains(&root) {
                n += 1;
                continue;
            }
            let (anchor, reach) = (self.anchors[slot / 2], if slot < 2 { 0.25 } else { 0.2 });
            if near(Segway::limb_tip(world, doll, part), world.b2().body(self.frame).world_point(anchor), reach) {
                self.joints[slot] = Some(world.b2_mut().create_revolute_joint(&self.defs[slot].clone()));
                n += 1;
            }
        }
        if n >= 4 {
            self.reattaching = false;
            doll.pose = 9;
        }
    }

    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        self.reattaching = false;
        doll.limit_override = None;
        self.relaxing = true;
        if let Some(p) = Self::pogo_mut(world, self.pogo) {
            p.motor_speed = 0.0;
        }
        doll.reset_joint_limits(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        for j in self.joints.iter_mut().filter_map(Option::take) {
            b.destroy_joint(j);
        }
        let shapes = [self.frame, self.rod].map(|body| (b.body(body).shapes[0], zero));
        b.set_shape_filters(&shapes);
    }
}

/// Shopping cart (guide `c4`, tag Char4): a cart body on two shocked wheels with a handle (shaft) welded to it,
/// ten grocery boxes (two sausage chains) riding in it. Same drive/hop scheme as the mower (both wheels at the
/// same speed, step 2, max 50). Smashing of cart/main/crackers and sounds are NOT ported.
pub struct Cart {
    pub body: BodyId,
    pub shaft: BodyId,
    pub back_wheel: BodyId,
    pub front_wheel: BodyId,
    pub groceries: Vec<BodyId>,
    /// `crackerSmash` / `colaSmash` happened (the cracker box is gone; the soda can shows frame 2 and spills).
    pub cracker_gone: bool,
    pub cola_open: bool,
    back_shock: BodyId,
    front_shock: BodyId,
    back_shock_joint: usize,
    front_shock_joint: usize,
    back_joint: usize,
    front_joint: usize,
    joints: [Option<usize>; 5],
    /// Creation defs of those five joints and the points on the cart they hang from, to board it again.
    defs: [RevoluteJointDef; 5],
    anchors: [Vec2; 5],
    reattaching: bool,
    pub ejected: bool,
    pub main_smashed: bool,
    /// The four loose pieces left by `mainSmash`.
    pub pieces: Vec<BodyId>,
}

impl Cart {
    pub const CART_SMASH_LIMIT: f64 = 100.0;

    /// `motorSound` (ElectricMotor1 on the back wheel) plays while the wheel motors are on (up/down held, not ejected).
    pub fn motor_on(&self, world: &World) -> bool {
        matches!(&world.b2().joint(self.back_joint).kind, box2d::joints::JointKind::Revolute(r) if r.enable_motor)
    }
    pub const MAIN_SMASH_LIMIT: f64 = 200.0;

    /// `cartSmash` (original 27335, limit 100 on the third body shape): the basket shape is removed from the body.
    pub fn cart_smash(&mut self, world: &mut World) {
        {
            let at = world.b2().body(self.body).sweep.c;
            world.play("MetalSmashLight", 1, at);
            let body = world.b2().body(self.body);
            let lc = body.sweep.local_center;
            let p = body.world_point(Vec2::new(lc.x + 35.0 / 62.5, lc.y - 15.0 / 62.5));
            world.shard_burst(crate::blood::ShardKind::Cart, p, 30.0, 30.0, 20);
        }
        let b = world.b2_mut();
        if let Some(&s) = b.body(self.body).shapes.get(2) {
            b.destroy_shape(s);
        }
    }

    /// `mainSmash` (original 27367, limit 200 on the first body shape): eject, drop the basket shape if still there, and turn the
    /// four remaining body shapes (front, base, seat, rear) into loose bodies (density 4, zero filter) created at the body's pose
    /// with its velocities, assigned newest shape first; the body and both shock bodies are destroyed.
    pub fn main_smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.main_smashed {
            return;
        }
        self.eject(world, doll);
        self.main_smashed = true;
        {
            let at = world.b2().body(self.body).sweep.c;
            world.play("MetalSmashHeavy", 1, at);
        }
        self.cart_smash(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let (pos, ang) = (b.body(self.body).xf.position, b.body(self.body).sweep.a);
        let (lv, av) = (b.body(self.body).linear_velocity, b.body(self.body).angular_velocity);
        let shapes: Vec<_> = b.body(self.body).shapes.iter().rev().copied().collect();
        let verts: Vec<Vec<Vec2>> = shapes
            .iter()
            .map(|&s| match &b.c.shapes[s].shape.kind {
                box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices.clone(),
                _ => Vec::new(),
            })
            .collect();
        let bodies: Vec<BodyId> = (0..4)
            .map(|_| {
                let n = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
                b.body_mut(n).linear_velocity = lv;
                b.body_mut(n).angular_velocity = av;
                n
            })
            .collect();
        for (n, v) in bodies.iter().zip(verts) {
            let mut d = ShapeDef::new(Geometry::Polygon(v));
            d.density = 4.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(*n, &d);
            b.set_mass_from_shapes(*n);
        }
        b.destroy_body(self.body);
        b.destroy_body(self.front_shock);
        b.destroy_body(self.back_shock);
        self.pieces = bodies;
    }

    /// Limb-break reactions (original 27463, shoulders and hips in the subclass at 27789): shoulder/elbow → that hand's joint,
    /// hip/knee → that foot's; `checkEject` fires once all four are gone (the pelvis joint, slot 0, does not count).
    pub fn limbs_broken(&mut self, world: &mut World, doll: &mut Ragdoll, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        let b = world.b2_mut();
        for l in broken {
            let slot = match l {
                Limb::Shoulder1 | Limb::Elbow1 => 1,
                Limb::Shoulder2 | Limb::Elbow2 => 2,
                Limb::Hip1 | Limb::Knee1 => 3,
                Limb::Hip2 | Limb::Knee2 => 4,
                _ => continue,
            };
            if let Some(j) = self.joints[slot].take() {
                b.destroy_joint(j);
            }
        }
        if !broken.is_empty() && self.joints[1..].iter().all(Option::is_none) {
            self.eject(world, doll);
        }
    }

    pub fn back_shock(&self) -> BodyId {
        self.back_shock
    }

    pub fn front_shock(&self) -> BodyId {
        self.front_shock
    }

    const MAX_SPEED: f64 = 50.0;
    const ACCEL: f64 = 2.0;
    const MAX_SPIN: f64 = 5.0;
    const JUMP: f64 = 20.0 / 62.5;

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let poly = |prefix: &str, n: usize| -> Option<Vec<Vec2>> { (1..=n).map(|i| at(&format!("{prefix}{i}"))).collect() };
        let boxed = |name: &str| -> Option<Vec<Vec2>> {
            let g = guide.get(name)?;
            Some(oriented_box(g[3] * 5.0 / CHARACTER_SCALE, g[4] * 5.0 / CHARACTER_SCALE, at(name)?, g[2] / (180.0 / std::f64::consts::PI)))
        };
        let deg = 180.0 / std::f64::consts::PI;
        let main_f = Filter { category_bits: 513, mask_bits: 0xFFFF, group_index: -2 };
        let zero2 = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: -2 };
        let b = world.b2_mut();
        let body = b.create_body(&BodyDef::default());
        let shaft = b.create_body(&BodyDef::default());
        let add = |b: &mut box2d::world::World, on: BodyId, verts: Vec<Vec2>, filter: Filter| {
            let mut d = ShapeDef::new(Geometry::Polygon(verts));
            d.density = 4.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = filter;
            b.create_shape(on, &d);
        };
        add(b, body, poly("frontVert", 4)?, main_f);
        add(b, body, boxed("baseShape")?, main_f);
        add(b, body, boxed("cartShape")?, main_f);
        add(b, shaft, boxed("handleShape")?, main_f);
        add(b, body, boxed("seatShape")?, zero2);
        add(b, body, poly("rearVert", 4)?, zero2);
        b.set_mass_from_shapes(body);
        add(b, shaft, boxed("shaftShape")?, zero2);
        b.set_mass_from_shapes(shaft);
        let wheel = |b: &mut box2d::world::World, name: &str| -> Option<BodyId> {
            let g = guide.get(name)?;
            let w = b.create_body(&BodyDef { position: at(name)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: g[5] / 2.0 / CHARACTER_SCALE });
            d.density = 5.0;
            d.friction = 1.0;
            d.restitution = 0.3;
            d.filter = main_f;
            b.create_shape(w, &d);
            b.set_mass_from_shapes(w);
            Some(w)
        };
        let back_wheel = wheel(b, "backWheelShape")?;
        let front_wheel = wheel(b, "frontWheelShape")?;
        let shock = |b: &mut box2d::world::World, name: &str| -> Option<BodyId> {
            let s = b.create_body(&BodyDef::default());
            add(b, s, oriented_box(12.0 / CHARACTER_SCALE, 12.0 / CHARACTER_SCALE, at(name)?, 0.0), zero2);
            b.set_mass_from_shapes(s);
            Some(s)
        };
        let back_shock = shock(b, "backWheelShape")?;
        let front_shock = shock(b, "frontWheelShape")?;
        let mut groceries = Vec::new();
        for i in 0..10 {
            let name = format!("box{i}");
            let g = guide.get(&name)?;
            let gb = b.create_body(&BodyDef { position: at(&name)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::boxed(g[3] * 5.0 / CHARACTER_SCALE, g[4] * 5.0 / CHARACTER_SCALE);
            d.density = 0.25;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
            b.create_shape(gb, &d);
            b.set_mass_from_shapes(gb);
            groceries.push(gb);
        }
        doll.finish_joints(world);
        doll.set_limits(world, Limb::Neck, -10.0 / deg, 10.0 / deg);
        let b = world.b2_mut();
        let prismatic = |b: &mut box2d::world::World, sb: BodyId, name: &str| -> Option<usize> {
            let mut d = box2d::joints::PrismaticJointDef::initialize(b.body(body), body, b.body(sb), sb, at(name)?, Vec2::new(0.0, 1.0));
            d.max_motor_force = 1000.0;
            d.enable_limit = true;
            d.lower_translation = 0.0;
            d.upper_translation = 0.0;
            Some(b.create_prismatic_joint(&d))
        };
        let back_shock_joint = prismatic(b, back_shock, "backWheelShape")?;
        let front_shock_joint = prismatic(b, front_shock, "frontWheelShape")?;
        let def = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2, limit: bool| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 100000.0;
            d.enable_limit = limit;
            d
        };
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2, limit: bool| {
            let d = def(b, b1, b2, p, limit);
            b.create_revolute_joint(&d)
        };
        rev(b, body, shaft, at("shaftAnchor")?, true);
        let pelvis = doll.bodies[&Part::Pelvis];
        let pp = b.body(pelvis).xf.position;
        let pelvis_def = def(b, body, pelvis, pp, false);
        let frame_pelvis = b.create_revolute_joint(&pelvis_def);
        let back_joint = rev(b, back_shock, back_wheel, at("backWheelShape")?, false);
        let front_joint = rev(b, front_shock, front_wheel, at("frontWheelShape")?, false);
        let hand = at("handleAnchor")?;
        let hand1_def = def(b, body, doll.bodies[&Part::LowerArm1], hand, false);
        let hand1 = b.create_revolute_joint(&hand1_def);
        let hand2_def = def(b, body, doll.bodies[&Part::LowerArm2], hand, false);
        let hand2 = b.create_revolute_joint(&hand2_def);
        let (l1, l2) = (doll.bodies[&Part::UpperLeg1], doll.bodies[&Part::UpperLeg2]);
        let (p1, p2) = (b.body(l1).xf.position, b.body(l2).xf.position);
        let leg1_def = def(b, body, l1, p1, false);
        let leg1 = b.create_revolute_joint(&leg1_def);
        let leg2_def = def(b, body, l2, p2, false);
        let leg2 = b.create_revolute_joint(&leg2_def);
        let anchors = [pp, hand, hand, p1, p2].map(|p| b.body(body).local_point(p));
        let defs = [pelvis_def, hand1_def, hand2_def, leg1_def, leg2_def];
        rev(b, groceries[7], groceries[8], at("sausageAnchor1")?, false);
        rev(b, groceries[8], groceries[9], at("sausageAnchor2")?, false);
        doll.poses = PoseSet::Cart;
        Some(Self {
            body,
            shaft,
            back_wheel,
            front_wheel,
            groceries,
            cracker_gone: false,
            cola_open: false,
            back_shock,
            front_shock,
            back_shock_joint,
            front_shock_joint,
            back_joint,
            front_joint,
            joints: [Some(frame_pelvis), Some(hand1), Some(hand2), Some(leg1), Some(leg2)],
            defs,
            anchors,
            reattaching: false,
            ejected: false,
            main_smashed: false,
            pieces: Vec::new(),
        })
    }

    fn rev_mut(world: &mut World, id: usize) -> Option<&mut box2d::joints::Revolute> {
        match &mut world.b2_mut().joint_mut(id).kind {
            box2d::joints::JointKind::Revolute(r) => Some(r),
            _ => None,
        }
    }

    fn shock_mut(world: &mut World, id: usize) -> Option<&mut box2d::joints::Prismatic> {
        match &mut world.b2_mut().joint_mut(id).kind {
            box2d::joints::JointKind::Prismatic(p) => Some(p),
            _ => None,
        }
    }

    fn shocks(&self, world: &mut World, motor: Option<bool>, limits: Option<(f64, f64)>, speed: f64) {
        for id in [self.back_shock_joint, self.front_shock_joint] {
            if let Some(p) = Self::shock_mut(world, id) {
                if let Some(m) = motor {
                    p.enable_motor = m;
                }
                if let Some((lo, hi)) = limits {
                    p.lower_translation = lo;
                    p.upper_translation = hi;
                }
                p.motor_speed = speed;
            }
        }
    }

    /// `spacePressedActions` (`pressed`) / `spaceNullActions` while seated.
    fn hop(&self, world: &mut World, pressed: bool) {
        let translation = world.b2().prismatic_translation(self.back_shock_joint).unwrap_or(0.0);
        let Some(p) = Self::shock_mut(world, self.back_shock_joint) else { return };
        let (enabled, speed) = (p.enable_motor, p.motor_speed);
        if enabled {
            if speed > 0.0 {
                if translation > Self::JUMP {
                    self.shocks(world, None, None, -1.0);
                }
            } else if speed < 0.0 && translation < 0.0 {
                self.shocks(world, Some(false), Some((0.0, 0.0)), 0.0);
            }
        } else if pressed {
            self.shocks(world, Some(true), Some((0.0, Self::JUMP)), 5.0);
            let at = world.b2().body(self.back_wheel).sweep.c;
            world.play("SegwayJump", 1, at);
        }
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            match (k.left, k.right) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.left { 1 } else { 2 }),
                (true, false) | (false, true) => {
                    let left = k.left;
                    let b = world.b2_mut();
                    let (angle, av) = (b.body(self.body).sweep.a, b.body(self.body).angular_velocity);
                    let f = if left { (av + Self::MAX_SPIN) / Self::MAX_SPIN } else { (av - Self::MAX_SPIN) / -Self::MAX_SPIN };
                    let f = f.clamp(0.0, 1.0);
                    // original: `Math.cos(a) * impulseMagnitude * f` (left to right)
                    let (c, s) = (box2d::fdlibm::cos(angle) * 1.25 * f, box2d::fdlibm::sin(angle) * 1.25 * f);
                    let lc = b.body(self.body).sweep.local_center;
                    let off = if left { 1.0 } else { -1.0 };
                    let point = b.body(self.body).world_point(Vec2::new(lc.x + off, lc.y));
                    b.body_mut(self.body).apply_impulse(Vec2::new(s, -c), point);
                }
                _ if ejected && (doll.pose == 1 || doll.pose == 2) => doll.set_pose(0),
                _ => {}
            }
            match (k.up, k.down) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.up { 3 } else { 4 }),
                (true, false) | (false, true) => {
                    let dir = if k.up { 1.0 } else { -1.0 };
                    if !Self::rev_mut(world, self.back_joint).is_some_and(|r| r.enable_motor) {
                        for id in [self.back_joint, self.front_joint] {
                            if let Some(r) = Self::rev_mut(world, id) {
                                r.enable_motor = true;
                            }
                        }
                    }
                    let b = world.b2();
                    let cur = b.body(self.back_wheel).angular_velocity - b.body(self.back_shock).angular_velocity;
                    let new = if cur * dir < 0.0 {
                        0.0
                    } else if cur * dir < Self::MAX_SPEED {
                        cur + dir * Self::ACCEL
                    } else {
                        cur
                    };
                    for id in [self.back_joint, self.front_joint] {
                        if let Some(r) = Self::rev_mut(world, id) {
                            r.motor_speed = new;
                        }
                    }
                }
                _ => {
                    if Self::rev_mut(world, self.back_joint).is_some_and(|r| r.enable_motor) {
                        for id in [self.back_joint, self.front_joint] {
                            if let Some(r) = Self::rev_mut(world, id) {
                                r.enable_motor = false;
                            }
                        }
                    }
                    if ejected && (doll.pose == 3 || doll.pose == 4) {
                        doll.set_pose(0);
                    }
                }
            }
            if ejected {
                doll.set_grab(world, k.space);
            } else {
                self.hop(world, k.space);
            }
            if k.shift {
                if ejected {
                    doll.set_pose(6);
                }
            } else if doll.pose == 6 {
                doll.set_pose(0);
            }
            if k.ctrl {
                if ejected {
                    doll.set_pose(7);
                }
            } else if doll.pose == 7 {
                doll.set_pose(0);
            }
            if k.z {
                self.eject(world, doll);
            }
        }
        doll.apply_pose(world);
        if self.reattaching {
            self.re_attaching(world, doll);
        }
    }

    /// `reAttach` (original 27756): neck −10..10°, that hand's joint, the handle and shaft collide with the rider again (not the rest of
    /// the cart), `reAttaching` started.
    fn board(&mut self, world: &mut World, doll: &mut Ragdoll, k: usize, tip: Vec2) -> bool {
        if !self.ejected || self.main_smashed || doll.dead || !near(tip, world.b2().body(self.body).world_point(self.anchors[1]), 0.25) {
            return false;
        }
        let deg = 180.0 / std::f64::consts::PI;
        self.ejected = false;
        doll.set_pose(0);
        doll.set_grab(world, false);
        doll.set_limits_now(world, Limb::Neck, -10.0 / deg, 10.0 / deg);
        let b = world.b2_mut();
        self.joints[1 + k] = Some(b.create_revolute_joint(&self.defs[1 + k].clone()));
        let shapes = [b.body(self.shaft).shapes[0], b.body(self.shaft).shapes[1]];
        for (s, category_bits) in shapes.into_iter().zip([513, 260]) {
            b.set_shape_filter(s, Filter { category_bits, mask_bits: 0xFFFF, group_index: -2 });
        }
        self.reattaching = true;
        true
    }

    /// The `reAttaching` action (original 27597): hands near the handle, the pelvis near its seat while both knees are bent 0.25..0.5 rad
    /// (or broken), then the thighs near their places; it ends once all five hold or cannot.
    fn re_attaching(&mut self, world: &mut World, doll: &Ragdoll) {
        if self.ejected || doll.dead {
            self.reattaching = false;
            return;
        }
        let mut n = 0;
        for k in 0..2 {
            let (arm, shoulder, elbow) = if k == 0 { (Part::LowerArm1, Limb::Shoulder1, Limb::Elbow1) } else { (Part::LowerArm2, Limb::Shoulder2, Limb::Elbow2) };
            if self.joints[1 + k].is_some() || doll.broken.contains(&elbow) || doll.broken.contains(&shoulder) {
                n += 1;
            } else if near(Segway::limb_tip(world, doll, arm), world.b2().body(self.body).world_point(self.anchors[1 + k]), 0.25) {
                self.joints[1 + k] = Some(world.b2_mut().create_revolute_joint(&self.defs[1 + k].clone()));
                n += 1;
            }
        }
        let bent = [Limb::Knee1, Limb::Knee2].into_iter().all(|knee| {
            doll.broken.contains(&knee) || doll.joints.get(&knee).and_then(|&j| world.b2().revolute_angle(j)).is_some_and(|a| a < 0.5 && a > 0.25)
        });
        // slot 0 is the pelvis, 3 and 4 the thighs (these only once the pelvis sits)
        for (slot, part) in [(0, Part::Pelvis), (3, Part::UpperLeg1), (4, Part::UpperLeg2)] {
            if slot != 0 && self.joints[0].is_none() {
                continue;
            }
            if self.joints[slot].is_some() {
                n += 1;
                continue;
            }
            let at = world.b2().body(doll.bodies[&part]).xf.position;
            if bent && near(at, world.b2().body(self.body).world_point(self.anchors[slot]), 0.25) {
                self.joints[slot] = Some(world.b2_mut().create_revolute_joint(&self.defs[slot].clone()));
                n += 1;
            }
        }
        if n >= 5 {
            self.reattaching = false;
        }
    }

    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        self.reattaching = false;
        doll.reset_joint_limits(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let zero2 = Filter { group_index: -2, ..zero };
        let b = world.b2_mut();
        for j in self.joints.iter_mut().filter_map(Option::take) {
            b.destroy_joint(j);
        }
        let wheels = [self.front_wheel, self.back_wheel].map(|w| (b.body(w).shapes[0], zero2));
        b.set_shape_filters(&wheels);
        self.shocks(world, Some(false), Some((0.0, 0.0)), 0.0);
        let b = world.b2_mut();
        let shapes: Vec<_> = b.body(self.body).shapes.iter().rev().copied().collect();
        for s in shapes {
            b.set_shape_filter(s, zero2);
        }
        let shapes: Vec<_> = b.body(self.shaft).shapes.iter().rev().copied().collect();
        for s in shapes {
            b.set_shape_filter(s, zero);
        }
        // the cart's own `eject` (original 27746): `ejectImpulse` 4 on chest and pelvis, perpendicular to the cart body
        let a = b.body(self.body).sweep.a - std::f64::consts::FRAC_PI_2;
        let imp = Vec2::new(box2d::fdlibm::cos(a) * 4.0, box2d::fdlibm::sin(a) * 4.0);
        for part in [Part::Chest, Part::Pelvis] {
            let id = doll.bodies[&part];
            let wc = b.body(id).sweep.c;
            b.body_mut(id).apply_impulse(imp, wc);
        }
    }
}

/// Rescue helicopter (guide `c11`, tag Heli; `shapeRefScale` 50): a copter body hovering on per-frame impulses
/// (`hoverCopter`) and levelled by `balanceCopter`, a magnet body hanging from it on a rope joint (type 7).
/// Left/right integrate `targetAng`, up/down pick the hover state, shift/ctrl reel the rope in/out.
/// NOT ported: magnet pick-up joints (`handleMagnetContacts`), blade/copter smash, rope drawing, sounds.
pub struct Heli {
    pub copter: BodyId,
    pub magnet: BodyId,
    rope: usize,
    rope_min: f64,
    joints: [Option<usize>; 3],
    com: Vec<BodyId>,
    curr_com: Vec2,
    total_mass: f64,
    hover_state: i32,
    target_ang: f64,
    spin_acceleration: f64,
    pub magnetized: bool,
    pub magnet_shape: box2d::world::ShapeId,
    /// `magnetVert0..3` (local to the magnet body): the pick-up range, only ever used for its AABB.
    range: Vec<Vec2>,
    space_off: bool,
    pub ejected: bool,
    /// Copter shapes in creation order: blade, back, leg1, leg2, wheel1, wheel2.
    shapes: [box2d::world::ShapeId; 6],
    /// `vertsBrokenCopter` (guide `broken1Vert0..` 1..7, local to the copter body).
    broken: Vec<Vec<Vec2>>,
    /// `bladeLocalCenter` = the blade box centre (copter-local).
    blade_center: Vec2,
    pub copter_smashed: bool,
    pub blade_smashed: bool,
    legs_present: [bool; 2],
    /// Bodies born from smashes (copter pieces, blade shards, legs).
    pub debris: Vec<BodyId>,
    /// Sprite clip of each debris body (`broken1..7`; empty for blade shards).
    pub debris_names: Vec<String>,
    /// The drawn rope (not the joint): `numRopeSegments + 1` Verlet points as (current, previous), metres.
    rope_points: Vec<(Vec2, Vec2)>,
}

impl Heli {
    const ROPE_SEGMENTS: usize = 20;

    /// The rope part of `paint`: both ends follow the joint anchors (the upper one hangs free once the copter is smashed),
    /// then gravity and 20 passes over the springs, whose length is 0.8 of the rope joint's limit.
    pub(crate) fn step_rope(&mut self, world: &World) {
        let last = Self::ROPE_SEGMENTS;
        if let Some(a) = joint_anchor(world, self.rope, false).filter(|_| !self.copter_smashed) {
            self.rope_points[0] = (a, a);
        }
        if let Some(b) = joint_anchor(world, self.rope, true) {
            self.rope_points[last] = (b, b);
        }
        let free: Vec<f64> = (0..=last).map(|i| if i == last || (i == 0 && !self.copter_smashed) { 0.0 } else { 1.0 }).collect();
        for (p, &f) in self.rope_points.iter_mut().zip(&free) {
            if f > 0.0 {
                let (dx, dy) = (p.0.x - p.1.x, p.0.y - p.1.y);
                p.1 = p.0;
                p.0.x += dx;
                p.0.y += dy + (10.0 / 30.0) / 30.0;
            }
        }
        let box2d::joints::JointKind::Rope(rope) = &world.b2().joint(self.rope).kind else { return };
        let length = rope.max_length * 0.8 / last as f64;
        for _ in 0..20 {
            for i in 0..last {
                let (a, b) = (self.rope_points[i].0, self.rope_points[i + 1].0);
                let (dx, dy) = (b.x - a.x, b.y - a.y);
                let d = (dx * dx + dy * dy).sqrt();
                let total = free[i] + free[i + 1];
                if d == 0.0 || total == 0.0 {
                    continue;
                }
                let k = (d - length) / d;
                let (cx, cy) = (dx * k, dy * k);
                self.rope_points[i].0.x += cx * (free[i] / total);
                self.rope_points[i].0.y += cy * (free[i] / total);
                self.rope_points[i + 1].0.x -= cx * (free[i + 1] / total);
                self.rope_points[i + 1].0.y -= cy * (free[i + 1] / total);
            }
        }
    }

    /// `heliLoop` volume: 0.75 while up/down is held (`isLoud`), 0.5 otherwise.
    pub fn loop_volume(&self) -> f32 {
        if self.hover_state != 0 { 0.75 } else { 0.5 }
    }

    /// Limb-break reactions (original ~24294): shoulder/elbow breaks drop that hand's joint; eject once both hands are gone
    /// (legs only affect the original's centre-of-mass list, not modelled).
    pub fn limbs_broken(&mut self, world: &mut World, doll: &mut Ragdoll, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        let b = world.b2_mut();
        for l in broken {
            let slot = match l {
                Limb::Shoulder1 | Limb::Elbow1 => 1,
                Limb::Shoulder2 | Limb::Elbow2 => 2,
                _ => continue,
            };
            if let Some(j) = self.joints[slot].take() {
                b.destroy_joint(j);
                if self.joints[1].is_none() && self.joints[2].is_none() {
                    self.eject(world, doll);
                    return;
                }
            }
        }
    }


    /// `handleMagnetContacts`: while magnetised, a body touching the magnet below its centre and inside the range
    /// AABB is pinned to it by a ±3° revolute joint (collideConnected); switched off, every pick-up joint is destroyed.
    /// `hits` = per touched shape the strongest `Result` event this frame (`magnetContactResult`).
    pub fn handle_magnet(&mut self, world: &mut World, hits: &[(box2d::world::ShapeId, Vec2, f64)]) {
        let b = world.b2_mut();
        let held: Vec<(usize, BodyId)> = b.body(self.magnet).joints.iter().filter(|j| j.0 != self.rope).map(|j| (j.0, b.joint(j.0).body2)).collect();
        if self.magnetized {
            let xf = b.body(self.magnet).xf;
            let pts: Vec<Vec2> = self.range.iter().map(|v| b.body(self.magnet).world_point(*v)).collect();
            let _ = xf;
            let (mut lo, mut hi) = (pts[0], pts[0]);
            for p in &pts {
                lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
                hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
            }
            for &(shape, pos, _) in hits {
                let body = b.shape_body(shape);
                let mass = b.body(body).mass;
                let aabb = b.shape_aabb(shape);
                if aabb.lower.x > hi.x || aabb.upper.x < lo.x || aabb.lower.y > hi.y || aabb.upper.y < lo.y {
                    continue;
                }
                let local = b.body(self.magnet).local_point(pos);
                if local.y > 0.0 && mass > 0.0 && !self.com.contains(&body) && !held.iter().any(|h| h.1 == body) {
                    let mut d = RevoluteJointDef::initialize(b.body(self.magnet), self.magnet, b.body(body), body, pos);
                    d.collide_connected = true;
                    d.enable_limit = true;
                    d.upper_angle = std::f64::consts::PI * 3.0 / 180.0;
                    d.lower_angle = std::f64::consts::PI * -3.0 / 180.0;
                    b.create_revolute_joint(&d);
                }
            }
        } else {
            for (id, _) in held {
                b.destroy_joint(id);
            }
        }
    }
    const ACCEL: f64 = 0.025;
    const DECEL: f64 = 0.02;
    const MAX_STEP: f64 = 0.1;
    const ROPE_SPEED: f64 = 0.05;
    const ROPE_MAX: f64 = 3.0;

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let boxed = |name: &str| -> Option<Vec<Vec2>> {
            let g = guide.get(name)?;
            Some(oriented_box(g[3] * 50.0 / CHARACTER_SCALE, g[4] * 50.0 / CHARACTER_SCALE, at(name)?, g[2] / (180.0 / std::f64::consts::PI)))
        };
        let poly = |prefix: &str, n: usize| -> Option<Vec<Vec2>> { (0..n).map(|i| at(&format!("{prefix}{i}"))).collect() };
        let deg = 180.0 / std::f64::consts::PI;
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let default_filter = Filter { category_bits: 260, mask_bits: 270, group_index: -1 };
        let b = world.b2_mut();
        let copter = b.create_body(&BodyDef { linear_damping: 0.4, ..Default::default() });
        let add = |b: &mut box2d::world::World, on: BodyId, geometry: Geometry, density: f64, friction: f64, restitution: f64, filter: Filter| {
            let mut d = ShapeDef::new(geometry);
            d.density = density;
            d.friction = friction;
            d.restitution = restitution;
            d.filter = filter;
            b.create_shape(on, &d);
        };
        add(b, copter, Geometry::Polygon(boxed("bladeShape")?), 3.0, 0.3, 0.1, zero);
        add(b, copter, Geometry::Polygon(boxed("stemShape")?), 3.0, 0.3, 0.1, zero);
        add(b, copter, Geometry::Polygon(poly("baseVert", 4)?), 3.0, 0.3, 0.1, zero);
        add(b, copter, Geometry::Polygon(poly("backVert", 5)?), 3.0, 0.3, 0.1, zero);
        let magnet_guide = *guide.get("magnetShape")?;
        let magnet = b.create_body(&BodyDef { position: at("magnetShape")?, angular_damping: 1.0, ..Default::default() });
        add(b, magnet, Geometry::Polygon(oriented_box(magnet_guide[3] * 50.0 / CHARACTER_SCALE, magnet_guide[4] * 50.0 / CHARACTER_SCALE, Vec2::default(), 0.0)), 3.0, 0.3, 0.1, zero);
        b.set_mass_from_shapes(magnet);
        add(b, copter, Geometry::Polygon(boxed("leg1Shape")?), 3.0, 0.3, 0.1, default_filter);
        add(b, copter, Geometry::Polygon(boxed("leg2Shape")?), 3.0, 0.3, 0.1, default_filter);
        for name in ["wheel1Shape", "wheel2Shape"] {
            let r = guide.get(name)?[3] * 5.0 / CHARACTER_SCALE;
            add(b, copter, Geometry::Circle { local_position: at(name)?, radius: r }, 3.0, 0.0, 0.3, zero);
        }
        b.set_mass_from_shapes(copter);
        // `magnetRangeSensor`: a sensor polygon added to the magnet body and destroyed again (this still consumes and
        // frees a broad-phase proxy, which can change later pair order).
        let range: Vec<Vec2> = (0..4).map(|i| guide.get(&format!("magnetVert{i}")).map(|g| Vec2::new(g[0] / CHARACTER_SCALE, g[1] / CHARACTER_SCALE))).collect::<Option<_>>()?;
        let mut rd = ShapeDef::new(Geometry::Polygon(range.clone()));
        rd.density = 0.0;
        rd.is_sensor = true;
        rd.friction = 0.3;
        rd.restitution = 0.1;
        rd.filter = default_filter;
        let range_shape = b.create_shape(magnet, &rd);
        b.set_mass_from_shapes(magnet);
        b.destroy_shape(range_shape);
        let magnet_shape = b.body(magnet).shapes[0];
        doll.finish_joints(world);
        doll.set_limits(world, Limb::Neck, -10.0 / deg, 10.0 / deg);
        let b = world.b2_mut();
        let rev = |b: &mut box2d::world::World, b2: BodyId, p: Vec2| {
            let mut d = RevoluteJointDef::initialize(b.body(copter), copter, b.body(b2), b2, p);
            d.max_motor_torque = 100000.0;
            b.create_revolute_joint(&d)
        };
        let pelvis = rev(b, doll.bodies[&Part::Pelvis], at("seatAnchor")?);
        let hand = at("handleAnchor")?;
        let hand1 = rev(b, doll.bodies[&Part::LowerArm1], hand);
        let hand2 = rev(b, doll.bodies[&Part::LowerArm2], hand);
        let com: Vec<BodyId> = [Part::Head, Part::Chest, Part::Pelvis, Part::UpperArm1, Part::LowerArm1, Part::UpperArm2, Part::LowerArm2, Part::UpperLeg1, Part::LowerLeg1, Part::UpperLeg2, Part::LowerLeg2]
            .iter()
            .map(|p| doll.bodies[p])
            .collect();
        let mut s = Self {
            copter,
            magnet,
            rope: 0,
            rope_min: 0.0,
            joints: [Some(pelvis), Some(hand1), Some(hand2)],
            com: [vec![copter], com].concat(),
            curr_com: Vec2::default(),
            total_mass: 0.0,
            hover_state: 0,
            target_ang: 0.0,
            spin_acceleration: 0.0,
            magnetized: false,
            magnet_shape,
            range,
            space_off: false,
            ejected: false,
            shapes: {
                let sh = &b.body(copter).shapes;
                [sh[0], sh[3], sh[4], sh[5], sh[6], sh[7]]
            },
            broken: (1..8).map(|i| (0..4).filter_map(|k| at(&format!("broken{i}Vert{k}"))).collect()).collect(),
            blade_center: at("bladeShape")?,
            copter_smashed: false,
            blade_smashed: false,
            legs_present: [true, true],
            debris: Vec::new(),
            debris_names: Vec::new(),
            rope_points: Vec::new(),
        };
        let ic = s.center_of_mass(world);
        let b = world.b2_mut();
        let cy = b.body(magnet).sweep.c.y;
        b.set_xform(magnet, Vec2::new(ic.x, cy), 0.0);
        s.com.push(magnet);
        let mut anchor1 = at("magnetAnchor")?;
        anchor1.x = ic.x;
        // `im = magnetBody.GetWorldCenter()` is the LIVE sweep centre in the original: `im.y -= ...` moves the
        // body's centre of mass (but not its transform) up, and the body jumps there on the next step.
        b.body_mut(magnet).sweep.c.y -= 20.0 / CHARACTER_SCALE;
        let anchor2 = b.body(magnet).sweep.c;
        let mut d = box2d::joints::DistanceJointDef::initialize(b.body(copter), copter, b.body(magnet), magnet, anchor1, anchor2);
        d.collide_connected = true;
        s.rope_min = d.length;
        s.rope = b.create_rope_joint(&d);
        s.rope_points = (0..=Self::ROPE_SEGMENTS)
            .map(|i| {
                let t = i as f64 / Self::ROPE_SEGMENTS as f64;
                let p = Vec2::new(anchor1.x + (anchor2.x - anchor1.x) * t, anchor1.y + (anchor2.y - anchor1.y) * t);
                (p, p)
            })
            .collect();
        doll.poses = PoseSet::Heli;
        Some(s)
    }

    /// `getCenterOfMass` (also refreshes `totalMass`).
    fn center_of_mass(&mut self, world: &World) -> Vec2 {
        let b = world.b2();
        let (mut x, mut y, mut m) = (0.0, 0.0, 0.0);
        for &id in &self.com {
            let body = b.body(id);
            x += body.sweep.c.x * body.mass;
            y += body.sweep.c.y * body.mass;
            m += body.mass;
        }
        self.total_mass = m;
        Vec2::new(x * (1.0 / m), y * (1.0 / m))
    }

    fn rope_mut(world: &mut World, id: usize) -> Option<&mut box2d::joints::Rope> {
        match &mut world.b2_mut().joint_mut(id).kind {
            box2d::joints::JointKind::Rope(r) => Some(r),
            _ => None,
        }
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        let ejected = self.ejected;
        if !self.copter_smashed {
            self.curr_com = self.center_of_mass(world); // preActions
        }
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            match (k.left, k.right) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.left { 1 } else { 2 }),
                (true, false) => {
                    if self.spin_acceleration > 0.0 {
                        self.spin_acceleration = -self.spin_acceleration;
                    }
                    self.spin_acceleration -= Self::ACCEL;
                    if self.spin_acceleration < -Self::MAX_STEP {
                        self.spin_acceleration = -Self::MAX_STEP;
                    }
                    self.target_ang += self.spin_acceleration;
                    doll.set_pose(7);
                }
                (false, true) => {
                    if self.spin_acceleration < 0.0 {
                        self.spin_acceleration = -self.spin_acceleration;
                    }
                    self.spin_acceleration += Self::ACCEL;
                    if self.spin_acceleration > Self::MAX_STEP {
                        self.spin_acceleration = Self::MAX_STEP;
                    }
                    self.target_ang += self.spin_acceleration;
                    doll.set_pose(8);
                }
                _ if ejected => {
                    if doll.pose == 1 || doll.pose == 2 {
                        doll.set_pose(0);
                    }
                }
                _ => {
                    if self.target_ang > 0.0 {
                        self.spin_acceleration -= Self::DECEL;
                        if self.spin_acceleration < -Self::MAX_STEP {
                            self.spin_acceleration = -Self::MAX_STEP;
                        }
                        self.target_ang += self.spin_acceleration;
                        if self.target_ang < 0.0 {
                            self.target_ang = 0.0;
                        }
                    } else if self.target_ang < 0.0 {
                        self.spin_acceleration += Self::DECEL;
                        if self.spin_acceleration > Self::MAX_STEP {
                            self.spin_acceleration = Self::MAX_STEP;
                        }
                        self.target_ang += self.spin_acceleration;
                        if self.target_ang > 0.0 {
                            self.target_ang = 0.0;
                        }
                    }
                    if doll.pose == 7 || doll.pose == 8 {
                        doll.set_pose(0);
                    }
                }
            }
            match (k.up, k.down) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.up { 3 } else { 4 }),
                (true, false) => self.hover_state = 1,
                (false, true) => self.hover_state = -1,
                _ if ejected => {
                    if doll.pose == 3 || doll.pose == 4 {
                        doll.set_pose(0);
                    }
                }
                _ => self.hover_state = 0,
            }
            if k.space {
                if ejected {
                    doll.set_grab(world, true);
                } else if self.space_off {
                    self.space_off = false;
                    self.magnetized = !self.magnetized;
                }
            } else if ejected {
                doll.set_grab(world, false);
            } else {
                self.space_off = true;
            }
            if k.shift {
                if ejected {
                    doll.set_pose(7);
                } else {
                    let min = self.rope_min;
                    if let Some(r) = Self::rope_mut(world, self.rope) {
                        r.max_length -= Self::ROPE_SPEED;
                        if r.max_length < min {
                            r.max_length = min;
                        }
                    }
                }
            } else if ejected && doll.pose == 7 {
                doll.set_pose(0);
            }
            if k.ctrl {
                if ejected {
                    doll.set_pose(8);
                } else if let Some(r) = Self::rope_mut(world, self.rope) {
                    r.max_length += Self::ROPE_SPEED;
                    if r.max_length > Self::ROPE_MAX {
                        r.max_length = Self::ROPE_MAX;
                    }
                }
            } else if ejected && doll.pose == 8 {
                doll.set_pose(0);
            }
            if k.z {
                self.eject(world, doll);
            }
        }
        doll.apply_pose(world);
        if !self.copter_smashed {
            self.balance(world);
            self.hover(world);
        }
    }

    fn balance(&mut self, world: &mut World) {
        use std::f64::consts::PI;
        let b = world.b2_mut();
        let av = b.body(self.copter).angular_velocity;
        while self.target_ang > PI {
            self.target_ang -= PI * 2.0;
        }
        while self.target_ang < -PI {
            self.target_ang += PI * 2.0;
        }
        let mut a = b.body(self.copter).sweep.a;
        while a > PI {
            a -= PI * 2.0;
        }
        while a < -PI {
            a += PI * 2.0;
        }
        let d = av + box2d::fdlibm::sin(a - self.target_ang) * 3.0;
        b.body_mut(self.copter).angular_velocity -= d;
    }

    fn hover(&mut self, world: &mut World) {
        let b = world.b2_mut();
        let angle = b.body(self.copter).sweep.a;
        let up = Vec2::new(box2d::fdlibm::sin(angle), -box2d::fdlibm::cos(angle));
        let body = b.body(self.copter);
        let (rx, ry) = (self.curr_com.x - body.sweep.c.x, self.curr_com.y - body.sweep.c.y);
        // NB: this Box2D build's GetLinearVelocityFromWorldPoint has the opposite sign of the textbook formula.
        let v = Vec2::new(body.linear_velocity.x + body.angular_velocity * ry, body.linear_velocity.y - body.angular_velocity * rx);
        let mut speed = v.x * up.x + v.y * up.y;
        let l = self.total_mass * (1.0 / 3.0);
        let r = self.total_mass * 0.31;
        speed = if speed > 0.0 { speed.min(10.0) } else { speed.max(-10.0) };
        let extra = if self.hover_state >= 0 {
            b.body_mut(self.copter).apply_impulse(Vec2::new(up.x * l, up.y * l), self.curr_com);
            if self.hover_state == 0 {
                return;
            }
            r - r * speed / 10.0
        } else {
            -0.5 - (r + r * speed / 10.0)
        };
        b.body_mut(self.copter).apply_impulse(Vec2::new(up.x * extra, up.y * extra), self.curr_com);
    }

    /// `copterLegSmash(wheelShape, …)`: the leg and its wheel shape go, a broken-leg piece (`broken6`/`broken7`) is born at the copter's pose.
    fn leg_smash(&mut self, world: &mut World, leg: usize, sound: bool) {
        if !self.legs_present[leg] {
            return;
        }
        self.legs_present[leg] = false;
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let (pos, ang, w) = {
            let c = world.b2().body(self.copter);
            (c.xf.position, c.sweep.a, c.angular_velocity)
        };
        let (leg_shape, wheel_shape) = (self.shapes[2 + leg], self.shapes[4 + leg]);
        let b = world.b2_mut();
        b.destroy_shape(leg_shape);
        let piece = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
        let mut d = ShapeDef::new(Geometry::Polygon(self.broken[5 + leg].clone()));
        d.density = 3.0;
        d.friction = 0.3;
        d.restitution = 0.3;
        d.filter = zero;
        b.create_shape(piece, &d);
        b.set_mass_from_shapes(piece);
        b.body_mut(piece).angular_velocity = w;
        let lc = b.body(piece).sweep.local_center;
        let v = b.body(self.copter).linear_velocity_from_local_point(lc);
        b.body_mut(piece).linear_velocity = v;
        self.debris.push(piece);
        self.debris_names.push(format!("broken{}", 6 + leg));
        b.destroy_shape(wheel_shape);
        if sound {
            let at = b.body(self.copter).sweep.c;
            world.play("StemSnap", 1, at);
        }
    }

    /// `bladeSmash`: the blade box is cut into two quad + two triangle shards (blade hit at `at`, world position) that fly off with the
    /// copter's velocity plus 50× their offset from the centre; the blade shape is destroyed.
    fn blade_smash(&mut self, world: &mut World, at: Vec2) {
        if self.blade_smashed {
            return;
        }
        self.blade_smashed = true;
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let cs = CHARACTER_SCALE;
        let mut q = world.b2().body(self.copter).local_point(at);
        q.x -= self.blade_center.x;
        q.y -= self.blade_center.y;
        let (ia, iw, il, mut ir) = (20.0 / cs, -5.0 / cs, 5.0 / cs, 235.0 / cs);
        let (mut left, ic, im);
        if q.x < 0.0 {
            left = false;
            ic = -200.0 / cs;
            im = -35.0 / cs;
        } else {
            left = true;
            ic = 35.0 / cs;
            im = 200.0 / cs;
            ir = -ir;
        }
        let _ = &mut left;
        let (mut k, mut t, mut o, mut x) = (Vec2::new(ic, iw), Vec2::new(im, iw), Vec2::new(im, il), Vec2::new(ic, il));
        let mut xx = Vec2::new(0.0, 0.0);
        let (tri, quad): (Vec<usize>, Vec<usize>);
        // vertex ids: 0=x(shared point) 1=k 2=t 3=o 4=x2 (ix)
        if q.x < ic + ia {
            xx.x = im - ia;
            tri = vec![0, 4, 1];
            if q.y > 0.0 {
                xx.y = iw;
                quad = vec![0, 2, 3, 4];
            } else {
                xx.y = il;
                quad = vec![0, 1, 2, 3];
            }
        } else if q.x > im - ia {
            xx.x = ic + ia;
            tri = vec![0, 2, 3];
            if q.y > 0.0 {
                xx.y = iw;
                quad = vec![0, 3, 4, 1];
            } else {
                xx.y = il;
                quad = vec![0, 4, 1, 2];
            }
        } else {
            xx.x = q.x;
            tri = vec![0, 2, 3];
            if q.y > 0.0 {
                xx.y = il;
                quad = vec![0, 4, 1, 2];
            } else {
                xx.y = iw;
                quad = vec![0, 3, 4, 1];
            }
        }
        let (origin, angle) = {
            let c = world.b2().body(self.copter);
            (c.world_point(self.blade_center), c.sweep.a)
        };
        for side in 0..2 {
            if side == 1 {
                for v in [&mut k, &mut t, &mut o, &mut x, &mut xx] {
                    v.x += ir;
                }
            }
            let pts = [xx, k, t, o, x];
            let mut quad_center = Vec2::default();
            for (shape_ids, is_quad) in [(&quad, true), (&tri, false)] {
                let b = world.b2_mut();
                let body = b.create_body(&BodyDef { position: origin, angle, ..Default::default() });
                let mut d = ShapeDef::new(Geometry::Polygon(shape_ids.iter().map(|&i| pts[i]).collect()));
                d.density = 3.0;
                d.friction = 0.3;
                d.restitution = 0.1;
                d.filter = zero;
                b.create_shape(body, &d);
                b.set_mass_from_shapes(body);
                if is_quad {
                    quad_center = b.body(body).sweep.c;
                }
                let c = b.body(body).sweep.c;
                let mut v = b.body(self.copter).linear_velocity_from_world_point(c);
                let r = Vec2::new((quad_center.x - b.body(body).xf.position.x) * 50.0, (quad_center.y - b.body(body).xf.position.y) * 50.0);
                v.x += r.x;
                v.y += r.y;
                b.body_mut(body).linear_velocity = v;
                self.debris.push(body);
                self.debris_names.push(String::new());
                world.add_blade_shard(body);
            }
        }
        let b = world.b2_mut();
        let blade = self.shapes[0];
        b.destroy_shape(blade);
        let c = b.body(self.copter).sweep.c;
        world.shard_burst(crate::blood::ShardKind::Copter, at, 30.0, 60.0, 70);
        world.play("MetalSmashHeavy4", 1, c);
    }

    /// Test/driver access to the single smashes (`bladeSmash(contact)` / `copterLegSmash(wheel, contact)` with sound).
    pub fn blade_smash_at(&mut self, world: &mut World, at: Vec2) {
        self.blade_smash(world, at);
    }

    pub fn leg_smash_at(&mut self, world: &mut World, leg: usize) {
        self.leg_smash(world, leg, true);
    }

    /// `copterSmash`: blade and legs smash first, the rider is ejected, the copter body is replaced by five zero-filter pieces (`broken1..5`)
    /// carrying its angular velocity and point velocities. `at` = world position of the biggest contact.
    pub fn copter_smash(&mut self, world: &mut World, doll: &mut Ragdoll, at: Vec2) {
        if self.copter_smashed {
            return;
        }
        self.copter_smashed = true;
        self.blade_smash(world, at);
        self.leg_smash(world, 0, false);
        self.leg_smash(world, 1, false);
        self.eject(world, doll);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let (pos, ang, w, c) = {
            let b = world.b2().body(self.copter);
            (b.xf.position, b.sweep.a, b.angular_velocity, b.sweep.c)
        };
        // the destroyed body object stays readable in the original (`GetLinearVelocityFromLocalPoint` after `DestroyBody`)
        let copter = world.b2().body(self.copter).clone();
        let b = world.b2_mut();
        b.destroy_body(self.copter);
        for i in 0..5 {
            let piece = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Polygon(self.broken[i].clone()));
            d.density = 3.0;
            d.friction = 0.3;
            d.restitution = 0.3;
            d.filter = zero;
            b.create_shape(piece, &d);
            b.set_mass_from_shapes(piece);
            b.body_mut(piece).angular_velocity = w;
            let lc = b.body(piece).sweep.local_center;
            b.body_mut(piece).linear_velocity = copter.linear_velocity_from_local_point(lc);
            self.debris.push(piece);
            self.debris_names.push(format!("broken{}", i + 1));
        }
        world.shard_burst(crate::blood::ShardKind::Copter, c, 30.0, 30.0, 70);
        world.play("MetalSmashHeavy2", 1, c);
    }

    /// `handleContactResults` for the copter shapes: back/wheel results above 40 smash the copter / break a leg, blade results above 30 shatter
    /// the blade. `hits` = (shape, position, impulse) strongest result per shape. Returns nothing; the blade contact impulses (`handleBladeContacts`)
    /// are applied here too while the blade is intact.
    pub fn contacts(&mut self, world: &mut World, doll: &mut Ragdoll, events: &[box2d::world::ContactEvent]) {
        if self.copter_smashed {
            return;
        }
        let [blade, back, _, _, w1, w2] = self.shapes;
        let strongest = |s: box2d::world::ShapeId| events.iter().filter(|e| e.kind == box2d::world::ContactEventKind::Result && (e.shape1 == s || e.shape2 == s)).max_by(|a, b| a.normal_impulse.total_cmp(&b.normal_impulse));
        if !self.blade_smashed {
            for ev in events.iter().filter(|e| e.kind == box2d::world::ContactEventKind::Result && (e.shape1 == blade || e.shape2 == blade)) {
                let other = if ev.shape1 == blade { ev.shape2 } else { ev.shape1 };
                let b = world.b2_mut();
                let ob = b.shape_body(other);
                let c = b.body(self.copter).local_point(ev.position);
                let mut im = Vec2::new(c.x - self.blade_center.x, c.y - self.blade_center.y);
                let len = im.x.hypot(im.y);
                if len > f64::EPSILON {
                    im = Vec2::new(im.x / len, im.y / len);
                }
                let r = b.body(self.copter).xf.r;
                im = Vec2::new(r.col1.x * im.x + r.col2.x * im.y, r.col1.y * im.x + r.col2.y * im.y);
                let (om, oinv, cinv) = (b.body(ob).mass, b.body(ob).inv_mass, b.body(self.copter).inv_mass);
                if om > 0.0 {
                    let n = oinv + cinv;
                    let (iw, ik) = (oinv * 3.0 / n, cinv * 3.0 / n);
                    b.body_mut(ob).apply_impulse(Vec2::new(im.x * iw, im.y * iw), ev.position);
                    b.body_mut(self.copter).apply_impulse(Vec2::new(-im.x * ik, -im.y * ik), ev.position);
                } else {
                    b.body_mut(self.copter).apply_impulse(Vec2::new(-im.x * 3.0, -im.y * 3.0), ev.position);
                }
            }
        }
        if let Some(ev) = strongest(back).filter(|e| e.normal_impulse > 40.0) {
            self.copter_smash(world, doll, ev.position);
            return;
        }
        if let Some(ev) = strongest(blade).filter(|e| e.normal_impulse > 30.0) {
            self.blade_smash(world, ev.position);
        }
        if strongest(w1).is_some_and(|e| e.normal_impulse > 40.0) {
            self.leg_smash(world, 0, true);
        }
        if strongest(w2).is_some_and(|e| e.normal_impulse > 40.0) {
            self.leg_smash(world, 1, true);
        }
    }

    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        doll.reset_joint_limits(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        for j in self.joints.iter_mut().filter_map(Option::take) {
            b.destroy_joint(j);
        }
        let shapes: Vec<_> = b.body(self.copter).shapes.iter().rev().copied().collect();
        // the blade is the first shape created
        let blade = b.body(self.copter).shapes[0];
        for s in shapes {
            if s != blade {
                b.set_shape_filter(s, zero);
            }
        }
        let a = b.body(self.copter).sweep.a;
        let imp = Vec2::new(box2d::fdlibm::cos(a) * 2.0, box2d::fdlibm::sin(a) * 2.0);
        for part in [Part::Chest, Part::Pelvis] {
            let id = doll.bodies[&part];
            let c = b.body(id).sweep.c;
            b.body_mut(id).apply_impulse(imp, c);
        }
        self.com = vec![self.copter, self.magnet];
        self.target_ang = 0.0;
        world.remove_camera_second_focus(self.magnet);
    }
}

/// Rail grinder (guide `c7`, tag Char2, class at deobfuscated.js:33042): a frame (bottom box + two side polygons) on two
/// wheels (torque 40), hands/feet pinned to the frame, left/right spin impulses, up/down accelerate/brake the wheel
/// motors, shift/ctrl lean forward/back. The original starts the whole rig at `start + (20, -40)` px.
/// Space attaches the wheels to Rail specials (`checkAddRailJoints`: sensor dongle bodies + prismatic joints).
/// NOT ported: hat/frame smash, knee-break eject, sounds, rail sparks.
pub struct Grinder {
    pub frame: BodyId,
    pub back_wheel: BodyId,
    pub front_wheel: BodyId,
    pub front_shape: box2d::world::ShapeId,
    pub back_shape: box2d::world::ShapeId,
    front_joint: usize,
    back_joint: usize,
    joints: [Option<usize>; 4],
    pub ejected: bool,
    /// Space held: wheels touching a rail (material bit 4) get a dongle.
    pub connecting: bool,
    /// `frontWheelPos` / `backWheelPos`: wheel centres in the frame's local space (the frame starts at the origin).
    wheel_pos: [Vec2; 2],
    /// Per wheel (front, back): the rail grabbed by a contact this frame, to be attached by the next `actions`.
    pub new_rail: [Option<BodyId>; 2],
    pub dongles: [Option<Dongle>; 2],
    one_dongle_counter: u32,
}

/// A rail attachment: a sensor body riding the wheel, tied to the frame by a free revolute and to the rail by a
/// prismatic joint (the wheel's motor then drives that slider along the rail).
#[derive(Debug, Clone, Copy)]
pub struct Dongle {
    pub body: BodyId,
    pub rail: BodyId,
    rev: usize,
    slider: usize,
}

impl Grinder {
    /// Which feet are still joined to the cart (`cartFoot1Joint`, `cartFoot2Joint`).
    pub fn feet_held(&self) -> [bool; 2] {
        [self.joints[2].is_some(), self.joints[3].is_some()]
    }

    /// Limb-break reactions (original ~34025): shoulder/elbow breaks drop that hand's joint, hip/knee breaks that foot's;
    /// `checkEject` fires once all four are gone.
    pub fn limbs_broken(&mut self, world: &mut World, doll: &mut Ragdoll, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        let b = world.b2_mut();
        for l in broken {
            let slot = match l {
                Limb::Shoulder1 | Limb::Elbow1 => 0,
                Limb::Shoulder2 | Limb::Elbow2 => 1,
                Limb::Hip1 | Limb::Knee1 => 2,
                Limb::Hip2 | Limb::Knee2 => 3,
                _ => continue,
            };
            if let Some(j) = self.joints[slot].take() {
                b.destroy_joint(j);
            }
        }
        if !broken.is_empty() && self.joints.iter().all(Option::is_none) {
            self.eject(world, doll);
        }
    }

    const MAX_SPEED: f64 = 1000.0;
    const ACCEL: f64 = 1.0;
    const MAX_SPIN: f64 = 5.0;
    const IMPULSE: f64 = 0.7;
    const OFFSET: f64 = 1.0;
    const PRIS_ACCEL: f64 = 0.5;
    /// The constructor shifts the start point.
    pub const START_SHIFT: [f32; 2] = [20.0, -40.0];

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        let frame = b.create_body(&BodyDef::default());
        let mut d = ShapeDef::new(Geometry::Polygon({
            let g = guide.get("frameBottomShape")?;
            oriented_box(g[3] * 5.0 / CHARACTER_SCALE, g[4] * 5.0 / CHARACTER_SCALE, at("frameBottomShape")?, g[2] / deg)
        }));
        d.density = 1.5;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = Filter { category_bits: 514, mask_bits: 0xFFFF, group_index: 0 };
        b.create_shape(frame, &d);
        for side in 0..2 {
            let verts: Option<Vec<Vec2>> = (0..4).map(|i| at(&format!("sidePoint{side}_{i}"))).collect();
            d.geometry = Geometry::Polygon(verts?);
            b.create_shape(frame, &d);
        }
        b.set_mass_from_shapes(frame);
        let wheel = |b: &mut box2d::world::World, name: &str| -> Option<BodyId> {
            let g = guide.get(name)?;
            let w = b.create_body(&BodyDef { position: at(name)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: g[5] / 2.0 / CHARACTER_SCALE });
            d.density = 12.0;
            d.friction = 1.0;
            d.restitution = 0.3;
            d.filter = Filter { category_bits: 260, mask_bits: 268, group_index: -1 };
            b.create_shape(w, &d);
            b.set_mass_from_shapes(w);
            Some(w)
        };
        let front_wheel = wheel(b, "frontWheelShape")?;
        let back_wheel = wheel(b, "backWheelShape")?;
        doll.finish_joints(world);
        doll.set_limits(world, Limb::Hip1, -50.0 / deg, 10.0 / deg);
        doll.set_limits(world, Limb::Hip2, -50.0 / deg, 10.0 / deg);
        doll.set_limits(world, Limb::Elbow1, -60.0 / deg, 0.0);
        doll.set_limits(world, Limb::Elbow2, -60.0 / deg, 0.0);
        doll.set_limits(world, Limb::Neck, 0.0, 20.0 / deg);
        let b = world.b2_mut();
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2, limit: Option<f64>| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 40.0;
            if let Some(l) = limit {
                d.enable_limit = true;
                d.lower_angle = -l;
                d.upper_angle = l;
            }
            b.create_revolute_joint(&d)
        };
        let front_joint = rev(b, frame, front_wheel, at("frontWheelShape")?, None);
        let back_joint = rev(b, frame, back_wheel, at("backWheelShape")?, None);
        let foot = at("footAnchor")?;
        let foot1 = rev(b, doll.bodies[&Part::LowerLeg1], frame, foot, Some(15.0 / deg));
        let foot2 = rev(b, doll.bodies[&Part::LowerLeg2], frame, foot, Some(15.0 / deg));
        let hand = at("handAnchor")?;
        let hand1 = rev(b, doll.bodies[&Part::LowerArm1], frame, hand, None);
        let hand2 = rev(b, doll.bodies[&Part::LowerArm2], frame, hand, None);
        doll.poses = PoseSet::Grinder;
        let (front_shape, back_shape) = (world.b2().body(front_wheel).shapes[0], world.b2().body(back_wheel).shapes[0]);
        Some(Self {
            frame,
            back_wheel,
            front_wheel,
            front_shape,
            back_shape,
            front_joint,
            back_joint,
            joints: [Some(hand1), Some(hand2), Some(foot1), Some(foot2)],
            ejected: false,
            connecting: false,
            wheel_pos: [at("frontWheelShape")?, at("backWheelShape")?],
            new_rail: [None, None],
            dongles: [None, None],
            one_dongle_counter: 0,
        })
    }

    fn rev_mut(world: &mut World, id: usize) -> Option<&mut box2d::joints::Revolute> {
        match &mut world.b2_mut().joint_mut(id).kind {
            box2d::joints::JointKind::Revolute(r) => Some(r),
            _ => None,
        }
    }

    /// `accelerate/decelerateMotorSpeed` for both wheels: a wheel on a rail drives its slider (step 0.5), a free wheel its
    /// revolute motor (step 1). `up` accelerates; otherwise the speed is braked to 0 / reversed.
    fn drive(&self, world: &mut World, up: bool) {
        for (n, (id, wheel)) in [(self.front_joint, self.front_wheel), (self.back_joint, self.back_wheel)].into_iter().enumerate() {
            if let Some(d) = self.dongles[n] {
                // wheel motor off, slider motor on (the "decelerate" branch's `EnableMotor(false)` only fires when
                // the motor is already off, i.e. it is a no-op)
                if up {
                    if let Some(r) = Self::rev_mut(world, id) {
                        r.enable_motor = false;
                    }
                }
                let speed = world.b2().prismatic_speed(d.slider).unwrap_or(0.0);
                if let box2d::joints::JointKind::Prismatic(p) = &mut world.b2_mut().joint_mut(d.slider).kind {
                    p.enable_motor = true;
                    p.motor_speed = if up {
                        if speed < Self::MAX_SPEED { speed + Self::PRIS_ACCEL } else { speed }
                    } else if speed > 0.0 {
                        0.0
                    } else if speed > -Self::MAX_SPEED {
                        speed - Self::PRIS_ACCEL
                    } else {
                        speed
                    };
                }
            } else {
                // GetJointSpeed = ω(body2) − ω(body1), body1 = frame
                let speed = world.b2().body(wheel).angular_velocity - world.b2().body(self.frame).angular_velocity;
                let Some(r) = Self::rev_mut(world, id) else { continue };
                r.enable_motor = true;
                r.motor_speed = if up {
                    if speed < Self::MAX_SPEED { speed + Self::ACCEL } else { speed }
                } else if speed > 0.0 {
                    0.0
                } else if speed > -Self::MAX_SPEED {
                    speed - Self::ACCEL
                } else {
                    speed
                };
            }
        }
    }

    fn remove_dongle(&mut self, world: &mut World, n: usize) {
        if let Some(d) = self.dongles[n].take() {
            world.b2_mut().destroy_body(d.body);
            self.one_dongle_counter = 0;
        }
    }

    /// `wheelIsCloseToRail`: within 37 px of the rail's centre line and inside its length (+ the wheel radius).
    fn wheel_close_to_rail(&self, world: &World, wheel: BodyId, rail: BodyId) -> bool {
        let b = world.b2();
        let local = b.body(rail).local_point(b.body(wheel).xf.position);
        if (local.y * 62.5).abs() > 37.0 {
            return false;
        }
        // `GetShapeList()` is the newest shape of the rail; its first vertex has x = −half-width
        let rail_shape = *b.body(rail).shapes.last().unwrap_or(&0);
        let half = match &b.shape_first_vertex(rail_shape) {
            Some(v) => v.x.abs(),
            None => 0.0,
        };
        let radius = b.shape_radius(b.body(wheel).shapes[0]);
        local.x.abs() <= half + radius
    }

    /// `createDongle` + `createDongleRevJoint` + `createPrisJoint`.
    fn attach_dongle(&mut self, world: &mut World, n: usize, rail: BodyId) {
        let wheel = if n == 0 { self.front_wheel } else { self.back_wheel };
        let pos = self.wheel_pos[n];
        let b = world.b2_mut();
        let existing = self.dongles[n];
        let (body, rev) = match existing {
            Some(d) => {
                b.destroy_joint(d.slider);
                (d.body, d.rev)
            }
            None => {
                let frame_pt = b.body(self.frame).world_point(pos);
                let dongle = b.create_body(&BodyDef { position: frame_pt, angle: b.body(rail).sweep.a, fixed_rotation: true, ..Default::default() });
                let mut sd = ShapeDef::boxed(0.25, 0.25);
                sd.density = 375.0;
                sd.is_sensor = true;
                b.create_shape(dongle, &sd);
                b.set_mass_from_shapes(dongle);
                let mut rd = RevoluteJointDef::initialize(b.body(dongle), dongle, b.body(self.frame), self.frame, b.body(dongle).xf.position);
                rd.local_anchor1 = Vec2::default();
                rd.local_anchor2 = pos;
                rd.max_motor_torque = 300.0;
                (dongle, b.create_revolute_joint(&rd))
            }
        };
        // getPrisJointSpeed: the dongle takes the wheel's velocity component along the rail
        let ang = b.body(rail).sweep.a;
        let dir = Vec2::new(box2d::fdlibm::cos(ang), box2d::fdlibm::sin(ang));
        let v = b.body(wheel).linear_velocity;
        let dot = v.x * dir.x + v.y * dir.y;
        b.body_mut(body).linear_velocity = Vec2::new(dir.x * dot, dir.y * dot);
        let rot = ang + std::f64::consts::FRAC_PI_2;
        let attach_world = b.body(self.frame).world_point(pos);
        let mut local = b.body(rail).local_point(attach_world);
        local.y = -23.0 / 62.5;
        let anchor = b.body(rail).world_point(local);
        let mut pd = PrismaticJointDef::initialize(b.body(body), body, b.body(rail), rail, anchor, Vec2::new(-box2d::fdlibm::sin(rot), box2d::fdlibm::cos(rot)));
        pd.max_motor_force = 1000.0;
        pd.motor_speed = dot;
        pd.enable_motor = false;
        pd.local_anchor1 = Vec2::default();
        pd.local_anchor2 = local;
        let slider = b.create_prismatic_joint(&pd);
        self.dongles[n] = Some(Dongle { body, rail, rev, slider });
    }

    /// `actions`: drop dongles when space is released or the wheel left the rail, attach newly touched rails, cancel
    /// gravity on dongles, and drop a lone dongle after 35 frames.
    fn rail_actions(&mut self, world: &mut World) {
        if self.dongles.iter().any(Option::is_some) {
            if !self.connecting {
                for n in 0..2 {
                    self.remove_dongle(world, n);
                }
                // `checkRemoveRailJoints`: `DoubleClickReverse` at the frame, volume `clickVolume` 0.6
                let at = world.b2().body(self.frame).sweep.c;
                world.play_scaled("DoubleClickReverse", at, 0.6);
            } else {
                for n in 0..2 {
                    let Some(d) = self.dongles[n] else { continue };
                    let wheel = if n == 0 { self.front_wheel } else { self.back_wheel };
                    let gap = world.b2().revolute_reaction(d.rev).map_or(0.0, |r| r.1);
                    if !self.wheel_close_to_rail(world, wheel, d.rail) || gap * gap > 0.1 {
                        self.remove_dongle(world, n);
                    }
                }
            }
        }
        if !self.ejected {
            // `DoubleClick` (volume 0.6) at the newly created dongle (the back one wins when both were created)
            let mut created = None;
            for n in 0..2 {
                if let Some(rail) = self.new_rail[n].take() {
                    let fresh = self.dongles[n].is_none();
                    self.attach_dongle(world, n, rail);
                    if fresh {
                        created = self.dongles[n].map(|d| d.body);
                    }
                }
            }
            if let Some(body) = created {
                let at = world.b2().body(body).sweep.c;
                world.play_scaled("DoubleClick", at, 0.6);
            }
        }
        for d in self.dongles.iter().flatten() {
            world.b2_mut().body_mut(d.body).linear_velocity.y += 1.0 / 30.0 * -10.0;
        }
        let (front, back) = (self.dongles[0].is_some(), self.dongles[1].is_some());
        if front != back {
            self.one_dongle_counter += 1;
            if self.one_dongle_counter == 35 {
                self.remove_dongle(world, if front { 0 } else { 1 });
            }
        }
    }
    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            match (k.left, k.right) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.left { 1 } else { 2 }),
                (true, false) | (false, true) => {
                    let left = k.left;
                    let b = world.b2_mut();
                    let (angle, av) = (b.body(self.frame).sweep.a, b.body(self.frame).angular_velocity);
                    let f = if left { (av + Self::MAX_SPIN) / Self::MAX_SPIN } else { (av - Self::MAX_SPIN) / -Self::MAX_SPIN };
                    let f = f.clamp(0.0, 1.0);
                    let (c, s) = (box2d::fdlibm::cos(angle) * Self::IMPULSE * f, box2d::fdlibm::sin(angle) * Self::IMPULSE * f);
                    let lc = b.body(self.frame).sweep.local_center;
                    let off = if left { Self::OFFSET } else { -Self::OFFSET };
                    let point = b.body(self.frame).world_point(Vec2::new(lc.x + off, lc.y));
                    b.body_mut(self.frame).apply_impulse(Vec2::new(s, -c), point);
                }
                _ if ejected && (doll.pose == 1 || doll.pose == 2) => doll.set_pose(0),
                _ => {}
            }
            match (k.up, k.down) {
                (true, false) | (false, true) if ejected => doll.set_pose(if k.up { 3 } else { 4 }),
                (true, false) | (false, true) => self.drive(world, k.up),
                _ => {
                    for d in self.dongles.iter().flatten() {
                        if let box2d::joints::JointKind::Prismatic(p) = &mut world.b2_mut().joint_mut(d.slider).kind {
                            p.enable_motor = false;
                        }
                    }
                    let motors = Self::rev_mut(world, self.back_joint).is_some_and(|r| r.enable_motor);
                    if motors {
                        for id in [self.back_joint, self.front_joint] {
                            if let Some(r) = Self::rev_mut(world, id) {
                                r.enable_motor = false;
                            }
                        }
                    }
                    if ejected && (doll.pose == 3 || doll.pose == 4) {
                        doll.set_pose(0);
                    }
                }
            }
            if k.space {
                if ejected {
                    doll.set_grab(world, true);
                } else {
                    self.connecting = true;
                }
            } else {
                if ejected {
                    doll.set_grab(world, false);
                }
                self.connecting = false;
                for n in 0..2 {
                    self.remove_dongle(world, n);
                }
            }
            if k.shift {
                doll.set_pose(if ejected { 5 } else { 8 });
            } else if doll.pose == 5 || doll.pose == 8 {
                doll.set_pose(0);
            }
            if k.ctrl {
                doll.set_pose(if ejected { 6 } else { 7 });
            } else if doll.pose == 6 || doll.pose == 7 {
                doll.set_pose(0);
            }
            if k.z {
                self.eject(world, doll);
            }
        }
        doll.apply_pose(world);
        self.rail_actions(world);
    }

    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        doll.reset_joint_limits(world);
        let zero2 = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: -2 };
        let b = world.b2_mut();
        for j in self.joints.iter_mut().filter_map(Option::take) {
            b.destroy_joint(j);
        }
        for w in [self.front_wheel, self.back_wheel] {
            let s = b.body(w).shapes[0];
            b.refilter(s);
        }
        let shapes: Vec<_> = b.body(self.frame).shapes.iter().rev().copied().collect();
        for s in shapes {
            b.set_shape_filter(s, zero2);
        }
    }
}

/// Daughter of the c10 family (class `My`, deobfuscated.js:20113, groupID −2, shapeRefScale 50): a ragdoll on a
/// trailer bike (frame of five polygons, wheel, gear, gear joint) hitched to the dad's frame by a revolute
/// joint limited to ±35°. Her wheel motor (torque 20, max speed 27.7, step 1.385) follows up/down, space brakes,
/// ctrl/z eject her. NOT ported: frame smash / `detachFrame` (joint break at reaction 400), reAttach, sounds.
pub struct Daughter {
    pub doll: Ragdoll,
    pub frame: BodyId,
    pub wheel: BodyId,
    pub gear: BodyId,
    wheel_joint: usize,
    connecting: Option<usize>,
    pelvis: Option<usize>,
    hands: [Option<usize>; 2],
    feet: [Option<usize>; 2],
    pub ejected: bool,
    frame_shape: box2d::world::ShapeId,
    /// `frameSmash` happened: the frame body is gone, three pieces (fork, broken frame, seat) took its place.
    pub frame_gone: bool,
    pub debris: Vec<BodyId>,
    broken: [Vec<Vec2>; 3],
}

impl Daughter {
    const MAX_SPEED: f64 = 27.7;
    const ACCEL: f64 = 1.385;

    pub fn attach(world: &mut World, start_px: [f32; 2], dad_frame: BodyId) -> Option<Self> {
        let guide = crate::character::subguide("c10_daughter")?;
        // The kid's guide has no chest/foot extras the base needs beyond the standard parts.
        let mut doll = Ragdoll::spawn_full_deferred(world, &guide, start_px, false, 50.0, -2)?;
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let poly = |prefix: &str, n: usize| -> Option<Vec<Vec2>> { (1..=n).map(|i| at(&format!("{prefix}{i}"))).collect() };
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        let seat_f = Filter { category_bits: 513, mask_bits: 0xFFFF, group_index: 0 };
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let frame = b.create_body(&BodyDef::default());
        let mut sensors = Vec::new();
        for (verts, filter) in [
            (poly("seatVert", 3)?, seat_f),
            (poly("frameVert", 3)?, zero),
            (poly("handleVert", 3)?, zero),
            (poly("midVert", 4)?, zero),
            (poly("endVert", 4)?, zero),
        ] {
            let mut d = ShapeDef::new(Geometry::Polygon(verts));
            d.density = 2.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = filter;
            sensors.push(b.create_shape(frame, &d));
        }
        b.set_mass_from_shapes(frame);
        let mut round = |name: &str| -> Option<BodyId> {
            let g = guide.get(name)?;
            let body = b.create_body(&BodyDef { position: at(name)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: g[5] / 2.0 / CHARACTER_SCALE });
            d.density = 5.0;
            d.friction = 1.0;
            d.restitution = 0.3;
            d.filter = seat_f;
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            Some(body)
        };
        let wheel = round("wheelShape")?;
        let gear = round("gearShape")?;
        for part in [Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2] {
            let s = b.body(doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = true;
        }
        b.shape_mut(sensors[4]).is_sensor = true;
        doll.finish_joints(world);
        for (limb, lo, hi) in [(Limb::Hip1, -110.0, 10.0), (Limb::Hip2, -110.0, 10.0), (Limb::Elbow1, -60.0, 0.0), (Limb::Elbow2, -60.0, 0.0), (Limb::Neck, -20.0, 0.0)] {
            doll.set_limits(world, limb, lo / deg, hi / deg);
        }
        let b = world.b2_mut();
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2, limit: Option<f64>| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 20.0;
            if let Some(l) = limit {
                d.enable_limit = true;
                d.lower_angle = -l;
                d.upper_angle = l;
            }
            b.create_revolute_joint(&d)
        };
        let wheel_joint = rev(b, frame, wheel, at("wheelShape")?, None);
        let pelvis_body = doll.bodies[&Part::Pelvis];
        let pp = b.body(pelvis_body).xf.position;
        let pelvis = rev(b, frame, pelvis_body, pp, None);
        let hand = at("handleAnchor")?;
        let h1 = rev(b, frame, doll.bodies[&Part::LowerArm1], hand, None);
        let h2 = rev(b, frame, doll.bodies[&Part::LowerArm2], hand, None);
        let frame_gear = rev(b, frame, gear, at("gearShape")?, None);
        let gj = b.create_gear_joint(wheel_joint, frame_gear, 1.0)?;
        b.flip_gear_ratio(gj);
        let f1 = rev(b, gear, doll.bodies[&Part::LowerLeg1], at("gearAnchor1")?, None);
        let f2 = rev(b, gear, doll.bodies[&Part::LowerLeg2], at("gearAnchor2")?, None);
        let connecting = rev(b, dad_frame, frame, at("connectAnchor")?, Some(35.0 / deg));
        Some(Self { doll, frame, wheel, gear, wheel_joint, connecting: Some(connecting), pelvis: Some(pelvis), hands: [Some(h1), Some(h2)], feet: [Some(f1), Some(f2)], ejected: false, frame_shape: sensors[1], frame_gone: false, debris: Vec::new(), broken: [poly("broken2Vert", 3)?, poly("brokenVert", 4)?, poly("seatVert", 3)?] })
    }

    fn motor_mut<'a>(&self, world: &'a mut World) -> Option<&'a mut box2d::joints::Revolute> {
        match &mut world.b2_mut().joint_mut(self.wheel_joint).kind {
            box2d::joints::JointKind::Revolute(r) => Some(r),
            _ => None,
        }
    }

    pub fn keys(&mut self, world: &mut World, k: Keys) {
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !self.doll.dead {
            match (k.left, k.right) {
                (true, false) | (false, true) if ejected => self.doll.set_pose(if k.left { 1 } else { 2 }),
                _ if ejected && (self.doll.pose == 1 || self.doll.pose == 2) => self.doll.set_pose(0),
                _ => {}
            }
            match (k.up, k.down) {
                (true, false) | (false, true) => {
                    let up = k.up;
                    if ejected {
                        self.doll.set_pose(if up { 3 } else { 4 });
                    } else {
                        let b = world.b2();
                        let speed = b.body(self.wheel).angular_velocity - b.body(self.frame).angular_velocity;
                        let new = if up {
                            if speed < 0.0 { 0.0 } else if speed < Self::MAX_SPEED { speed + Self::ACCEL } else { speed }
                        } else if speed > 0.0 {
                            0.0
                        } else if speed > -Self::MAX_SPEED {
                            speed - Self::ACCEL
                        } else {
                            speed
                        };
                        if let Some(r) = self.motor_mut(world) {
                            r.enable_motor = true;
                            r.motor_speed = new;
                        }
                    }
                }
                _ => {
                    if let Some(r) = self.motor_mut(world) {
                        r.enable_motor = false;
                    }
                    if ejected && (self.doll.pose == 3 || self.doll.pose == 4) {
                        self.doll.set_pose(0);
                    }
                }
            }
            if ejected {
                self.doll.set_grab(world, k.space);
            } else if k.space {
                if let Some(r) = self.motor_mut(world) {
                    r.enable_motor = true;
                    r.motor_speed = 0.0;
                }
            }
            if k.ctrl || k.z {
                self.eject(world);
            }
        }
        self.doll.apply_pose(world);
    }

    /// `checkJoints`: her ragdoll's limbs, then `checkRevJoint(connectingJoint, detachLimit 400, detachFrame)`.
    /// Limb-break reactions (original ~20660): shoulder/elbow breaks drop that hand's joint, hip/knee breaks that foot's (and the
    /// pelvis joint once both feet are gone); eject once all four hands/feet are gone.
    fn limbs_broken(&mut self, world: &mut World, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        for l in broken {
            if *l == Limb::Waist {
                self.eject(world);
                return;
            }
            let (is_hand, k) = match l {
                Limb::Shoulder1 | Limb::Elbow1 => (true, 0),
                Limb::Shoulder2 | Limb::Elbow2 => (true, 1),
                Limb::Hip1 | Limb::Knee1 => (false, 0),
                Limb::Hip2 | Limb::Knee2 => (false, 1),
                _ => continue,
            };
            let slot = if is_hand { &mut self.hands[k] } else { &mut self.feet[k] };
            if let Some(j) = slot.take() {
                world.b2_mut().destroy_joint(j);
                if !is_hand && self.feet.iter().all(Option::is_none) {
                    if let Some(p) = self.pelvis.take() {
                        world.b2_mut().destroy_joint(p);
                    }
                }
            }
            if self.hands.iter().chain(&self.feet).all(Option::is_none) {
                self.eject(world);
                return;
            }
        }
    }

    pub fn check_joints(&mut self, world: &mut World) {
        // each override (`hipBreak1`, …) runs right after its own break
        for limb in crate::character::CHECK_ORDER {
            if self.doll.check_joint(world, limb) {
                self.limbs_broken(world, &[limb]);
            }
        }
        self.doll.check_composites(world);
        if let Some(id) = self.connecting {
            if let Some((force, gap)) = world.b2().revolute_reaction(id) {
                if force > 400.0 || gap > 0.5 {
                    self.detach(world);
                }
            }
        }
    }

    /// `detachFrame`: the hitch breaks and her body parts leave the seat's filter group.
    pub fn detach(&mut self, world: &mut World) {
        let Some(id) = self.connecting.take() else { return };
        let extra = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: self.doll.group };
        let b = world.b2_mut();
        b.destroy_joint(id);
        use Part::*;
        for part in [Head, Pelvis, Chest, UpperArm1, UpperArm2, LowerArm1, LowerArm2] {
            let s = b.body(self.doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = false;
            if b.shape_mut(s).filter.mask_bits == 270 {
                b.shape_mut(s).filter = extra;
            }
            b.refilter(s);
        }
        // the rear quad, a sensor while she is hitched
        let end = b.body(self.frame).shapes[4];
        b.shape_mut(end).is_sensor = false;
        b.refilter(end);
        // `playAreaSoundInstance("StrapSnap1", frameBody)`
        let at = world.b2().body(self.frame).sweep.c;
        world.play("StrapSnap1", 1, at);
    }

    /// `frameSmash` (limit 50 on the frame shape): eject, then the frame body is replaced by fork / broken frame / seat bodies (density 2,
    /// zero filter) that keep its linear and angular velocity. Hitch, wheel and gear joints go with the body.
    pub fn smashed(&mut self, world: &mut World, events: &[box2d::world::ContactEvent]) {
        // the body parts come before the frame shape in `contactImpulseDict`
        let hits = world.doll_smash_hits(&self.doll, events);
        for part in [Part::Head, Part::Chest, Part::Pelvis] {
            if !hits.contains(&part) {
                continue;
            }
            smash_part(self, world, part, |p| &mut p.doll, Self::limbs_broken);
            self.eject(world);
        }
        for (leg, part) in [Part::LowerLeg1, Part::LowerLeg2].into_iter().enumerate() {
            if hits.contains(&part) {
                self.doll.smash_foot(world, leg);
            }
        }
        if self.frame_gone {
            return;
        }
        let s = self.frame_shape;
        if !events.iter().any(|e| e.kind == box2d::world::ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > 50.0) {
            return;
        }
        self.frame_smash(world);
    }

    pub fn frame_smash(&mut self, world: &mut World) {
        if self.frame_gone {
            return;
        }
        self.eject(world);
        self.frame_gone = true;
        self.connecting = None;
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let src = world.b2().body(self.frame).clone();
        let b = world.b2_mut();
        b.destroy_body(self.frame);
        for v in &self.broken {
            let piece = b.create_body(&BodyDef { position: src.xf.position, angle: src.sweep.a, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Polygon(v.clone()));
            d.density = 2.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(piece, &d);
            b.set_mass_from_shapes(piece);
            b.body_mut(piece).linear_velocity = src.linear_velocity;
            b.body_mut(piece).angular_velocity = src.angular_velocity;
            self.debris.push(piece);
        }
        world.play("BikeSmash1", 1, src.sweep.c);
    }

    pub fn eject(&mut self, world: &mut World) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        self.doll.reset_joint_limits(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let extra = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: self.doll.group };
        let b = world.b2_mut();
        for j in [self.pelvis.take(), self.hands[0].take(), self.hands[1].take(), self.feet[0].take(), self.feet[1].take()].into_iter().flatten() {
            b.destroy_joint(j);
        }
        // wheel, then the frame's shapes (seat, frame, mid, handle; the end shape keeps its sensor state)
        let wheel_s = b.body(self.wheel).shapes[0];
        b.set_shape_filter(wheel_s, zero);
        // original order: seat, frame, mid, handle (creation order is seat, frame, handle, mid, end)
        let made: Vec<_> = b.body(self.frame).shapes.clone();
        for idx in [0, 1, 3, 2] {
            b.set_shape_filter(made[idx], zero);
        }
        // refilterShit: sensors off, default-mask shapes move to the group's "extra" filter
        use Part::*;
        for part in [Head, Pelvis, Chest, UpperArm1, UpperArm2, LowerArm1, LowerArm2, UpperLeg1, UpperLeg2, LowerLeg1, LowerLeg2] {
            let s = b.body(self.doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = false;
            if b.shape_mut(s).filter.mask_bits == 270 {
                b.shape_mut(s).filter = extra;
            }
            b.refilter(s);
        }
        if let Some(r) = self.motor_mut(world) {
            r.enable_motor = false;
        }
    }
}

/// Son of the c10 family (class `MK`, deobfuscated.js:20830, groupID −3, shapeRefScale 50): a ragdoll sitting in a
/// basket (three quads, density 1, collides with nothing of the dad's group) that is welded to the dad's frame by a
/// locked revolute (breaks at reaction 100 → `detachBasket`), with his hands on the basket and his pelvis on a short
/// prismatic joint (0..0.25 m along 25° off vertical). Shift/z eject him (impulse 0.75 perpendicular to the basket).
/// NOT ported: sounds.
pub struct Son {
    pub doll: Ragdoll,
    pub basket: BodyId,
    connecting: Option<usize>,
    pelvis: Option<usize>,
    hands: [Option<usize>; 2],
    basket_shapes: Vec<box2d::world::ShapeId>,
    /// The replacement walls of `detachBasket`.
    spare_walls: Vec<Vec<Vec2>>,
    /// How far the basket's centre of mass moved when its walls were replaced (body-local m): the picture stays put.
    pub centre_shift: Vec2,
    pub ejected: bool,
    /// `basketSmash` happened: the basket body is gone.
    pub basket_gone: bool,
}

impl Son {
    const EJECT_IMPULSE: f64 = 0.75;

    pub fn attach(world: &mut World, start_px: [f32; 2], dad_frame: BodyId) -> Option<Self> {
        let guide = crate::character::subguide("c10_son")?;
        let mut doll = Ragdoll::spawn_full_deferred(world, &guide, start_px, false, 50.0, -3)?;
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        let basket = b.create_body(&BodyDef::default());
        let mut shapes = Vec::new();
        for prefix in ["crateRight", "crateBottom", "crateLeft"] {
            let verts: Option<Vec<Vec2>> = (1..=4).map(|i| at(&format!("{prefix}{i}"))).collect();
            let mut d = ShapeDef::new(Geometry::Polygon(verts?));
            d.density = 1.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: -1 }; // mom.defaultFilter
            shapes.push(b.create_shape(basket, &d));
        }
        b.set_mass_from_shapes(basket);
        let spare_walls = ["crate2Right", "crate2Bottom", "crate2Left"].iter().map(|prefix| (1..=4).map(|i| at(&format!("{prefix}{i}"))).collect::<Option<Vec<Vec2>>>()).collect::<Option<Vec<_>>>()?;
        doll.finish_joints(world);
        for (limb, lo, hi) in [(Limb::Knee1, 50.0, 150.0), (Limb::Knee2, 50.0, 150.0), (Limb::Elbow1, -160.0, -20.0), (Limb::Elbow2, -160.0, -20.0), (Limb::Neck, -10.0, 10.0)] {
            doll.set_limits(world, limb, lo / deg, hi / deg);
        }
        let b = world.b2_mut();
        let anchor = at("crateAnchor")?;
        let mut d = RevoluteJointDef::initialize(b.body(dad_frame), dad_frame, b.body(basket), basket, anchor);
        d.enable_limit = true;
        d.lower_angle = 0.0;
        d.upper_angle = 0.0;
        let connecting = b.create_revolute_joint(&d);
        let mut hands = [None, None];
        for (k, part) in [Part::LowerArm1, Part::LowerArm2].into_iter().enumerate() {
            let arm = doll.bodies[&part];
            let g = guide.get(if k == 0 { "lowerArm1Shape" } else { "lowerArm2Shape" })?;
            let p = b.body(arm).world_point(Vec2::new(0.0, g[4] * 50.0 / CHARACTER_SCALE));
            let hd = RevoluteJointDef::initialize(b.body(basket), basket, b.body(arm), arm, p);
            hands[k] = Some(b.create_revolute_joint(&hd));
        }
        let pelvis_body = doll.bodies[&Part::Pelvis];
        let c = b.body(pelvis_body).sweep.c;
        let a = 25.0 / deg;
        let mut pd = box2d::joints::PrismaticJointDef::initialize(b.body(basket), basket, b.body(pelvis_body), pelvis_body, c, Vec2::new(box2d::fdlibm::sin(a), -box2d::fdlibm::cos(a)));
        pd.enable_limit = true;
        pd.lower_translation = 0.0;
        pd.upper_translation = 0.25;
        let pelvis = b.create_prismatic_joint(&pd);
        Some(Self { doll, basket, connecting: Some(connecting), pelvis: Some(pelvis), hands, basket_shapes: shapes, spare_walls, centre_shift: Vec2::default(), ejected: false, basket_gone: false })
    }

    pub fn keys(&mut self, world: &mut World, k: Keys) {
        // `checkKeyStates` of a dead character runs none of the key actions
        if !self.doll.dead {
            if self.ejected {
                match (k.left, k.right) {
                    (true, false) => self.doll.set_pose(1),
                    (false, true) => self.doll.set_pose(2),
                    _ if self.doll.pose == 1 || self.doll.pose == 2 => self.doll.set_pose(0),
                    _ => {}
                }
                match (k.up, k.down) {
                    (true, false) => self.doll.set_pose(3),
                    (false, true) => self.doll.set_pose(4),
                    _ if self.doll.pose == 3 || self.doll.pose == 4 => self.doll.set_pose(0),
                    _ => {}
                }
                self.doll.set_grab(world, k.space);
            }
            if k.shift || k.z {
                self.eject(world);
            }
        }
        self.doll.apply_pose(world);
    }

    /// `headSmash1/chestSmash/pelvisSmash`: the base class's smash, then `eject()`; feet smash as in the base class.
    pub fn smashed(&mut self, world: &mut World, events: &[box2d::world::ContactEvent]) {
        let hits = world.doll_smash_hits(&self.doll, events);
        for part in [Part::Head, Part::Chest, Part::Pelvis] {
            if !hits.contains(&part) {
                continue;
            }
            smash_part(self, world, part, |p| &mut p.doll, Self::limbs_broken);
            self.eject(world);
        }
        for (leg, part) in [Part::LowerLeg1, Part::LowerLeg2].into_iter().enumerate() {
            if hits.contains(&part) {
                self.doll.smash_foot(world, leg);
            }
        }
    }

    fn refilter_parts(&self, world: &mut World, parts: &[Part]) {
        let extra = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: self.doll.group };
        let b = world.b2_mut();
        for part in parts {
            let s = b.body(self.doll.bodies[part]).shapes[0];
            if b.shape_mut(s).filter.mask_bits == 270 {
                b.shape_mut(s).filter = extra;
                b.refilter(s);
            }
        }
    }

    const ALL: [Part; 11] = [Part::Head, Part::Pelvis, Part::Chest, Part::UpperArm1, Part::UpperArm2, Part::LowerArm1, Part::LowerArm2, Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2];

    /// Limb-break reactions (original ~21240): shoulder/elbow breaks drop that hand's joint; eject once both hands are gone.
    fn limbs_broken(&mut self, world: &mut World, broken: &[Limb]) {
        if self.ejected {
            return;
        }
        for l in broken {
            if *l == Limb::Waist {
                self.eject(world);
                return;
            }
            let k = match l {
                Limb::Shoulder1 | Limb::Elbow1 => 0,
                Limb::Shoulder2 | Limb::Elbow2 => 1,
                _ => continue,
            };
            if let Some(j) = self.hands[k].take() {
                world.b2_mut().destroy_joint(j);
            }
            if self.hands.iter().all(Option::is_none) {
                self.eject(world);
                return;
            }
        }
    }

    pub fn check_joints(&mut self, world: &mut World) {
        // each override (`hipBreak1`, …) runs right after its own break
        for limb in crate::character::CHECK_ORDER {
            if self.doll.check_joint(world, limb) {
                self.limbs_broken(world, &[limb]);
            }
        }
        self.doll.check_composites(world);
        if let Some(id) = self.connecting {
            if let Some((force, gap)) = world.b2().revolute_reaction(id) {
                if force > 100.0 || gap > 0.5 {
                    self.detach(world);
                }
            }
        }
    }

    /// `contactBasketResultHandler` + `basketSmash` (limit 3 on shape 1): eject, drop the hitch and destroy the basket body.
    pub fn smash_basket(&mut self, world: &mut World, events: &[box2d::world::ContactEvent]) {
        if self.basket_gone || self.basket_shapes.is_empty() {
            return;
        }
        // one handler is registered on all three walls
        let hit = events.iter().find(|e| e.kind == box2d::world::ContactEventKind::Result && (self.basket_shapes.contains(&e.shape1) || self.basket_shapes.contains(&e.shape2)) && e.normal_impulse > 3.0);
        if hit.is_none() {
            return;
        }
        self.eject(world);
        self.basket_gone = true;
        self.basket_shapes.clear();
        if let Some(j) = self.connecting.take() {
            world.b2_mut().destroy_joint(j);
        }
        let c = world.b2().body(self.basket).sweep.c;
        world.b2_mut().destroy_body(self.basket);
        world.shard_burst(crate::blood::ShardKind::Basket, c, 5.0, 20.0, 30);
        world.play("BasketSmash", 1, c);
    }

    /// `detachBasket`: the hitch breaks, the rider leaves the group filter, and the walls are replaced by ones that
    /// collide with everything.
    /// `detached`: the basket is no longer welded to the dad's bike.
    pub fn detached(&self) -> bool {
        self.connecting.is_none()
    }

    pub fn detach(&mut self, world: &mut World) {
        let Some(id) = self.connecting.take() else { return };
        world.b2_mut().destroy_joint(id);
        for s in std::mem::take(&mut self.basket_shapes) {
            world.b2_mut().destroy_shape(s);
        }
        if !self.ejected {
            self.refilter_parts(world, &Self::ALL);
        }
        let b = world.b2_mut();
        let before = b.body(self.basket).sweep.local_center;
        for wall in &self.spare_walls {
            let mut d = ShapeDef::new(Geometry::Polygon(wall.clone()));
            d.density = 1.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
            self.basket_shapes.push(b.create_shape(self.basket, &d));
        }
        b.set_mass_from_shapes(self.basket);
        let after = b.body(self.basket).sweep.local_center;
        self.centre_shift = Vec2::new(after.x - before.x, after.y - before.y);
    }

    pub fn eject(&mut self, world: &mut World) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        self.doll.reset_joint_limits(world);
        let b = world.b2_mut();
        for j in [self.pelvis.take(), self.hands[0].take(), self.hands[1].take()].into_iter().flatten() {
            b.destroy_joint(j);
        }
        if self.connecting.is_some() {
            self.refilter_parts(world, &Self::ALL);
        }
        let b = world.b2_mut();
        let a = b.body(self.basket).sweep.a - std::f64::consts::FRAC_PI_2;
        let imp = Vec2::new(box2d::fdlibm::cos(a) * Self::EJECT_IMPULSE, box2d::fdlibm::sin(a) * Self::EJECT_IMPULSE);
        for part in [Part::Chest, Part::Pelvis] {
            let id = self.doll.bodies[&part];
            let wc = b.body(id).sweep.c;
            b.body_mut(id).apply_impulse(imp, wc);
        }
    }
}

/// The c3 dad's child (class `MY`, deobfuscated.js:19750, groupID −2, ref scale 5, guide `c3_kid`): a plain ragdoll on a child seat (two
/// quads, density 1, plus a sensor quad) that is welded (limit 0..0) to the dad's frame at `frameSeatAnchor` — breaks at reaction 200
/// (`detachSeat`) — and joined to the seat by free revolutes at chest, pelvis and both thighs. Ctrl/z eject it (chest/pelvis smash and
/// neck/waist breaks too); once ejected it takes poses/grab like the other kids. NOT ported: kid smash sounds/mourning, `dead` setter.
pub struct Kid {
    pub doll: Ragdoll,
    pub seat: BodyId,
    frame_seat: Option<usize>,
    chest: Option<usize>,
    pelvis: Option<usize>,
    legs: [Option<usize>; 2],
    pub ejected: bool,
    pub detached: bool,
}

impl Kid {
    const DETACH_LIMIT: f64 = 200.0;
    const ALL: [Part; 11] = [Part::Head, Part::Pelvis, Part::Chest, Part::UpperArm1, Part::UpperArm2, Part::LowerArm1, Part::LowerArm2, Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2];

    pub fn attach(world: &mut World, start_px: [f32; 2], dad_frame: BodyId) -> Option<Self> {
        let guide = crate::character::subguide("c3_kid")?;
        let mut doll = Ragdoll::spawn_full_deferred(world, &guide, start_px, false, 5.0, -2)?;
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let b = world.b2_mut();
        let seat = b.create_body(&BodyDef::default());
        for (n, sensor) in [(1, false), (2, false), (3, true)] {
            let verts: Option<Vec<Vec2>> = (1..=4).map(|i| at(&format!("seat{n}Vert{i}"))).collect();
            let mut d = ShapeDef::new(Geometry::Polygon(verts?));
            d.density = 1.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: -2 };
            d.is_sensor = sensor;
            b.create_shape(seat, &d);
        }
        b.set_mass_from_shapes(seat);
        doll.finish_joints(world);
        let b = world.b2_mut();
        let anchor = at("frameSeatAnchor")?;
        let mut d = RevoluteJointDef::initialize(b.body(dad_frame), dad_frame, b.body(seat), seat, anchor);
        d.enable_limit = true;
        d.lower_angle = 0.0;
        d.upper_angle = 0.0;
        let frame_seat = b.create_revolute_joint(&d);
        let free = |b: &mut box2d::world::World, part: Part| {
            let body = doll.bodies[&part];
            let p = b.body(body).xf.position;
            let d = RevoluteJointDef::initialize(b.body(seat), seat, b.body(body), body, p);
            b.create_revolute_joint(&d)
        };
        let chest = free(b, Part::Chest);
        let pelvis = free(b, Part::Pelvis);
        let leg1 = free(b, Part::UpperLeg1);
        let leg2 = free(b, Part::UpperLeg2);
        Some(Self { doll, seat, frame_seat: Some(frame_seat), chest: Some(chest), pelvis: Some(pelvis), legs: [Some(leg1), Some(leg2)], ejected: false, detached: false })
    }

    pub fn keys(&mut self, world: &mut World, k: Keys) {
        // `checkKeyStates` of a dead character runs none of the key actions
        if !self.doll.dead {
            if self.ejected {
                match (k.left, k.right) {
                    (true, false) => self.doll.set_pose(1),
                    (false, true) => self.doll.set_pose(2),
                    _ if self.doll.pose == 1 || self.doll.pose == 2 => self.doll.set_pose(0),
                    _ => {}
                }
                match (k.up, k.down) {
                    (true, false) => self.doll.set_pose(3),
                    (false, true) => self.doll.set_pose(4),
                    _ if self.doll.pose == 3 || self.doll.pose == 4 => self.doll.set_pose(0),
                    _ => {}
                }
                self.doll.set_grab(world, k.space);
            }
            if k.ctrl || k.z {
                self.eject(world);
            }
        }
        self.doll.apply_pose(world);
    }

    /// `refilterShit`: parts still on the group mask 270 move to the group's extra filter (collide with everything).
    fn refilter_parts(&self, world: &mut World) {
        let extra = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: self.doll.group };
        let b = world.b2_mut();
        for part in Self::ALL {
            let s = b.body(self.doll.bodies[&part]).shapes[0];
            if b.shape_mut(s).filter.mask_bits == 270 {
                b.shape_mut(s).filter = extra;
                b.refilter(s);
            }
        }
    }

    /// Limb-break reactions (original 20026): a hip break drops that thigh's seat joint; chest/pelvis smash and neck/waist breaks eject.
    fn limbs_broken(&mut self, world: &mut World, broken: &[Limb]) {
        for l in broken {
            match l {
                Limb::Hip1 | Limb::Hip2 => {
                    let k = usize::from(matches!(l, Limb::Hip2));
                    if let Some(j) = self.legs[k].take() {
                        world.b2_mut().destroy_joint(j);
                    }
                }
                Limb::Waist => self.eject(world),
                _ => {}
            }
        }
    }

    /// `contactResultHandler` smash reactions: head/chest/pelvis smash their parts (base class), kid: chest/pelvis eject, head only smashes.
    pub fn smashed(&mut self, world: &mut World, events: &[box2d::world::ContactEvent]) {
        let hits = world.doll_smash_hits(&self.doll, events);
        for part in [Part::Head, Part::Chest, Part::Pelvis] {
            if !hits.contains(&part) {
                continue;
            }
            smash_part(self, world, part, |p| &mut p.doll, Self::limbs_broken);
            if part != Part::Head {
                self.eject(world);
            }
        }
    }

    /// `checkJoints`: the kid's limbs, then `checkRevJoint(frameSeatJoint, 200, detachSeat)`.
    pub fn check_joints(&mut self, world: &mut World) {
        // each override (`hipBreak1`, …) runs right after its own break
        for limb in crate::character::CHECK_ORDER {
            if self.doll.check_joint(world, limb) {
                self.limbs_broken(world, &[limb]);
            }
        }
        self.doll.check_composites(world);
        if let Some(id) = self.frame_seat {
            if let Some((force, gap)) = world.b2().revolute_reaction(id) {
                if force > Self::DETACH_LIMIT || gap > 0.5 {
                    self.detach(world);
                }
            }
        }
    }

    /// `detachSeat`: the seat leaves the dad; seat shapes become solid (ejected: just unsensored; seated: kid parts and seat move to the extra filter).
    pub fn detach(&mut self, world: &mut World) {
        let Some(id) = self.frame_seat.take() else { return };
        self.detached = true;
        world.b2_mut().destroy_joint(id);
        if !self.ejected {
            self.refilter_parts(world);
        }
        let extra = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: self.doll.group };
        let b = world.b2_mut();
        let shapes: Vec<_> = b.body(self.seat).shapes.iter().rev().copied().collect();
        for s in shapes {
            if !self.ejected {
                b.shape_mut(s).filter = extra;
            }
            b.shape_mut(s).is_sensor = false;
            b.refilter(s);
        }
    }

    pub fn eject(&mut self, world: &mut World) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        self.doll.reset_joint_limits(world);
        let b = world.b2_mut();
        for j in [self.pelvis.take(), self.chest.take(), self.legs[0].take(), self.legs[1].take()].into_iter().flatten() {
            b.destroy_joint(j);
        }
        if !self.detached {
            self.refilter_parts(world);
        }
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let shapes: Vec<_> = b.body(self.seat).shapes.iter().rev().copied().collect();
        for s in shapes {
            b.set_shape_filter(s, zero);
        }
    }
}

/// The moped girl (class `MD`, deobfuscated.js:26051, group −2, ref scale 5, guide `c5_girl`) riding behind the c5 rider: a ragdoll whose
/// pelvis is pinned to his frame, hands to his chest and feet to his frame (revolutes, torque 30); legs and lower arms are sensors until
/// she is ejected and her thighs stop touching his seat (`legsFree`). Only the rider's shift ejects her (the base z/ctrl do nothing), and his
/// frame smash does too; a second eject of the rider lets her let go of his torso (`releaseGuy`). Limits: elbows −90..0°, neck −20..0°
/// (hips are computed but never applied). NOT ported: her head/chest/pelvis smash reactions, both sides' `reAttaching`, sounds, `dead`.
pub struct Girl {
    pub doll: Ragdoll,
    frame_pelvis: Option<usize>,
    torso_hands: [Option<usize>; 2],
    frame_feet: [Option<usize>; 2],
    seat_shape: box2d::world::ShapeId,
    hand_defs: [RevoluteJointDef; 2],
    torso_anchor: Vec2,
    reattaching: bool,
    leg_contacts: [i32; 2],
    listening: bool,
    legs_free_pending: bool,
    /// The bike's frame is smashed (`!guy.tankShape`): nothing is left for her legs to be stuck in.
    frame_gone: bool,
    pub ejected: bool,
}

impl Girl {
    pub fn attach(world: &mut World, start_px: [f32; 2], guy_frame: BodyId, guy_chest: BodyId) -> Option<Self> {
        let guide = crate::character::subguide("c5_girl")?;
        let mut doll = Ragdoll::spawn_full_deferred(world, &guide, start_px, false, 5.0, -2)?;
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        for part in [Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2, Part::LowerArm1, Part::LowerArm2] {
            let s = b.body(doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = true;
        }
        let seat_shape = b.body(guy_frame).shapes.get(3).copied()?;
        doll.finish_joints(world);
        for (limb, lo, hi) in [(Limb::Elbow1, -90.0, 0.0), (Limb::Elbow2, -90.0, 0.0), (Limb::Neck, -20.0, 0.0)] {
            doll.set_limits(world, limb, lo / deg, hi / deg);
        }
        let b = world.b2_mut();
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 30.0;
            b.create_revolute_joint(&d)
        };
        let pelvis = doll.bodies[&Part::Pelvis];
        let pp = b.body(pelvis).xf.position;
        let frame_pelvis = rev(b, guy_frame, pelvis, pp);
        let hand = at("handleAnchor")?;
        let mk = |b: &mut box2d::world::World, arm: Part| {
            let mut d = RevoluteJointDef::initialize(b.body(guy_chest), guy_chest, b.body(doll.bodies[&arm]), doll.bodies[&arm], hand);
            d.max_motor_torque = 30.0;
            d
        };
        let hand_defs = [mk(b, Part::LowerArm1), mk(b, Part::LowerArm2)];
        let torso_anchor = b.body(guy_chest).local_point(hand);
        let h1 = b.create_revolute_joint(&hand_defs[0]);
        let h2 = b.create_revolute_joint(&hand_defs[1]);
        let foot = at("footAnchor")?;
        let f1 = rev(b, guy_frame, doll.bodies[&Part::LowerLeg1], foot);
        let f2 = rev(b, guy_frame, doll.bodies[&Part::LowerLeg2], foot);
        Some(Self { doll, frame_pelvis: Some(frame_pelvis), torso_hands: [Some(h1), Some(h2)], frame_feet: [Some(f1), Some(f2)], seat_shape, hand_defs, torso_anchor, reattaching: false, leg_contacts: [0; 2], listening: true, legs_free_pending: false, frame_gone: false, ejected: false })
    }

    pub fn start_re_attach(&mut self) {
        self.reattaching = true;
    }

    /// `reAttaching` of the girl (original 26119): each forearm whose limb is intact and whose tip is within 0.25 of the rider's torso anchor
    /// gets its hand joint back; once both hold (or are broken) the action ends. Stops when she is ejected or dead.
    pub fn re_attaching(&mut self, world: &mut World, guy_chest: BodyId) {
        if !self.reattaching {
            return;
        }
        if self.ejected || self.doll.dead {
            self.reattaching = false;
            return;
        }
        let mut n = 0;
        for k in 0..2 {
            let (arm, shoulder, elbow) = if k == 0 { (Part::LowerArm1, Limb::Shoulder1, Limb::Elbow1) } else { (Part::LowerArm2, Limb::Shoulder2, Limb::Elbow2) };
            if self.torso_hands[k].is_some() || self.doll.broken.contains(&elbow) || self.doll.broken.contains(&shoulder) {
                n += 1;
                continue;
            }
            let body = self.doll.bodies[&arm];
            let b = world.b2();
            let y = match &b.c.shapes[b.body(body).shapes[0]].shape.kind {
                box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices.get(2).map_or(0.0, |v| v.y),
                _ => 0.0,
            };
            let (t, h) = (b.body(body).world_point(Vec2::new(0.0, y)), b.body(guy_chest).world_point(self.torso_anchor));
            if (t.x - h.x).abs() < 0.25 && (t.y - h.y).abs() < 0.25 {
                self.torso_hands[k] = Some(world.b2_mut().create_revolute_joint(&self.hand_defs[k].clone()));
                n += 1;
            }
        }
        if n >= 2 {
            self.reattaching = false;
        }
    }

    pub fn keys(&mut self, world: &mut World, k: Keys) {
        if self.legs_free_pending {
            self.legs_free(world);
        }
        // `checkKeyStates` of a dead character runs none of the key actions
        if !self.doll.dead {
            if self.ejected {
                match (k.left, k.right) {
                    (true, false) => self.doll.set_pose(1),
                    (false, true) => self.doll.set_pose(2),
                    _ if self.doll.pose == 1 || self.doll.pose == 2 => self.doll.set_pose(0),
                    _ => {}
                }
                match (k.up, k.down) {
                    (true, false) => self.doll.set_pose(3),
                    (false, true) => self.doll.set_pose(4),
                    _ if self.doll.pose == 3 || self.doll.pose == 4 => self.doll.set_pose(0),
                    _ => {}
                }
                self.doll.set_grab(world, k.space);
            }
            if k.shift {
                self.eject(world);
            }
        }
        self.doll.apply_pose(world);
    }

    /// `leg1Stuck`/`leg1Free` (+ 2): sensor contacts of her thighs with the rider's seat shape (the seat must be the contact's second shape).
    pub fn contact_events(&mut self, world: &World, events: &[box2d::world::ContactEvent]) {
        if !self.listening {
            return;
        }
        for (k, part) in [Part::UpperLeg1, Part::UpperLeg2].into_iter().enumerate() {
            let Some(&leg) = world.b2().body(self.doll.bodies[&part]).shapes.first() else { continue };
            for e in events {
                // a listener registered on a shape is called for contacts where it is shape2 too (with shape1/shape2 swapped)
                if !((e.shape1 == leg && e.shape2 == self.seat_shape) || (e.shape2 == leg && e.shape1 == self.seat_shape)) {
                    continue;
                }
                match e.kind {
                    box2d::world::ContactEventKind::Add => self.leg_contacts[k] += 1,
                    box2d::world::ContactEventKind::Remove => {
                        self.leg_contacts[k] -= 1;
                        if self.ejected {
                            self.check_legs_free();
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    fn check_legs_free(&mut self) {
        if self.leg_contacts == [0, 0] {
            self.listening = false;
            self.legs_free_pending = true;
        }
    }

    /// `legsFree` action: thighs, shins and forearms become solid again.
    fn legs_free(&mut self, world: &mut World) {
        self.legs_free_pending = false;
        let b = world.b2_mut();
        for part in [Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2, Part::LowerArm1, Part::LowerArm2] {
            let s = b.body(self.doll.bodies[&part]).shapes[0];
            b.shape_mut(s).is_sensor = false;
            b.refilter(s);
        }
    }

    /// `contactResultHandler` smash reactions: head/chest/pelvis smash their parts (base class), then the passenger ejects.
    pub fn smashed(&mut self, world: &mut World, events: &[box2d::world::ContactEvent]) {
        let hits = world.doll_smash_hits(&self.doll, events);
        for part in [Part::Head, Part::Chest, Part::Pelvis] {
            if !hits.contains(&part) {
                continue;
            }
            smash_part(self, world, part, |p| &mut p.doll, Self::limbs_broken);
            self.eject(world);
        }
    }

    pub fn release_guy(&mut self, world: &mut World) {
        for j in self.torso_hands.iter_mut().filter_map(Option::take) {
            world.b2_mut().destroy_joint(j);
        }
    }

    fn arm_free(&mut self, world: &mut World, k: usize, drop_hand: bool) {
        let part = if k == 0 { Part::LowerArm1 } else { Part::LowerArm2 };
        let b = world.b2_mut();
        let s = b.body(self.doll.bodies[&part]).shapes[0];
        b.shape_mut(s).is_sensor = false;
        b.refilter(s);
        if drop_hand {
            if let Some(j) = self.torso_hands[k].take() {
                b.destroy_joint(j);
            }
        }
    }

    /// Limb-break reactions (original 26358): neck break drops both hands and unsensors the forearms; shoulder/elbow breaks unsensor
    /// that forearm and drop its hand; hip/knee breaks drop that foot; torso break ejects.
    fn limbs_broken(&mut self, world: &mut World, broken: &[Limb]) {
        for l in broken {
            match l {
                Limb::Neck => {
                    self.release_guy(world);
                    self.arm_free(world, 0, false);
                    self.arm_free(world, 1, false);
                }
                Limb::Waist => self.eject(world),
                Limb::Shoulder1 | Limb::Elbow1 => self.arm_free(world, 0, true),
                Limb::Shoulder2 | Limb::Elbow2 => self.arm_free(world, 1, true),
                Limb::Hip1 | Limb::Knee1 => {
                    if let Some(j) = self.frame_feet[0].take() {
                        world.b2_mut().destroy_joint(j);
                    }
                }
                Limb::Hip2 | Limb::Knee2 => {
                    if let Some(j) = self.frame_feet[1].take() {
                        world.b2_mut().destroy_joint(j);
                    }
                }
            }
        }
    }

    pub fn check_joints(&mut self, world: &mut World) {
        for limb in crate::character::CHECK_ORDER {
            if self.doll.check_joint(world, limb) {
                self.limbs_broken(world, &[limb]);
            }
        }
        self.doll.check_composites(world);
    }

    pub fn eject(&mut self, world: &mut World) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        self.doll.reset_joint_limits(world);
        let b = world.b2_mut();
        if let Some(j) = self.frame_pelvis.take() {
            b.destroy_joint(j);
        }
        for j in self.frame_feet.iter_mut().filter_map(Option::take) {
            b.destroy_joint(j);
        }
        self.release_guy(world);
        // with the frame there the legs wait until their contacts with the seat end
        if self.frame_gone {
            self.leg_contacts = [0; 2];
            self.check_legs_free();
        }
    }
}

/// The c10 family: the dad's bike (class of c3 at ref scale 50) towing the daughter's trailer bike.
pub struct Family {
    pub dad: Bike,
    pub daughter: Daughter,
    pub son: Son,
}

impl Family {
    pub const DAD_FRAME_SMASH_LIMIT: f64 = 100.0;

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        self.dad.keys(world, doll, k);
        self.daughter.keys(world, k);
        self.son.keys(world, k);
    }

    /// `frameSmash`: the dad's bike breaks, and the daughter's frame and the son's basket come off it.
    pub fn frame_smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.dad.frame_smashed {
            return;
        }
        self.dad.frame_smash(world, doll);
        self.children_detach(world);
    }

    /// What the dad's broken frame does to the children (the hitch joints themselves went with the frame body).
    pub fn children_detach(&mut self, world: &mut World) {
        self.daughter.detach(world);
        self.son.detach(world);
    }

    pub fn check_joints(&mut self, world: &mut World) {
        self.daughter.check_joints(world);
        self.son.check_joints(world);
    }

    pub fn bodies(&self) -> Vec<BodyId> {
        let d = &self.dad;
        let k = &self.daughter;
        let mut v = vec![d.frame, d.back_wheel, d.front_wheel, d.gear, k.wheel, k.gear];
        v.extend(&d.pieces);
        if !k.frame_gone {
            v.push(k.frame);
        }
        v.extend(&k.debris);
        v.extend(k.doll.bodies.values());
        if !self.son.basket_gone {
            v.push(self.son.basket);
        }
        v.extend(self.son.doll.bodies.values());
        v
    }
}

// ───────────────────────────── c8: Santa's sleigh + two elves ─────────────────────────────

fn strap_ball(world: &mut World, pos: Vec2) -> BodyId {
    let b = world.b2_mut();
    let body = b.create_body(&BodyDef { position: pos, ..Default::default() });
    let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 3.0 / 62.5 });
    d.density = 0.25;
    d.friction = 1.0;
    d.restitution = 0.1;
    d.filter = Filter { category_bits: 0, mask_bits: 0, group_index: 0 };
    b.create_shape(body, &d);
    b.set_mass_from_shapes(body);
    body
}

fn joint_speed(world: &World, id: usize) -> f64 {
    let b = world.b2();
    let j = b.joint(id);
    b.body(j.body2).angular_velocity - b.body(j.body1).angular_velocity
}

fn revolute_mut(world: &mut World, id: usize) -> Option<&mut box2d::joints::Revolute> {
    match &mut world.b2_mut().joint_mut(id).kind {
        box2d::joints::JointKind::Revolute(r) => Some(r),
        _ => None,
    }
}

/// One of Santa's elves (class `Mv`): a ragdoll running on a wheel that hangs from the sleigh on a vertical slider.
/// NOT ported: smash/break reactions of the elf (`headSmash1`…`hipBreak2`), sounds.
pub struct Elf {
    pub doll: Ragdoll,
    pub stem: Option<BodyId>,
    pub wheel: Option<BodyId>,
    /// The ids survive `eject` (which destroys the bodies) so tests/renderers can still address them.
    pub stem_id: BodyId,
    pub wheel_id: BodyId,
    stem_wheel: Option<usize>,
    feet: [Option<usize>; 2],
    pub ejected: bool,
    pub legs_ok: bool,
    pub head_attached: bool,
    pub chest_attached: bool,
    pub strapped_in: bool,
    /// `sign`: which half of the wheel's turn the last step sound was played in.
    step_sign: bool,
    /// `wheelContacts`: ADD/REMOVE contact events of the wheel's shape with non-sensor shapes (`wheelContactAdd/Remove`).
    wheel_contacts: i32,
    wheel_max_speed: f64,
    accel_step: f64,
    anti_grav: Vec<BodyId>,
}

impl Elf {
    /// While a foot rides the unicycle (`wheelFootNJoint`), `paint` eases that leg's clips from their bodies towards the pose on the stem
    /// the faster the wheel stands still: the clip centre (m) and angle of `lowerLeg` (`lower`) or `upperLeg` of leg `side`.
    pub fn leg_paint(&self, world: &World, side: usize, lower: bool) -> Option<([f64; 2], f64)> {
        self.feet[side]?;
        let (stem, wheel) = (self.stem?, self.wheel?);
        let part = match (lower, side) {
            (true, 0) => Part::LowerLeg1,
            (true, _) => Part::LowerLeg2,
            (false, 0) => Part::UpperLeg1,
            (false, _) => Part::UpperLeg2,
        };
        let leg = world.b2().body(*self.doll.bodies.get(&part)?);
        let (stem, wheel) = (world.b2().body(stem), world.b2().body(wheel));
        let rest = 1.0 - (wheel.angular_velocity.abs() / 5.0).clamp(0.0, 1.0);
        let (anchor, offset) = if lower { ([0.115, 0.5702], 0.305) } else { ([0.1553, 0.1994], -0.1952) };
        let (sin, cos) = stem.sweep.a.sin_cos();
        let (x, y) = (anchor[0] - stem.sweep.local_center.x, anchor[1] - stem.sweep.local_center.y);
        let target = [stem.sweep.c.x + cos * x - sin * y, stem.sweep.c.y + sin * x + cos * y];
        let centre = [leg.sweep.c.x - (leg.sweep.c.x - target[0]) * rest, leg.sweep.c.y - (leg.sweep.c.y - target[1]) * rest];
        Some((centre, leg.sweep.a - (leg.sweep.a - (stem.sweep.a + offset)) * rest))
    }

    fn attach(world: &mut World, name: &str, group: i16, start_px: [f32; 2], sleigh: BodyId, vertical_offset: f64, santa_wheel_radius: f64) -> Option<Self> {
        let guide = crate::character::subguide(name)?;
        let mut doll = Ragdoll::spawn_full_deferred(world, &guide, start_px, false, 50.0, group)?;
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |n: &str| -> Option<Vec2> {
            let g = guide.get(n)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        let gs = guide.get("stem")?;
        let stem = b.create_body(&BodyDef { position: at("stem")?, angle: gs[2] / deg, ..Default::default() });
        let mut d = ShapeDef::boxed(gs[3] * 50.0 / CHARACTER_SCALE, gs[4] * 50.0 / CHARACTER_SCALE);
        d.density = 4.0;
        d.friction = 0.3;
        d.restitution = 0.3;
        d.filter = Filter { category_bits: 0, mask_bits: 0, group_index: 0 };
        b.create_shape(stem, &d);
        b.set_mass_from_shapes(stem);
        let gw = guide.get("wheel")?;
        let wheel = b.create_body(&BodyDef { position: at("wheel")?, angle: gw[2] / deg, ..Default::default() });
        // the sub-guide dump has no usable width (6.0); the oracle's wheel mass gives the real clip width, 61.20001220703125 (not 0.6 * 102 = 61.2)
        let width = 61.200_012_207_031_25;
        let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: width / 2.0 / CHARACTER_SCALE });
        d.density = 6.0;
        d.friction = 1.0;
        d.restitution = 0.1;
        d.filter = Filter { category_bits: 62464, mask_bits: 520, group_index: group };
        b.create_shape(wheel, &d);
        b.set_mass_from_shapes(wheel);
        let multiplier = santa_wheel_radius / (width * 0.5);
        let mut anti_grav: Vec<BodyId> = [Part::Head, Part::Chest, Part::Pelvis, Part::UpperArm1, Part::UpperArm2, Part::LowerArm1, Part::LowerArm2, Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2]
            .iter()
            .map(|p| doll.bodies[p])
            .collect();
        anti_grav.push(stem);
        anti_grav.push(wheel);
        doll.finish_joints(world);
        for (limb, lo, hi) in [(Limb::Elbow1, -90.0, 0.0), (Limb::Elbow2, -90.0, 0.0), (Limb::Shoulder1, -170.0, 0.0), (Limb::Shoulder2, -170.0, 0.0), (Limb::Neck, -10.0, 0.0)] {
            doll.set_limits(world, limb, lo / deg, hi / deg);
        }
        let b = world.b2_mut();
        let sp = b.body(stem).xf.position;
        let mut pd = PrismaticJointDef::initialize(b.body(sleigh), sleigh, b.body(stem), stem, sp, Vec2::new(0.0, 1.0));
        pd.enable_limit = true;
        pd.upper_translation = vertical_offset / 62.5;
        pd.lower_translation = -vertical_offset / 62.5;
        b.create_prismatic_joint(&pd);
        let mut rd = RevoluteJointDef::initialize(b.body(stem), stem, b.body(wheel), wheel, b.body(wheel).xf.position);
        rd.max_motor_torque = 100000.0;
        let stem_wheel = b.create_revolute_joint(&rd);
        let mut feet = [None, None];
        for (k, (leg, anchor)) in [(Part::LowerLeg1, "foot1Anchor"), (Part::LowerLeg2, "foot2Anchor")].into_iter().enumerate() {
            let l = doll.bodies[&leg];
            let mut fd = RevoluteJointDef::initialize(b.body(wheel), wheel, b.body(l), l, at(anchor)?);
            fd.max_motor_torque = 0.0;
            feet[k] = Some(b.create_revolute_joint(&fd));
        }
        let chest = doll.bodies[&Part::Chest];
        let cp = b.body(chest).xf.position;
        let mut cd = RevoluteJointDef::initialize(b.body(stem), stem, b.body(chest), chest, cp);
        cd.max_motor_torque = 0.0;
        cd.enable_limit = true;
        cd.lower_angle = 0.0;
        cd.upper_angle = 20.0 / deg;
        b.create_revolute_joint(&cd);
        Some(Self {
            doll,
            stem: Some(stem),
            wheel: Some(wheel),
            stem_id: stem,
            wheel_id: wheel,
            stem_wheel: Some(stem_wheel),
            feet,
            ejected: false,
            legs_ok: true,
            head_attached: true,
            chest_attached: true,
            strapped_in: true,
            step_sign: false,
            wheel_contacts: 0,
            wheel_max_speed: 30.0 * multiplier,
            accel_step: 1.25 * multiplier,
            anti_grav,
        })
    }

    fn keys(&mut self, world: &mut World, k: Keys) {
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !self.doll.dead {
            match (k.left, k.right) {
                (true, false) if ejected => self.doll.set_pose(1),
                (false, true) if ejected => self.doll.set_pose(2),
                (true, true) | (false, false) if ejected && (self.doll.pose == 1 || self.doll.pose == 2) => self.doll.set_pose(0),
                _ => {}
            }
            match (k.up, k.down) {
                (true, false) | (false, true) => {
                    let up = k.up;
                    if ejected {
                        self.doll.set_pose(if up { 3 } else { 4 });
                    } else if self.legs_ok {
                        if let Some(id) = self.stem_wheel {
                            let speed = joint_speed(world, id);
                            let new = if up {
                                if speed < self.wheel_max_speed { speed + self.accel_step } else { speed }
                            } else if speed > -self.wheel_max_speed {
                                speed - self.accel_step
                            } else {
                                speed
                            };
                            if let Some(r) = revolute_mut(world, id) {
                                r.enable_motor = true;
                                r.motor_speed = new;
                            }
                        }
                        self.doll.set_pose(if up { 5 } else { 6 });
                    }
                }
                _ => {
                    if ejected {
                        if self.doll.pose == 3 || self.doll.pose == 4 {
                            self.doll.set_pose(0);
                        }
                    } else if self.legs_ok {
                        if let Some(r) = self.stem_wheel.and_then(|id| revolute_mut(world, id)) {
                            r.enable_motor = false;
                        }
                        self.doll.set_pose(0);
                    }
                }
            }
            if ejected {
                self.doll.set_grab(world, k.space);
            }
        }
        self.apply_pose(world);
    }

    /// `checkPose`: base poses plus the running poses 5/6 (the arms mirror the opposite leg).
    fn apply_pose(&mut self, world: &mut World) {
        self.doll.apply_pose(world);
        let (forward, backward) = (self.doll.pose == 5, self.doll.pose == 6);
        if !(forward || backward) {
            return;
        }
        let deg = 180.0 / std::f64::consts::PI;
        let frac = |world: &World, joint: usize, range: f64| -> f64 {
            let b = world.b2();
            let lower = match &b.joint(joint).kind {
                box2d::joints::JointKind::Revolute(r) => r.lower_angle,
                _ => 0.0,
            };
            (b.revolute_angle(joint).unwrap_or(0.0) - lower) / (range / deg)
        };
        let (e_scale, s_scale) = if forward { (1.0, 3.0) } else { (2.6, 4.18) };
        for (foot, knee, hip, elbow, shoulder) in [(0, Limb::Knee1, Limb::Hip1, Limb::Elbow2, Limb::Shoulder2), (1, Limb::Knee2, Limb::Hip2, Limb::Elbow1, Limb::Shoulder1)] {
            if self.feet[foot].is_none() {
                continue;
            }
            let (Some(&kj), Some(&hj)) = (self.doll.joints.get(&knee), self.doll.joints.get(&hip)) else { continue };
            let kf = frac(world, kj, 150.0);
            let hf = frac(world, hj, 160.0);
            self.doll.set_joint_raw(world, self.doll.joints[&elbow], kf * e_scale, 15.0);
            self.doll.set_joint_raw(world, self.doll.joints[&shoulder], hf * s_scale, 15.0);
        }
    }

    /// `eject`: the joint limits come back and the legs leave the wheel; the strap ends are handled by `Santa`.
    fn eject(&mut self, world: &mut World) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        self.doll.reset_joint_limits(world);
        self.disable_running(world);
    }

    /// `kneeBreak1/2`, `hipBreak1/2`: that foot leaves the wheel (unless ejected); `torsoBreak`: both do.
    fn limbs_broken(&mut self, world: &mut World, broken: &[Limb]) {
        for limb in broken {
            let foot = match limb {
                Limb::Waist => {
                    self.disable_running(world);
                    continue;
                }
                Limb::Knee1 | Limb::Hip1 => 0,
                Limb::Knee2 | Limb::Hip2 => 1,
                _ => continue,
            };
            if !self.ejected {
                if let Some(id) = self.feet[foot].take() {
                    world.b2_mut().destroy_joint(id);
                    self.check_legs_ok(world);
                }
            }
        }
        // the `dead` setter (a broken neck, say) stops the running
        if self.doll.dead && self.feet.iter().any(Option::is_some) {
            self.disable_running(world);
        }
    }

    /// `disableRunning` (also run by the `dead` setter).
    fn disable_running(&mut self, world: &mut World) {
        for f in &mut self.feet {
            if let Some(id) = f.take() {
                world.b2_mut().destroy_joint(id);
            }
        }
        self.check_legs_ok(world);
    }

    /// `checkLegsOk`: with no foot on the wheel the wheel and stem go; the pose is dropped either way.
    fn check_legs_ok(&mut self, world: &mut World) {
        if self.feet.iter().all(Option::is_none) {
            let b = world.b2_mut();
            if let Some(w) = self.wheel.take() {
                b.destroy_body(w);
            }
            if let Some(s) = self.stem.take() {
                b.destroy_body(s);
            }
            self.stem_wheel = None;
            self.legs_ok = false;
        }
        // `currentPose = 0` runs `cancelPose` at once
        self.doll.set_pose(0);
        self.doll.apply_pose(world);
    }
}

/// A passenger's `headSmash1` / `chestSmash` / `pelvisSmash`: the class's break overrides run in the middle of the smash,
/// before the chunks are made, so the doll steps out of its owner and back for every reaction.
fn smash_part<T>(owner: &mut T, world: &mut World, part: Part, doll: fn(&mut T) -> &mut Ragdoll, react: fn(&mut T, &mut World, &[Limb])) {
    let mut smashing = doll(owner).clone();
    let mut on_break = |world: &mut World, current: &mut Ragdoll, limb: Limb| {
        std::mem::swap(doll(owner), current);
        react(owner, world, &[limb]);
        std::mem::swap(doll(owner), current);
    };
    match part {
        Part::Head => smashing.smash_head(world),
        Part::Chest => smashing.smash_chest_with(world, &mut on_break),
        _ => smashing.smash_pelvis_with(world, &mut on_break),
    }
    *doll(owner) = smashing;
}

struct HeadLinks {
    at: Vec2,
    velocity: Vec2,
    idx: usize,
    old: usize,
    j1: BodyId,
    pos1: Vec2,
    length: f64,
    next: Option<(usize, BodyId, Vec2)>,
}

struct ChestLink {
    at: Vec2,
    velocity: Vec2,
    old: usize,
    j1: BodyId,
    a1: Vec2,
    length: f64,
}

/// Santa's sleigh (class at deobfuscated.js:27862; guide `c8`, shapeRefScale 50, group −1).
/// Sleigh = one body (rear/base/back/seat/front polygons, stems in group −2, bumper circle), a sensor "reins" body on a
/// prismatic joint that the hands hold, 8 loose crates and 4 wheels (back, mid1, mid2, front). The elves are tied on
/// by strap chains. NOT ported: sleigh/ski/stem/helmet smash, `newbase`/broken sleigh pieces, snow, sounds,
/// `floatElves`, the elves' smash reactions.
pub struct Santa {
    pub sleigh: BodyId,
    pub reigns: Option<BodyId>,
    pub reigns_id: BodyId,
    /// The first 16 strap balls in creation order (reins→elf1 head 7, →elf2 head 3, chest1 2, chest2 4).
    pub strap_balls: Vec<BodyId>,
    pub wheels: [BodyId; 4],
    pub boxes: Vec<BodyId>,
    pub elf1: Elf,
    pub elf2: Elf,
    wheel_joints: [usize; 4],
    sleigh_reigns: Option<usize>,
    reigns_hands: [Option<usize>; 2],
    sleigh_pelvis: Option<usize>,
    sleigh_feet: [Option<usize>; 2],
    strap_head: Vec<usize>,
    strap_chest1: Vec<usize>,
    strap_chest2: Vec<usize>,
    anti_grav: Vec<BodyId>,
    pub ejected: bool,
    eject_impulse: f64,
    pump: f64,
    pub boost: f64,
    /// SleighBellLoop wanted (`sleighBellLoop`: space held, not ejected, boost below max).
    pub bell: bool,
    /// Sleigh shapes hit by `sleighSmash` (front) and `skiSmash` (the `base` polygon, the original's `skiShape`).
    front_shape: box2d::world::ShapeId,
    ski_shape: Option<box2d::world::ShapeId>,
    stem_shapes: Vec<box2d::world::ShapeId>,
    broken_stem: Vec<Vec<Vec2>>,
    pub stem_gone: bool,
    new_base: Vec<Vec2>,
    broken_sleigh: Vec<Vec<Vec2>>,
    broken_ski: Vec<Vec<Vec2>>,
    pub sleigh_gone: bool,
    /// The sleigh body as it was when destroyed: the skis and the stem breaking afterwards are born from it, and its id
    /// may by then belong to one of the new pieces.
    wreck: Option<box2d::world::Body>,
    pub wheels_gone: bool,
    /// Pieces born from smashes (broken sleigh / ski).
    pub debris: Vec<BodyId>,
    /// Clip name of each `debris` body (`sleigh1..7`, `ski1..3`, `stem`).
    pub debris_names: Vec<String>,
    /// The sleigh body's local centre at creation (the sleigh clip is re-centred when `skiSmash` swaps the base shape and moves it).
    pub sleigh_lc0: Vec2,
    /// Start position in metres (`_startX/_startY`; the sleigh's body-local points are start-relative).
    pub start_m: Vec2,
}

impl Santa {
    const WHEEL_MAX_SPEED: f64 = 30.0;
    const ACCEL: f64 = 1.25;
    const IMPULSE_LEFT: f64 = 1.3;
    const IMPULSE_RIGHT: f64 = 1.7;
    const OFFSET: f64 = 1.0;
    const MAX_SPIN: f64 = 3.5;
    const BOOST_MAX: f64 = 50.0;
    const BOOST_STEP_UP: f64 = 0.5;
    const BOOST_STEP_DOWN: f64 = 0.5;
    const GRAVITY_DISPLACEMENT: f64 = 1.0 / 30.0 * -10.0;

    pub fn attach(world: &mut World, guide: &Guide, start_px: [f32; 2], doll: &mut Ragdoll) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let at = |name: &str| -> Option<Vec2> {
            let g = guide.get(name)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let poly = |prefix: &str, n: usize| -> Option<Vec<Vec2>> { (1..=n).map(|i| at(&format!("{prefix}{i}"))).collect() };
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        let sleigh = b.create_body(&BodyDef::default());
        let (mut ski_shape, mut front_shape) = (None, None);
        let mut stem_shapes = Vec::new();
        let mut d = ShapeDef::boxed(1.0, 1.0);
        d.density = 3.0;
        d.friction = 0.3;
        d.restitution = 0.3;
        d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: -1 };
        for (prefix, n, group) in [("rear_", 5, -1), ("base_", 6, -1), ("back_", 4, -1), ("seat_", 5, -1), ("front_", 5, -1), ("stem1_", 4, -2), ("stem2_", 4, -2), ("stem3_", 4, -2)] {
            d.geometry = Geometry::Polygon(poly(prefix, n)?);
            d.filter.group_index = group;
            let id = b.create_shape(sleigh, &d);
            match prefix {
                "base_" => ski_shape = Some(id),
                "front_" => front_shape = Some(id),
                "stem1_" | "stem2_" | "stem3_" => stem_shapes.push(id),
                _ => {}
            }
        }
        // reins: sensor box at the hand anchor (density 3, group −1)
        let reigns = b.create_body(&BodyDef { position: at("handAnchor")?, ..Default::default() });
        let mut rd = ShapeDef::boxed(0.1, 0.1);
        rd.density = 3.0;
        rd.friction = 0.3;
        rd.restitution = 0.3;
        rd.is_sensor = true;
        rd.filter = Filter { category_bits: 260, mask_bits: 270, group_index: -1 };
        b.create_shape(reigns, &rd);
        b.set_mass_from_shapes(reigns);
        let gb = guide.get("bumper")?;
        let mut bd = ShapeDef::new(Geometry::Circle { local_position: at("bumper")?, radius: gb[5] / 2.0 / CHARACTER_SCALE });
        bd.density = 3.0;
        bd.friction = 1.0;
        bd.restitution = 0.1;
        bd.filter = Filter { category_bits: 260, mask_bits: 270, group_index: -1 };
        b.create_shape(sleigh, &bd);
        b.set_mass_from_shapes(sleigh);
        let mut boxes = Vec::new();
        for i in 1..=8 {
            let n = format!("box{i}");
            let g = guide.get(&n)?;
            let body = b.create_body(&BodyDef { position: at(&n)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::boxed(g[3] * 50.0 / CHARACTER_SCALE, g[4] * 50.0 / CHARACTER_SCALE);
            d.density = 0.25;
            d.friction = 0.3;
            d.restitution = 0.3;
            d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: 0 };
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            boxes.push(body);
        }
        let mut wheel_radius_px = 0.0;
        let mut wheels = Vec::new();
        for (name, grp) in [("backWheel", 0i16), ("midWheel1", 0), ("midWheel2", -1), ("frontWheel", 0)] {
            let g = guide.get(name)?;
            let width = g[5];
            if name == "backWheel" {
                wheel_radius_px = width * 0.5;
            }
            let body = b.create_body(&BodyDef { position: at(name)?, angle: g[2] / deg, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: width / 2.0 / CHARACTER_SCALE });
            d.density = 3.0;
            d.friction = 1.0;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: grp };
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            wheels.push(body);
        }
        let wheels: [BodyId; 4] = wheels.try_into().ok()?;
        doll.finish_joints(world);
        doll.set_limits(world, Limb::Neck, -5.0 / deg, 5.0 / deg);
        let b = world.b2_mut();
        let hand = at("handAnchor")?;
        let mut pd = PrismaticJointDef::initialize(b.body(sleigh), sleigh, b.body(reigns), reigns, hand, Vec2::new(0.0, 1.0));
        pd.enable_limit = true;
        pd.max_motor_force = 1000.0;
        pd.upper_translation = 15.0 / 62.5;
        pd.lower_translation = -100.0 / 62.5;
        let sleigh_reigns = b.create_prismatic_joint(&pd);
        let rev = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p: Vec2, limit: bool| {
            let mut d = RevoluteJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p);
            d.max_motor_torque = 100000.0;
            d.enable_limit = limit;
            b.create_revolute_joint(&d)
        };
        let (a1, a2) = (doll.bodies[&Part::LowerArm1], doll.bodies[&Part::LowerArm2]);
        let h1 = rev(b, reigns, a1, hand, false);
        let h2 = rev(b, reigns, a2, hand, false);
        let pelvis = doll.bodies[&Part::Pelvis];
        let pc = b.body(pelvis).sweep.c;
        let sleigh_pelvis = rev(b, sleigh, pelvis, pc, true);
        let mut wj = Vec::new();
        for (w, name) in wheels.iter().zip(["backWheel", "midWheel1", "midWheel2", "frontWheel"]) {
            wj.push(rev(b, sleigh, *w, at(name)?, false));
        }
        let foot = at("footAnchor")?;
        let f1 = rev(b, sleigh, doll.bodies[&Part::LowerLeg1], foot, false);
        let f2 = rev(b, sleigh, doll.bodies[&Part::LowerLeg2], foot, false);
        let wheel_joints: [usize; 4] = wj.try_into().ok()?;
        let mut anti_grav: Vec<BodyId> = boxes.clone();
        anti_grav.extend([Part::Head, Part::Chest, Part::Pelvis, Part::UpperArm1, Part::UpperArm2, Part::LowerArm1, Part::LowerArm2, Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2].iter().map(|p| doll.bodies[p]));
        anti_grav.push(sleigh);
        anti_grav.push(reigns);
        anti_grav.extend(wheels);
        let elf1 = Elf::attach(world, "c8_elf1", -2, start_px, sleigh, 35.0, wheel_radius_px)?;
        let elf2 = Elf::attach(world, "c8_elf2", -4, start_px, sleigh, 70.0, wheel_radius_px)?;
        let mut s = Self {
            sleigh,
            reigns: Some(reigns),
            reigns_id: reigns,
            strap_balls: Vec::new(),
            wheels,
            boxes,
            elf1,
            elf2,
            wheel_joints,
            sleigh_reigns: Some(sleigh_reigns),
            reigns_hands: [Some(h1), Some(h2)],
            sleigh_pelvis: Some(sleigh_pelvis),
            sleigh_feet: [Some(f1), Some(f2)],
            strap_head: Vec::new(),
            strap_chest1: Vec::new(),
            strap_chest2: Vec::new(),
            anti_grav,
            ejected: false,
            eject_impulse: 0.0,
            pump: 0.0,
            boost: 0.0,
            bell: false,
            front_shape: front_shape?,
            ski_shape,
            stem_shapes,
            broken_stem: vec![poly("stem1_", 4)?, poly("stem2_", 4)?, poly("stem3_", 4)?],
            stem_gone: false,
            new_base: poly("newbase_", 5)?,
            broken_sleigh: (1..=7).map(|i| (1..=6).filter_map(|j| at(&format!("broken_{i}_{j}"))).collect()).collect(),
            broken_ski: (1..=3).map(|i| (1..=6).filter_map(|j| at(&format!("ski_{i}_{j}"))).collect()).collect(),
            sleigh_gone: false,
            wreck: None,
            wheels_gone: false,
            debris: Vec::new(),
            debris_names: Vec::new(),
            sleigh_lc0: world.b2().body(sleigh).sweep.local_center,
            start_m: Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5),
        };
        s.create_straps(world, guide, start)?;
        Some(s)
    }

    /// One strap chain: `count` links from `from` (on `first`) to `to` (on `last`); the inner links are tiny balls.
    /// `bell` offsets the next link's anchor by (0, −1 px) in the ball's frame (the bell chain).
    #[allow(clippy::too_many_arguments)]
    fn chain(world: &mut World, from: Vec2, to: Vec2, count: usize, first: BodyId, last: BodyId, bell: bool, grav: &mut Vec<BodyId>) -> Vec<usize> {
        let (dx, dy) = ((to.x - from.x) / count as f64, (to.y - from.y) / count as f64);
        let mut out = Vec::new();
        let mut anchor1 = from;
        let mut prev = first;
        for k in 0..count {
            let (cur, anchor2) = if k < count - 1 {
                let ball = strap_ball(world, Vec2::new(from.x + dx * (k + 1) as f64, from.y + dy * (k + 1) as f64));
                let b = world.b2();
                let p = if bell { b.body(ball).world_point(Vec2::new(0.0, -1.0 / 62.5)) } else { b.body(ball).xf.position };
                grav.push(ball);
                (ball, p)
            } else {
                (last, to)
            };
            let b = world.b2_mut();
            let d = DistanceJointDef::initialize(b.body(prev), prev, b.body(cur), cur, anchor1, anchor2);
            out.push(b.create_distance_joint(&d));
            prev = cur;
            anchor1 = anchor2;
        }
        out
    }

    /// `createStraps`: reins → elf1 head (8 links) → elf2 head (4); sleigh → elf1 chest (3), → elf2 chest (5).
    fn create_straps(&mut self, world: &mut World, guide: &Guide, start: Vec2) -> Option<()> {
        let at = |n: &str| -> Option<Vec2> {
            let g = guide.get(n)?;
            Some(Vec2::new(start.x + g[0] / CHARACTER_SCALE, start.y + g[1] / CHARACTER_SCALE))
        };
        let reigns = self.reigns?;
        let head_pt = |w: &World, elf: &Elf| w.b2().body(elf.doll.bodies[&Part::Head]).world_point(Vec2::new(-18.0 / CHARACTER_SCALE, 4.0 / CHARACTER_SCALE));
        let chest_pt = |w: &World, elf: &Elf| w.b2().body(elf.doll.bodies[&Part::Chest]).world_point(Vec2::new(0.0, 20.0 / CHARACTER_SCALE));
        let (h1, h2) = (self.elf1.doll.bodies[&Part::Head], self.elf2.doll.bodies[&Part::Head]);
        let (c1, c2) = (self.elf1.doll.bodies[&Part::Chest], self.elf2.doll.bodies[&Part::Chest]);
        let r0 = world.b2().body(reigns).xf.position;
        let e1 = head_pt(world, &self.elf1);
        let mut grav = Vec::new();
        self.strap_head = Self::chain(world, r0, e1, 8, reigns, h1, false, &mut grav);
        let e2 = head_pt(world, &self.elf2);
        let more = Self::chain(world, e1, e2, 4, h1, h2, false, &mut grav);
        self.strap_head.extend(more);
        let s1 = at("strap1Anchor")?;
        let t1 = chest_pt(world, &self.elf1);
        self.strap_chest1 = Self::chain(world, s1, t1, 3, self.sleigh, c1, false, &mut grav);
        let s2 = at("strap2Anchor")?;
        let t2 = chest_pt(world, &self.elf2);
        self.strap_chest2 = Self::chain(world, s2, t2, 5, self.sleigh, c2, true, &mut grav);
        self.strap_balls = grav.clone();
        self.anti_grav.extend(grav);
        Some(())
    }

    fn motors(&self, world: &mut World, on: bool) {
        for id in self.wheel_joints {
            if let Some(r) = revolute_mut(world, id) {
                r.enable_motor = on;
            }
        }
    }

    fn wheel_speed(&self, world: &mut World, speed: f64) {
        for id in self.wheel_joints {
            if let Some(r) = revolute_mut(world, id) {
                r.motor_speed = speed;
            }
        }
    }

    fn set_reigns_motor(&self, world: &mut World, enable: Option<bool>, speed: Option<f64>) {
        if let Some(id) = self.sleigh_reigns {
            if let box2d::joints::JointKind::Prismatic(p) = &mut world.b2_mut().joint_mut(id).kind {
                if let Some(e) = enable {
                    p.enable_motor = e;
                }
                if let Some(s) = speed {
                    p.motor_speed = s;
                }
            }
        }
    }

    pub fn keys(&mut self, world: &mut World, doll: &mut Ragdoll, k: Keys) {
        let ejected = self.ejected;
        // `checkKeyStates` of a dead character runs none of the key actions
        if !doll.dead {
            // left / right
            match (k.left, k.right) {
                (true, false) => {
                    if ejected {
                        doll.set_pose(1);
                    } else {
                        self.lean(world, true);
                    }
                }
                (false, true) => {
                    if ejected {
                        doll.set_pose(2);
                    } else {
                        self.lean(world, false);
                    }
                }
                _ => {
                    if ejected && (doll.pose == 1 || doll.pose == 2) {
                        doll.set_pose(0);
                    }
                }
            }
            // up / down
            match (k.up, k.down) {
                (true, false) | (false, true) => {
                    let up = k.up;
                    if ejected {
                        doll.set_pose(if up { 3 } else { 4 });
                    } else {
                        let first = !revolute_mut(world, self.wheel_joints[0]).is_some_and(|r| r.enable_motor);
                        if first {
                            self.motors(world, true);
                            if self.sleigh_reigns.is_some() {
                                self.set_reigns_motor(world, Some(true), None);
                                self.pump = 0.0;
                            }
                        }
                        let cur = joint_speed(world, self.wheel_joints[if up { 1 } else { 2 }]);
                        let new = if up {
                            if cur < 0.0 { 0.0 } else if cur < Self::WHEEL_MAX_SPEED { cur + Self::ACCEL } else { cur }
                        } else if cur > 0.0 {
                            0.0
                        } else if cur > -Self::WHEEL_MAX_SPEED {
                            cur - Self::ACCEL
                        } else {
                            cur
                        };
                        self.wheel_speed(world, new);
                        if self.sleigh_reigns.is_some() {
                            self.set_reigns_motor(world, None, Some(box2d::fdlibm::sin(self.pump) * 6.0));
                            self.pump += 0.6;
                        }
                    }
                }
                _ => {
                    if ejected {
                        if doll.pose == 3 || doll.pose == 4 {
                            doll.set_pose(0);
                        }
                    } else if revolute_mut(world, self.wheel_joints[0]).is_some_and(|r| r.enable_motor) {
                        self.motors(world, false);
                        self.set_reigns_motor(world, Some(false), None);
                    }
                }
            }
            // space
            self.bell = false;
            if k.space {
                if ejected {
                    doll.set_grab(world, true);
                } else {
                    self.boost = (self.boost + Self::BOOST_STEP_UP).min(Self::BOOST_MAX);
                    self.bell = self.boost < Self::BOOST_MAX;
                    if self.boost < Self::BOOST_MAX {
                        let mut all = self.anti_grav.clone();
                        all.extend(&self.elf1.anti_grav);
                        all.extend(&self.elf2.anti_grav);
                        // upper arms and thighs torn off Santa or an elf float like the rest of them
                        all.extend(doll.sev.iter().chain(&self.elf1.doll.sev).chain(&self.elf2.doll.sev));
                        let b = world.b2_mut();
                        for id in all {
                            let body = b.body_mut(id);
                            if body.alive && !body.is_sleeping() {
                                body.linear_velocity.y += Self::GRAVITY_DISPLACEMENT;
                            }
                        }
                    }
                }
            } else {
                if ejected {
                    doll.set_grab(world, false);
                }
                self.boost = (self.boost - Self::BOOST_STEP_DOWN).max(0.0);
            }
            // shift
            if k.shift {
                if ejected {
                    doll.set_pose(6);
                } else {
                    if !self.elf1.legs_ok {
                        self.eject_elf(world, 1);
                    }
                    if !self.elf2.legs_ok {
                        self.eject_elf(world, 2);
                    }
                }
            } else if doll.pose == 6 {
                doll.set_pose(0);
            }
            // ctrl
            if k.ctrl {
                if ejected {
                    doll.set_pose(7);
                }
            } else if doll.pose == 7 {
                doll.set_pose(0);
            }
            // z
            if k.z {
                if ejected {
                    self.eject_elf(world, 1);
                    self.eject_elf(world, 2);
                } else {
                    self.eject_impulse = 5.0;
                    self.eject(world, doll);
                }
            }
        }
        doll.apply_pose(world);
        // the elves read the same keys
        for n in [1, 2] {
            let elf = if n == 1 { &mut self.elf1 } else { &mut self.elf2 };
            elf.keys(world, k);
            // `checkElfKeys` goes on after Santa dies; the elf's `zPressedActions` then lets go
            if k.z && doll.dead && !elf.doll.dead {
                self.eject_elf(world, n);
            }
        }
    }

    /// `eject` of an elf: from Santa's shift/z, or from the elf's own z once Santa is dead.
    fn eject_elf(&mut self, world: &mut World, n: u8) {
        let elf = if n == 1 { &mut self.elf1 } else { &mut self.elf2 };
        if elf.ejected {
            return;
        }
        elf.eject(world);
        if let Some(l) = self.head_links(world, n) {
            self.hang_head(world, n, l, false);
        }
        if let Some(l) = self.chest_link(world, n) {
            self.hang_chest(world, n, l, false);
        }
    }

    /// `leftPressedActions` / `rightPressedActions` while seated: a pair of opposing impulses at ±1 m from the centre.
    fn lean(&self, world: &mut World, left: bool) {
        let b = world.b2_mut();
        let body = b.body(self.sleigh);
        let (ang, av) = (body.sweep.a, body.angular_velocity);
        let (imp, f) = if left { (Self::IMPULSE_LEFT, ((av + Self::MAX_SPIN) / Self::MAX_SPIN).clamp(0.0, 1.0)) } else { (Self::IMPULSE_RIGHT, ((av - Self::MAX_SPIN) / -Self::MAX_SPIN).clamp(0.0, 1.0)) };
        let (a, w) = (box2d::fdlibm::cos(ang) * imp * f, box2d::fdlibm::sin(ang) * imp * f);
        let lc = body.sweep.local_center;
        let p1 = body.world_point(Vec2::new(lc.x + Self::OFFSET, lc.y));
        let p2 = body.world_point(Vec2::new(lc.x - Self::OFFSET, lc.y));
        let (i1, i2) = if left { (Vec2::new(w, -a), Vec2::new(-w, a)) } else { (Vec2::new(-w, a), Vec2::new(w, -a)) };
        b.body_mut(self.sleigh).apply_impulse(i1, p1);
        b.body_mut(self.sleigh).apply_impulse(i2, p2);
    }

    fn spawn_pieces(&mut self, world: &mut World, src: &box2d::world::Body, verts: &[Vec<Vec2>]) -> Vec<BodyId> {
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let (pos, ang, w) = (src.xf.position, src.sweep.a, src.angular_velocity);
        let b = world.b2_mut();
        let mut out = Vec::new();
        for v in verts {
            let piece = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Polygon(v.clone()));
            d.density = 3.0;
            d.friction = 0.3;
            d.restitution = 0.3;
            d.filter = zero;
            b.create_shape(piece, &d);
            b.set_mass_from_shapes(piece);
            b.body_mut(piece).angular_velocity = w;
            let lc = b.body(piece).sweep.local_center;
            b.body_mut(piece).linear_velocity = src.linear_velocity_from_local_point(lc);
            out.push(piece);
        }
        out
    }

    /// `skiSmash`: the four wheels are destroyed, three ski pieces are born and (while the sleigh is intact) the `base` shape is replaced by `newbase`.
    pub fn ski_smash(&mut self, world: &mut World) {
        if self.wheels_gone {
            return;
        }
        self.wheels_gone = true;
        let src = self.sleigh_state(world);
        for w in self.wheels {
            world.b2_mut().destroy_body(w);
        }
        self.anti_grav.retain(|id| !self.wheels.contains(id));
        let ski = self.broken_ski.clone();
        let pieces = self.spawn_pieces(world, &src, &ski);
        self.debris_names.extend((1..=pieces.len()).map(|i| format!("ski{i}")));
        self.debris.extend(&pieces);
        if !self.sleigh_gone {
            if let Some(old) = self.ski_shape.take() {
                let mut d = ShapeDef::boxed(1.0, 1.0);
                d.density = 3.0;
                d.friction = 0.3;
                d.restitution = 0.3;
                d.filter = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
                d.geometry = Geometry::Polygon(self.new_base.clone());
                let b = world.b2_mut();
                b.destroy_shape(old);
                b.create_shape(self.sleigh, &d);
                b.set_mass_from_shapes(self.sleigh);
            }
            self.anti_grav.extend(pieces);
        }
        world.play("SkiSmash", 1, src.sweep.c);
    }

    fn sleigh_state(&self, world: &World) -> box2d::world::Body {
        self.wreck.clone().unwrap_or_else(|| world.b2().body(self.sleigh).clone())
    }

    /// `stemSmash(impulse, sound)`: one body with the three stem polygons is born at the sleigh's pose and velocity; while the sleigh is intact its three
    /// stem shapes go and the chest straps' first links are re-attached to the new body (same anchors and length).
    pub fn stem_smash(&mut self, world: &mut World, sound: bool) {
        if self.stem_gone {
            return;
        }
        self.stem_gone = true;
        let src = self.sleigh_state(world);
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let piece = b.create_body(&BodyDef { position: src.xf.position, angle: src.sweep.a, ..Default::default() });
        for v in &self.broken_stem {
            let mut d = ShapeDef::new(Geometry::Polygon(v.clone()));
            d.density = 3.0;
            d.friction = 0.3;
            d.restitution = 0.3;
            d.filter = zero;
            b.create_shape(piece, &d);
        }
        b.set_mass_from_shapes(piece);
        b.body_mut(piece).angular_velocity = src.angular_velocity;
        let lc = b.body(piece).sweep.local_center;
        b.body_mut(piece).linear_velocity = src.linear_velocity_from_local_point(lc);
        self.debris.push(piece);
        self.debris_names.push("stem".into());
        if !self.sleigh_gone {
            for s in std::mem::take(&mut self.stem_shapes) {
                world.b2_mut().destroy_shape(s);
            }
            world.b2_mut().set_mass_from_shapes(self.sleigh);
            for n in [1u8, 2] {
                let strapped = if n == 1 { self.elf1.strapped_in } else { self.elf2.strapped_in };
                if !strapped {
                    continue;
                }
                let chain = if n == 1 { &mut self.strap_chest1 } else { &mut self.strap_chest2 };
                let old = chain[0];
                let b = world.b2_mut();
                let j = b.joint(old);
                let body2 = j.body2;
                let box2d::joints::JointKind::Distance(dj) = &j.kind else { continue };
                let d = DistanceJointDef {
                    body1: piece,
                    body2,
                    local_anchor1: dj.local_anchor1,
                    local_anchor2: dj.local_anchor2,
                    length: dj.length,
                    frequency_hz: 0.0,
                    damping_ratio: 0.0,
                    collide_connected: false,
                    user_data: 0,
                };
                let nj = b.create_distance_joint(&d);
                b.destroy_joint(old);
                chain[0] = nj;
            }
            self.anti_grav.push(piece);
        }
        if sound {
            world.play("StemSnap", 1, src.sweep.c);
        }
    }

    /// `sleighSmash`: eject, the sleigh body is replaced by seven broken pieces, skis go, straps snap, the elves are thrown off.
    pub fn sleigh_smash(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.sleigh_gone {
            return;
        }
        self.eject(world, doll);
        let src = world.b2().body(self.sleigh).clone();
        world.b2_mut().destroy_body(self.sleigh);
        self.sleigh_gone = true;
        self.wreck = Some(src.clone());
        let broken = self.broken_sleigh.clone();
        let pieces = self.spawn_pieces(world, &src, &broken);
        self.debris_names.extend((1..=pieces.len()).map(|i| format!("sleigh{i}")));
        self.debris.extend(pieces);
        self.ski_smash(world);
        self.stem_smash(world, false);
        for n in [1u8, 2] {
            let strapped = if n == 1 { self.elf1.strapped_in } else { self.elf2.strapped_in };
            if strapped {
                self.break_strap(world, n);
            }
        }
        self.eject_elf(world, 1);
        self.eject_elf(world, 2);
        world.shard_burst(crate::blood::ShardKind::Sleigh, src.sweep.c, 100.0, 30.0, 20);
        world.play("SleighSmash", 1, src.sweep.c);
    }

    /// `handleContactResults` for the sleigh: Result > 200 on the front shape smashes the sleigh, on the base shape the skis.
    pub fn smashed(&mut self, world: &mut World, doll: &mut Ragdoll, events: &[box2d::world::ContactEvent]) {
        if self.sleigh_gone {
            return;
        }
        let hit = |s: box2d::world::ShapeId| events.iter().any(|e| e.kind == box2d::world::ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > 200.0);
        if hit(self.front_shape) {
            self.sleigh_smash(world, doll);
        } else if self.ski_shape.is_some_and(hit) {
            self.ski_smash(world);
        }
        if let Some(&stem3) = self.stem_shapes.get(2) {
            if hit(stem3) {
                self.stem_smash(world, true);
            }
        }
    }

    /// `eject`: hands/feet/pelvis let go, sleigh shapes in group −1 (and mid wheel 2) go to group 0.
    pub fn eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.ejected {
            return;
        }
        self.ejected = true;
        doll.reset_joint_limits(world);
        for f in &mut self.sleigh_feet {
            if let Some(id) = f.take() {
                world.b2_mut().destroy_joint(id);
            }
        }
        if let Some(id) = self.sleigh_pelvis.take() {
            world.b2_mut().destroy_joint(id);
        }
        for i in 0..2 {
            if let Some(id) = self.reigns_hands[i].take() {
                world.b2_mut().destroy_joint(id);
                self.check_reigns(world, doll);
            }
        }
        let b = world.b2_mut();
        let shapes: Vec<_> = b.body(self.sleigh).shapes.iter().rev().copied().collect();
        for s in shapes {
            if b.shape_mut(s).filter.group_index == -1 {
                let mut f = b.shape_mut(s).filter;
                f.group_index = 0;
                b.set_shape_filter(s, f);
            }
        }
        if !self.wheels_gone {
            let mid2 = b.body(self.wheels[2]).shapes[0];
            let mut f = b.shape_mut(mid2).filter;
            f.group_index = 0;
            b.set_shape_filter(mid2, f);
        }
        let a = b.body(self.sleigh).sweep.a - std::f64::consts::FRAC_PI_2;
        let imp = Vec2::new(box2d::fdlibm::cos(a) * self.eject_impulse, box2d::fdlibm::sin(a) * self.eject_impulse);
        for part in [Part::Chest, Part::Pelvis] {
            let id = doll.bodies[&part];
            let c = b.body(id).sweep.c;
            b.body_mut(id).apply_impulse(imp, c);
        }
        if revolute_mut(world, self.wheel_joints[0]).is_some_and(|r| r.enable_motor) {
            self.motors(world, false);
        }
    }

    fn check_eject(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.reigns_hands.iter().all(Option::is_none) && self.sleigh_feet.iter().all(Option::is_none) {
            self.eject(world, doll);
        }
    }

    /// `checkReigns`: with both hands off the reins, the reins body is replaced by a loose strap ball.
    fn check_reigns(&mut self, world: &mut World, doll: &mut Ragdoll) {
        if self.reigns_hands.iter().any(Option::is_some) {
            return;
        }
        let (Some(sr), Some(reigns)) = (self.sleigh_reigns.take(), self.reigns.take()) else { return };
        let b = world.b2_mut();
        b.destroy_joint(sr);
        let first = self.strap_head[0];
        let Some((anchor, length)) = b.distance_anchor1(first) else { return };
        let body2 = b.joint(first).body2;
        let ball = strap_ball(world, anchor);
        let b = world.b2_mut();
        b.body_mut(ball).linear_velocity = b.body(reigns).linear_velocity;
        self.anti_grav.push(ball);
        let target = b.body(body2).xf.position;
        let mut d = DistanceJointDef::initialize(b.body(ball), ball, b.body(body2), body2, anchor, target);
        d.length = length;
        let nj = b.create_distance_joint(&d);
        b.destroy_joint(first);
        self.strap_head[0] = nj;
        b.destroy_body(reigns);
        self.check_eject(world, doll);
    }

    /// What `elfHeadRemove` reads from the head and the strap, captured first: a smashed head takes its joints with it.
    fn head_links(&self, world: &World, n: u8) -> Option<HeadLinks> {
        let elf = if n == 1 { &self.elf1 } else { &self.elf2 };
        if !elf.head_attached {
            return None;
        }
        let head = elf.doll.bodies[&Part::Head];
        let b = world.b2();
        let end = |j: usize| (b.joint(j).body1, b.distance_anchor1(j).map_or(0.0, |x| x.1));
        let idx = if n == 1 { 7 } else { self.strap_head.len() - 1 };
        let old = self.strap_head[idx];
        let (j1, length) = end(old);
        let next = (n == 1).then(|| {
            let next = self.strap_head[idx + 1];
            let j2 = b.joint(next).body2;
            (next, j2, b.body(j2).xf.position)
        });
        Some(HeadLinks { at: b.body(head).world_point(Vec2::new(-0.288, 0.064)), velocity: b.body(head).linear_velocity, idx, old, j1, pos1: b.body(j1).xf.position, length, next })
    }

    /// `elfHeadRemove`: the strap's head link is re-hung on a ball at the head's back. With `keep` the old links stay
    /// (a smashed head has already taken them).
    fn hang_head(&mut self, world: &mut World, n: u8, l: HeadLinks, keep: bool) {
        let elf = if n == 1 { &mut self.elf1 } else { &mut self.elf2 };
        elf.head_attached = false;
        let ball = strap_ball(world, l.at);
        let b = world.b2_mut();
        b.body_mut(ball).linear_velocity = l.velocity;
        self.anti_grav.push(ball);
        let new_link = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, a1: Vec2, a2: Vec2| {
            let mut d = DistanceJointDef::initialize(b.body(b1), b1, b.body(b2), b2, a1, a2);
            d.length = l.length;
            b.create_distance_joint(&d)
        };
        self.strap_head[l.idx] = new_link(b, l.j1, ball, l.pos1, l.at);
        if let Some((next, j2, pos2)) = l.next {
            self.strap_head[l.idx + 1] = new_link(b, ball, j2, l.at, pos2);
            if !keep {
                b.destroy_joint(next);
            }
        }
        if !keep {
            b.destroy_joint(l.old);
        }
    }

    fn chest_link(&self, world: &World, n: u8) -> Option<ChestLink> {
        let elf = if n == 1 { &self.elf1 } else { &self.elf2 };
        if !elf.chest_attached {
            return None;
        }
        let chest = elf.doll.bodies[&Part::Chest];
        let b = world.b2();
        let chain = if n == 1 { &self.strap_chest1 } else { &self.strap_chest2 };
        let old = chain[chain.len() - 1];
        let j1 = b.joint(old).body1;
        Some(ChestLink { at: b.body(chest).world_point(Vec2::new(0.0, 0.16)), velocity: b.body(chest).linear_velocity, old, j1, a1: b.body(j1).world_point(Vec2::new(0.0, -0.016)), length: b.distance_anchor1(old).map_or(0.0, |x| x.1) })
    }

    /// `elfChestRemove`: the same for the last link of the elf's chest strap.
    fn hang_chest(&mut self, world: &mut World, n: u8, l: ChestLink, gone: bool) {
        let elf = if n == 1 { &mut self.elf1 } else { &mut self.elf2 };
        elf.chest_attached = false;
        let ball = strap_ball(world, l.at);
        let b = world.b2_mut();
        b.body_mut(ball).linear_velocity = l.velocity;
        self.anti_grav.push(ball);
        let chain = if n == 1 { &mut self.strap_chest1 } else { &mut self.strap_chest2 };
        let idx = chain.len() - 1;
        let mut d = DistanceJointDef::initialize(b.body(l.j1), l.j1, b.body(ball), ball, l.a1, l.at);
        d.length = l.length;
        chain[idx] = b.create_distance_joint(&d);
        if !gone {
            b.destroy_joint(l.old);
        }
    }

    /// The elves' `handleContactResults`: their parts smash like anybody's; the debris floats like the rest of the sleigh team, and a
    /// strapped-in elf's strap is re-hung.
    pub fn elf_smashed(&mut self, world: &mut World, events: &[box2d::world::ContactEvent]) {
        for n in [1, 2] {
            let elf = if n == 1 { &self.elf1 } else { &self.elf2 };
            let hits = world.doll_smash_hits(&elf.doll, events);
            for part in [Part::Head, Part::Chest, Part::Pelvis, Part::LowerLeg1, Part::LowerLeg2] {
                if !hits.contains(&part) {
                    continue;
                }
                let head = if part == Part::Head { self.head_links(world, n) } else { None };
                let chest = if part == Part::Chest { self.chest_link(world, n) } else { None };
                let elf = if n == 1 { &mut self.elf1 } else { &mut self.elf2 };
                let (chunks, brain, heart) = (elf.doll.chunks.len(), elf.doll.brain, elf.doll.heart);
                // the `dead` setter at the head of `headSmash1` / `chestSmash` stops the running
                if matches!(part, Part::Head | Part::Chest) && elf.feet.iter().any(Option::is_some) {
                    elf.disable_running(world);
                }
                match part {
                    Part::LowerLeg1 => elf.doll.smash_foot(world, 0),
                    Part::LowerLeg2 => elf.doll.smash_foot(world, 1),
                    _ => smash_part(elf, world, part, |e| &mut e.doll, Elf::limbs_broken),
                }
                let mut made: Vec<BodyId> = elf.doll.chunks[chunks..].to_vec();
                made.extend(elf.doll.brain.filter(|_| brain.is_none()));
                made.extend(elf.doll.heart.filter(|_| heart.is_none()));
                elf.anti_grav.extend(made);
                if !elf.ejected {
                    if let Some(l) = head {
                        self.hang_head(world, n, l, true);
                    }
                    if let Some(l) = chest {
                        self.hang_chest(world, n, l, true);
                    }
                }
            }
        }
    }

    /// `breakStrap`: a chest strap whose first link carries |impulse| > 0.4 is cut at the sleigh end.
    pub fn break_strap(&mut self, world: &mut World, n: u8) {
        let chain = if n == 1 { &mut self.strap_chest1 } else { &mut self.strap_chest2 };
        let first = chain[0];
        let b = world.b2_mut();
        let Some((anchor, length)) = b.distance_anchor1(first) else { return };
        let sleigh_v = b.body(self.sleigh).linear_velocity_from_world_point(anchor);
        let body2 = b.joint(first).body2;
        let ball = strap_ball(world, anchor);
        let b = world.b2_mut();
        b.body_mut(ball).linear_velocity = sleigh_v;
        self.anti_grav.push(ball);
        let local = if n == 1 { Vec2::default() } else { Vec2::new(0.0, -0.016) };
        let target = b.body(body2).world_point(local);
        let mut d = DistanceJointDef::initialize(b.body(ball), ball, b.body(body2), body2, anchor, target);
        d.length = length;
        let nj = b.create_distance_joint(&d);
        b.destroy_joint(first);
        let chain = if n == 1 { &mut self.strap_chest1 } else { &mut self.strap_chest2 };
        chain[0] = nj;
        let elf = if n == 1 { &mut self.elf1 } else { &mut self.elf2 };
        elf.strapped_in = false;
        // `playAreaSoundInstance("StrapSnap1" / "StrapSnap2", ball)`
        let at = world.b2().body(ball).sweep.c;
        world.play(if n == 1 { "StrapSnap1" } else { "StrapSnap2" }, 1, at);
    }

    /// Elf `actions`: while the wheel touches something and spins above 5 rad/s, `Step{odd}` plays when the wheel's cosine falls below −0.9396 (first foot on)
    /// and `Step{even}` when it rises above 0.9396 (second foot on); the number is `ceil(random·5)·2 (−1)`, taken from the frame counter here.
    /// `wheelContactAdd` / `wheelContactRemove` of both elves.
    pub fn elf_contact_events(&mut self, world: &World, events: &[box2d::world::ContactEvent]) {
        for elf in [&mut self.elf1, &mut self.elf2] {
            let Some(&shape) = world.b2().body(elf.wheel_id).shapes.first() else { continue };
            for e in events {
                let other = if e.shape1 == shape {
                    e.shape2
                } else if e.shape2 == shape {
                    e.shape1
                } else {
                    continue;
                };
                if world.b2().c.shapes[other].shape.is_sensor {
                    continue;
                }
                match e.kind {
                    box2d::world::ContactEventKind::Add => elf.wheel_contacts += 1,
                    box2d::world::ContactEventKind::Remove => elf.wheel_contacts -= 1,
                    _ => {}
                }
            }
        }
    }

    pub fn elf_steps(&mut self, world: &mut World) {
        let frame = world.frame();
        for elf in [&mut self.elf1, &mut self.elf2] {
            let wheel = elf.wheel_id;
            if elf.wheel_contacts <= 0 || world.b2().body(wheel).angular_velocity.abs() <= 5.0 {
                continue;
            }
            let cos = box2d::fdlibm::cos(world.b2().body(wheel).sweep.a);
            let at = world.b2().body(wheel).sweep.c;
            let pick = (frame % 5) as usize + 1;
            if elf.step_sign {
                if cos < -0.9396 {
                    elf.step_sign = false;
                    if elf.feet[0].is_some() {
                        world.play(&format!("Step{}", pick * 2 - 1), 1, at);
                    }
                }
            } else if cos > 0.9396 {
                elf.step_sign = true;
                if elf.feet[1].is_some() {
                    world.play(&format!("Step{}", pick * 2), 1, at);
                }
            }
        }
    }

    /// Santa's limb-break reactions: they let go of the reins and the sleigh.
    pub fn limbs_broken(&mut self, world: &mut World, doll: &mut Ragdoll, broken: &[Limb]) {
        for &limb in broken {
            match limb {
                Limb::Neck | Limb::Waist => self.eject(world, doll),
                Limb::Elbow1 | Limb::Shoulder1 if !self.ejected => {
                    if let Some(id) = self.reigns_hands[0].take() {
                        world.b2_mut().destroy_joint(id);
                        self.check_reigns(world, doll);
                    }
                }
                Limb::Elbow2 | Limb::Shoulder2 if !self.ejected => {
                    if let Some(id) = self.reigns_hands[1].take() {
                        world.b2_mut().destroy_joint(id);
                        self.check_reigns(world, doll);
                    }
                }
                Limb::Knee1 | Limb::Hip1 if !self.ejected => {
                    if let Some(id) = self.sleigh_feet[0].take() {
                        world.b2_mut().destroy_joint(id);
                        self.check_eject(world, doll);
                    }
                }
                Limb::Knee2 | Limb::Hip2 if !self.ejected => {
                    if let Some(id) = self.sleigh_feet[1].take() {
                        world.b2_mut().destroy_joint(id);
                        self.check_eject(world, doll);
                    }
                }
                _ => {}
            }
        }
    }

    /// `checkJoints`: Santa's limb breaks let go of the sleigh; straps snap above 0.4; the elves' limbs break too.
    pub fn check_joints(&mut self, world: &mut World, doll: &mut Ragdoll) {
        doll.check_joints_with(world, |world, doll, limb| self.limbs_broken(world, doll, &[limb]));
        for n in [1u8, 2] {
            let (strapped, chain0) = if n == 1 { (self.elf1.strapped_in, self.strap_chest1[0]) } else { (self.elf2.strapped_in, self.strap_chest2[0]) };
            if strapped && world.b2().distance_impulse(chain0).is_some_and(|i| i.abs() > 0.4) {
                self.break_strap(world, n);
            }
        }
        for elf in [&mut self.elf1, &mut self.elf2] {
            // the `dead` setter runs at the head of `neckBreak` / `torsoBreak`, before they touch the body
            if elf.doll.about_to_die(world) && elf.feet.iter().any(Option::is_some) {
                elf.disable_running(world);
            }
            for limb in crate::character::CHECK_ORDER {
                if elf.doll.check_joint(world, limb) {
                    elf.limbs_broken(world, &[limb]);
                }
            }
            elf.doll.check_composites(world);
            // the `dead` setter runs `disableRunning` (also for a death by smash earlier in this step)
            if elf.doll.dead && elf.feet.iter().any(Option::is_some) {
                elf.disable_running(world);
            }
        }
    }

    /// |m_impulse| of the first link of both chest straps (what `checkJoints` compares with 0.4); 0 once a strap is gone.
    pub fn strap_impulses(&self, world: &World) -> [f64; 2] {
        [self.strap_chest1[0], self.strap_chest2[0]].map(|j| world.b2().distance_impulse(j).map_or(0.0, f64::abs))
    }

    pub fn bodies(&self, doll_bodies: &[BodyId]) -> Vec<BodyId> {
        let _ = doll_bodies;
        let mut v = Vec::new();
        if !self.sleigh_gone {
            v.push(self.sleigh);
        }
        v.extend(self.reigns);
        if !self.wheels_gone {
            v.extend(self.wheels);
        }
        v.extend(&self.debris);
        v.extend(&self.boxes);
        for e in [&self.elf1, &self.elf2] {
            v.extend(e.doll.bodies.values());
            v.extend(e.stem);
            v.extend(e.wheel);
        }
        v
    }
}

#[cfg(test)]
mod pogo_limit_tests {
    use crate::character::Limb;

    #[test]
    fn pogo_rider_breaks_at_the_originals_limits() {
        let level = crate::level::parse("<levelXML><info v='1.99' x='600' y='300' c='9'/></levelXML>").unwrap();
        let mut w = crate::physics::World::from_level(&level);
        w.spawn_bike(9, level.info.start);
        let mut doll = w.character.take().expect("rider");
        let limits = |d: &crate::character::Ragdoll, w: &crate::physics::World| [Limb::Elbow1, Limb::Knee1].map(|l| d.break_limit(w, l));
        // read from the running original: 90 / 120 on the stick, 52 / 76 off it
        assert_eq!(limits(&doll, &w), [90.0, 120.0]);
        doll.limit_override = None;
        assert_eq!(limits(&doll, &w), [52.0, 76.0]);
    }
}
