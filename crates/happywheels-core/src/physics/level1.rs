//! The level built into the original (id 1, class `Level1`): its `shapeGuide` clip (dumped through the oracle, `analysis/level1-dump.js`) turned into
//! bodies in the original creation order — static geometry, sleeping blocks, the finish block, the plank bridge, two motor gears, three pistons and a mine.

use super::{World, PS};
use box2d::collision::Aabb;
use box2d::joints::{JointKind, PrismaticJointDef, RevoluteJointDef};
use box2d::math::Vec2;
use box2d::shape::{Filter, Geometry, ShapeDef};
use box2d::world::{BodyDef, BodyId, ContactEvent, ContactEventKind, ShapeId};

/// `startPoint` of `Level1`.
pub const START: [f32; 2] = [400.0, 100.0];

struct Guide {
    name: String,
    x: f64,
    y: f64,
    rotation: f64,
    sx: f64,
    sy: f64,
    width: f64,
}

fn guide() -> Vec<Guide> {
    let doc: serde_json::Value = serde_json::from_str(include_str!("../../data/level1-guide.json")).expect("level1 guide");
    let f = |v: &serde_json::Value| v.as_f64().unwrap_or(0.0);
    doc["guide"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|g| Guide { name: g[0].as_str().unwrap_or("").into(), x: f(&g[1]), y: f(&g[2]), rotation: f(&g[3]), sx: f(&g[4]), sy: f(&g[5]), width: f(&g[6]) })
        .collect()
}

impl Guide {
    fn centre(&self) -> Vec2 {
        Vec2::new(self.x / PS, self.y / PS)
    }
    fn angle(&self) -> f64 {
        self.rotation * std::f64::consts::PI / 180.0
    }
    fn half(&self) -> (f64, f64) {
        (self.sx * 5.0 / PS, self.sy * 5.0 / PS)
    }
    fn boxed(&self, density: f64, group: i16) -> ShapeDef {
        let (hx, hy) = self.half();
        solid(ShapeDef::oriented_box(hx, hy, self.centre(), self.angle()), density, group)
    }
}

fn solid(mut d: ShapeDef, density: f64, group: i16) -> ShapeDef {
    d.density = density;
    d.friction = 1.0;
    d.restitution = 0.1;
    d.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: group };
    d
}

/// Where a clip of the level's `backGround` that follows a body is now.
pub struct Clip {
    pub name: String,
    /// World px and radians.
    pub position: [f64; 2],
    pub angle: f64,
    pub visible: bool,
    /// Rotation of the clip's `inner` node (gears).
    pub inner: Option<f64>,
}

struct Piston {
    joint: usize,
    body: BodyId,
    speed: f64,
    wait: i32,
    counter: i32,
    waiting: bool,
    raising: bool,
}

struct Mine {
    body: BodyId,
    sensor: ShapeId,
    triggered: bool,
    gone: bool,
}

pub(super) struct Level1 {
    pistons: Vec<Piston>,
    mine: Mine,
    end_shape: ShapeId,
    /// Finish zone (m): lower and upper corner.
    zone: [Vec2; 2],
    end_done: bool,
    /// Clip name (`<guide name>mc`), its body and the gears' `inner` rotation.
    clips: Vec<(String, BodyId, Option<f64>)>,
}

impl World {
    /// The built-in level. The rider starts at [`START`].
    pub fn level1() -> Self {
        let g = guide();
        let named = |n: &str| g.iter().find(|s| s.name == n).expect("level1 guide element");
        // world bounds from the `upper`/`lower` markers, camera limits from `camUpper`/`camLower`
        let (lower, upper) = (named("lower"), named("upper"));
        let (cam_lower, cam_upper) = (named("camLower"), named("camUpper"));
        let mut w = World::empty(lower.centre(), upper.centre());
        w.camera_bounds = Some([cam_lower.x, cam_lower.y, cam_upper.x - cam_lower.x, cam_upper.y - cam_lower.y]);
        let level_body = w.level_body;

        // createStaticShapes, in guide order
        for s in &g {
            match s.name.as_str() {
                "rp" => {
                    w.b.create_shape(level_body, &s.boxed(0.0, -10));
                }
                "cp" => {
                    let circle = ShapeDef::new(Geometry::Circle { local_position: s.centre(), radius: s.sx * 5.0 / PS });
                    w.b.create_shape(level_body, &solid(circle, 0.0, -10));
                }
                "tp" => {
                    let (v, d) = (s.sx * 5.0 / PS, s.sy * 10.0 / PS);
                    let (c, n, at) = (box2d::fdlibm::cos(s.angle()), box2d::fdlibm::sin(s.angle()), s.centre());
                    let points = [Vec2::new(0.0, d * -2.0), Vec2::new(v, d), Vec2::new(-v, d)]
                        .map(|p| Vec2::new(at.x + (c * p.x - n * p.y), at.y + (c * p.y + n * p.x)));
                    w.b.create_shape(level_body, &solid(ShapeDef::new(Geometry::Polygon(points.to_vec())), 0.0, -10));
                }
                _ => {}
            }
        }
        let mut clips: Vec<(String, BodyId, Option<f64>)> = Vec::new();
        // createDynamicShapes: `<name>_<density>_<sleep>`
        for s in &g {
            let parts: Vec<&str> = s.name.split('_').collect();
            if parts.len() < 2 {
                continue;
            }
            let id = w.b.create_body(&BodyDef { is_sleeping: parts.get(2) == Some(&"sleep"), ..BodyDef::default() });
            w.b.create_shape(id, &s.boxed(parts[1].parse().unwrap_or(0.0), 0));
            w.b.set_mass_from_shapes(id);
            clips.push((format!("{}mc", parts[0]), id, None));
        }
        // createItems: the finish block first
        let end = named("end");
        let end_body = w.b.create_body(&BodyDef::default());
        let end_shape = w.b.create_shape(end_body, &end.boxed(0.0, 0));
        let (hx, _) = end.half();
        let at = end.centre();
        // (the original uses the half width for the height of the zone too)
        let zone = [Vec2::new(at.x - hx, at.y - hx * 2.0), Vec2::new(at.x + hx, at.y)];

        // bridge: six planks between two pins on the level
        let planks: Vec<&Guide> = (0..6).map(|i| named(&format!("ba{i}"))).collect();
        let bodies: Vec<BodyId> = planks
            .iter()
            .map(|s| {
                let id = w.b.create_body(&BodyDef::default());
                w.b.create_shape(id, &s.boxed(1.0, -10));
                w.b.set_mass_from_shapes(id);
                id
            })
            .collect();
        clips.extend(bodies.iter().enumerate().map(|(i, &id)| (format!("bamc{i}"), id, None)));
        let pin = |w: &mut World, a: BodyId, b: BodyId, x: f64, y: f64| {
            let d = RevoluteJointDef::initialize(w.b.body(a), a, w.b.body(b), b, Vec2::new(x / PS, y / PS));
            w.b.create_revolute_joint(&d);
        };
        pin(&mut w, level_body, bodies[0], planks[0].x - planks[0].width / 2.0, planks[0].y);
        for i in 0..5 {
            pin(&mut w, bodies[i], bodies[i + 1], (planks[i].x + planks[i + 1].x) / 2.0, planks[i].y);
        }
        pin(&mut w, level_body, bodies[5], planks[5].x + planks[5].width / 2.0, planks[5].y);

        // gears: a hub circle and spokes, pinned and driven
        for (name, motor, spokes, hub, thickness) in [("geara", 0.1, 5, 0.9, 0.12), ("gearb", 0.1, 3, 0.75, 0.1)] {
            let s = named(name);
            let id = w.b.create_body(&BodyDef::default());
            let circle = ShapeDef::new(Geometry::Circle { local_position: s.centre(), radius: s.sx * 5.0 * hub / PS });
            w.b.create_shape(id, &solid(circle, 1.0, -10));
            let step = 180.0 / f64::from(spokes);
            let (hx, hy) = (s.sx * 5.0 / PS, s.sx * 5.0 * thickness / PS);
            for k in 0..spokes {
                let angle = s.angle() + f64::from(k) * step * std::f64::consts::PI / 180.0;
                w.b.create_shape(id, &solid(ShapeDef::oriented_box(hx, hy, s.centre(), angle), 1.0, -10));
            }
            w.b.set_mass_from_shapes(id);
            let mut d = RevoluteJointDef::initialize(w.b.body(level_body), level_body, w.b.body(id), id, s.centre());
            d.enable_motor = true;
            d.max_motor_torque = 10_000_000.0;
            d.motor_speed = motor;
            w.b.create_revolute_joint(&d);
            clips.push((format!("{name}mc"), id, Some(s.angle())));
        }

        // pistons (speed 3 m/s, travel 3.4 m); the second starts 90 frames into a 60 frame wait
        let mut pistons = Vec::new();
        for (name, offset) in [("pb1", 0), ("pb2", 90), ("pb3", 0)] {
            let (speed, distance, wait) = (3.0, 3.4, 60);
            let s = named(name);
            let id = w.b.create_body(&BodyDef { allow_sleep: false, ..BodyDef::default() });
            w.b.create_shape(id, &s.boxed(50.0, -10));
            w.b.set_mass_from_shapes(id);
            let centre = w.b.body(id).sweep.c;
            let mut d = PrismaticJointDef::initialize(w.b.body(level_body), level_body, w.b.body(id), id, centre, Vec2::new(0.0, 1.0));
            d.lower_translation = -distance;
            d.upper_translation = 0.0;
            d.enable_limit = true;
            d.max_motor_force = 100_000.0;
            let joint = w.b.create_prismatic_joint(&d);
            let (waiting, counter) = if offset > 0 { (true, wait - offset) } else { (false, 0) };
            clips.push((format!("{name}mc"), id, None));
            pistons.push(Piston { joint, body: id, speed, wait, counter, waiting, raising: true });
        }

        // the mine
        let s = named("mine1");
        let body = w.b.create_body(&BodyDef { position: s.centre(), angle: s.angle(), ..BodyDef::default() });
        let (hx, hy) = s.half();
        let boxed = ShapeDef::oriented_box(hx, hy, Vec2::default(), 0.0);
        let sensor = w.b.create_shape(body, &solid(boxed, 1.0, 0));

        w.b.c.report_results = true;
        w.level1 = Some(Box::new(Level1 { pistons, mine: Mine { body, sensor, triggered: false, gone: false }, end_shape, zone, end_done: false, clips }));
        w
    }

    /// The moving bodies of the built-in level.
    pub fn level1_bodies(&self) -> Vec<BodyId> {
        self.level1.iter().flat_map(|l| l.clips.iter().map(|c| c.1)).collect()
    }

    /// The clips of the built-in level that follow bodies (`paint`), and the mine, which disappears when it goes off.
    pub fn level1_clips(&self) -> Vec<Clip> {
        let Some(level) = &self.level1 else { return Vec::new() };
        let mut out: Vec<Clip> = level
            .clips
            .iter()
            .map(|(name, id, inner)| {
                let body = self.b.body(*id);
                let c = body.sweep.c;
                Clip { name: name.clone(), position: [c.x * PS, c.y * PS], angle: body.sweep.a, visible: true, inner: *inner }
            })
            .collect();
        if level.mine.gone {
            out.push(Clip { name: "mine1mc".into(), position: [0.0; 2], angle: 0.0, visible: false, inner: None });
        }
        out
    }

    /// `Level1.actions` and the level's contact listeners, after the step: the finish zone, the pistons, the mine.
    pub(super) fn run_level1(&mut self, events: &[ContactEvent]) {
        let Some(mut level) = self.level1.take() else { return };
        let hit = |shape: ShapeId| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape));
        if !level.end_done && hit(level.end_shape) {
            if let Some(character) = self.character.as_ref() {
                if character.dead {
                    level.end_done = true;
                } else if let Some(focus) = character.heart.or_else(|| character.bodies.get(&crate::character::Part::Chest).copied()) {
                    let at = self.b.body(focus).sweep.c;
                    let [lo, hi] = level.zone;
                    if at.x < hi.x && at.x > lo.x && at.y < hi.y && at.y > lo.y {
                        level.end_done = true;
                        self.level_complete = true;
                        self.play("Victory", 1, at);
                    }
                }
            }
        }
        level.mine.triggered |= !level.mine.gone && hit(level.mine.sensor);

        for p in &mut level.pistons {
            let translation = self.b.prismatic_translation(p.joint).unwrap_or(0.0);
            if p.waiting {
                if p.counter != p.wait {
                    p.counter += 1;
                    continue;
                }
                self.set_piston_motor(p.joint, false);
                p.counter = 0;
                p.waiting = false;
            }
            let (lower, upper) = self.piston_limits(p.joint);
            if p.raising {
                if translation < lower {
                    p.raising = false;
                    p.waiting = true;
                    self.set_piston_motor(p.joint, true);
                } else {
                    self.b.body_mut(p.body).linear_velocity = Vec2::new(0.0, -p.speed);
                }
            } else if translation > upper {
                p.raising = true;
                p.waiting = true;
                self.set_piston_motor(p.joint, true);
            } else {
                self.b.body_mut(p.body).linear_velocity = Vec2::new(0.0, p.speed);
            }
        }

        if level.mine.triggered && !level.mine.gone {
            level.mine.gone = true;
            let centre = self.b.body(level.mine.body).sweep.c;
            self.b.destroy_body(level.mine.body);
            let area = Aabb { lower: Vec2::new(centre.x - 2.0, centre.y - 2.0), upper: Vec2::new(centre.x + 2.0, centre.y + 2.0) };
            // one impulse per shape found, also for a body with several of them
            for shape in self.b.query(&area, 30) {
                let id = self.b.shape_body(shape);
                let body = self.b.body(id);
                if body.is_static {
                    continue;
                }
                let at = body.sweep.c;
                let (dx, dy) = (at.x - centre.x, at.y - centre.y);
                let strength = 1.0 - (dx * dx + dy * dy).sqrt().min(2.0) / 2.0;
                let angle = dy.atan2(dx);
                let impulse = Vec2::new(box2d::fdlibm::cos(angle) * strength * 8.0, box2d::fdlibm::sin(angle) * strength * 8.0);
                self.b.body_mut(id).apply_impulse(impulse, at);
            }
            self.play("MineExplosion", 1, centre);
        }
        self.level1 = Some(level);
    }

    /// `EnableMotor` with the speed `SetMotorSpeed(0)` set before it: a waiting piston is held by its motor.
    fn set_piston_motor(&mut self, joint: usize, on: bool) {
        if let JointKind::Prismatic(p) = &mut self.b.joint_mut(joint).kind {
            p.enable_motor = on;
            if on {
                p.motor_speed = 0.0;
            }
        }
    }

    fn piston_limits(&mut self, joint: usize) -> (f64, f64) {
        match &self.b.joint_mut(joint).kind {
            JointKind::Prismatic(p) => (p.lower_translation, p.upper_translation),
            _ => (0.0, 0.0),
        }
    }
}
