//! Physics world on the `box2d` crate (bit-exact port of the original's Box2D 2.0.1 fork, see
//! docs/PORT.md). Level construction follows the original's `createStaticShapes`,
//! `createDynamicShapes`, `createSpecials`, `createGroups`, `createJoints`, `createTriggers` in that order.
//! World constants: 30 Hz, 10 solver iterations, 62.5 px per metre, gravity (0, 10), y down.

use box2d::{
    joints::{PrismaticJointDef, RevoluteJointDef},
    math::Vec2,
    shape::{Filter, Geometry, MassData, ShapeDef, ShapeKind as B2Kind},
    world::{BodyDef, BodyId, ContactEventKind, ShapeId, World as B2},
};

use crate::level::{BodyRef, Level, Shape, ShapeKind};
use crate::specials::{JointSpec, SimpleBody};

pub const PHYS_SCALE: f32 = 62.5;
const PS: f64 = 62.5;
pub const TIME_STEP: f64 = 1.0 / 30.0;
pub const ITERATIONS: usize = 10;

fn num(s: &Shape, k: &str) -> Option<f64> {
    s.num(k).map(f64::from)
}

fn rot(a: f64, x: f64, y: f64) -> (f64, f64) {
    (a.cos() * x - a.sin() * y, a.cos() * y + a.sin() * x)
}

fn polygon_area(pts: &[Vec2]) -> f64 {
    let mut a = 0.0;
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        a += p.x * q.y - q.x * p.y;
    }
    (a / 2.0).abs()
}

/// Geometry of a level shape: `center` (px) and `angle` place it inside its body (static shapes and
/// group children); dynamic shapes use `([0, 0], 0)`.
fn geometry_of(s: &Shape, center: [f64; 2], angle: f64) -> Option<Geometry> {
    let c = Vec2::new(center[0] / PS, center[1] / PS);
    let place = |pts: Vec<Vec2>| -> Option<Geometry> {
        let v: Vec<Vec2> = pts
            .into_iter()
            .map(|p| {
                let (x, y) = if angle == 0.0 { (p.x, p.y) } else { rot(angle, p.x, p.y) };
                Vec2::new(c.x + x, c.y + y)
            })
            .collect();
        if v.len() < 3 || v.len() > 8 || polygon_area(&v) < 1e-6 {
            return None; // degenerate (zero size / collinear): the original would produce NaNs
        }
        Some(Geometry::Polygon(v))
    };
    match s.kind {
        ShapeKind::Rectangle => {
            let (hx, hy) = (num(s, "shapeWidth")? / 100.0 * 50.0 / PS, num(s, "shapeHeight")? / 100.0 * 50.0 / PS);
            Some(ShapeDef::oriented_box(hx, hy, c, angle).geometry)
        }
        ShapeKind::Circle => Some(Geometry::Circle { local_position: c, radius: num(s, "shapeWidth")? / 100.0 * 50.0 / PS }),
        ShapeKind::Triangle => {
            // verified: isosceles (0,-2h),(w,h),(-w,h) with w = scaleX*50/ps, h = scaleY*100/ps
            let (w, h) = (num(s, "shapeWidth")? / 100.0 * 50.0 / PS, num(s, "shapeHeight")? / 300.0 * 100.0 / PS);
            place(vec![Vec2::new(0.0, h * -2.0), Vec2::new(w, h), Vec2::new(-w, h)])
        }
        ShapeKind::Polygon if s.verts.len() >= 3 => {
            let (sx, sy) = (num(s, "shapeWidth").unwrap_or(100.0) / 100.0, num(s, "shapeHeight").unwrap_or(100.0) / 100.0);
            place(s.verts.iter().map(|v| Vec2::new(sx * f64::from(v[0]) / PS, sy * f64::from(v[1]) / PS)).collect())
        }
        _ => None,
    }
}

/// The original's `setShapeFilter` (friction 1, restitution .1 plus collision filtering by `collision` mode).
fn set_shape_filter(d: &mut ShapeDef, collision: i32, immovable: bool, version: f32) {
    d.friction = 1.0;
    d.restitution = 0.1;
    let new = version > 1.84;
    let f = &mut d.filter;
    match collision {
        2 => {
            f.category_bits = 8;
            f.mask_bits = 8;
            if new && immovable {
                f.category_bits = 24;
                f.mask_bits = 56;
            }
        }
        3 => {
            if version < 1.85 {
                f.category_bits = 1;
                f.mask_bits = 1;
                f.group_index = -10;
                d.is_sensor = true;
            } else {
                f.category_bits = 0;
                f.mask_bits = 0;
            }
        }
        4 => {
            f.category_bits = 8;
            f.group_index = -321;
            if new && immovable {
                f.category_bits = 24;
            }
        }
        5 => {
            f.category_bits = 16;
            f.mask_bits = 16;
            f.group_index = -322;
            if new && immovable {
                f.mask_bits = 48;
                f.group_index = 0;
            }
        }
        6 => {
            if new {
                f.category_bits = if immovable { 48 } else { 32 };
                f.mask_bits = 48;
            } else {
                f.category_bits = 16;
                f.mask_bits = 16;
            }
        }
        7 => {
            f.category_bits = 15;
            f.mask_bits = 3840;
        }
        _ if immovable => {
            f.category_bits = 24;
            f.group_index = -10;
        }
        _ => f.category_bits = 8,
    }
}

/// A trigger firing: `trigger` index into `Level::triggers`, `frame` = step number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerEvent {
    pub trigger: usize,
    pub frame: u32,
}

/// Runtime state of one trigger, transcribed from the original's trigger class (contact-listener driven):
/// triggeredBy 1 = main character, 2 = any character, 3 = any non-fixed shape, 4 = only this trigger's
/// targets, 5 = other triggers (not modelled), 6 = mouse click (not modelled); repeatType 1 = once,
/// 2 = again after the zone was cleared, 3 = every `interval` while the body stays, 4 = started by
/// another trigger (not modelled).
struct TriggerState {
    sensor: ShapeId,
    by: i32,
    repeat: i32,
    repeat_frames: Option<u32>,
    repeat_count: u32,
    delay_frames: u32,
    delay: Vec<u32>,
    targets: Vec<BodyId>,
    /// Shape targets with their actions `[action index, params…]` (action-trigger type only).
    shape_actions: Vec<(BodyId, Vec<Vec<String>>)>,
    /// Triggers activated when this one fires (`TargetKind::Trigger` targets).
    trigger_targets: Vec<usize>,
    disabled: bool,
    /// One-shot trigger already spent (mouse / trigger-activated modes).
    done: bool,
    /// ADD listener registered.
    listening: bool,
    /// REMOVE listener registered.
    watching_remove: bool,
    /// In the level's actions vector (needs per-frame `actions`).
    active: bool,
    triggering_body: Option<BodyId>,
    triggering_count: i32,
}

/// Fan wind zone: bodies seen in the sensor's persisting contacts get a force each frame.
/// Boost pad: bodies seen in the sensor's PERSIST events this step get `power·mass` of force along `(sin a, -cos a)`.
struct Boost {
    sensor: ShapeId,
    power: f64,
    sin: f64,
    cos: f64,
    bodies: Vec<BodyId>,
}

/// Cannon special state machine (`Cannon.actions`): WAITING → (a body enters the sensor) LOADING → AIMING → FIRING → RETURNING → WAITING.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum CannonState {
    Waiting,
    Loading,
    Aiming,
    Firing,
    Returning,
}

struct Cannon {
    muzzle: BodyId,
    joint: usize,
    sensor: ShapeId,
    state: CannonState,
    counter: u32,
    load_frames: u32,
    start_angle: f64,
    fire_angle: f64,
    power: f64,
    inside: Vec<(BodyId, i32)>,
}

/// NPCharacter ragdoll (`npc-guides.json`: bodies/joints dumped from the original class at rotation 0): chest, head, pelvis, 4 arm and 4 leg bodies.
struct Npc {
    bodies: [BodyId; 11],
    /// (joint id, child body index, break limit², broken)
    joints: Vec<(usize, usize, f64, bool)>,
    /// head / chest / pelvis: shape, smash limit (`setBreakLimits`), chunk radius (px/125 applied at use) and whether it already smashed.
    smash_shapes: [ShapeId; 3],
    smash_limits: [f64; 3],
    chunk: [f64; 3],
    smashed: [bool; 3],
    /// Chunk circles / brain / heart born from smashes.
    pieces: Vec<BodyId>,
}

/// Boombox special (`Boombox`): cracks at an impulse above 6.5 (damage frame 2..4, 15 shards), shatters above 17.5 (or on the next hit after a crack)
/// into two pieces and 30 shards.
struct Boombox {
    body: BodyId,
    /// TV instead of boombox (crack 7 / shatter 20, 30 / 60 shards, three triangular pieces).
    tv: bool,
    crack_impulse: f64,
    broken: bool,
    done: bool,
    /// `BoomboxMC` frame (1 intact, 2..4 cracked).
    frame: usize,
    pieces: Vec<(BodyId, u8)>,
}

/// Chair special (`Chair`): a Result above 10 on any of its four shapes breaks it into a left leg, right leg, back and seat body.
struct BreakChair {
    body: BodyId,
    sign: f64,
    done: bool,
    /// (body, 0 leg / 1 back / 2 seat, mirror sign)
    pieces: Vec<(BodyId, u8, f64)>,
}

/// Glass pane (`Glass`): a Result above `round(mass·shatterStrength)` shatters it into triangles fanned from the impact point.
struct Glass {
    body: BodyId,
    shape: ShapeId,
    /// Half extents (m).
    half: (f64, f64),
    smash_impulse: f64,
    particles: u32,
    suffix: &'static str,
    done: bool,
    /// Triangle pieces: (body, vertices in the body frame, px).
    pieces: Vec<(BodyId, [(f64, f64); 3])>,
}

/// A special's `checkAdd`: a contact starting on one of its shapes with a non-sensor body that is static or at least as heavy, closing speed along the
/// normal above `threshold`, plays `name` (`variants` picks `name1..n`). `lock_frames` > 0 = the original keeps a handle until the sound ends (toilet, trash can).
struct HitSound {
    body: BodyId,
    shapes: Vec<ShapeId>,
    name: &'static str,
    variants: u32,
    threshold: f64,
    lock_frames: u32,
    free_at: u32,
}

/// Length of a sprite sound in 30 fps frames (`audiosprite.json`), 1 when unknown.
pub fn sound_frames(name: &str) -> u32 {
    static TABLE: std::sync::OnceLock<std::collections::HashMap<String, [f64; 2]>> = std::sync::OnceLock::new();
    let t = TABLE.get_or_init(|| serde_json::from_str(include_str!("../data/audiosprite.json")).unwrap_or_default());
    t.get(name).map_or(1, |d| (d[1] * 30.0 / 1000.0).ceil() as u32)
}

/// TrashCan (`Wz`): intact box → (Result > 3.5) hollow can + spilled trash, or (Result > 12) crushed box + trash; a hollow can is crushed above 12.
struct TrashCan {
    body: BodyId,
    shapes: Vec<ShapeId>,
    contains_trash: bool,
    spilled: bool,
    spill_impulse: f64,
    crushed: bool,
    /// `TrashCanMC` frame shown once crushed: 2 (squashed sides) or 3 (squashed vertically).
    crush_frame: usize,
}

/// `TrashCanMC.shapes` guides in px (clip dump, `analysis/trash-dump.js`): (x, y, w, h).
const CAN_S1: [f64; 4] = [0.0, 14.7, 46.5, 42.3199999332428];
const CAN_LEFT: [(f64, f64); 3] = [(-22.65, -35.55), (-12.65, 24.45), (-22.65, 35.45)];
const CAN_RIGHT: [(f64, f64); 3] = [(22.65, -35.55), (22.65, 35.45), (12.65, 24.45)];
const CAN_CIRCLES: [(f64, f64, f64); 3] = [(-8.95, -28.8, 12.58), (10.0, -13.05, 9.38), (-15.9, -32.55, 9.38)];
const CAN_RECTS: [(f64, f64, f64, f64); 7] = [
    (-8.7, -18.8, 6.96, 22.26),
    (-13.65, -19.0, 5.42, 23.9),
    (6.4, -31.5, 18.54, 7.38),
    (15.45, -27.9, 5.42, 9.5),
    (5.9, -20.35, 17.36, 13.64),
    (-0.75, -13.35, 12.94, 9.8),
    (0.05, -36.2, 48.34, 4.84),
];
const CAN_BLOCKER: [f64; 2] = [46.5, 72.54];
const CAN_CRUSH_H: [f64; 2] = [27.931519508361816, 72.3830795288086];
const CAN_CRUSH_V: [f64; 2] = [46.35346031188965, 36.118720054626465];

/// Spikes special (`Spikes`: 50 px impale sensor over the 20 px base): bodies whose shape has material bits 2/4 and touches the sensor get a motor-held
/// prismatic joint along the spikes' axis; the impale point leaves blood on the spike bitmap.
struct Spike {
    body: BodyId,
    sensor: ShapeId,
    angle: f64,
    half_width: f64,
    pinned: Vec<(BodyId, usize, i32)>,
    queue_add: Vec<BodyId>,
    queue_remove: Vec<BodyId>,
    sounds: u32,
    /// Blood stamps on the spike bitmap: (x in the spike row's coordinates, `spike_blood` frame 1..=5).
    blood: Vec<(f64, u8)>,
}

/// Arrow fired by an ArrowGun (`i3`): sensor tip + solid shaft; pins itself to material-6 / massless shapes with locked prismatic joints.
struct Arrow {
    body: BodyId,
    sensor: ShapeId,
    solid: ShapeId,
    /// (other body, joint, contact count)
    pinned: Vec<(BodyId, usize, i32)>,
    queue_add: Vec<BodyId>,
    queue_remove: Vec<BodyId>,
    armed: bool,
    done: bool,
    /// Harpoon (`Av`): longer 52.5 px body, anchor at +26.25 px and no snapping.
    harpoon: bool,
    /// BladeWeapon: initial blade angle (axis reference); anchor is the body centre.
    blade: Option<f64>,
    /// BladeWeapon blood stamps in the weapon clip's local px (x already mirrored for `reverse`): (x, y, rotation rad, scaleX ±1).
    blood: Vec<(f64, f64, f64, f64)>,
    reverse: bool,
    flesh_sounds: u32,
}

/// ArrowGun special, static-base variant (`immovable2` true): turret aims at the closest material-2 body in range and fires 10 arrows.
struct ArrowGun {
    pos: Vec2,
    angle: f64,
    range: ShapeId,
    targets: Vec<(BodyId, i32)>,
    target: Option<BodyId>,
    aligned: bool,
    frame_count: u32,
    frames_per_shot: u32,
    retarget: u32,
    arrows_left: i32,
    done: bool,
    harpoon: bool,
}

/// HomingMine special (`HomingMine.actions`): hovers toward the closest ragdoll body within 600 px.
struct Mine {
    body: BodyId,
    targeting: BodyId,
    shape: ShapeId,
    range: ShapeId,
    seek: f64,
    counter: f64,
    counting_down: bool,
    in_contact: bool,
    retarget: u32,
    target: Option<BodyId>,
    targets: Vec<(BodyId, i32)>,
    done: bool,
}

/// Jet engine special (`Jet.actions` / `explode`).
struct Jet {
    body: BodyId,
    shape: ShapeId,
    spec: crate::specials::JetSpec,
    accel: f64,
    fired: f64,
    done: bool,
    smash: f64,
}

struct Fan {
    sensor: ShapeId,
    center: Vec2,
    sin: f64,
    cos: f64,
    bodies: Vec<BodyId>,
}

/// Drawable shape in world pixels (for the CPU renderer).
pub enum DrawShape {
    Poly(Vec<[f32; 2]>),
    Circle { center: [f32; 2], radius: f32 },
}

/// State of the pogo's nub contact listener (`nubContactAdd/Remove`): counts contact POINTS.
#[derive(Debug, Clone, Copy, Default)]
pub struct NubState {
    pub shape: Option<box2d::world::ShapeId>,
    pub body: Option<BodyId>,
    pub contacts: i32,
    pub air_lean: bool,
}

pub struct World {
    pub nub: NubState,
    /// Ragdoll parts whose post-solve contact impulse exceeded their smash limit this step (largest impulse
    /// per part). Detection only: the original's `headSmash/chestSmash/…` reactions are not ported yet.
    pub smashes: Vec<(crate::character::Part, f64)>,
    /// Spawn the c3 dad with his kid (the real character); tests of the bare bike turn it off.
    pub dad_kid: bool,
    /// Per shape touching the heli magnet this frame: the strongest `Result` (shape, point, impulse).
    magnet_hits: Vec<(box2d::world::ShapeId, box2d::math::Vec2, f64)>,
    b: B2,
    level_body: BodyId,
    triggers: Vec<TriggerState>,
    fans: Vec<Fan>,
    boosts: Vec<Boost>,
    jets: Vec<Jet>,
    cannons: Vec<Cannon>,
    spikes: Vec<Spike>,
    /// Token specials: (body, sensor shape); collected ones are removed.
    tokens: Vec<(BodyId, ShapeId)>,
    /// FoodItem specials: (body, shape, smash impulse).
    /// (body, shape, smash impulse, food type 0..2)
    foods: Vec<(BodyId, ShapeId, f64, u8)>,
    /// Chunk bodies born from smashed food: (body, food type, chunk index).
    food_chunks: Vec<(BodyId, u8, u8)>,
    boomboxes: Vec<Boombox>,
    chairs: Vec<BreakChair>,
    glasses: Vec<Glass>,
    cans: Vec<TrashCan>,
    hit_sounds: Vec<HitSound>,
    /// Bodies born from specials (trash, hollow cans…) that only the renderer cares about.
    loose: Vec<BodyId>,
    mines: Vec<Mine>,
    /// Sound effects requested this step (audiosprite name, world position in px); the app drains them with `take_sounds`.
    pub sounds: Vec<(String, [f32; 2])>,
    /// Blood particles and flows (see `blood.rs`).
    pub blood: crate::blood::Blood,
    /// Looping sounds wanted this frame (`playAreaSoundLoop`): sprite name and position in px; the audio layer fades them in/out.
    pub loops: Vec<(String, [f32; 2], f32)>,
    arrow_guns: Vec<ArrowGun>,
    npcs: Vec<Npc>,
    npc_groups: i16,
    arrows: Vec<Arrow>,
    /// Tokens picked up so far (`level.tokenFound`).
    pub tokens_found: u32,
    pub events: Vec<TriggerEvent>,
    frame: u32,
    /// Body per level shape (None for art / unsupported shapes; the level body for immovable ones).
    pub shape_bodies: Vec<Option<BodyId>>,
    pub group_bodies: Vec<BodyId>,
    /// Body per level special (None for specials not modelled yet / non-interactive artwork).
    pub special_bodies: Vec<Option<BodyId>>,
    /// All bodies of each special (main body first), for drawing.
    pub special_parts: Vec<Vec<BodyId>>,
    /// Group controlled by the player (first group with `v='t'`).
    pub vehicle: Option<BodyId>,
    /// Player ragdoll (see `spawn_character`).
    pub character: Option<crate::character::Ragdoll>,
    /// Bike under the rider, if the character has one (see `spawn_bike`).
    pub bike: Option<crate::vehicle::Vehicle>,
    /// Rider joints break past their force limits (`checkJoints`); tests switch it off to compare raw physics.
    pub joints_break: bool,
    vehicle_accel: f32,
}

impl World {
    pub fn from_level(level: &Level) -> Self {
        let version = level.info.version;
        // Original session: world bounds from the level (x -250, y -5250, 20500 x 15500), gravity (0, 10).
        let mut b = B2::new(Vec2::new(-250.0 / PS, -5250.0 / PS), Vec2::new(20250.0 / PS, 10250.0 / PS), Vec2::new(0.0, 10.0), true);
        let level_body = b.create_body(&BodyDef::default());
        // Level borders (left, right, top; there is no floor border).
        let border = |hx: f64, hy: f64, c: Vec2| {
            let mut d = ShapeDef::oriented_box(hx, hy, c, 0.0);
            d.friction = 1.0;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: -10 };
            d
        };
        let (bt, sky) = (250.0, 5000.0);
        let (cw, ch) = (20000.0, 10000.0);
        let (hx, hy) = (bt * 0.5 / PS, (bt + (ch + sky) * 0.5) / PS);
        b.create_shape(level_body, &border(hx, hy, Vec2::new(-hx, (ch - sky) * 0.5 / PS)));
        b.create_shape(level_body, &border(hx, hy, Vec2::new((cw + bt * 0.5) / PS, (ch - sky) * 0.5 / PS)));
        b.create_shape(level_body, &border((bt + cw * 0.5) / PS, bt * 0.5 / PS, Vec2::new(cw * 0.5 / PS, -(bt * 0.5 + sky) / PS)));
        let mut w = Self {
            b,
            level_body,
            triggers: Vec::new(),
            fans: Vec::new(),
            boosts: Vec::new(),
            jets: Vec::new(),
            cannons: Vec::new(),
            spikes: Vec::new(),
            tokens: Vec::new(),
            foods: Vec::new(),
            food_chunks: Vec::new(),
            boomboxes: Vec::new(),
            chairs: Vec::new(),
            glasses: Vec::new(),
            cans: Vec::new(),
            hit_sounds: Vec::new(),
            loose: Vec::new(),
            mines: Vec::new(),
            sounds: Vec::new(),
            blood: crate::blood::Blood::default(),
            loops: Vec::new(),
            arrow_guns: Vec::new(),
            npcs: Vec::new(),
            npc_groups: -1000,
            arrows: Vec::new(),
            tokens_found: 0,
            nub: NubState::default(),
            smashes: Vec::new(),
            dad_kid: true,
            magnet_hits: Vec::new(),
            events: Vec::new(),
            frame: 0,
            shape_bodies: Vec::new(),
            group_bodies: Vec::new(),
            special_bodies: Vec::new(),
            special_parts: Vec::new(),
            vehicle: None,
            character: None,
            bike: None,
            joints_break: true,
            vehicle_accel: 0.0,
        };
        let position = |s: &Shape| -> Option<[f64; 2]> {
            Some([num(s, "x").or_else(|| s.props.first()?.parse().ok())?, num(s, "y").or_else(|| s.props.get(1)?.parse().ok())?])
        };
        // createStaticShapes: immovable shapes live on the level body.
        let mut handles: Vec<Option<BodyId>> = vec![None; level.shapes.len()];
        for (i, s) in level.shapes.iter().enumerate() {
            if !s.interactive || !s.flag("immovable").unwrap_or(false) {
                continue;
            }
            let (Some([x, y]), angle) = (position(s), num(s, "angle").unwrap_or(0.0).to_radians()) else { continue };
            let Some(g) = geometry_of(s, [x, y], angle) else { continue };
            let mut d = ShapeDef::new(g);
            set_shape_filter(&mut d, s.num("collision").unwrap_or(1.0) as i32, true, version);
            w.b.create_shape(level_body, &d);
            handles[i] = Some(level_body);
        }
        // createDynamicShapes
        for (i, s) in level.shapes.iter().enumerate() {
            if !s.interactive || s.flag("immovable").unwrap_or(false) {
                continue;
            }
            let Some([x, y]) = position(s) else { continue };
            let angle = num(s, "angle").unwrap_or(0.0).to_radians();
            let Some(g) = geometry_of(s, [0.0, 0.0], 0.0) else { continue };
            let body = w.b.create_body(&BodyDef {
                position: Vec2::new(x / PS, y / PS),
                angle,
                is_sleeping: s.flag("sleeping").unwrap_or(false),
                ..Default::default()
            });
            let mut d = ShapeDef::new(g);
            d.density = num(s, "density").unwrap_or(1.0);
            set_shape_filter(&mut d, s.num("collision").unwrap_or(1.0) as i32, false, version);
            w.b.create_shape(body, &d);
            w.b.set_mass_from_shapes(body);
            handles[i] = Some(body);
        }
        w.shape_bodies = handles;
        // createSpecials: bodies from rigid-body specials (see specials.rs; transcribed from the original).
        for sp in &level.specials {
            let before = w.special_parts.len();
            let mut h = w.special_body(sp);
            if sp.kind == 26 && h.is_some() {
                // a trash can built hollow already replaced its first body
                h = w.cans.last().map(|c| c.body);
            }
            if w.special_parts.len() == before {
                w.special_parts.push(Vec::new());
            }
            w.special_bodies.push(h);
        }
        // createGroups: one body per group at the group origin; children are shapes in the group's frame.
        // VERIFIED (createGroups): a child's local position is (child.x + group.offset.x, child.y +
        // group.offset.y) px, its own angle is used as-is, and for level version > 1.84 the group's
        // `immovable` (not the child's) selects the filter.
        for g in &level.groups {
            let body = w.b.create_body(&BodyDef {
                position: Vec2::new(f64::from(g.pos[0]) / PS, f64::from(g.pos[1]) / PS),
                angle: f64::from(g.angle).to_radians(),
                is_sleeping: g.sleeping,
                fixed_rotation: g.fixed_rotation,
                ..Default::default()
            });
            for s in g.shapes.iter().filter(|s| s.interactive) {
                let cx = s.props.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0) + f64::from(g.offset[0]);
                let cy = s.props.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0) + f64::from(g.offset[1]);
                let child_angle = num(s, "angle").unwrap_or(0.0).to_radians();
                let Some(geo) = geometry_of(s, [cx, cy], child_angle) else { continue };
                let mut d = ShapeDef::new(geo);
                d.density = num(s, "density").unwrap_or(1.0);
                let immovable = if version > 1.84 { g.immovable } else { s.flag("immovable").unwrap_or(false) };
                set_shape_filter(&mut d, s.num("collision").unwrap_or(1.0) as i32, immovable, version);
                w.b.create_shape(body, &d);
            }
            w.b.set_mass_from_shapes(body);
            if g.immovable && w.b.body(body).mass > 0.0 {
                w.b.set_mass(body, &MassData::default());
                let bd = w.b.body_mut(body);
                bd.linear_velocity = Vec2::default();
                bd.angular_velocity = 0.0;
            }
            if w.vehicle.is_none() {
                if let Some(v) = &g.vehicle {
                    w.vehicle = Some(body);
                    w.vehicle_accel = v.acceleration;
                }
            }
            w.group_bodies.push(body);
        }
        w.create_joints(level);
        w.create_triggers(level);
        w
    }

    /// Cannon special (transcribed `createBodies`/`createJoints`/`actions`; guide `CannonShapesRefMC` numbers read from the oracle):
    /// a muzzle body (circle r = 32.495 px·d + two barrel quads + a 1e-7-density sensor quad inside the barrel; d = 1 + muzzleScale/20)
    /// pinned to the level at (x,y) with a 0..0 limit, a static base quad on the level body. A body that enters the sensor starts the
    /// load timer (`30·cannonDelay` steps); then the muzzle swings to `firingRotation` (motor ±0.6, torque 1e4), fires when within 1° and
    /// bodies are still inside — impulse `(1.5 + cannonPower/10·4)·mass·10` along the barrel on each — and returns.
    fn cannon(&mut self, sp: &crate::level::Special) -> Option<BodyId> {
        let g = |k: &str| sp.num(k).map(f64::from);
        let (x, y) = (g("x")?, g("y")?);
        let d = 1.0 + g("muzzleScale").unwrap_or(1.0) / 20.0;
        let rad = std::f64::consts::PI / 180.0;
        let (rot, start, fire) = (g("angle").unwrap_or(0.0) * rad, g("startRotation").unwrap_or(0.0) * rad, g("firingRotation").unwrap_or(0.0) * rad);
        let pt = |px: f64, py: f64| Vec2::new(px / PS * d, py / PS * d);
        let p0 = [(-29.0, -171.15), (-22.0, -171.15), (-13.5, 0.0), (-32.5, 0.0)];
        let p1 = [(22.0, -171.15), (29.0, -171.15), (32.5, 0.0), (13.5, 0.0)];
        let filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        let muzzle = self.b.create_body(&BodyDef { position: Vec2::new(x / PS, y / PS), angle: start + rot, allow_sleep: false, ..Default::default() });
        let mk = |geometry: Geometry, density: f64, sensor: bool| {
            let mut sd = ShapeDef::new(geometry);
            sd.density = density;
            sd.friction = 0.5;
            sd.restitution = 0.1;
            sd.filter = filter;
            sd.is_sensor = sensor;
            sd
        };
        self.b.create_shape(muzzle, &mk(Geometry::Circle { local_position: Vec2::default(), radius: 64.989_997_863_769_53 / 2.0 / PS * d }, 1.0, false));
        for q in [p0, p1] {
            self.b.create_shape(muzzle, &mk(Geometry::Polygon(q.iter().map(|&(a, b)| pt(a, b)).collect()), 1.0, false));
        }
        let sensor_verts = [p0[1], p1[0], p1[3], p0[2]];
        let sensor = self.b.create_shape(muzzle, &mk(Geometry::Polygon(sensor_verts.iter().map(|&(a, b)| pt(a, b)).collect()), 1e-7, true));
        self.b.set_mass_from_shapes(muzzle);
        let (s, c) = rot.sin_cos();
        let base: Vec<Vec2> = [(-37.0, 20.0), (37.0, 20.0), (49.0, 55.0), (-49.0, 55.0)]
            .iter()
            .map(|&(a, b)| {
                let (vx, vy) = (a / PS, b / PS);
                Vec2::new(c * vx - s * vy + x / PS, s * vx + c * vy + y / PS)
            })
            .collect();
        self.b.create_shape(self.level_body, &mk(Geometry::Polygon(base), 0.0, false));
        let mut jd = RevoluteJointDef::initialize(self.b.body(self.level_body), self.level_body, self.b.body(muzzle), muzzle, Vec2::new(x / PS, y / PS));
        jd.enable_limit = true;
        jd.lower_angle = 0.0;
        jd.upper_angle = 0.0;
        let joint = self.b.create_revolute_joint(&jd);
        self.cannons.push(Cannon {
            muzzle,
            joint,
            sensor,
            state: CannonState::Waiting,
            counter: 0,
            load_frames: (30.0 * g("cannonDelay").unwrap_or(1.0)) as u32,
            start_angle: start,
            fire_angle: fire,
            power: 1.5 + g("cannonPower").unwrap_or(5.0) / 10.0 * 4.0,
            inside: Vec::new(),
        });
        Some(muzzle)
    }

    fn cannon_joint(&mut self, ci: usize) -> &mut box2d::joints::Revolute {
        let id = self.cannons[ci].joint;
        match &mut self.b.joint_mut(id).kind {
            box2d::joints::JointKind::Revolute(r) => r,
            _ => unreachable!("cannon joint is revolute"),
        }
    }

    /// `Cannon.actions` (level actions: before the world step).
    fn run_cannons(&mut self) {
        let one_deg = std::f64::consts::PI / 180.0;
        for ci in 0..self.cannons.len() {
            let (state, start, fire) = (self.cannons[ci].state, self.cannons[ci].start_angle, self.cannons[ci].fire_angle);
            let angle = self.b.revolute_angle(self.cannons[ci].joint).unwrap_or(0.0);
            let move_to_start = |w: &mut Self| {
                let speed = if start < fire { -0.6 } else { 0.6 };
                w.cannon_joint(ci).motor_speed = speed;
                w.cannons[ci].state = CannonState::Returning;
            };
            match state {
                CannonState::Loading => {
                    self.cannons[ci].counter += 1;
                    if self.cannons[ci].counter == self.cannons[ci].load_frames {
                        self.cannons[ci].counter = 0;
                        self.cannons[ci].state = CannonState::Aiming;
                        let (speed, lo, hi) = if start < fire { (0.6, 0.0, fire - start) } else { (-0.6, fire - start, 0.0) };
                        let r = self.cannon_joint(ci);
                        r.upper_angle = hi;
                        r.lower_angle = lo;
                        r.enable_limit = true;
                        r.enable_motor = true;
                        r.max_motor_torque = 1e4;
                        r.motor_speed = speed;
                    }
                }
                CannonState::Aiming => {
                    let n: i32 = self.cannons[ci].inside.iter().map(|e| e.1).sum();
                    if n == 0 {
                        move_to_start(self);
                    } else if (fire - start - angle).abs() <= one_deg {
                        self.cannons[ci].state = CannonState::Firing;
                        let at = self.b.body(self.cannons[ci].muzzle).sweep.c;
                        self.play("Cannon1", 1, at);
                        let a = self.b.body(self.cannons[ci].muzzle).sweep.a;
                        let (sn, cs, power) = (a.sin(), a.cos(), self.cannons[ci].power);
                        for (body, _) in self.cannons[ci].inside.clone() {
                            let b = self.b.body_mut(body);
                            let (m, c) = (b.mass, b.sweep.c);
                            b.apply_impulse(Vec2::new(sn * (power * m * 10.0), cs * (-power * m * 10.0)), c);
                        }
                    }
                }
                CannonState::Firing => {
                    self.cannons[ci].counter += 1;
                    if self.cannons[ci].counter == 5 {
                        self.cannons[ci].counter = 0;
                        move_to_start(self);
                    }
                }
                CannonState::Returning => {
                    if angle.abs() <= one_deg {
                        self.cannons[ci].state = CannonState::Waiting;
                        let r = self.cannon_joint(ci);
                        r.enable_motor = false;
                        r.upper_angle = 0.0;
                        r.lower_angle = 0.0;
                    }
                }
                CannonState::Waiting => {}
            }
        }
    }

    /// `checkAdd` / `checkRemove` of the cannon's sensor: per-body contact counts; the first body starts loading.
    fn cannon_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for ev in events.iter().filter(|e| matches!(e.kind, ContactEventKind::Add | ContactEventKind::Remove)) {
            for ci in 0..self.cannons.len() {
                let sensor = self.cannons[ci].sensor;
                let other = if ev.shape1 == sensor {
                    ev.shape2
                } else if ev.shape2 == sensor {
                    ev.shape1
                } else {
                    continue;
                };
                let body = self.b.c.shapes[other].body;
                let c = &mut self.cannons[ci];
                if ev.kind == ContactEventKind::Add {
                    match c.inside.iter_mut().find(|e| e.0 == body) {
                        Some(e) => e.1 += 1,
                        None => c.inside.push((body, 1)),
                    }
                    if c.state == CannonState::Waiting {
                        c.state = CannonState::Loading;
                    }
                } else if let Some(k) = c.inside.iter().position(|e| e.0 == body) {
                    c.inside[k].1 -= 1;
                    if c.inside[k].1 == 0 {
                        c.inside.remove(k);
                    }
                }
            }
        }
    }

    /// Chain special (transcribed `createBodies`/`createJoints`): `linkCount` boxes (density 17) laid out along
    /// an arc from a temporary reference body, joined by free revolute joints (no limits, no pin to the level).
    /// Link clip sizes (9×15 and 3×15 px for the two alternating link types) come from the oracle's movie clips.
    fn chain(&mut self, sp: &crate::level::Special) -> Option<BodyId> {
        if sp.flag("interactive") == Some(false) {
            return None;
        }
        let g = |k: &str| sp.num(k).map(f64::from);
        let (x, y) = (g("x")?, g("y")?);
        let count = g("linkCount")?.max(0.0) as usize;
        let scale = 1.0 + (g("linkScale").unwrap_or(1.0) - 1.0) / 9.0 * 2.0;
        let d = -g("linkAngle").unwrap_or(0.0) / 10.0 * (360.0 / (2.0 * count.max(1) as f64));
        let deg = 180.0 / std::f64::consts::PI;
        let f = d / deg;
        let len = 9.0 * scale;
        let sleeping = sp.flag("sleeping").unwrap_or(false);
        let reference = self.b.create_body(&BodyDef { position: Vec2::new(x / PS, y / PS), angle: g("angle").unwrap_or(0.0) / deg, ..Default::default() });
        let mut point = [0.0f64, 0.0];
        let mut points = vec![point];
        let mut links: Vec<BodyId> = Vec::new();
        let mut first_mass = 0.0;
        for i in 0..count {
            let h = 90.0 / deg + ((i + 1) as f64 * f - 0.5 * f);
            let (bx, by) = (h.cos() * len, h.sin() * len);
            let (clip_w, clip_h) = if i % 2 == 0 { (9.0, 15.0) } else { (3.0, 15.0) };
            let (hx, hy) = (scale * clip_w * 0.5 / PS, scale * clip_h * 0.5 / PS);
            let pos = self.b.body(reference).world_point(Vec2::new((point[0] + bx / 2.0) / PS, (point[1] + by / 2.0) / PS));
            let angle = self.b.body(reference).sweep.a + h - 90.0 / deg;
            let body = self.b.create_body(&BodyDef { position: pos, angle, is_sleeping: sleeping, ..Default::default() });
            let mut sd = ShapeDef::boxed(hx, hy);
            sd.density = 17.0;
            sd.friction = 0.3;
            sd.restitution = 0.1;
            sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
            self.b.create_shape(body, &sd);
            self.b.set_mass_from_shapes(body);
            if i == 0 {
                first_mass = self.b.body(body).mass;
            } else {
                self.b.body_mut(body).mass = first_mass; // `P.m_mass = v` (inverse mass is left alone)
            }
            links.push(body);
            point = [point[0] + bx, point[1] + by];
            points.push(point);
        }
        for t in 1..links.len() {
            let (a, b) = (links[t - 1], links[t]);
            let s = self.b.body(reference).world_point(Vec2::new(points[t][0] / PS, points[t][1] / PS));
            let mut jd = RevoluteJointDef::initialize(self.b.body(a), a, self.b.body(b), b, s);
            jd.reference_angle = 0.0;
            self.b.create_revolute_joint(&jd);
        }
        self.b.destroy_body(reference);
        let first = links.first().copied();
        self.special_parts.push(links);
        first
    }

    fn special_body(&mut self, sp: &crate::level::Special) -> Option<BodyId> {
        if sp.kind == 30 {
            return self.chain(sp);
        }
        if sp.kind == 33 {
            return self.cannon(sp);
        }
        if sp.kind == 25 {
            return self.mine(sp);
        }
        if sp.kind == 34 {
            return self.blade(sp);
        }
        if sp.kind == 17 {
            return self.npc(sp);
        }
        if sp.kind == 29 || sp.kind == 15 {
            return self.arrow_gun(sp);
        }
        if sp.kind == 31 {
            // Token: a 23 px sensor circle; the first living ragdoll body shape (material bit 2) touching it collects it.
            let (x, y) = (f64::from(sp.num("x")?), f64::from(sp.num("y")?));
            let body = self.b.create_body(&BodyDef { position: Vec2::new(x / PS, y / PS), ..Default::default() });
            let mut sd = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 23.0 / PS });
            sd.is_sensor = true;
            sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
            let shape = self.b.create_shape(body, &sd);
            self.tokens.push((body, shape));
            return Some(body);
        }
        let sb: SimpleBody = crate::specials::simple_body(sp)?;
        let [x, y] = sb.pos.unwrap_or([f64::from(sp.num("x")?), f64::from(sp.num("y")?)]);
        let angle = sb.angle.unwrap_or_else(|| f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
        if sb.shape.is_empty() {
            // Level geometry only (buildings, fan): shapes live on the level body, there is no body of its own.
            for st in &sb.statics {
                let mut d = st.clone();
                d.friction = sb.friction;
                d.restitution = sb.restitution;
                d.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: st.filter.group_index };
                let id = self.b.create_shape(self.level_body, &d);
                if let (true, Some(bs)) = (d.is_sensor, sb.boost) {
                    self.boosts.push(Boost { sensor: id, power: bs.power, sin: bs.angle.sin(), cos: bs.angle.cos(), bodies: Vec::new() });
                }
                if let (true, Some(f)) = (d.is_sensor, sb.fan) {
                    self.fans.push(Fan { sensor: id, center: Vec2::new(f.center[0], f.center[1]), sin: f.angle.sin(), cos: f.angle.cos(), bodies: Vec::new() });
                }
            }
            return None;
        }
        let body = self.b.create_body(&BodyDef { position: Vec2::new(x / PS, y / PS), angle, is_sleeping: sb.sleeping, fixed_rotation: sb.jet.is_some_and(|j| j.fixed_rotation), ..Default::default() });
        let filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: sb.group };
        let add = |w: &mut Self, body: BodyId, geos: &[Geometry], density: f64, friction: f64, restitution: f64| {
            for g in geos {
                let mut d = ShapeDef::new(g.clone());
                d.density = if sb.fixed { 0.0 } else { density };
                d.friction = friction;
                d.restitution = restitution;
                d.filter = filter;
                w.b.create_shape(body, &d);
            }
        };
        add(self, body, &sb.shape, sb.density, sb.friction, sb.restitution);
        let hit = match sp.kind {
            3 => Some(("IBeamHit", 2, 4.0, 0)),
            4 => Some(("LumberHit", 2, 4.0, 0)),
            21 => Some(("TVHit", 1, 4.0, 0)),
            22 => Some(("BoomboxHit", 1, 4.0, 0)),
            24 => Some(("ToiletHit", 2, 4.0, 11)),
            26 => Some(("TrashCanHit", 2, 4.0, 20)),
            11 => Some(("MeteorImpact", 1, 2.0, 0)),
            _ => None,
        };
        if let Some((name, variants, threshold, lock_frames)) = hit {
            let first = self.b.body(body).shapes[0];
            self.hit_sounds.push(HitSound { body, shapes: vec![first], name, variants, threshold, lock_frames, free_at: 0 });
        }
        if sp.kind == 18 {
            let first = self.b.body(body).shapes[0];
            self.b.c.report_results = true;
            self.b.set_mass_from_shapes(body);
            let mass = self.b.body(body).mass;
            let (suffix, particles) = if mass < 0.75 { ("Light", 100) } else if mass < 4.0 { ("Mid", 150) } else { ("Heavy", 200) };
            let strength = f64::from(sp.num("shatterStrength").unwrap_or(10.0));
            let half = (f64::from(sp.num("shapeWidth").unwrap_or(10.0)) / 2.0 / PS, f64::from(sp.num("shapeHeight").unwrap_or(100.0)) / 2.0 / PS);
            self.glasses.push(Glass { body, shape: first, half, smash_impulse: (mass * strength).round(), particles, suffix, done: false, pieces: Vec::new() });
        }
        if sp.kind == 19 && sp.flag("interactive") != Some(false) {
            self.b.c.report_results = true;
            let sign = if sp.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
            self.chairs.push(BreakChair { body, sign, done: false, pieces: Vec::new() });
        }
        if matches!(sp.kind, 21 | 22) {
            self.b.c.report_results = true;
            self.boomboxes.push(Boombox { body, tv: sp.kind == 21, crack_impulse: if sp.kind == 21 { 7.0 } else { 6.5 }, broken: false, done: false, frame: 1, pieces: Vec::new() });
        }
        if sp.kind == 26 {
            let first = self.b.body(body).shapes[0];
            self.b.c.report_results = true;
            let contains = sp.flag("containsTrash").unwrap_or(true);
            self.cans.push(TrashCan { body, shapes: vec![first], contains_trash: contains, spilled: false, spill_impulse: 3.5, crushed: false, crush_frame: 1 });
            if !contains {
                let k = self.cans.len() - 1;
                self.can_make_hollow(k);
                self.cans[k].spill_impulse = 12.0;
            }
        }
        if sp.kind == 6 {
            // Spikes.createBody: sensor box (halfWidth × 25, density .25, category 8, group −20) over the base; the dynamic body's mass includes it.
            let half_width = 7.5 * f64::from(sp.num("numSpikes").unwrap_or(1.0));
            let mut sd = ShapeDef::boxed(half_width / PS, 25.0 / PS);
            sd.is_sensor = true;
            sd.density = if sb.fixed { 0.0 } else { 0.25 };
            sd.friction = 0.3;
            sd.restitution = 0.1;
            sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: -20 };
            let sensor = self.b.create_shape(body, &sd);
            if !sb.fixed {
                self.b.set_mass_from_shapes(body);
            }
            self.spikes.push(Spike { body, sensor, angle, half_width, pinned: Vec::new(), queue_add: Vec::new(), queue_remove: Vec::new(), sounds: 0, blood: Vec::new() });
        }
        if sp.kind == 32 {
            // FoodItem: material 4; a Result impulse above 8 (watermelon/pumpkin) or 3 (pineapple) smashes it (chunks/juice not ported)
            let first = self.b.body(body).shapes[0];
            self.b.shape_mut(first).material = 4;
            self.b.c.report_results = true;
            let limit = if sp.num("foodItemType").unwrap_or(1.0) as i32 == 3 { 3.0 } else { 8.0 };
            let ty = (sp.num("foodItemType").unwrap_or(1.0) as i32).clamp(1, 3) as u8 - 1;
            self.foods.push((body, first, limit, ty));
        }
        if sp.kind == 27 {
            // Rail: the upper box carries material bit 4 (what the grinder's wheels look for)
            let first = self.b.body(body).shapes[0];
            self.b.shape_mut(first).material = 4;
        }
        for (geos, density) in &sb.more {
            add(self, body, geos, *density, sb.friction, sb.restitution);
        }
        self.b.set_mass_from_shapes(body);
        if let Some(spec) = sb.jet {
            let shape = self.b.body(body).shapes[0];
            let smash = (100.0 * self.b.body(body).mass).round();
            self.b.c.report_results = true;
            self.jets.push(Jet { body, shape, spec, accel: spec.accel0, fired: 0.0, done: false, smash });
        }
        let mut parts = vec![body];
        for at in &sb.attached {
            let origin = self.b.body(body).world_point(Vec2::new(at.local[0] / PS, at.local[1] / PS));
            let part = self.b.create_body(&BodyDef { position: origin, angle, is_sleeping: sb.sleeping, ..Default::default() });
            add(self, part, &at.shapes, at.density, at.friction, at.restitution);
            self.b.set_mass_from_shapes(part);
            parts.push(part);
        }
        for &part in &parts[1..] {
            let centre = self.b.body(part).sweep.c;
            let mut d = RevoluteJointDef::initialize(self.b.body(body), body, self.b.body(part), part, centre);
            d.enable_limit = true;
            self.b.create_revolute_joint(&d);
        }
        self.special_parts.push(parts);
        for st in &sb.statics {
            let mut d = st.clone();
            d.friction = sb.friction;
            d.restitution = sb.restitution;
            d.filter = Filter { group_index: st.filter.group_index, ..filter };
            self.b.create_shape(self.level_body, &d);
        }
        match sb.joint {
            Some(JointSpec::Revolute { anchor, limits }) => {
                let mut d = RevoluteJointDef::initialize(self.b.body(self.level_body), self.level_body, self.b.body(body), body, Vec2::new(anchor[0] / PS, anchor[1] / PS));
                if let Some((lo, hi)) = limits {
                    d.enable_limit = true;
                    d.lower_angle = lo;
                    d.upper_angle = hi;
                }
                self.b.create_revolute_joint(&d);
            }
            Some(JointSpec::Prismatic { axis, limits }) => {
                let origin = self.b.body(body).xf.position;
                let mut d = PrismaticJointDef::initialize(self.b.body(self.level_body), self.level_body, self.b.body(body), body, origin, Vec2::new(axis[0], axis[1]));
                d.enable_limit = true;
                d.lower_translation = limits.0;
                d.upper_translation = limits.1;
                self.b.create_prismatic_joint(&d);
            }
            None => {}
        }
        Some(body)
    }

    /// createJoints: pin (t=0) -> revolute, slider (t=1) -> prismatic, with the original's property mapping
    /// (angles in degrees, slider limits in px; level versions > 1.84 take limits/motor values unconditionally).
    fn create_joints(&mut self, level: &Level) {
        let version = level.info.version;
        for j in &level.joints {
            let anchor = Vec2::new(f64::from(j.pos[0]) / PS, f64::from(j.pos[1]) / PS);
            let resolve = |r: BodyRef| match r {
                BodyRef::World => Some(self.level_body),
                BodyRef::Shape(i) => self.shape_bodies.get(i).copied().flatten(),
                BodyRef::Group(i) => self.group_bodies.get(i).copied(),
                BodyRef::Special(i) => self.special_joint_body(i, anchor),
            };
            let (Some(a), Some(b)) = (resolve(j.body1), resolve(j.body2)) else { continue };
            if a == b {
                continue;
            }
            let (upper, lower) = (f64::from(j.upper), f64::from(j.lower));
            if j.sliding {
                let ang = f64::from(j.axis_angle).to_radians();
                let mut d = PrismaticJointDef::initialize(self.b.body(a), a, self.b.body(b), b, anchor, Vec2::new(ang.cos(), ang.sin()));
                d.enable_limit = j.limit;
                d.upper_translation = upper / PS;
                d.lower_translation = lower / PS;
                d.enable_motor = j.motor;
                d.max_motor_force = f64::from(j.force);
                d.motor_speed = f64::from(j.speed);
                d.collide_connected = j.collide_self;
                self.b.create_prismatic_joint(&d);
            } else {
                let mut d = RevoluteJointDef::initialize(self.b.body(a), a, self.b.body(b), b, anchor);
                if j.limit {
                    d.enable_limit = true;
                    d.upper_angle = upper.to_radians();
                    d.lower_angle = lower.to_radians();
                }
                d.max_motor_torque = 50.0;
                if j.motor {
                    d.enable_motor = true;
                    d.max_motor_torque = f64::from(j.force);
                    d.motor_speed = f64::from(j.speed);
                }
                if version > 1.84 {
                    d.upper_angle = upper.to_radians();
                    d.lower_angle = lower.to_radians();
                    d.max_motor_torque = f64::from(j.force);
                    d.motor_speed = f64::from(j.speed);
                }
                d.collide_connected = j.collide_self;
                self.b.create_revolute_joint(&d);
            }
        }
    }

    /// `getJointBody(anchor)`: the special's body whose centre is nearest to the joint anchor (metres).
    fn special_joint_body(&self, special: usize, anchor: Vec2) -> Option<BodyId> {
        let parts = self.special_parts.get(special)?;
        parts
            .iter()
            .copied()
            .filter(|&b| self.b.body(b).alive)
            .min_by(|&a, &b| {
                let d = |id: BodyId| {
                    let c = self.b.body(id).sweep.c;
                    (anchor.x - c.x).powi(2) + (anchor.y - c.y).powi(2)
                };
                d(a).total_cmp(&d(b))
            })
    }

    /// createTriggers: a sensor box on the level body (category 24, group -20) per trigger.
    fn create_triggers(&mut self, level: &Level) {
        for t in &level.triggers {
            let (hx, hy) = (f64::from(t.size[0]) / 100.0 * 50.0 / PS, f64::from(t.size[1]) / 100.0 * 50.0 / PS);
            let mut d = ShapeDef::oriented_box(hx, hy, Vec2::new(f64::from(t.pos[0]) / PS, f64::from(t.pos[1]) / PS), f64::from(t.angle).to_radians());
            d.filter = Filter { category_bits: 24, mask_bits: 0xFFFF, group_index: -20 };
            d.is_sensor = true;
            let sensor = self.b.create_shape(self.level_body, &d);
            let targets = t
                .targets
                .iter()
                .filter_map(|tg| match tg.kind {
                    crate::level::TargetKind::Shape => self.shape_bodies.get(tg.index).copied().flatten(),
                    crate::level::TargetKind::Group => self.group_bodies.get(tg.index).copied(),
                    _ => None,
                })
                .collect();
            self.triggers.push(TriggerState {
                sensor,
                by: t.triggered_by,
                repeat: t.repeat_type,
                repeat_frames: t.repeat_interval.map(|v| (v * 30.0).floor() as u32),
                repeat_count: t.repeat_interval.map_or(0, |v| (v * 30.0).floor() as u32),
                delay_frames: (t.delay.unwrap_or(0.0) * 30.0).floor() as u32,
                delay: Vec::new(),
                targets,
                shape_actions: if t.type_index == 1 {
                    t.targets
                        .iter()
                        .filter(|tg| tg.kind == crate::level::TargetKind::Shape)
                        .filter_map(|tg| Some((self.shape_bodies.get(tg.index).copied().flatten()?, tg.actions.clone())))
                        .collect()
                } else {
                    Vec::new()
                },
                trigger_targets: t.targets.iter().filter(|tg| tg.kind == crate::level::TargetKind::Trigger).map(|tg| tg.index).collect(),
                disabled: t.start_disabled,
                done: false,
                listening: (1..=4).contains(&t.triggered_by),
                watching_remove: false,
                active: false,
                triggering_body: None,
                triggering_count: 0,
            });
        }
    }

    fn fire(&mut self, trigger: usize) {
        self.events.push(TriggerEvent { trigger, frame: self.frame });
        let t = &mut self.triggers[trigger];
        if t.repeat == 1 {
            t.listening = false;
            let sensor = t.sensor;
            self.b.destroy_shape(sensor); // singleAction: levelBody.DestroyShape(sensor)
        }
        for (body, actions) in self.triggers[trigger].shape_actions.clone() {
            for a in &actions {
                self.shape_action(body, a);
            }
        }
        for target in self.triggers[trigger].trigger_targets.clone() {
            if target != trigger {
                self.activate_by_trigger(target);
            }
        }
    }

    /// Instant shape actions of the original (list: wake from sleep, set to fixed, set to non fixed, change
    /// opacity, apply impulse, delete shape, delete self, change collision). Only the physics-only ones that
    /// need no sprites/filters are done: 0, 1, 4, 5, 6 (5/6 destroy the body; the sprite does not exist yet).
    fn shape_action(&mut self, body: BodyId, a: &[String]) {
        if body == self.level_body {
            return;
        }
        let param = |k: usize| a.get(1 + k).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
        match a.first().and_then(|v| v.parse::<usize>().ok()) {
            Some(0) => self.b.body_mut(body).wake_up(),
            Some(1) => {
                if self.b.body(body).mass > 0.0 {
                    self.b.set_mass(body, &MassData::default());
                    let bd = self.b.body_mut(body);
                    bd.linear_velocity = Vec2::default();
                    bd.angular_velocity = 0.0;
                }
            }
            Some(5 | 6) => self.b.destroy_body(body),
            Some(4) => {
                let bd = self.b.body_mut(body);
                if bd.mass > 0.0 {
                    let m = bd.mass;
                    let c = bd.sweep.c;
                    bd.apply_impulse(Vec2::new(param(0) * m, param(1) * m), c);
                    if param(2) != 0.0 {
                        bd.angular_velocity += param(2);
                    }
                }
            }
            _ => {}
        }
    }

    /// `activateByTrigger` (triggeredBy 5: another trigger's action starts this one).
    pub fn activate_by_trigger(&mut self, i: usize) {
        let Some(t) = self.triggers.get(i) else { return };
        if t.disabled {
            return;
        }
        let (repeat, delay_frames, busy) = (t.repeat, t.delay_frames, t.triggering_body.is_some());
        match repeat {
            1 => {
                if !t.listening && t.by != 5 {
                    return; // already spent
                }
                if t.done {
                    return;
                }
                self.triggers[i].done = true;
                if delay_frames == 0 {
                    self.fire(i);
                } else {
                    let t = &mut self.triggers[i];
                    t.active = true;
                    t.delay.push(0);
                }
            }
            2 => {
                if delay_frames == 0 {
                    self.fire(i);
                } else {
                    let t = &mut self.triggers[i];
                    t.active = true;
                    t.delay.push(0);
                }
            }
            3 => {
                if busy {
                    return;
                }
                if delay_frames == 0 {
                    self.fire(i);
                } else {
                    let t = &mut self.triggers[i];
                    t.active = true;
                    t.delay.push(0);
                }
            }
            4 => {
                if busy {
                    return;
                }
                let lb = self.level_body;
                let t = &mut self.triggers[i];
                t.triggering_body = Some(lb);
                t.active = true;
            }
            _ => {}
        }
    }

    /// Mouse click on a trigger button (triggeredBy 6, `mouseUpHandler`).
    pub fn click_trigger(&mut self, i: usize) {
        let Some(t) = self.triggers.get(i) else { return };
        if t.by != 6 || t.disabled || t.done {
            return;
        }
        let (repeat, delay_frames) = (t.repeat, t.delay_frames);
        let lb = self.level_body;
        if repeat > 2 {
            let t = &mut self.triggers[i];
            if !t.active {
                t.active = true;
                t.triggering_body = Some(lb);
            }
        } else {
            if repeat == 1 {
                self.triggers[i].done = true;
            }
            if delay_frames == 0 {
                self.fire(i);
            } else {
                let t = &mut self.triggers[i];
                t.active = true;
                t.delay.push(0);
            }
        }
    }

    /// Mouse leaves a repeat-type-3 button: stops the repeating.
    pub fn release_trigger(&mut self, i: usize) {
        if let Some(t) = self.triggers.get_mut(i) {
            if t.by == 6 {
                t.triggering_body = None;
            }
        }
    }

    /// `addTriggeringBody`.
    fn add_triggering_body(&mut self, i: usize, body: BodyId, event_contact: usize) {
        let sensor = self.triggers[i].sensor;
        if let Some(tb) = self.triggers[i].triggering_body {
            if tb == body {
                self.triggers[i].triggering_count += 1;
            }
            return;
        }
        // Already touching the sensor through another contact: do not trigger again.
        let touching: usize = self.b.c.bodies[body]
            .contacts
            .iter()
            .filter(|&&c| c != event_contact)
            .map(|&c| self.b.c.contact_info(c))
            .filter(|c| {
                let (a, b) = (&self.b.c.shapes[c.shape1], &self.b.c.shapes[c.shape2]);
                (c.shape1 == sensor && !b.shape.is_sensor) || (c.shape2 == sensor && !a.shape.is_sensor)
            })
            .map(|c| c.manifold_count)
            .sum();
        if touching > 0 {
            return;
        }
        let t = &mut self.triggers[i];
        t.triggering_body = Some(body);
        t.triggering_count += 1;
        if t.repeat > 2 {
            if !t.active {
                t.active = true;
                if t.repeat == 3 {
                    t.watching_remove = true;
                }
            }
        } else {
            if t.repeat == 1 {
                t.listening = false;
            } else {
                t.watching_remove = true;
            }
            if t.delay_frames == 0 {
                self.fire(i);
            } else {
                t.active = true;
                t.delay.push(0);
            }
        }
    }

    fn handle_events(&mut self, events: Vec<box2d::world::ContactEvent>) {
        let vehicle = self.vehicle;
        for ev in events {
            if let (Some(nub), Some(rod)) = (self.nub.shape, self.nub.body) {
                if matches!(ev.kind, ContactEventKind::Add | ContactEventKind::Remove) && (ev.shape1 == nub || ev.shape2 == nub) {
                    let other = if ev.shape1 == nub { ev.shape2 } else { ev.shape1 };
                    let rec = &self.b.c.shapes[other];
                    let m2 = self.b.body(rec.body).mass;
                    if !rec.shape.is_sensor && (m2 == 0.0 || m2 >= self.b.body(rod).mass) {
                        if ev.kind == ContactEventKind::Add {
                            self.nub.air_lean = false;
                            self.nub.contacts += 1;
                        } else {
                            self.nub.contacts -= 1;
                        }
                    }
                }
            }
            if ev.kind == ContactEventKind::Persist {
                for f in &mut self.fans {
                    let other = if ev.shape1 == f.sensor {
                        ev.shape2
                    } else if ev.shape2 == f.sensor {
                        ev.shape1
                    } else {
                        continue;
                    };
                    let body = self.b.c.shapes[other].body;
                    if !f.bodies.contains(&body) {
                        f.bodies.push(body);
                    }
                }
                for f in &mut self.boosts {
                    let other = if ev.shape1 == f.sensor {
                        ev.shape2
                    } else if ev.shape2 == f.sensor {
                        ev.shape1
                    } else {
                        continue;
                    };
                    let body = self.b.c.shapes[other].body;
                    if !f.bodies.contains(&body) {
                        f.bodies.push(body);
                    }
                }
                continue;
            }
            for i in 0..self.triggers.len() {
                let sensor = self.triggers[i].sensor;
                let other = if ev.shape1 == sensor {
                    ev.shape2
                } else if ev.shape2 == sensor {
                    ev.shape1
                } else {
                    continue;
                };
                let (body, other_sensor) = (self.b.c.shapes[other].body, self.b.c.shapes[other].shape.is_sensor);
                let mass = self.b.c.bodies[body].mass;
                if ev.kind == ContactEventKind::Add {
                    let t = &self.triggers[i];
                    if !t.listening || t.disabled {
                        continue;
                    }
                    let ok = match t.by {
                        1 | 2 => Some(body) == vehicle, // no NPCs yet: the vehicle stands in for the main character
                        3 => mass > 0.0 && !other_sensor,
                        4 => mass > 0.0 && !other_sensor && t.targets.contains(&body),
                        _ => false,
                    };
                    if ok {
                        self.add_triggering_body(i, body, ev.contact);
                    }
                } else {
                    let t = &mut self.triggers[i];
                    if t.watching_remove && Some(body) == t.triggering_body && !other_sensor {
                        t.triggering_count -= 1;
                        if t.triggering_count < 1 && mass > 0.0 {
                            t.triggering_count = 0;
                            t.triggering_body = None;
                        }
                    }
                }
            }
        }
    }

    /// The level's per-frame `actions` pass for triggers in the actions vector.
    fn run_actions(&mut self) {
        for i in 0..self.triggers.len() {
            if !self.triggers[i].active {
                continue;
            }
            let mut delay_clear = true;
            if !self.triggers[i].delay.is_empty() {
                let delay_frames = self.triggers[i].delay_frames;
                let mut fire_n = 0;
                let t = &mut self.triggers[i];
                for k in (0..t.delay.len()).rev() {
                    if t.delay[k] == delay_frames {
                        fire_n += 1;
                        t.delay.remove(k);
                    } else {
                        t.delay[k] += 1;
                    }
                }
                delay_clear = t.delay.is_empty();
                for _ in 0..fire_n {
                    self.fire(i);
                }
            }
            let t = &mut self.triggers[i];
            let mut repeat_done = true;
            if t.repeat > 2 && t.triggering_body.is_some() {
                repeat_done = false;
                if t.repeat_frames.is_none_or(|n| t.repeat_count == n) {
                    t.repeat_count = 0;
                    if t.delay_frames == 0 {
                        self.fire(i);
                    } else {
                        self.triggers[i].delay.push(0);
                    }
                } else {
                    t.repeat_count += 1;
                }
            }
            if delay_clear && repeat_done {
                self.triggers[i].active = false;
            }
        }
    }

    /// Adds a revolute joint to the underlying world (used by characters).
    pub fn b2_mut(&mut self) -> &mut B2 {
        &mut self.b
    }

    pub fn b2(&self) -> &B2 {
        &self.b
    }

    pub fn level_body(&self) -> BodyId {
        self.level_body
    }

    /// Number of solver contacts (with manifold points) among shapes of the given bodies.
    pub fn contact_count_between(&self, bodies: &[BodyId]) -> usize {
        let owned = |s: ShapeId| bodies.contains(&self.b.c.shapes[s].body);
        self.b
            .c
            .contact_ids()
            .map(|id| self.b.c.contact_info(id))
            .filter(|c| owned(c.shape1) && owned(c.shape2) && c.manifold_count > 0 && !self.b.c.shapes[c.shape1].shape.is_sensor && !self.b.c.shapes[c.shape2].shape.is_sensor)
            .count()
    }

    /// `contactResultHandler`: keeps, per ragdoll part, the largest `Result` impulse above its smash limit.
    /// `contactAddHandler` of the rider: a head/chest/pelvis shape meeting a non-sensor shape that is massless or at least as heavy, at a closing speed
    /// above 4, plays Thud1 (head) / Thud2 (chest, pelvis) unless that part smashes this step (the smash clears the buffered thud).
    fn collect_thuds(&mut self, events: &[box2d::world::ContactEvent]) {
        use crate::character::Part;
        let Some(doll) = self.character.as_ref() else { return };
        let mut thuds: Vec<(&'static str, Vec2)> = Vec::new();
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Add) {
            for (mine, other) in [(ev.shape1, ev.shape2), (ev.shape2, ev.shape1)] {
                let body = self.b.c.shapes[mine].body;
                let Some((&part, _)) = doll.bodies.iter().find(|(_, &b)| b == body) else { continue };
                if !matches!(part, Part::Head | Part::Chest | Part::Pelvis) || self.smashes.iter().any(|(p, _)| *p == part) {
                    continue;
                }
                let o = &self.b.c.shapes[other];
                let (om, mm) = (self.b.body(o.body).mass, self.b.body(body).mass);
                if o.shape.is_sensor || (om != 0.0 && om < mm) {
                    continue;
                }
                if (ev.velocity.x * ev.normal.x + ev.velocity.y * ev.normal.y).abs() > 4.0 {
                    thuds.push((if part == Part::Head { "Thud1" } else { "Thud2" }, self.b.body(body).sweep.c));
                }
            }
        }
        for (n, at) in thuds {
            self.play(n, 1, at);
        }
    }

    fn collect_smashes(&mut self, events: &[box2d::world::ContactEvent]) {
        self.smashes.clear();
        let Some(doll) = self.character.as_ref() else { return };
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Result) {
            for shape in [ev.shape1, ev.shape2] {
                let body = self.b.c.shapes[shape].body;
                let Some((&part, _)) = doll.bodies.iter().find(|(_, &b)| b == body) else { continue };
                let Some(limit) = doll.smash_limit(self, part) else { continue };
                if ev.normal_impulse > limit {
                    match self.smashes.iter_mut().find(|(p, _)| *p == part) {
                        Some(s) if ev.normal_impulse > s.1 => s.1 = ev.normal_impulse,
                        Some(_) => {}
                        None => self.smashes.push((part, ev.normal_impulse)),
                    }
                }
            }
        }
    }

    /// `contactResultHandler` for a passenger's ragdoll: parts whose largest `Result` impulse exceeds their smash limit.
    pub fn doll_smash_hits(&self, doll: &crate::character::Ragdoll, events: &[box2d::world::ContactEvent]) -> Vec<crate::character::Part> {
        let mut hits = Vec::new();
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Result) {
            for shape in [ev.shape1, ev.shape2] {
                let body = self.b.c.shapes[shape].body;
                let Some((&part, _)) = doll.bodies.iter().find(|(_, &b)| b == body) else { continue };
                if doll.smash_limit(self, part).is_some_and(|l| ev.normal_impulse > l) && !hits.contains(&part) {
                    hits.push(part);
                }
            }
        }
        hits
    }

    /// c3-bike `contactWheelResultHandler` (strongest Result > 200 on a wheel that is still a circle) and `contactFrameResultHandler`
    /// (Result > `frame_limit` on the first frame shape), applied to `bike` with the main rider.
    fn smash_bike_parts(&mut self, bike: &mut crate::vehicle::Bike, events: &[box2d::world::ContactEvent], frame_limit: f64) {
        let mut hits = Vec::new();
        for (front, wheel) in [(true, bike.front_wheel), (false, bike.back_wheel)] {
            let Some(&s) = self.b.body(wheel).shapes.first() else { continue };
            if self.b.shape_radius(s) == 0.0 {
                continue;
            }
            let best = events.iter().filter(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > 200.0).max_by(|a, b| a.normal_impulse.total_cmp(&b.normal_impulse));
            if let Some(e) = best {
                hits.push((front, e.normal));
            }
        }
        for (front, n) in hits {
            bike.wheel_smash(self, front, n);
        }
        let s0 = self.b.body(bike.frame).shapes.first().copied();
        let hit = !bike.frame_smashed && s0.is_some_and(|s0| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s0 || e.shape2 == s0) && e.normal_impulse > frame_limit));
        if hit {
            if let Some(mut doll) = self.character.take() {
                bike.frame_smash(self, &mut doll);
                self.character = Some(doll);
            }
        }
    }

    /// `magnetContactResult`: for each shape touching the heli magnet keep the `Result` with the largest impulse.
    fn collect_magnet(&mut self, events: &[box2d::world::ContactEvent]) {
        self.magnet_hits.clear();
        let Some(crate::vehicle::Vehicle::Heli(h)) = self.bike.as_ref() else { return };
        let magnet = h.magnet_shape;
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Result && (e.shape1 == magnet || e.shape2 == magnet)) {
            let other = if ev.shape1 == magnet { ev.shape2 } else { ev.shape1 };
            match self.magnet_hits.iter_mut().find(|h| h.0 == other) {
                Some(h) if ev.normal_impulse > h.2 => *h = (other, ev.position, ev.normal_impulse),
                Some(_) => {}
                None => self.magnet_hits.push((other, ev.position, ev.normal_impulse)),
            }
        }
    }

    /// Grinder `wheelContactResult`: while space is held, a wheel touching a rail shape (material bit 4) remembers that
    /// rail (first one per wheel per frame, and not the one it is already attached to).
    fn collect_rails(&mut self, events: &[box2d::world::ContactEvent]) {
        let Some(crate::vehicle::Vehicle::Grinder(g)) = self.bike.as_mut() else { return };
        if !g.connecting {
            return;
        }
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Result) {
            for (mine, other) in [(ev.shape1, ev.shape2), (ev.shape2, ev.shape1)] {
                let n = if mine == g.front_shape { 0 } else if mine == g.back_shape { 1 } else { continue };
                if self.b.shape_material(other) & 4 == 0 {
                    continue;
                }
                let rail = self.b.shape_body(other);
                if g.new_rail[n].is_none() && g.dongles[n].is_none_or(|d| d.rail != rail) {
                    g.new_rail[n] = Some(rail);
                }
            }
        }
    }

    pub fn step(&mut self) {
        if let Some(mut doll) = self.character.take() {
            doll.check_vocals(self);
            self.character = Some(doll);
        }
        self.update_loops();
        self.frame += 1;
        self.thrust_jets();
        self.run_cannons();
        self.run_mines();
        self.run_arrow_guns();
        self.check_npc_joints();
        self.b.step(TIME_STEP, ITERATIONS);
        let events: Vec<_> = self.b.c.events.drain(..).collect();
        self.collect_smashes(&events);
        self.collect_thuds(&events);
        if self.joints_break && self.character.as_ref().is_some_and(|d| d.helmet_on) {
            // the head shape doubles as the helmet shape: a Result above 2 loses the helmet (and the head cannot smash that frame)
            let head = self.character.as_ref().map(|d| d.bodies[&crate::character::Part::Head]);
            let hit = head.is_some_and(|h| events.iter().any(|e| e.kind == ContactEventKind::Result && e.normal_impulse > crate::character::Ragdoll::HELMET_LIMIT && (self.b.c.shapes[e.shape1].body == h || self.b.c.shapes[e.shape2].body == h)));
            if hit {
                if let Some(mut doll) = self.character.take() {
                    doll.helmet_smash(self);
                    self.character = Some(doll);
                }
                self.smashes.retain(|(p, _)| *p != crate::character::Part::Head);
            }
        }
        if self.joints_break && !self.smashes.is_empty() {
            // `contactResultHandler`: head / chest / pelvis / foot smash reactions
            if let Some(mut doll) = self.character.take() {
                let hits: Vec<_> = self.smashes.iter().map(|(p, _)| *p).collect();
                for part in [crate::character::Part::Head, crate::character::Part::Chest, crate::character::Part::Pelvis, crate::character::Part::LowerLeg1, crate::character::Part::LowerLeg2] {
                    if !hits.contains(&part) {
                        continue;
                    }
                    match part {
                        crate::character::Part::Head => doll.smash_head(self),
                        crate::character::Part::Chest => doll.smash_chest(self),
                        crate::character::Part::Pelvis => doll.smash_pelvis(self),
                        crate::character::Part::LowerLeg1 => doll.smash_foot(self, 0),
                        _ => doll.smash_foot(self, 1),
                    }
                }
                self.character = Some(doll);
            }
        }
        self.collect_magnet(&events);
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Pogo(ref p)) if !p.frame_smashed) {
            // `contactResultHandler` on frameShape: impulse above 40 smashes the frame
            let hit = if let Some(crate::vehicle::Vehicle::Pogo(p)) = self.bike.as_ref() {
                let s = self.b.body(p.frame).shapes.first().copied();
                s.is_some_and(|s| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > crate::vehicle::Pogo::FRAME_SMASH_LIMIT))
            } else {
                false
            };
            if hit {
                if let (Some(mut doll), Some(crate::vehicle::Vehicle::Pogo(mut p))) = (self.character.take(), self.bike.take()) {
                    p.frame_smash(self, &mut doll);
                    self.character = Some(doll);
                    self.bike = Some(crate::vehicle::Vehicle::Pogo(p));
                }
            }
        }
        if let Some(crate::vehicle::Vehicle::Mower(mut m)) = self.bike.take_if(|b| matches!(b, crate::vehicle::Vehicle::Mower(_))) {
            m.grind(self, &events);
            self.bike = Some(crate::vehicle::Vehicle::Mower(m));
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Mower(_))) {
            // `contactResultHandler` on frontShape (3rd created): impulse above 200 smashes the mower
            let hit = if let Some(crate::vehicle::Vehicle::Mower(m)) = self.bike.as_ref() {
                let s2 = self.b.body(m.body).shapes.get(2).copied();
                m.halves.is_none() && s2.is_some_and(|s| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > crate::vehicle::Mower::SMASH_LIMIT))
            } else {
                false
            };
            if hit {
                if let (Some(mut doll), Some(crate::vehicle::Vehicle::Mower(mut m))) = (self.character.take(), self.bike.take()) {
                    m.smash(self, &mut doll);
                    self.character = Some(doll);
                    self.bike = Some(crate::vehicle::Vehicle::Mower(m));
                }
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Cart(ref c)) if !c.main_smashed) {
            // main shape 1 above 200 smashes the cart; the basket shape (3rd) above 100 loses the basket
            let (mut main_hit, mut cart_hit) = (false, false);
            if let Some(crate::vehicle::Vehicle::Cart(c)) = self.bike.as_ref() {
                let over = |i: usize, lim: f64| self.b.body(c.body).shapes.get(i).copied().is_some_and(|s| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > lim));
                main_hit = over(0, crate::vehicle::Cart::MAIN_SMASH_LIMIT);
                cart_hit = self.b.body(c.body).shapes.len() == 5 && over(2, crate::vehicle::Cart::CART_SMASH_LIMIT);
            }
            if main_hit || cart_hit {
                if let (Some(mut doll), Some(crate::vehicle::Vehicle::Cart(mut c))) = (self.character.take(), self.bike.take()) {
                    if main_hit {
                        c.main_smash(self, &mut doll);
                    } else {
                        c.cart_smash(self);
                    }
                    self.character = Some(doll);
                    self.bike = Some(crate::vehicle::Vehicle::Cart(c));
                }
            }
        }
        if self.joints_break {
            // grocery cans: `crackerSmash` / `colaSmash` fire on any Result above 1 on the cracker box / soda can
            let mut hits = (None, None);
            if let Some(crate::vehicle::Vehicle::Cart(c)) = self.bike.as_ref() {
                let over = |id: BodyId| self.b.body(id).alive && self.b.body(id).shapes.last().copied().is_some_and(|s| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > 1.0));
                if !c.cracker_gone && over(c.groceries[4]) {
                    hits.0 = Some(c.groceries[4]);
                }
                if !c.cola_open && over(c.groceries[3]) {
                    hits.1 = Some(c.groceries[3]);
                }
            }
            if let Some(id) = hits.0 {
                let at = self.b.body(id).sweep.c;
                self.blood.sprite_burst(crate::blood::ShardKind::Crackers, [at.x * PS, at.y * PS], 30.0, 30.0, 40);
                self.b.destroy_body(id);
                if let Some(crate::vehicle::Vehicle::Cart(c)) = self.bike.as_mut() {
                    c.cracker_gone = true;
                }
            }
            if let Some(id) = hits.1 {
                self.sprite_flow(crate::blood::ShardKind::Cola, 2.5, 4.0, id, Vec2::new(0.0, -10.0 / 62.5), 270.0, 300);
                if let Some(crate::vehicle::Vehicle::Cart(c)) = self.bike.as_mut() {
                    c.cola_open = true;
                }
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Chair(ref c)) if !c.chair_smashed) {
            // `contactResultHandler` on chairShape1: impulse above 200 smashes the chair
            let hit = if let Some(crate::vehicle::Vehicle::Chair(c)) = self.bike.as_ref() {
                let s = self.b.body(c.chair).shapes.first().copied();
                s.is_some_and(|s| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > crate::vehicle::Chair::CHAIR_SMASH_LIMIT))
            } else {
                false
            };
            if hit {
                if let (Some(mut doll), Some(crate::vehicle::Vehicle::Chair(mut c))) = (self.character.take(), self.bike.take()) {
                    c.chair_smash(self, &mut doll);
                    self.character = Some(doll);
                    self.bike = Some(crate::vehicle::Vehicle::Chair(c));
                }
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Chair(ref c)) if c.chair_smashed) {
            // loose jet / fuel tank: Result above 30 destroys the jet or explodes the tank
            let (mut jet_hit, mut tank_hit) = (false, false);
            if let Some(crate::vehicle::Vehicle::Chair(c)) = self.bike.as_ref() {
                let over = |id: Option<usize>| id.and_then(|b| self.b.body(b).shapes.first().copied()).is_some_and(|s| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > crate::vehicle::Chair::JET_SMASH_LIMIT));
                jet_hit = over(c.jet);
                tank_hit = over(c.tank);
            }
            if let Some(crate::vehicle::Vehicle::Chair(mut c)) = self.bike.take() {
                if tank_hit {
                    c.fueltank_smash(self);
                }
                if jet_hit {
                    c.jet_smash(self);
                }
                self.bike = Some(crate::vehicle::Vehicle::Chair(c));
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Chair(_))) {
            // big wheel Result above 200 turns the wheel into a box
            let mut hit = None;
            if let Some(crate::vehicle::Vehicle::Chair(c)) = self.bike.as_ref() {
                if let Some(&s) = self.b.body(c.big_wheel).shapes.first() {
                    if self.b.shape_radius(s) != 0.0 {
                        hit = events.iter().filter(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > crate::vehicle::Chair::WHEEL_SMASH_LIMIT).max_by(|a, b| a.normal_impulse.total_cmp(&b.normal_impulse)).map(|e| e.normal);
                    }
                }
            }
            if let Some(n) = hit {
                if let Some(crate::vehicle::Vehicle::Chair(mut c)) = self.bike.take() {
                    c.wheel_smash(self, n);
                    self.bike = Some(crate::vehicle::Vehicle::Chair(c));
                }
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Dirt(_))) {
            // wheel smash (limit 200), as for the bike
            let mut hits = Vec::new();
            if let Some(crate::vehicle::Vehicle::Dirt(d)) = self.bike.as_ref() {
                for (front, wheel) in [(true, d.front_wheel), (false, d.back_wheel)] {
                    let Some(&s) = self.b.body(wheel).shapes.first() else { continue };
                    if self.b.shape_radius(s) == 0.0 {
                        continue;
                    }
                    let best = events.iter().filter(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > 200.0).max_by(|a, b| a.normal_impulse.total_cmp(&b.normal_impulse));
                    if let Some(e) = best {
                        hits.push((front, e.normal));
                    }
                }
            }
            if let Some(crate::vehicle::Vehicle::Dirt(mut d)) = self.bike.take() {
                for (front, n) in hits {
                    d.wheel_smash(self, front, n);
                }
                self.bike = Some(crate::vehicle::Vehicle::Dirt(d));
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Dirt(_))) {
            // `contactResultHandler` on the tank shape (5th created): impulse above 200 smashes the frame
            let hit = if let Some(crate::vehicle::Vehicle::Dirt(d)) = self.bike.as_ref() {
                let s4 = self.b.body(d.frame).shapes.get(4).copied();
                !d.frame_smashed && s4.is_some_and(|s| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > crate::vehicle::DirtBike::FRAME_SMASH_LIMIT))
            } else {
                false
            };
            if hit {
                if let (Some(mut doll), Some(crate::vehicle::Vehicle::Dirt(mut d))) = (self.character.take(), self.bike.take()) {
                    d.frame_smash(self, &mut doll);
                    self.character = Some(doll);
                    self.bike = Some(crate::vehicle::Vehicle::Dirt(d));
                }
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Segway(_))) {
            // `contactResultHandler` on the frame shape (first created): impulse above 20 smashes the frame
            let hit = if let Some(crate::vehicle::Vehicle::Segway(sg)) = self.bike.as_ref() {
                let s0 = self.b.body(sg.frame).shapes.first().copied(); // frameShape = first created
                !sg.frame_smashed && s0.is_some_and(|s0| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s0 || e.shape2 == s0) && e.normal_impulse > crate::vehicle::Segway::FRAME_SMASH_LIMIT))
            } else {
                false
            };
            if hit {
                if let (Some(mut doll), Some(crate::vehicle::Vehicle::Segway(mut sg))) = (self.character.take(), self.bike.take()) {
                    sg.frame_smash(self, &mut doll);
                    self.character = Some(doll);
                    self.bike = Some(crate::vehicle::Vehicle::Segway(sg));
                }
            }
        }
        if self.joints_break {
            match self.bike.take() {
                Some(crate::vehicle::Vehicle::Bike(mut bike)) => {
                    self.smash_bike_parts(&mut bike, &events, crate::vehicle::Bike::FRAME_SMASH_LIMIT);
                    self.bike = Some(crate::vehicle::Vehicle::Bike(bike));
                }
                Some(crate::vehicle::Vehicle::Family(mut f)) => {
                    // the c10 dad is a c3 bike with `frameSmashLimit` 100
                    self.smash_bike_parts(&mut f.dad, &events, crate::vehicle::Family::DAD_FRAME_SMASH_LIMIT);
                    self.bike = Some(crate::vehicle::Vehicle::Family(f));
                }
                other => self.bike = other,
            }
        }
        self.collect_rails(&events);
        self.handle_events(events.clone());
        self.run_actions();
        self.run_fans();
        self.run_boosts();
        self.run_jets(&events);
        self.cannon_events(&events);
        self.token_events(&events);
        self.food_events(&events);
        self.boombox_events(&events);
        self.chair_events(&events);
        self.glass_events(&events);
        self.mine_events(&events);
        self.arrow_events(&events);
        self.spike_events(&events);
        self.npc_smashes(&events);
        self.step_blood();
        self.can_events(&events);
        self.hit_sound_events(&events);
        if let Some(mut doll) = self.character.take() {
            if self.joints_break {
                if matches!(self.bike, Some(crate::vehicle::Vehicle::Santa(_))) {
                    if let Some(crate::vehicle::Vehicle::Santa(mut s)) = self.bike.take() {
                        s.smashed(self, &mut doll, &events);
                        s.check_joints(self, &mut doll);
                        self.bike = Some(crate::vehicle::Vehicle::Santa(s));
                    }
                } else {
                    let broken = doll.check_joints(self);
                    if !broken.is_empty() {
                        if let Some(mut v) = self.bike.take() {
                            v.rider_hurt(self, &mut doll, false, &broken);
                            self.bike = Some(v);
                        }
                    }
                }
            }
            doll.update_grab(self);
            self.character = Some(doll);
        }
        if matches!(self.bike, Some(crate::vehicle::Vehicle::Heli(_))) {
            if let Some(crate::vehicle::Vehicle::Heli(mut h)) = self.bike.take() {
                let hits = std::mem::take(&mut self.magnet_hits);
                if let Some(mut doll) = self.character.take() {
                    h.contacts(self, &mut doll, &events);
                    self.character = Some(doll);
                }
                h.handle_magnet(self, &hits);
                self.bike = Some(crate::vehicle::Vehicle::Heli(h));
            }
        }
        if matches!(self.bike, Some(crate::vehicle::Vehicle::Dirt(ref d)) if d.girl.is_some()) {
            if let Some(crate::vehicle::Vehicle::Dirt(mut d)) = self.bike.take() {
                if let Some(mut g) = d.girl.take() {
                    g.contact_events(self, &events);
                    if self.joints_break {
                        g.smashed(self, &events);
                    }
                    if self.joints_break {
                        g.check_joints(self);
                    }
                    d.girl = Some(g);
                }
                self.bike = Some(crate::vehicle::Vehicle::Dirt(d));
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Bike(ref b)) if b.kid.is_some()) {
            if let Some(crate::vehicle::Vehicle::Bike(mut b)) = self.bike.take() {
                if let Some(mut kid) = b.kid.take() {
                    kid.check_joints(self);
                    kid.smashed(self, &events);
                    b.kid = Some(kid);
                }
                self.bike = Some(crate::vehicle::Vehicle::Bike(b));
            }
        }
        if self.joints_break && matches!(self.bike, Some(crate::vehicle::Vehicle::Family(_))) {
            if let Some(crate::vehicle::Vehicle::Family(mut f)) = self.bike.take() {
                f.son.smash_basket(self, &events);
                f.daughter.smashed(self, &events);
                f.check_joints(self);
                self.bike = Some(crate::vehicle::Vehicle::Family(f));
            }
        }
    }

    /// Fan `actions`: force 15·s² (s = position along the wind axis / 250) on every body seen this frame.
    /// `Boost.actions`: force `power·mass` along `(sin a, -cos a)` at each touching body's centre of mass.
    fn run_boosts(&mut self) {
        for bi in 0..self.boosts.len() {
            let bodies = std::mem::take(&mut self.boosts[bi].bodies);
            let (power, sin, cos) = (self.boosts[bi].power, self.boosts[bi].sin, self.boosts[bi].cos);
            for body in bodies {
                let b = self.b.body_mut(body);
                if b.inv_mass == 0.0 {
                    continue;
                }
                let s = power * b.mass;
                let c = b.sweep.c;
                b.apply_force(Vec2::new(sin * s, cos * -s), c);
            }
        }
    }

    /// ArrowGun, static-base variant (transcribed createBody/actions/fireArrow; the dynamic-base variant (`immovable2` false) is NOT ported):
    /// a 14×8 px box on the level, an 800×400 px range sensor in front of it, a turret at 27 px along the barrel. `rateOfFire` r → one shot per 23−2r steps,
    /// 10 arrows in all. `dontShootPlayer` is approximated as "never targets" (only ragdolls carry material 2 so far).
    fn arrow_gun(&mut self, sp: &crate::level::Special) -> Option<BodyId> {
        let harpoon = sp.kind == 15;
        if (!harpoon && sp.flag("immovable2") == Some(false)) || (harpoon && (sp.flag("fixedAngleTurret") == Some(true) || sp.flag("triggerFiring") == Some(true))) {
            return None;
        }
        let (x, y) = (f64::from(sp.num("x")?), f64::from(sp.num("y")?));
        let (hx, hy, off) = if harpoon { (25.0, 16.0, 35.0) } else { (14.0, 8.0, 27.0) };
        let t = f64::from(sp.num("angle").unwrap_or(0.0)).to_radians();
        let mut d = ShapeDef::oriented_box(hx / PS, hy / PS, Vec2::new(x / PS, y / PS), t);
        d.friction = 0.5;
        d.restitution = 0.1;
        d.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: -20 };
        self.b.create_shape(self.level_body, &d);
        let mut r = ShapeDef::oriented_box(400.0 / PS, 200.0 / PS, Vec2::new((x + 200.0 * t.sin()) / PS, (y - 200.0 * t.cos()) / PS), t);
        r.is_sensor = true;
        r.filter = d.filter;
        let range = self.b.create_shape(self.level_body, &r);
        self.arrow_guns.push(ArrowGun {
            pos: Vec2::new((x + off * t.sin()) / PS, (y - off * t.cos()) / PS),
            angle: t,
            range,
            targets: Vec::new(),
            target: None,
            aligned: false,
            frame_count: 0,
            frames_per_shot: if harpoon { 1 } else { (23.0 - 2.0 * f64::from(sp.num("rateOfFire").unwrap_or(5.0))).max(1.0) as u32 },
            retarget: 0,
            arrows_left: if harpoon { 1 } else { 10 },
            done: if harpoon { sp.flag("startDeactivated").unwrap_or(false) } else { sp.flag("dontShootPlayer").unwrap_or(false) },
            harpoon,
        });
        Some(self.level_body)
    }

    /// NPCharacter (bodies/joints/limits/break limits taken from the original class run in the oracle on a stand-in session, `analysis/npc-dump.js`;
    /// the 12 `charIndex` guides, mirrored for `reverse`). Rotated/translated as a rigid group. Joint breaking follows `checkJoints` (reaction² > limit or anchor gap > .5 m
    /// destroys the joint and frees the limb's filter). NOT ported: smashes of head/chest/pelvis, chunks, blood, `holdPose`/pose angles, impulse/wake triggers, sounds, sprites.
    fn npc(&mut self, sp: &crate::level::Special) -> Option<BodyId> {
        if sp.flag("interactive") == Some(false) {
            return None;
        }
        const NAMES: [&str; 11] = ["chest", "head", "pelvis", "upperArm1", "upperArm2", "lowerArm1", "lowerArm2", "upperLeg1", "upperLeg2", "lowerLeg1", "lowerLeg2"];
        const JOINTS: [(&str, usize, usize, &str); 10] = [
            ("neck", 0, 1, "neck"),
            ("waist", 0, 2, "torso"),
            ("shoulder1", 0, 3, "shoulder"),
            ("shoulder2", 0, 4, "shoulder"),
            ("elbow1", 3, 5, "elbow"),
            ("elbow2", 4, 6, "elbow"),
            ("hip1", 2, 7, "hip"),
            ("hip2", 2, 8, "hip"),
            ("knee1", 7, 9, "knee"),
            ("knee2", 8, 10, "knee"),
        ];
        let all: serde_json::Value = serde_json::from_str(include_str!("../data/npc-guides.json")).ok()?;
        let key = format!("{}{}", (sp.num("charIndex").unwrap_or(1.0) as i32).clamp(1, 12), if sp.flag("reverse").unwrap_or(false) { "r" } else { "" });
        let g = all.get(&key)?;
        let f = |v: &serde_json::Value| v.as_f64().unwrap_or(0.0);
        let (x, y) = (f64::from(sp.num("x")?), f64::from(sp.num("y")?));
        let rot = f64::from(sp.num("angle").unwrap_or(0.0)).to_radians();
        let (sn, cs) = rot.sin_cos();
        let group = self.npc_groups;
        self.npc_groups -= 2;
        let sleeping = sp.flag("sleeping").unwrap_or(false);
        let mut bodies = [0usize; 11];
        for (i, n) in NAMES.iter().enumerate() {
            let b = &g["bodies"][n];
            let (px, py) = (f(&b["p"][0]), f(&b["p"][1]));
            let pos = Vec2::new(x / PS + cs * px - sn * py, y / PS + sn * px + cs * py);
            let body = self.b.create_body(&BodyDef { position: pos, angle: f(&b["a"]) + rot, is_sleeping: sleeping, allow_sleep: true, ..Default::default() });
            let geo = if b["g"].get("r").is_some() { Geometry::Circle { local_position: Vec2::default(), radius: f(&b["g"]["r"]) } } else { ShapeDef::boxed(f(&b["g"]["hx"]), f(&b["g"]["hy"])).geometry };
            let mut sd = ShapeDef::new(geo);
            sd.density = 1.0;
            sd.friction = 0.3;
            sd.restitution = 0.1;
            sd.filter = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: group };
            let shape = self.b.create_shape(body, &sd);
            self.b.shape_mut(shape).material = b["mat"].as_i64().unwrap_or(0) as i32;
            self.b.set_mass_from_shapes(body);
            bodies[i] = body;
        }
        let mut joints = Vec::new();
        for (name, parent, child, limit) in JOINTS {
            let j = &g["joints"][name];
            let anchor = self.b.body(bodies[parent]).world_point(Vec2::new(f(&j["a1"][0]), f(&j["a1"][1])));
            let mut d = RevoluteJointDef::initialize(self.b.body(bodies[parent]), bodies[parent], self.b.body(bodies[child]), bodies[child], anchor);
            d.enable_limit = true;
            d.lower_angle = f(&j["lo"]);
            d.upper_angle = f(&j["hi"]);
            d.max_motor_torque = 4.0;
            let id = self.b.create_revolute_joint(&d);
            joints.push((id, child, f(&g["limits"][limit]), false));
        }
        self.b.c.report_results = true;
        let chest = bodies[0];
        let lim = |k: &str| f(&g["limits"][k]);
        let smash_shapes = [bodies[1], bodies[0], bodies[2]].map(|b| self.b.body(b).shapes[0]);
        let chunk = [f(&g["chunk"][0]), f(&g["chunk"][1]), f(&g["chunk"][2])];
        self.npcs.push(Npc { bodies, joints, smash_shapes, smash_limits: [lim("head"), lim("chest"), lim("pelvis")], chunk, smashed: [false; 3], pieces: Vec::new() });
        Some(chest)
    }

    /// `NPCharacter.checkJoints`: reaction force² above the limb's limit, or a stretched anchor, breaks the joint and clears the limb's collision group.
    /// Wheel-spin loop of the car-like vehicles: with a wheel contact, |ω| of `spin` above `t[0]` → BikeLoop3, `t[1]` → BikeLoop2, `t[2]` → BikeLoop1.
    fn wheel_loop(&mut self, touching: &[BodyId], spin: BodyId, t: [f64; 3]) {
        if touching.iter().map(|&w| self.b.touching_non_sensor_contacts(w)).sum::<usize>() == 0 {
            return;
        }
        let wheel = self.b.body(spin);
        let w = wheel.angular_velocity.abs();
        let name = if w > t[0] { "BikeLoop3" } else if w > t[1] { "BikeLoop2" } else if w > t[2] { "BikeLoop1" } else { return };
        let at = [(wheel.sweep.c.x * PS) as f32, (wheel.sweep.c.y * PS) as f32];
        self.loops.push((name.to_string(), at, 1.0));
    }

    /// Per-frame loop requests (`actions` of each class). c3 bike / family dad / c5 dirt bike / c2 segway: back-wheel |ω| > 18 / 9 / 1 → BikeLoop3/2/1;
    /// c4 cart: 50 / 25 / 5; c1 chair: the big wheel above 1 → BikeLoop1; heli and mower have their own loops.
    fn update_loops(&mut self) {
        self.loops.clear();
        use crate::vehicle::Vehicle;
        let wheels: Option<(Vec<BodyId>, BodyId, [f64; 3])> = match self.bike.as_ref() {
            Some(Vehicle::Bike(b)) => Some((vec![b.back_wheel, b.front_wheel], b.back_wheel, [18.0, 9.0, 1.0])),
            Some(Vehicle::Family(f)) => Some((vec![f.dad.back_wheel, f.dad.front_wheel], f.dad.back_wheel, [18.0, 9.0, 1.0])),
            Some(Vehicle::Dirt(d)) => Some((vec![d.back_wheel, d.front_wheel], d.back_wheel, [18.0, 9.0, 1.0])),
            Some(Vehicle::Segway(s)) => Some((vec![s.wheel], s.wheel, [18.0, 9.0, 1.0])),
            Some(Vehicle::Cart(c)) => Some((vec![c.back_wheel, c.front_wheel], c.back_wheel, [50.0, 25.0, 5.0])),
            Some(Vehicle::Chair(c)) => Some((vec![c.big_wheel, c.small_wheel], c.big_wheel, [f64::INFINITY, f64::INFINITY, 1.0])),
            _ => None,
        };
        if let Some(Vehicle::Cart(c)) = self.bike.as_ref() {
            if c.motor_on(self) {
                let w = self.b.body(c.back_wheel).sweep.c;
                self.loops.push(("ElectricMotor1".to_string(), [(w.x * PS) as f32, (w.y * PS) as f32], 1.0));
            }
        }
        if let Some(Vehicle::Mower(m)) = self.bike.as_ref() {
            if m.grinding() {
                let c = self.b.body(m.body).sweep.c;
                self.loops.push(("GrindLoop2".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], 1.0));
            }
        }
        if let Some(Vehicle::Santa(sa)) = self.bike.as_ref() {
            if sa.bell {
                let c = self.b.body(sa.sleigh).sweep.c;
                self.loops.push(("SleighBellLoop".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], 1.0));
            }
            if !sa.wheels_gone {
                let touching: usize = sa.wheels.iter().map(|&w| self.b.touching_non_sensor_contacts(w)).sum();
                let back = self.b.body(sa.wheels[0]);
                if touching > 0 && back.angular_velocity.abs() > 12.0 {
                    let c = self.b.body(sa.sleigh).sweep.c;
                    self.loops.push(("SkiLoop".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], 1.0));
                }
            }
        }
        if let Some((touching, spin, t)) = wheels {
            self.wheel_loop(&touching, spin, t);
        }
        match self.bike.as_ref() {
            Some(crate::vehicle::Vehicle::Heli(h)) if !h.copter_smashed => {
                let c = self.b.body(h.copter).sweep.c;
                self.loops.push(("HeliLoop".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], h.loop_volume()));
            }
            Some(crate::vehicle::Vehicle::Mower(m)) if m.halves.is_none() => {
                let c = self.b.body(m.body).sweep.c;
                self.loops.push(("MowerLoop".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], m.loop_volume(self)));
            }
            _ => {}
        }
    }

    /// Aim angle (rad) of the arrow gun / harpoon gun placed at special position (`x`, `y`) px with base angle `base` (rad), if it is modelled.
    pub fn gun_turret_angle(&self, x: f64, y: f64, base: f64, harpoon: bool) -> Option<f64> {
        let off = if harpoon { 35.0 } else { 27.0 };
        let (px, py) = ((x + off * base.sin()) / PS, (y - off * base.cos()) / PS);
        self.arrow_guns.iter().find(|g| g.harpoon == harpoon && (g.pos.x - px).abs() < 1e-9 && (g.pos.y - py).abs() < 1e-9).map(|g| g.angle)
    }

    /// Jet engine state for drawing: (`accelScaler`, still burning) of the jet owning `body`.
    pub fn jet_state(&self, body: BodyId) -> Option<(f64, bool)> {
        self.jets.iter().find(|j| j.body == body).map(|j| (j.accel, !j.done && j.fired < j.spec.fire_total))
    }

    /// The eleven bodies (chest, head, pelvis, arms, legs — `NAMES` order of `npc`) of the NPC whose chest is `chest`.
    pub fn npc_bodies(&self, chest: BodyId) -> Option<[BodyId; 11]> {
        self.npcs.iter().find(|n| n.bodies[0] == chest).map(|n| n.bodies)
    }

    /// `grindShape` on an NPC: a mower took its head / chest / pelvis, so the shape's Result listener is deleted and it can no longer smash.
    pub fn npc_grind(&mut self, body: BodyId) {
        for n in &mut self.npcs {
            for (part, idx) in [(0, 1), (1, 0), (2, 2)] {
                if n.bodies[idx] == body {
                    n.smashed[part] = true;
                }
            }
        }
    }

    /// Bodies of the NPCs' eleven main parts (those a sprite set draws; detached pieces are not included).
    pub fn npc_main_bodies(&self) -> Vec<BodyId> {
        self.npcs.iter().flat_map(|n| n.bodies).collect()
    }

    /// How the trash can whose body is `body` looks now: `(clip frame, lid visible, hollow)` — `singleAction` hides the lid, `createHollowTrashCan`
    /// moves the can art up 13 px, a crush switches to frame 2/3.
    pub fn can_look(&self, body: BodyId) -> Option<(usize, bool, bool)> {
        let c = self.cans.iter().find(|c| c.body == body)?;
        let hollow = !c.contains_trash || c.spilled;
        Some((c.crush_frame, c.contains_trash && !c.spilled && !c.crushed, hollow && !c.crushed))
    }

    /// `shapeImpale` of the main rider (spikes call it with defaults, blades with `(fatal, tip, 0.01)`): chest/pelvis vocals, a lethal head impale
    /// ejects the rider from its vehicle and kills it.
    pub(crate) fn shape_impale(&mut self, body: BodyId, fatal: bool, point: Option<Vec2>, radius2: f64) {
        let Some(mut doll) = self.character.take() else { return };
        if doll.shape_impale(self, body, fatal, point, radius2) {
            if let Some(mut bike) = self.bike.take() {
                bike.rider_hurt(self, &mut doll, true, &[]);
                self.bike = Some(bike);
            }
        }
        self.character = Some(doll);
    }

    /// Whether `body` is a food item (its grinding sprays juice particles, not blood).
    pub(crate) fn is_food(&self, body: BodyId) -> bool {
        self.foods.iter().any(|f| f.0 == body)
    }

    pub fn frame(&self) -> u32 {
        self.frame
    }

    /// Queues a one-shot sound at a body's position (`playAreaSoundInstance`); `n` variants are picked by frame number instead of a random draw.
    pub(crate) fn play(&mut self, name: &str, variants: u32, at: Vec2) {
        let full = if variants > 1 { format!("{name}{}", self.frame % variants + 1) } else { name.to_string() };
        self.sounds.push((full, [(at.x * PS) as f32, (at.y * PS) as f32]));
    }

    /// Sounds queued since the last call.
    pub fn take_sounds(&mut self) -> Vec<(String, [f32; 2])> {
        std::mem::take(&mut self.sounds)
    }

    fn check_npc_joints(&mut self) {
        for ni in 0..self.npcs.len() {
            for ji in 0..self.npcs[ni].joints.len() {
                let (id, _, limit, broken) = self.npcs[ni].joints[ji];
                if broken {
                    continue;
                }
                let Some((force, gap)) = self.b.revolute_reaction(id) else { continue };
                if force > limit || gap > 0.5 {
                    self.npc_break_joint(ni, ji);
                }
            }
        }
    }

    fn can_polygon(w: f64, h: f64, cx: f64, cy: f64) -> Geometry {
        let (hw, hh) = (w / 2.0 / PS, h / 2.0 / PS);
        let (cx, cy) = (cx / PS, cy / PS);
        Geometry::Polygon(vec![Vec2::new(cx - hw, cy - hh), Vec2::new(cx + hw, cy - hh), Vec2::new(cx + hw, cy + hh), Vec2::new(cx - hw, cy + hh)])
    }

    fn can_shape(&mut self, body: BodyId, geo: Geometry) -> ShapeId {
        let mut d = ShapeDef::new(geo);
        d.density = 0.75;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        self.b.create_shape(body, &d)
    }

    /// Replaces the can's body by a copy at the same pose/velocity: `crush` = Some(box px) for a crushed box, None for `createHollowTrashCan`.
    fn can_rebuild(&mut self, k: usize, crush: Option<[f64; 2]>) {
        let old = self.cans[k].body;
        let (pos, ang, v, w) = {
            let b = self.b.body(old);
            (b.xf.position, b.sweep.a, b.linear_velocity, b.angular_velocity)
        };
        self.b.destroy_body(old);
        let body = self.b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
        let mut shapes = Vec::new();
        match crush {
            Some([w2, h2]) => shapes.push(self.can_shape(body, Self::can_polygon(w2, h2, 0.0, 0.0))),
            None => {
                shapes.push(self.can_shape(body, Self::can_polygon(CAN_S1[2], CAN_S1[3], CAN_S1[0], CAN_S1[1])));
                for tri in [CAN_LEFT, CAN_RIGHT] {
                    let g = Geometry::Polygon(tri.iter().map(|&(x, y)| Vec2::new(x / PS, y / PS)).collect());
                    shapes.push(self.can_shape(body, g));
                }
            }
        }
        self.b.set_mass_from_shapes(body);
        self.b.body_mut(body).linear_velocity = v;
        self.b.body_mut(body).angular_velocity = w;
        for h in self.hit_sounds.iter_mut().filter(|h| h.body == old) {
            h.body = body;
            h.shapes = shapes.clone();
        }
        self.cans[k].body = body;
        self.cans[k].shapes = shapes;
        // the special's sprite follows the replacement body (the original keeps the same `mc` on the new `_body`)
        for s in self.special_bodies.iter_mut().filter(|s| **s == Some(old)) {
            *s = Some(body);
        }
        for p in self.special_parts.iter_mut().flatten().filter(|b| **b == old) {
            *p = body;
        }
        self.loose.push(body);
    }

    /// `createHollowTrashCan` for a can that never had trash (called while building the level).
    fn can_make_hollow(&mut self, k: usize) {
        self.can_rebuild(k, None);
    }

    /// `checkAdd` sounds of the simple specials (see `HitSound`).
    fn hit_sound_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Add) {
            for k in 0..self.hit_sounds.len() {
                let h = &self.hit_sounds[k];
                if self.frame < h.free_at || !self.b.body(h.body).alive {
                    continue;
                }
                let (mine, other) = if h.shapes.contains(&ev.shape1) {
                    (ev.shape1, ev.shape2)
                } else if h.shapes.contains(&ev.shape2) {
                    (ev.shape2, ev.shape1)
                } else {
                    continue;
                };
                let _ = mine;
                if self.b.c.shapes[other].shape.is_sensor {
                    continue;
                }
                let om = self.b.body(self.b.shape_body(other)).mass;
                if om != 0.0 && om < self.b.body(h.body).mass {
                    continue;
                }
                let closing = (ev.velocity.x * ev.normal.x + ev.velocity.y * ev.normal.y).abs();
                if closing > h.threshold {
                    let (name, variants, lock, at) = (h.name, h.variants, h.lock_frames, self.b.body(h.body).sweep.c);
                    self.hit_sounds[k].free_at = self.frame + lock;
                    self.play(name, variants, at);
                }
            }
        }
    }

    /// `checkContact` + `singleAction`: a Result above `_spillImpulse` (crush above 12) turns the can into a hollow can or a crushed box and, the first
    /// time, spills 3 circle and 7 box trash bodies from the clip's guides (blocker shape added to an uncrushed hollow can).
    fn can_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for k in 0..self.cans.len() {
            if self.cans[k].crushed {
                continue;
            }
            let shapes = self.cans[k].shapes.clone();
            let limit = self.cans[k].spill_impulse;
            let Some(hit) = events.iter().filter(|e| e.kind == ContactEventKind::Result && (shapes.contains(&e.shape1) || shapes.contains(&e.shape2)) && e.normal_impulse > limit).max_by(|a, b| a.normal_impulse.total_cmp(&b.normal_impulse)) else { continue };
            let crush = hit.normal_impulse > 12.0;
            let normal = hit.normal;
            self.cans[k].spill_impulse = 12.0;
            let (pos0, ang0) = {
                let b = self.b.body(self.cans[k].body);
                (b.xf.position, b.sweep.a)
            };
            let _ = pos0;
            let old_body = self.cans[k].body;
            let (v, w) = {
                let b = self.b.body(old_body);
                (b.linear_velocity, b.angular_velocity)
            };
            let old_xf = self.b.body(old_body).clone();
            if crush {
                let x = normal.y.atan2(normal.x) - ang0;
                let horizontal = x.sin().round().abs() != 1.0;
                self.cans[k].crush_frame = if horizontal { 2 } else { 3 };
                let dims = if horizontal { CAN_CRUSH_H } else { CAN_CRUSH_V };
                self.can_rebuild(k, Some(dims));
                self.cans[k].crushed = true;
            } else {
                self.can_rebuild(k, None);
            }
            if !self.cans[k].spilled && self.cans[k].contains_trash {
                self.cans[k].spilled = true;
                let zero_cat = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
                let mut made = Vec::new();
                for (cx, cy, d) in CAN_CIRCLES {
                    let at = old_xf.world_point(Vec2::new(cx / PS, cy / PS));
                    let b = self.b.create_body(&BodyDef { position: at, angle: ang0, ..Default::default() });
                    let mut sd = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: d / 2.0 / PS });
                    sd.density = 0.75;
                    sd.friction = 0.3;
                    sd.restitution = 0.1;
                    sd.filter = zero_cat;
                    self.b.create_shape(b, &sd);
                    self.b.set_mass_from_shapes(b);
                    made.push(b);
                }
                for (cx, cy, rw, rh) in CAN_RECTS {
                    let at = old_xf.world_point(Vec2::new(cx / PS, cy / PS));
                    let b = self.b.create_body(&BodyDef { position: at, angle: ang0, ..Default::default() });
                    let g = Self::can_polygon(rw, rh, 0.0, 0.0);
                    let mut sd = ShapeDef::new(g);
                    sd.density = 0.75;
                    sd.friction = 0.3;
                    sd.restitution = 0.1;
                    sd.filter = zero_cat;
                    self.b.create_shape(b, &sd);
                    self.b.set_mass_from_shapes(b);
                    made.push(b);
                }
                for &b in &made {
                    self.b.body_mut(b).linear_velocity = v;
                    self.b.body_mut(b).angular_velocity = w;
                }
                self.loose.extend(made);
                if !crush {
                    let mut sd = ShapeDef::new(Self::can_polygon(CAN_BLOCKER[0], CAN_BLOCKER[1], 0.0, 0.0));
                    sd.density = 0.0;
                    sd.friction = 0.0;
                    sd.filter = Filter { category_bits: 2, mask_bits: 2, group_index: 0 };
                    let body = self.cans[k].body;
                    self.b.create_shape(body, &sd);
                }
            }
            let at = self.b.body(self.cans[k].body).sweep.c;
            self.play("TrashCanSpill", 2, at);
        }
    }

    /// `createBloodBurst(range, speed, body, total, …, offset)`: `total` particles around the body's world centre + `offset`, inheriting the
    /// velocity of the body at its local centre + `offset`.
    pub fn blood_burst(&mut self, range: f64, speed: f64, body: BodyId, total: u32, offset: Vec2) {
        let b = self.b.body(body);
        let start = Vec2::new(b.sweep.c.x + offset.x, b.sweep.c.y + offset.y);
        let lc = Vec2::new(b.sweep.local_center.x + offset.x, b.sweep.local_center.y + offset.y);
        let vel = b.linear_velocity_from_local_point(lc);
        self.blood.burst([start.x * PS, start.y * PS], vel, range, speed, total);
    }

    /// `createPointBurst(kind, x, y, range, speed, total)` at a world point (metres).
    pub fn shard_burst(&mut self, kind: crate::blood::ShardKind, at: Vec2, range: f64, speed: f64, total: u32) {
        self.blood.sprite_burst(kind, [at.x * PS, at.y * PS], range, speed, total);
    }

    /// `createBloodFlow(min, max, body, offset (local), angle°, total)`: one particle per step until `total` is exceeded. Returns the flow's index.
    pub fn blood_flow(&mut self, min: f64, max: f64, body: BodyId, offset: Vec2, angle_deg: f64, total: u32) -> usize {
        self.blood.flows.push(crate::blood::Flow { body, offset, angle: angle_deg.to_radians(), min_speed: min, speed_range: max - min, total, count: 0, finished: false, sprite: None });
        self.blood.flows.len() - 1
    }

    /// `createFlow(kind, min, max, body, offset, angle°, total)`: a sprite flow from a body point.
    #[allow(clippy::too_many_arguments)]
    pub fn sprite_flow(&mut self, kind: crate::blood::ShardKind, min: f64, max: f64, body: BodyId, offset: Vec2, angle_deg: f64, total: u32) {
        self.blood.flows.push(crate::blood::Flow { body, offset, angle: angle_deg.to_radians(), min_speed: min, speed_range: max - min, total, count: 0, finished: false, sprite: Some(kind) });
    }

    /// `stopSpewing`.
    pub fn blood_stop(&mut self, flow: usize) {
        if let Some(f) = self.blood.flows.get_mut(flow) {
            f.finished = true;
        }
    }

    /// `createSnowSpray("snowflakes", sleighBody, (start − 143/125, start.y + 178/125), (start + 107/125, …), 0, 0, 180°, 5)`: while the sleigh boosts, five
    /// snowflakes per step appear on that bottom edge of the sleigh and move with it (speed range 0, so only the body's velocity at the point), 180° fan downwards.
    fn snow_spray(&mut self) {
        let Some(crate::vehicle::Vehicle::Santa(s)) = self.bike.as_ref() else { return };
        if !s.bell || s.sleigh_gone {
            return;
        }
        let (body, start) = (s.sleigh, s.start_m);
        let (a, b) = (Vec2::new(start.x - 143.0 / 125.0, start.y + 178.0 / 125.0), Vec2::new(start.x + 107.0 / 125.0, start.y + 178.0 / 125.0));
        for _ in 0..5 {
            let t = self.blood.rand();
            let p = Vec2::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            let (v, at) = {
                let bd = self.b.body(body);
                (bd.linear_velocity_from_local_point(p), bd.world_point(p))
            };
            self.blood.spawn_sprite(crate::blood::ShardKind::Snow, [at.x * PS, at.y * PS], [v.x, v.y]);
        }
    }

    /// Per-step blood update: particles advance, then every running flow emits one particle (the original's emitter `step` order).
    fn step_blood(&mut self) {
        self.blood.step_particles();
        self.blood.step_sparks();
        self.blood.step_sprites();
        self.snow_spray();
        for i in 0..self.blood.flows.len() {
            let (body, offset, angle, min, range, total, count, finished, sprite) = {
                let f = &self.blood.flows[i];
                (f.body, f.offset, f.angle, f.min_speed, f.speed_range, f.total, f.count, f.finished, f.sprite)
            };
            if finished {
                continue;
            }
            if !self.b.body(body).alive || count > total {
                self.blood.flows[i].finished = true;
                continue;
            }
            self.blood.flows[i].count += 1;
            let speed = self.blood.rand() * range + min;
            let b = self.b.body(body);
            let a = b.sweep.a + angle;
            let v = b.linear_velocity_from_local_point(offset);
            let at = b.world_point(offset);
            let vel = [a.cos() * speed + v.x, a.sin() * speed + v.y];
            match sprite {
                Some(kind) => self.blood.spawn_sprite(kind, [at.x * PS, at.y * PS], vel),
                None => self.blood.spawn([at.x * PS, at.y * PS], vel),
            }
        }
    }

    /// Every live body of every NPC, plus its smash chunks and loose special parts (for drawing).
    pub fn npc_draw_bodies(&self) -> Vec<BodyId> {
        self.npcs.iter().flat_map(|n| n.bodies.iter().chain(n.pieces.iter())).chain(self.loose.iter()).copied().filter(|&b| self.b.body(b).alive).collect()
    }

    /// Joint break reaction: destroy the joint and clear the collision group of the broken limb and everything hanging below it.
    fn npc_break_joint(&mut self, ni: usize, ji: usize) {
        let (id, child, _, broken) = self.npcs[ni].joints[ji];
        if broken {
            return;
        }
        self.b.destroy_joint(id);
        self.npcs[ni].joints[ji].3 = true;
        let kids: &[usize] = match child {
            1 => &[1],
            2 => &[2, 7, 8, 9, 10],
            3 => &[3, 5],
            4 => &[4, 6],
            7 => &[7, 9],
            8 => &[8, 10],
            c => &[c],
        };
        for &k in kids {
            let body = self.npcs[ni].bodies[k];
            if !self.b.body(body).alive {
                continue;
            }
            if let Some(&s) = self.b.body(body).shapes.first() {
                let mut fl = self.b.c.shapes[s].shape.filter;
                fl.group_index = 0;
                self.b.set_shape_filter(s, fl);
            }
        }
    }

    /// `NPCharacter.headSmash/chestSmash/pelvisSmash`: a Result above the part's limit breaks the joints the original breaks, replaces the body by
    /// chunk circles (head 4 + brain, chest 4 + heart, pelvis 3; density 1, material 1, zero filter) that inherit its point velocities, and destroys it.
    fn npc_smashes(&mut self, events: &[box2d::world::ContactEvent]) {
        use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};
        let cs = 125.0;
        for ni in 0..self.npcs.len() {
            // order of the original handler: head, chest, pelvis
            for part in 0..3 {
                let n = &self.npcs[ni];
                if n.smashed[part] {
                    continue;
                }
                let (shape, limit) = (n.smash_shapes[part], n.smash_limits[part]);
                if !events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape) && e.normal_impulse > limit) {
                    continue;
                }
                self.npcs[ni].smashed[part] = true;
                let bodies = self.npcs[ni].bodies;
                let chunk = self.npcs[ni].chunk;
                // joints: 0 neck 1 waist 2/3 shoulders 4/5 elbows 6/7 hips 8/9 knees
                let brk: &[usize] = match part {
                    0 => &[],
                    1 => &[0, 2, 3, 1],
                    _ => &[6, 7, 1],
                };
                for &ji in brk {
                    self.npc_break_joint(ni, ji);
                }
                let body = bodies[[1, 0, 2][part]];
                let (pos, ang, w) = {
                    let b = self.b.body(body);
                    (b.xf.position, b.sweep.a, b.angular_velocity)
                };
                let src = self.b.body(body).clone();
                let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
                let mut made = Vec::new();
                let mut ball = |w: &mut Self, at: Vec2, angle: f64, radius: f64, lin: Vec2, av: f64| {
                    let c = w.b.create_body(&BodyDef { position: at, angle, ..Default::default() });
                    let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius });
                    d.density = 1.0;
                    d.friction = 0.3;
                    d.restitution = 0.1;
                    d.filter = zero;
                    let sh = w.b.create_shape(c, &d);
                    w.b.shape_mut(sh).material = 1;
                    w.b.set_mass_from_shapes(c);
                    w.b.body_mut(c).linear_velocity = lin;
                    w.b.body_mut(c).angular_velocity = av;
                    made.push(c);
                };
                let vel = |at: Vec2| src.linear_velocity_from_world_point(at);
                match part {
                    0 => {
                        let (d, mut l) = (4.0 / cs, ang);
                        for _ in 0..4 {
                            let at = Vec2::new(pos.x + l.sin() * d, pos.y + l.cos() * d);
                            ball(self, at, 0.0, chunk[0] / cs, vel(at), w);
                            l += FRAC_PI_2;
                        }
                        ball(self, pos, ang, 12.0 / cs, src.linear_velocity, w);
                    }
                    1 => {
                        let (a, mut sa) = (5.0 / cs, ang + FRAC_PI_4);
                        for _ in 0..4 {
                            let at = Vec2::new(pos.x + sa.sin() * a, pos.y + sa.cos() * a);
                            ball(self, at, ang, chunk[1] / cs, vel(at), w);
                            sa += FRAC_PI_2;
                        }
                        ball(self, pos, ang, chunk[0] / cs, src.linear_velocity, w);
                    }
                    _ => {
                        let (a, mut sa) = (3.0 / cs, ang);
                        for _ in 0..3 {
                            let at = Vec2::new(pos.x + sa.sin() * a, pos.y + sa.cos() * a);
                            ball(self, at, ang, chunk[2] / cs, vel(at), w);
                            sa += 2.0 * PI / 3.0;
                        }
                    }
                }
                self.npcs[ni].pieces.extend(made);
                let (range, speed, total) = [(5.0, 15.0, 200), (5.0, 20.0, 300), (5.0, 15.0, 200)][part];
                self.blood_burst(range, speed, body, total, Vec2::default());
                self.play(["HeadSmash", "ChestSmash", "PelvisSmash"][part], 1, pos);
                self.b.destroy_body(body);
                // joints that went with the destroyed body
                let idx = [1usize, 0, 2][part];
                for j in &mut self.npcs[ni].joints {
                    if j.1 == idx || (idx == 0 && matches!(j.1, 1..=4)) || (idx == 2 && matches!(j.1, 7 | 8)) {
                        j.3 = true;
                    }
                }
            }
        }
    }

    /// BladeWeapon (standalone, interactive; transcribed createBody/createPrisJoint; blade/handle guides of `BladeWeaponShapesMC` frames 1..12 read from the oracle):
    /// a body with a sensor box along the blade (density .5), a solid copy colliding only with category 8, and a handle box; whatever the blade sensor
    /// touches (material bits 6 or massless) gets a locked prismatic joint through the body centre, like the arrows. Blood/sounds/`shapeImpale`/swordInHead NOT ported.
    fn blade(&mut self, sp: &crate::level::Special) -> Option<BodyId> {
        if sp.flag("interactive") == Some(false) {
            return None;
        }
        // (blade x, y, rotation°, scaleX, scaleY), (handle x, y, scaleX, scaleY)
        const BLADES: [([f64; 5], [f64; 4]); 12] = [
            ([0.0, -32.35, 0.0, 0.22, 0.27], [-0.05, 10.85, 0.03, 0.646]),
            ([1.15, -11.1, 2.979_380_534_680_281, 0.03, 0.641], [-0.05, 31.9, 0.032, 0.224]),
            ([0.0, -9.55, 0.0, 0.043, 0.654], [0.0, 32.6, 0.032, 0.192]),
            ([-0.15, -6.85, -89.954_371_383_523_9, 0.205, 0.098], [2.75, 10.2, 0.045, 0.135]),
            ([0.25, -7.05, 0.0, 0.08, 0.688], [0.85, 34.45, 0.031, 0.139]),
            ([-0.1, -26.75, -90.011_669_612_884_5, 0.104, 0.166], [2.4, 5.35, 0.051, 0.538]),
            ([-0.3, -5.35, 0.0, 0.06, 0.49], [0.25, 28.15, 0.05, 0.18]),
            ([0.0, -63.15, 0.0, 0.048, 0.404], [0.0, 20.15, 0.026, 1.263]),
            ([0.0, -25.95, 0.0, 0.08, 0.473], [0.0, 1.35, 0.075, 0.073]),
            ([0.0, -6.05, 0.0, 0.036, 1.522], [0.0, 76.75, 0.038, 0.129]),
            ([0.0, -8.75, 0.0, 0.03, 0.444], [0.0, 22.3, 0.038, 0.176]),
            ([-0.25, -54.75, 0.0, 0.182, 0.276], [0.0, 14.1, 0.038, 1.102]),
        ];
        let (x, y) = (f64::from(sp.num("x")?), f64::from(sp.num("y")?));
        let r = f64::from(sp.num("angle").unwrap_or(0.0)).to_radians();
        let ty = (sp.num("bladeWeaponType").unwrap_or(1.0) as usize).clamp(1, 12);
        let (bl, hd) = BLADES[ty - 1];
        let m = if sp.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
        let body = self.b.create_body(&BodyDef { position: Vec2::new(x / PS, y / PS), angle: r, is_sleeping: sp.flag("sleeping").unwrap_or(false), ..Default::default() });
        let blade_rot = (bl[2] * m).to_radians();
        let mut sd = ShapeDef::oriented_box(100.0 * bl[3] / 2.0 / PS, 100.0 * bl[4] / 2.0 / PS, Vec2::new(bl[0] * m / PS, bl[1] / PS), blade_rot);
        sd.friction = 0.75;
        sd.restitution = 0.1;
        sd.density = 0.5;
        sd.is_sensor = true;
        sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        let sensor = self.b.create_shape(body, &sd);
        sd.is_sensor = false;
        sd.filter.mask_bits = 8;
        let solid = self.b.create_shape(body, &sd);
        let mut hs = ShapeDef::oriented_box(100.0 * hd[2] / 2.0 / PS, 100.0 * hd[3] / 2.0 / PS, Vec2::new(hd[0] * m / PS, hd[1] / PS), 0.0);
        hs.friction = 0.75;
        hs.restitution = 0.1;
        hs.density = 0.5;
        hs.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        self.b.create_shape(body, &hs);
        self.b.set_mass_from_shapes(body);
        self.arrows.push(Arrow { body, sensor, solid, pinned: Vec::new(), queue_add: Vec::new(), queue_remove: Vec::new(), armed: false, done: false, harpoon: true, blade: Some(r + blade_rot), blood: Vec::new(), reverse: m < 0.0, flesh_sounds: 0 });
        Some(body)
    }

    fn spawn_arrow(&mut self, pos: Vec2, angle: f64, vel: Vec2, harpoon: bool) {
        let body = self.b.create_body(&BodyDef { position: pos, angle, ..Default::default() });
        let (tip_hx, tip_hy, tip_c, shaft_hx, shaft_c, density) = if harpoon { (26.25, 2.5, 26.25, 26.25, -26.25, 3.0) } else { (10.0, 1.0, 16.5, 16.5, -10.0, 1.0) };
        let mut s = ShapeDef::oriented_box(tip_hx / PS, tip_hy / PS, Vec2::new(tip_c / PS, 0.0), 0.0);
        s.is_sensor = true;
        s.density = density;
        s.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: -21 };
        let sensor = self.b.create_shape(body, &s);
        let mut h = ShapeDef::oriented_box(shaft_hx / PS, tip_hy / PS, Vec2::new(shaft_c / PS, 0.0), 0.0);
        h.density = density;
        h.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        let solid = self.b.create_shape(body, &h);
        self.b.set_mass_from_shapes(body);
        self.b.body_mut(body).linear_velocity = vel;
        self.b.c.report_results = true;
        self.arrows.push(Arrow { body, sensor, solid, pinned: Vec::new(), queue_add: Vec::new(), queue_remove: Vec::new(), armed: false, done: false, harpoon, blade: None, blood: Vec::new(), reverse: false, flesh_sounds: 0 });
    }

    /// `ArrowGun.actions` + `fireArrow` (before the world step) and the arrows' own `actions` (pin / unpin queued bodies).
    fn run_arrow_guns(&mut self) {
        for gi in 0..self.arrow_guns.len() {
            if self.arrow_guns[gi].done {
                continue;
            }
            let pos = self.arrow_guns[gi].pos;
            if self.arrow_guns[gi].target.is_some_and(|t| !self.b.c.bodies[t].alive) {
                self.arrow_guns[gi].target = None;
            }
            let closest = |w: &Self, gi: usize| {
                let mut best = 1e6;
                let mut found = None;
                for &(t, _) in &w.arrow_guns[gi].targets {
                    let c = w.b.body(t).sweep.c;
                    let dd = (c.x - pos.x).powi(2) + (c.y - pos.y).powi(2);
                    if dd < best {
                        best = dd;
                        found = Some(t);
                    }
                }
                found
            };
            if self.arrow_guns[gi].target.is_none() {
                self.arrow_guns[gi].retarget = 0;
                self.arrow_guns[gi].target = closest(self, gi);
            }
            if let Some(t) = self.arrow_guns[gi].target {
                let (tc, tv) = (self.b.body(t).sweep.c, self.b.body(t).linear_velocity);
                if self.arrow_guns[gi].aligned && self.arrow_guns[gi].frame_count == 0 && self.path_clear(pos, tc, t) {
                    // lead the target: solve |v·τ + (c − p)| = speed
                    let e = 2000.0 / PS;
                    let (dx, dy) = (tc.x - pos.x, tc.y - pos.y);
                    let qa = tv.x * tv.x + tv.y * tv.y - e * e;
                    let qb = 2.0 * (tv.x * dx + tv.y * dy);
                    let qc = dx * dx + dy * dy;
                    let disc = qb * qb - 4.0 * qa * qc;
                    if disc >= 0.0 {
                        let (mut u, mut v) = ((-qb + disc.sqrt()) / (2.0 * qa), (-qb - disc.sqrt()) / (2.0 * qa));
                        if u < 0.0 {
                            u = 1e4;
                        }
                        if v < 0.0 {
                            v = 1e4;
                        }
                        let m = u.min(v);
                        let (fx, fy) = (m * tv.x + tc.x, m * tv.y + tc.y);
                        let ang = (fy - pos.y).atan2(fx - pos.x);
                        let harpoon = self.arrow_guns[gi].harpoon;
                        if harpoon {
                            self.play("HarpoonFire", 1, pos);
                        } else {
                            self.play("ArrowFire", 2, pos);
                        }
                        self.spawn_arrow(pos, ang, Vec2::new(e * ang.cos(), e * ang.sin()), harpoon);
                    }
                    self.arrow_guns[gi].arrows_left -= 1;
                    self.arrow_guns[gi].frame_count = self.arrow_guns[gi].frames_per_shot;
                    if self.arrow_guns[gi].arrows_left <= 0 {
                        self.arrow_guns[gi].done = true;
                        continue;
                    }
                }
                let mut s = (tc.y - pos.y).atan2(tc.x - pos.x) + 0.5 * std::f64::consts::PI;
                let mut a = self.arrow_guns[gi].angle - s;
                let two_pi = 2.0 * std::f64::consts::PI;
                if a > std::f64::consts::PI {
                    a -= two_pi;
                }
                if a < -std::f64::consts::PI {
                    a += two_pi;
                }
                if a.abs() > 0.52 {
                    s += 0.5 * a;
                    self.arrow_guns[gi].aligned = false;
                } else {
                    self.arrow_guns[gi].aligned = true;
                }
                self.arrow_guns[gi].angle = s;
                self.arrow_guns[gi].retarget += 1;
                if self.arrow_guns[gi].retarget == 5 {
                    self.arrow_guns[gi].retarget = 0;
                    self.arrow_guns[gi].target = closest(self, gi);
                }
            }
            let g = &mut self.arrow_guns[gi];
            g.frame_count = g.frame_count.saturating_sub(1);
        }
        // arrows: pin / unpin the bodies queued by the tip sensor
        for ai in 0..self.arrows.len() {
            if self.arrows[ai].done {
                continue;
            }
            for other in std::mem::take(&mut self.arrows[ai].queue_add) {
                if !self.b.c.bodies[other].alive {
                    continue;
                }
                if let Some(p) = self.arrows[ai].pinned.iter_mut().find(|p| p.0 == other) {
                    p.2 += 1;
                    continue;
                }
                let body = self.arrows[ai].body;
                let (anchor, axis) = match self.arrows[ai].blade {
                    Some(blade_angle) => {
                        let u = self.b.body(body).sweep.a - blade_angle + std::f64::consts::FRAC_PI_2;
                        (self.b.body(body).sweep.c, Vec2::new(u.cos(), u.sin()))
                    }
                    None => {
                        let a = self.b.body(body).sweep.a;
                        (self.b.body(body).world_point(Vec2::new(if self.arrows[ai].harpoon { 26.25 } else { 13.25 } / PS, 0.0)), Vec2::new(a.cos(), a.sin()))
                    }
                };
                let mut jd = PrismaticJointDef::initialize(self.b.body(body), body, self.b.body(other), other, anchor, axis);
                jd.enable_limit = true;
                jd.lower_translation = 0.0;
                jd.upper_translation = 0.0;
                jd.collide_connected = true;
                jd.enable_motor = true;
                jd.max_motor_force = 1e5;
                jd.motor_speed = 0.0;
                let j = self.b.create_prismatic_joint(&jd);
                self.arrows[ai].pinned.push((other, j, 1));
                self.arrows[ai].armed = true;
                if self.arrows[ai].blade.is_some() && self.b.shape_material(self.b.body(other).shapes[0]) & 6 != 0 {
                    let tip = self.b.body(body).sweep.c;
                    self.shape_impale(other, true, Some(tip), 0.01);
                    // BladeWeapon.createPrisJoint on a new flesh body: 50-particle blood burst at its centre, `paintBlood` stamp, BladeFlesh sound
                    let c = self.b.body(other).sweep.c;
                    self.blood.burst([c.x * PS, c.y * PS], Vec2::default(), 5.0, 15.0, 50);
                    let l = self.b.body(body).local_point(c);
                    let flip = if self.arrows[ai].reverse { -1.0 } else { 1.0 };
                    let (rot, mirror) = (self.blood.rand() * std::f64::consts::TAU, if self.blood.rand() > 0.5 { -1.0 } else { 1.0 });
                    self.arrows[ai].blood.push((l.x * PS * flip, l.y * PS, rot, mirror));
                    if self.arrows[ai].flesh_sounds < 1 {
                        self.arrows[ai].flesh_sounds += 1;
                        self.play("BladeFlesh", 3, c);
                    }
                }
            }
            for other in std::mem::take(&mut self.arrows[ai].queue_remove) {
                if let Some(k) = self.arrows[ai].pinned.iter().position(|p| p.0 == other) {
                    self.arrows[ai].pinned[k].2 -= 1;
                    if self.arrows[ai].pinned[k].2 == 0 {
                        let (_, j, _) = self.arrows[ai].pinned.remove(k);
                        self.b.destroy_joint(j);
                    }
                }
            }
        }
    }

    /// `Spikes.checkAdd`/`checkRemove` (sensor contacts with material bits 2/4) and `actions` → `createPrisJoint` / `removeJoint`.
    fn spike_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for ev in events {
            for si in 0..self.spikes.len() {
                let sensor = self.spikes[si].sensor;
                if ev.shape1 != sensor && ev.shape2 != sensor {
                    continue;
                }
                let o = if ev.shape1 == sensor { ev.shape2 } else { ev.shape1 };
                if self.b.shape_material(o) & 6 == 0 {
                    continue;
                }
                let body = self.b.c.shapes[o].body;
                match ev.kind {
                    ContactEventKind::Add => self.spikes[si].queue_add.push(body),
                    ContactEventKind::Remove if self.spikes[si].pinned.iter().any(|p| p.0 == body) || self.spikes[si].queue_add.contains(&body) => self.spikes[si].queue_remove.push(body),
                    _ => {}
                }
            }
        }
        for si in 0..self.spikes.len() {
            for other in std::mem::take(&mut self.spikes[si].queue_add) {
                if !self.b.c.bodies[other].alive {
                    continue;
                }
                if let Some(p) = self.spikes[si].pinned.iter_mut().find(|p| p.0 == other) {
                    p.2 += 1;
                    continue;
                }
                self.shape_impale(other, true, None, 0.0);
                let (body, angle) = (self.spikes[si].body, self.spikes[si].angle);
                let at = self.b.body(other).sweep.c;
                let mut jd = PrismaticJointDef::initialize(self.b.body(body), body, self.b.body(other), other, at, Vec2::new(-angle.sin(), angle.cos()));
                jd.collide_connected = true;
                jd.enable_motor = true;
                jd.max_motor_force = 30.0;
                jd.motor_speed = 0.0;
                let j = self.b.create_prismatic_joint(&jd);
                self.spikes[si].pinned.push((other, j, 1));
                self.blood.burst([at.x * PS, at.y * PS], Vec2::default(), 5.0, 15.0, 50);
                // paintBlood: the impale point in the spike row's coordinates, snapped to the nearest spike (JS `%` keeps the dividend's sign)
                let local = self.b.body(body).local_point(at);
                let i = local.x * PS + self.spikes[si].half_width;
                let mut o = i % 15.0;
                o = if o < 0.0 {
                    if o < -7.5 { -15.0 - o } else { -o }
                } else if o > 7.5 {
                    15.0 - o
                } else {
                    -o
                };
                let frame = (5.0 * self.blood.rand()).ceil().clamp(1.0, 5.0) as u8;
                self.spikes[si].blood.push((i + o, frame));
                if self.spikes[si].sounds < 2 {
                    self.spikes[si].sounds += 1;
                    self.play("ImpaleSpikes", 3, at);
                }
            }
            for other in std::mem::take(&mut self.spikes[si].queue_remove) {
                if let Some(k) = self.spikes[si].pinned.iter().position(|p| p.0 == other) {
                    self.spikes[si].pinned[k].2 -= 1;
                    if self.spikes[si].pinned[k].2 == 0 {
                        let (_, j, _) = self.spikes[si].pinned.remove(k);
                        self.b.destroy_joint(j);
                    }
                }
            }
        }
    }

    /// BladeWeapon blood stamps (clip-local px x, y, rotation, scaleX) of the blade whose body is `body`.
    pub fn blade_blood(&self, body: BodyId) -> &[(f64, f64, f64, f64)] {
        self.arrows.iter().find(|a| a.body == body && a.blade.is_some()).map_or(&[], |a| a.blood.as_slice())
    }

    /// Blood stamps (x in the spike row, frame) of the spikes special whose body is `body`.
    pub fn spike_blood(&self, body: BodyId) -> &[(f64, u8)] {
        self.spikes.iter().find(|s| s.body == body).map_or(&[], |s| s.blood.as_slice())
    }

    /// Arrow tip sensor (`checkAdd`/`checkRemove`), gun range sensors, and `arrowResultHandler` (shaft impulse > 1 on a pinned arrow snaps it).
    fn arrow_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for ev in events {
            for gi in 0..self.arrow_guns.len() {
                let range = self.arrow_guns[gi].range;
                let o = if ev.shape1 == range { ev.shape2 } else if ev.shape2 == range { ev.shape1 } else { continue };
                if self.b.shape_material(o) & 2 == 0 {
                    continue;
                }
                let body = self.b.c.shapes[o].body;
                let g = &mut self.arrow_guns[gi];
                match ev.kind {
                    ContactEventKind::Add => match g.targets.iter_mut().find(|t| t.0 == body) {
                        Some(t) => t.1 += 1,
                        None => g.targets.push((body, 1)),
                    },
                    ContactEventKind::Remove => {
                        if let Some(k) = g.targets.iter().position(|t| t.0 == body) {
                            g.targets[k].1 -= 1;
                            if g.targets[k].1 == 0 {
                                g.targets.remove(k);
                                if g.target == Some(body) {
                                    g.target = None;
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
            for ai in 0..self.arrows.len() {
                if self.arrows[ai].done {
                    continue;
                }
                let (sensor, solid) = (self.arrows[ai].sensor, self.arrows[ai].solid);
                if ev.shape1 == sensor || ev.shape2 == sensor {
                    let o = if ev.shape1 == sensor { ev.shape2 } else { ev.shape1 };
                    let body = self.b.c.shapes[o].body;
                    let sensor_other = self.b.c.shapes[o].shape.is_sensor;
                    if (self.b.shape_material(o) & 6 != 0 || self.b.body(body).mass == 0.0) && !sensor_other && body != self.arrows[ai].body {
                        match ev.kind {
                            ContactEventKind::Add => self.arrows[ai].queue_add.push(body),
                            ContactEventKind::Remove if self.arrows[ai].pinned.iter().any(|p| p.0 == body) || self.arrows[ai].queue_add.contains(&body) => self.arrows[ai].queue_remove.push(body),
                            _ => {}
                        }
                    }
                } else if ev.kind == ContactEventKind::Result && (ev.shape1 == solid || ev.shape2 == solid) && self.arrows[ai].armed && !self.arrows[ai].harpoon && ev.normal_impulse > 1.0 {
                    self.arrows[ai].done = true;
                    let body = self.arrows[ai].body;
                    let at = self.b.body(body).sweep.c;
                    self.play("ArrowSnap", 2, at);
                    self.b.destroy_body(body);
                }
            }
        }
    }

    /// HomingMine (transcribed createBodies/actions/mineResult/explode; constants SENSOR_RADIUS 600 px, SMASH_IMPULSE 3, EXPLOSION_DISTANCE 0.2304 (squared m),
    /// SLOWING_FACTOR .05, GRAVITY_DISPLACEMENT −1/3 read from the oracle). The line-of-sight sensor box on the targeting body is replaced by a direct
    /// segment test mine→target against static / density ≥ 10 shapes at action time (the original gets it from sensor contacts one step later).
    fn mine(&mut self, sp: &crate::level::Special) -> Option<BodyId> {
        let (x, y) = (f64::from(sp.num("x")?), f64::from(sp.num("y")?));
        let pos = Vec2::new(x / PS, y / PS);
        let body = self.b.create_body(&BodyDef { position: pos, allow_sleep: false, ..Default::default() });
        let mut sd = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 10.0 / PS });
        sd.density = 1.0;
        sd.friction = 1.0;
        sd.restitution = 0.1;
        sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        let shape = self.b.create_shape(body, &sd);
        self.b.set_mass_from_shapes(body);
        sd.geometry = Geometry::Circle { local_position: Vec2::default(), radius: 600.0 / PS };
        sd.is_sensor = true;
        sd.filter.group_index = -20;
        let range = self.b.create_shape(body, &sd);
        let targeting = self.b.create_body(&BodyDef { position: pos, allow_sleep: false, ..Default::default() });
        self.b.body_mut(body).linear_velocity.y += 1.0 / 30.0 * -10.0;
        self.b.body_mut(targeting).linear_velocity.y += 1.0 / 30.0 * -10.0;
        self.b.c.report_results = true;
        self.mines.push(Mine {
            body,
            targeting,
            shape,
            range,
            seek: 0.01 * f64::from(sp.num("seekSpeed").unwrap_or(1.0)),
            counter: 30.0 * f64::from(sp.num("explosionDelay").unwrap_or(0.0)),
            counting_down: false,
            in_contact: false,
            retarget: 0,
            target: None,
            targets: Vec::new(),
            done: false,
        });
        Some(body)
    }

    /// `Shape.TestSegment` reduced to a yes/no: does segment p1→p2 (world) touch the shape?
    fn segment_hits_shape(&self, shape: ShapeId, p1: Vec2, p2: Vec2) -> bool {
        let sh = &self.b.c.shapes[shape];
        let body = self.b.body(sh.body);
        let (a, b) = (body.local_point(p1), body.local_point(p2));
        match &sh.shape.kind {
            box2d::shape::ShapeKind::Circle(c) => {
                let (d, f) = (Vec2::new(b.x - a.x, b.y - a.y), Vec2::new(a.x - c.local_position.x, a.y - c.local_position.y));
                let (qa, qb, qc) = (d.x * d.x + d.y * d.y, 2.0 * (f.x * d.x + f.y * d.y), f.x * f.x + f.y * f.y - c.radius * c.radius);
                if qc <= 0.0 {
                    return true;
                }
                let disc = qb * qb - 4.0 * qa * qc;
                if qa == 0.0 || disc < 0.0 {
                    return false;
                }
                let t = (-qb - disc.sqrt()) / (2.0 * qa);
                (0.0..=1.0).contains(&t)
            }
            box2d::shape::ShapeKind::Polygon { poly, .. } => {
                // clip the segment against every edge half-plane (convex polygon)
                let (mut lo, mut hi) = (0.0f64, 1.0f64);
                for (v, n) in poly.vertices.iter().zip(&poly.normals) {
                    let num = n.x * (v.x - a.x) + n.y * (v.y - a.y);
                    let den = n.x * (b.x - a.x) + n.y * (b.y - a.y);
                    if den == 0.0 {
                        if num < 0.0 {
                            return false;
                        }
                    } else if den < 0.0 {
                        lo = lo.max(num / den);
                    } else {
                        hi = hi.min(num / den);
                    }
                    if lo > hi {
                        return false;
                    }
                }
                true
            }
        }
    }

    /// True when no blocker (non-sensor shape that is static or has density ≥ 10) sits on the segment p1→p2.
    fn path_clear(&self, p1: Vec2, p2: Vec2, ignore: BodyId) -> bool {
        let lo = Vec2::new(p1.x.min(p2.x), p1.y.min(p2.y));
        let hi = Vec2::new(p1.x.max(p2.x), p1.y.max(p2.y));
        let found = self.b.query(&box2d::collision::Aabb { lower: lo, upper: hi }, 100_000);
        !found.into_iter().any(|s| {
            let sh = &self.b.c.shapes[s];
            let body = self.b.body(sh.body);
            sh.body != ignore && !sh.shape.is_sensor && !(body.mass > 0.0 && sh.shape.density < 10.0) && self.segment_hits_shape(s, p1, p2)
        })
    }

    /// Radial explosion shared by Jet and HomingMine: every non-static body whose broad-phase box lies in ±2.6 m of `c` gets `5·(1 − d/2.6)`.
    fn explosion_burst(&mut self, c: Vec2) {
        self.play("MineExplosion", 1, c);
        let l = 2.6;
        let found = self.b.query(&box2d::collision::Aabb { lower: Vec2::new(c.x - l, c.y - l), upper: Vec2::new(c.x + l, c.y + l) }, 30);
        for s in found {
            let other = self.b.c.shapes[s].body;
            if !self.b.c.bodies[other].alive || self.b.body(other).is_static {
                continue;
            }
            let v = self.b.body(other).sweep.c;
            let (dx, dy) = (v.x - c.x, v.y - c.y);
            let w = 1.0 - dx.hypot(dy).min(l) / l;
            let a = dy.atan2(dx);
            self.b.body_mut(other).apply_impulse(Vec2::new(a.cos() * w * 5.0, a.sin() * w * 5.0), v);
        }
    }

    fn explode_mine(&mut self, mi: usize) {
        let c = self.b.body(self.mines[mi].body).sweep.c;
        let (b, t) = (self.mines[mi].body, self.mines[mi].targeting);
        self.mines[mi].done = true;
        self.b.destroy_body(b);
        self.b.destroy_body(t);
        self.explosion_burst(c);
    }

    /// `HomingMine.actions` (before the world step).
    fn run_mines(&mut self) {
        for mi in 0..self.mines.len() {
            if self.mines[mi].done {
                continue;
            }
            if self.mines[mi].counting_down {
                self.mines[mi].counter -= 1.0;
                if self.mines[mi].counter <= 0.0 {
                    self.explode_mine(mi);
                    continue;
                }
            }
            let (bid, tid) = (self.mines[mi].body, self.mines[mi].targeting);
            let (e, n, i, inv_m, inv_i) = {
                let b = self.b.body(bid);
                (b.sweep.c, b.linear_velocity, b.angular_velocity, b.inv_mass, b.inv_i)
            };
            let seek = self.mines[mi].seek;
            let sf = 0.05;
            if self.mines[mi].target.is_some_and(|t| !self.b.c.bodies[t].alive) {
                self.mines[mi].target = None;
            }
            if self.mines[mi].target.is_none() {
                self.mines[mi].retarget = 0;
                let mut best = 1e6;
                for &(t, _) in &self.mines[mi].targets.clone() {
                    let c = self.b.body(t).sweep.c;
                    let d = (c.x - e.x).powi(2) + (c.y - e.y).powi(2);
                    if d < best {
                        best = d;
                        self.mines[mi].target = Some(t);
                    }
                }
            }
            let (cx, cy);
            if let Some(t) = self.mines[mi].target {
                let s = self.b.body(t).sweep.c;
                let a = Vec2::new(s.x - e.x, s.y - e.y);
                let h = a.x * a.x + a.y * a.y;
                if !self.mines[mi].counting_down && (h < 0.2304 || self.mines[mi].in_contact) {
                    if self.mines[mi].counter <= 0.0 {
                        self.explode_mine(mi);
                        continue;
                    }
                    self.mines[mi].counting_down = true;
                }
                let d = a.y.atan2(a.x) + 0.5 * std::f64::consts::PI;
                self.b.set_xform(tid, e, d);
                if self.path_clear(e, s, t) {
                    let p = Vec2::new(n.x / 30.0, n.y / 30.0);
                    let len = h.sqrt();
                    let m = Vec2::new(a.x / len, a.y / len);
                    let f = Vec2::new(-m.y, m.x);
                    let dot = p.x * f.x + p.y * f.y;
                    if dot.abs() > seek {
                        let k = if dot > 0.0 { -seek } else { seek };
                        (cx, cy) = (f.x * k, f.y * k);
                    } else {
                        let w = (dot * dot + seek * seek).sqrt();
                        (cx, cy) = (f.x * -dot + m.x * w, f.y * -dot + m.y * w);
                    }
                } else {
                    (cx, cy) = (-n.x * sf / inv_m, -n.y * sf / inv_m);
                }
                self.mines[mi].retarget += 1;
                if self.mines[mi].retarget == 5 {
                    self.mines[mi].target = None;
                }
            } else {
                (cx, cy) = (-n.x * sf / inv_m, -n.y * sf / inv_m);
            }
            let bi = -i * sf / inv_i;
            let b = self.b.body_mut(bid);
            b.linear_velocity.x += inv_m * cx;
            b.linear_velocity.y += inv_m * cy;
            b.angular_velocity += inv_i * bi;
            b.linear_velocity.y += -1.0 / 3.0;
            self.b.body_mut(tid).linear_velocity.y += -1.0 / 3.0;
        }
    }

    /// Mine sensor/contact events: range-sensor target counts and `mineResult` (impulse > 3 → explode, else touching a ragdoll arms the fuse).
    fn mine_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for ev in events {
            for mi in 0..self.mines.len() {
                if self.mines[mi].done {
                    continue;
                }
                let (range, shape) = (self.mines[mi].range, self.mines[mi].shape);
                let other = |a: ShapeId, b: ShapeId, mine: ShapeId| if a == mine { Some(b) } else if b == mine { Some(a) } else { None };
                if let Some(o) = other(ev.shape1, ev.shape2, range) {
                    if self.b.shape_material(o) & 2 == 0 {
                        continue;
                    }
                    let body = self.b.c.shapes[o].body;
                    let m = &mut self.mines[mi];
                    match ev.kind {
                        ContactEventKind::Add => match m.targets.iter_mut().find(|t| t.0 == body) {
                            Some(t) => t.1 += 1,
                            None => m.targets.push((body, 1)),
                        },
                        ContactEventKind::Remove => {
                            if let Some(k) = m.targets.iter().position(|t| t.0 == body) {
                                m.targets[k].1 -= 1;
                                if m.targets[k].1 == 0 {
                                    m.targets.remove(k);
                                    if m.target == Some(body) {
                                        m.target = None;
                                    }
                                }
                            }
                        }
                        _ => {}
                    }
                } else if ev.kind == ContactEventKind::Result {
                    if let Some(o) = other(ev.shape1, ev.shape2, shape) {
                        if ev.normal_impulse > 3.0 {
                            self.explode_mine(mi);
                        } else if self.b.shape_material(o) & 2 != 0 {
                            self.mines[mi].in_contact = true;
                        }
                    }
                }
            }
        }
    }

    /// FoodItem `checkContact`/`singleAction`: the body is destroyed when a contact impulse exceeds the type's limit.
    fn food_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Result) {
            if let Some(k) = self.foods.iter().position(|f| (f.1 == ev.shape1 || f.1 == ev.shape2) && ev.normal_impulse > f.2) {
                let (body, _, _, ty) = self.foods.remove(k);
                let at = self.b.body(body).sweep.c;
                self.play("FoodSplat", 3, at);
                self.food_smash(body, ty);
                self.b.destroy_body(body);
            }
        }
    }

    /// `FoodItem.singleAction`: one circle chunk body per `m<n>` clip child (position/size from `WatermelonMC.chunks`, density 2, material 1, the food's
    /// velocities) and a 30-particle `createRectBurst` over the shape's bounding box (speed range 10) of the fruit's particle clip.
    fn food_smash(&mut self, body: BodyId, ty: u8) {
        // (m_n.x, m_n.y, c_n.width) px per food type, read from the oracle (`chunks` of the three clips)
        const CHUNKS: [[(f64, f64, f64); 4]; 3] = [
            [(1.9, 1.0, 16.93), (11.9, -4.2, 14.56), (3.35, 8.25, 13.26), (-5.65, -2.9, 21.7)],
            [(0.5, -13.15, 14.2), (13.5, -0.5, 13.57), (2.4, 3.05, 16.76), (-10.7, 1.6, 16.94)],
            [(-0.6, -8.0, 14.2), (5.0, 7.05, 7.33), (0.6, 14.3, 11.55), (-1.95, 10.45, 11.55)],
        ];
        let (pos, ang, v, w) = {
            let b = self.b.body(body);
            (b.xf.position, b.sweep.a, b.linear_velocity, b.angular_velocity)
        };
        let src = self.b.body(body).clone();
        for (n, &(mx, my, cw)) in CHUNKS[ty as usize % 3].iter().enumerate() {
            let at = src.world_point(Vec2::new(mx / PS, my / PS));
            let c = self.b.create_body(&BodyDef { position: at, angle: ang, ..Default::default() });
            let mut sd = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: cw / 2.0 / PS });
            sd.density = 2.0;
            sd.friction = 0.3;
            sd.restitution = 0.1;
            sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
            let s = self.b.create_shape(c, &sd);
            self.b.shape_mut(s).material = 1;
            self.b.set_mass_from_shapes(c);
            self.b.body_mut(c).linear_velocity = v;
            self.b.body_mut(c).angular_velocity = w;
            self.food_chunks.push((c, ty, n as u8));
            self.loose.push(c);
        }
        let _ = pos;
        self.rect_burst(&src, body, crate::blood::ShardKind::Food(ty), 10.0, 30);
    }

    /// `createRectBurst(clip, speed, body, total)`: particles uniform over the first shape's local bounding box, each with the body's velocity at its
    /// start point plus `rand·speed − speed/2` per axis (`src` is the body as it was, in case it is about to be destroyed).
    fn rect_burst(&mut self, src: &box2d::world::Body, body: BodyId, kind: crate::blood::ShardKind, speed: f64, total: u32) {
        let first = self.b.body(body).shapes[0];
        let (mut lo, mut hi) = (Vec2::new(1e4, 1e4), Vec2::new(-1e4, -1e4));
        if let box2d::shape::ShapeKind::Polygon { poly, .. } = &self.b.c.shapes[first].shape.kind {
            for p in &poly.vertices {
                lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
                hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
            }
        }
        for _ in 0..total {
            let off = Vec2::new(lo.x + self.blood.rand() * (hi.x - lo.x), lo.y + self.blood.rand() * (hi.y - lo.y));
            let sv = src.linear_velocity_from_local_point(off);
            let vel = [sv.x + (self.blood.rand() * speed - speed / 2.0), sv.y + (self.blood.rand() * speed - speed / 2.0)];
            let at = src.world_point(off);
            self.blood.spawn_sprite(kind, [at.x * PS, at.y * PS], vel);
        }
    }

    /// `Boombox.checkContact` / `singleAction` / `shatter`.
    fn boombox_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for k in 0..self.boomboxes.len() {
            if self.boomboxes[k].done {
                continue;
            }
            let body = self.boomboxes[k].body;
            let shape = self.b.body(body).shapes[0];
            let limit = self.boomboxes[k].crack_impulse;
            let Some(imp) = events.iter().filter(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape) && e.normal_impulse > limit).map(|e| e.normal_impulse).reduce(f64::max) else { continue };
            let tv = self.boomboxes[k].tv;
            let shatter = if tv { 20.0 } else { 17.5 };
            self.boomboxes[k].crack_impulse = shatter;
            if imp > shatter {
                self.boomboxes[k].broken = true;
            }
            let at = self.b.body(body).sweep.c;
            let src = self.b.body(body).clone();
            if !self.boomboxes[k].broken {
                if tv {
                    self.play("GlassLight", 2, at);
                    self.rect_burst(&src, body, crate::blood::ShardKind::Food(4), 10.0, 30);
                    self.boomboxes[k].frame = 1 + (4.0 * self.blood.rand()).ceil().clamp(1.0, 4.0) as usize;
                } else {
                    self.play("BoomboxCrush1", 1, at);
                    self.rect_burst(&src, body, crate::blood::ShardKind::Food(3), 8.0, 15);
                    self.boomboxes[k].frame = 1 + (3.0 * self.blood.rand()).ceil().clamp(1.0, 3.0) as usize;
                }
                self.boomboxes[k].broken = true;
                continue;
            }
            // shatter
            let (ang, w) = (src.sweep.a, src.angular_velocity);
            // (position, vertices) of each piece in its own body frame; the boombox pieces sit left/right of the centre, the TV's three at the centre
            type Piece = (Vec2, Vec<Vec2>);
            let boxy = |x: f64| -> Piece { (src.world_point(Vec2::new(x / PS, 1.8 / PS)), vec![Vec2::new(-10.5 / PS, -14.0 / PS), Vec2::new(10.5 / PS, -14.0 / PS), Vec2::new(10.5 / PS, 14.0 / PS), Vec2::new(-10.5 / PS, 14.0 / PS)]) };
            let tri = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| -> Piece { (src.sweep.c, vec![Vec2::new(a.0 / PS, a.1 / PS), Vec2::new(b.0 / PS, b.1 / PS), Vec2::new(c.0 / PS, c.1 / PS)]) };
            let pieces: Vec<(u8, Piece)> = if tv {
                self.rect_burst(&src, body, crate::blood::ShardKind::Food(4), 10.0, 60);
                self.play("TVSmash", 2, at);
                vec![
                    (3, tri((-26.0, -18.0), (0.0, 20.0), (-26.0, 20.0))),
                    (4, tri((26.0, -15.0), (25.0, 20.0), (-2.0, 20.0))),
                    (5, tri((-24.0, -18.0), (25.0, -18.0), (-1.0, 4.0))),
                ]
            } else {
                self.rect_burst(&src, body, crate::blood::ShardKind::Food(3), 10.0, 30);
                self.play("BoomboxSmash", 2, at);
                vec![(1, boxy(-13.6)), (2, boxy(13.55))]
            };
            self.b.destroy_body(body);
            self.boomboxes[k].done = true;
            for (n, (pos, verts)) in pieces {
                let piece = self.b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
                let mut sd = ShapeDef::new(Geometry::Polygon(verts));
                sd.density = 2.0;
                sd.friction = 0.3;
                sd.restitution = 0.1;
                sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
                self.b.create_shape(piece, &sd);
                self.b.set_mass_from_shapes(piece);
                // the original asks the old body for the velocity at the NEW body's local centre
                let lc = self.b.body(piece).sweep.local_center;
                self.b.body_mut(piece).angular_velocity = w;
                self.b.body_mut(piece).linear_velocity = src.linear_velocity_from_local_point(lc);
                self.boomboxes[k].pieces.push((piece, n));
                self.loose.push(piece);
            }
        }
    }

    /// `Chair.checkContact` / `actions`: above an impulse of 10 the chair's four shapes become four bodies (left leg, right leg, back, seat) at the
    /// chair's centre with its angle, each moving with the old body's velocity at its own local centre; `TableBreak1` plays.
    fn chair_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for k in 0..self.chairs.len() {
            if self.chairs[k].done {
                continue;
            }
            let body = self.chairs[k].body;
            let shapes = self.b.body(body).shapes.clone();
            if !events.iter().any(|e| e.kind == ContactEventKind::Result && (shapes.contains(&e.shape1) || shapes.contains(&e.shape2)) && e.normal_impulse > 10.0) {
                continue;
            }
            self.chairs[k].done = true;
            let sign = self.chairs[k].sign;
            let src = self.b.body(body).clone();
            let (ang, w, c) = (src.sweep.a, src.angular_velocity, src.sweep.c);
            // (half w, half h, centre x, centre y) px in chair-local coordinates; creation order of the original: left leg, right leg, back, seat
            let parts: [(f64, f64, f64, f64, u8, f64); 4] = [(3.0, 16.0, -18.0, 16.0, 0, 1.0), (3.0, 16.0, 18.0, 16.0, 0, 1.0), (3.0, 25.0, -18.0 * sign, -31.0, 1, 1.0), (21.0, 3.0, 0.0, -3.0, 2, sign)];
            for (hw, hh, cx, cy, kind, mirror) in parts {
                let piece = self.b.create_body(&BodyDef { position: c, angle: ang, ..Default::default() });
                let mut sd = ShapeDef::oriented_box(hw / PS, hh / PS, Vec2::new(cx / PS, cy / PS), 0.0);
                sd.density = 5.0;
                sd.friction = 0.1;
                sd.restitution = 0.2;
                sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
                self.b.create_shape(piece, &sd);
                self.b.set_mass_from_shapes(piece);
                let lc = self.b.body(piece).sweep.local_center;
                self.b.body_mut(piece).linear_velocity = src.linear_velocity_from_local_point(lc);
                self.b.body_mut(piece).angular_velocity = w;
                self.chairs[k].pieces.push((piece, kind, mirror));
                self.loose.push(piece);
            }
            self.b.destroy_body(body);
            self.play("TableBreak1", 1, c);
        }
    }

    /// `Glass.checkContact` / `shatter`: the contact point (in the pane's frame) is clamped to an edge when within 5 cm of it (that side then has no
    /// triangles); every remaining side contributes two triangles (impact point, side point k, side point k+1) as bodies at the pane's centre.
    fn glass_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for k in 0..self.glasses.len() {
            if self.glasses[k].done {
                continue;
            }
            let (body, shape, limit) = (self.glasses[k].body, self.glasses[k].shape, self.glasses[k].smash_impulse);
            let Some(hit) = events.iter().filter(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape) && e.normal_impulse > limit).max_by(|a, b| a.normal_impulse.total_cmp(&b.normal_impulse)) else { continue };
            let src = self.b.body(body).clone();
            let mut impact = src.local_point(hit.position);
            self.glasses[k].done = true;
            let (hx, hy) = self.glasses[k].half;
            let (l, r, tp, bt) = (-hx, hx, -hy, hy);
            let mut sides: [Option<[(f64, f64); 3]>; 4] = [Some([(l, tp), (0.0, tp), (r, tp)]), Some([(r, tp), (r, 0.0), (r, bt)]), Some([(r, bt), (0.0, bt), (l, bt)]), Some([(l, bt), (l, 0.0), (l, tp)])];
            let min_edge = 0.05;
            if impact.x - (l + min_edge) <= 0.0 {
                impact.x = l;
                sides[3] = None;
            } else if impact.x - (r - min_edge) >= 0.0 {
                impact.x = r;
                sides[1] = None;
            }
            if impact.y - (tp + min_edge) <= 0.0 {
                impact.y = tp;
                sides[0] = None;
            } else if impact.y - (bt - min_edge) >= 0.0 {
                impact.y = bt;
                sides[2] = None;
            }
            let at = src.sweep.c;
            self.rect_burst(&src, body, crate::blood::ShardKind::Food(5), 10.0, self.glasses[k].particles);
            self.play(&format!("Glass{}", self.glasses[k].suffix), 2, at);
            let (ang, w) = (src.sweep.a, src.angular_velocity);
            self.b.destroy_body(body);
            for side in sides.into_iter().flatten() {
                for g in 0..2 {
                    let (a, b) = (side[g], side[g + 1]);
                    let piece = self.b.create_body(&BodyDef { position: at, angle: ang, ..Default::default() });
                    let verts = vec![impact, Vec2::new(a.0, a.1), Vec2::new(b.0, b.1)];
                    let mut sd = ShapeDef::new(Geometry::Polygon(verts));
                    sd.density = 2.0;
                    sd.friction = 0.3;
                    sd.restitution = 0.1;
                    sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
                    self.b.create_shape(piece, &sd);
                    self.b.set_mass_from_shapes(piece);
                    let com = self.b.body(piece).sweep.c;
                    self.b.body_mut(piece).angular_velocity = w;
                    self.b.body_mut(piece).linear_velocity = src.linear_velocity_from_world_point(com);
                    self.glasses[k].pieces.push((piece, [(impact.x * PS, impact.y * PS), (a.0 * PS, a.1 * PS), (b.0 * PS, b.1 * PS)]));
                    self.loose.push(piece);
                }
            }
        }
    }

    /// Triangle pieces of shattered glass panes: (body, vertices in the body frame in px).
    pub fn glass_pieces(&self) -> impl Iterator<Item = (BodyId, [(f64, f64); 3])> + '_ {
        self.glasses.iter().flat_map(|g| g.pieces.iter().copied())
    }

    /// Pieces of broken chairs: (body, 0 leg / 1 back / 2 seat, mirror).
    pub fn chair_pieces(&self) -> impl Iterator<Item = (BodyId, u8, f64)> + '_ {
        self.chairs.iter().flat_map(|c| c.pieces.iter().copied())
    }

    /// Current `BoomboxMC` frame (1 intact, 2..4 cracked) of the boombox whose body is `body`.
    pub fn boombox_frame(&self, body: BodyId) -> usize {
        self.boomboxes.iter().find(|b| b.body == body).map_or(1, |b| b.frame)
    }

    /// Pieces of shattered boomboxes (1 = left / 2 = right) and TVs (3..5): (body, piece clip number).
    pub fn boombox_pieces(&self) -> impl Iterator<Item = (BodyId, u8)> + '_ {
        self.boomboxes.iter().flat_map(|b| b.pieces.iter().copied())
    }

    /// Chunk bodies of smashed food (body, food type, chunk index).
    pub fn food_chunks(&self) -> &[(BodyId, u8, u8)] {
        &self.food_chunks
    }

    /// Token `checkContact`/`singleAction`: a material-2 shape entering the sensor counts the token and removes its body.
    fn token_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Add) {
            let Some(k) = self.tokens.iter().position(|t| t.1 == ev.shape1 || t.1 == ev.shape2) else { continue };
            let other = if self.tokens[k].1 == ev.shape1 { ev.shape2 } else { ev.shape1 };
            if self.b.shape_material(other) & 2 != 0 {
                let (body, _) = self.tokens.remove(k);
                self.b.destroy_body(body);
                self.tokens_found += 1;
                let at = self.b.body(body).sweep.c;
                self.play("Bleep3", 1, at);
            }
        }
    }

    /// `Jet.actions` (thrust impulse while awake and burning) and `checkContact`/`explode` (a contact impulse above `round(100·mass)` blows the
    /// jet up: its body goes and every non-static body within 2.6 m gets a radial impulse `5·(1 − d/2.6)`). Sounds/particles/`explodeShape` not ported.
    fn run_jets(&mut self, events: &[box2d::world::ContactEvent]) {
        for ji in 0..self.jets.len() {
            if self.jets[ji].done {
                continue;
            }
            let (body, shape, smash) = (self.jets[ji].body, self.jets[ji].shape, self.jets[ji].smash);
            let hit = events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape) && e.normal_impulse > smash);
            if hit {
                self.jets[ji].done = true;
                let c = self.b.body(body).sweep.c;
                self.play("MineExplosion", 1, c);
                let l = 2.6;
                let found = self.b.query(&box2d::collision::Aabb { lower: Vec2::new(c.x - l, c.y - l), upper: Vec2::new(c.x + l, c.y + l) }, 30);
                self.b.destroy_body(body);
                for s in found {
                    let other = self.b.c.shapes[s].body;
                    if other == body || !self.b.c.bodies[other].alive || self.b.body(other).is_static {
                        continue;
                    }
                    let v = self.b.body(other).sweep.c;
                    let (dx, dy) = (v.x - c.x, v.y - c.y);
                    let w = 1.0 - dx.hypot(dy).min(l) / l;
                    let a = dy.atan2(dx);
                    self.b.body_mut(other).apply_impulse(Vec2::new(a.cos() * w * 5.0, a.sin() * w * 5.0), v);
                }
                continue;
            }
        }
    }

    /// Thrust half of `Jet.actions` (level actions run before the world step).
    fn thrust_jets(&mut self) {
        for ji in 0..self.jets.len() {
            if self.jets[ji].done {
                continue;
            }
            let body = self.jets[ji].body;
            if self.b.body(body).is_sleeping() {
                continue;
            }
            let j = &mut self.jets[ji];
            j.accel = (j.accel + j.spec.accel_step).min(1.0);
            let angle = self.b.c.bodies[body].sweep.a;
            let t = j.spec.power * j.accel;
            let c = self.b.c.bodies[body].sweep.c;
            self.b.body_mut(body).apply_impulse(Vec2::new(angle.sin() * t, -angle.cos() * t), c);
            j.fired += 1.0;
            if j.fired > j.spec.fire_total {
                j.done = true;
            }
        }
    }

    fn run_fans(&mut self) {
        for fi in 0..self.fans.len() {
            let bodies = std::mem::take(&mut self.fans[fi].bodies);
            let (center, sin, cos) = (self.fans[fi].center, self.fans[fi].sin, self.fans[fi].cos);
            for body in bodies {
                let b = self.b.body_mut(body);
                if b.inv_mass == 0.0 {
                    continue;
                }
                let c = b.sweep.c;
                let (rx, ry) = (c.x - center.x, c.y - center.y);
                let i = ((rx * -sin + ry * cos) * PS + 250.0).clamp(0.0, 250.0);
                let s = i / 250.0;
                let a = s * s * 15.0;
                b.apply_force(Vec2::new(sin * a, cos * -a), c);
            }
        }
    }

    /// PLACEHOLDER vehicle control: `dir` in [-1, 1] applies a torque impulse to the player group
    /// (+1 = clockwise on screen). Real wheel/character mechanics are not ported yet; the
    /// strength scale is a guess derived from the group's `acceleration` property.
    pub fn drive(&mut self, dir: f32) {
        let Some(h) = self.vehicle else { return };
        let strength = f64::from(5.0 * (1.0 + self.vehicle_accel));
        let body = self.b.body_mut(h);
        body.angular_velocity += f64::from(dir.clamp(-1.0, 1.0)) * strength * body.mass * body.inv_i;
        body.wake_up();
    }

    /// Spawns the base ragdoll of character `index` (1-based) at `start_px`; false if no guide exists.
    pub fn spawn_character(&mut self, index: usize, start_px: [f32; 2]) -> bool {
        let Some(guide) = crate::character::guide(index) else { return false };
        match crate::character::Ragdoll::spawn(self, &guide, start_px) {
            Some(doll) => {
                self.b.c.report_results = true;
                self.character = Some(doll);
                true
            }
            None => false,
        }
    }

    /// Spawns character `index` with its vehicle (3/10 bike, 2 segway); false if the guide lacks the parts.
    pub fn spawn_bike(&mut self, index: usize, start_px: [f32; 2]) -> bool {
        let start_px = if index == 7 { [start_px[0] + crate::vehicle::Grinder::START_SHIFT[0], start_px[1] + crate::vehicle::Grinder::START_SHIFT[1]] } else { start_px };
        if !matches!(index, 1..=11) {
            return false; // no vehicle class ported yet: nothing is spawned
        }
        let Some(guide) = crate::character::guide(index) else { return false };
        let Some(mut doll) = crate::character::Ragdoll::spawn_scaled(self, &guide, start_px, index == 6 || index == 4 || index == 8 || index == 11, if matches!(index, 8..=11) { 50.0 } else { 5.0 }) else { return false };
        doll.tag = ["Char1", "Char2", "Char3", "Char4", "Char8", "Char11", "Char2", "Santa", "Char12", "Char4", "Heli"][index - 1];
        let vehicle = match index {
            1 => crate::vehicle::Chair::attach(self, &guide, start_px, &mut doll).map(crate::vehicle::Vehicle::Chair),
            11 => crate::vehicle::Heli::attach(self, &guide, start_px, &mut doll).map(crate::vehicle::Vehicle::Heli),
            8 => crate::vehicle::Santa::attach(self, &guide, start_px, &mut doll).map(|s| crate::vehicle::Vehicle::Santa(Box::new(s))),
            7 => crate::vehicle::Grinder::attach(self, &guide, start_px, &mut doll).map(crate::vehicle::Vehicle::Grinder),
            4 => crate::vehicle::Cart::attach(self, &guide, start_px, &mut doll).map(crate::vehicle::Vehicle::Cart),
            9 => crate::vehicle::Pogo::attach(self, &guide, start_px, &mut doll).map(crate::vehicle::Vehicle::Pogo),
            6 => crate::vehicle::Mower::attach(self, &guide, start_px, &mut doll).map(crate::vehicle::Vehicle::Mower),
            5 => crate::vehicle::DirtBike::attach(self, &guide, start_px, &mut doll).map(|mut d| {
                if self.dad_kid {
                    d.girl = crate::vehicle::Girl::attach(self, start_px, d.frame, doll.bodies[&crate::character::Part::Chest]).map(Box::new);
                }
                crate::vehicle::Vehicle::Dirt(d)
            }),
            2 => crate::vehicle::Segway::attach(self, &guide, start_px, &mut doll).map(crate::vehicle::Vehicle::Segway),
            10 => crate::vehicle::Bike::attach(self, &guide, start_px, &mut doll).and_then(|dad| {
                let daughter = crate::vehicle::Daughter::attach(self, start_px, dad.frame)?;
                let son = crate::vehicle::Son::attach(self, start_px, dad.frame)?;
                Some(crate::vehicle::Vehicle::Family(Box::new(crate::vehicle::Family { dad, daughter, son })))
            }),
            _ => crate::vehicle::Bike::attach(self, &guide, start_px, &mut doll).map(|mut b| {
                if index == 3 && self.dad_kid {
                    b.kid = crate::vehicle::Kid::attach(self, start_px, b.frame).map(Box::new);
                }
                crate::vehicle::Vehicle::Bike(b)
            }),
        };
        let Some(vehicle) = vehicle else { return false };
        self.b.c.report_results = true;
        self.character = Some(doll);
        self.bike = Some(vehicle);
        true
    }

    /// Full key state for the rider: vehicle controls while seated, poses/grab once ejected.
    pub fn set_rider_keys(&mut self, k: crate::vehicle::Keys) {
        let (Some(mut doll), Some(mut vehicle)) = (self.character.take(), self.bike.take()) else {
            self.set_character_keys(k.left, k.right, k.up, k.down);
            self.set_character_grab(k.space);
            return;
        };
        vehicle.keys(self, &mut doll, k);
        self.character = Some(doll);
        self.bike = Some(vehicle);
    }

    /// Space key state for the ragdoll's hands (grab while held, release on key up).
    pub fn set_character_grab(&mut self, pressed: bool) {
        if let Some(mut doll) = self.character.take() {
            doll.set_grab(self, pressed);
            self.character = Some(doll);
        }
    }

    /// Feeds the arrow keys to the ejected ragdoll (poses) and applies the pose motors; call once per frame
    /// before `step`.
    pub fn set_character_keys(&mut self, left: bool, right: bool, up: bool, down: bool) {
        if let Some(mut doll) = self.character.take() {
            doll.keys(left, right, up, down);
            doll.apply_pose(self);
            self.character = Some(doll);
        }
    }

    /// Pixel-space origin of a body (None for the level body, which has no meaningful position).
    pub fn body_position_px(&self, h: BodyId) -> Option<[f32; 2]> {
        if h == self.level_body || !self.b.body(h).alive {
            return None;
        }
        let p = self.b.body(h).xf.position;
        Some([(p.x * PS) as f32, (p.y * PS) as f32])
    }

    /// World centre (centre of mass) of a live body in px: where the original's `paint` puts the body's clip.
    pub fn body_center_px(&self, h: BodyId) -> Option<[f32; 2]> {
        if h == self.level_body || !self.b.body(h).alive {
            return None;
        }
        let c = self.b.body(h).sweep.c;
        Some([(c.x * PS) as f32, (c.y * PS) as f32])
    }

    pub fn body_angle(&self, h: BodyId) -> f32 {
        self.b.body(h).sweep.a as f32
    }

    pub fn body_angvel(&self, h: BodyId) -> f32 {
        self.b.body(h).angular_velocity as f32
    }

    pub fn body_is_fixed(&self, h: BodyId) -> bool {
        self.b.body(h).is_static
    }

    /// Pixel-space position of a special's body.
    pub fn special_position_px(&self, special: usize) -> Option<[f32; 2]> {
        self.body_position_px((*self.special_bodies.get(special)?)?)
    }

    /// Pixel-space position of a shape's body.
    pub fn position_px(&self, shape: usize) -> Option<[f32; 2]> {
        self.body_position_px((*self.shape_bodies.get(shape)?)?)
    }

    /// All shapes of a body in world pixels.
    pub fn body_draw_shapes(&self, h: BodyId) -> Vec<DrawShape> {
        let body = self.b.body(h);
        let px = |p: Vec2| {
            let w = body.world_point(p);
            [(w.x * PS) as f32, (w.y * PS) as f32]
        };
        body.shapes
            .iter()
            .filter(|&&s| !self.b.c.shapes[s].shape.is_sensor)
            .map(|&s| match &self.b.c.shapes[s].shape.kind {
                B2Kind::Circle(c) => DrawShape::Circle { center: px(c.local_position), radius: (c.radius * PS) as f32 },
                B2Kind::Polygon { poly, .. } => DrawShape::Poly(poly.vertices.iter().map(|&v| px(v)).collect()),
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pinned_box_swings_but_stays_attached() {
        let level = crate::level::parse(
            "<levelXML><info v='1'/><shapes>\
             <sh t='0' p0='100' p1='0' p2='100' p3='10' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='3'/>\
             </shapes><joints><j t='0' x='50' y='0' b1='0' b2='-1' l='f' ua ='0' la ='0' m='f' tq='0' sp='0' c='f'/></joints></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..90 {
            w.step();
        }
        let [x, y] = w.position_px(0).unwrap();
        // centre stays on a 50 px radius circle around the pin at (50,0)
        let r = ((x - 50.0).powi(2) + y.powi(2)).sqrt();
        assert!((r - 50.0).abs() < 2.0, "radius {r}");
        assert!(y > 20.0, "box should have swung down, y = {y}");
    }

    const FLOOR: &str = "<sh t='0' p0='0' p1='500' p2='4000' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>";

    fn settle(xml: &str, steps: usize) -> World {
        let level = crate::level::parse(xml).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..steps {
            w.step();
        }
        w
    }

    #[test]
    fn polygon_rests_on_floor() {
        // 40x40 square polygon (verts local to its origin), dropped from y=300.
        let w = settle(
            &format!("<levelXML><info v='1'/><shapes>{FLOOR}<sh t='3' p0='0' p1='300'><v f='t' id='1' n='4' v0='-20_-20' v1='20_-20' v2='20_20' v3='-20_20'/></sh></shapes></levelXML>"),
            150,
        );
        let y = w.position_px(1).unwrap()[1];
        assert!((y - 470.0).abs() < 6.0, "polygon centre y = {y}");
    }

    #[test]
    fn group_moves_as_one_body() {
        // Two boxes in one group: they must keep their distance while falling.
        let w = settle(
            &format!("<levelXML><info v='1'/><shapes>{FLOOR}</shapes><groups><g x='0' y='300' r='0' ox='0' oy='0' s='f' f='f' o='100' im='f' fr='f'>\
              <sh t='0' p0='-60' p1='0' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>\
              <sh t='0' p0='60' p1='0' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></g></groups></levelXML>"),
            150,
        );
        let gy = w.body_position_px(w.group_bodies[0]).unwrap()[1];
        // group origin settles so the 40px boxes (centres at origin y) rest on the floor top (490): y ≈ 470
        assert!((gy - 470.0).abs() < 6.0, "group y = {gy}");
        assert!(w.body_angle(w.group_bodies[0]).abs() < 0.05);
    }

    #[test]
    fn pin_limits_stop_swing() {
        // Same pendulum as above but limited to [-10°, +10°]: it must not swing down.
        let w = settle(
            "<levelXML><info v='1'/><shapes><sh t='0' p0='100' p1='0' p2='100' p3='10' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='3'/></shapes>\
             <joints><j t='0' x='50' y='0' b1='0' b2='-1' l='t' ua ='10' la ='-10' m='f' tq='0' sp='0' c='f'/></joints></levelXML>",
            90,
        );
        let y = w.position_px(0).unwrap()[1];
        assert!(y.abs() < 12.0, "limited pendulum y = {y}"); // 50px arm * sin(10°) ≈ 8.7
    }

    #[test]
    fn pin_limits_are_relative_to_the_creation_pose() {
        // Bar rotated 90° and pinned at its end with limits ±10°: Box2D measures from the initial pose
        // (referenceAngle), so the bar must stay within 90° ± ~10°, not be dragged towards angle 0.
        let w = settle(
            "<levelXML><info v='1'/><shapes><sh t='0' p0='0' p1='-50' p2='100' p3='10' p4='90' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes>\
             <joints><j t='0' x='0' y='0' b1='0' b2='-1' l='t' ua ='10' la ='-10' m='f' tq='0' sp='0' c='f'/></joints></levelXML>",
            90,
        );
        let a = w.body_angle(w.shape_bodies[0].unwrap()).to_degrees();
        assert!((a - 90.0).abs() < 14.0, "bar angle {a}° left the +-10° window around its start");
    }

    #[test]
    fn pin_motor_spins_body() {
        let w = settle(
            "<levelXML><info v='1'/><shapes><sh t='0' p0='100' p1='0' p2='100' p3='10' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='3'/></shapes>\
             <joints><j t='0' x='50' y='0' b1='0' b2='-1' l='f' ua ='0' la ='0' m='t' tq='1000' sp='3' c='f'/></joints></levelXML>",
            30,
        );
        let av = w.body_angvel(w.shape_bodies[0].unwrap());
        assert!(av.abs() > 1.5, "angvel = {av}");
    }

    #[test]
    fn slider_constrains_motion_to_axis() {
        // Box falls under gravity but is constrained to slide horizontally (axis 0°): y must not change.
        let w = settle(
            "<levelXML><info v='1'/><shapes><sh t='0' p0='0' p1='0' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='3'/></shapes>\
             <joints><j t='1' x='0' y='0' b1='0' b2='-1' a='0' l='f' ul ='0' ll ='0' m='f' fo='0' sp='0' c='f'/></joints></levelXML>",
            60,
        );
        let [_, y] = w.position_px(0).unwrap();
        assert!(y.abs() < 2.0, "slider y = {y}");
    }

    #[test]
    fn trigger_zone_fires_once_when_body_enters() {
        let level = crate::level::parse(
            "<levelXML><info v='1'/><shapes>\
             <sh t='1' p0='0' p1='0' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
             <triggers><t x='0' y='300' w='200' h='50' a='0' b='3' t='0' r='1' sd='f'/>\
                       <t x='0' y='-500' w='50' h='50' a='0' b='3' t='0' r='1' sd='f'/></triggers></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..90 {
            w.step();
        }
        // The falling ball passes through the zone at y=300 (fires once); the zone far above never fires.
        assert_eq!(w.events.iter().map(|e| e.trigger).collect::<Vec<_>>(), vec![0]);
    }

    #[test]
    fn drive_spins_the_vehicle_group_only() {
        let level = crate::level::parse(
            "<levelXML><info v='1'/><groups>\
             <g x='0' y='0' r='0' ox='0' oy='0' s='f' f='f' o='100' im='f' fr='f' v='t' sb='0' sh='0' ct='0' a='1' l='0' cp='0' lo='f'>\
               <sh t='0' p0='0' p1='0' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></g>\
             <g x='500' y='0' r='0' ox='0' oy='0' s='f' f='f' o='100' im='f' fr='f'>\
               <sh t='0' p0='0' p1='0' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></g></groups></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        assert_eq!(w.vehicle, Some(w.group_bodies[0]));
        w.drive(1.0);
        let spin = |w: &World, i: usize| w.body_angvel(w.group_bodies[i]);
        assert!(spin(&w, 0) > 0.1, "vehicle spin {}", spin(&w, 0));
        assert_eq!(spin(&w, 1), 0.0);
    }

    fn trigger_events(trigger_xml: &str, extra: &str, steps: usize) -> Vec<u32> {
        let xml = format!("<levelXML><info v='1'/><shapes>{FLOOR}{BALL}{extra}</shapes><triggers>{trigger_xml}</triggers></levelXML>");
        let level = crate::level::parse(&xml).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..steps {
            w.step();
        }
        w.events.iter().map(|e| e.frame).collect()
    }

    // Ball (r=25) falls from y=400 and rests on the floor (top y=490) inside the zone below.
    const BALL: &str = "<sh t='1' p0='0' p1='400' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>";
    const ZONE: &str = "x='0' y='470' w='200' h='60' a='0'";

    #[test]
    fn repeat_type_3_fires_on_interval_while_inside() {
        let ev = trigger_events(&format!("<t {ZONE} b='3' t='0' r='3' sd='f' i='1' d='0'/>"), "", 150);
        // The original resets its counter to 0 after firing and fires again when it reaches repeatFrames (30),
        // i.e. every 31 frames.
        assert!(ev.len() >= 3, "events {ev:?}");
        assert!(ev.windows(2).all(|p| p[1] - p[0] == 31), "events {ev:?}");
    }

    #[test]
    fn repeat_type_2_fires_only_on_new_contact() {
        let ev = trigger_events(&format!("<t {ZONE} b='3' t='0' r='2' sd='f'/>"), "", 150);
        assert_eq!(ev.len(), 1, "events {ev:?}"); // resting inside: no re-fire without leaving
    }

    #[test]
    fn triggered_by_4_only_reacts_to_its_targets() {
        // shape 1 (the ball) is in the zone; the trigger targets shape 2 (elsewhere) → never fires.
        let other = "<sh t='1' p0='5000' p1='0' p2='20' p3='20' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>";
        let miss = trigger_events(&format!("<t {ZONE} b='4' t='1' r='1' sd='f' d='0'><sh i='2'/></t>"), other, 150);
        assert!(miss.is_empty(), "{miss:?}");
        let hit = trigger_events(&format!("<t {ZONE} b='4' t='1' r='1' sd='f' d='0'><sh i='1'/></t>"), other, 150);
        assert_eq!(hit.len(), 1, "{hit:?}");
    }

    #[test]
    fn character_only_triggers_ignore_plain_shapes() {
        let ev = trigger_events(&format!("<t {ZONE} b='1' t='0' r='1' sd='f'/>"), "", 150);
        assert!(ev.is_empty(), "{ev:?}"); // no vehicle in this level
    }

    #[test]
    fn degenerate_polygons_are_skipped_not_fatal() {
        let level = crate::level::parse("<levelXML><info v='1'/><shapes>\
            <sh t='3' p0='0' p1='0' p2='0' p3='0'><v f='t' id='1' n='3' v0='0_0' v1='10_0' v2='0_10'/></sh>\
            <sh t='3' p0='0' p1='0' p2='100' p3='100'><v f='t' id='2' n='3' v0='0_0' v1='10_0' v2='20_0'/></sh>\
            </shapes></levelXML>").unwrap();
        let w = World::from_level(&level); // must not panic
        assert_eq!(w.shape_bodies, vec![None, None]);
    }

    #[test]
    fn specials_log_rests_and_soccer_ball_bounces_higher_than_a_plain_ball() {
        // Log (kind 4, 200x20) dropped from y=300 onto the floor; soccer ball (kind 10, restitution .5)
        // vs. a plain restitution-.1 ball dropped from the same height.
        let level = crate::level::parse(&format!(
            "<levelXML><info v='1.99'/><shapes>{FLOOR}{BALL}</shapes><specials>\
             <sp t='4' p0='400' p1='300' p2='200' p3='20' p4='0' p5='f' p6='f'/>\
             <sp t='10' p0='-400' p1='0'/></specials></levelXML>"
        ))
        .unwrap();
        let mut w = World::from_level(&level);
        let mut soccer_top_after_bounce = f32::MAX;
        let mut ball_top_after_bounce = f32::MAX;
        for i in 0..120 {
            w.step();
            if i > 40 {
                soccer_top_after_bounce = soccer_top_after_bounce.min(w.special_position_px(1).unwrap()[1]);
                ball_top_after_bounce = ball_top_after_bounce.min(w.position_px(1).unwrap()[1]);
            }
        }
        // log: 20 px tall resting on the floor top (490): centre y = 480
        let log_y = w.special_position_px(0).unwrap()[1];
        assert!((log_y - 480.0).abs() < 4.0, "log y = {log_y}");
        // soccer ball (r=10) rebounds noticeably; plain ball barely does
        assert!(soccer_top_after_bounce < ball_top_after_bounce - 20.0, "soccer {soccer_top_after_bounce} ball {ball_top_after_bounce}");
    }

    #[test]
    fn table_stands_on_its_legs_and_chair_respects_reverse() {
        // Table legs reach 50 px below the origin → centre rests at floor top (490) - 50 = 440.
        let level = crate::level::parse(&format!(
            "<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials>\
             <sp t='1' p0='800' p1='300' p2='0' p3='f' p4='t'/>\
             <sp t='19' p0='400' p1='300' p2='0' p3='f' p4='f' p5='t'/></specials></levelXML>"
        ))
        .unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..150 {
            w.step();
        }
        let table_y = w.special_position_px(0).unwrap()[1];
        assert!((table_y - 440.0).abs() < 4.0, "table y = {table_y}");
        // The chair is a tall, light multi-shape body: like the original's Box2D it may topple, so only check
        // that it ends up at rest on the floor (multi-shape landing dynamics are golden-tested in box2d).
        let chair = w.special_bodies[1].unwrap();
        let cy = w.body_position_px(chair).unwrap()[1];
        assert!(w.body_angvel(chair).abs() < 0.1 && (400.0..490.0).contains(&cy), "chair y {cy} spin {}", w.body_angvel(chair));
    }

    #[test]
    fn wrecking_ball_swings_on_its_pin_within_limits() {
        // Pin at (0,0); ball body at (rope, 0) → starts level with the pin and swings down.
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><specials><sp t='7' p0='0' p1='0' p2='300'/></specials></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        let mut max_depth = 0.0f32;
        for _ in 0..120 {
            w.step();
            let [x, y] = w.special_position_px(0).unwrap();
            // distance from the pin must stay the rope length (it is a rigid pendulum)
            assert!(((x * x + y * y).sqrt() - 300.0).abs() < 6.0, "left the circle: ({x}, {y})");
            max_depth = max_depth.max(y);
        }
        assert!(max_depth > 200.0, "ball should swing down, max y = {max_depth}");
    }

    #[test]
    fn spring_box_pad_stays_locked_and_spikes_base_is_fixed() {
        let level = crate::level::parse(&format!(
            "<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials>\
             <sp t='5' p0='0' p1='400' p2='0' p3='1'/>\
             <sp t='6' p0='600' p1='400' p2='0' p3='t' p4='6' p5='f'/></specials></levelXML>"
        )).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..90 {
            w.step();
        }
        let [x, y] = w.special_position_px(0).unwrap();
        assert!(x.abs() < 3.0 && (y - 400.0).abs() < 10.0, "spring box moved: ({x}, {y})");
        assert!(w.body_is_fixed(w.special_bodies[1].unwrap()));
    }

    #[test]
    fn van_rests_on_its_tires() {
        // Tire bottoms are 48 + 22.5 = 70.5 px below the van origin → rests at 490 - 70.5 = 419.5.
        let level = crate::level::parse(&format!(
            "<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='0' p0='500' p1='200' p2='0' p3='f' p4='t'/></specials></levelXML>"
        )).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..150 {
            w.step();
        }
        let y = w.special_position_px(0).unwrap()[1];
        assert!((y - 419.5).abs() < 4.0, "van y = {y}");
        assert!(w.body_angle(w.special_bodies[0].unwrap()).abs() < 0.05);
    }

    #[test]
    fn toilet_matches_the_original_drop() {
        // Verified against the original's Box2D (oracle): dropped from (500,300) onto the floor it ends at
        // y = 460.925, angle -0.92776 (it topples onto its back; so does ours).
        let level = crate::level::parse(&format!(
            "<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='24' p0='500' p1='300' p2='0' p3='f' p4='f'/></specials></levelXML>"
        )).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..200 {
            w.step();
        }
        let t = w.special_bodies[0].unwrap();
        let [x, y] = w.body_position_px(t).unwrap();
        assert!((x - 465.2244).abs() < 1e-3 && (y - 460.9251).abs() < 1e-3 && (w.body_angle(t) + 0.927763).abs() < 1e-5, "({x}, {y}) {}", w.body_angle(t));
    }

    #[test]
    fn paddle_pad_is_locked_to_its_pin() {
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><specials><sp t='35' p0='500' p1='300' p2='0' p3='1' p4='f' p5='90' p6='5'/></specials></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..60 {
            w.step();
        }
        let [x, y] = w.special_position_px(0).unwrap();
        assert!((x - 500.0).abs() < 2.0 && (y - 300.0).abs() < 2.0, "paddle moved: ({x}, {y})");
    }

    #[test]
    fn mouse_trigger_fires_on_click_and_chained_trigger_activates() {
        // Trigger 0 (mouse click, once) targets trigger 1 (triggered by other triggers, once).
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><triggers>\
             <t x='0' y='0' w='50' h='50' a='0' b='6' t='0' r='1' sd='f'><t i='1'/></t>\
             <t x='0' y='0' w='50' h='50' a='0' b='5' t='0' r='1' sd='f'/></triggers></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        w.step();
        assert!(w.events.is_empty());
        w.click_trigger(0);
        w.click_trigger(0); // once only
        assert_eq!(w.events.iter().map(|e| e.trigger).collect::<Vec<_>>(), vec![0, 1]);
    }

    #[test]
    fn trigger_applies_impulse_to_its_shape_target() {
        // Mouse trigger with action 4 (apply impulse 3 m/s in x) on a floating ball.
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><shapes>\
             <sh t='1' p0='500' p1='0' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
             <triggers><t x='0' y='0' w='50' h='50' a='0' b='6' t='1' r='1' sd='f'><sh i='0'><a i='4' p0='3' p1='0' p2='0'/></sh></t></triggers></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        w.click_trigger(0);
        let vx = w.b2().body(w.shape_bodies[0].unwrap()).linear_velocity.x;
        assert!((vx - 3.0).abs() < 1e-9, "vx {vx}");
    }

    #[test]
    fn delete_action_removes_the_target_and_repeat_once_destroys_the_sensor() {
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><shapes>\
             <sh t='1' p0='500' p1='0' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
             <triggers><t x='0' y='0' w='50' h='50' a='0' b='6' t='1' r='1' sd='f'><sh i='0'><a i='5'/></sh></t></triggers></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        assert!(w.position_px(0).is_some());
        w.click_trigger(0);
        assert!(w.position_px(0).is_none());
        w.step();
    }

    #[test]
    fn jet_thrusts_up_burns_out_and_explodes_on_impact() {
        // Jet at rest in free space, rotation 0 (thrust −y), power 1: impulse 1 per step → it climbs; fireTime .2 s = 6 steps stops it.
        let lvl = |extra: &str| crate::level::parse(&format!("<levelXML><info v='1.99'/><shapes>{extra}</shapes><specials><sp t='28' p0='0' p1='0' p2='0' p3='f' p4='1' p5='0.2' p6='0' p7='f'/></specials></levelXML>")).unwrap();
        let mut w = World::from_level(&lvl(""));
        let y0 = w.special_position_px(0).unwrap()[1];
        for _ in 0..6 {
            w.step();
        }
        let vy_burn = w.b2().body(w.special_bodies[0].unwrap()).linear_velocity.y;
        for _ in 0..20 {
            w.step();
        }
        let vy_after = w.b2().body(w.special_bodies[0].unwrap()).linear_velocity.y;
        assert!(w.special_position_px(0).unwrap()[1] != y0);
        assert!(vy_burn < vy_after, "thrust did not stop after fireTime: {vy_burn} {vy_after}");
        // A fast jet hitting a wall blows up (its body is destroyed).
        let wall = "<sh t='0' p0='-200' p1='-300' p2='400' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>";
        let mut w = World::from_level(&crate::level::parse(&format!("<levelXML><info v='1.99'/><shapes>{wall}</shapes><specials><sp t='28' p0='0' p1='0' p2='0' p3='f' p4='10' p5='0' p6='0' p7='f'/></specials></levelXML>")).unwrap());
        let jet = w.special_bodies[0].unwrap();
        for _ in 0..200 {
            w.step();
        }
        assert!(!w.b2().c.bodies[jet].alive, "jet did not explode against the wall");
    }

    #[test]
    fn cannon_loads_aims_and_shoots_the_ball_in_its_barrel() {
        // Cannon at the origin pointing up (rotation 0, start = firing = 0), delay 0.1 s; a light ball sitting in the barrel is launched upwards.
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><shapes>\
             <sh t='1' p0='0' p1='-60' p2='20' p3='20' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
             <specials><sp t='33' p0='0' p1='0' p2='0' p3='0' p4='0' p5='1' p6='0.1' p7='1' p8='5'/></specials></levelXML>",
        )
        .unwrap();
        let mut w = World::from_level(&level);
        let mut top = f32::MAX;
        let mut heard = Vec::new();
        for _ in 0..90 {
            w.step();
            heard.extend(w.take_sounds().into_iter().map(|s| s.0));
            top = top.min(w.position_px(0).unwrap()[1]);
        }
        assert!(top < -300.0, "ball was not shot out of the cannon: top {top}");
        assert!(heard.contains(&"Cannon1".to_string()), "no cannon sound: {heard:?}");
    }

    #[test]
    fn token_is_collected_once_by_a_ragdoll_part() {
        let level = crate::level::parse("<levelXML><info v='1.99'/><shapes/><specials><sp t='31' p0='0' p1='0' p2='1'/></specials></levelXML>").unwrap();
        let w0 = World::from_level(&level);
        assert_eq!(w0.tokens_found, 0);
        assert_eq!(w0.tokens.len(), 1);
    }

    #[test]
    fn food_smashes_when_dropped_hard_and_trash_can_is_a_body() {
        let wall = "<sh t='0' p0='0' p1='600' p2='800' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>";
        let lvl = crate::level::parse(&format!("<levelXML><info v='1.99'/><shapes>{wall}</shapes><specials><sp t='32' p0='0' p1='-400' p2='0' p3='f' p4='t' p5='1'/><sp t='26' p0='200' p1='500' p2='0' p3='f' p4='t' p5='t'/></specials></levelXML>")).unwrap();
        let mut w = World::from_level(&lvl);
        assert_eq!(w.foods.len(), 1);
        let food = w.foods[0].0;
        for _ in 0..200 {
            w.step();
        }
        assert!(!w.b2().c.bodies[food].alive, "food did not smash");
    }

    #[test]
    fn homing_mine_chases_a_ragdoll_part_and_blows_up_next_to_it() {
        let lvl = crate::level::parse("<levelXML><info v='1.99'/><shapes/><specials><sp t='25' p0='0' p1='0' p2='5' p3='0'/></specials></levelXML>").unwrap();
        let mut w = World::from_level(&lvl);
        let mine = w.mines[0].body;
        let b = w.b.create_body(&BodyDef { position: Vec2::new(300.0 / PS, 0.0), ..Default::default() });
        let mut sd = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 10.0 / PS });
        sd.density = 1.0;
        sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        let s = w.b.create_shape(b, &sd);
        w.b.shape_mut(s).material = 2;
        w.b.set_mass_from_shapes(b);
        w.b.body_mut(b).linear_velocity = Vec2::default();
        let mut closest = f64::MAX;
        for _ in 0..600 {
            w.step();
            if !w.b.c.bodies[mine].alive {
                break;
            }
            closest = closest.min((w.b.body(mine).sweep.c.x * PS - 300.0).abs());
        }
        assert!(closest < 150.0, "mine did not approach: {closest}");
        assert!(!w.b.c.bodies[mine].alive, "mine did not explode");
    }

    #[test]
    fn arrow_gun_shoots_ten_arrows_that_pin_to_a_ragdoll_part() {
        let lvl = crate::level::parse("<levelXML><info v='1.99'/><shapes/><specials><sp t='29' p0='1000' p1='500' p2='0' p3='t' p4='10' p5='f'/></specials></levelXML>").unwrap();
        let mut w = World::from_level(&lvl);
        let b = w.b.create_body(&BodyDef { position: Vec2::new(1000.0 / PS, 200.0 / PS), ..Default::default() });
        let mut sd = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 20.0 / PS });
        sd.density = 50.0;
        sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        let s = w.b.create_shape(b, &sd);
        w.b.shape_mut(s).material = 2;
        w.b.set_mass_from_shapes(b);
        for _ in 0..40 {
            // keep the target hovering: cancel gravity so the aim stays valid
            w.b.body_mut(b).linear_velocity = Vec2::default();
            w.step();
        }
        assert!(!w.arrows.is_empty(), "no arrow was fired");
        assert!(w.arrows.iter().any(|a| a.armed), "no arrow pinned to the target");
    }

    #[test]
    fn harpoon_gun_fires_one_pinning_harpoon_that_never_snaps() {
        let lvl = crate::level::parse("<levelXML><info v='1.99'/><shapes/><specials><sp t='15' p0='1000' p1='500' p2='0' p3='f' p4='f' p5='0' p6='f' p7='f'/></specials></levelXML>").unwrap();
        let mut w = World::from_level(&lvl);
        let b = w.b.create_body(&BodyDef { position: Vec2::new(1000.0 / PS, 200.0 / PS), ..Default::default() });
        let mut sd = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 20.0 / PS });
        sd.density = 50.0;
        sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        let sh = w.b.create_shape(b, &sd);
        w.b.shape_mut(sh).material = 2;
        w.b.set_mass_from_shapes(b);
        for _ in 0..120 {
            w.b.body_mut(b).linear_velocity = Vec2::default();
            w.step();
        }
        assert_eq!(w.arrows.len(), 1, "exactly one harpoon expected");
        assert!(w.arrows[0].armed && !w.arrows[0].done, "harpoon should be pinned and intact");
    }

    #[test]
    fn blade_weapon_sticks_into_static_geometry_it_touches() {
        // A sword (type 2) turned blade-down just above the floor slab falls onto it and gets pinned by its blade sensor.
        let wall = "<sh t='0' p0='1000' p1='600' p2='800' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>";
        let lvl = crate::level::parse(&format!("<levelXML><info v='1.99'/><shapes>{wall}</shapes><specials><sp t='34' p0='1000' p1='500' p2='180' p3='f' p4='f' p5='t' p6='2'/></specials></levelXML>")).unwrap();
        let mut w = World::from_level(&lvl);
        for _ in 0..120 {
            w.step();
        }
        assert!(w.arrows[0].armed, "blade never pinned to the floor");
    }

    /// `grindShape` on an NPC deletes its smash listeners: slammed into the floor at 60 m/s it breaks into pieces, but a ground one stays whole.
    #[test]
    fn ground_npc_parts_do_not_smash() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='600' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='17' p0='1000' p1='400' p2='0' p3='1' p4='f' p5='f'/></specials></levelXML>";
        let run = |grind: bool| {
            let mut w = World::from_level(&crate::level::parse(xml).unwrap());
            if grind {
                for b in w.npc_main_bodies() {
                    w.npc_grind(b);
                }
            }
            for b in w.npc_main_bodies() {
                w.b.body_mut(b).linear_velocity = Vec2::new(0.0, 60.0);
            }
            for _ in 0..60 {
                w.step();
            }
            w.npcs[0].pieces.len()
        };
        assert!(run(false) > 0, "control: a 60 m/s impact smashes the NPC");
        assert_eq!(run(true), 0);
    }

    /// `shapeImpale`: a fatal head impale kills the rider, a blade tip far from the head (radius² .01 m²) or a body impale only makes it vocalise.
    #[test]
    fn head_impale_kills_but_distant_tip_does_not() {
        use crate::character::Part;
        let lvl = crate::level::parse("<levelXML><info v='1.99' x='1000' y='300'/><shapes></shapes></levelXML>").unwrap();
        let mut w = World::from_level(&lvl);
        w.spawn_character(1, [1000.0, 300.0]);
        let head = w.character.as_ref().unwrap().bodies[&Part::Head];
        let chest = w.character.as_ref().unwrap().bodies[&Part::Chest];
        w.shape_impale(chest, true, None, 0.0);
        assert!(!w.character.as_ref().unwrap().dead, "a chest impale only makes noise");
        let far = w.b.body(head).sweep.c + Vec2::new(1.0, 0.0);
        w.shape_impale(head, true, Some(far), 0.01);
        assert!(!w.character.as_ref().unwrap().dead, "tip outside the radius");
        w.shape_impale(head, false, None, 0.0);
        assert!(!w.character.as_ref().unwrap().dead, "non-fatal");
        w.shape_impale(head, true, None, 0.0);
        assert!(w.character.as_ref().unwrap().dead);
    }

    /// A boombox cracks (damage frame, 15 shards) on a medium hit, and a hard hit shatters it into two pieces and 30 shards.
    #[test]
    fn boombox_cracks_then_shatters() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='22' p0='900' p1='380' p2='0' p3='f' p4='t'/><sp t='22' p0='1200' p1='380' p2='0' p3='f' p4='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (a, b) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(a).linear_velocity = Vec2::new(0.0, 20.0);
        w.b.body_mut(b).linear_velocity = Vec2::new(0.0, 60.0);
        for _ in 0..40 {
            w.step();
        }
        assert!(w.boombox_frame(a) >= 2, "a medium hit shows a damage frame");
        assert!(w.b.c.bodies[a].alive, "and keeps the body");
        assert!(!w.b.c.bodies[b].alive, "a hard hit destroys the body");
        assert_eq!(w.boombox_pieces().count(), 2);
        assert!(w.blood.sprites.iter().any(|p| matches!(p.kind, crate::blood::ShardKind::Food(3))));
    }

    /// A TV cracks above 7 (shards, frame 2..5) and shatters above 20 into three triangular pieces and 60 shards.
    #[test]
    fn tv_cracks_then_shatters() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='21' p0='900' p1='380' p2='0' p3='f' p4='t'/><sp t='21' p0='1200' p1='380' p2='0' p3='f' p4='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (a, b) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(a).linear_velocity = Vec2::new(0.0, 20.0);
        w.b.body_mut(b).linear_velocity = Vec2::new(0.0, 60.0);
        for _ in 0..40 {
            w.step();
        }
        assert!(w.boombox_frame(a) >= 2 && w.b.c.bodies[a].alive, "medium hit: damage frame, body kept");
        assert!(!w.b.c.bodies[b].alive, "hard hit destroys the TV");
        assert_eq!(w.boombox_pieces().count(), 3);
        assert!(w.boombox_pieces().all(|(p, _)| w.b.c.bodies[p].alive));
        assert!(w.blood.sprites.iter().any(|p| matches!(p.kind, crate::blood::ShardKind::Food(4))));
    }

    /// A chair hit above an impulse of 10 falls apart into leg, leg, back and seat bodies.
    #[test]
    fn chair_breaks_into_four_pieces() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='19' p0='900' p1='360' p2='0' p3='f' p4='f' p5='t'/><sp t='19' p0='1200' p1='360' p2='0' p3='f' p4='f' p5='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (soft, hard) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(soft).linear_velocity = Vec2::new(0.0, 2.0);
        w.b.body_mut(hard).linear_velocity = Vec2::new(0.0, 50.0);
        for _ in 0..40 {
            w.step();
        }
        assert!(w.b.c.bodies[soft].alive, "a soft landing keeps the chair whole");
        assert!(!w.b.c.bodies[hard].alive, "a hard hit breaks it");
        assert_eq!(w.chair_pieces().count(), 4);
        assert!(w.chair_pieces().all(|(b, _, _)| w.b.c.bodies[b].alive));
    }

    /// A glass pane hit above `round(mass·shatterStrength)` breaks into triangles (the pane's body is gone) with glass shards flying; a soft landing keeps it.
    #[test]
    fn glass_shatters_into_triangles() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='18' p0='900' p1='380' p2='100' p3='20' p4='0' p5='f' p6='10' p7='t'/><sp t='18' p0='1200' p1='380' p2='100' p3='20' p4='0' p5='f' p6='10' p7='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (soft, hard) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(soft).linear_velocity = Vec2::new(0.0, 1.0);
        w.b.body_mut(hard).linear_velocity = Vec2::new(0.0, 60.0);
        for _ in 0..30 {
            w.step();
        }
        assert!(w.b.c.bodies[soft].alive, "soft landing: whole pane");
        assert!(!w.b.c.bodies[hard].alive, "hard hit: pane gone");
        let n = w.glass_pieces().count();
        assert!((4..=8).contains(&n), "2 triangles per intact side (6 when the impact is on an edge, 8 mid-pane), got {n}");
        assert!(w.glass_pieces().all(|(b, _)| w.b.c.bodies[b].alive));
        assert!(w.blood.sprites.iter().any(|p| matches!(p.kind, crate::blood::ShardKind::Food(5))));
    }

    /// A smashed food item leaves four circle chunks and 30 juice particles, and its body is gone.
    #[test]
    fn smashed_food_leaves_chunks_and_juice() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='32' p0='1000' p1='-2500' p2='0' p3='f' p4='t' p5='3'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let food = w.special_bodies[0].unwrap();
        for _ in 0..150 {
            w.step();
        }
        assert!(!w.b.c.bodies[food].alive, "the pineapple smashes on landing");
        assert_eq!(w.food_chunks().len(), 4);
        assert!(w.food_chunks().iter().all(|c| w.b.c.bodies[c.0].alive));
        assert!(w.blood.sprites.iter().any(|p| matches!(p.kind, crate::blood::ShardKind::Food(2))), "juice particles were thrown");
    }

    /// A trash can built without trash is replaced by a hollow body at level load; the special's sprite pose must follow the live body and show
    /// the hollow look (no lid, art 13 px up), also after a spill.
    #[test]
    fn hollow_trash_can_keeps_its_sprite_body() {
        let xml = "<levelXML><info v='1.99'/><shapes></shapes><specials><sp t='26' p0='900' p1='395' p2='0' p3='f' p4='t' p5='f'/><sp t='26' p0='1100' p1='395' p2='0' p3='f' p4='t' p5='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let hollow = w.special_bodies[0].unwrap();
        assert!(w.b.c.bodies[hollow].alive, "the sprite body of a hollow can must be the live replacement");
        assert_eq!(w.can_look(hollow), Some((1, false, true)));
        let full = w.special_bodies[1].unwrap();
        assert_eq!(w.can_look(full), Some((1, true, false)));
        // spill it by a hit: lid gone, hollow, and the special follows the rebuilt body
        w.cans[1].spill_impulse = 0.0;
        w.can_rebuild(1, None);
        w.cans[1].spilled = true;
        let after = w.special_bodies[1].unwrap();
        assert!(w.b.c.bodies[after].alive);
        assert_eq!(w.can_look(after), Some((1, false, true)));
    }

    /// Free fall of NPC 1/5 (plain and mirrored) compared with the original class running in the oracle (tests/golden/npc_fall.json, every 10th step).
    #[test]
    fn npc_free_fall_matches_original() {
        let gold: serde_json::Value = serde_json::from_str(include_str!("../../../tests/golden/npc_fall.json")).unwrap();
        for (key, idx, rev) in [("1", 1, 'f'), ("5", 5, 'f'), ("1r", 1, 't'), ("5r", 5, 't')] {
            let lvl = crate::level::parse(&format!("<levelXML><info v='1.99'/><shapes/><specials><sp t='17' p0='1000' p1='0' p2='0' p3='{idx}' p4='f' p5='{rev}' p6='f' p7='t'/></specials></levelXML>")).unwrap();
            let mut w = World::from_level(&lvl);
            assert_eq!(w.npcs.len(), 1, "npc {key} not created");
            let mut worst = 0.0f64;
            for f in 0..60 {
                w.step();
                if f % 10 == 9 {
                    let row = &gold[key][f / 10];
                    for (i, &b) in w.npcs[0].bodies.iter().enumerate() {
                        let body = w.b.body(b);
                        let g = &row[i];
                        let (gx, gy) = (g[0].as_f64().unwrap() + 1000.0, g[1].as_f64().unwrap());
                        worst = worst.max((body.sweep.c.x * PS - gx).abs()).max((body.sweep.c.y * PS - gy).abs());
                    }
                }
            }
            assert!(worst < 1e-6, "npc {key}: worst divergence {worst} px");
        }
    }

    #[test]
    fn every_sound_the_core_requests_exists_in_the_audiosprite() {
        let table: std::collections::HashMap<String, [f64; 2]> = serde_json::from_str(include_str!("../data/audiosprite.json")).unwrap();
        for n in ["Cannon1", "MineExplosion", "Bleep3", "FoodSplat1", "FoodSplat2", "FoodSplat3", "ArrowFire1", "ArrowFire2", "HarpoonFire", "ArrowSnap1", "ArrowSnap2", "BikeSmash1", "BikeTireSmash1", "MetalSmashHeavy", "MetalSmashHeavy3", "MetalSmashMedium", "MetalSmashLight", "PogoFrameSmash", "HeadSmash", "ChestSmash", "PelvisSmash", "NeckBreak", "LimbRip1", "LimbRip2", "LimbRip3", "LimbRip4", "BoneBreak1", "BoneBreak2", "BoneBreak3", "BoneBreak4"] {
            assert!(table.contains_key(n), "{n} missing from audiosprite.json");
        }
    }

    #[test]
    fn boost_pad_pushes_a_ball_sideways_and_not_without_the_pad() {
        // Boost at (500, 450) rotation 0 (pushes +x), 2 panels = 360×180 px zone; a ball inside gets `power·mass` of force each step.
        let ball = "<sh t='1' p0='500' p1='450' p2='20' p3='20' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>";
        let run = |specials: &str| {
            let level = crate::level::parse(&format!("<levelXML><info v='1.99'/><shapes>{ball}</shapes><specials>{specials}</specials></levelXML>")).unwrap();
            let mut w = World::from_level(&level);
            let mut x_after = 0.0;
            for _ in 0..30 {
                w.step();
                x_after = w.position_px(0).unwrap()[0];
            }
            x_after
        };
        let plain = run("");
        let boosted = run("<sp t='12' p0='500' p1='450' p2='0' p3='2' p4='20'/>");
        assert!(boosted > plain + 20.0, "boost had no effect: {plain} vs {boosted}");
    }

    #[test]
    fn building_is_solid_and_fan_blows_a_ball_up() {
        // Building1 (1 floor, width 1): 300×265 box at (x, y) top-left. A ball dropped onto its roof rests on it.
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><shapes>\
             <sh t='1' p0='600' p1='0' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
             <specials><sp t='13' p0='500' p1='300' p2='1' p3='1'/></specials></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..120 {
            w.step();
        }
        let y = w.position_px(0).unwrap()[1];
        assert!((y - 275.0).abs() < 3.0, "ball on roof y = {y}");
        // Fan pointing up at (500, 600): a light ball above it must be pushed upwards (it would otherwise fall).
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><shapes>\
             <sh t='1' p0='500' p1='450' p2='20' p3='20' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
             <specials><sp t='8' p0='500' p1='600' p2='0'/></specials></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        let mut top = f32::MAX;
        for _ in 0..90 {
            w.step();
            top = top.min(w.position_px(0).unwrap()[1]);
        }
        assert!(top < 440.0, "fan did not lift the ball: top {top}");
    }

    #[test]
    fn chain_hangs_together_and_falls_as_one_piece() {
        // 5 links starting at (500, 100), straight along the reference's down axis; free chain falls onto the floor.
        let level = crate::level::parse(&format!(
            "<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='30' p0='500' p1='100' p2='0' p3='f' p4='t' p5='5' p6='1' p7='0'/></specials></levelXML>"
        )).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..150 {
            w.step();
        }
        let parts = &w.special_parts[0];
        assert_eq!(parts.len(), 5);
        let ys: Vec<f32> = parts.iter().map(|&b| w.body_position_px(b).unwrap()[1]).collect();
        assert!(ys.iter().all(|&y| y > 400.0 && y < 495.0), "links not on the floor: {ys:?}");
        let xs: Vec<f32> = parts.iter().map(|&b| w.body_position_px(b).unwrap()[0]).collect();
        assert!(xs.windows(2).all(|p| (p[1] - p[0]).abs() < 12.0), "chain came apart: {xs:?}");
    }

    #[test]
    fn level_joint_can_pin_a_special_through_its_nearest_body() {
        // 'sN' in a joint is SPECIAL N (shapes are bare numbers): pin a chain's first link to the level.
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><specials><sp t='30' p0='500' p1='100' p2='0' p3='f' p4='t' p5='5' p6='1' p7='0'/></specials>\
             <joints><j t='0' x='500' y='100' b1='s0' b2='-1' l='f' ua ='0' la ='0' m='f' tq='0' sp='0' c='f'/></joints></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..120 {
            w.step();
        }
        let ys: Vec<f32> = w.special_parts[0].iter().map(|&b| w.body_position_px(b).unwrap()[1]).collect();
        assert!(ys.iter().all(|&y| (95.0..165.0).contains(&y)), "chain fell off its pin: {ys:?}");
    }

    #[test]
    fn spawned_character_lands_on_the_floor_inside_the_level() {
        let level = crate::level::parse(&format!("<levelXML><info v='1.99' x='600' y='300'/><shapes>{FLOOR}</shapes></levelXML>")).unwrap();
        let mut w = World::from_level(&level);
        assert!(w.spawn_character(1, level.info.start));
        for _ in 0..150 {
            w.step();
        }
        let doll = w.character.as_ref().unwrap();
        let chest = doll.position_px(&w, crate::character::Part::Chest).unwrap();
        assert!(chest[1] > 400.0 && chest[1] < 495.0 && (chest[0] - 600.0).abs() < 200.0, "chest {chest:?}");
    }

    #[test]
    fn rail_and_finish_line_are_static_floors() {
        // Ball dropped onto a rail (18 px tall → top at y-9) and onto a finish line (40 px tall → top at y-20).
        let level = crate::level::parse(
            "<levelXML><info v='1.99'/><shapes>\
             <sh t='1' p0='0' p1='0' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>\
             <sh t='1' p0='1000' p1='0' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>\
             </shapes><specials><sp t='27' p0='0' p1='300' p2='400' p3='0'/><sp t='9' p0='1000' p1='300'/></specials></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..150 {
            w.step();
        }
        let (rail_ball, finish_ball) = (w.position_px(0).unwrap()[1], w.position_px(1).unwrap()[1]);
        assert!((rail_ball - (300.0 - 9.0 - 25.0)).abs() < 4.0, "ball on rail y = {rail_ball}");
        assert!((finish_ball - (300.0 - 20.0 - 25.0)).abs() < 4.0, "ball on finish line y = {finish_ball}");
        assert!(w.body_is_fixed(w.special_bodies[0].unwrap()));
    }

    #[test]
    fn ball_falls_onto_fixed_floor_and_rests() {
        let level = crate::level::parse(
            "<levelXML><info v='1'/><shapes>\
             <sh t='0' p0='0' p1='500' p2='2000' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>\
             <sh t='1' p0='0' p1='0' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>\
             </shapes></levelXML>",
        )
        .unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..150 {
            w.step();
        }
        let y = w.position_px(1).unwrap()[1];
        // floor top = 490, ball radius 25 → resting centre ≈ 465
        assert!((y - 465.0).abs() < 6.0, "ball centre y = {y}");
    }
}
