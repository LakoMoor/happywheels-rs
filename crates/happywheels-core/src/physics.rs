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

#[derive(Debug, Clone, serde::Serialize)]
pub struct TriggerSound {
    pub name: String,
    pub position: [f32; 2],
    pub global: bool,
    pub volume: f32,
    pub pan: f32,
}

/// Runtime state of one trigger, transcribed from the original's trigger class (contact-listener driven):
/// triggeredBy 1 = main character, 2 = any character, 3 = any non-fixed shape, 4 = only this trigger's
/// targets, 5 = other triggers (not modelled), 6 = mouse click (not modelled); repeatType 1 = once,
/// 2 = again after the zone was cleared, 3 = every `interval` while the body stays, 4 = started by
/// another trigger (not modelled).
struct TriggerState {
    sensor: ShapeId,
    kind: i32,
    position: [f32; 2],
    sound: Option<TriggerSound>,
    by: i32,
    repeat: i32,
    repeat_frames: Option<u32>,
    repeat_count: u32,
    delay_frames: u32,
    delay: Vec<u32>,
    targets: Vec<BodyId>,
    target_shapes: Vec<usize>,
    /// Shape targets with their actions `[action index, params…]` (action-trigger type only).
    shape_actions: Vec<(usize, Vec<Vec<String>>)>,
    /// Group actions use a different index ordering from individual shapes.
    group_actions: Vec<(BodyId, Vec<Vec<String>>)>,
    /// Special targets (NPC chest body, actions).
    special_actions: Vec<(BodyId, Vec<Vec<String>>)>,
    /// Triggers activated when this one fires (`TargetKind::Trigger` targets).
    trigger_targets: Vec<(usize, Vec<u8>)>,
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
    reversed: bool,
    /// `destroyJointsUponDeath`: a dead NPC ignores the impulse / hold pose / release pose trigger actions.
    destroy_on_death: bool,
    /// Sound-set prefix (`tag` of the character index), queued vocals by priority, frame the current voice ends, its priority, and `dead` (no more vocals).
    tag: &'static str,
    voices: [Option<&'static str>; 6],
    voice_until: u32,
    voice_prio: i32,
    dead: bool,
    /// Chunk circles / brain / heart born from smashes.
    pieces: Vec<BodyId>,
}

/// `tag` getter of the level character: the prefix of its voice sounds.
fn npc_tag(char_index: i32) -> &'static str {
    match char_index {
        1 => "Char1",
        2 | 9 => "Char2",
        4 | 15 => "Kid1",
        5 | 13 => "Char4",
        6 => "Char8",
        7 => "Char9",
        8 => "Char11",
        10 => "Santa",
        11 => "Elf1",
        12 => "Char12",
        14 => "Kid2",
        16 => "Heli",
        _ => "Char3",
    }
}

/// SpringBox special: the pad rides a prismatic joint locked at 0..0; after a Result on the pad and `delay` frames the motor drives it 32 px out, then back,
/// and the joint is locked again (`actions` of the original).
struct SpringBox {
    body: BodyId,
    pad: ShapeId,
    joint: usize,
    hit: bool,
    /// Motor running (the listener is off until the pad is locked again).
    active: bool,
    delay_total: i32,
    delay_counter: i32,
}

/// Paddle special: a pad pinned to the level by a locked revolute joint; after a hit and `delay` frames the motor swings it to `target` and back, then it locks again.
struct Paddle {
    body: BodyId,
    pad: ShapeId,
    joint: usize,
    dir: f64,
    target: f64,
    speed: f64,
    hit: bool,
    active: bool,
    broken: bool,
    delay_total: i32,
    delay_counter: i32,
}

/// Mine special (`Mine`): a Result on its small trigger box detonates it: the body goes and every non-static body within 2 m gets an impulse `10·(1 − d/2)` away from it.
struct PlainMine {
    body: BodyId,
    trigger: ShapeId,
    done: bool,
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
    /// Per piece (`nd`): (body, shape, shatter impulse 10·mass, particles, sound suffix); a Result above the impulse shatters the piece too.
    shards: Vec<Shard>,
    stabbing: bool,
}

/// One glass triangle (`nd`/`te`): shatters above `shatter` (10·mass) and, when the pane is `stabbing`, pins flesh it hits above `stab_limit` with a sensor +
/// prismatic joints like spikes (motor force 30), until nothing is pinned any more.
/// Which `b2*Contact` class evaluated a contact: (shape1 is a circle, shape2 is a circle).
type ContactClass = (bool, bool);

/// Helicopter blade shard (`MN`): a Result above 3·mass from flesh turns it into a pin (solid shape mask 8 + a sensor with mask 4), flesh entering the sensor is
/// joined with a prismatic joint (`bladeActions`); with no joints left it goes back to colliding normally.
struct BladeShard {
    body: BodyId,
    shape: ShapeId,
    sensor: Option<ShapeId>,
    stab_impulse: f64,
    pinned: Vec<(BodyId, usize, u32)>,
    queue_add: Vec<(BodyId, Vec2, Option<ContactClass>)>,
    queue_remove: Vec<BodyId>,
    previous: Option<BodyId>,
    sounds: u32,
}

struct Shard {
    body: BodyId,
    shape: ShapeId,
    shatter: f64,
    stab_limit: f64,
    particles: u32,
    blood_particles: u32,
    suffix: &'static str,
    fatal: bool,
    /// Sensor shape while in stab mode.
    sensor: Option<ShapeId>,
    /// (flesh body, joint, count)
    pinned: Vec<(BodyId, usize, u32)>,
    /// (flesh body, contact normal, aliased): the original's listener hands the sensor-as-shape1 case the shared `s_evalCP` itself, whose normal keeps changing
    /// until the queue runs, so `aliased` entries take the normal of the last Add/Persist/Remove event instead of their own.
    queue_add: Vec<(BodyId, Vec2, Option<ContactClass>)>,
    queue_remove: Vec<BodyId>,
    sounds: u32,
    /// `GlassBloodMC` stamps in the shard body's local px: (x, y, rotation rad, scaleX ±1), one per impaled body.
    blood: Vec<(f64, f64, f64, f64)>,
}

/// Toilet special (`$e`): a Result above 12 on the back breaks off the back (two pieces), on the base or bowl the base (two pieces).
struct Toilet {
    body: BodyId,
    sign: f64,
    back: Option<ShapeId>,
    base: Option<ShapeId>,
    bowl: Option<ShapeId>,
    done: bool,
    /// Frame until which the `ToiletSmash` handle is still playing.
    sound_until: u32,
    /// (body, 0 base1 / 1 base2 / 2 back1 / 3 back2, mirror sign)
    pieces: Vec<(BodyId, u8, f64)>,
}

/// Log special (`xe`): a Result above 2250 splits a free log across its middle into two half bodies.
struct Log {
    body: BodyId,
    shape: ShapeId,
    w: f64,
    h: f64,
    /// Sprite stretch (shapeWidth / 36, shapeHeight / 400).
    stretch: (f64, f64),
    done: bool,
    halves: Vec<(BodyId, u8)>,
}

/// Width in px of the bounding box of a triangle given in metres (`sprite.width` of the original's shard graphics).
fn tri_width_px(v: &[Vec2; 3]) -> f64 {
    let (lo, hi) = v.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.x), hi.max(p.x)));
    (hi - lo) * PS
}

/// `createSpray` emitter (`Mu`): `per_frame` sprite particles per step from a segment of the body (local m), until `count` passes `total`.
struct Spray {
    body: BodyId,
    start: Vec2,
    range: Vec2,
    min: f64,
    max: f64,
    angle_range: f64,
    rot: f64,
    per_frame: u32,
    total: u32,
    count: u32,
    kind: crate::blood::ShardKind,
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
    hovering: bool,
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
    level_version: f32,
    triggers: Vec<TriggerState>,
    fans: Vec<Fan>,
    boosts: Vec<Boost>,
    jets: Vec<Jet>,
    cannons: Vec<Cannon>,
    spikes: Vec<Spike>,
    /// Token specials: (body, sensor shape); collected ones are removed.
    tokens: Vec<(BodyId, ShapeId)>,
    /// Finish platform shape, centre in pixels, and whether its listener was removed.
    finishes: Vec<(ShapeId, [f64; 2], bool)>,
    pub level_complete: bool,
    /// FoodItem specials: (body, shape, smash impulse).
    /// (body, shape, smash impulse, food type 0..2)
    foods: Vec<(BodyId, ShapeId, f64, u8)>,
    /// Chunk bodies born from smashed food: (body, food type, chunk index).
    food_chunks: Vec<(BodyId, u8, u8)>,
    boomboxes: Vec<Boombox>,
    spring_boxes: Vec<SpringBox>,
    paddles: Vec<Paddle>,
    plain_mines: Vec<PlainMine>,
    chairs: Vec<BreakChair>,
    sprays: Vec<Spray>,
    logs: Vec<Log>,
    toilets: Vec<Toilet>,
    glasses: Vec<Glass>,
    blade_shards: Vec<BladeShard>,
    cans: Vec<TrashCan>,
    hit_sounds: Vec<HitSound>,
    /// Dynamic level shapes with a `ShapeHit1..4` contact sound (`registerShapeSound`).
    shape_hits: Vec<(ShapeId, &'static str)>,
    /// Bodies born from specials (trash, hollow cans…) that only the renderer cares about.
    loose: Vec<BodyId>,
    mines: Vec<Mine>,
    /// Sound effects requested this step (audiosprite name, world position in px); the app drains them with `take_sounds`.
    pub sounds: Vec<(String, [f32; 2])>,
    /// Sounds played with a volume argument (`playAreaSoundInstance(name, body, volume)`): (name, position px, volume 0..1).
    scaled_sounds: Vec<(String, [f32; 2], f32)>,
    pub trigger_sounds: Vec<TriggerSound>,
    /// Blood particles and flows (see `blood.rs`).
    pub blood: crate::blood::Blood,
    /// Looping sounds wanted this frame (`playAreaSoundLoop`): sprite name and position in px; the audio layer fades them in/out.
    pub loops: Vec<(String, [f32; 2], f32, BodyId)>,
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
    /// Individual fixtures for shapes attached to the shared static body.
    static_shape_fixtures: Vec<Option<ShapeId>>,
    shape_initial_angle: Vec<f64>,
    shape_initial_position: Vec<[f64; 2]>,
    /// Sprite-origin offset in the body's local pixel coordinates.
    pub shape_render_offset: Vec<[f32; 2]>,
    /// Visibility and last sprite pose after a trigger removes a shape body.
    pub shape_visible: Vec<bool>,
    pub shape_frozen_pose: Vec<Option<([f32; 2], f32)>>,
    pub group_bodies: Vec<BodyId>,
    pub group_visible: Vec<bool>,
    pub group_frozen_pose: Vec<Option<([f32; 2], f32)>>,
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
            level_version: version,
            triggers: Vec::new(),
            fans: Vec::new(),
            boosts: Vec::new(),
            jets: Vec::new(),
            cannons: Vec::new(),
            spikes: Vec::new(),
            tokens: Vec::new(),
            finishes: Vec::new(),
            level_complete: false,
            foods: Vec::new(),
            food_chunks: Vec::new(),
            boomboxes: Vec::new(),
            spring_boxes: Vec::new(),
            paddles: Vec::new(),
            plain_mines: Vec::new(),
            chairs: Vec::new(),
            sprays: Vec::new(),
            logs: Vec::new(),
            toilets: Vec::new(),
            glasses: Vec::new(),
            blade_shards: Vec::new(),
            cans: Vec::new(),
            hit_sounds: Vec::new(),
            shape_hits: Vec::new(),
            loose: Vec::new(),
            mines: Vec::new(),
            sounds: Vec::new(),
            scaled_sounds: Vec::new(),
            trigger_sounds: Vec::new(),
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
            static_shape_fixtures: vec![None; level.shapes.len()],
            shape_initial_angle: level.shapes.iter().map(|s| num(s, "angle").unwrap_or(0.0).to_radians()).collect(),
            shape_initial_position: level.shapes.iter().map(|s| [num(s, "x").unwrap_or(0.0), num(s, "y").unwrap_or(0.0)]).collect(),
            shape_render_offset: vec![[0.0, 0.0]; level.shapes.len()],
            shape_visible: vec![true; level.shapes.len()],
            shape_frozen_pose: vec![None; level.shapes.len()],
            group_bodies: Vec::new(),
            group_visible: vec![true; level.groups.len()],
            group_frozen_pose: vec![None; level.groups.len()],
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
            w.static_shape_fixtures[i] = Some(w.b.create_shape(level_body, &d));
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
            let shape = w.b.create_shape(body, &d);
            w.b.set_mass_from_shapes(body);
            // `registerShapeSound`: the body's mass picks ShapeHit1..4
            let m = w.b.body(body).mass;
            let name = if m > 15.0 { "ShapeHit4" } else if m > 10.0 { "ShapeHit3" } else if m > 5.0 { "ShapeHit2" } else if m > 1.0 { "ShapeHit1" } else { "" };
            if !name.is_empty() {
                w.shape_hits.push((shape, name));
            }
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
        if sp.kind == 9 {
            self.b.c.report_results = true;
            self.finishes.push((self.b.body(body).shapes[0], [x, y], false));
        }
        let hit = match sp.kind {
            3 => Some(("IBeamHit", 2, 4.0, 0)),
            4 => Some(("LumberHit", 2, 4.0, 0)),
            21 => Some(("TVHit", 1, 4.0, 0)),
            22 => Some(("BoomboxHit", 1, 4.0, 0)),
            24 => Some(("ToiletHit", 2, 4.0, 11)),
            26 => Some(("TrashCanHit", 2, 4.0, 20)),
            11 => Some(("MeteorImpact", 1, 2.0, 0)),
            18 => Some(("GlassImpact", 2, 4.0, 0)),
            _ => None,
        };
        if let Some((name, variants, threshold, lock_frames)) = hit {
            let first = self.b.body(body).shapes[0];
            self.hit_sounds.push(HitSound { body, shapes: vec![first], name, variants, threshold, lock_frames, free_at: 0 });
        }
        if sp.kind == 4 {
            self.b.c.report_results = true;
            let (w, h) = (f64::from(sp.num("shapeWidth").unwrap_or(36.0)), f64::from(sp.num("shapeHeight").unwrap_or(400.0)));
            let shape = self.b.body(body).shapes[0];
            self.logs.push(Log { body, shape, w, h, stretch: (w / 36.0, h / 400.0), done: false, halves: Vec::new() });
        }
        if sp.kind == 18 {
            let first = self.b.body(body).shapes[0];
            self.b.c.report_results = true;
            self.b.set_mass_from_shapes(body);
            let mass = self.b.body(body).mass;
            let (suffix, particles) = if mass < 0.75 { ("Light", 100) } else if mass < 4.0 { ("Mid", 150) } else { ("Heavy", 200) };
            let strength = f64::from(sp.num("shatterStrength").unwrap_or(10.0));
            let half = (f64::from(sp.num("shapeWidth").unwrap_or(10.0)) / 2.0 / PS, f64::from(sp.num("shapeHeight").unwrap_or(100.0)) / 2.0 / PS);
            self.glasses.push(Glass { body, shape: first, half, smash_impulse: (mass * strength).round(), particles, suffix, done: false, pieces: Vec::new(), shards: Vec::new(), stabbing: sp.flag("stabbing").unwrap_or(true) });
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
        if sp.kind == 24 && sp.flag("interactive") != Some(false) {
            self.b.c.report_results = true;
            let s = self.b.body(body).shapes.clone();
            let sign = if sp.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
            self.toilets.push(Toilet { body, sign, back: s.first().copied(), base: s.get(1).copied(), bowl: s.get(2).copied(), done: false, sound_until: 0, pieces: Vec::new() });
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
        if sp.kind == 2 && self.b.body(body).shapes.len() == 2 {
            self.b.c.report_results = true;
            let trigger = self.b.body(body).shapes[1];
            self.plain_mines.push(PlainMine { body, trigger, done: false });
        }
        if sp.kind == 35 {
            if let Some(joint) = (0..self.b.c.joints.len()).find(|&j| self.b.c.joints[j].body2 == body && matches!(self.b.c.joints[j].kind, box2d::joints::JointKind::Revolute(_))) {
                self.b.c.report_results = true;
                if let box2d::joints::JointKind::Revolute(r) = &mut self.b.joint_mut(joint).kind {
                    r.max_motor_torque = 1e6;
                }
                let dir = if sp.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
                let delay = (f64::from(sp.num("springDelay").unwrap_or(1.0)) * 30.0).round() as i32;
                let target = f64::from(sp.num("paddleAngle").unwrap_or(90.0)).to_radians() * dir;
                let speed = 0.5 + 5.5 * f64::from(sp.num("paddleSpeed").unwrap_or(5.0)) / 10.0;
                let pad = self.b.body(body).shapes[0];
                self.paddles.push(Paddle { body, pad, joint, dir, target, speed, hit: false, active: false, broken: false, delay_total: delay, delay_counter: delay });
            }
        }
        if sp.kind == 5 {
            if let Some(joint) = (0..self.b.c.joints.len()).find(|&j| self.b.c.joints[j].body2 == body && matches!(self.b.c.joints[j].kind, box2d::joints::JointKind::Prismatic(_))) {
                self.b.c.report_results = true;
                let delay = (f64::from(sp.num("springDelay").unwrap_or(1.0)) * 30.0).round() as i32;
                let pad = self.b.body(body).shapes[0];
                if let box2d::joints::JointKind::Prismatic(p) = &mut self.b.joint_mut(joint).kind {
                    p.max_motor_force = 1e6;
                }
                self.spring_boxes.push(SpringBox { body, pad, joint, hit: false, active: false, delay_total: delay, delay_counter: delay });
            }
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
                kind: t.type_index,
                position: t.pos,
                sound: t.sound_index.and_then(|index| {
                    static NAMES: std::sync::OnceLock<Vec<String>> = std::sync::OnceLock::new();
                    let names = NAMES.get_or_init(|| serde_json::from_str(include_str!("../data/trigger-sounds.json")).expect("valid trigger sound table"));
                    Some(TriggerSound { name: names.get(index)?.clone(), position: t.pos, global: t.sound_location == 1, volume: t.sound_volume, pan: t.sound_pan })
                }),
                by: t.triggered_by,
                repeat: t.repeat_type,
                repeat_frames: t.repeat_interval.map(|v| (v * 30.0).floor() as u32),
                repeat_count: t.repeat_interval.map_or(0, |v| (v * 30.0).floor() as u32),
                delay_frames: (t.delay.unwrap_or(0.0) * 30.0).floor() as u32,
                delay: Vec::new(),
                targets,
                target_shapes: t.targets.iter().filter(|tg| tg.kind == crate::level::TargetKind::Shape).map(|tg| tg.index).collect(),
                shape_actions: if t.type_index == 1 {
                    t.targets
                        .iter()
                        .filter(|tg| tg.kind == crate::level::TargetKind::Shape)
                        .filter(|tg| self.shape_bodies.get(tg.index).copied().flatten().is_some())
                        .map(|tg| (tg.index, tg.actions.clone()))
                        .collect()
                } else {
                    Vec::new()
                },
                special_actions: if t.type_index == 1 {
                    t.targets.iter().filter(|tg| tg.kind == crate::level::TargetKind::Special)
                        .filter_map(|tg| Some((self.special_bodies.get(tg.index).copied().flatten()?, tg.actions.clone())))
                        .collect()
                } else { Vec::new() },
                group_actions: if t.type_index == 1 {
                    t.targets.iter().filter(|tg| tg.kind == crate::level::TargetKind::Group)
                        .filter_map(|tg| Some((*self.group_bodies.get(tg.index)?, tg.actions.clone())))
                        .collect()
                } else { Vec::new() },
                trigger_targets: t.targets.iter().filter(|tg| tg.kind == crate::level::TargetKind::Trigger).map(|tg| {
                    let actions: Vec<u8> = tg.actions.iter().filter_map(|a| a.first()?.parse().ok()).collect();
                    (tg.index, if actions.is_empty() { vec![0] } else { actions })
                }).collect(),
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
        let kind = self.triggers[trigger].kind;
        let position = self.triggers[trigger].position;
        let t = &mut self.triggers[trigger];
        if t.repeat == 1 {
            t.listening = false;
            t.done = true;
            let sensor = t.sensor;
            self.b.destroy_shape(sensor); // singleAction: levelBody.DestroyShape(sensor)
        }
        if kind == 3 {
            self.level_complete = true;
            self.play("Victory", 1, Vec2::new(f64::from(position[0]) / PS, f64::from(position[1]) / PS));
        }
        if kind == 2 {
            if let Some(sound) = self.triggers[trigger].sound.clone() { self.trigger_sounds.push(sound); }
        }
        for (index, actions) in self.triggers[trigger].shape_actions.clone() {
            for a in &actions {
                let Some(body) = self.shape_bodies[index] else { continue };
                if body == self.level_body {
                    match a.first().and_then(|s| s.parse::<usize>().ok()) {
                        Some(2) => self.unfix_static_shape(index),
                        Some(5 | 6) => {
                            if let Some(shape) = self.static_shape_fixtures[index].take() { self.b.destroy_shape(shape); }
                            if a[0] == "6" { self.shape_visible[index] = false; }
                        }
                        Some(7) => {
                            if let Some(shape) = self.static_shape_fixtures[index] {
                                let mode = a.get(1).and_then(|s| s.parse::<i32>().ok()).unwrap_or(0);
                                self.change_shape_collision(shape, mode, true);
                            }
                        }
                        _ => {}
                    }
                } else {
                    if a.first().is_some_and(|s| s == "1") && self.b.c.bodies[body].alive && self.b.body(body).mass > 0.0 {
                        self.fix_dynamic_shape(index, body);
                    } else {
                        self.shape_action(body, a);
                    }
                }
            }
        }
        for (chest, actions) in self.triggers[trigger].special_actions.clone() {
            for a in &actions {
                self.npc_trigger_action(chest, a);
            }
        }
        for (body, actions) in self.triggers[trigger].group_actions.clone() {
            for mut action in actions {
                let mapped = action.first().and_then(|s| s.parse::<usize>().ok()).and_then(|index| match index {
                    0 => Some(0), 2 => Some(4), 3 => Some(1), 4 => Some(2), 5 => Some(5), 6 => Some(6), 7 => Some(7), _ => None,
                });
                if let Some(mapped) = mapped {
                    action[0] = mapped.to_string();
                    self.shape_action(body, &action);
                }
            }
        }
        if kind == 1 {
            for (target, actions) in self.triggers[trigger].trigger_targets.clone() {
                if target != trigger {
                    for action in actions {
                        match action {
                            0 => self.activate_by_trigger(target),
                            1 => self.set_trigger_disabled(target, true),
                            2 => self.set_trigger_disabled(target, false),
                            _ => {}
                        }
                    }
                }
            }
        }
    }

    /// Trigger.disabled setter: disable clears pending actions; enabling refilters its live sensor.
    fn set_trigger_disabled(&mut self, index: usize, disabled: bool) {
        let Some(trigger) = self.triggers.get_mut(index) else { return };
        if trigger.disabled == disabled { return; }
        trigger.disabled = disabled;
        if disabled {
            trigger.triggering_body = None;
            trigger.triggering_count = 0;
            trigger.repeat_count = trigger.repeat_frames.unwrap_or(0);
            trigger.delay.clear();
            trigger.active = false;
            trigger.watching_remove = false;
            trigger.listening = false;
        } else if !trigger.done {
            trigger.listening = (1..=4).contains(&trigger.by);
            let sensor = trigger.sensor;
            self.b.refilter(sensor);
        }
    }

    fn unfix_static_shape(&mut self, index: usize) {
        let Some(old) = self.static_shape_fixtures[index] else { return };
        let shape = self.b.c.shapes[old].shape.clone();
        let angle = self.shape_initial_angle[index];
        let (origin, geometry) = match &shape.kind {
            B2Kind::Circle(circle) => (circle.local_position, Geometry::Circle { local_position: Vec2::default(), radius: circle.radius }),
            B2Kind::Polygon { poly, .. } => {
                let origin = poly.centroid;
                let (sin, cos) = angle.sin_cos();
                let verts = poly.vertices.iter().map(|v| {
                    let x = v.x - origin.x;
                    let y = v.y - origin.y;
                    Vec2::new(cos * x + sin * y, -sin * x + cos * y)
                }).collect();
                (origin, Geometry::Polygon(verts))
            }
        };
        let mut def = ShapeDef::new(geometry);
        let original = self.shape_initial_position[index];
        let dx = original[0] - origin.x * PS;
        let dy = original[1] - origin.y * PS;
        let (sin, cos) = angle.sin_cos();
        self.shape_render_offset[index] = [(cos * dx + sin * dy) as f32, (-sin * dx + cos * dy) as f32];
        def.density = if shape.density > 0.0 { shape.density } else { 1.0 };
        def.filter = shape.filter;
        let body = self.b.create_body(&BodyDef { position: origin, angle, ..Default::default() });
        let fixture = self.b.create_shape(body, &def);
        self.b.destroy_shape(old);
        self.static_shape_fixtures[index] = None;
        self.shape_bodies[index] = Some(body);
        self.shape_frozen_pose[index] = None;
        self.b.set_mass_from_shapes(body);
        {
            let mut filter = self.b.c.shapes[fixture].shape.filter;
            if self.level_version > 1.84 {
            match filter.category_bits {
                24 => { filter.category_bits = 8; if filter.mask_bits == 56 { filter.mask_bits = 8; } else if filter.group_index == -10 { filter.group_index = 0; } }
                16 => { filter.mask_bits = 16; filter.group_index = -322; }
                48 => filter.category_bits = 32,
                _ => {}
            }
            } else if filter.group_index == -10 && filter.mask_bits == u16::MAX {
                filter.group_index = 0;
                filter.category_bits = 8;
            }
            self.b.set_shape_filter(fixture, filter);
        }
        for trigger in &mut self.triggers {
            if trigger.target_shapes.contains(&index) {
                trigger.targets.push(body);
            }
        }
    }

    /// Individual shape freeze replaces its body with a fixture on the level body in the original.
    fn fix_dynamic_shape(&mut self, index: usize, body: BodyId) {
        let Some(&old) = self.b.body(body).shapes.first() else { return };
        let src = self.b.c.shapes[old].shape.clone();
        let bd = self.b.body(body);
        let position = self.body_position_px(body).unwrap_or([0.0, 0.0]);
        let angle = bd.sweep.a;
        let geometry = match &src.kind {
            B2Kind::Circle(c) => Geometry::Circle { local_position: bd.world_point(c.local_position), radius: c.radius },
            B2Kind::Polygon { poly, .. } => Geometry::Polygon(poly.vertices.iter().map(|&v| bd.world_point(v)).collect()),
        };
        let [ox, oy] = self.shape_render_offset[index];
        let (sin, cos) = angle.sin_cos();
        self.shape_initial_position[index] = [f64::from(position[0]) + f64::from(ox) * cos - f64::from(oy) * sin, f64::from(position[1]) + f64::from(ox) * sin + f64::from(oy) * cos];
        self.shape_initial_angle[index] = angle;
        self.shape_frozen_pose[index] = Some((position, angle as f32));
        let mut def = ShapeDef::new(geometry);
        def.density = src.density;
        def.filter = src.filter;
        if self.level_version > 1.84 {
            match def.filter.category_bits {
                8 => { def.filter.category_bits = 24; if def.filter.mask_bits == 8 { def.filter.mask_bits = 56; } else if def.filter.group_index == 0 { def.filter.group_index = -10; } }
                16 => { def.filter.mask_bits = 48; def.filter.group_index = 0; }
                32 => def.filter.category_bits = 48,
                _ => {}
            }
        } else if def.filter.mask_bits == u16::MAX && def.filter.group_index == 0 {
            def.filter.category_bits = 24;
            def.filter.group_index = -10;
        }
        let fixture = self.b.create_shape(self.level_body, &def);
        self.b.destroy_body(body);
        self.static_shape_fixtures[index] = Some(fixture);
        self.shape_bodies[index] = Some(self.level_body);
        for trigger in &mut self.triggers { trigger.targets.retain(|&target| target != body); }
    }

    /// Instant shape actions of the original (list: wake from sleep, set to fixed, set to non fixed, change
    /// opacity, apply impulse, delete shape, delete self, change collision). Supports 0, 1, 2, 4, 5, 6, 7;
    /// shapes born on the shared level body cannot yet be made dynamic individually.
    fn shape_action(&mut self, body: BodyId, a: &[String]) {
        if body == self.level_body {
            return;
        }
        if !self.b.c.bodies[body].alive {
            if a.first().is_some_and(|action| action == "6") {
                for (i, handle) in self.shape_bodies.iter().enumerate() {
                    if *handle == Some(body) { self.shape_visible[i] = false; }
                }
                for (i, &handle) in self.group_bodies.iter().enumerate() {
                    if handle == body { self.group_visible[i] = false; }
                }
            }
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
                    if self.level_version > 1.84 {
                        for shape in self.b.body(body).shapes.clone() {
                            let mut filter = self.b.c.shapes[shape].shape.filter;
                            match filter.category_bits {
                                8 => { filter.category_bits = 24; if filter.mask_bits == 8 { filter.mask_bits = 56; } else if filter.group_index == 0 { filter.group_index = -10; } }
                                16 => { filter.mask_bits = 48; filter.group_index = 0; }
                                32 => filter.category_bits = 48,
                                _ => {}
                            }
                            self.b.set_shape_filter(shape, filter);
                        }
                    }
                }
            }
            Some(2) => {
                if self.b.body(body).mass == 0.0 {
                    self.b.set_mass_from_shapes(body);
                    self.b.body_mut(body).wake_up();
                    if self.level_version > 1.84 {
                        for shape in self.b.body(body).shapes.clone() {
                            let mut filter = self.b.c.shapes[shape].shape.filter;
                            match filter.category_bits {
                                24 => { filter.category_bits = 8; if filter.mask_bits == 56 { filter.mask_bits = 8; } else if filter.group_index == -10 { filter.group_index = 0; } }
                                16 => { filter.mask_bits = 16; filter.group_index = -322; }
                                48 => filter.category_bits = 32,
                                _ => {}
                            }
                            self.b.set_shape_filter(shape, filter);
                        }
                    }
                }
            }
            Some(5 | 6) => {
                let remove_sprite = a.first().is_some_and(|s| s == "6");
                let position = self.body_position_px(body);
                let angle = self.body_angle(body);
                for (i, handle) in self.shape_bodies.iter().enumerate() {
                    if *handle == Some(body) {
                        self.shape_visible[i] = !remove_sprite;
                        self.shape_frozen_pose[i] = position.map(|p| (p, angle));
                    }
                }
                for (i, &handle) in self.group_bodies.iter().enumerate() {
                    if handle == body {
                        self.group_visible[i] = !remove_sprite;
                        self.group_frozen_pose[i] = position.map(|p| (p, angle));
                    }
                }
                self.b.destroy_body(body);
            }
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
            Some(7) => {
                let mode = param(0) as i32;
                let fixed = self.b.body(body).mass == 0.0;
                for shape in self.b.body(body).shapes.clone() {
                    self.change_shape_collision(shape, mode, fixed);
                }
            }
            _ => {}
        }
    }

    fn change_shape_collision(&mut self, shape: ShapeId, mode: i32, fixed: bool) {
        let modern = self.level_version > 1.84;
        let mut filter = self.b.c.shapes[shape].shape.filter;
        match mode {
            2 => { filter.category_bits = if modern && fixed { 24 } else { 8 }; filter.mask_bits = if modern && fixed { 56 } else { 8 }; filter.group_index = 0; }
            3 => { filter.category_bits = u16::from(self.level_version < 1.82); filter.mask_bits = filter.category_bits; filter.group_index = if self.level_version < 1.82 { -10 } else { 0 }; }
            4 => { filter.category_bits = if modern && fixed { 24 } else { 8 }; filter.mask_bits = u16::MAX; filter.group_index = -321; }
            5 => { filter.category_bits = 16; filter.mask_bits = if modern && fixed { 48 } else { 16 }; filter.group_index = if modern && fixed { 0 } else { -322 }; }
            6 => { filter.category_bits = if modern { if fixed { 48 } else { 32 } } else { 16 }; filter.mask_bits = if modern { 48 } else { 16 }; filter.group_index = 0; }
            7 => { filter.category_bits = 15; filter.mask_bits = 3840; filter.group_index = 0; }
            _ => { filter.category_bits = if fixed { 24 } else { 8 }; filter.mask_bits = u16::MAX; if fixed { filter.group_index = -10; } }
        }
        self.b.shape_mut(shape).is_sensor = mode == 3 && self.level_version < 1.82;
        self.b.set_shape_filter(shape, filter);
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
    pub fn mouse_trigger_enabled(&self, i: usize) -> bool {
        self.triggers.get(i).is_some_and(|t| t.by == 6 && !t.disabled && !t.done)
    }

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
            if t.by == 6 && t.repeat == 3 {
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
        let central = |d: &crate::character::Ragdoll| d.heart.or_else(|| d.bodies.get(&crate::character::Part::Chest).copied());
        let main_body = self.character.as_ref().and_then(central);
        let mut character_bodies = Vec::new();
        if let Some(body) = main_body { character_bodies.push(body); }
        match self.bike.as_ref() {
            Some(crate::vehicle::Vehicle::Bike(b)) => { if let Some(k) = &b.kid { character_bodies.extend(central(&k.doll)); } }
            Some(crate::vehicle::Vehicle::Dirt(d)) => { if let Some(g) = &d.girl { character_bodies.extend(central(&g.doll)); } }
            Some(crate::vehicle::Vehicle::Family(f)) => { character_bodies.extend(central(&f.daughter.doll)); character_bodies.extend(central(&f.son.doll)); }
            Some(crate::vehicle::Vehicle::Santa(s)) => { character_bodies.extend(central(&s.elf1.doll)); character_bodies.extend(central(&s.elf2.doll)); }
            _ => {}
        }
        character_bodies.extend(self.npcs.iter().filter(|npc| !npc.smashed[1]).map(|npc| npc.bodies[0]));
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
                        1 => Some(body) == main_body,
                        // The original checks the contacting shape's character userData. Heart debris has no such owner.
                        2 => character_bodies.contains(&body) && self.b.shape_material(other) & 2 != 0,
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
        let mut dolls: Vec<(&crate::character::Ragdoll, bool)> = Vec::new();
        if let Some(d) = self.character.as_ref() {
            dolls.push((d, true));
        }
        if let Some(crate::vehicle::Vehicle::Bike(b)) = self.bike.as_ref() {
            if let Some(k) = b.kid.as_ref() {
                dolls.push((&k.doll, false));
            }
        }
        if let Some(crate::vehicle::Vehicle::Dirt(d)) = self.bike.as_ref() {
            if let Some(g) = d.girl.as_ref() {
                dolls.push((&g.doll, false));
            }
        }
        if let Some(crate::vehicle::Vehicle::Family(f)) = self.bike.as_ref() {
            dolls.push((&f.daughter.doll, false));
            dolls.push((&f.son.doll, false));
        }
        let mut thuds: Vec<(ShapeId, &'static str, Vec2)> = Vec::new();
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Add) {
            for (mine, other) in [(ev.shape1, ev.shape2), (ev.shape2, ev.shape1)] {
                let body = self.b.c.shapes[mine].body;
                for &(doll, is_rider) in &dolls {
                    let Some((&part, _)) = doll.bodies.iter().find(|(_, &b)| b == body) else { continue };
                    if !matches!(part, Part::Head | Part::Chest | Part::Pelvis) || (is_rider && self.smashes.iter().any(|(p, _)| *p == part)) {
                        continue;
                    }
                    // `contactAddBuffer.get(shape)`: only the first qualifying event of a step is kept per shape
                    if thuds.iter().any(|t| t.0 == mine) {
                        continue;
                    }
                    let o = &self.b.c.shapes[other];
                    // (the group test of `contactAddHandler` only gates `bodyContact`, not the sound buffer: VERIFIED against the carts' real sessions)
                    let (om, mm) = (self.b.body(o.body).mass, self.b.body(body).mass);
                    if o.shape.is_sensor || (om != 0.0 && om < mm) {
                        continue;
                    }
                    if (ev.velocity.x * ev.normal.x + ev.velocity.y * ev.normal.y).abs() > 4.0 {
                        thuds.push((mine, if part == Part::Head { "Thud1" } else { "Thud2" }, self.b.body(body).sweep.c));
                    }
                }
            }
        }
        for (_, n, at) in thuds {
            self.play(n, 1, at);
        }
    }

    /// Vehicle `contactAddSounds` (`BikeHit*`, `TireHit*`, ...): same buffering as the rider's thuds (closing speed above 4 against a non-sensor shape that is
    /// massless or at least as heavy), one sound per shape per step. Only the vehicles listed in `vehicle_contact_sounds` are covered so far; a vehicle smash
    /// does not clear the buffered sound here (the original does).
    fn collect_vehicle_sounds(&mut self, events: &[box2d::world::ContactEvent]) {
        let table = self.vehicle_contact_sounds();
        if table.is_empty() {
            return;
        }
        let mut heard: Vec<(ShapeId, &'static str, Vec2)> = Vec::new();
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Add) {
            for (mine, other) in [(ev.shape1, ev.shape2), (ev.shape2, ev.shape1)] {
                let Some(&(_, name)) = table.iter().find(|(s, _)| *s == mine) else { continue };
                if heard.iter().any(|h| h.0 == mine) {
                    continue;
                }
                let o = &self.b.c.shapes[other];
                let body = self.b.c.shapes[mine].body;
                let (om, mm) = (self.b.body(o.body).mass, self.b.body(body).mass);
                if o.shape.is_sensor || (om != 0.0 && om < mm) {
                    continue;
                }
                if (ev.velocity.x * ev.normal.x + ev.velocity.y * ev.normal.y).abs() > 4.0 {
                    heard.push((mine, name, self.b.body(body).sweep.c));
                }
            }
        }
        for (_, n, at) in heard {
            self.play(n, 1, at);
        }
    }

    /// `shapeContactAdd` + `handleContactAdds` of dynamic level shapes: `ShapeHit*` at volume `min(closing speed / 12, 1)`.
    fn collect_shape_hits(&mut self, events: &[box2d::world::ContactEvent]) {
        if self.shape_hits.is_empty() {
            return;
        }
        let mut heard: Vec<(ShapeId, &'static str, Vec2, f64)> = Vec::new();
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Add) {
            for (mine, other) in [(ev.shape1, ev.shape2), (ev.shape2, ev.shape1)] {
                let Some(&(_, name)) = self.shape_hits.iter().find(|(s, _)| *s == mine) else { continue };
                if heard.iter().any(|h| h.0 == mine) {
                    continue;
                }
                let o = &self.b.c.shapes[other];
                let body = self.b.c.shapes[mine].body;
                let (om, mm) = (self.b.body(o.body).mass, self.b.body(body).mass);
                if o.shape.is_sensor || (om != 0.0 && om < mm) {
                    continue;
                }
                let closing = (ev.velocity.x * ev.normal.x + ev.velocity.y * ev.normal.y).abs();
                if closing > 4.0 {
                    heard.push((mine, name, self.b.body(body).sweep.c, (closing / 12.0).min(1.0)));
                }
            }
        }
        for (_, n, at, v) in heard {
            self.play_scaled(n, at, v);
        }
    }

    fn vehicle_contact_sounds(&self) -> Vec<(ShapeId, &'static str)> {
        let first = |b: BodyId, k: usize| self.b.body(b).shapes.get(k).copied();
        let mut out = Vec::new();
        let bmx = match self.bike.as_ref() {
            Some(crate::vehicle::Vehicle::Bike(v)) => Some(v),
            Some(crate::vehicle::Vehicle::Family(f)) => Some(&f.dad),
            _ => None,
        };
        if let Some(v) = bmx {
            for (k, n) in ["BikeHit3", "BikeHit2", "BikeHit1"].into_iter().enumerate() {
                out.extend(first(v.frame, k).map(|s| (s, n)));
            }
            out.extend(first(v.back_wheel, 0).map(|s| (s, "TireHit1")));
            out.extend(first(v.front_wheel, 0).map(|s| (s, "TireHit2")));
            if let Some(k) = v.kid.as_ref() {
                out.extend(first(k.seat, 0).map(|s| (s, "BikeHit3")));
            }
        }
        if let Some(crate::vehicle::Vehicle::Dirt(v)) = self.bike.as_ref() {
            if !v.frame_smashed {
                for (k, n) in [(0, "ChairHit3"), (2, "ChairHit2"), (4, "ChairHit1"), (5, "ChairHit1")] {
                    out.extend(first(v.frame, k).map(|s| (s, n)));
                }
            }
            out.extend(first(v.back_wheel, 0).map(|s| (s, "TireHit1")));
            out.extend(first(v.front_wheel, 0).map(|s| (s, "TireHit2")));
        }
        if let Some(crate::vehicle::Vehicle::Pogo(v)) = self.bike.as_ref() {
            if !v.frame_smashed {
                out.extend(first(v.frame, 0).map(|s| (s, "Thud1")));
            }
        }
        if let Some(crate::vehicle::Vehicle::Heli(v)) = self.bike.as_ref() {
            if !v.copter_smashed {
                for (k, n) in [(2, "BikeHit1"), (3, "BikeHit3"), (6, "CarTire1"), (7, "CarTire1")] {
                    out.extend(first(v.copter, k).map(|s| (s, n)));
                }
            }
        }
        if let Some(crate::vehicle::Vehicle::Cart(v)) = self.bike.as_ref() {
            if !v.main_smashed {
                for (k, n) in [(2, "BikeHit1"), (3, "BikeHit3")] {
                    out.extend(first(v.body, k).map(|s| (s, n)));
                }
            }
            out.extend(first(v.back_wheel, 0).map(|s| (s, "CarTire1")));
            out.extend(first(v.front_wheel, 0).map(|s| (s, "CarTire1")));
        }
        if let Some(crate::vehicle::Vehicle::Mower(v)) = self.bike.as_ref() {
            for (k, n) in [(2, "ChairHit3"), (6, "ChairHit2"), (7, "BikeHit3")] {
                out.extend(first(v.body, k).map(|s| (s, n)));
            }
            out.extend(first(v.back_wheel, 0).map(|s| (s, "CarTire1")));
            out.extend(first(v.front_wheel, 0).map(|s| (s, "CarTire1")));
        }
        if let Some(crate::vehicle::Vehicle::Santa(v)) = self.bike.as_ref() {
            // the sleigh's shape list only keeps its creation order while nothing has smashed
            if self.b.body(v.sleigh).shapes.len() == 9 {
                for (k, n) in [(0, "SleighImpact3"), (4, "SleighImpact1"), (5, "SleighImpact2"), (6, "SleighImpact1"), (7, "SleighImpact1"), (8, "SleighImpact2")] {
                    out.extend(first(v.sleigh, k).map(|s| (s, n)));
                }
            }
            out.extend(first(v.wheels[0], 0).map(|s| (s, "SkiImpact1")));
            out.extend(first(v.wheels[3], 0).map(|s| (s, "SkiImpact2")));
        }
        if let Some(crate::vehicle::Vehicle::Segway(v)) = self.bike.as_ref() {
            if !v.frame_smashed {
                out.extend(first(v.stand, 0).map(|s| (s, "Thud2")));
            }
            out.extend(first(v.wheel, 0).map(|s| (s, "TireHit1")));
        }
        if let Some(crate::vehicle::Vehicle::Grinder(v)) = self.bike.as_ref() {
            out.extend(first(v.back_wheel, 0).map(|s| (s, "CarTire1")));
            out.extend(first(v.front_wheel, 0).map(|s| (s, "CarTire1")));
        }
        if let Some(crate::vehicle::Vehicle::Chair(v)) = self.bike.as_ref() {
            if !v.chair_smashed {
                for (k, n) in ["ChairHit1", "ChairHit2", "ChairHit3"].into_iter().enumerate() {
                    out.extend(first(v.chair, k).map(|s| (s, n)));
                }
            }
            out.extend(first(v.big_wheel, 0).map(|s| (s, "TireHit1")));
        }
        out
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
        self.frame += 1;
        self.thrust_jets();
        self.run_cannons();
        self.run_mines();
        if matches!(self.bike, Some(crate::vehicle::Vehicle::Santa(_))) {
            if let Some(crate::vehicle::Vehicle::Santa(mut s)) = self.bike.take() {
                s.elf_steps(self);
                self.bike = Some(crate::vehicle::Vehicle::Santa(s));
            }
        }
        self.update_loops();
        self.run_arrow_guns();
        self.run_spring_boxes();
        self.run_paddles();
        self.check_npc_joints();
        self.check_npc_vocals();
        self.b.step(TIME_STEP, ITERATIONS);
        let events: Vec<_> = self.b.c.events.drain(..).collect();
        self.collect_smashes(&events);
        self.collect_thuds(&events);
        self.collect_vehicle_sounds(&events);
        self.collect_shape_hits(&events);
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
        self.finish_events(&events);
        self.handle_events(events.clone());
        self.run_actions();
        self.run_fans();
        self.run_boosts();
        self.run_jets(&events);
        self.cannon_events(&events);
        self.token_events(&events);
        self.food_events(&events);
        self.boombox_events(&events);
        self.spring_box_events(&events);
        self.paddle_events(&events);
        self.blade_shard_events(&events);
        self.plain_mine_events(&events);
        self.chair_events(&events);
        self.glass_events(&events);
        self.toilet_events(&events);
        self.log_events(&events);
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
        if let Some(crate::vehicle::Vehicle::Santa(mut s)) = self.bike.take_if(|v| matches!(v, crate::vehicle::Vehicle::Santa(_))) {
            s.elf_contact_events(self, &events);
            self.bike = Some(crate::vehicle::Vehicle::Santa(s));
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
    /// destroys the joint and frees the limb's filter). Pose angles and the old-level `holdPose` motors are ported (`analysis/npc-pose.js`). NOT ported here: impulse/wake/hold-pose triggers, sounds, sprites.
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
        // Pose angles (`neckAngle` … `knee2Angle`, degrees): the level's NPC sprite parts are rotated about their joint anchors before the bodies are made
        // (checked against the real parser: values are clamped to the joint's own range, `reverse` negates the clamped value, a shoulder/hip turns the
        // whole limb, an elbow/knee only the lower part, and each joint's limits are shifted by the change of its relative angle).
        let reverse = sp.flag("reverse").unwrap_or(false);
        let pose = |name: &str, lo: f64, hi: f64| -> f64 {
            let v = f64::from(sp.num(name).unwrap_or(0.0)).clamp(lo, hi);
            (if reverse { -v } else { v }).to_radians()
        };
        let (d_neck, d_sh, d_el, d_hip, d_knee) = (
            pose("neckAngle", -20.0, 20.0),
            [pose("shoulder1Angle", -180.0, 60.0), pose("shoulder2Angle", -180.0, 60.0)],
            [pose("elbow1Angle", -160.0, 0.0), pose("elbow2Angle", -160.0, 0.0)],
            [pose("hip1Angle", -150.0, 10.0), pose("hip2Angle", -150.0, 10.0)],
            [pose("knee1Angle", 0.0, 150.0), pose("knee2Angle", 0.0, 150.0)],
        );
        let mut lpos = [Vec2::default(); 11];
        let mut lang = [0.0f64; 11];
        for (i, n) in NAMES.iter().enumerate() {
            lpos[i] = Vec2::new(f(&g["bodies"][n]["p"][0]) * 0.5, f(&g["bodies"][n]["p"][1]) * 0.5);
            lang[i] = f(&g["bodies"][n]["a"]);
        }
        let mut turn = |joint: &str, parent: usize, group: &[usize], d: f64| {
            if d == 0.0 {
                return;
            }
            let a1 = &g["joints"][joint]["a1"];
            let (c, s) = (lang[parent].cos(), lang[parent].sin());
            // the sprite part turns about the joint anchor (guide anchors/offsets are the stand-in session's, twice the size of a level NPC's: measured with `analysis/npc-pivots.js`)
            let (ax, ay) = (f(&a1[0]) * 0.5, f(&a1[1]) * 0.5);
            let anchor = Vec2::new(lpos[parent].x + c * ax - s * ay, lpos[parent].y + s * ax + c * ay);
            let (dc, ds) = (d.cos(), d.sin());
            for &i in group {
                let (rx, ry) = (lpos[i].x - anchor.x, lpos[i].y - anchor.y);
                lpos[i] = Vec2::new(anchor.x + dc * rx - ds * ry, anchor.y + ds * rx + dc * ry);
                lang[i] += d;
            }
        };
        turn("neck", 0, &[1], d_neck);
        turn("shoulder1", 0, &[3, 5], d_sh[0]);
        turn("shoulder2", 0, &[4, 6], d_sh[1]);
        turn("elbow1", 3, &[5], d_el[0]);
        turn("elbow2", 4, &[6], d_el[1]);
        turn("hip1", 2, &[7, 9], d_hip[0]);
        turn("hip2", 2, &[8, 10], d_hip[1]);
        turn("knee1", 7, &[9], d_knee[0]);
        turn("knee2", 8, &[10], d_knee[1]);
        let joint_shift = |name: &str| -> f64 {
            match name {
                "neck" => d_neck,
                "shoulder1" => d_sh[0],
                "shoulder2" => d_sh[1],
                "elbow1" => d_el[0],
                "elbow2" => d_el[1],
                "hip1" => d_hip[0],
                "hip2" => d_hip[1],
                "knee1" => d_knee[0],
                "knee2" => d_knee[1],
                _ => 0.0,
            }
        };
        let mut bodies = [0usize; 11];
        for (i, n) in NAMES.iter().enumerate() {
            let b = &g["bodies"][n];
            let (px, py) = (lpos[i].x, lpos[i].y);
            let pos = Vec2::new(x / PS + cs * px - sn * py, y / PS + sn * px + cs * py);
            let body = self.b.create_body(&BodyDef { position: pos, angle: lang[i] + rot, is_sleeping: sleeping, allow_sleep: true, ..Default::default() });
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
            let anchor = self.b.body(bodies[parent]).world_point(Vec2::new(f(&j["a1"][0]) * 0.5, f(&j["a1"][1]) * 0.5));
            let mut d = RevoluteJointDef::initialize(self.b.body(bodies[parent]), bodies[parent], self.b.body(bodies[child]), bodies[child], anchor);
            d.enable_limit = true;
            d.lower_angle = f(&j["lo"]) - joint_shift(name);
            d.upper_angle = f(&j["hi"]) - joint_shift(name);
            d.max_motor_torque = 4.0;
            if sp.flag("holdPose").unwrap_or(false) && self.level_version < 1.75 {
                d.enable_motor = true;
                d.motor_speed = 0.0;
            }
            let id = self.b.create_revolute_joint(&d);
            joints.push((id, child, f(&g["limits"][limit]), false));
        }
        self.b.c.report_results = true;
        let chest = bodies[0];
        let lim = |k: &str| f(&g["limits"][k]);
        let smash_shapes = [bodies[1], bodies[0], bodies[2]].map(|b| self.b.body(b).shapes[0]);
        let chunk = [f(&g["chunk"][0]), f(&g["chunk"][1]), f(&g["chunk"][2])];
        self.npcs.push(Npc { bodies, joints, smash_shapes, smash_limits: [lim("head"), lim("chest"), lim("pelvis")], chunk, smashed: [false; 3], reversed: reverse, destroy_on_death: sp.flag("destroyJointsUponDeath").unwrap_or(false), tag: npc_tag(sp.num("charIndex").unwrap_or(1.0) as i32), voices: [None; 6], voice_until: 0, voice_prio: -1, dead: false, pieces: Vec::new() });
        Some(chest)
    }

    /// `NPCharacter.triggerSingleActivation`: action list wake from sleep / apply impulse (x, y, spin) / hold pose / release pose (index or name in the XML).
    fn npc_trigger_action(&mut self, chest: BodyId, a: &[String]) {
        let Some(ni) = self.npcs.iter().position(|n| n.bodies[0] == chest) else { return };
        let action = match a.first().map(String::as_str) {
            Some("0" | "wake from sleep") => 0,
            Some("1" | "apply impulse") => 1,
            Some("2" | "hold pose") => 2,
            Some("3" | "release pose") => 3,
            _ => return,
        };
        if action != 0 && self.npcs[ni].destroy_on_death && self.npcs[ni].dead {
            return;
        }
        let bodies = self.npcs[ni].bodies;
        let joints = self.npcs[ni].joints.clone();
        match action {
            0 => self.b.body_mut(chest).wake_up(),
            1 => {
                let param = |k: usize| a.get(1 + k).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
                // chest, head, pelvis, then each limb whose joints are still intact, in the original's order arm1, arm2, leg1, leg2 (joints: neck waist sh1 sh2 el1 el2 hip1 hip2 knee1 knee2)
                let mut ordered = vec![bodies[0], bodies[1], bodies[2]];
                for (sh, el, up, low) in [(2usize, 4usize, 3usize, 5usize), (3, 5, 4, 6), (6, 8, 7, 9), (7, 9, 8, 10)] {
                    if !joints[sh].3 {
                        ordered.push(bodies[up]);
                        if !joints[el].3 { ordered.push(bodies[low]); }
                    }
                }
                for b in ordered {
                    let bd = self.b.body_mut(b);
                    let m = bd.mass;
                    let c = bd.sweep.c;
                    bd.apply_impulse(Vec2::new(param(0) * m, param(1) * m), c);
                }
                if param(2) != 0.0 {
                    self.b.body_mut(chest).angular_velocity += 10.0 * param(2);
                }
            }
            2 => {
                for (id, _, _, _) in &joints {
                    if let Some(angle) = self.b.revolute_angle(*id) {
                        if let box2d::joints::JointKind::Revolute(r) = &mut self.b.joint_mut(*id).kind {
                            r.lower_angle = angle;
                            r.upper_angle = angle;
                        }
                    }
                }
            }
            _ => {
                let d = if self.npcs[ni].reversed { 1.0 } else { 0.0 };
                // (lower, upper) in degrees per joint, mirrored for `reverse`
                let ranges: [(f64, f64); 10] = if d > 0.0 {
                    [(-20.0, 20.0), (-5.0, 5.0), (-60.0, 180.0), (-60.0, 180.0), (0.0, 160.0), (0.0, 160.0), (-10.0, 150.0), (-10.0, 150.0), (-150.0, 0.0), (-150.0, 0.0)]
                } else {
                    [(-20.0, 20.0), (-5.0, 5.0), (-180.0, 60.0), (-180.0, 60.0), (-160.0, 0.0), (-160.0, 0.0), (-150.0, 10.0), (-150.0, 10.0), (0.0, 150.0), (0.0, 150.0)]
                };
                for ((id, _, _, _), (lo, hi)) in joints.iter().zip(ranges) {
                    if let box2d::joints::JointKind::Revolute(r) = &mut self.b.joint_mut(*id).kind {
                        r.lower_angle = lo.to_radians() - r.reference_angle;
                        r.upper_angle = hi.to_radians() - r.reference_angle;
                    }
                }
            }
        }
    }

    /// `NPCharacter.checkJoints`: reaction force² above the limb's limit, or a stretched anchor, breaks the joint and clears the limb's collision group.
    /// Wheel-spin loop of the car-like vehicles: with a wheel contact, |ω| of `spin` above `t[0]` → BikeLoop3, `t[1]` → BikeLoop2, `t[2]` → BikeLoop1.
    fn wheel_loop(&mut self, touching: &[BodyId], spin: BodyId, t: [f64; 3], names: [&str; 3]) {
        if touching.iter().map(|&w| self.b.touching_non_sensor_contacts(w)).sum::<usize>() == 0 {
            return;
        }
        let wheel = self.b.body(spin);
        let w = wheel.angular_velocity.abs();
        let name = if w > t[0] { names[0] } else if w > t[1] { names[1] } else if w > t[2] { names[2] } else { return };
        let at = [(wheel.sweep.c.x * PS) as f32, (wheel.sweep.c.y * PS) as f32];
        self.loops.push((name.to_string(), at, 1.0, spin));
    }

    /// Per-frame loop requests (`actions` of each class). c3 bike / family dad / c5 dirt bike / c2 segway: back-wheel |ω| > 18 / 9 / 1 → BikeLoop3/2/1;
    /// c4 cart: 50 / 25 / 5; c1 chair: the big wheel above 1 → BikeLoop1; heli and mower have their own loops.
    fn update_loops(&mut self) {
        self.loops.clear();
        for mine in &self.mines {
            if mine.done { continue; }
            let c = self.b.body(mine.body).sweep.c;
            let at = [(c.x * PS) as f32, (c.y * PS) as f32];
            if mine.hovering { self.loops.push(("MineHover".into(), at, 1.0, mine.body)); }
            if mine.counting_down { self.loops.push(("HomingMineBeep".into(), at, 1.0, mine.body)); }
        }
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
                self.loops.push(("ElectricMotor1".to_string(), [(w.x * PS) as f32, (w.y * PS) as f32], 1.0, c.back_wheel));
            }
        }
        if let Some(Vehicle::Mower(m)) = self.bike.as_ref() {
            if m.grinding() {
                let c = self.b.body(m.body).sweep.c;
                self.loops.push(("GrindLoop2".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], 1.0, m.body));
            }
        }
        if let Some(Vehicle::Santa(sa)) = self.bike.as_ref() {
            if sa.bell {
                let c = self.b.body(sa.sleigh).sweep.c;
                self.loops.push(("SleighBellLoop".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], 1.0, sa.sleigh));
            }
            if !sa.wheels_gone {
                let touching: usize = sa.wheels.iter().map(|&w| self.b.touching_non_sensor_contacts(w)).sum();
                let back = self.b.body(sa.wheels[0]);
                if touching > 0 && back.angular_velocity.abs() > 12.0 {
                    let c = self.b.body(sa.sleigh).sweep.c;
                    self.loops.push(("SkiLoop".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], 1.0, sa.sleigh));
                }
            }
        }
        if let Some((touching, spin, t)) = wheels {
            self.wheel_loop(&touching, spin, t, ["BikeLoop3", "BikeLoop2", "BikeLoop1"]);
        }
        // rail grinder: `ExplorerRoll3/2/1` above |ω| 50 / 25 / 5 while a wheel touches something; chair jet: `JetBlast` while firing; heli magnet: `MagnetBuzz`
        if let Some(Vehicle::Grinder(g)) = self.bike.as_ref() {
            let (wheels, back) = (vec![g.back_wheel, g.front_wheel], g.back_wheel);
            self.wheel_loop(&wheels, back, [50.0, 25.0, 5.0], ["ExplorerRoll3", "ExplorerRoll2", "ExplorerRoll1"]);
        }
        if let Some(Vehicle::Chair(c)) = self.bike.as_ref() {
            if c.firing {
                let w = self.b.body(c.big_wheel).sweep.c;
                self.loops.push(("JetBlast".to_string(), [(w.x * PS) as f32, (w.y * PS) as f32], 1.0, c.big_wheel));
            }
        }
        if let Some(Vehicle::Heli(h)) = self.bike.as_ref() {
            if h.magnetized {
                let m = self.b.body(h.magnet).sweep.c;
                self.loops.push(("MagnetBuzz".to_string(), [(m.x * PS) as f32, (m.y * PS) as f32], 1.0, h.magnet));
            }
        }
        match self.bike.as_ref() {
            Some(crate::vehicle::Vehicle::Heli(h)) if !h.copter_smashed => {
                let c = self.b.body(h.copter).sweep.c;
                self.loops.push(("HeliLoop".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], h.loop_volume(), h.copter));
            }
            Some(crate::vehicle::Vehicle::Mower(m)) if m.halves.is_none() => {
                let c = self.b.body(m.body).sweep.c;
                self.loops.push(("MowerLoop".to_string(), [(c.x * PS) as f32, (c.y * PS) as f32], m.loop_volume(self), m.body));
            }
            _ => {}
        }
    }

    /// Live gun projectiles for drawing: (body, harpoon?) — blade weapons (arrows carrying a `blade`) have their own art.
    pub fn arrow_bodies(&self) -> impl Iterator<Item = (BodyId, bool)> + '_ {
        self.arrows.iter().filter(|a| a.blade.is_none()).map(|a| (a.body, a.harpoon))
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
    /// Gives every NPC body the linear velocity `v` (m/s) — the oracle scenarios slam NPCs into the floor this way.
    pub fn npc_set_velocity(&mut self, v: [f64; 2]) {
        for b in self.npc_main_bodies() {
            self.b.body_mut(b).linear_velocity = Vec2::new(v[0], v[1]);
        }
    }

    pub fn npc_grind(&mut self, body: BodyId) {
        for n in &mut self.npcs {
            for (part, idx) in [(0, 1), (1, 0), (2, 2)] {
                if n.bodies[idx] == body {
                    n.smashed[part] = true;
                }
            }
        }
    }

    /// `NPCharacter.removeBody` (the mower calls it on a ground target): breaks the joints that held the removed part — head: neck; chest: neck, both
    /// shoulders, waist; pelvis: both hips, waist; an upper arm/leg: its shoulder/hip and its elbow/knee. The original's break handlers also start
    /// blood flows and sounds, which are not repeated here (only the joint effect).
    pub fn npc_remove_body(&mut self, body: BodyId) {
        // bodies: 0 chest 1 head 2 pelvis 3/4 upper arms 5/6 lower arms 7/8 upper legs 9/10 lower legs; joints: 0 neck 1 waist 2/3 shoulders 4/5 elbows 6/7 hips 8/9 knees
        for ni in 0..self.npcs.len() {
            let Some(part) = self.npcs[ni].bodies.iter().position(|&b| b == body) else { continue };
            let brk: &[usize] = match part {
                1 => &[0],
                0 => &[0, 2, 3, 1],
                2 => &[6, 7, 1],
                3 => &[2, 4],
                4 => &[3, 5],
                7 => &[6, 8],
                8 => &[7, 9],
                _ => &[],
            };
            for &ji in brk {
                if ji < self.npcs[ni].joints.len() {
                    self.npc_break_joint(ni, ji, true);
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
        if let Some(ni) = self.npcs.iter().position(|n| n.bodies.contains(&body)) {
            // `NPCharacter.shapeImpale`: chest/pelvis → random vocal; head → `dead` for a fatal impale at the head's centre (or without a point), else a vocal
            let part = self.npcs[ni].bodies.iter().position(|&b| b == body).unwrap_or(0);
            let digit = format!("{}", self.b.body(body).xf.position.x).chars().last().and_then(|c| c.to_digit(10)).unwrap_or(0) as usize;
            let vocal = ["Elbow1", "Elbow2", "Knee1", "Knee2", "Shoulder1", "Shoulder2", "Hip1", "Hip2", "Knee1", "Spikes"][digit];
            match part {
                0 | 2 => self.npcs[ni].voices[5] = Some(vocal),
                1 => {
                    let c = self.b.body(body).sweep.c;
                    if fatal && point.is_none_or(|p| (p.x - c.x).powi(2) + (p.y - c.y).powi(2) < radius2) {
                        self.npcs[ni].dead = true;
                    } else {
                        self.npcs[ni].voices[5] = Some(vocal);
                    }
                }
                _ => {}
            }
            return;
        }
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

    /// `playAreaSoundInstance(name, body, volume)`: like [`play`](Self::play) with a volume multiplier.
    pub(crate) fn play_scaled(&mut self, name: &str, at: Vec2, volume: f64) {
        self.scaled_sounds.push((name.to_string(), [(at.x * PS) as f32, (at.y * PS) as f32], volume as f32));
    }

    pub fn take_scaled_sounds(&mut self) -> Vec<(String, [f32; 2], f32)> {
        std::mem::take(&mut self.scaled_sounds)
    }

    /// Sounds queued since the last call.
    pub fn take_sounds(&mut self) -> Vec<(String, [f32; 2])> {
        std::mem::take(&mut self.sounds)
    }

    pub fn take_trigger_sounds(&mut self) -> Vec<TriggerSound> {
        std::mem::take(&mut self.trigger_sounds)
    }

    /// Original character cameraFocus follows the chest, then the heart after a chest smash.
    pub fn camera_focus_px(&self) -> Option<[f32; 2]> {
        let character = self.character.as_ref()?;
        let body = character.heart.or_else(|| character.bodies.get(&crate::character::Part::Chest).copied())?;
        self.body_center_px(body)
    }

    /// FinishLine.checkResult: any result on the platform checks the living rider's camera focus against its axis-aligned zone.
    fn finish_events(&mut self, events: &[box2d::world::ContactEvent]) {
        let Some(character) = self.character.as_ref() else { return };
        let dead = character.dead;
        let focus = self.camera_focus_px();
        for i in 0..self.finishes.len() {
            let (shape, [x, y], done) = self.finishes[i];
            if done || !events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape)) {
                continue;
            }
            if dead {
                self.finishes[i].2 = true;
            } else if focus.is_some_and(|[fx, fy]| f64::from(fx) > x - 200.0 && f64::from(fx) < x + 200.0 && f64::from(fy) > y - 200.0 && f64::from(fy) < y) {
                self.finishes[i].2 = true;
                self.level_complete = true;
                self.play("Victory", 1, Vec2::new(x / PS, y / PS));
            }
        }
    }

    /// `NPCharacter.checkVocals` (once per step, not for a dead NPC): the highest queued vocal plays from the head as `tag + name`; while one is
    /// sounding only a higher priority replaces it. The queue is cleared afterwards.
    fn check_npc_vocals(&mut self) {
        for ni in 0..self.npcs.len() {
            let n = &self.npcs[ni];
            if n.dead {
                self.npcs[ni].voices = [None; 6];
                continue;
            }
            let top = n.voices.iter().rposition(Option::is_some).map_or(-1, |i| i as i32);
            let busy = self.frame < n.voice_until;
            if top >= 0 && (!busy || top > n.voice_prio) {
                let name = format!("{}{}", n.tag, n.voices[top as usize].unwrap_or(""));
                let at = self.b.body(n.bodies[1]).sweep.c;
                self.npcs[ni].voice_until = self.frame + sound_frames(&name);
                self.npcs[ni].voice_prio = top;
                self.play(&name, 1, at);
            } else if !busy {
                self.npcs[ni].voice_prio = -1;
            }
            self.npcs[ni].voices = [None; 6];
        }
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
                    self.npc_break_joint(ni, ji, true);
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
        self.step_sprays();
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

    /// `ParticleController.createSpray(kind, body, from, to, minSpeed, maxSpeed, angleDeg, perFrame, total)`: positions `from`/`to` are in the body's local px.
    #[allow(clippy::too_many_arguments)]
    fn create_spray(&mut self, kind: crate::blood::ShardKind, body: BodyId, from: (f64, f64), to: (f64, f64), min: f64, max: f64, angle_deg: f64, per_frame: u32, total: u32) {
        let (start, range) = (Vec2::new(from.0 / PS, from.1 / PS), Vec2::new((to.0 - from.0) / PS, (to.1 - from.1) / PS));
        let angle_range = angle_deg.to_radians();
        let rot = range.y.atan2(range.x) + (std::f64::consts::FRAC_PI_2 - angle_range * 0.5);
        self.sprays.push(Spray { body, start, range, min, max, angle_range, rot, per_frame, total, count: 0, kind });
    }

    /// `Mu.step`: each emitter creates `per_frame` particles per step, placed along its segment and thrown at `rot + angleRange·rand` plus the body's velocity there.
    fn step_sprays(&mut self) {
        for i in 0..self.sprays.len() {
            let s = &self.sprays[i];
            if s.count > s.total || !self.b.body(s.body).alive {
                continue;
            }
            for _ in 0..self.sprays[i].per_frame {
                let s = &self.sprays[i];
                if s.count > s.total {
                    break;
                }
                let (body, start, range, min, max, angle_range, rot, kind) = (s.body, s.start, s.range, s.min, s.max, s.angle_range, s.rot, s.kind);
                self.sprays[i].count += 1;
                let t = self.blood.rand();
                let local = Vec2::new(start.x + range.x * t, start.y + range.y * t);
                let speed = self.blood.rand() * (max - min) + min;
                let b = self.b.body(body);
                let a = b.sweep.a + rot + angle_range * self.blood.rand();
                let v = b.linear_velocity_from_local_point(local);
                let at = b.world_point(local);
                self.blood.spawn_sprite(kind, [at.x * PS, at.y * PS], [a.cos() * speed + v.x, a.sin() * speed + v.y]);
            }
        }
        self.sprays.retain(|s| s.count <= s.total && self.b.c.bodies[s.body].alive);
    }

    /// Whether `h` is a broken-special piece (chair, glass, toilet, log, boombox/TV, food chunk) that the sprite renderer draws with its own art.
    pub fn is_piece_with_art(&self, h: BodyId) -> bool {
        self.chair_pieces().any(|p| p.0 == h)
            || self.glass_pieces().any(|p| p.0 == h)
            || self.toilet_pieces().any(|p| p.0 == h)
            || self.log_halves().any(|p| p.0 == h)
            || self.boombox_pieces().any(|p| p.0 == h)
            || self.food_chunks.iter().any(|c| c.0 == h)
    }

    /// Every live body of every NPC, plus its smash chunks and loose special parts (for drawing).
    pub fn npc_draw_bodies(&self) -> Vec<BodyId> {
        self.npcs.iter().flat_map(|n| n.bodies.iter().chain(n.pieces.iter())).chain(self.loose.iter()).copied().filter(|&b| self.b.body(b).alive).collect()
    }

    /// Blood and sound of the original's `neckBreak/torsoBreak/shoulderBreakN/elbowBreakN/hipBreakN/kneeBreakN` (defaults: flows and sound on). `a1`/`a2` are the
    /// joint's local anchors on the parent/child body. Approximation: the neck's chest flow starts at the neck anchor instead of the `spineRef` clip point.
    fn npc_break_effects(&mut self, ni: usize, ji: usize, a1: Vec2, a2: Vec2) {
        // bodies: 0 chest 1 head 2 pelvis 3/4 upper arms 5/6 lower arms 7/8 upper legs 9/10 lower legs
        let b = self.npcs[ni].bodies;
        let at = |w: &World, i: usize| w.b.body(b[i]).sweep.c;
        // `addVocals(name, priority)` of each handler
        let vocal = match ji {
            1 => Some(("Torso", 5)),
            2 => Some(("Shoulder1", 3)),
            3 => Some(("Shoulder2", 3)),
            4 => Some(("Elbow1", 1)),
            5 => Some(("Elbow2", 1)),
            6 => Some(("Hip1", 4)),
            7 => Some(("Hip2", 4)),
            8 => Some(("Knee1", 2)),
            9 => Some(("Knee2", 2)),
            _ => None,
        };
        if let Some((name, prio)) = vocal {
            self.npcs[ni].voices[prio] = Some(name);
        }
        if ji == 0 {
            self.npcs[ni].dead = true;
        }
        match ji {
            0 => {
                self.blood_flow(2.5, 4.0, b[1], Vec2::default(), 90.0, 150);
                self.blood_flow(2.5, 4.0, b[0], a1, 270.0, 500);
                let p = at(self, 1);
                self.play("NeckBreak", 1, p);
            }
            1 => {
                self.blood_flow(2.0, 3.0, b[0], Vec2::default(), 90.0, 500);
                let p = at(self, 2);
                self.play("LimbRip1", 1, p);
            }
            2 | 3 => {
                let arm = if ji == 2 { 3 } else { 4 };
                self.blood_flow(1.0, 3.0, b[arm], a2, 270.0, 200);
                self.blood_flow(0.0, 1.0, b[0], a1, 270.0, 500);
                let p = at(self, arm);
                self.play("LimbRip2", 1, p);
            }
            6 | 7 => {
                let leg = if ji == 6 { 7 } else { 8 };
                self.blood_flow(1.0, 3.0, b[leg], a2, 270.0, 200);
                self.blood_flow(0.0, 1.0, b[2], Vec2::default(), 90.0, 500);
                let p = at(self, leg);
                self.play("LimbRip3", 1, p);
            }
            4 | 5 | 8 | 9 => {
                // elbows and knees: 50-drop blood burst at the child's joint anchor, BoneBreak1..4
                let lower = match ji { 4 => 5, 5 => 6, 8 => 9, _ => 10 };
                let w = self.b.body(b[lower]).world_point(a2);
                self.blood.burst([w.x * PS, w.y * PS], Vec2::default(), 5.0, 15.0, 50);
                let p = at(self, lower);
                self.play("BoneBreak", 4, p);
            }
            _ => {}
        }
    }

    /// Joint break reaction: destroy the joint and clear the collision group of the broken limb and everything hanging below it.
    fn npc_break_joint(&mut self, ni: usize, ji: usize, effects: bool) {
        let (id, child, _, broken) = self.npcs[ni].joints[ji];
        if broken {
            return;
        }
        let anchors = match &self.b.joint(id).kind {
            box2d::joints::JointKind::Revolute(r) => Some((r.local_anchor1, r.local_anchor2)),
            _ => None,
        };
        self.b.destroy_joint(id);
        self.npcs[ni].joints[ji].3 = true;
        if let (true, Some((a1, a2))) = (effects, anchors) {
            self.npc_break_effects(ni, ji, a1, a2);
        }
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
                if part < 2 {
                    // `headSmash`/`chestSmash` set `dead` (the NPC stops voicing)
                    self.npcs[ni].dead = true;
                }
                let bodies = self.npcs[ni].bodies;
                let chunk = self.npcs[ni].chunk;
                // joints: 0 neck 1 waist 2/3 shoulders 4/5 elbows 6/7 hips 8/9 knees
                let brk: &[usize] = match part {
                    0 => &[],
                    1 => &[0, 2, 3, 1],
                    _ => &[6, 7, 1],
                };
                // `chestSmash`/`pelvisSmash` call the break handlers with blood and sound off
                for &ji in brk {
                    self.npc_break_joint(ni, ji, false);
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
                if self.arrows[ai].harpoon && self.arrows[ai].blade.is_none() {
                    // Harpoon.createPrisJoint on a new body: flesh (material bits 6) → shapeImpale + 50-particle blood burst at the tip + HarpoonFlesh1/2;
                    // anything else (solid or static) → HarpoonSolid1/2; each sound plays once per harpoon (the original keeps a handle until it ends)
                    let tip = self.b.body(body).world_point(Vec2::new(26.25 / PS, 0.0));
                    let at = [tip.x * PS, tip.y * PS];
                    if self.b.shape_material(self.b.body(other).shapes[0]) & 6 != 0 {
                        self.shape_impale(other, true, Some(tip), 0.01);
                        self.blood.burst(at, Vec2::default(), 5.0, 15.0, 50);
                        if !self.blood.off {
                            // `HarpoonBloodMC.gotoAndStop(ceil(5·random))` drawn into the harpoon's 106×16 bitmap, once per new body
                            let frame = (self.blood.rand() * 5.0).floor().min(4.0);
                            self.arrows[ai].blood.push((frame, 0.0, 0.0, 1.0));
                        }
                        if self.arrows[ai].flesh_sounds & 1 == 0 {
                            self.arrows[ai].flesh_sounds |= 1;
                            self.play("HarpoonFlesh", 2, tip);
                        }
                    } else if self.arrows[ai].flesh_sounds & 2 == 0 {
                        self.arrows[ai].flesh_sounds |= 2;
                        self.play("HarpoonSolid", 2, tip);
                    }
                }
                if self.arrows[ai].blade.is_some() && self.b.shape_material(self.b.body(other).shapes[0]) & 6 != 0 {
                    let tip = self.b.body(body).sweep.c;
                    self.shape_impale(other, true, Some(tip), 0.01);
                    // BladeWeapon.createPrisJoint on a new flesh body: 50-particle blood burst at its centre, `paintBlood` stamp, BladeFlesh sound
                    let c = self.b.body(other).sweep.c;
                    self.blood.burst([c.x * PS, c.y * PS], Vec2::default(), 5.0, 15.0, 50);
                    let l = self.b.body(body).local_point(c);
                    let flip = if self.arrows[ai].reverse { -1.0 } else { 1.0 };
                    let (rot, mirror) = (self.blood.rand() * std::f64::consts::TAU, if self.blood.rand() > 0.5 { -1.0 } else { 1.0 });
                    if !self.blood.off {
                        self.arrows[ai].blood.push((l.x * PS * flip, l.y * PS, rot, mirror));
                    }
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
                if !self.blood.off {
                    self.spikes[si].blood.push((i + o, frame));
                }
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

    /// Harpoon blood stamps (`HarpoonBloodMC` frame indices 0..4, in the first tuple field) of the harpoon whose body is `body`.
    pub fn harpoon_blood(&self, body: BodyId) -> &[(f64, f64, f64, f64)] {
        self.arrows.iter().find(|a| a.body == body && a.blade.is_none() && a.harpoon).map_or(&[], |a| a.blood.as_slice())
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
            hovering: false,
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
            let mut hovering = false;
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
                    hovering = true;
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
            if hovering != self.mines[mi].hovering {
                self.play(if hovering { "HomingMineFind" } else { "HomingMineLose" }, 1, e);
                self.mines[mi].hovering = hovering;
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
    /// `SpringBox.checkContact`: the first Result on the pad arms the box (the listener is gone until the pad is back).
    fn spring_box_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for s in &mut self.spring_boxes {
            if !s.hit && events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s.pad || e.shape2 == s.pad)) && !s.active {
                s.hit = true;
            }
        }
    }

    /// `Mine.checkContact/actions`: the explosion (sound `MineExplosion`, impulse `10·(1 − d/2)` within 2 m, AABB query of up to 30 shapes).
    fn plain_mine_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for k in 0..self.plain_mines.len() {
            if self.plain_mines[k].done {
                continue;
            }
            let (body, trigger) = (self.plain_mines[k].body, self.plain_mines[k].trigger);
            if !events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == trigger || e.shape2 == trigger)) {
                continue;
            }
            self.plain_mines[k].done = true;
            let c = self.b.body(body).sweep.c;
            self.b.destroy_body(body);
            let found = self.b.query(&box2d::collision::Aabb { lower: Vec2::new(c.x - 2.0, c.y - 2.0), upper: Vec2::new(c.x + 2.0, c.y + 2.0) }, 30);
            for s in found {
                let other = self.b.c.shapes[s].body;
                if other == body || !self.b.c.bodies[other].alive || self.b.body(other).is_static {
                    continue;
                }
                let v = self.b.body(other).sweep.c;
                let (dx, dy) = (v.x - c.x, v.y - c.y);
                let f = 1.0 - dx.hypot(dy).min(2.0) / 2.0;
                let a = dy.atan2(dx);
                self.b.body_mut(other).apply_impulse(Vec2::new(a.cos() * f * 10.0, a.sin() * f * 10.0), v);
            }
            self.play("MineExplosion", 1, c);
        }
    }

    /// `Paddle.checkContact`: a Result on the pad against anything but its own blocker (group 15) arms it.
    fn paddle_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for k in 0..self.paddles.len() {
            let (pad, armed) = (self.paddles[k].pad, self.paddles[k].hit || self.paddles[k].active || self.paddles[k].broken);
            if armed {
                continue;
            }
            let touched = events.iter().any(|e| {
                if e.kind != ContactEventKind::Result {
                    return false;
                }
                let other = if e.shape1 == pad { e.shape2 } else if e.shape2 == pad { e.shape1 } else { return false };
                self.b.c.shapes[other].shape.filter.group_index != 15
            });
            if touched {
                self.paddles[k].hit = true;
            }
        }
    }

    /// `Paddle.actions`: armed + counter 0 → bullet body, motor `dir·speed`, limits 0..90° (or -90°..0), `SpringBoxBounce`; moving, the speed flips to ∓2 past the
    /// target angle and, back at the start, the joint is locked again. A joint stretched by more than 0.43 (squared anchor gap) breaks (pad density 10, `PaddleBreak`).
    fn run_paddles(&mut self) {
        for k in 0..self.paddles.len() {
            if self.paddles[k].broken {
                continue;
            }
            let (body, joint, dir) = (self.paddles[k].body, self.paddles[k].joint, self.paddles[k].dir);
            if self.paddles[k].hit {
                if self.paddles[k].delay_counter == 0 {
                    self.paddles[k].hit = false;
                    self.paddles[k].active = true;
                    self.b.body_mut(body).wake_up();
                    self.b.body_mut(body).set_bullet(true);
                    let speed = dir * self.paddles[k].speed;
                    if let box2d::joints::JointKind::Revolute(r) = &mut self.b.joint_mut(joint).kind {
                        r.motor_speed = speed;
                        (r.lower_angle, r.upper_angle) = if dir < 0.0 { (-std::f64::consts::FRAC_PI_2, 0.0) } else { (0.0, std::f64::consts::FRAC_PI_2) };
                        r.enable_motor = true;
                    }
                    self.paddles[k].delay_counter = self.paddles[k].delay_total;
                    let at = self.b.body(body).sweep.c;
                    self.play("SpringBoxBounce", 1, at);
                } else {
                    self.paddles[k].delay_counter -= 1;
                }
            } else if self.paddles[k].active {
                if let Some((_, gap)) = self.b.revolute_reaction(joint) {
                    if gap * gap > 0.43 {
                        self.b.destroy_joint(joint);
                        self.paddles[k].broken = true;
                        self.paddles[k].active = false;
                        let pad = self.paddles[k].pad;
                        self.b.shape_mut(pad).density = 10.0;
                        self.b.set_mass_from_shapes(body);
                        let at = self.b.body(body).sweep.c;
                        self.play("PaddleBreak", 1, at);
                        continue;
                    }
                }
                let angle = self.b.revolute_angle(joint).unwrap_or(0.0);
                let target = self.paddles[k].target;
                if let box2d::joints::JointKind::Revolute(r) = &mut self.b.joint_mut(joint).kind {
                    let forward = (r.motor_speed > 0.0 && dir > 0.0) || (r.motor_speed < 0.0 && dir < 0.0);
                    if forward {
                        if dir > 0.0 && angle > target {
                            r.motor_speed = -2.0;
                        } else if dir < 0.0 && angle < target {
                            r.motor_speed = 2.0;
                        }
                    } else if (dir > 0.0 && angle < 0.0) || (dir < 0.0 && angle > 0.0) {
                        r.enable_motor = false;
                        r.lower_angle = 0.0;
                        r.upper_angle = 0.0;
                        r.motor_speed = 0.0;
                        self.paddles[k].active = false;
                        self.b.body_mut(body).set_bullet(false);
                    }
                }
            }
        }
    }

    /// `SpringBox.actions`: when armed and the delay counter is zero the pad is driven out (motor speed 10, limits 0..32 px) with a bounce sound; while armed the
    /// counter counts down. Afterwards the motor speed turns to -1 past 32 px and, back at 0, the joint is locked again and the contact listener restored.
    fn run_spring_boxes(&mut self) {
        for k in 0..self.spring_boxes.len() {
            let (body, joint) = (self.spring_boxes[k].body, self.spring_boxes[k].joint);
            if self.spring_boxes[k].hit {
                if self.spring_boxes[k].delay_counter == 0 {
                    self.spring_boxes[k].hit = false;
                    self.spring_boxes[k].active = true;
                    self.b.body_mut(body).wake_up();
                    if let box2d::joints::JointKind::Prismatic(p) = &mut self.b.joint_mut(joint).kind {
                        p.motor_speed = 10.0;
                        p.lower_translation = 0.0;
                        p.upper_translation = 32.0 / PS;
                        p.enable_motor = true;
                    }
                    self.spring_boxes[k].delay_counter = self.spring_boxes[k].delay_total;
                    let at = self.b.body(body).sweep.c;
                    self.play("SpringBoxBounce", 1, at);
                } else {
                    self.spring_boxes[k].delay_counter -= 1;
                }
            } else if self.spring_boxes[k].active {
                let tr = self.b.prismatic_translation(joint).unwrap_or(0.0) * PS;
                if let box2d::joints::JointKind::Prismatic(p) = &mut self.b.joint_mut(joint).kind {
                    if p.motor_speed > 0.0 {
                        if tr > 32.0 {
                            p.motor_speed = -1.0;
                        }
                    } else if p.motor_speed < 0.0 && tr < 0.0 {
                        p.enable_motor = false;
                        p.lower_translation = 0.0;
                        p.upper_translation = 0.0;
                        p.motor_speed = 0.0;
                        self.spring_boxes[k].active = false;
                    }
                }
            }
        }
    }

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
                if !tv {
                    // `Boombox.checkContact`: a shattering hit deletes the ADD listener, so BoomboxHit never plays again (the TV keeps its listener)
                    self.hit_sounds.retain(|h| h.body != body);
                }
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
    /// Registers a helicopter blade shard body (`new MN(body, rightSide, heli)` in `bladeSmash`).
    pub fn add_blade_shard(&mut self, body: BodyId) {
        let shape = self.b.body(body).shapes[0];
        let stab_impulse = self.b.body(body).mass * 3.0;
        self.b.c.report_results = true;
        self.blade_shards.push(BladeShard { body, shape, sensor: None, stab_impulse, pinned: Vec::new(), queue_add: Vec::new(), queue_remove: Vec::new(), previous: None, sounds: 0 });
    }

    fn blade_shard_events(&mut self, events: &[box2d::world::ContactEvent]) {
        let class = |w: &Self, e: &box2d::world::ContactEvent| (matches!(w.b.c.shapes[e.shape1].shape.kind, box2d::shape::ShapeKind::Circle(_)), matches!(w.b.c.shapes[e.shape2].shape.kind, box2d::shape::ShapeKind::Circle(_)));
        let last_eval_normal = |w: &Self, c: ContactClass| events.iter().rev().find(|e| e.kind != ContactEventKind::Result && class(w, e) == c).map(|e| e.normal);
        for k in 0..self.blade_shards.len() {
            let (b, shape) = (self.blade_shards[k].body, self.blade_shards[k].shape);
            if !self.b.c.bodies[b].alive {
                continue;
            }
            if self.blade_shards[k].sensor.is_none() {
                // `checkContact`: only while the plain solid shape is listening
                let stab = events.iter().any(|e| {
                    e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape) && e.normal_impulse > self.blade_shards[k].stab_impulse && {
                        let other = if e.shape1 == shape { e.shape2 } else { e.shape1 };
                        self.b.shape_material(other) & 2 != 0 && !self.b.c.shapes[other].shape.is_sensor
                    }
                });
                if stab {
                    let verts = match &self.b.c.shapes[shape].shape.kind {
                        box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices.clone(),
                        _ => continue,
                    };
                    self.b.shape_mut(shape).filter.mask_bits = 8;
                    self.b.refilter(shape);
                    let mut sd = ShapeDef::new(Geometry::Polygon(verts));
                    sd.is_sensor = true;
                    sd.filter = Filter { category_bits: 8, mask_bits: 4, group_index: 0 };
                    let sensor = self.b.create_shape(b, &sd);
                    self.blade_shards[k].sensor = Some(sensor);
                }
                continue;
            }
            let Some(sensor) = self.blade_shards[k].sensor else { continue };
            for e in events.iter().filter(|e| e.shape1 == sensor || e.shape2 == sensor) {
                let (other, flip) = if e.shape1 == sensor { (e.shape2, 1.0) } else { (e.shape1, -1.0) };
                if self.b.shape_material(other) & 2 == 0 {
                    continue;
                }
                let ob = self.b.c.shapes[other].body;
                let aliased_class = (flip > 0.0).then(|| class(self, e));
                match e.kind {
                    ContactEventKind::Add => self.blade_shards[k].queue_add.push((ob, Vec2::new(e.normal.x * flip, e.normal.y * flip), aliased_class)),
                    ContactEventKind::Remove if self.blade_shards[k].pinned.iter().any(|p| p.0 == ob) || self.blade_shards[k].queue_add.iter().any(|p| p.0 == ob) => self.blade_shards[k].queue_remove.push(ob),
                    _ => {}
                }
            }
            for (other, normal, aliased) in std::mem::take(&mut self.blade_shards[k].queue_add) {
                if !self.b.c.bodies[other].alive {
                    continue;
                }
                let normal = aliased.and_then(|c| last_eval_normal(self, c)).unwrap_or(normal);
                if let Some(p) = self.blade_shards[k].pinned.iter_mut().find(|p| p.0 == other) {
                    p.2 += 1;
                    continue;
                }
                let at = self.b.body(other).sweep.c;
                self.shape_impale(other, true, None, 0.0);
                let mut jd = PrismaticJointDef::initialize(self.b.body(b), b, self.b.body(other), other, at, normal);
                jd.collide_connected = true;
                jd.enable_motor = true;
                jd.max_motor_force = 30.0;
                jd.motor_speed = 0.0;
                let j = self.b.create_prismatic_joint(&jd);
                self.blade_shards[k].pinned.push((other, j, 1));
                if self.blade_shards[k].previous != Some(other) {
                    self.blade_shards[k].previous = Some(other);
                    if !self.blood.off {
                        self.blood.burst([at.x * PS, at.y * PS], Vec2::default(), 5.0, 15.0, 50);
                    }
                    if self.blade_shards[k].sounds < 1 {
                        self.blade_shards[k].sounds += 1;
                        self.play("ImpaleSpikes", 3, at);
                    }
                }
            }
            let had_removals = !self.blade_shards[k].queue_remove.is_empty();
            for other in std::mem::take(&mut self.blade_shards[k].queue_remove) {
                if let Some(i) = self.blade_shards[k].pinned.iter().position(|p| p.0 == other) {
                    self.blade_shards[k].pinned[i].2 -= 1;
                    if self.blade_shards[k].pinned[i].2 == 0 {
                        let (_, j, _) = self.blade_shards[k].pinned.remove(i);
                        self.b.destroy_joint(j);
                    }
                }
            }
            if had_removals && self.blade_shards[k].pinned.is_empty() {
                // `checkZeroJoints` (only after removals): back to a plain solid shard
                self.b.shape_mut(shape).filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
                self.b.refilter(shape);
                self.b.destroy_shape(sensor);
                self.blade_shards[k].sensor = None;
                self.blade_shards[k].sounds = 0;
            }
        }
    }

    fn glass_events(&mut self, events: &[box2d::world::ContactEvent]) {
        // `b2*Contact.Evaluate` fills one static `s_evalCP` per contact class (circle/circle, polygon/circle, polygon/polygon)
        let class = |w: &Self, e: &box2d::world::ContactEvent| (matches!(w.b.c.shapes[e.shape1].shape.kind, box2d::shape::ShapeKind::Circle(_)), matches!(w.b.c.shapes[e.shape2].shape.kind, box2d::shape::ShapeKind::Circle(_)));
        let last_eval_normal = |w: &Self, c: (bool, bool)| events.iter().rev().find(|e| e.kind != ContactEventKind::Result && class(w, e) == c).map(|e| e.normal);
        // Shards (`nd`/`te`): shatter above 10·mass; in stab mode a flesh hit above `stab_limit` turns the piece into a pin (sensor + prismatic joints).
        for k in 0..self.glasses.len() {
            let stabbing = self.glasses[k].stabbing;
            for s in 0..self.glasses[k].shards.len() {
                let b = self.glasses[k].shards[s].body;
                if !self.b.body(b).alive {
                    continue;
                }
                let (shape, shatter, stab_limit) = (self.glasses[k].shards[s].shape, self.glasses[k].shards[s].shatter, self.glasses[k].shards[s].stab_limit);
                let stab_limit = if stabbing { stab_limit } else { shatter };
                let mut broke = false;
                let mut begin_stab = false;
                for e in events.iter().filter(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape) && e.normal_impulse > stab_limit) {
                    if e.normal_impulse > shatter {
                        broke = true;
                        break;
                    }
                    let other = if e.shape1 == shape { e.shape2 } else { e.shape1 };
                    if self.b.shape_material(other) & 2 != 0 && !self.b.c.shapes[other].shape.is_sensor && self.glasses[k].shards[s].sensor.is_none() {
                        begin_stab = true;
                    }
                }
                if broke {
                    let sh = &self.glasses[k].shards[s];
                    let (particles, suffix, sensor) = (sh.particles, sh.suffix, sh.sensor);
                    for (_, j, _) in std::mem::take(&mut self.glasses[k].shards[s].pinned) {
                        self.b.destroy_joint(j);
                    }
                    let _ = sensor;
                    let src = self.b.body(b).clone();
                    self.rect_burst(&src, b, crate::blood::ShardKind::Food(5), 10.0, particles);
                    self.play(&format!("Glass{suffix}"), 2, src.sweep.c);
                    self.b.destroy_body(b);
                    continue;
                }
                if begin_stab {
                    // `te.singleAction` (stab): the solid shape only collides with category 8, a sensor copy (mask 4) watches for flesh
                    let verts = match &self.b.c.shapes[shape].shape.kind {
                        box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices.clone(),
                        _ => continue,
                    };
                    self.b.shape_mut(shape).filter.mask_bits = 8;
                    self.b.refilter(shape);
                    let mut sd = ShapeDef::new(Geometry::Polygon(verts));
                    sd.is_sensor = true;
                    sd.filter = Filter { category_bits: 8, mask_bits: 4, group_index: 0 };
                    let sensor = self.b.create_shape(b, &sd);
                    self.glasses[k].shards[s].sensor = Some(sensor);
                }
                let Some(sensor) = self.glasses[k].shards[s].sensor else { continue };
                for e in events.iter().filter(|e| e.shape1 == sensor || e.shape2 == sensor) {
                    let (other, flip) = if e.shape1 == sensor { (e.shape2, 1.0) } else { (e.shape1, -1.0) };
                    if self.b.shape_material(other) & 2 == 0 {
                        continue;
                    }
                    let ob = self.b.c.shapes[other].body;
                    let aliased_class = (flip > 0.0).then(|| class(self, e));
                    match e.kind {
                        ContactEventKind::Add => self.glasses[k].shards[s].queue_add.push((ob, Vec2::new(e.normal.x * flip, e.normal.y * flip), aliased_class)),
                        ContactEventKind::Remove if self.glasses[k].shards[s].pinned.iter().any(|p| p.0 == ob) || self.glasses[k].shards[s].queue_add.iter().any(|p| p.0 == ob) => self.glasses[k].shards[s].queue_remove.push(ob),
                        _ => {}
                    }
                }
                for (other, normal, aliased) in std::mem::take(&mut self.glasses[k].shards[s].queue_add) {
                    let normal = aliased.and_then(|c| last_eval_normal(self, c)).unwrap_or(normal);
                    if !self.b.c.bodies[other].alive {
                        continue;
                    }
                    if let Some(p) = self.glasses[k].shards[s].pinned.iter_mut().find(|p| p.0 == other) {
                        p.2 += 1;
                        continue;
                    }
                    let fatal = self.glasses[k].shards[s].fatal;
                    let at = self.b.body(other).sweep.c;
                    self.shape_impale(other, fatal, None, 0.0);
                    let mut jd = PrismaticJointDef::initialize(self.b.body(b), b, self.b.body(other), other, at, normal);
                    jd.collide_connected = true;
                    jd.enable_motor = true;
                    jd.max_motor_force = 30.0;
                    jd.motor_speed = 0.0;
                    let j = self.b.create_prismatic_joint(&jd);
                    let bp = self.glasses[k].shards[s].blood_particles;
                    self.glasses[k].shards[s].pinned.push((other, j, 1));
                    if !self.blood.off {
                        let l = self.b.body(b).local_point(at);
                        let (rot, mirror) = (self.blood.rand() * std::f64::consts::TAU, if self.blood.rand() > 0.5 { -1.0 } else { 1.0 });
                        self.glasses[k].shards[s].blood.push((l.x * PS, l.y * PS, rot, mirror));
                    }
                    self.blood.burst([at.x * PS, at.y * PS], Vec2::default(), 5.0, 15.0, bp);
                    if self.glasses[k].shards[s].sounds < 1 {
                        self.glasses[k].shards[s].sounds += 1;
                        self.play("ImpaleSpikes", 3, at);
                    }
                }
                let had_removals = !self.glasses[k].shards[s].queue_remove.is_empty();
                for other in std::mem::take(&mut self.glasses[k].shards[s].queue_remove) {
                    if let Some(i) = self.glasses[k].shards[s].pinned.iter().position(|p| p.0 == other) {
                        self.glasses[k].shards[s].pinned[i].2 -= 1;
                        if self.glasses[k].shards[s].pinned[i].2 == 0 {
                            let (_, j, _) = self.glasses[k].shards[s].pinned.remove(i);
                            self.b.destroy_joint(j);
                        }
                    }
                }
                if had_removals && self.glasses[k].shards[s].pinned.is_empty() {
                    // `checkZeroJoints` (only after removals, like `actions`): back to a plain solid piece
                    self.b.shape_mut(shape).filter.mask_bits = 0xFFFF;
                    self.b.refilter(shape);
                    self.b.destroy_shape(sensor);
                    self.glasses[k].shards[s].sensor = None;
                }
            }
        }
        for k in 0..self.glasses.len() {
            if self.glasses[k].done {
                continue;
            }
            let (body, shape, limit) = (self.glasses[k].body, self.glasses[k].shape, self.glasses[k].smash_impulse);
            let Some(hit) = events.iter().find(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape) && e.normal_impulse > limit) else { continue };
            let src = self.b.body(body).clone();
            let mut impact = src.local_point(hit.position);
            self.glasses[k].done = true;
            self.hit_sounds.retain(|h| h.body != body);
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
                    let psh = self.b.create_shape(piece, &sd);
                    self.b.set_mass_from_shapes(piece);
                    let pm = self.b.body(piece).mass;
                    let (psuffix, pparticles) = if pm < 0.75 { ("Light", 50) } else if pm < 4.0 { ("Mid", 100) } else { ("Heavy", 200) };
                    self.b.c.report_results = true;
                    let wide = tri_width_px(&[impact, Vec2::new(a.0, a.1), Vec2::new(b.0, b.1)]) >= 15.0;
                    let stabs = self.glasses[k].stabbing;
                    self.glasses[k].shards.push(Shard {
                        body: piece,
                        shape: psh,
                        shatter: 10.0 * pm,
                        stab_limit: if pm < 0.1 { 0.0 } else { (5.0 * pm).min(2.5) },
                        particles: pparticles,
                        blood_particles: if stabs && pm < 0.1 { 15 } else { 50 },
                        suffix: psuffix,
                        fatal: stabs && pm >= 0.25 && wide,
                        sensor: None,
                        pinned: Vec::new(),
                        queue_add: Vec::new(),
                        queue_remove: Vec::new(),
                        sounds: 0,
                        blood: Vec::new(),
                    });
                    let com = self.b.body(piece).sweep.c;
                    self.b.body_mut(piece).angular_velocity = w;
                    self.b.body_mut(piece).linear_velocity = src.linear_velocity_from_world_point(com);
                    self.glasses[k].pieces.push((piece, [(impact.x * PS, impact.y * PS), (a.0 * PS, a.1 * PS), (b.0 * PS, b.1 * PS)]));
                    self.loose.push(piece);
                }
            }
        }
    }

    /// `Toilet.singleAction`: the base (with the bowl) and/or the back break off into new bodies at the toilet's centre with its velocity; once both are
    /// gone the toilet body is removed. Pieces keep density 2, μ.3, e.1.
    fn toilet_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for k in 0..self.toilets.len() {
            let (body, sign) = (self.toilets[k].body, self.toilets[k].sign);
            let hit = |s: Option<ShapeId>| s.is_some_and(|s| events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == s || e.shape2 == s) && e.normal_impulse > 12.0));
            let (t_back, t_base) = (hit(self.toilets[k].back), hit(self.toilets[k].base) || hit(self.toilets[k].bowl));
            if !(t_back || t_base) {
                continue;
            }
            let src = self.b.body(body).clone();
            let (at, ang, vel, w) = (src.sweep.c, src.sweep.a, src.linear_velocity, src.angular_velocity);
            let v = |x: f64, y: f64| Vec2::new(x * sign / PS, y / PS);
            let make = |this: &mut Self, geoms: Vec<Geometry>| -> BodyId {
                let piece = this.b.create_body(&BodyDef { position: at, angle: ang, ..Default::default() });
                for g in geoms {
                    let mut sd = ShapeDef::new(g);
                    sd.density = 2.0;
                    sd.friction = 0.3;
                    sd.restitution = 0.1;
                    sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
                    this.b.create_shape(piece, &sd);
                }
                this.b.set_mass_from_shapes(piece);
                this.b.body_mut(piece).angular_velocity = w;
                this.b.body_mut(piece).linear_velocity = vel;
                this.loose.push(piece);
                piece
            };
            let bx = |hx: f64, hy: f64, cx: f64, cy: f64| ShapeDef::oriented_box(hx / PS, hy / PS, v(cx, cy), 0.0).geometry;
            if t_base && self.toilets[k].base.is_some() {
                let (base, bowl) = (self.toilets[k].base.take().unwrap(), self.toilets[k].bowl.take().unwrap());
                self.b.destroy_shape(base);
                self.b.destroy_shape(bowl);
                let mut b1 = vec![v(-25.0, 4.5), v(2.0, 4.5), v(17.0, 25.0), v(3.5, 30.5), v(-22.0, 17.0)];
                if sign < 0.0 {
                    b1 = vec![b1[1], b1[0], b1[4], b1[3], b1[2]];
                }
                let p1 = make(self, vec![bx(10.5, 7.5, 3.0, 31.5), Geometry::Polygon(b1)]);
                let (g, wv, x, bq) = (v(8.0, 4.5), v(31.0, 4.5), v(25.0, 20.0), v(17.0, 25.0));
                let quad = if sign > 0.0 { vec![g, wv, x, bq] } else { vec![wv, g, bq, x] };
                let p2 = make(self, vec![Geometry::Polygon(quad)]);
                self.rect_burst(&src, p1, crate::blood::ShardKind::Food(6), 20.0, 30);
                let (from, to) = if sign > 0.0 { ((11.0, 22.0), (1.0, 13.0)) } else { ((-1.0, 13.0), (-11.0, 22.0)) };
                self.create_spray(crate::blood::ShardKind::Food(crate::blood::WATER), p1, from, to, 0.0, 2.0, 15.0, 3, 95);
                self.toilets[k].pieces.push((p1, 0, sign));
                self.toilets[k].pieces.push((p2, 1, sign));
                if self.frame >= self.toilets[k].sound_until {
                    // `playSound`: one ToiletSmash handle per toilet until the sound ends
                    self.play("ToiletSmash", 3, at);
                    self.toilets[k].sound_until = self.frame + sound_frames(&self.sounds.last().map(|s| s.0.clone()).unwrap_or_default());
                }
            }
            if t_back && self.toilets[k].back.is_some() {
                let back = self.toilets[k].back.take().unwrap();
                self.b.destroy_shape(back);
                let p1 = make(self, vec![bx(10.0, 12.5, -14.0, -31.0)]);
                let p2 = make(self, vec![bx(10.0, 11.0, -14.0, -7.0)]);
                self.rect_burst(&src, p2, crate::blood::ShardKind::Food(6), 20.0, 30);
                let (from, to) = if sign > 0.0 { ((-7.0, -14.0), (-23.0, -13.0)) } else { ((23.0, -13.0), (7.0, -14.0)) };
                self.create_spray(crate::blood::ShardKind::Food(crate::blood::WATER), p2, from, to, 0.0, 1.0, 15.0, 3, 80);
                self.toilets[k].pieces.push((p1, 2, sign));
                self.toilets[k].pieces.push((p2, 3, sign));
                if self.frame >= self.toilets[k].sound_until {
                    // `playSound`: one ToiletSmash handle per toilet until the sound ends
                    self.play("ToiletSmash", 3, at);
                    self.toilets[k].sound_until = self.frame + sound_frames(&self.sounds.last().map(|s| s.0.clone()).unwrap_or_default());
                }
            }
            if self.toilets[k].base.is_none() && self.toilets[k].back.is_none() {
                self.b.destroy_body(body);
                self.toilets[k].done = true;
            } else {
                self.b.set_mass_from_shapes(body);
            }
        }
    }

    /// `Log.actions`: above an impulse of 2250 the log becomes two boxes (w × h/2, upper at y −h/4, lower at +h/4; density 30, μ.7, e.2) at its centre
    /// with its velocity, wood chips fly and `LumberBreak` plays.
    fn log_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for k in 0..self.logs.len() {
            if self.logs[k].done {
                continue;
            }
            let (body, shape, w, h) = (self.logs[k].body, self.logs[k].shape, self.logs[k].w, self.logs[k].h);
            if !events.iter().any(|e| e.kind == ContactEventKind::Result && (e.shape1 == shape || e.shape2 == shape) && e.normal_impulse > 2250.0) {
                continue;
            }
            self.logs[k].done = true;
            let src = self.b.body(body).clone();
            let (at, ang, vel, wv) = (src.sweep.c, src.sweep.a, src.linear_velocity, src.angular_velocity);
            let mut halves = Vec::new();
            for (i, cy) in [-h / 4.0, h / 4.0].into_iter().enumerate() {
                let piece = self.b.create_body(&BodyDef { position: at, angle: ang, ..Default::default() });
                let mut sd = ShapeDef::oriented_box(w / 2.0 / PS, h / 4.0 / PS, Vec2::new(0.0, cy / PS), 0.0);
                sd.density = 30.0;
                sd.friction = 0.7;
                sd.restitution = 0.2;
                sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
                self.b.create_shape(piece, &sd);
                self.b.set_mass_from_shapes(piece);
                self.b.body_mut(piece).linear_velocity = vel;
                self.b.body_mut(piece).angular_velocity = wv;
                self.loose.push(piece);
                halves.push((piece, i as u8));
            }
            self.rect_burst(&src, body, crate::blood::ShardKind::Food(7), 10.0, 30);
            self.b.destroy_body(body);
            self.play("LumberBreak", 1, at);
            self.logs[k].halves = halves;
        }
    }

    /// Halves of broken logs: (body, 0 upper / 1 lower, sprite stretch x, y).
    pub fn log_halves(&self) -> impl Iterator<Item = (BodyId, u8, f64, f64)> + '_ {
        self.logs.iter().flat_map(|l| l.halves.iter().map(move |&(b, i)| (b, i, l.stretch.0, l.stretch.1)))
    }

    /// Pieces of broken toilets: (body, 0 base1 / 1 base2 / 2 back1 / 3 back2, mirror).
    pub fn toilet_pieces(&self) -> impl Iterator<Item = (BodyId, u8, f64)> + '_ {
        self.toilets.iter().flat_map(|t| t.pieces.iter().copied())
    }

    /// Which parts of the toilet on `body` are still attached: (back, base).
    pub fn toilet_parts(&self, body: Option<BodyId>) -> (bool, bool) {
        body.and_then(|b| self.toilets.iter().find(|t| t.body == b)).map_or((true, true), |t| (t.back.is_some(), t.base.is_some()))
    }

    /// `GlassBloodMC` stamps (clip-local px x, y, rotation, scaleX) of the stab shard whose body is `body`.
    pub fn glass_blood(&self, body: BodyId) -> &[(f64, f64, f64, f64)] {
        self.glasses.iter().flat_map(|g| g.shards.iter()).find(|s| s.body == body).map_or(&[], |s| s.blood.as_slice())
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

    /// The original requires both flesh material and a living character owning the contacting shape.
    fn living_token_owner(&self, body: BodyId) -> bool {
        let has = |d: &crate::character::Ragdoll| !d.dead && d.bodies.values().any(|&b| b == body);
        if self.character.as_ref().is_some_and(has) {
            return true;
        }
        let passenger = match self.bike.as_ref() {
            Some(crate::vehicle::Vehicle::Bike(b)) => b.kid.as_ref().is_some_and(|k| has(&k.doll)),
            Some(crate::vehicle::Vehicle::Dirt(d)) => d.girl.as_ref().is_some_and(|g| has(&g.doll)),
            Some(crate::vehicle::Vehicle::Family(f)) => has(&f.daughter.doll) || has(&f.son.doll) || f.dad.kid.as_ref().is_some_and(|k| has(&k.doll)),
            Some(crate::vehicle::Vehicle::Santa(s)) => has(&s.elf1.doll) || has(&s.elf2.doll),
            _ => false,
        };
        passenger || self.npcs.iter().any(|npc| !npc.smashed[0] && npc.bodies.contains(&body))
    }

    /// Token `checkContact`/`singleAction`: a living character's flesh entering the sensor collects it.
    fn token_events(&mut self, events: &[box2d::world::ContactEvent]) {
        for ev in events.iter().filter(|e| e.kind == ContactEventKind::Add) {
            let Some(k) = self.tokens.iter().position(|t| t.1 == ev.shape1 || t.1 == ev.shape2) else { continue };
            let other = if self.tokens[k].1 == ev.shape1 { ev.shape2 } else { ev.shape1 };
            if self.b.shape_material(other) & 2 != 0 && self.living_token_owner(self.b.shape_body(other)) {
                let (body, _) = self.tokens.remove(k);
                self.b.destroy_body(body);
                self.tokens_found += 1;
                let at = self.b.body(body).sweep.c;
                self.play("Bleep3", 1, at);
                if self.tokens.is_empty() {
                    self.level_complete = true;
                    self.play("Victory", 1, at);
                }
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
        doll.finish_joints(self);
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
    fn character_triggers_use_ragdoll_central_body() {
        let xml = "<levelXML><info v='1.99'/><triggers><t x='1000' y='200' w='120' h='60' b='1' t='0' r='1'/><t x='1000' y='200' w='120' h='60' b='2' t='0' r='1'/></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        assert!(w.spawn_character(1, [1000.0, 100.0]));
        for _ in 0..60 { w.step(); }
        assert!(w.events.iter().any(|e| e.trigger == 0));
        assert!(w.events.iter().any(|e| e.trigger == 1));
    }

    #[test]
    fn any_character_trigger_accepts_npc_but_main_only_does_not() {
        let xml = "<levelXML><info v='1.99'/><specials><sp t='17' p0='1000' p1='100' p2='0' p3='1' p4='f' p5='f'/></specials><triggers><t x='1000' y='200' w='120' h='60' b='1' t='0' r='1'/><t x='1000' y='200' w='120' h='60' b='2' t='0' r='1'/></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        for _ in 0..60 { w.step(); }
        assert!(!w.events.iter().any(|e| e.trigger == 0));
        assert!(w.events.iter().any(|e| e.trigger == 1));
    }

    #[test]
    fn main_character_trigger_sees_heart_without_counting_it_as_owned_flesh() {
        let xml = "<levelXML><info v='1.99'/><triggers><t x='1000' y='300' w='120' h='60' b='1' t='0' r='1'/><t x='1000' y='300' w='120' h='60' b='2' t='0' r='1'/></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        assert!(w.spawn_character(1, [1000.0, 100.0]));
        let mut rider = w.character.take().unwrap();
        rider.smash_chest(&mut w);
        let heart = rider.heart.unwrap();
        w.character = Some(rider);
        // Remove the rest of the ragdoll so only the heart can enter the sensors.
        let other: Vec<_> = w.character.as_ref().unwrap().bodies.values().copied().filter(|&b| w.b.body(b).alive).collect();
        for body in other { w.b.destroy_body(body); }
        assert!(w.b.body(heart).alive);
        for _ in 0..80 { w.step(); }
        assert!(w.events.iter().any(|e| e.trigger == 0));
        assert!(!w.events.iter().any(|e| e.trigger == 1));
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
             <t x='0' y='0' w='50' h='50' a='0' b='6' t='1' r='1' sd='f'><t i='1'/></t>\
             <t x='0' y='0' w='50' h='50' a='0' b='5' t='0' r='1' sd='f'/></triggers></levelXML>",
        ).unwrap();
        let mut w = World::from_level(&level);
        w.step();
        assert!(w.events.is_empty());
        assert!(w.mouse_trigger_enabled(0));
        w.click_trigger(0);
        assert!(!w.mouse_trigger_enabled(0));
        w.click_trigger(0); // once only
        assert_eq!(w.events.iter().map(|e| e.trigger).collect::<Vec<_>>(), vec![0, 1]);
    }

    #[test]
    fn trigger_targets_can_disable_and_enable_other_triggers() {
        let xml = "<levelXML><info v='1.99'/><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='2'><t i='2'><a i='1'/></t></t><t x='100' y='0' w='50' h='50' b='6' t='1' r='2'><t i='2'><a i='2'/></t></t><t x='200' y='0' w='50' h='50' b='6' t='3' r='2' d='0.2'/></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        w.click_trigger(2);
        w.click_trigger(0);
        assert!(!w.mouse_trigger_enabled(2));
        for _ in 0..20 { w.step(); }
        assert!(!w.level_complete, "disabling cancels pending activation");
        w.click_trigger(2);
        assert!(!w.level_complete);
        w.click_trigger(1);
        assert!(w.mouse_trigger_enabled(2));
        w.click_trigger(2);
        for _ in 0..20 { w.step(); }
        assert!(w.level_complete);
    }

    #[test]
    fn enabling_trigger_detects_body_already_in_zone() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='500' p1='300' p2='300' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/><sh t='1' p0='500' p1='250' p2='30' p3='30' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><t i='1'><a i='2'/></t></t><t x='500' y='265' w='120' h='80' b='3' t='3' r='1' sd='t'/></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        for _ in 0..60 { w.step(); }
        assert!(!w.level_complete);
        w.click_trigger(0);
        for _ in 0..5 { w.step(); }
        assert!(w.level_complete, "refilter should create a fresh contact for an overlapping body");
    }

    #[test]
    fn mouse_repeat_three_stops_on_exit_but_four_keeps_running() {
        for repeat in [3, 4] {
            let xml = format!("<levelXML><info v='1.99'/><triggers><t x='100' y='100' w='50' h='50' b='6' t='3' r='{repeat}' i='0.1'/></triggers></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            w.click_trigger(0);
            for _ in 0..8 { w.step(); }
            assert!(!w.events.is_empty(), "repeat {repeat} never fired");
            w.release_trigger(0);
            let before = w.events.len();
            for _ in 0..12 { w.step(); }
            if repeat == 3 {
                assert_eq!(w.events.len(), before, "repeat 3 continued after exit");
            } else {
                assert!(w.events.len() > before, "repeat 4 stopped on exit");
            }
        }
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
    fn group_trigger_uses_group_action_indices() {
        let xml = "<levelXML><info v='1.99'/><groups><g x='500' y='100' r='0' ox='0' oy='0' s='f' f='f' o='100' im='f' fr='f'><sh t='0' p0='0' p1='0' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></g></groups><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><g i='0'><a i='2' p0='3' p1='0' p2='0'/></g></t><t x='100' y='0' w='50' h='50' b='6' t='1' r='1'><g i='0'><a i='3'/></g></t><t x='200' y='0' w='50' h='50' b='6' t='1' r='1'><g i='0'><a i='4'/></g></t></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let body = w.group_bodies[0];
        w.click_trigger(0);
        assert!((w.b.body(body).linear_velocity.x - 3.0).abs() < 1e-9);
        w.click_trigger(1);
        assert_eq!(w.b.body(body).mass, 0.0);
        w.click_trigger(2);
        assert!(w.b.body(body).mass > 0.0);
    }

    #[test]
    fn group_delete_shapes_keeps_art_until_delete_self() {
        let xml = "<levelXML><info v='1.99'/><groups><g x='500' y='100' r='0' ox='0' oy='0' s='f' f='f' o='100' im='f' fr='f'><sh t='0' p0='0' p1='0' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='16711680' p9='-1' p10='100' p11='1'/></g></groups><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><g i='0'><a i='5'/></g></t><t x='100' y='0' w='50' h='50' b='6' t='1' r='1'><g i='0'><a i='6'/></g></t></triggers></levelXML>";
        let level = crate::level::parse(xml).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..15 { w.step(); }
        let pose = w.body_position_px(w.group_bodies[0]).unwrap();
        w.click_trigger(0);
        assert_eq!(w.group_frozen_pose[0].unwrap().0, pose);
        let image = crate::render::render_level_at(&level, &w, 300, 300, pose);
        assert!(image.get_pixel(150, 150)[0] > image.get_pixel(150, 150)[1]);
        w.click_trigger(1);
        let image = crate::render::render_level_at(&level, &w, 300, 300, pose);
        assert_eq!(image.get_pixel(150, 150)[0], image.get_pixel(150, 150)[1]);
    }

    #[test]
    fn trigger_unfreezes_previously_fixed_dynamic_shape() {
        let level = crate::level::parse("<levelXML><info v='1.99'/><shapes><sh t='1' p0='500' p1='100' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='2' p12='0'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='1'/></sh></t><t x='100' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='2'/></sh></t></triggers></levelXML>").unwrap();
        let mut w = World::from_level(&level);
        let body = w.shape_bodies[0].unwrap();
        let shape = w.b.body(body).shapes[0];
        let original_filter = w.b.c.shapes[shape].shape.filter;
        w.click_trigger(0);
        assert!(!w.b.c.bodies[body].alive);
        assert_eq!(w.shape_bodies[0], Some(w.level_body));
        let static_shape = w.static_shape_fixtures[0].unwrap();
        assert_eq!(w.b.c.shapes[static_shape].shape.filter.category_bits, 24);
        w.click_trigger(1);
        let new_body = w.shape_bodies[0].unwrap();
        assert!(w.b.body(new_body).mass > 0.0);
        let new_shape = w.b.body(new_body).shapes[0];
        assert_eq!(w.b.c.shapes[new_shape].shape.filter, original_filter);
        let initial_y = w.position_px(0).unwrap()[1];
        for _ in 0..10 { w.step(); }
        assert!(w.position_px(0).unwrap()[1] > initial_y);
    }

    #[test]
    fn freezing_and_unfreezing_polygon_keeps_rendered_pose() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='3' p0='500' p1='200' p2='100' p3='100' p4='30' p5='f' p6='f' p7='1' p8='16711680' p9='-1' p10='100' p11='1'><v f='t' id='1' n='3' v0='0_0' v1='90_0' v2='0_60'/></sh></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='1'/></sh></t><t x='100' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='2'/></sh></t></triggers></levelXML>";
        let level = crate::level::parse(xml).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..10 { w.step(); }
        let camera = [500.0, 220.0];
        let before = crate::render::render_level_at(&level, &w, 300, 300, camera);
        w.click_trigger(0);
        let fixed = crate::render::render_level_at(&level, &w, 300, 300, camera);
        assert_eq!(before, fixed, "freezing should not move the artwork");
        w.click_trigger(1);
        let unfixed = crate::render::render_level_at(&level, &w, 300, 300, camera);
        assert_eq!(fixed, unfixed, "unfreezing should not move the artwork");
    }

    #[test]
    fn trigger_changes_collision_and_refilters_existing_contacts() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='500' p1='300' p2='500' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/><sh t='1' p0='500' p1='200' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='1'><a i='7' p0='3'/></sh></t></triggers></levelXML>";
        let level = crate::level::parse(xml).unwrap();
        let mut w = World::from_level(&level);
        for _ in 0..60 { w.step(); }
        assert!(w.position_px(1).unwrap()[1] < 300.0, "ball should rest on the floor");
        w.click_trigger(0);
        for _ in 0..30 { w.step(); }
        assert!(w.position_px(1).unwrap()[1] > 320.0, "ball should fall through after refiltering");
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
    fn deleting_shape_body_preserves_art_but_delete_self_hides_it() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='500' p1='100' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='16711680' p9='-1' p10='100' p11='1'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='ACTION'/></sh></t></triggers></levelXML>";
        for action in [5, 6] {
            let level = crate::level::parse(&xml.replace("ACTION", &action.to_string())).unwrap();
            let mut w = World::from_level(&level);
            for _ in 0..15 { w.step(); }
            let pose = w.position_px(0).unwrap();
            w.click_trigger(0);
            assert_eq!(w.shape_visible[0], action == 5);
            assert_eq!(w.shape_frozen_pose[0].unwrap().0, pose);
            let image = crate::render::render_level_at(&level, &w, 300, 300, pose);
            let pixel = image.get_pixel(150, 150);
            assert_eq!(pixel[0] > pixel[1], action == 5);
        }
    }

    #[test]
    fn delete_self_after_deleting_physics_hides_remaining_art() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='500' p1='100' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='16711680' p9='-1' p10='100' p11='1'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='5'/></sh></t><t x='100' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='6'/></sh></t></triggers></levelXML>";
        let level = crate::level::parse(xml).unwrap();
        let mut w = World::from_level(&level);
        w.click_trigger(0);
        assert!(w.shape_visible[0]);
        w.click_trigger(1);
        assert!(!w.shape_visible[0]);
    }

    #[test]
    fn deleting_static_shape_leaves_neighbor_intact() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='500' p1='300' p2='100' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/><sh t='0' p0='700' p1='300' p2='100' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='5'/></sh></t><t x='100' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='6'/></sh></t></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let neighbor = w.static_shape_fixtures[1].unwrap();
        w.click_trigger(0);
        assert!(w.static_shape_fixtures[0].is_none());
        assert!(w.b.body(w.level_body).shapes.contains(&neighbor));
        assert!(w.shape_visible[0]);
        w.click_trigger(1);
        assert!(!w.shape_visible[0]);
        assert!(w.shape_visible[1]);
    }

    #[test]
    fn static_shapes_can_become_dynamic_without_moving_neighbors() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='500' p1='200' p2='80' p3='20' p4='30' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/><sh t='1' p0='700' p1='200' p2='40' p3='40' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/><sh t='0' p0='900' p1='300' p2='100' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='2'/></sh><sh i='1'><a i='2'/></sh></t></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let neighbor = w.static_shape_fixtures[2].unwrap();
        w.click_trigger(0);
        for i in 0..2 {
            let body = w.shape_bodies[i].unwrap();
            assert_ne!(body, w.level_body);
            assert!(w.b.body(body).mass > 0.0);
            assert!((w.position_px(i).unwrap()[1] - 200.0).abs() < 0.01);
        }
        assert!(w.b.body(w.level_body).shapes.contains(&neighbor));
        for _ in 0..10 { w.step(); }
        assert!(w.position_px(0).unwrap()[1] > 200.0);
        assert!(w.position_px(1).unwrap()[1] > 200.0);
    }

    #[test]
    fn unfix_asymmetric_polygon_preserves_rendered_pose() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='3' p0='500' p1='200' p2='100' p3='100' p4='30' p5='t' p6='f' p7='1' p8='16711680' p9='-1' p10='100' p11='1'><v f='t' id='1' n='3' v0='0_0' v1='90_0' v2='0_60'/></sh></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='2'/></sh></t></triggers></levelXML>";
        let level = crate::level::parse(xml).unwrap();
        let mut w = World::from_level(&level);
        let before = crate::render::render_level_at(&level, &w, 300, 300, [500.0, 200.0]);
        w.click_trigger(0);
        assert_ne!(w.shape_render_offset[0], [0.0, 0.0]);
        let after = crate::render::render_level_at(&level, &w, 300, 300, [500.0, 200.0]);
        assert_eq!(before, after, "sprite must not jump when the physics origin changes");
    }

    #[test]
    fn unfixed_static_shape_updates_target_trigger_and_legacy_filter() {
        let xml = "<levelXML><info v='1.80'/><shapes><sh t='1' p0='500' p1='100' p2='40' p3='40' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='2'/></sh></t><t x='500' y='200' w='100' h='50' b='4' t='0' r='1'><sh i='0'/></t></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        w.click_trigger(0);
        let body = w.shape_bodies[0].unwrap();
        let shape = w.b.body(body).shapes[0];
        let filter = w.b.c.shapes[shape].shape.filter;
        assert_eq!((filter.category_bits, filter.group_index), (8, 0));
        for _ in 0..90 { w.step(); }
        assert!(w.events.iter().any(|e| e.trigger == 1), "new body must activate target-only trigger");
    }

    #[test]
    fn static_collision_change_does_not_affect_neighbor() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='500' p1='300' p2='100' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/><sh t='0' p0='700' p1='300' p2='100' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/><sh t='1' p0='500' p1='200' p2='30' p3='30' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/><sh t='1' p0='700' p1='200' p2='30' p3='30' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes><triggers><t x='0' y='0' w='50' h='50' b='6' t='1' r='1'><sh i='0'><a i='7' p0='3'/></sh></t></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        for _ in 0..60 { w.step(); }
        w.click_trigger(0);
        for _ in 0..40 { w.step(); }
        assert!(w.position_px(2).unwrap()[1] > 320.0);
        assert!(w.position_px(3).unwrap()[1] < 300.0);
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
    fn victory_trigger_completes_on_activation() {
        let xml = "<levelXML><info v='1.99'/><triggers><t x='300' y='100' w='100' h='100' a='0' b='6' t='3' r='1' sd='f'/></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        assert!(!w.level_complete);
        w.click_trigger(0);
        assert!(w.level_complete);
        assert_eq!(w.take_sounds().iter().filter(|s| s.0 == "Victory").count(), 1);
        w.click_trigger(0);
        assert!(w.take_sounds().is_empty());
    }

    #[test]
    fn sound_trigger_preserves_original_lookup_and_transform() {
        let xml = "<levelXML><info v='1.99'/><triggers><t x='400' y='200' w='50' h='50' b='6' t='2' r='1' s='0' l='1' v='0.4' p='-0.5'/></triggers></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        w.click_trigger(0);
        let sounds = w.take_trigger_sounds();
        assert_eq!(sounds.len(), 1);
        assert_eq!(sounds[0].name, "Char1Foot1");
        assert!(sounds[0].global);
        assert_eq!((sounds[0].volume, sounds[0].pan, sounds[0].position), (0.4, -0.5, [400.0, 200.0]));
    }

    #[test]
    fn collecting_last_token_completes_level() {
        let xml = "<levelXML><info v='1.99' x='1000' y='200'/><specials><sp t='31' p0='1000' p1='200' p2='1'/><sp t='31' p0='1000' p1='260' p2='1'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        assert!(w.spawn_character(1, [1000.0, 200.0]));
        let mut sounds = Vec::new();
        for _ in 0..60 {
            w.step();
            sounds.extend(w.take_sounds());
        }
        assert_eq!(w.tokens_found, 2);
        assert!(w.level_complete);
        assert_eq!(sounds.iter().filter(|s| s.0 == "Victory").count(), 1);
    }

    #[test]
    fn token_requires_a_living_character_owner() {
        let xml = "<levelXML><info v='1.99' x='1000' y='200'/><specials><sp t='31' p0='1000' p1='200' p2='1'/></specials></levelXML>";
        let level = crate::level::parse(xml).unwrap();
        let mut w = World::from_level(&level);
        assert!(w.spawn_character(1, [1000.0, 200.0]));
        let chest = w.character.as_ref().unwrap().bodies[&crate::character::Part::Chest];
        assert!(w.living_token_owner(chest));
        w.character.as_mut().unwrap().dead = true;
        assert!(!w.living_token_owner(chest));
        for _ in 0..10 { w.step(); }
        assert_eq!(w.tokens_found, 0, "dead rider must not collect token");
        assert!(!w.level_complete);
        let body = w.b.create_body(&BodyDef::default());
        assert!(!w.living_token_owner(body), "unrelated bodies are not characters");
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
    fn homing_mine_sound_follows_target_and_fuse() {
        let lvl = crate::level::parse("<levelXML><info v='1.99'/><specials><sp t='25' p0='1000' p1='500' p2='5' p3='2'/></specials></levelXML>").unwrap();
        let mut w = World::from_level(&lvl);
        let target = w.b.create_body(&BodyDef { position: Vec2::new(1000.0 / PS + 2.0, 500.0 / PS), ..Default::default() });
        let mut shape = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 0.1 });
        shape.density = 1.0;
        w.b.create_shape(target, &shape);
        w.b.set_mass_from_shapes(target);
        w.mines[0].target = Some(target);
        w.run_mines();
        w.update_loops();
        assert!(w.take_sounds().iter().any(|s| s.0 == "HomingMineFind"), "done={} hovering={} target={:?} mine={:?} target_pos={:?}", w.mines[0].done, w.mines[0].hovering, w.mines[0].target, w.b.body(w.mines[0].body).sweep.c, w.b.body(target).sweep.c);
        assert!(w.loops.iter().any(|s| s.0 == "MineHover"));
        w.mines[0].counting_down = true;
        w.mines[0].counter = 30.0;
        w.update_loops();
        assert!(w.loops.iter().any(|s| s.0 == "HomingMineBeep"));
        w.mines[0].target = None;
        w.run_mines();
        w.update_loops();
        assert!(w.take_sounds().iter().any(|s| s.0 == "HomingMineLose"));
        assert!(!w.loops.iter().any(|s| s.0 == "MineHover"));
        assert!(w.loops.iter().any(|s| s.0 == "HomingMineBeep"));
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
        let mut max_blood = 0;
        let mut sounds = Vec::new();
        for _ in 0..120 {
            w.b.body_mut(b).linear_velocity = Vec2::default();
            w.step();
            max_blood = max_blood.max(w.blood.live());
            sounds.extend(w.take_sounds());
        }
        assert_eq!(w.arrows.len(), 1, "exactly one harpoon expected");
        assert!(w.arrows[0].armed && !w.arrows[0].done, "harpoon should be pinned and intact");
        assert_eq!(max_blood, 50, "impaling flesh bursts 50 blood particles");
        assert!(sounds.iter().any(|s| s.0.starts_with("HarpoonFlesh")), "{sounds:?}");
        // the shaft keeps one `HarpoonBloodMC` stamp (frame 0..4) per impaled body; with blood switched off there is none
        let body = w.arrows[0].body;
        let stamps = w.harpoon_blood(body);
        assert_eq!(stamps.len(), 1);
        assert!(stamps[0].0 >= 0.0 && stamps[0].0 <= 4.0);
    }

    #[test]
    fn harpoon_leaves_no_blood_stamp_when_blood_is_off() {
        let lvl = crate::level::parse("<levelXML><info v='1.99'/><shapes/><specials><sp t='15' p0='1000' p1='500' p2='0' p3='f' p4='f' p5='0' p6='f' p7='f'/></specials></levelXML>").unwrap();
        let mut w = World::from_level(&lvl);
        w.blood.off = true;
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
        assert!(w.arrows[0].armed);
        assert!(w.harpoon_blood(w.arrows[0].body).is_empty());
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
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='725' p2='4000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='17' p0='1000' p1='300' p2='0' p3='1' p4='f' p5='f' p6='f' p7='t'/></specials></levelXML>";
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

    /// Boombox against the original `Boombox` class (oracle `analysis/golden-boombox-break.js`): floor at y 500, boombox at (1000,430) dropped at 100, 40 and 8 m/s:
    /// shatters into two pieces / cracks and keeps its body / is not damaged at all.
    #[test]
    fn boombox_break_matches_the_original() {
        let run = |vy: f32| {
            let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='22' p0='1000' p1='430' p2='0' p3='f' p4='t'/></specials></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let b = w.special_bodies[0].unwrap();
            w.b.body_mut(b).linear_velocity = Vec2::new(0.0, f64::from(vy));
            let mut frames = Vec::new();
            for _ in 0..40 {
                w.step();
                let list: Vec<BodyId> = if w.boombox_pieces().count() > 0 { w.boombox_pieces().map(|p| p.0).collect() } else { vec![b] };
                let mut v: Vec<[f64; 3]> = list.into_iter().map(|id| { let [x, y] = w.body_position_px(id).unwrap(); [f64::from(x), f64::from(y), w.body_angle(id) as f64] }).collect();
                v.sort_by(|a, b| a[0].total_cmp(&b[0]));
                frames.push((v, w.boombox_frame(b)));
            }
            frames
        };
        let close = |got: &[f64; 3], want: [f64; 3], what: &str| assert!((got[0] - want[0]).abs() < 0.01 && (got[1] - want[1]).abs() < 0.01 && (got[2] - want[2]).abs() < 0.002, "{what}: {got:?} vs {want:?}");
        let shatter = run(100.0);
        assert_eq!(shatter[0].0.len(), 2);
        for (g, w) in shatter[0].0.iter().zip([[986.418, 462.897, -0.001], [1013.568, 462.88, -0.001]]) { close(g, w, "shatter f0"); }
        for (g, w) in shatter[39].0.iter().zip([[987.281, 190.181, -0.031], [1014.431, 190.164, -0.031]]) { close(g, w, "shatter f39"); }
        let crack = run(40.0);
        assert_eq!(crack[39].0.len(), 1, "a crack keeps the body");
        assert!(crack[39].1 >= 2, "and shows a damage frame");
        close(&crack[0].0[0], [1000.004, 473.576, 0.0], "crack f0");
        close(&crack[39].0[0], [1000.216, 476.225, 0.001], "crack f39");
        let soft = run(8.0);
        assert_eq!(soft[39].1, 1, "a soft hit leaves the boombox intact");
        close(&soft[39].0[0], [1000.011, 476.317, 0.0], "soft f39");
    }

    /// Glass against the original `Glass` class (oracle `analysis/golden-glass-break.js`): 100×20 pane at (1000,430) dropped at 40 m/s with strength 10 (shatters)
    /// and at 8 m/s with strength 10000 (stays whole).
    #[test]
    fn glass_break_matches_the_original() {
        let run = |strength: u32, vy: f64| {
            let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='18' p0='1000' p1='430' p2='100' p3='20' p4='0' p5='f' p6='{strength}' p7='f'/></specials></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let b = w.special_bodies[0].unwrap();
            w.b.body_mut(b).linear_velocity = Vec2::new(0.0, vy);
            let mut frames = Vec::new();
            for _ in 0..40 {
                w.step();
                let list: Vec<BodyId> = if w.glass_pieces().count() > 0 { w.glass_pieces().map(|p| p.0).collect() } else { vec![b] };
                let mut v: Vec<[f64; 3]> = list.into_iter().map(|id| { let [x, y] = w.body_position_px(id).unwrap(); [f64::from(x), f64::from(y), w.body_angle(id) as f64] }).collect();
                v.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
                frames.push(v);
            }
            frames
        };
        let close = |got: &[f64; 3], want: [f64; 3], tol: f64, what: &str| assert!((got[0] - want[0]).abs() < tol && (got[1] - want[1]).abs() < tol && (got[2] - want[2]).abs() < 0.002, "{what}: {got:?} vs {want:?}");
        let broke = run(10, 40.0);
        assert_eq!(broke[0].len(), 4, "impact on a corner: two intact sides, four triangles");
        for g in &broke[0] { close(g, [1000.0, 477.972, 0.0], 0.01, "break frame"); }
        // frame 24 (first landing), 26 and 36 of the oracle, pieces sorted by x then y
        for (frame, want) in [
            (24, [[1000.0, 484.639, 0.0], [1000.0, 484.639, 0.0], [1000.004, 480.365, 0.001], [1000.123, 487.414, -0.14]]),
            (36, [[998.183, 486.96, -0.113], [999.477, 480.78, -0.001], [999.785, 483.809, -0.079], [1000.516, 473.557, 0.098]]),
        ] {
            for (g, w) in broke[frame].iter().zip(want) { close(g, w, 0.02, &format!("frame {frame}")); }
        }
        let whole = run(10000, 8.0);
        assert_eq!(whole[39].len(), 1);
        close(&whole[39][0], [1000.0, 480.318, 0.0], 0.01, "whole pane");
    }

    /// SpringBox launches a ball like the original (oracle `analysis/golden-springbox.js`): ball (r 20) dropped from y 300 onto a box at (1000,430) with a 0.5 s
    /// delay; the pad waits 15 frames, drives out (motor 10, 32 px) and comes back, the ball flies up.
    #[test]
    fn spring_box_launches_like_the_original() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='1' p0='1000' p1='300' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes><specials><sp t='5' p0='1000' p1='430' p2='0' p3='0.5'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let pad = w.special_bodies[0].unwrap();
        let ball = (0..w.b.c.bodies.len()).find(|&b| b != pad && !w.body_is_fixed(b) && w.b.c.bodies[b].alive && w.b.body(b).xf.position.y * PS < 400.0).unwrap();
        let mut got = Vec::new();
        for _ in 0..120 {
            w.step();
            got.push((w.body_position_px(pad).unwrap()[1], w.body_position_px(ball).unwrap()[1]));
        }
        for (k, pad_y, ball_y) in [(16, 430.0, 390.139), (30, 430.0, 390.365), (35, 399.771, 309.101), (40, 410.187, 222.296), (50, 431.021, 100.768), (60, 430.0, 48.685), (80, 430.0, 152.851), (119, 410.187, 222.303)] {
            assert!((f64::from(got[k].0) - pad_y).abs() < 0.05 && (f64::from(got[k].1) - ball_y).abs() < 0.05, "frame {k}: {:?} vs ({pad_y}, {ball_y})", got[k]);
        }
    }

    /// Paddle swings a ball like the original (oracle `analysis/golden-paddle.js`): ball (r 20) at (900,360) falls on a paddle at (1000,430), delay 0.5 s, angle 90°,
    /// speed 5; the ball's landing arms the pad, it swings to ~1.42 rad (flipping the motor at the target), returns and locks; the ball is launched.
    #[test]
    fn paddle_swings_like_the_original() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='1' p0='900' p1='360' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes><specials><sp t='35' p0='1000' p1='430' p2='0' p3='0.5' p4='f' p5='90' p6='5'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let pad = w.special_bodies[0].unwrap();
        let ball = (0..w.b.c.bodies.len()).find(|&b| b != pad && !w.body_is_fixed(b) && w.b.c.bodies[b].alive && w.b.body(b).xf.position.y * PS < 400.0).unwrap();
        let mut got = Vec::new();
        for _ in 0..120 {
            w.step();
            let p = w.body_position_px(pad).unwrap();
            let q = w.body_position_px(ball).unwrap();
            got.push([f64::from(p[0]), f64::from(p[1]), f64::from(q[0]), f64::from(q[1]), w.body_angle(pad) as f64]);
        }
        for (k, want) in [
            (12, [1000.000, 430.015, 900.000, 390.359, 0.000]),
            (24, [1000.000, 430.001, 900.000, 390.315, -0.000]),
            (30, [1027.577, 337.863, 924.218, 214.739, 0.648]),
            (36, [1105.227, 281.018, 992.749, 24.368, 1.296]),
            (42, [1123.416, 276.457, 1061.281, -141.004, 1.420]),
            (54, [1025.301, 341.180, 1198.343, -396.748, 0.621]),
            (60, [1002.594, 396.686, 1266.874, -487.119, 0.222]),
            (66, [1000.000, 430.013, 1335.405, -552.491, -0.000]),
            (78, [1000.000, 430.002, 1472.468, -608.235, 0.000]),
        ] {
            for i in 0..5 {
                assert!((got[k][i] - want[i]).abs() < 0.5, "frame {k} col {i}: {:?} vs {want:?}", got[k]);
            }
        }
    }

    /// Fan against the original `Fan` class (oracle `analysis/golden-fan.js`): a fan at (1000,480) pointing up pushes two balls (r 20, density 3 and 6) placed at
    /// (1000,400) and (1040,300); the light one rises and falls again, the heavy one only settles on the fan.
    #[test]
    fn fan_pushes_like_the_original() {
        let xml = "<levelXML><info v='1.99'/><shapes>\
            <sh t='1' p0='1000' p1='400' p2='40' p3='40' p4='0' p5='f' p6='f' p7='3' p8='0' p9='-1' p10='100' p11='1' p12='0'/>\
            <sh t='1' p0='1040' p1='300' p2='40' p3='40' p4='0' p5='f' p6='f' p7='6' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
            <specials><sp t='8' p0='1000' p1='480' p2='0'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let mut got = Vec::new();
        for _ in 0..120 {
            w.step();
            got.push([w.position_px(0).unwrap()[1], w.position_px(1).unwrap()[1]]);
        }
        for (k, want) in [
            (10, [397.263, 321.548]),
            (20, [355.342, 362.737]),
            (40, [165.761, 410.274]),
            (60, [-37.227, 410.274]),
            (90, [118.512, 410.274]),
            (119, [383.338, 410.274]),
        ] {
            for i in 0..2 {
                assert!((f64::from(got[k][i]) - want[i]).abs() < 0.2, "frame {k} ball {i}: {:?} vs {want:?}", got[k]);
            }
        }
    }

    /// Mine against the original `Mine` class (oracle `analysis/golden-mine.js`): a ball dropped on a mine at (1000,489) sets it off; two more balls beside it
    /// get the radial impulse (`10·(1 − d/2)`).
    #[test]
    fn mine_explodes_like_the_original() {
        let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}\
            <sh t='1' p0='1000' p1='420' p2='40' p3='40' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>\
            <sh t='1' p0='1060' p1='460' p2='30' p3='30' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>\
            <sh t='1' p0='940' p1='470' p2='24' p3='24' p4='0' p5='f' p6='f' p7='2' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
            <specials><sp t='2' p0='1000' p1='489' p2='0'/></specials></levelXML>");
        let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
        let mut got = Vec::new();
        for _ in 0..90 {
            w.step();
            got.push((1..4).flat_map(|i| { let p = w.position_px(i).unwrap_or([f32::NAN; 2]); [f64::from(p[0]), f64::from(p[1])] }).collect::<Vec<_>>());
        }
        assert!(w.take_sounds().iter().any(|s| s.0 == "MineExplosion"));
        for (k, want) in [
            (5, [1000.000, 434.583, 1060.000, 474.583, 940.000, 478.323]),
            (12, [1000.014, 470.805, 1060.000, 475.323, 940.000, 478.315]),
            (16, [1002.304, 283.449, 1286.944, 429.689, 758.291, 452.004]),
            (20, [1004.594, 107.203, 1513.889, 395.166, 576.583, 436.804]),
        ] {
            for i in 0..6 {
                assert!((got[k][i] - want[i]).abs() < 0.5, "frame {k} col {i}: {:?} vs {want:?}", got[k]);
            }
        }
    }

    /// Trash can against the original `TrashCan` (oracle `analysis/golden-trash.js`): can with trash at (1000,420) dropped at 40 and at 12 m/s.
    #[test]
    fn trash_can_break_matches_the_original() {
        let run = |vy: f64| {
            let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='26' p0='1000' p1='420' p2='0' p3='f' p4='t' p5='t'/></specials></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let can = w.special_bodies[0].unwrap();
            w.b.body_mut(can).linear_velocity = Vec2::new(0.0, vy);
            let mut frames = Vec::new();
            for _ in 0..40 {
                w.step();
                let can = w.special_bodies[0].unwrap();
                let mut ids: Vec<BodyId> = std::iter::once(can).chain(w.loose.iter().copied()).filter(|&b| w.b.c.bodies[b].alive).collect();
                ids.sort_unstable();
                ids.dedup();
                let mut v: Vec<[f64; 3]> = ids.into_iter().map(|b| { let [x, y] = w.body_position_px(b).unwrap(); [f64::from(x), f64::from(y), w.body_angle(b) as f64] }).collect();
                v.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
                frames.push(v);
            }
            frames
        };
        let a = run(40.0);
        let b = run(12.0);
        // crush (first Result 14.9 > 12): the can is replaced by a crushed box and ten trash bodies; spill (4.7 > 3.5, at frame 1) keeps the hollow can
        for (what, got, want) in [
            ("crush f1", &a[1], [[982.314, 406.123, 0.003], [985.116, 421.439, -0.175], [990.510, 422.891, -0.121], [991.847, 408.196, -0.022], [999.810, 427.176, -0.008], [999.861, 450.063, -0.127], [999.869, 401.859, 0.117], [1005.611, 422.564, -0.000], [1006.165, 408.947, 0.008], [1010.280, 426.878, -0.004], [1016.990, 412.428, 0.118]]),
            ("crush f39", &a[39], [[955.399, 446.179, 1.674], [964.374, 454.630, -1.792], [987.093, 443.611, 0.009], [987.371, 426.580, 0.057], [997.663, 420.135, 0.200], [997.923, 449.820, 0.017], [999.224, 472.421, 0.000], [1002.557, 428.428, 0.042], [1003.179, 438.585, 0.039], [1009.123, 450.020, 0.315], [1020.654, 449.855, 0.059]]),
            ("spill f39", &b[39], [[984.026, 420.025, -0.468], [985.897, 436.299, -0.030], [991.947, 437.211, -0.006], [995.531, 420.095, 0.363], [1001.366, 412.779, 0.215], [1001.573, 443.474, 0.002], [1001.732, 454.499, 0.001], [1007.997, 432.093, 0.071], [1010.814, 422.088, 0.071], [1012.397, 443.684, 0.228], [1019.084, 436.531, 0.164]]),
        ] {
            let want: Vec<[f64; 3]> = want.to_vec();
            assert_eq!(got.len(), want.len(), "{what}: body count");
            for (g, w) in got.iter().zip(&want) {
                assert!((g[0] - w[0]).abs() < 0.02 && (g[1] - w[1]).abs() < 0.02 && (g[2] - w[2]).abs() < 0.005, "{what}: {g:?} vs {w:?}");
            }
        }
        assert_eq!(b[0].len(), 1, "the spill is processed on the next frame");
    }

    /// Spikes impale against the original `Spikes` class (oracle `analysis/golden-spikes.js`): spikes (6, fixed) at (1000,450); a flesh ball (material 2) dropped from
    /// (1000,300) is pinned in the sensor by the prismatic joint (motor force 30) at y 434.325, a plain ball (material 0) from (1020,200) only rolls over the base.
    #[test]
    fn spikes_impale_like_the_original() {
        let xml = "<levelXML><info v='1.99'/><shapes>{FLOOR}\
            <sh t='1' p0='1000' p1='300' p2='30' p3='30' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/>\
            <sh t='1' p0='1020' p1='200' p2='24' p3='24' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
            <specials><sp t='6' p0='1000' p1='450' p2='0' p3='t' p4='6' p5='f'/></specials></levelXML>".replace("{FLOOR}", FLOOR);
        let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
        let flesh = w.shape_bodies[1].unwrap();
        let shape = w.b.body(flesh).shapes[0];
        w.b.shape_mut(shape).material = 2;
        let plain = w.shape_bodies[2].unwrap();
        let mut got = Vec::new();
        for _ in 0..120 {
            w.step();
            let a = w.body_position_px(flesh).unwrap();
            let b = w.body_position_px(plain).unwrap();
            got.push([f64::from(a[0]), f64::from(a[1]), f64::from(b[0]), f64::from(b[1])]);
        }
        for (k, want) in [
            (10, [1000.000, 345.833, 1020.000, 245.833]),
            (18, [1000.000, 431.944, 1020.000, 331.944]),
            (20, [1000.000, 434.320, 1020.000, 360.417]),
            (25, [1000.000, 434.325, 1024.258, 433.800]),
            (40, [1000.000, 434.325, 1117.182, 478.344]),
            (60, [1000.000, 434.325, 1251.531, 478.344]),
            (119, [1000.000, 434.325, 1647.861, 478.344]),
        ] {
            for i in 0..4 {
                assert!((got[k][i] - want[i]).abs() < 0.2, "frame {k} col {i}: {:?} vs {want:?}", got[k]);
            }
        }
    }

    /// Stab-mode glass against the original (oracle `analysis/golden-glass-stab.js`): a 100×20 pane (strength 10, stabbing) dropped at 40 m/s shatters into four
    /// stab shards; a heavy flesh ball (r 10, density 6, material 2) dropped from y 250 then breaks one of them.
    #[test]
    fn stab_glass_shards_match_the_original() {
        let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}\
            <sh t='1' p0='1000' p1='250' p2='20' p3='20' p4='0' p5='f' p6='f' p7='6' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
            <specials><sp t='18' p0='1000' p1='430' p2='100' p3='20' p4='0' p5='f' p6='10' p7='t'/></specials></levelXML>");
        let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
        let pane = w.special_bodies[0].unwrap();
        w.b.body_mut(pane).linear_velocity = Vec2::new(0.0, 40.0);
        let ball = w.shape_bodies[1].unwrap();
        let shape = w.b.body(ball).shapes[0];
        w.b.shape_mut(shape).material = 2;
        let mut frames = Vec::new();
        for _ in 0..100 {
            w.step();
            let mut v: Vec<[f64; 3]> = (0..w.b.c.bodies.len()).filter(|&b| w.b.c.bodies[b].alive && !w.b.body(b).is_static).map(|b| { let [x, y] = w.body_position_px(b).unwrap(); [f64::from(x), f64::from(y), w.body_angle(b) as f64] }).collect();
            v.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
            frames.push(v);
        }
        for (k, want) in [
            (1, vec![[1000.000, 470.264, 0.000], [1000.000, 470.264, 0.000], [1000.000, 470.264, 0.000], [1000.000, 470.264, 0.000], [1000.000, 252.083, 0.000]]),
            (20, vec![[1000.000, 455.750, 0.000], [1000.000, 455.749, 0.000], [1000.000, 455.750, 0.000], [1000.000, 455.750, 0.000], [1000.000, 410.417, 0.000]]),
            (26, vec![[999.556, 490.363, -0.197], [999.644, 481.907, -0.083], [1002.108, 493.472, -0.264], [1002.911, 473.545, 0.405]]),
            (40, vec![[998.833, 490.635, -0.197], [999.590, 476.722, 0.043], [1002.889, 487.851, -0.149], [1009.629, 467.162, 1.139]]),
            (99, vec![[998.479, 490.435, -0.197], [999.446, 476.450, 0.049], [1001.384, 466.957, 0.306], [1003.273, 487.493, -0.141]]),
        ] {
            assert_eq!(frames[k].len(), want.len(), "frame {k}: body count");
            let mut got = frames[k].clone();
            let mut want = want;
            let key = |r: &[f64; 3]| ((r[1] * 20.0).round() as i64, (r[0] * 20.0).round() as i64);
            got.sort_by_key(key);
            want.sort_by_key(key);
            for (g, w) in got.iter().zip(&want) {
                assert!((g[0] - w[0]).abs() < 0.05 && (g[1] - w[1]).abs() < 0.05 && (g[2] - w[2]).abs() < 0.01, "frame {k}: {g:?} vs {w:?}");
            }
        }
    }

    /// Stab pinning against the original (oracle `analysis/golden-glass-stab-pin.js`: same scene as above, but the flesh ball has `categoryBits` 4 so the stab sensor
    /// (mask 4) sees it, and the stub registers the shard's add/remove listeners): the ball pins on a shard with one prismatic joint, visible after frames 27..29 in the original.
    #[test]
    fn stab_glass_pins_flesh_like_the_original() {
        let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}\
            <sh t='1' p0='1000' p1='250' p2='20' p3='20' p4='0' p5='f' p6='f' p7='6' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes>\
            <specials><sp t='18' p0='1000' p1='430' p2='100' p3='20' p4='0' p5='f' p6='10' p7='t'/></specials></levelXML>");
        let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
        let pane = w.special_bodies[0].unwrap();
        w.b.body_mut(pane).linear_velocity = Vec2::new(0.0, 40.0);
        let ball = w.shape_bodies[1].unwrap();
        let shape = w.b.body(ball).shapes[0];
        w.b.shape_mut(shape).material = 2;
        w.b.shape_mut(shape).filter.category_bits = 4;
        let mut frames = Vec::new();
        let mut pinned_frames = Vec::new();
        for k in 0..100 {
            w.step();
            let n: usize = w.glasses.iter().flat_map(|g| g.shards.iter()).map(|s| s.pinned.len()).sum();
            if n > 0 {
                pinned_frames.push(k);
            }
            let mut v: Vec<[f64; 3]> = (0..w.b.c.bodies.len()).filter(|&b| w.b.c.bodies[b].alive && !w.b.body(b).is_static).map(|b| { let [x, y] = w.body_position_px(b).unwrap(); [f64::from(x), f64::from(y), f64::from(w.body_angle(b))] }).collect();
            v.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
            frames.push(v);
        }
        assert_eq!(pinned_frames, vec![26, 27, 28], "pin window (the joint is made right after the contact step, the original makes it at the start of the next frame: same physics)");
        for (k, want) in [
            (27, vec![[999.684, 480.906, -0.069], [999.983, 490.382, -0.197], [1002.336, 492.356, -0.249], [1002.609, 472.376, 0.383]]),
            (28, vec![[999.477, 479.720, -0.035], [1000.233, 490.361, -0.197], [1002.022, 490.973, -0.214], [1002.365, 471.042, 0.381]]),
            (99, vec![[999.313, 490.406, -0.197], [999.397, 476.258, 0.051], [1002.338, 487.046, -0.135], [1003.697, 467.086, 0.463]]),
        ] {
            assert_eq!(frames[k].len(), want.len(), "frame {k}: body count");
            for (g, w) in frames[k].iter().zip(&want) {
                assert!((g[0] - w[0]).abs() < 0.1 && (g[1] - w[1]).abs() < 0.1 && (g[2] - w[2]).abs() < 0.02, "frame {k}: {g:?} vs {w:?}");
            }
        }
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

    #[test]
    fn glass_impact_sound_and_listener_lifetime() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='18' p0='900' p1='380' p2='100' p3='20' p4='0' p5='f' p6='10000' p7='t'/><sp t='18' p0='1200' p1='380' p2='100' p3='20' p4='0' p5='f' p6='10' p7='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (intact, broken) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(intact).linear_velocity = Vec2::new(0.0, 10.0);
        w.b.body_mut(broken).linear_velocity = Vec2::new(0.0, 60.0);
        let mut sounds = Vec::new();
        for _ in 0..30 {
            w.step();
            sounds.extend(w.take_sounds());
        }
        assert!(w.b.body(intact).alive);
        assert!(!w.b.body(broken).alive);
        assert!(sounds.iter().any(|s| s.0.starts_with("GlassImpact")));
        assert!(w.hit_sounds.iter().any(|h| h.body == intact && h.name == "GlassImpact"));
        assert!(w.hit_sounds.iter().all(|h| h.body != broken));
    }

    /// Static check: every literal sound name handed to `play("Name", n, …)` (n variants → Name1..Name_n) exists in the audiosprite table.
    #[test]
    fn literal_play_calls_name_existing_sounds() {
        let table: serde_json::Value = serde_json::from_str(include_str!("../data/audiosprite.json")).unwrap();
        let mut missing = Vec::new();
        for src in [include_str!("physics.rs"), include_str!("vehicle.rs"), include_str!("character.rs")] {
            let mut rest = src;
            while let Some(i) = rest.find("play(\"") {
                rest = &rest[i + 6..];
                let Some(end) = rest.find('"') else { break };
                let (name, tail) = (&rest[..end], &rest[end + 1..]);
                let variants: u32 = tail.trim_start_matches(',').trim_start().split(|c: char| !c.is_ascii_digit()).next().and_then(|d| d.parse().ok()).unwrap_or(1);
                if name.contains('{') || name.is_empty() || name == "Name" {
                    continue;
                }
                let names: Vec<String> = if variants > 1 { (1..=variants).map(|v| format!("{name}{v}")).collect() } else { vec![name.to_string()] };
                missing.extend(names.into_iter().filter(|n| table.get(n.as_str()).is_none()));
            }
        }
        assert!(missing.is_empty(), "sounds missing from the audiosprite: {missing:?}");
    }

    /// Both pieces of a toilet that breaks in one step share one ToiletSmash handle (a single sound), and every requested sound exists in the sprite.
    #[test]
    fn toilet_smash_sounds_exist_and_play_once() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='24' p0='1300' p1='360' p2='0' p3='f' p4='f' p5='t'/><sp t='18' p0='900' p1='380' p2='100' p3='20' p4='0' p5='f' p6='10' p7='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (toilet, pane) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(toilet).linear_velocity = Vec2::new(0.0, 60.0);
        w.b.body_mut(pane).linear_velocity = Vec2::new(0.0, 60.0);
        let mut sounds = Vec::new();
        for _ in 0..40 {
            w.step();
            sounds.extend(w.take_sounds());
        }
        let table: serde_json::Value = serde_json::from_str(include_str!("../data/audiosprite.json")).unwrap();
        for (name, _) in &sounds {
            assert!(table.get(name.as_str()).is_some(), "{name} is not in the audiosprite");
        }
        assert!(sounds.iter().any(|s| s.0.starts_with("ToiletSmash")));
        assert!(sounds.iter().any(|s| s.0.starts_with("Glass")));
    }

    /// A toilet hit hard on the back loses its back (two pieces); hit on the base/bowl it loses the base (two pieces, with the bowl split); both -> body gone.
    #[test]
    fn toilet_breaks_off_pieces() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='24' p0='900' p1='380' p2='0' p3='f' p4='f' p5='t'/><sp t='24' p0='1300' p1='360' p2='0' p3='f' p4='f' p5='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (soft, hard) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(hard).linear_velocity = Vec2::new(0.0, 60.0);
        let _ = soft;
        let mut water = 0;
        for _ in 0..40 {
            w.step();
            water = water.max(w.blood.sprites.iter().filter(|s| s.kind == crate::blood::ShardKind::Food(crate::blood::WATER)).count());
        }
        assert!(water > 20, "a broken toilet sprays water ({water})");
        assert_eq!(w.toilet_parts(Some(soft)), (true, true));
        assert!(w.toilet_pieces().count() >= 2, "a hard drop breaks something off");
        assert!(w.toilet_pieces().all(|(b, _, _)| w.b.c.bodies[b].alive));
    }

    /// Toilet break against the original `Toilet` class (oracle `analysis/golden-toilet-break.js`): floor at y 500, toilet at (1000,400) dropped at 40 m/s.
    #[test]
    fn toilet_break_matches_the_original() {
        for (reverse, first, last) in [
            ("f", [[993.124_f64, 456.499, -0.033]; 2], [[990.484, 459.208, 1.242], [912.279, 462.305, -0.978]]),
            ("t", [[1005.63, 456.509, -0.003]; 2], [[953.281, 459.808, -1.199], [1014.899, 451.497, 0.408]]),
        ] {
            let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='24' p0='1000' p1='400' p2='0' p3='{reverse}' p4='f'/></specials></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let toilet = w.special_bodies[0].unwrap();
            w.b.body_mut(toilet).linear_velocity = Vec2::new(0.0, 40.0);
            let mut frames = Vec::new();
            for _ in 0..40 {
                w.step();
                let mut v: Vec<[f64; 3]> = w.toilet_pieces().map(|p| { let [x, y] = w.body_position_px(p.0).unwrap(); [f64::from(x), f64::from(y), w.body_angle(p.0) as f64] }).collect();
                v.sort_by(|a, b| a[0].total_cmp(&b[0]));
                frames.push(v);
            }
            for (got, mut want, what) in [(&frames[0], first.to_vec(), "break frame"), (&frames[39], last.to_vec(), "frame 39")] {
                want.sort_by(|a, b| a[0].total_cmp(&b[0]));
                assert_eq!(got.len(), want.len(), "{reverse} {what}: piece count");
                for (g, w) in got.iter().zip(&want) {
                    assert!((g[0] - w[0]).abs() < 0.01 && (g[1] - w[1]).abs() < 0.01 && (g[2] - w[2]).abs() < 0.001, "{reverse} {what}: {g:?} vs {w:?}");
                }
            }
        }
    }

    /// Log against the original `Log` class (oracle `analysis/golden-log-break.js`): floor at y 500, logs at (1000,380) dropped at 40 m/s. The 200×72 log
    /// splits on the first contact (halves inherit its velocity); the lighter 150×50 one stays whole.
    #[test]
    fn log_break_matches_the_original() {
        let run = |w: f64, h: f64| {
            let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='4' p0='1000' p1='380' p2='{w}' p3='{h}' p4='0' p5='f' p6='f'/></specials></levelXML>");
            let mut world = World::from_level(&crate::level::parse(&xml).unwrap());
            let log = world.special_bodies[0].unwrap();
            world.b.body_mut(log).linear_velocity = Vec2::new(0.0, 40.0);
            let mut frames = Vec::new();
            for _ in 0..40 {
                world.step();
                let list: Vec<BodyId> = if world.log_halves().count() > 0 { world.log_halves().map(|p| p.0).collect() } else { vec![log] };
                frames.push(list.into_iter().map(|b| { let [x, y] = world.body_position_px(b).unwrap(); [f64::from(x), f64::from(y), world.body_angle(b) as f64] }).collect::<Vec<_>>());
            }
            frames
        };
        let broke = run(200.0, 72.0);
        assert_eq!(broke[0].len(), 2);
        for (frame, want) in [(0, [1000.0, 453.494, 0.0]), (39, [1000.018, 339.688, -0.001])] {
            for g in &broke[frame] {
                assert!((g[0] - want[0]).abs() < 0.1 && (g[2] - want[2]).abs() < 0.002, "frame {frame}: {g:?}");
            }
        }
        assert!((broke[0][0][1] - 453.494).abs() < 0.01 && (broke[39][0][1] - 339.688).abs() < 0.01 && (broke[39][1][1] - 339.688).abs() < 0.01, "{:?} {:?}", broke[0], broke[39]);
        let whole = run(150.0, 50.0);
        assert_eq!(whole[0].len(), 1, "a lighter log does not split");
        assert!((whole[39][0][1] - 320.399).abs() < 0.01 && (whole[39][0][0] - 1000.023).abs() < 0.1, "{:?}", whole[39]);
    }

    /// An immovable log (a shape of the level body in the original) breaks like a free one: the heavy hitter above 2250 splits it, the halves start at rest.
    #[test]
    fn immovable_log_splits_too() {
        let xml = "<levelXML><info v='1.99'/><shapes/><specials><sp t='4' p0='1000' p1='400' p2='36' p3='100' p4='0' p5='t' p6='f'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let log = w.special_bodies[0].unwrap();
        assert!(w.body_is_fixed(log));
        let hitter = w.b.create_body(&BodyDef { position: Vec2::new(1000.0 / PS, 250.0 / PS), ..Default::default() });
        let mut sd = ShapeDef::oriented_box(0.4, 0.4, Vec2::default(), 0.0);
        sd.density = 5000.0;
        sd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
        w.b.create_shape(hitter, &sd);
        w.b.set_mass_from_shapes(hitter);
        w.b.body_mut(hitter).linear_velocity = Vec2::new(0.0, 20.0);
        for _ in 0..30 {
            w.step();
        }
        assert!(!w.b.c.bodies[log].alive);
        assert_eq!(w.log_halves().count(), 2);
    }

    /// A log hit above an impulse of 2250 splits into two halves; a gentle one does not.
    #[test]
    fn log_splits_into_halves() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='4' p0='900' p1='380' p2='36' p3='100' p4='0' p5='f' p6='f' p7='f'/><sp t='4' p0='1300' p1='300' p2='36' p3='100' p4='0' p5='f' p6='f' p7='f'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (soft, hard) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(hard).linear_velocity = Vec2::new(0.0, 200.0);
        for _ in 0..30 {
            w.step();
        }
        assert!(w.b.c.bodies[soft].alive);
        assert_eq!(w.log_halves().count(), 2, "{:?}", w.b.c.bodies[hard].alive);
        assert!(!w.b.c.bodies[hard].alive);
    }

    /// `NPCharacter.removeBody`: removing the chest breaks neck, both shoulders and waist; removing a thigh breaks its hip and knee; the rest stays joined.
    #[test]
    fn npc_remove_body_breaks_the_right_joints() {
        let xml = "<levelXML><info v='1.99'/><specials><sp t='17' p0='1000' p1='100' p2='0' p3='1' p4='f' p5='f'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (chest, thigh) = (w.npcs[0].bodies[0], w.npcs[0].bodies[7]);
        w.npc_remove_body(thigh);
        let broken = |w: &World| w.npcs[0].joints.iter().map(|j| j.3).collect::<Vec<_>>();
        assert_eq!(broken(&w), [false, false, false, false, false, false, true, false, true, false]);
        w.npc_remove_body(chest);
        assert_eq!(broken(&w), [true, true, true, true, false, false, true, false, true, false]);
    }

    /// Breaking an NPC's joints makes blood flows/bursts and the original's sounds (neck, limb rips, bone breaks), all existing sprite names.
    #[test]
    fn npc_joint_breaks_bleed_and_make_noise() {
        let xml = "<levelXML><info v='1.99'/><specials><sp t='17' p0='1000' p1='100' p2='0' p3='1' p4='f' p5='f'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let chest = w.npcs[0].bodies[0];
        w.npc_remove_body(chest);
        let names: Vec<String> = w.take_sounds().into_iter().map(|s| s.0).collect();
        for n in ["NeckBreak", "LimbRip1", "LimbRip2"] {
            assert!(names.iter().any(|s| s == n), "{n} missing in {names:?}");
        }
        assert_eq!(w.blood.flows.len(), 2 + 1 + 4, "neck 2, waist 1, two shoulders 2 each");
        // the neck break made the NPC dead, so it stays silent (original `_dead` skips `checkVocals`)
        w.step();
        assert!(!w.take_sounds().iter().any(|s| s.0.starts_with("Char1")));
        let table: serde_json::Value = serde_json::from_str(include_str!("../data/audiosprite.json")).unwrap();
        assert!(names.iter().all(|n| table.get(n.as_str()).is_some()));
    }

    /// A living NPC whose elbow breaks voices `{tag}Elbow1` from its head; the highest priority queued wins.
    #[test]
    fn npc_vocals_play_for_a_living_npc_and_the_highest_priority_wins() {
        let xml = "<levelXML><info v='1.99'/><specials><sp t='17' p0='1000' p1='100' p2='0' p3='1' p4='f' p5='f'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        w.npc_break_joint(0, 4, true);
        w.npc_break_joint(0, 6, true);
        w.step();
        let names: Vec<String> = w.take_sounds().into_iter().map(|s| s.0).collect();
        assert!(names.iter().any(|n| n == "Char1Hip1"), "{names:?}");
        assert!(!names.iter().any(|n| n == "Char1Elbow1"), "{names:?}");
        let table: serde_json::Value = serde_json::from_str(include_str!("../data/audiosprite.json")).unwrap();
        assert!(table.get("Char1Hip1").is_some());
    }

    /// `NPCharacter.shapeImpale`: a chest impale queues the random vocal (priority 5) and is voiced; a fatal head impale without a point kills the NPC (silent).
    #[test]
    fn npc_shape_impale_voices_or_kills() {
        let xml = "<levelXML><info v='1.99'/><specials><sp t='17' p0='1000' p1='100' p2='0' p3='1' p4='f' p5='f'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (chest, head) = (w.npcs[0].bodies[0], w.npcs[0].bodies[1]);
        w.shape_impale(chest, true, None, 0.0);
        w.step();
        let names: Vec<String> = w.take_sounds().into_iter().map(|s| s.0).collect();
        assert!(names.iter().any(|n| n.starts_with("Char1")), "{names:?}");
        w.shape_impale(head, true, None, 0.0);
        assert!(w.npcs[0].dead);
    }

    /// Chair break against the original's `Chair` class (oracle `analysis/golden-chair-break.js`): floor 4000×20 at y 500, chair at (1000,380) dropped at 40 m/s.
    #[test]
    fn chair_break_matches_the_original() {
        for (reverse, first, last) in [
            ("f", [[995.1255, 454.3433, 0.0247], [995.1255, 454.3433, 0.0247], [995.1255, 454.3433, 0.0247], [995.1255, 454.3433, 0.0247]], [[1566.544, 484.4102, -3.1413], [1180.963, 481.0451, 11.9779], [1968.429, 140.9048, 16.9869], [1835.3525, 469.1534, 17.2762]]),
            ("t", [[1007.2058, 455.2993, 0.057]; 4], [[1924.0745, 490.1777, 0.0064], [992.1411, 470.6779, 32.0996], [2496.1112, 540.7303, 13.1208], [2458.1406, 577.2445, 91.0487]]),
        ] {
            let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes><specials><sp t='19' p0='1000' p1='380' p2='0' p3='{reverse}' p4='f' p5='t'/></specials></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let chair = w.special_bodies[0].unwrap();
            w.b.body_mut(chair).linear_velocity = Vec2::new(0.0, 40.0);
            let mut frames = Vec::new();
            for _ in 0..40 {
                w.step();
                frames.push(w.chair_pieces().map(|p| { let [x, y] = w.body_position_px(p.0).unwrap(); [f64::from(x), f64::from(y), w.body_angle(p.0) as f64] }).collect::<Vec<_>>());
            }
            // pieces are compared as sets (the port creates them in another order), sorted by x
            let sorted = |mut v: Vec<[f64; 3]>| { v.sort_by(|a, b| a[0].total_cmp(&b[0])); v };
            for (got, want, what) in [(&frames[0], first.to_vec(), "break frame"), (&frames[39], last.to_vec(), "frame 39")] {
                let (got, want) = (sorted(got.clone()), sorted(want));
                assert_eq!(got.len(), want.len(), "{reverse} {what}: piece count");
                for (g, w) in got.iter().zip(&want) {
                    assert!((g[0] - w[0]).abs() < 0.05 && (g[1] - w[1]).abs() < 0.05 && (g[2] - w[2]).abs() < 0.01, "{reverse} {what}: {g:?} vs {w:?}");
                }
            }
        }
    }

    /// A boombox hit hard enough to shatter drops its `BoomboxHit` impact sound listener; a merely cracked one keeps it.
    #[test]
    fn shattered_boombox_stops_playing_hit_sounds() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='22' p0='900' p1='380' p2='0' p3='f' p4='t'/><sp t='22' p0='1200' p1='380' p2='0' p3='f' p4='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let (a, b) = (w.special_bodies[0].unwrap(), w.special_bodies[1].unwrap());
        w.b.body_mut(b).linear_velocity = Vec2::new(0.0, 60.0);
        let _ = a;
        assert_eq!(w.hit_sounds.iter().filter(|h| h.name == "BoomboxHit").count(), 2);
        for _ in 0..20 {
            w.step();
        }
        assert!(w.hit_sounds.iter().all(|h| h.body != b), "shattered one lost its listener");
        assert!(w.hit_sounds.iter().any(|h| h.body == a && h.name == "BoomboxHit"), "the untouched one keeps it");
    }

    /// A stabbing pane's triangles pin flesh that lands on them (sensor + prismatic joint, blood, ImpaleSpikes); a non-stabbing pane's do not.
    #[test]
    fn glass_shards_stab_flesh() {
        let run = |stabbing: &str| {
            let xml = format!("<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='18' p0='900' p1='400' p2='200' p3='20' p4='0' p5='f' p6='10' p7='{stabbing}'/></specials></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let pane = w.special_bodies[0].unwrap();
            w.b.body_mut(pane).linear_velocity = Vec2::new(0.0, 60.0);
            for _ in 0..3 {
                w.step();
            }
            assert!(w.glass_pieces().count() > 0, "the pane broke");
            let mut pinned = 0;
            let mut sounds = false;
            // hold the first triangle still and drop a flesh block onto it
            let target = w.glass_pieces().next().unwrap().0;
            w.b.body_mut(target).linear_velocity = Vec2::default();
            w.b.body_mut(target).angular_velocity = 0.0;
            let tp = w.b.body(target).sweep.c;
            let flesh = w.b.create_body(&BodyDef { position: Vec2::new(tp.x, tp.y - 40.0 / PS), ..Default::default() });
            let mut sd = ShapeDef::oriented_box(0.5, 0.15, Vec2::default(), 0.0);
            sd.density = 7.0;
            sd.filter = Filter { category_bits: 4, mask_bits: 0xFFFF, group_index: 0 };
            let s = w.b.create_shape(flesh, &sd);
            w.b.shape_mut(s).material = 2;
            w.b.set_mass_from_shapes(flesh);
            w.b.body_mut(flesh).linear_velocity = Vec2::new(0.0, 3.5);
            for _ in 0..60 {
                w.step();
                pinned = pinned.max(w.glasses.iter().flat_map(|g| g.shards.iter()).map(|s| s.pinned.len()).sum::<usize>());
                sounds |= w.take_sounds().iter().any(|s| s.0.starts_with("ImpaleSpikes"));
            }
            (pinned, sounds)
        };
        let (pinned, sounds) = run("t");
        assert!(pinned > 0 && sounds, "stabbing glass pins flesh ({pinned}, {sounds})");
        assert_eq!(run("f").0, 0, "non-stabbing glass does not");
    }

    /// Helicopter blade shards stab like `MN` (oracle `analysis/golden-blade-shard.js`, class `34531.J7`): a quad shard (box 1.0×0.1 m, density 3, category 260) lies on the
    /// floor at (1000, 483.7); a flesh box (0.3×0.15, density 7, material 2, category 4) hits it at 14 m/s, bounces, and is pinned from frame 11 (visible after that step; the
    /// port's joint exists one step earlier in its own bookkeeping) with the shard switched to stab mode; non-flesh material is never pinned.
    #[test]
    fn heli_blade_shards_pin_flesh_like_the_original() {
        let run = |material: i32| {
            let xml = format!("<levelXML><info v='1.99'/><shapes>{FLOOR}</shapes></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let shard = {
                let b = w.b2_mut();
                let id = b.create_body(&BodyDef { position: Vec2::new(1000.0 / PS, 483.7 / PS), ..Default::default() });
                let mut sd = ShapeDef::oriented_box(1.0, 0.1, Vec2::default(), 0.0);
                sd.density = 3.0;
                sd.friction = 0.3;
                sd.restitution = 0.1;
                sd.filter = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
                b.create_shape(id, &sd);
                b.set_mass_from_shapes(id);
                id
            };
            w.add_blade_shard(shard);
            let flesh = w.b.create_body(&BodyDef { position: Vec2::new(1000.0 / PS, 430.0 / PS), ..Default::default() });
            let mut sd = ShapeDef::oriented_box(0.3, 0.15, Vec2::default(), 0.0);
            sd.density = 7.0;
            sd.friction = 0.3;
            sd.restitution = 0.1;
            sd.filter = Filter { category_bits: 4, mask_bits: 0xFFFF, group_index: 0 };
            let s = w.b.create_shape(flesh, &sd);
            w.b.shape_mut(s).material = material;
            w.b.set_mass_from_shapes(flesh);
            w.b.body_mut(flesh).linear_velocity = Vec2::new(0.0, 14.0);
            let mut frames = Vec::new();
            for _ in 0..100 {
                w.step();
                let n = w.blade_shards.iter().map(|s| s.pinned.len()).sum::<usize>();
                let pos: Vec<[f64; 3]> = [flesh, shard].iter().map(|&b| { let [x, y] = w.body_position_px(b).unwrap(); [f64::from(x), f64::from(y), f64::from(w.body_angle(b))] }).collect();
                frames.push((n, pos));
            }
            frames
        };
        let frames = run(2);
        assert_eq!(frames.iter().position(|f| f.0 > 0), Some(10), "first frame with the pin");
        assert!(frames[10..].iter().all(|f| f.0 == 1), "stays pinned");
        for (k, want) in [
            (5, [[999.198, 463.473, 0.016], [999.970, 484.128, 0.000]]),
            (9, [[998.396, 468.110, 0.032], [999.970, 484.104, 0.000]]),
            (12, [[998.195, 473.919, 0.036], [999.970, 484.092, 0.000]]),
            (39, [[998.196, 473.926, 0.036], [999.971, 484.065, 0.000]]),
            (99, [[998.196, 473.926, 0.036], [999.971, 484.065, 0.000]]),
        ] {
            for (g, w) in frames[k].1.iter().zip(&want) {
                assert!((g[0] - w[0]).abs() < 0.05 && (g[1] - w[1]).abs() < 0.05 && (g[2] - w[2]).abs() < 0.01, "frame {k}: {g:?} vs {w:?}");
            }
        }
        assert!(run(1).iter().all(|f| f.0 == 0), "non-flesh is not pinned");
    }

    /// Glass pieces are shards too: a hard hit on a piece (Result above 10·its mass) destroys it with a burst of glass.
    #[test]
    fn glass_pieces_shatter_again() {
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='18' p0='900' p1='380' p2='100' p3='20' p4='0' p5='f' p6='10' p7='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let pane = w.special_bodies[0].unwrap();
        w.b.body_mut(pane).linear_velocity = Vec2::new(0.0, 60.0);
        for _ in 0..4 {
            w.step();
        }
        let pieces: Vec<_> = w.glass_pieces().map(|p| p.0).collect();
        assert!(!pieces.is_empty());
        for &p in &pieces {
            w.b.body_mut(p).linear_velocity = Vec2::new(0.0, 120.0);
        }
        for _ in 0..40 {
            w.step();
        }
        let alive = pieces.iter().filter(|&&p| w.b.c.bodies[p].alive).count();
        assert!(alive < pieces.len(), "some pieces break again ({alive}/{})", pieces.len());
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
            let lvl = crate::level::parse(&format!("<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='725' p2='4000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='17' p0='1000' p1='300' p2='0' p3='{idx}' p4='f' p5='{rev}' p6='f' p7='t'/></specials></levelXML>")).unwrap();
            let mut w = World::from_level(&lvl);
            assert_eq!(w.npcs.len(), 1, "npc {key} not created");
            let mut worst = 0.0f64;
            for f in 0..30 {
                w.step();
                if f % 10 == 9 {
                    let row = &gold[key][f / 10];
                    for (i, &b) in w.npcs[0].bodies.iter().enumerate() {
                        let body = w.b.body(b);
                        let g = &row[i];
                        let (gx, gy) = (g[0].as_f64().unwrap() + 1000.0, g[1].as_f64().unwrap());
                        let e = (body.sweep.c.x * PS - gx).abs().max((body.sweep.c.y * PS - gy).abs()); worst = worst.max(e);
                    }
                }
            }
            assert!(worst < 1e-6, "npc {key}: worst divergence {worst} px");
        }
    }

    #[test]
    fn npc_pose_angles_match_original() {
        let gold: serde_json::Value = serde_json::from_str(include_str!("../../../tests/golden/npc_pose.json")).unwrap();
        for case in gold.as_array().unwrap() {
            let names = ["neck", "shoulder1", "shoulder2", "elbow1", "elbow2", "hip1", "hip2", "knee1", "knee2"];
            let mut attrs = String::new();
            for (i, n) in names.iter().enumerate() {
                attrs += &format!(" p{}='{}'", 8 + i, case["p"].get(n).and_then(|v| v.as_f64()).unwrap_or(0.0));
            }
            let tf = |k: &str| if case[k].as_bool().unwrap() { 't' } else { 'f' };
            let xml = format!("<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='725' p2='4000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='17' p0='1000' p1='300' p2='0' p3='{}' p4='f' p5='{}' p6='{}' p7='t'{attrs}/></specials></levelXML>", case["char"], tf("reverse"), tf("hold"));
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let check = |w: &World, key: &str, tol: f64| {
                for (i, &b) in w.npcs[0].bodies.iter().enumerate() {
                    let g = &case[key][i];
                    let body = w.b.body(b);
                    let (gx, gy, ga) = (g[0].as_f64().unwrap(), g[1].as_f64().unwrap(), g[2].as_f64().unwrap());
                    let err = (body.sweep.c.x * PS - gx).abs().max((body.sweep.c.y * PS - gy).abs()).max((body.sweep.a - ga).abs() * 100.0);
                    assert!(err < tol, "{key} body {i} off by {err}: port ({}, {}, {}) gold ({gx}, {gy}, {ga})", body.sweep.c.x * PS, body.sweep.c.y * PS, body.sweep.a);
                }
            };
            check(&w, "start", 1e-6);
            for _ in 0..30 { w.step(); }
            check(&w, "after", 1e-3);
        }
    }

    #[test]
    fn npc_trigger_actions_match_original() {
        let gold: serde_json::Value = serde_json::from_str(include_str!("../../../tests/golden/npc_trigger.json")).unwrap();
        let names = ["neck", "shoulder1", "shoulder2", "elbow1", "elbow2", "hip1", "hip2", "knee1", "knee2"];
        for case in gold.as_array().unwrap() {
            let mut attrs = String::new();
            for (i, n) in names.iter().enumerate() {
                attrs += &format!(" p{}='{}'", 8 + i, case["p"].get(n).and_then(|v| v.as_f64()).unwrap_or(0.0));
            }
            let rev = if case["reverse"].as_bool().unwrap() { 't' } else { 'f' };
            let xml = format!("<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='725' p2='4000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='17' p0='1000' p1='300' p2='0' p3='1' p4='f' p5='{rev}' p6='f' p7='t'{attrs}/></specials></levelXML>");
            let mut w = World::from_level(&crate::level::parse(&xml).unwrap());
            let chest = w.npcs[0].bodies[0];
            let mut action = vec![case["act"].as_str().unwrap().to_string()];
            action.extend(case["props"].as_array().unwrap().iter().map(|v| v.to_string()));
            for f in 0..40 {
                if f == case["actAt"].as_u64().unwrap() { w.npc_trigger_action(chest, &action); }
                w.step();
            }
            for (i, &b) in w.npcs[0].bodies.iter().enumerate() {
                let g = &case["after"][i];
                let body = w.b.body(b);
                let err = (body.sweep.c.x * PS - g[0].as_f64().unwrap()).abs().max((body.sweep.c.y * PS - g[1].as_f64().unwrap()).abs()).max((body.sweep.a - g[2].as_f64().unwrap()).abs() * 100.0);
                assert!(err < 1e-3, "{} body {i} off by {err}", case["act"]);
            }
        }
    }

    #[test]
    fn npc_slammed_into_the_floor_breaks_like_the_original() {
        // oracle (`analysis/npc-pose.js`, vel 60, level actions on): 38 -> 42 dynamic bodies after 60 frames
        let xml = "<levelXML><info v='1.99'/><shapes><sh t='0' p0='1000' p1='725' p2='4000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='17' p0='1000' p1='300' p2='0' p3='1' p4='f' p5='f' p6='f' p7='t'/></specials></levelXML>";
        let mut w = World::from_level(&crate::level::parse(xml).unwrap());
        let count = |w: &World| w.b.c.bodies.iter().filter(|b| b.alive && b.mass > 0.0).count();
        let before = count(&w);
        for b in w.npc_main_bodies() { w.b.body_mut(b).linear_velocity = Vec2::new(0.0, 60.0); }
        for _ in 0..60 { w.step(); }
        assert_eq!(count(&w) - before, 4, "the original ends with four extra bodies");
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
    fn finish_line_completes_only_for_living_rider_in_zone() {
        let level = crate::level::parse("<levelXML><info v='1.99' x='1000' y='200'/><specials><sp t='9' p0='1000' p1='300'/></specials></levelXML>").unwrap();
        for (start, dead, expected) in [(1000.0, false, true), (1500.0, false, false), (1000.0, true, false)] {
            let mut w = World::from_level(&level);
            assert!(w.spawn_character(1, [start, 200.0]));
            w.character.as_mut().unwrap().dead = dead;
            // An independent object can contact the finish in the original; the focus position decides victory.
            let body = w.b.create_body(&BodyDef { position: Vec2::new(1000.0 / PS, 250.0 / PS), ..Default::default() });
            let mut shape = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 10.0 / PS });
            shape.density = 1.0;
            w.b.create_shape(body, &shape);
            w.b.set_mass_from_shapes(body);
            let mut victories = 0;
            for _ in 0..30 {
                w.step();
                victories += w.take_sounds().iter().filter(|s| s.0 == "Victory").count();
            }
            assert_eq!(w.level_complete, expected, "start={start}, dead={dead}");
            assert_eq!(victories, usize::from(expected));
        }
    }

    #[test]
    fn camera_follows_heart_after_chest_smash() {
        let level = crate::level::parse("<levelXML><info v='1.99' x='1000' y='200'/></levelXML>").unwrap();
        let mut w = World::from_level(&level);
        assert!(w.spawn_character(1, level.info.start));
        let mut rider = w.character.take().unwrap();
        let chest = rider.bodies[&crate::character::Part::Chest];
        let before = w.body_center_px(chest).unwrap();
        w.character = Some(rider);
        assert_eq!(w.camera_focus_px(), Some(before));
        rider = w.character.take().unwrap();
        rider.smash_chest(&mut w);
        let heart = rider.heart.unwrap();
        w.character = Some(rider);
        assert_eq!(w.camera_focus_px(), w.body_center_px(heart));
        assert!(w.camera_focus_px().is_some());
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
