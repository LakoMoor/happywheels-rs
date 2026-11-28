//! Base human ragdoll (class `sP` of the original): 11 bodies + 10 revolute joints.
//! Placement comes from the character's `shapeGuide` (data/character-guides.json): each part is
//! `[x, y, rotation°, scaleX, scaleY]` in character-clip pixels. Conversion (verified from the
//! original): `character_scale = PHYS_SCALE * 2`, `shapeRefScale = 5`; body origin = start + (x, y) /
//! character_scale (metres), box half-extents = scale * 5 / character_scale, head radius = scaleX * 5 /
//! character_scale. Material: density 1, friction .3, restitution .1; filter category 260, mask 270,
//! group index `-1` (limbs of one character never collide; enforced by a physics hook).

use box2d::{
    joints::{DistanceJointDef, RevoluteJointDef},
    math::Vec2,
    shape::{Filter, Geometry, ShapeDef},
    world::{BodyDef, BodyId},
};
use std::collections::HashMap;

use crate::physics::World;

const GUIDES: &str = include_str!("../data/character-guides.json");
const SUBGUIDES: &str = include_str!("../data/character-subguides.json");
const CHARACTER_SCALE: f64 = 62.5 * 2.0;
const REF_SCALE: f64 = 5.0;

pub type Guide = HashMap<String, [f64; 7]>;

/// Guide of character `index` (1-based), if it exists.
pub fn guide(index: usize) -> Option<Guide> {
    let all: HashMap<String, serde_json::Value> = serde_json::from_str(GUIDES).ok()?;
    let g = all.get(&format!("c{index}"))?;
    serde_json::from_value(g.clone()).ok()
}

/// Nested guide of a sub-character (`c10_daughter`, `c10_son`, `c8_elf1`, `c8_elf2`).
pub fn subguide(name: &str) -> Option<Guide> {
    let all: HashMap<String, serde_json::Value> = serde_json::from_str(SUBGUIDES).ok()?;
    serde_json::from_value(all.get(name)?.get("guide")?.clone()).ok()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Part {
    Chest,
    Head,
    Pelvis,
    UpperArm1,
    UpperArm2,
    UpperLeg1,
    UpperLeg2,
    LowerArm1,
    LowerArm2,
    LowerLeg1,
    LowerLeg2,
}

const PARTS: [(Part, &str); 11] = [
    (Part::Chest, "chestShape"),
    (Part::Head, "headShape"),
    (Part::Pelvis, "pelvisShape"),
    (Part::UpperArm1, "upperArm1Shape"),
    (Part::UpperArm2, "upperArm2Shape"),
    (Part::UpperLeg1, "upperLeg1Shape"),
    (Part::UpperLeg2, "upperLeg2Shape"),
    (Part::LowerArm1, "lowerArm1Shape"),
    (Part::LowerArm2, "lowerArm2Shape"),
    (Part::LowerLeg1, "lowerLeg1Shape"),
    (Part::LowerLeg2, "lowerLeg2Shape"),
];

/// (anchor name, parent, child, lower°, upper°) — limits are absolute relative angles in degrees.
const JOINTS: [(&str, Part, Part, f64, f64); 10] = [
    ("headAnchor", Part::Chest, Part::Head, -20.0, 20.0),
    ("pelvisAnchor", Part::Chest, Part::Pelvis, -5.0, 5.0),
    ("upperArmAnchor", Part::Chest, Part::UpperArm1, -180.0, 60.0),
    ("upperArmAnchor", Part::Chest, Part::UpperArm2, -180.0, 60.0),
    ("lowerArm1Anchor", Part::UpperArm1, Part::LowerArm1, -160.0, 0.0),
    ("lowerArm2Anchor", Part::UpperArm2, Part::LowerArm2, -160.0, 0.0),
    ("upperLegAnchor", Part::Pelvis, Part::UpperLeg1, -150.0, 10.0),
    ("upperLegAnchor", Part::Pelvis, Part::UpperLeg2, -150.0, 10.0),
    ("lowerLeg1Anchor", Part::UpperLeg1, Part::LowerLeg1, 0.0, 150.0),
    ("lowerLeg2Anchor", Part::UpperLeg2, Part::LowerLeg2, 0.0, 150.0),
];

/// Parts in the original's creation order (also the order of tests/golden/ragdoll.json).
pub const PART_ORDER: [Part; 11] = [
    Part::Chest,
    Part::Head,
    Part::Pelvis,
    Part::UpperArm1,
    Part::UpperArm2,
    Part::UpperLeg1,
    Part::UpperLeg2,
    Part::LowerArm1,
    Part::LowerArm2,
    Part::LowerLeg1,
    Part::LowerLeg2,
];

/// Joints by role (creation order of `JOINTS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Limb {
    Neck,
    Waist,
    Shoulder1,
    Shoulder2,
    Elbow1,
    Elbow2,
    Hip1,
    Hip2,
    Knee1,
    Knee2,
}

const LIMB_ORDER: [Limb; 10] = [Limb::Neck, Limb::Waist, Limb::Shoulder1, Limb::Shoulder2, Limb::Elbow1, Limb::Elbow2, Limb::Hip1, Limb::Hip2, Limb::Knee1, Limb::Knee2];

/// Pose table of the original (`supermanPose`, `tuckPose`, `pushupPose`, `archPose`, …): per joint
/// `(target offset above the lower limit, speed gain)`; `setJoint` drives the motor towards `lower + offset`.
fn pose_table(set: PoseSet, pose: u8) -> &'static [(Limb, f64, f64)] {
    use Limb::*;
    match (set, pose) {
        (PoseSet::Segway, p) if (5..=8).contains(&p) => match p {
            5 => &[(Knee1, 0.0, 10.0), (Knee2, 0.0, 10.0)],
            6 => &[(Knee1, 1.5, 10.0), (Knee2, 1.5, 10.0)],
            7 => &[(Neck, 0.5, 2.0), (Hip1, 1.5, 2.0), (Hip2, 3.5, 2.0), (Knee1, 1.7, 10.0), (Knee2, 0.0, 10.0), (Shoulder1, 3.0, 20.0), (Shoulder2, 1.0, 20.0), (Elbow1, 1.5, 15.0), (Elbow2, 3.0, 15.0)],
            8 => &[(Neck, 0.5, 2.0), (Hip1, 3.5, 2.0), (Hip2, 1.5, 2.0), (Knee1, 0.0, 10.0), (Knee2, 1.7, 10.0), (Shoulder1, 1.0, 20.0), (Shoulder2, 3.0, 20.0), (Elbow1, 3.0, 15.0), (Elbow2, 1.5, 15.0)],
            _ => &[],
        },
        (PoseSet::Chair, 5) => &[(Neck, 0.5, 2.0), (Hip1, 0.5, 5.0), (Hip2, 0.5, 5.0), (Knee1, 1.0, 10.0), (Knee2, 1.0, 10.0), (Shoulder1, 3.0, 20.0), (Shoulder2, 3.0, 20.0), (Elbow1, 1.4, 15.0), (Elbow2, 1.4, 15.0)],
        (PoseSet::Chair, 6) => &[(Neck, 0.5, 2.0), (Hip1, 3.5, 2.0), (Hip2, 1.5, 2.0), (Knee1, 0.0, 10.0), (Knee2, 0.0, 10.0), (Shoulder1, -0.5, 20.0), (Shoulder2, 1.0, 20.0), (Elbow1, 3.0, 15.0), (Elbow2, 3.0, 15.0)],
        (PoseSet::Chair, 7) => &[(Neck, 0.5, 2.0), (Hip1, 1.5, 2.0), (Hip2, 3.5, 2.0), (Knee1, 0.0, 10.0), (Knee2, 0.0, 10.0), (Shoulder1, 1.0, 20.0), (Shoulder2, -0.5, 20.0), (Elbow1, 3.0, 15.0), (Elbow2, 3.0, 15.0)],
        (PoseSet::Cart, 6) => pose_table(PoseSet::Mower, 6),
        (PoseSet::Cart, 7) => &[(Neck, 0.5, 2.0), (Hip1, 0.0, 2.0), (Hip2, 3.5, 2.0), (Knee1, 2.0, 10.0), (Knee2, 0.0, 10.0), (Shoulder1, 1.0, 20.0), (Shoulder2, 3.0, 20.0), (Elbow1, 3.0, 15.0), (Elbow2, 1.5, 15.0)],
        (PoseSet::Heli, 6) => pose_table(PoseSet::Mower, 6),
        (PoseSet::Heli, 7) => &[(Neck, 0.0, 10.0), (Elbow1, 2.0, 5.0), (Elbow2, 2.0, 5.0)],
        (PoseSet::Heli, 8) => &[(Neck, 0.4, 10.0), (Elbow1, 0.5, 5.0), (Elbow2, 0.5, 5.0)],
        (PoseSet::Mower, 6) => &[(Neck, 0.5, 2.0), (Hip1, 3.5, 2.0), (Hip2, 0.0, 2.0), (Knee1, 0.0, 10.0), (Knee2, 2.0, 10.0), (Shoulder1, 3.0, 20.0), (Shoulder2, 1.0, 20.0), (Elbow1, 1.5, 15.0), (Elbow2, 3.0, 15.0)],
        (PoseSet::Mower, 7) => &[(Neck, 0.0, 2.0), (Elbow1, 2.5, 15.0), (Elbow2, 2.5, 15.0)],
        (PoseSet::Mower, 8) => &[(Neck, 1.0, 1.0), (Elbow1, 0.0, 15.0), (Elbow2, 0.0, 15.0)],
        (PoseSet::Grinder, 7) => &[(Knee1, 45.0, 10.0), (Knee2, 45.0, 10.0), (Hip1, -45.0 / (180.0 / std::f64::consts::PI), 10.0), (Hip2, -45.0 / (180.0 / std::f64::consts::PI), 10.0)],
        (PoseSet::Grinder, 8) => &[(Knee1, 0.0, 10.0), (Knee2, 0.0, 10.0), (Hip1, 0.0, 10.0), (Hip2, 0.0, 10.0), (Elbow1, -15.0 / (180.0 / std::f64::consts::PI), 10.0), (Elbow2, -15.0 / (180.0 / std::f64::consts::PI), 10.0)],
        (PoseSet::Pogo, 5) => &[(Knee1, 0.25, 10.0), (Knee2, 0.25, 10.0)],
        (PoseSet::Pogo, 6) => &[(Knee1, 2.5, 10.0), (Knee2, 2.5, 10.0)],
        (PoseSet::Pogo, 9) => &[(Knee1, 1.2, 10.0), (Knee2, 1.2, 10.0)],
        (PoseSet::Pogo, 7) => pose_table(PoseSet::Segway, 7),
        (PoseSet::Pogo, 8) => pose_table(PoseSet::Segway, 8),
        (_, 5..=9) => &[],
        (_, pose) => base_pose(pose),
    }
}

fn base_pose(pose: u8) -> &'static [(Limb, f64, f64)] {
    use Limb::*;
    match pose {
        1 => &[(Neck, 0.5, 2.0), (Hip1, 3.0, 2.0), (Hip2, 3.0, 2.0), (Knee1, 2.0, 10.0), (Knee2, 2.0, 10.0), (Shoulder1, 0.0, 20.0), (Shoulder2, 0.0, 20.0), (Elbow1, 1.0, 15.0), (Elbow2, 1.0, 15.0)],
        2 => &[(Neck, 0.5, 2.0), (Hip1, 2.6, 2.0), (Hip2, 2.6, 2.0), (Knee1, 0.0, 10.0), (Knee2, 0.0, 10.0), (Shoulder1, 1.7, 20.0), (Shoulder2, 1.7, 20.0), (Elbow1, 2.5, 15.0), (Elbow2, 2.5, 15.0)],
        3 => &[(Neck, 0.5, 2.0), (Hip1, 2.6, 2.0), (Hip2, 2.6, 2.0), (Knee1, 0.0, 10.0), (Knee2, 0.0, 10.0), (Shoulder1, 0.0, 20.0), (Shoulder2, 0.0, 20.0), (Elbow1, 2.5, 15.0), (Elbow2, 2.5, 15.0)],
        4 => &[(Neck, 0.69, 2.0), (Hip1, 0.0, 2.0), (Hip2, 0.0, 2.0), (Knee1, 2.0, 10.0), (Knee2, 2.0, 10.0), (Shoulder1, 3.0, 20.0), (Shoulder2, 3.0, 20.0), (Elbow1, 1.5, 15.0), (Elbow2, 1.5, 15.0)],
        10 => &[(Neck, 0.5, 2.0), (Shoulder1, 1.5, 20.0), (Shoulder2, 1.5, 20.0), (Elbow1, 2.0, 15.0), (Elbow2, 2.0, 15.0)],
        11 => &[(Neck, 0.5, 2.0), (Shoulder1, 0.0, 20.0), (Shoulder2, 0.0, 20.0), (Elbow1, 2.5, 15.0), (Elbow2, 2.5, 15.0)],
        _ => &[],
    }
}

/// Which table the vehicle-specific pose numbers 5..=8 refer to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PoseSet {
    #[default]
    Base,
    Segway,
    Chair,
    Mower,
    Pogo,
    Cart,
    Heli,
    Grinder,
}

pub struct Ragdoll {
    /// Joints not created yet (`spawn_scaled`): the original creates every body of the character AND its vehicle before any joint, and a joint's
    /// `RefilterProxy` reorders the contact list, so the vehicle calls `finish_joints` after its own bodies.
    deferred: Option<(Guide, Vec2)>,
    pub poses: PoseSet,
    pub bodies: HashMap<Part, BodyId>,
    pub joints: HashMap<Limb, usize>,
    /// Current pose (0 = none, 1 arch, 2 pushup, 3 superman, 4 tuck, 10 arms forward, 11 arms overhead, 12 hold).
    pub pose: u8,
    /// Set when the pose returned to 0: `cancelPose` (motors off) runs once, like the original's setter.
    cancel: bool,
    /// Joints already broken (destroyed) by `check_joints`.
    pub broken: Vec<Limb>,
    /// Space held: hands grab what they touch (`startGrab`).
    pub grabbing: bool,
    /// `lowerArm*MC.hand` shows its open frame: after `releaseGrip`, or after the eject of the vehicles that open the hands.
    pub hands_open: bool,
    /// The eject has already been seen by [`World::step`](crate::physics::World::step).
    pub(crate) eject_noted: bool,
    /// Grip joints of the two hands (`gripJoint1/2`).
    pub grips: [Option<usize>; 2],
    /// Collision group of the character (`groupID`, −1 for the player, −2/−3 for sub-characters).
    pub group: i16,
    /// Radius of a head chunk in metres (`headChunkShape` width / 2 / 125) and the debris left by `smash_head`.
    head_chunk_radius: f64,
    chest_chunk_radius: f64,
    pelvis_chunk_radius: f64,
    foot_guide: [f64; 7],
    ref_scale: f64,
    pub chunks: Vec<BodyId>,
    /// Sprite clip name of each entry of `chunks` (`headChunk1`…, `chestChunk1`…, `pelvisChunk1`…).
    pub chunk_names: Vec<String>,
    pub brain: Option<BodyId>,
    pub heart: Option<BodyId>,
    /// Detached foot bodies (`foot1Body` / `foot2Body`).
    pub feet: [Option<BodyId>; 2],
    /// `dead` (set by a head smash; the original's setter also ejects the vehicle).
    pub dead: bool,
    /// Helmet (guides with `helmetVert1..4`): while `helmet_on` the head's smash limit is `HELMET_LIMIT`; `helmet_smash` drops a loose helmet body.
    pub helmet_geo: Vec<Vec2>,
    pub helmet_on: bool,
    pub helmet: Option<BodyId>,
    /// Severed-limb stumps (`upperArm3/4Body`, `upperLeg3/4Body`) left when a shoulder/hip breaks softly (below its snap limit).
    pub sev: Vec<BodyId>,
    /// The limb each `sev` stump belongs to.
    pub sev_limbs: Vec<Limb>,
    /// Parts in `GRIND_STATE` (a mower blade took them): their smash reactions are off (`grindShape` deletes the Result/Add listeners).
    pub ground: Vec<Part>,
    /// `pelvisSmash` happened (stump thighs then show frame 8).
    pelvis_smashed: bool,
    sev_guides: [[f64; 7]; 3],
    /// Exposed intestines (torso break below `intestineLimit`) and spinal cord (neck break below `spineLimit`).
    pub intestines: Option<Chain>,
    /// Running blood flows `(kind, flow index)`: 0 head, 1 neck→chest, 2 shoulder→chest, 3 arm/thigh stump, 4 stomach, 5 hip→pelvis.
    blood_flows: Vec<(u8, usize)>,
    /// Sound-set prefix of the original's `tag` ("Char3", "Santa", …) and its vocal queue (`addVocals(name, priority)`, played by `check_vocals`).
    pub tag: &'static str,
    voices: [Option<&'static str>; 6],
    voice_until: u32,
    voice_prio: i32,
    pub spine: Option<Chain>,
    /// Elbow 1/2 and knee 1/2 ligaments (class `i$`, built when the joint breaks below `elbow/kneeLigamentLimit`).
    pub ligaments: [Option<Chain>; 4],
    /// Link lengths in metres (`intestineShape.width`, `ligamentShape.width` over the character scale) and the spine anchor (guide px).
    chain_len: [f64; 2],
    spine_anchor: [f64; 2],
}

/// Called after each joint a smash breaks.
pub type BreakHook<'a> = dyn FnMut(&mut World, &mut Ragdoll, Limb) + 'a;

/// A string of fixed-rotation balls joined by rigid distance joints between two bodies (original classes `h` and `e1`): `joints[0]` ties the
/// first end, `joints.last()` the second; only those two can tear (impulse above 1.5).
#[derive(Debug, Clone)]
pub struct Chain {
    pub bodies: Vec<BodyId>,
    pub joints: Vec<Option<usize>>,
    /// Ligaments only: the main joint already tore once (`mainJoint.broken`).
    pub torn: bool,
}

impl Ragdoll {
    pub const HELMET_LIMIT: f64 = 2.0;

    /// `helmetSmash` (original e.g. 30895, same in every class): the head shape stops being the helmet (its limit becomes the normal head limit) and a
    /// loose helmet body (the 4 guide vertices as local coordinates, density 1, zero filter) appears at the head with its velocities.
    pub fn helmet_smash(&mut self, world: &mut World) {
        if !self.helmet_on {
            return;
        }
        self.helmet_on = false;
        let head = self.bodies[&Part::Head];
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let (pos, ang) = (b.body(head).xf.position, b.body(head).sweep.a);
        let (v, w) = (b.body(head).linear_velocity, b.body(head).angular_velocity);
        let h = b.create_body(&BodyDef { position: pos, angle: ang, ..Default::default() });
        let mut d = ShapeDef::new(Geometry::Polygon(self.helmet_geo.clone()));
        d.density = 1.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = zero;
        b.create_shape(h, &d);
        b.set_mass_from_shapes(h);
        b.body_mut(h).linear_velocity = v;
        b.body_mut(h).angular_velocity = w;
        self.helmet = Some(h);
    }

    /// Spawns the ragdoll at `start_px` (level start, pixels) into `world`. `None` if the guide lacks parts.
    /// Matches the original's Box2D simulation to ~1e-10 px (tests/golden.rs in the box2d crate).
    pub fn spawn(world: &mut World, guide: &Guide, start_px: [f32; 2]) -> Option<Self> {
        Self::spawn_with(world, guide, start_px, false)
    }

    /// `mower_torso`: the lawnmower class destroys the base chest/pelvis boxes and recreates them as polygons
    /// (`chestVert0..5`, `pelvisVert0..4`, local coordinates) before the joints are made.
    pub fn spawn_with(world: &mut World, guide: &Guide, start_px: [f32; 2], mower_torso: bool) -> Option<Self> {
        let mut d = Self::spawn_scaled(world, guide, start_px, mower_torso, REF_SCALE)?;
        d.finish_joints(world);
        Some(d)
    }

    /// `ref_scale` is the class's `shapeRefScale` (5 for most characters, 50 for the pogo whose guide scales are 1/10).
    pub fn spawn_scaled(world: &mut World, guide: &Guide, start_px: [f32; 2], mower_torso: bool, ref_scale: f64) -> Option<Self> {
        let mut d = Self::spawn_inner(world, guide, start_px, mower_torso, ref_scale, -1, true)?;
        d.deferred = Some((guide.clone(), Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5)));
        Some(d)
    }

    /// Creates the joints left out by `spawn_scaled` (no-op when already created).
    pub fn finish_joints(&mut self, world: &mut World) {
        let Some((guide, start)) = self.deferred.take() else { return };
        let b = world.b2_mut();
        let ids = Self::make_joints(b, &guide, start, &self.bodies);
        if let Some(ids) = ids {
            self.joints = LIMB_ORDER.iter().copied().zip(ids).collect();
        }
    }

    fn make_joints(b: &mut box2d::world::World, guide: &Guide, start: Vec2, bodies: &HashMap<Part, BodyId>) -> Option<Vec<usize>> {
        let deg = 180.0 / std::f64::consts::PI;
        let mut joint_ids = Vec::new();
        for (anchor, parent, child, lo, hi) in JOINTS {
            let [x, y, ..] = *guide.get(anchor)?;
            let pt = Vec2::new(start.x + x / CHARACTER_SCALE, start.y + y / CHARACTER_SCALE);
            let (p, c) = (bodies[&parent], bodies[&child]);
            let mut d = RevoluteJointDef::initialize(b.body(p), p, b.body(c), c, pt);
            // Limits are absolute relative angles; Box2D measures from the creation pose.
            let rel = b.body(c).sweep.a - b.body(p).sweep.a;
            d.enable_limit = true;
            d.max_motor_torque = 4.0;
            d.lower_angle = lo / deg - rel;
            d.upper_angle = hi / deg - rel;
            joint_ids.push(b.create_revolute_joint(&d));
        }
        Some(joint_ids)
    }

    /// Full constructor: `group` is the character's `groupID` (limbs of one group never collide with each other).
    pub fn spawn_full(world: &mut World, guide: &Guide, start_px: [f32; 2], mower_torso: bool, ref_scale: f64, group: i16) -> Option<Self> {
        Self::spawn_inner(world, guide, start_px, mower_torso, ref_scale, group, false)
    }

    /// Like `spawn_full` but without the joints (see `finish_joints`): for characters whose class changes shapes between `createBodies` and `createJoints`.
    pub fn spawn_full_deferred(world: &mut World, guide: &Guide, start_px: [f32; 2], mower_torso: bool, ref_scale: f64, group: i16) -> Option<Self> {
        let mut d = Self::spawn_inner(world, guide, start_px, mower_torso, ref_scale, group, true)?;
        d.deferred = Some((guide.clone(), Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5)));
        Some(d)
    }

    fn spawn_inner(world: &mut World, guide: &Guide, start_px: [f32; 2], mower_torso: bool, ref_scale: f64, group: i16, defer: bool) -> Option<Self> {
        let start = Vec2::new(f64::from(start_px[0]) / 62.5, f64::from(start_px[1]) / 62.5);
        let b = world.b2_mut();
        let mut bodies = HashMap::new();
        for (part, name) in PARTS {
            let [x, y, rot, sx, sy, ..] = *guide.get(name)?;
            let body = b.create_body(&BodyDef {
                position: Vec2::new(start.x + x / CHARACTER_SCALE, start.y + y / CHARACTER_SCALE),
                angle: rot / (180.0 / std::f64::consts::PI),
                // `AllowSleeping(false)` is called for the chest, head and pelvis only
                allow_sleep: !matches!(part, Part::Chest | Part::Head | Part::Pelvis),
                ..Default::default()
            });
            let mut d = if part == Part::Head {
                ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: sx * ref_scale / CHARACTER_SCALE })
            } else {
                ShapeDef::boxed(sx * ref_scale / CHARACTER_SCALE, sy * ref_scale / CHARACTER_SCALE)
            };
            d.density = 1.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: group };
            let sid = b.create_shape(body, &d);
            if matches!(part, Part::Chest | Part::Head | Part::Pelvis) {
                b.shape_mut(sid).material = 2;
            }
            b.set_mass_from_shapes(body);
            bodies.insert(part, body);
        }
        let deg = 180.0 / std::f64::consts::PI;
        if mower_torso {
            for (part, name, prefix, n) in [(Part::Chest, "chestShape", "chestVert", 6), (Part::Pelvis, "pelvisShape", "pelvisVert", 5)] {
                b.destroy_body(bodies[&part]);
                let [x, y, rot, ..] = *guide.get(name)?;
                let body = b.create_body(&BodyDef {
                    position: Vec2::new(start.x + x / CHARACTER_SCALE, start.y + y / CHARACTER_SCALE),
                    angle: rot / deg,
                    allow_sleep: false,
                    ..Default::default()
                });
                let verts: Option<Vec<Vec2>> = (0..n)
                    .map(|i| guide.get(&format!("{prefix}{i}")).map(|v| Vec2::new(v[0] / CHARACTER_SCALE, v[1] / CHARACTER_SCALE)))
                    .collect();
                let mut d = ShapeDef::new(Geometry::Polygon(verts?));
                d.density = 1.0;
                d.friction = 0.3;
                d.restitution = 0.1;
                d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: group };
                b.create_shape(body, &d);
                b.set_mass_from_shapes(body);
                bodies.insert(part, body);
            }
        }
        let joint_ids = if defer { Vec::new() } else { Self::make_joints(b, guide, start, &bodies)? };
        let joints = LIMB_ORDER.iter().copied().zip(joint_ids).collect();
        Some(Self { deferred: None, poses: PoseSet::Base, bodies, joints, pose: 0, cancel: false, broken: Vec::new(), grabbing: false, hands_open: false, eject_noted: false, grips: [None; 2], group, head_chunk_radius: guide.get("headChunkShape").map_or(0.0, |g| g[3] * 5.0 / CHARACTER_SCALE), chest_chunk_radius: guide.get("chestChunkShape").map_or(0.0, |g| g[3] * 5.0 / CHARACTER_SCALE), pelvis_chunk_radius: guide.get("pelvisChunkShape").map_or(0.0, |g| g[3] * 5.0 / CHARACTER_SCALE), foot_guide: guide.get("footShape").copied().unwrap_or_default(), ref_scale, chunks: Vec::new(), chunk_names: Vec::new(), brain: None, heart: None, feet: [None; 2], dead: false, helmet_geo: (1..=4).map(|k| guide.get(&format!("helmetVert{k}")).map(|g| Vec2::new(g[0] / CHARACTER_SCALE, g[1] / CHARACTER_SCALE))).collect::<Option<Vec<_>>>().unwrap_or_default(), helmet_on: guide.get("helmetVert1").is_some(), helmet: None, sev: Vec::new(), sev_limbs: Vec::new(), ground: Vec::new(), pelvis_smashed: false, blood_flows: Vec::new(), tag: "Char3", voices: [None; 6], voice_until: 0, voice_prio: -1, intestines: None, spine: None, ligaments: [None, None, None, None], chain_len: ["intestineShape", "ligamentShape"].map(|n| guide.get(n).map_or(0.0, |g| g[5] / CHARACTER_SCALE)), spine_anchor: guide.get("spineAnchor").map_or([0.0; 2], |g| [g[0], g[1]]), sev_guides: ["sevArm1Shape", "sevLeg1Shape", "sevLeg2Shape"].map(|n| guide.get(n).copied().unwrap_or_default()) })
    }

    /// `headSmash1`: the head body is destroyed (taking the neck joint with it) and replaced by four chunk balls on a
    /// ring (4 px from the head centre, 90° apart) plus a brain ball; the character is dead.
    pub fn smash_head(&mut self, world: &mut World) {
        if self.dead {
            return;
        }
        self.sound(world, Part::Head, "HeadSmash", 1);
        world.blood_burst(5.0, 15.0, self.bodies[&Part::Head], 200, Vec2::default());
        self.stop_flows(world, &[0]);
        self.die(world);
        if self.broken.contains(&Limb::Neck) {
            self.tear_chain(world, true, 1);
        }
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let head = self.bodies[&Part::Head];
        let b = world.b2_mut();
        let (pos, angle) = (b.body(head).xf.position, b.body(head).sweep.a);
        let mut a = angle;
        let ring = 4.0 / CHARACTER_SCALE;
        let circle = |b: &mut box2d::world::World, at: Vec2, ang: f64, radius: f64| {
            let body = b.create_body(&BodyDef { position: at, angle: ang, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius });
            d.density = 1.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            body
        };
        for _ in 0..4 {
            let at = Vec2::new(pos.x + box2d::fdlibm::sin(a) * ring, pos.y + box2d::fdlibm::cos(a) * ring);
            let chunk = circle(b, at, 0.0, self.head_chunk_radius);
            let v = b.body(head).linear_velocity_from_world_point(at);
            let w = b.body(head).angular_velocity;
            b.body_mut(chunk).linear_velocity = v;
            b.body_mut(chunk).angular_velocity = w;
            self.chunks.push(chunk);
            let k = self.chunk_names.iter().filter(|s| s.starts_with("headChunk")).count() + 1;
            self.chunk_names.push(format!("headChunk{}", k.min(4)));
            a += std::f64::consts::FRAC_PI_2;
        }
        let brain = circle(b, pos, angle, 12.0 / CHARACTER_SCALE);
        let (v, w) = (b.body(head).linear_velocity, b.body(head).angular_velocity);
        b.body_mut(brain).linear_velocity = v;
        b.body_mut(brain).angular_velocity = w;
        self.brain = Some(brain);
        b.destroy_body(head);
        self.broken.push(Limb::Neck);
    }

    /// The `dead` setter: pose reset and grip release (`userVehicleEject` only matters for grabbed vehicles).
    fn die(&mut self, world: &mut World) {
        self.dead = true;
        self.set_pose(0);
        self.set_grab(world, false);
    }

    /// Destroys `limb`'s joint (like a `check_joints` break) unless it is already broken.
    /// Break a limb by hand with the given break value (what `checkRevJoint` passes to the break handler).
    pub fn break_limb(&mut self, world: &mut World, limb: Limb, force: f64) {
        self.break_joint(world, limb, force);
    }

    fn break_then(&mut self, world: &mut World, limb: Limb, force: f64, on_break: &mut BreakHook) {
        let before = self.broken.len();
        self.break_joint(world, limb, force);
        if self.broken.len() > before {
            on_break(world, self, limb);
        }
    }

    fn break_joint(&mut self, world: &mut World, limb: Limb, force: f64) {
        if self.broken.contains(&limb) {
            return;
        }
        let Some(&id) = self.joints.get(&limb) else { return };
        world.b2_mut().destroy_joint(id);
        self.broken.push(limb);
        self.break_side_effects(world, limb, force);
    }

    /// A zero-filter debris ball (density 1, friction .3, restitution .1) at `at`.
    fn debris_ball(world: &mut World, at: Vec2, angle: f64, radius: f64) -> BodyId {
        let b = world.b2_mut();
        let body = b.create_body(&BodyDef { position: at, angle, ..Default::default() });
        let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius });
        d.density = 1.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        b.create_shape(body, &d);
        b.set_mass_from_shapes(body);
        body
    }

    /// Ring of `count` chunk balls around `part`: first at `part angle + start`, `step` apart, `ring` from the centre.
    #[allow(clippy::too_many_arguments)]
    fn chunk_ring(&mut self, world: &mut World, part: Part, count: usize, start: f64, step: f64, ring: f64, radius: f64) {
        let src = self.bodies[&part];
        let (pos, angle) = (world.b2().body(src).xf.position, world.b2().body(src).sweep.a);
        let mut a = angle + start;
        for _ in 0..count {
            let at = Vec2::new(pos.x + box2d::fdlibm::sin(a) * ring, pos.y + box2d::fdlibm::cos(a) * ring);
            let chunk = Self::debris_ball(world, at, angle, radius);
            let b = world.b2_mut();
            let v = b.body(src).linear_velocity_from_world_point(at);
            let w = b.body(src).angular_velocity;
            b.body_mut(chunk).linear_velocity = v;
            b.body_mut(chunk).angular_velocity = w;
            self.chunks.push(chunk);
            let (kind, n) = if part == Part::Chest { ("chestChunk", 4) } else { ("pelvisChunk", 3) };
            let k = self.chunk_names.iter().filter(|s| s.starts_with(kind)).count() + 1;
            self.chunk_names.push(format!("{kind}{}", k.min(n)));
            a += step;
        }
    }

    /// `chestSmash`: dead; neck, both shoulders and the waist are torn off (filters only), four chunk balls (ring 5 px,
    /// starting 45° from the chest angle) + a heart ball (head-chunk radius), the chest body is stopped and destroyed.
    pub fn smash_chest(&mut self, world: &mut World) {
        self.smash_chest_with(world, &mut |_, _, _| {});
    }

    /// [`smash_chest`](Self::smash_chest) with the vehicle's break overrides: the original's `chestSmash` breaks joints through the
    /// virtual `neckBreak` / `shoulderBreak` / `torsoBreak`, so the vehicle reacts (and ejects) before the chunks are born.
    pub fn smash_chest_with(&mut self, world: &mut World, on_break: &mut BreakHook) {
        if self.chest_gone() {
            return;
        }
        self.sound(world, Part::Chest, "ChestSmash", 1);
        world.blood_burst(5.0, 20.0, self.bodies[&Part::Chest], 300, Vec2::default());
        self.stop_flows(world, &[1, 2, 4]);
        self.die(world);
        if self.broken.contains(&Limb::Neck) {
            self.tear_chain(world, true, 0);
        }
        let ratio = world.b2().body(self.bodies[&Part::UpperArm1]).mass / 0.07019;
        let snap = (ratio * 90.0).round() + 1.0;
        let spine = (world.b2().body(self.bodies[&Part::Head]).mass / 0.05456 * 105.0).round();
        self.break_then(world, Limb::Neck, spine, on_break);
        self.break_then(world, Limb::Shoulder1, snap, on_break);
        self.break_then(world, Limb::Shoulder2, snap, on_break);
        let m = world.b2().body(self.bodies[&Part::Pelvis]).mass / 0.0735;
        self.break_then(world, Limb::Waist, (m * 260.0).round() - 1.0, on_break);
        self.tear_chain(world, false, 1);
        self.chunk_ring(world, Part::Chest, 4, std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_2, 5.0 / CHARACTER_SCALE, self.chest_chunk_radius);
        let chest = self.bodies[&Part::Chest];
        let (pos, angle) = (world.b2().body(chest).xf.position, world.b2().body(chest).sweep.a);
        let heart = Self::debris_ball(world, pos, angle, self.head_chunk_radius);
        let b = world.b2_mut();
        let (v, w) = (b.body(chest).linear_velocity, b.body(chest).angular_velocity);
        b.body_mut(heart).linear_velocity = v;
        b.body_mut(heart).angular_velocity = w;
        self.heart = Some(heart);
        b.body_mut(chest).angular_velocity = 0.0;
        b.body_mut(chest).linear_velocity = Vec2::default();
        b.destroy_body(chest);
    }

    /// Frame of a part clip (`gotoAndStop` calls of the original's break functions): limbs show 1 intact, 3 soft hip/shoulder break, 4 hard break,
    /// +elbow/knee break → 5/6/7; lower arms 2 once the elbow is gone; lower legs 2 knee / 3 foot / 4 both; head and pelvis 2 when
    /// the neck / a hard hip break happened; chest 2 after the waist; stump clips (`upperArm3/4`, `upperLeg3/4`) 2.
    pub fn part_frame(&self, name: &str) -> usize {
        let has = |l: Limb| self.broken.contains(&l);
        let sev = |l: Limb| self.sev_limbs.contains(&l);
        let limb = |joint: Limb, bend: Limb| match (has(joint), sev(joint), has(bend)) {
            (false, _, false) => 1,
            (false, _, true) => 5,
            (true, true, false) => 3,
            (true, true, true) => 6,
            (true, false, false) => 4,
            (true, false, true) => 7,
        };
        let side = usize::from(name.ends_with('2'));
        match name {
            "upperArm1" => limb(Limb::Shoulder1, Limb::Elbow1),
            "upperArm2" => limb(Limb::Shoulder2, Limb::Elbow2),
            "upperLeg1" => limb(Limb::Hip1, Limb::Knee1),
            "upperLeg2" => limb(Limb::Hip2, Limb::Knee2),
            "lowerArm1" | "lowerArm2" => if has(if side == 0 { Limb::Elbow1 } else { Limb::Elbow2 }) { 2 } else { 1 },
            "lowerLeg1" | "lowerLeg2" => {
                let knee = has(if side == 0 { Limb::Knee1 } else { Limb::Knee2 });
                match (knee, self.feet[side].is_some()) {
                    (false, false) => 1,
                    (true, false) => 2,
                    (false, true) => 3,
                    (true, true) => 4,
                }
            }
            "head" => if has(Limb::Neck) { 2 } else { 1 },
            "chest" => if has(Limb::Waist) { 2 } else { 1 },
            "pelvis" => if has(Limb::Hip1) && !sev(Limb::Hip1) || has(Limb::Hip2) && !sev(Limb::Hip2) { 2 } else { 1 },
            "upperArm3" if sev(Limb::Shoulder1) => if self.chest_gone() { 8 } else { 2 },
            "upperArm4" if sev(Limb::Shoulder2) => if self.chest_gone() { 8 } else { 2 },
            "upperLeg3" if sev(Limb::Hip1) => if self.pelvis_smashed { 8 } else { 2 },
            "upperLeg4" if sev(Limb::Hip2) => if self.pelvis_smashed { 8 } else { 2 },
            _ => 1,
        }
    }

    /// `wound.visible`: the chest's after a hard shoulder break (17739), the pelvis's after the waist tears (17652).
    pub fn part_wound(&self, name: &str) -> bool {
        match name {
            "chest" => [Limb::Shoulder1, Limb::Shoulder2].iter().any(|l| self.broken.contains(l) && !self.sev_limbs.contains(l)),
            "pelvis" => self.broken.contains(&Limb::Waist),
            _ => false,
        }
    }

    fn chest_gone(&self) -> bool {
        self.heart.is_some()
    }

    /// `pelvisSmash`: both hips and the waist are torn off, three chunk balls (ring 3 px, 120° apart), pelvis destroyed.
    pub fn smash_pelvis(&mut self, world: &mut World) {
        self.smash_pelvis_with(world, &mut |_, _, _| {});
    }

    /// [`smash_pelvis`](Self::smash_pelvis) with the vehicle's `hipBreak` / `torsoBreak` overrides, as for the chest.
    pub fn smash_pelvis_with(&mut self, world: &mut World, on_break: &mut BreakHook) {
        if !world.b2().body(self.bodies[&Part::Pelvis]).alive {
            return;
        }
        self.pelvis_smashed = true;
        self.sound(world, Part::Pelvis, "PelvisSmash", 1);
        world.blood_burst(5.0, 15.0, self.bodies[&Part::Pelvis], 200, Vec2::default());
        self.stop_flows(world, &[5]);
        let ratio = world.b2().body(self.bodies[&Part::UpperLeg1]).mass / 0.14914;
        let snap = (ratio * 110.0).round() + 1.0;
        self.break_then(world, Limb::Hip1, snap, on_break);
        self.break_then(world, Limb::Hip2, snap, on_break);
        let m = world.b2().body(self.bodies[&Part::Pelvis]).mass / 0.0735;
        if self.broken.contains(&Limb::Waist) {
            self.tear_chain(world, false, 0);
        }
        self.break_then(world, Limb::Waist, (m * 260.0).round(), on_break);
        self.chunk_ring(world, Part::Pelvis, 3, 0.0, std::f64::consts::PI * 2.0 / 3.0, 3.0 / CHARACTER_SCALE, self.pelvis_chunk_radius);
        self.vocal("Pelvis", 5);
        world.b2_mut().destroy_body(self.bodies[&Part::Pelvis]);
    }

    /// `footSmash1/2` (`leg` 0/1): a loose foot box appears at the lower leg's `footShape` point (guide numbers used as
    /// local coordinates, as in the original) with the leg's velocity; the leg itself stays.
    pub fn smash_foot(&mut self, world: &mut World, leg: usize) {
        if self.feet[leg].is_some() {
            return;
        }
        let part = if leg == 0 { Part::LowerLeg1 } else { Part::LowerLeg2 };
        self.vocal(if leg == 0 { "Foot1" } else { "Foot2" }, 0);
        let src = self.bodies[&part];
        let g = self.foot_guide;
        let b = world.b2_mut();
        let at = b.body(src).world_point(Vec2::new(g[0] / CHARACTER_SCALE, g[1] / CHARACTER_SCALE));
        let angle = b.body(src).sweep.a;
        let foot = b.create_body(&BodyDef { position: at, angle, ..Default::default() });
        let mut d = ShapeDef::boxed(g[3] * self.ref_scale / CHARACTER_SCALE, g[4] * self.ref_scale / CHARACTER_SCALE);
        d.density = 1.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        b.create_shape(foot, &d);
        b.set_mass_from_shapes(foot);
        let (v, w) = (b.body(src).linear_velocity, b.body(src).angular_velocity);
        b.body_mut(foot).linear_velocity = v;
        b.body_mut(foot).angular_velocity = w;
        self.feet[leg] = Some(foot);
    }

    /// Break limits of the original's `setLimits` (mass ratio × base constant, rounded), per limb.
    pub fn break_limit(&self, world: &World, limb: Limb) -> f64 {
        let ratio = |part: Part, def: f64| world.b2().body(self.bodies[&part]).mass / def;
        let r = |v: f64| v.round();
        match limb {
            Limb::Neck => r(ratio(Part::Head, 0.05456) * 85.0),
            Limb::Waist => r(ratio(Part::Pelvis, 0.0735) * 180.0),
            Limb::Shoulder1 | Limb::Shoulder2 => r(ratio(Part::UpperArm1, 0.07019) * 75.0),
            Limb::Elbow1 | Limb::Elbow2 => r(ratio(Part::LowerArm1, 0.07784) * 70.0),
            Limb::Hip1 | Limb::Hip2 => r(ratio(Part::UpperLeg1, 0.14914) * 95.0),
            Limb::Knee1 | Limb::Knee2 => r(ratio(Part::LowerLeg1, 0.10218) * 80.0),
        }
    }

    /// `setLimits` smash thresholds (`contactImpulseDict`): impulse above which a part is smashed
    /// (head 3×, chest 7.5×, pelvis 5.5×, feet 4× their mass ratio). `None` for parts without a limit.
    pub fn smash_limit(&self, world: &World, part: Part) -> Option<f64> {
        if self.ground.contains(&part) {
            return None;
        }
        let ratio = |p: Part, def: f64| world.b2().body(self.bodies[&p]).mass / def;
        match part {
            Part::Head => Some(3.0 * ratio(Part::Head, 0.05456)),
            Part::Chest => Some(7.5 * ratio(Part::Chest, 0.17171)),
            Part::Pelvis => Some(5.5 * ratio(Part::Pelvis, 0.0735)),
            Part::LowerLeg1 | Part::LowerLeg2 => Some(4.0 * ratio(Part::LowerLeg1, 0.10218)),
            _ => None,
        }
    }

    /// `checkJoints`/`checkRevJoint`: a joint whose reaction force exceeds its limit, or whose anchors drifted
    /// more than 0.5 m apart, breaks (is destroyed). Returns the limbs that broke this frame. The original's
    /// break callbacks (blood, ligaments, sprites, dying) are not modelled.
    pub fn check_joints(&mut self, world: &mut World) -> Vec<Limb> {
        let mut now = Vec::new();
        for limb in LIMB_ORDER {
            if self.broken.contains(&limb) {
                continue;
            }
            let Some(&id) = self.joints.get(&limb) else { continue };
            let limit = self.break_limit(world, limb);
            let Some((force, gap)) = world.b2().revolute_reaction(id) else { continue };
            if force > limit || gap > 0.5 {
                world.b2_mut().destroy_joint(id);
                self.broken.push(limb);
                now.push(limb);
                self.break_side_effects(world, limb, force.max(if gap > 0.5 { 1000.0 } else { 0.0 }));
            }
        }
        self.check_chains(world);
        self.check_ligaments(world);
        now
    }

    /// Soft shoulder/hip break (original 17756 / 17993): a stump box (`sevArm1Shape` / `sevLeg1Shape`, density 1, default filter) is born at the
    /// joint's parent anchor with the limb's angle and velocities and hinged to the torso/pelvis with the base limits; the limb itself
    /// goes zero-filter (a thigh also swaps its shape for the `sevLeg2Shape` box), its lower part zero-filter.
    fn sever(&mut self, world: &mut World, limb: Limb, upper: Part, lower: Part) {
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let shoulder = matches!(limb, Limb::Shoulder1 | Limb::Shoulder2);
        let parent = self.bodies[&if shoulder { Part::Chest } else { Part::Pelvis }];
        let src = self.bodies[&upper];
        let g = self.sev_guides[if shoulder { 0 } else { 1 }];
        let deg = 180.0 / std::f64::consts::PI;
        let (lo, hi) = if shoulder { (-180.0, 60.0) } else { (-150.0, 10.0) };
        let b = world.b2_mut();
        let box_verts = |g: [f64; 7], rs: f64| {
            let (hx, hy, cy) = (g[3] * rs / CHARACTER_SCALE, g[4] * rs / CHARACTER_SCALE, g[1] / CHARACTER_SCALE);
            vec![Vec2::new(-hx, cy - hy), Vec2::new(hx, cy - hy), Vec2::new(hx, cy + hy), Vec2::new(-hx, cy + hy)]
        };
        let id = self.joints[&limb];
        let at = match &b.joint(id).kind {
            box2d::joints::JointKind::Revolute(r) => b.body(parent).world_point(r.local_anchor1),
            _ => return,
        };
        let angle = b.body(src).sweep.a;
        let stump = b.create_body(&BodyDef { position: at, angle, ..Default::default() });
        let mut d = ShapeDef::new(Geometry::Polygon(box_verts(g, self.ref_scale)));
        d.density = 1.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: self.group };
        b.create_shape(stump, &d);
        b.set_mass_from_shapes(stump);
        let (v, w) = (b.body(src).linear_velocity, b.body(src).angular_velocity);
        b.body_mut(stump).linear_velocity = v;
        b.body_mut(stump).angular_velocity = w;
        let rel = b.body(stump).sweep.a - b.body(parent).sweep.a;
        let mut jd = RevoluteJointDef::initialize(b.body(parent), parent, b.body(stump), stump, at);
        jd.enable_limit = true;
        jd.max_motor_torque = 4.0;
        jd.lower_angle = lo / deg - rel;
        jd.upper_angle = hi / deg - rel;
        let new_joint = b.create_revolute_joint(&jd);
        self.joints.insert(limb, new_joint);
        self.sev.push(stump);
        self.sev_limbs.push(limb);
        let (us, ls) = (b.body(src).shapes[0], b.body(self.bodies[&lower]).shapes[0]);
        if shoulder {
            b.set_shape_filter(us, zero);
        } else {
            b.destroy_shape(us);
            let mut d2 = ShapeDef::new(Geometry::Polygon(box_verts(self.sev_guides[2], self.ref_scale)));
            d2.density = 1.0;
            d2.friction = 0.3;
            d2.restitution = 0.1;
            d2.filter = zero;
            b.create_shape(src, &d2);
            b.set_mass_from_shapes(src);
        }
        b.set_shape_filter(ls, zero);
    }

    /// `new A.e1(character, n, ligamentLength)` (spine, `spine == true`) / `new A.h(character, n, intestineLength)`: `n` fixed-rotation circles
    /// (radius 5 px, density 1, zero filter, the head's / chest's velocity) evenly spaced between the two end anchors, chained by rigid
    /// distance joints of the guide's link length. Spine: chest `spineAnchor` → head top; intestines: pelvis edge → chest edge.
    fn make_chain(&self, world: &mut World, spine: bool, n: usize) -> Option<Chain> {
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let b = world.b2_mut();
        let (end_a, end_b, vel_src, ba, bb) = if spine {
            let (chest, head) = (self.bodies[&Part::Chest], self.bodies[&Part::Head]);
            let r = match &b.c.shapes[*b.body(head).shapes.first()?].shape.kind {
                box2d::shape::ShapeKind::Circle(c) => c.radius,
                box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices[0].y,
            };
            let a = b.body(chest).world_point(Vec2::new(self.spine_anchor[0] / CHARACTER_SCALE, self.spine_anchor[1] / CHARACTER_SCALE));
            (a, b.body(head).world_point(Vec2::new(0.0, r)), head, chest, head)
        } else {
            let (pelvis, chest) = (self.bodies[&Part::Pelvis], self.bodies[&Part::Chest]);
            let vy = |id: BodyId, i: usize| match &b.c.shapes[b.body(id).shapes[0]].shape.kind {
                box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices[i].y,
                box2d::shape::ShapeKind::Circle(c) => c.radius,
            };
            let at = |b: &box2d::world::World, id: BodyId, y: f64| {
                let ang = b.body(id).sweep.a + std::f64::consts::FRAC_PI_2;
                let p = b.body(id).xf.position;
                Vec2::new(p.x + box2d::fdlibm::cos(ang) * y, p.y + box2d::fdlibm::sin(ang) * y)
            };
            let (ya, yb) = (vy(pelvis, 0), vy(chest, 2));
            (at(b, pelvis, ya), at(b, chest, yb), chest, pelvis, chest)
        };
        let len = self.chain_len[usize::from(spine)];
        let (dx, dy) = ((end_b.x - end_a.x) / (n + 1) as f64, (end_b.y - end_a.y) / (n + 1) as f64);
        let v = b.body(vel_src).linear_velocity;
        let mut bodies = Vec::new();
        for k in 1..=n {
            let at = Vec2::new(end_a.x + dx * k as f64, end_a.y + dy * k as f64);
            let body = b.create_body(&BodyDef { position: at, fixed_rotation: true, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 5.0 / CHARACTER_SCALE });
            d.density = 1.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = zero;
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            b.body_mut(body).linear_velocity = v;
            bodies.push(body);
        }
        let link = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p1: Vec2, p2: Vec2| {
            let mut d = DistanceJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p1, p2);
            d.length = len;
            d.collide_connected = false;
            Some(b.create_distance_joint(&d))
        };
        let mut joints = vec![];
        let first = b.body(bodies[0]).xf.position;
        joints.push(link(b, ba, bodies[0], end_a, first));
        for w in bodies.windows(2) {
            let (p1, p2) = (b.body(w[0]).xf.position, b.body(w[1]).xf.position);
            joints.push(link(b, w[0], w[1], p1, p2));
        }
        let last = *bodies.last()?;
        let lp = b.body(last).xf.position;
        joints.push(link(b, last, bb, lp, end_b));
        Some(Chain { bodies, joints, torn: false })
    }

    /// `new A.i$(upper, lower, ligamentLength, 125, character)`: two fixed-rotation balls (radius 7 px, density 1, filter 2056/2056) strung on three
    /// rigid distance joints between the upper body's far edge (vertex[2].y) and the lower body's near edge (vertex[0].y). The original's
    /// `iw = iq.y - iq.y` bug (zero y step, so the balls lie on the upper edge's height) is reproduced.
    fn make_ligament(&self, world: &mut World, upper: Part, lower: Part) -> Option<Chain> {
        let b = world.b2_mut();
        let (ub, lb) = (self.bodies[&upper], self.bodies[&lower]);
        let vy = |b: &box2d::world::World, id: BodyId, i: usize| match &b.c.shapes[b.body(id).shapes[0]].shape.kind {
            box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices[i].y,
            box2d::shape::ShapeKind::Circle(c) => c.radius,
        };
        let edge = |b: &box2d::world::World, id: BodyId, y: f64| {
            let ang = b.body(id).sweep.a + std::f64::consts::FRAC_PI_2;
            let p = b.body(id).xf.position;
            Vec2::new(p.x + box2d::fdlibm::cos(ang) * y, p.y + box2d::fdlibm::sin(ang) * y)
        };
        let (ya, yb) = (vy(b, ub, 2), vy(b, lb, 0));
        let (a, c) = (edge(b, ub, ya), edge(b, lb, yb));
        let dx = (c.x - a.x) / 3.0;
        let v = b.body(ub).linear_velocity;
        let mut bodies = Vec::new();
        for k in 1..=2 {
            let body = b.create_body(&BodyDef { position: Vec2::new(a.x + dx * k as f64, a.y), fixed_rotation: true, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 7.0 / CHARACTER_SCALE });
            d.density = 1.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 2056, mask_bits: 2056, group_index: 0 };
            b.create_shape(body, &d);
            b.set_mass_from_shapes(body);
            b.body_mut(body).linear_velocity = v;
            bodies.push(body);
        }
        let len = self.chain_len[1];
        let link = |b: &mut box2d::world::World, b1: BodyId, b2: BodyId, p1: Vec2, p2: Vec2| {
            let mut d = DistanceJointDef::initialize(b.body(b1), b1, b.body(b2), b2, p1, p2);
            d.length = len;
            d.collide_connected = false;
            Some(b.create_distance_joint(&d))
        };
        let (p0, p1) = (b.body(bodies[0]).xf.position, b.body(bodies[1]).xf.position);
        let joints = vec![link(b, ub, bodies[0], a, p0), link(b, bodies[0], bodies[1], p0, p1), link(b, bodies[1], lb, p1, c)];
        Some(Chain { bodies, joints, torn: false })
    }

    /// `i$.checkJoints`/`breakJoint`: a ligament whose main joint takes an impulse above 0.5 tears off the upper body: a small ball (3 px,
    /// filter 2056/2056, the upper body's velocity) at the old anchor takes over as the joint's first body. The original rewires
    /// the joint in place (`m_body1`), keeping its impulse, its solver position and its island edges.
    fn check_ligaments(&mut self, world: &mut World) {
        let parts = [Part::UpperArm1, Part::UpperArm2, Part::UpperLeg1, Part::UpperLeg2];
        for (slot, up) in parts.into_iter().enumerate() {
            let Some(chain) = self.ligaments[slot].as_mut() else { continue };
            let Some(main) = chain.joints[0] else { continue };
            if chain.torn || !world.b2().distance_impulse(main).is_some_and(|i| i.abs() > 0.5) {
                continue;
            }
            let b = world.b2_mut();
            let Some((anchor, _)) = b.distance_anchor1(main) else { continue };
            let upper = self.bodies[&up];
            let ball = b.create_body(&BodyDef { position: anchor, fixed_rotation: true, ..Default::default() });
            let mut d = ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: 3.0 / CHARACTER_SCALE });
            d.density = 1.0;
            d.friction = 0.3;
            d.restitution = 0.1;
            d.filter = Filter { category_bits: 2056, mask_bits: 2056, group_index: 0 };
            b.create_shape(ball, &d);
            b.set_mass_from_shapes(ball);
            let v = b.body(upper).linear_velocity;
            b.body_mut(ball).linear_velocity = v;
            b.rewire_distance_joint(main, ball);
            chain.bodies.push(ball);
            chain.torn = true;
        }
    }

    /// Where `paint` puts the clips `intestine1..10` / `spine1..10`: (clip number from 0, a ball whose existence the clip depends on,
    /// position in m, rotation, length over rest length). Intestine segments stretch along their links; vertebrae only turn.
    /// An intestine end that has torn is hidden.
    pub(crate) fn chain_clips(&self, world: &World, spine: bool) -> Vec<(usize, BodyId, Vec2, f64, f64)> {
        let Some(chain) = (if spine { self.spine.as_ref() } else { self.intestines.as_ref() }) else { return Vec::new() };
        let n = chain.bodies.len();
        if n == 0 || chain.joints.len() != n + 1 {
            return Vec::new();
        }
        let at = |k: usize| world.b2().body(chain.bodies[k]).xf.position;
        let anchor = |k: usize, second: bool| chain.joints[k].and_then(|j| crate::vehicle::joint_anchor(world, j, second));
        let towards = |from: Vec2, to: Vec2| (from.y - to.y).atan2(from.x - to.x);
        let length = self.chain_len[usize::from(spine)];
        let stretch = |a: Vec2, b: Vec2| ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt() / length;
        let mut clips = Vec::new();
        if spine {
            for k in 0..n - 1 {
                clips.push((k, chain.bodies[k], at(k), towards(at(k), at(k + 1)), 1.0));
            }
            let turn = match anchor(n, true) {
                Some(head) => towards(at(n - 1), head),
                None => clips.last().map_or(0.0, |c| c.3),
            };
            clips.push((n - 1, chain.bodies[n - 1], at(n - 1), turn, 1.0));
        } else {
            if let Some(pelvis) = anchor(0, false) {
                clips.push((0, chain.bodies[0], pelvis, towards(pelvis, at(0)), stretch(pelvis, at(0))));
            }
            for k in 0..n - 1 {
                clips.push((k + 1, chain.bodies[k], at(k), towards(at(k), at(k + 1)), stretch(at(k), at(k + 1))));
            }
            if let Some(chest) = anchor(n, true) {
                // the last of the ten clips takes the chest end
                clips.push((9, chain.bodies[n - 1], at(n - 1), towards(at(n - 1), chest), stretch(at(n - 1), chest)));
            }
        }
        clips
    }

    /// The red lines of the spinal cord and of the elbow/knee ligaments; `prefix` is the clip a sub-character's parts live in.
    pub(crate) fn gore_strokes(&self, world: &World, prefix: Option<&str>) -> Vec<crate::vehicle::Stroke> {
        let layer = |name: &str| std::borrow::Cow::Owned(prefix.map_or_else(|| name.to_string(), |p| format!("{p}/{name}")));
        let px = |p: Vec2| [p.x * 62.5, p.y * 62.5];
        let anchor = |j: Option<usize>, second: bool| j.and_then(|j| crate::vehicle::joint_anchor(world, j, second));
        let at = |id: BodyId| world.b2().body(id).xf.position;
        let mut lines = Vec::new();
        if let Some(chain) = &self.spine {
            let mut points: Vec<_> = anchor(chain.joints[0], false).into_iter().chain(chain.bodies.iter().map(|&b| at(b))).map(px).collect();
            points.extend(anchor(chain.joints[chain.joints.len() - 1], true).map(px));
            lines.push(crate::vehicle::Stroke { points, width: 1.0, color: 0xff0000, layer: layer("spine1"), above: false });
        }
        for (slot, (upper, lower)) in [("upperArm1", Part::LowerArm1), ("upperArm2", Part::LowerArm2), ("upperLeg1", Part::LowerLeg1), ("upperLeg2", Part::LowerLeg2)].into_iter().enumerate() {
            let Some(chain) = &self.ligaments[slot] else { continue };
            // `bodies` may hold a third ball (the torn end); the three links are what is drawn
            let [first, middle, last] = chain.joints[..] else { continue };
            let Some(start) = anchor(first, false) else { continue };
            let mut points = vec![px(start)];
            points.extend([first, middle].into_iter().flatten().map(|j| px(at(world.b2().joint(j).body2))));
            points.extend(anchor(last, true).map(px));
            points.push(px(world.b2().body(self.bodies[&lower]).sweep.c));
            lines.push(crate::vehicle::Stroke { points, width: 2.0, color: 0xff0000, layer: layer(upper), above: false });
        }
        lines
    }

    /// `checkJoints` of both chains: an end joint whose |impulse| exceeds 1.5 tears.
    pub fn check_chains(&mut self, world: &mut World) {
        let mut torn = Vec::new();
        for (spine, chain) in [(false, self.intestines.as_mut()), (true, self.spine.as_mut())] {
            let Some(chain) = chain else { continue };
            let last = chain.joints.len() - 1;
            for k in [0, last] {
                if let Some(j) = chain.joints[k] {
                    if world.b2().distance_impulse(j).is_some_and(|i| i.abs() > 1.5) {
                        world.b2_mut().destroy_joint(j);
                        chain.joints[k] = None;
                        torn.push((spine, k == 0));
                    }
                }
            }
        }
        for (spine, first) in torn {
            self.tear_sound(world, spine, first);
        }
    }

    /// `LigTear1` of `spineBreak1` (chest) / `intestineBreak1` (pelvis) / `intestineBreak2` (chest); `spineBreak2` is silent.
    fn tear_sound(&self, world: &mut World, spine: bool, first: bool) {
        match (spine, first) {
            (true, true) | (false, false) => self.sound(world, Part::Chest, "LigTear1", 1),
            (false, true) => self.sound(world, Part::Pelvis, "LigTear1", 1),
            (true, false) => {}
        }
    }

    /// `spineBreak1/2`, `intestineBreak1/2`: tear one end of a chain explicitly (end 0 = first joint, 1 = last).
    fn tear_chain(&mut self, world: &mut World, spine: bool, end: usize) {
        let Some(chain) = (if spine { self.spine.as_mut() } else { self.intestines.as_mut() }) else { return };
        let k = if end == 0 { 0 } else { chain.joints.len() - 1 };
        if let Some(j) = chain.joints[k].take() {
            world.b2_mut().destroy_joint(j);
            self.tear_sound(world, spine, end == 0);
        }
    }

    fn refilter_part(&self, world: &mut World, part: Part, f: Filter) {
        let b = world.b2_mut();
        if let Some(&s) = b.body(self.bodies[&part]).shapes.first() {
            b.set_shape_filter(s, f);
        }
    }

    /// Physical side effects of `neckBreak/torsoBreak/shoulderBreak/elbowBreak/hipBreak/kneeBreak` (filters only;
    /// blood, sprites, severed-limb bodies and ligament chains are not modelled). `force` is the break value.
    /// Queues a one-shot sound at `part`'s body (`playAreaSoundInstance(name, body)`).
    fn sound(&self, world: &mut World, part: Part, name: &str, variants: u32) {
        let at = world.b2().body(self.bodies[&part]).sweep.c;
        world.play(name, variants, at);
    }

    /// `shapeImpale(shape, fatal, point, radius²)`: chest/pelvis impaled → a random vocal (name picked by the last digit of the body's x position);
    /// head: a fatal impale (no point given, or within `radius²` of the head's centre) ejects the rider and kills it, otherwise just a vocal.
    /// Returns `true` when the rider has to be ejected from its vehicle.
    pub fn shape_impale(&mut self, world: &mut World, body: BodyId, fatal: bool, point: Option<Vec2>, radius2: f64) -> bool {
        let Some((&part, _)) = self.bodies.iter().find(|(_, &b)| b == body) else { return false };
        let vocal = |s: &mut Self, world: &World| {
            let x = format!("{}", world.b2().body(body).xf.position.x);
            let digit = x.chars().last().and_then(|c| c.to_digit(10)).unwrap_or(0) as usize;
            s.vocal(["Elbow1", "Elbow2", "Knee1", "Knee2", "Shoulder1", "Shoulder2", "Hip1", "Hip2", "Knee1", "Spikes"][digit], 5);
        };
        match part {
            Part::Chest | Part::Pelvis => vocal(self, world),
            Part::Head => {
                let center = world.b2().body(body).sweep.c;
                let lethal = fatal && point.is_none_or(|p| (p.x - center.x).powi(2) + (p.y - center.y).powi(2) < radius2);
                if lethal {
                    self.die(world);
                    return true;
                }
                vocal(self, world);
            }
            _ => {}
        }
        false
    }

    /// `addVocals(name, priority)`.
    pub fn vocal(&mut self, name: &'static str, priority: usize) {
        self.voices[priority] = Some(name);
    }

    /// `checkVocals` (once per frame): the highest-priority queued vocal plays from the head (`tag + name`); while one is sounding only a higher
    /// priority interrupts it (the old one keeps playing here, the mixer cannot cut a voice). The queue is cleared afterwards.
    pub fn check_vocals(&mut self, world: &mut World) {
        let top = self.voices.iter().rposition(Option::is_some).map(|i| i as i32).unwrap_or(-1);
        let busy = world.frame() < self.voice_until;
        if top >= 0 && (!busy || top > self.voice_prio) {
            let name = format!("{}{}", self.tag, self.voices[top as usize].unwrap());
            let at = world.b2().body(self.bodies[&Part::Head]).sweep.c;
            self.voice_until = world.frame() + crate::physics::sound_frames(&name);
            self.voice_prio = top;
            world.play(&name, 1, at);
        } else if !busy {
            self.voice_prio = -1;
        }
        self.voices = [None; 6];
    }

    /// `grindShape(shape)`: head / chest / pelvis go to `GRIND_STATE` (no more smash reactions) and the blood flows that feed from them stop
    /// (head: head flow; chest: neck, shoulders, stomach; pelvis: hips). Limb shapes are not tracked (their stump flows keep running).
    pub fn grind_shape(&mut self, world: &mut World, body: BodyId) {
        let Some((&part, _)) = self.bodies.iter().find(|(_, &b)| b == body) else { return };
        let kinds: &[u8] = match part {
            Part::Head => &[0],
            Part::Chest => &[1, 2, 4],
            Part::Pelvis => &[5],
            _ => return,
        };
        if !self.ground.contains(&part) {
            self.ground.push(part);
        }
        self.stop_flows(world, kinds);
    }

    fn stop_flows(&mut self, world: &mut World, kinds: &[u8]) {
        for &(k, f) in &self.blood_flows {
            if kinds.contains(&k) {
                world.blood_stop(f);
            }
        }
        self.blood_flows.retain(|(k, _)| !kinds.contains(k));
    }

    /// The `*BloodFlow` emitters the original creates in `neckBreak`/`shoulderBreak`/`hipBreak`/`torsoBreak` (angles in degrees, local to the body).
    fn break_blood(&mut self, world: &mut World, limb: Limb) {
        let anchors = |world: &World, limb: Limb| match &world.b2().joint(self.joints[&limb]).kind {
            box2d::joints::JointKind::Revolute(r) => (r.local_anchor1, r.local_anchor2),
            _ => (Vec2::default(), Vec2::default()),
        };
        let zero = Vec2::default();
        let body = |p: Part| self.bodies[&p];
        let mut made = Vec::new();
        match limb {
            Limb::Neck => made.push((0u8, world.blood_flow(2.5, 4.0, body(Part::Head), zero, 90.0, 150))),
            Limb::Waist => made.push((4, world.blood_flow(2.0, 3.0, body(Part::Chest), zero, 90.0, 500))),
            Limb::Shoulder1 | Limb::Shoulder2 => {
                let (a1, a2) = anchors(world, limb);
                let arm = if limb == Limb::Shoulder1 { Part::UpperArm1 } else { Part::UpperArm2 };
                made.push((3, world.blood_flow(1.0, 3.0, body(arm), a2, 270.0, 200)));
                made.push((2, world.blood_flow(0.0, 1.0, body(Part::Chest), a1, 270.0, 500)));
            }
            Limb::Hip1 | Limb::Hip2 => {
                let (_, a2) = anchors(world, limb);
                let leg = if limb == Limb::Hip1 { Part::UpperLeg1 } else { Part::UpperLeg2 };
                made.push((3, world.blood_flow(1.0, 3.0, body(leg), a2, 270.0, 200)));
                made.push((5, world.blood_flow(0.0, 1.0, body(Part::Pelvis), zero, 90.0, 500)));
            }
            _ => {}
        }
        self.blood_flows.extend(made);
    }

    fn break_side_effects(&mut self, world: &mut World, limb: Limb, force: f64) {
        // `neckBreak` sets `dead`
        if limb == Limb::Neck {
            self.die(world);
        }
        self.break_blood(world, limb);
        match limb {
            Limb::Elbow1 => self.vocal("Elbow1", 1),
            Limb::Elbow2 => self.vocal("Elbow2", 1),
            Limb::Knee1 => self.vocal("Knee1", 2),
            Limb::Knee2 => self.vocal("Knee2", 2),
            Limb::Shoulder1 => self.vocal("Shoulder1", 3),
            Limb::Shoulder2 => self.vocal("Shoulder2", 3),
            Limb::Hip1 => self.vocal("Hip1", 4),
            Limb::Hip2 => self.vocal("Hip2", 4),
            Limb::Waist => self.vocal("Torso", 5),
            Limb::Neck => {}
        }
        match limb {
            Limb::Neck => self.sound(world, Part::Head, "NeckBreak", 1),
            Limb::Elbow1 | Limb::Elbow2 => self.sound(world, if limb == Limb::Elbow1 { Part::LowerArm1 } else { Part::LowerArm2 }, "BoneBreak", 4),
            Limb::Knee1 | Limb::Knee2 => self.sound(world, if limb == Limb::Knee1 { Part::UpperLeg1 } else { Part::UpperLeg2 }, "BoneBreak", 4),
            Limb::Shoulder1 => self.sound(world, Part::UpperArm1, "LimbRip2", 1),
            Limb::Shoulder2 => self.sound(world, Part::UpperArm2, "LimbRip2", 1),
            Limb::Hip1 => self.sound(world, Part::UpperLeg1, "LimbRip3", 1),
            Limb::Hip2 => self.sound(world, Part::UpperLeg2, "LimbRip4", 1),
            Limb::Waist => self.sound(world, Part::Pelvis, "LimbRip1", 1),
        }
        let zero = Filter { category_bits: 260, mask_bits: 0xFFFF, group_index: 0 };
        let lower = Filter { category_bits: 260, mask_bits: 270, group_index: self.group - 5 }; // groupID - 5
        let masses: HashMap<Part, f64> = self.bodies.iter().map(|(&p, &id)| (p, world.b2().body(id).mass)).collect();
        let ratio = move |p: Part, def: f64| masses[&p] / def;
        match limb {
            Limb::Neck => {
                self.refilter_part(world, Part::Head, zero);
                let (neck, spine) = ((ratio(Part::Head, 0.05456) * 85.0).round(), (ratio(Part::Head, 0.05456) * 105.0).round());
                if force < spine && self.spine.is_none() {
                    let n = ((force - neck) / ((spine - neck) / 9.0)).ceil() as usize + 1;
                    self.spine = self.make_chain(world, true, n);
                }
            }
            Limb::Waist => {
                for p in [Part::Pelvis, Part::UpperLeg1, Part::UpperLeg2, Part::LowerLeg1, Part::LowerLeg2] {
                    self.refilter_part(world, p, lower);
                }
                let (torso, gut) = ((ratio(Part::Pelvis, 0.0735) * 180.0).round(), (ratio(Part::Pelvis, 0.0735) * 260.0).round());
                if force < gut && self.intestines.is_none() {
                    let n = ((force - torso) / ((gut - torso) / 8.0)).ceil() as usize + 1;
                    self.intestines = self.make_chain(world, false, n);
                }
            }
            Limb::Shoulder1 | Limb::Shoulder2 | Limb::Hip1 | Limb::Hip2 => {
                let shoulder = matches!(limb, Limb::Shoulder1 | Limb::Shoulder2);
                let first = matches!(limb, Limb::Shoulder1 | Limb::Hip1);
                let (u, l) = match (shoulder, first) {
                    (true, true) => (Part::UpperArm1, Part::LowerArm1),
                    (true, false) => (Part::UpperArm2, Part::LowerArm2),
                    (false, true) => (Part::UpperLeg1, Part::LowerLeg1),
                    (false, false) => (Part::UpperLeg2, Part::LowerLeg2),
                };
                let snap = if shoulder { (ratio(Part::UpperArm1, 0.07019) * 90.0).round() } else { (ratio(Part::UpperLeg1, 0.14914) * 110.0).round() };
                if force > snap {
                    self.refilter_part(world, u, zero);
                    self.refilter_part(world, l, zero);
                } else {
                    self.sever(world, limb, u, l);
                }
            }
            Limb::Elbow1 | Limb::Elbow2 | Limb::Knee1 | Limb::Knee2 => {
                let (slot, up, low, limit) = match limb {
                    Limb::Elbow1 => (0, Part::UpperArm1, Part::LowerArm1, (ratio(Part::LowerArm1, 0.07784) * 80.0).round()),
                    Limb::Elbow2 => (1, Part::UpperArm2, Part::LowerArm2, (ratio(Part::LowerArm1, 0.07784) * 80.0).round()),
                    Limb::Knee1 => (2, Part::UpperLeg1, Part::LowerLeg1, (ratio(Part::LowerLeg1, 0.10218) * 95.0).round()),
                    _ => (3, Part::UpperLeg2, Part::LowerLeg2, (ratio(Part::LowerLeg1, 0.10218) * 95.0).round()),
                };
                let joint = self.joints[&limb];
                let anchor2 = match &world.b2().joint(joint).kind {
                    box2d::joints::JointKind::Revolute(r) => r.local_anchor2,
                    _ => Vec2::default(),
                };
                world.blood_burst(5.0, 15.0, self.bodies[&low], 50, anchor2);
                self.refilter_part(world, low, zero);
                if force < limit && self.ligaments[slot].is_none() {
                    self.ligaments[slot] = self.make_ligament(world, up, low);
                }
            }
        }
    }

    /// `resetJointLimits`: restores the base joint limits (relative to each joint's reference angle).
    pub fn reset_joint_limits(&self, world: &mut World) {
        let deg = 180.0 / std::f64::consts::PI;
        let b = world.b2_mut();
        for (k, (_, _, _, lo, hi)) in JOINTS.iter().enumerate() {
            let id = self.joints[&LIMB_ORDER[k]];
            if self.broken.contains(&LIMB_ORDER[k]) {
                continue;
            }
            if let box2d::joints::JointKind::Revolute(r) = &mut b.joint_mut(id).kind {
                r.lower_angle = lo / deg - r.reference_angle;
                r.upper_angle = hi / deg - r.reference_angle;
            }
        }
    }

    /// Sets the limits of a revolute limb (radians, relative to the joint's reference angle).
    pub fn set_limits(&self, world: &mut World, limb: Limb, lo: f64, hi: f64) {
        let id = self.joints[&limb];
        if let box2d::joints::JointKind::Revolute(r) = &mut world.b2_mut().joint_mut(id).kind {
            r.lower_angle = lo - r.reference_angle;
            r.upper_angle = hi - r.reference_angle;
        }
    }

    /// `setJoint(joint, offset, gain)` for an arbitrary revolute joint id (e.g. a vehicle's).
    pub fn set_joint_raw(&self, world: &mut World, id: usize, offset: f64, gain: f64) {
        let b = world.b2_mut();
        let angle = b.revolute_angle(id).unwrap_or(0.0);
        if let box2d::joints::JointKind::Revolute(r) = &mut b.joint_mut(id).kind {
            r.enable_motor = true;
            let err = angle - (r.lower_angle + offset);
            let speed = (gain * (err * err)).min(10.0);
            r.motor_speed = if err < 0.0 { speed } else { -speed };
        }
    }

    /// `currentPose` setter: switching to 0 schedules `cancelPose`.
    pub fn set_pose(&mut self, p: u8) {
        // `checkKeyStates` of a dead character only records the key string: no key can pose it any more
        if p != self.pose && !(self.dead && p != 0) {
            self.pose = p;
            // original: setting the pose to 0 runs `cancelPose` at once, i.e. before whatever pose is set next in the same frame
            if p == 0 {
                self.cancel = true;
            }
        }
    }

    /// `setJoint` for several joints: `(limb, offset, gain)`.
    pub fn set_joints(&self, world: &mut World, table: &[(Limb, f64, f64)]) {
        let b = world.b2_mut();
        for &(limb, offset, gain) in table {
            let Some(&id) = self.joints.get(&limb) else { continue };
            let angle = b.revolute_angle(id).unwrap_or(0.0);
            if let box2d::joints::JointKind::Revolute(r) = &mut b.joint_mut(id).kind {
                r.enable_motor = true;
                let err = angle - (r.lower_angle + offset);
                let speed = (gain * (err * err)).min(10.0);
                r.motor_speed = if err < 0.0 { speed } else { -speed };
            }
        }
    }

    /// Space key: `startGrab` (arms start looking for something to hold) / `releaseGrip` (joints destroyed).
    pub fn set_grab(&mut self, world: &mut World, pressed: bool) {
        if pressed && self.dead {
            return;
        }
        if pressed {
            self.grabbing = true;
            self.hands_open = false;
        } else if self.grabbing {
            self.grabbing = false;
            self.hands_open = true;
            for g in &mut self.grips {
                if let Some(id) = g.take() {
                    world.b2_mut().destroy_joint(id);
                }
            }
        }
    }

    /// The `grabAction` part of the frame: a hand that is pressed against something (contact with a normal
    /// impulse above 1e-8, the original's threshold for the lower-arm shapes) gets a revolute grip joint to
    /// the other body at the hand's tip (locked, limits 0..0, unless the other body is static).
    pub fn update_grab(&mut self, world: &mut World) {
        if !self.grabbing {
            return;
        }
        for (k, (part, shoulder, elbow)) in [(Part::LowerArm1, Limb::Shoulder1, Limb::Elbow1), (Part::LowerArm2, Limb::Shoulder2, Limb::Elbow2)].into_iter().enumerate() {
            if self.grips[k].is_some() || self.broken.contains(&shoulder) || self.broken.contains(&elbow) {
                continue;
            }
            let arm = self.bodies[&part];
            let b = world.b2();
            let mut hit = None;
            'contacts: for &cid in b.body(arm).contacts.iter().rev() {
                let c = b.c.contact_info(cid);
                if c.manifold_count == 0 {
                    continue;
                }
                let (s1, s2) = (&b.c.shapes[c.shape1], &b.c.shapes[c.shape2]);
                if s1.shape.is_sensor || s2.shape.is_sensor {
                    continue;
                }
                let other = if s1.body == arm { s2.body } else { s1.body };
                if other == arm {
                    continue;
                }
                for p in c.manifold.points.iter().take(c.manifold.point_count) {
                    if p.normal_impulse > 1e-8 {
                        hit = Some(other);
                        break 'contacts;
                    }
                }
            }
            let Some(other) = hit else { continue };
            let tip = {
                let body = b.body(arm);
                let hy = match &b.c.shapes[body.shapes[0]].shape.kind {
                    box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices[2].y,
                    box2d::shape::ShapeKind::Circle(c) => c.radius,
                };
                body.world_point(Vec2::new(0.0, hy))
            };
            let mut d = RevoluteJointDef::initialize(b.body(other), other, b.body(arm), arm, tip);
            if !b.body(other).is_static {
                d.enable_limit = true;
            }
            d.max_motor_torque = 4.0;
            self.grips[k] = Some(world.b2_mut().create_revolute_joint(&d));
        }
    }

    /// Ejected-character key handling of the original (`leftPressedActions` … `upAndDownActions`): left = arch,
    /// right = pushup, up = superman, down = tuck; releasing the pair cancels poses 1/2 (left/right) or 3/4
    /// (up/down). Returns the new pose.
    pub fn keys(&mut self, left: bool, right: bool, up: bool, down: bool) -> u8 {
        if self.dead {
            return self.pose;
        }
        let set = |this: &mut Self, p: u8| {
            if p != this.pose {
                this.pose = p;
                this.cancel = p == 0;
            }
        };
        match (left, right) {
            (true, false) => set(self, 1),
            (false, true) => set(self, 2),
            _ if self.pose == 1 || self.pose == 2 => set(self, 0),
            _ => {}
        }
        match (up, down) {
            (true, false) => set(self, 3),
            (false, true) => set(self, 4),
            _ if self.pose == 3 || self.pose == 4 => set(self, 0),
            _ => {}
        }
        self.pose
    }

    /// `checkPose` / `cancelPose`: drives the joint motors towards the current pose (`setJoint`:
    /// speed = ±min(gain·err², 10) with err = jointAngle − (lowerAngle + offset)).
    pub fn apply_pose(&mut self, world: &mut World) {
        let b = world.b2_mut();
        if self.pose != 0 && std::mem::take(&mut self.cancel) {
            for &id in self.joints.values() {
                if let box2d::joints::JointKind::Revolute(r) = &mut b.joint_mut(id).kind {
                    r.enable_motor = false;
                }
            }
        }
        if self.pose == 0 {
            if !std::mem::take(&mut self.cancel) {
                return;
            }
            for &id in self.joints.values() {
                if let box2d::joints::JointKind::Revolute(r) = &mut b.joint_mut(id).kind {
                    r.enable_motor = false;
                }
            }
            return;
        }
        if self.pose == 12 {
            for (&limb, &id) in &self.joints {
                if limb == Limb::Waist {
                    continue;
                }
                if let box2d::joints::JointKind::Revolute(r) = &mut b.joint_mut(id).kind {
                    r.enable_motor = true;
                    r.motor_speed = 0.0;
                }
            }
            return;
        }
        for &(limb, offset, gain) in pose_table(self.poses, self.pose) {
            let Some(&id) = self.joints.get(&limb) else { continue };
            let angle = b.revolute_angle(id).unwrap_or(0.0);
            if let box2d::joints::JointKind::Revolute(r) = &mut b.joint_mut(id).kind {
                r.enable_motor = true;
                let err = angle - (r.lower_angle + offset);
                let speed = (gain * (err * err)).min(10.0);
                r.motor_speed = if err < 0.0 { speed } else { -speed };
            }
        }
    }

    pub fn position_px(&self, world: &World, part: Part) -> Option<[f32; 2]> {
        world.body_position_px(*self.bodies.get(&part)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::level::parse;

    #[test]
    fn guides_exist_for_all_characters() {
        for c in 1..=11 {
            assert!(guide(c).is_some(), "c{c}");
        }
        assert!(guide(1).unwrap().contains_key("chestShape"));
    }

    #[test]
    fn limbs_of_one_ragdoll_do_not_collide_with_each_other() {
        let level = parse("<levelXML><info v='1.99'/></levelXML>").unwrap();
        let mut w = World::from_level(&level);
        let doll = Ragdoll::spawn(&mut w, &guide(1).unwrap(), [0.0, 0.0]).unwrap();
        for _ in 0..30 {
            w.step();
        }
        let handles: Vec<_> = doll.bodies.values().copied().collect();
        let touching = w.contact_count_between(&handles);
        assert_eq!(touching, 0, "limbs generate contact solver pairs");
    }

    #[test]
    fn ragdoll_falls_stays_connected_and_rests_on_floor() {
        let level = parse("<levelXML><info v='1.99' x='0' y='0'/><shapes>\
            <sh t='0' p0='0' p1='400' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>").unwrap();
        let mut w = World::from_level(&level);
        let doll = Ragdoll::spawn(&mut w, &guide(1).unwrap(), [0.0, 0.0]).unwrap();
        for _ in 0..240 {
            w.step();
        }
        let head = doll.position_px(&w, Part::Head).unwrap();
        let pelvis = doll.position_px(&w, Part::Pelvis).unwrap();
        let chest = doll.position_px(&w, Part::Chest).unwrap();
        for p in [head, pelvis, chest] {
            assert!(p[1] < 400.0 && p[1] > 250.0, "outside floor band: {p:?}");
        }
        let d = ((head[0] - chest[0]).powi(2) + (head[1] - chest[1]).powi(2)).sqrt();
        assert!(d < 120.0, "head detached from chest: {d}");
    }

    #[test]
    fn tuck_pose_bends_the_knees_and_cancelling_frees_the_motors() {
        let level = parse("<levelXML><info v='1.99' x='600' y='0'/></levelXML>").unwrap();
        let knee = |w: &World, d: &Ragdoll| w.b2().revolute_angle(d.joints[&Limb::Knee1]).unwrap();
        // Free fall without a pose: knees stay at their initial (straight) angle.
        let mut free = World::from_level(&level);
        free.spawn_character(1, [600.0, 0.0]);
        for _ in 0..20 {
            free.step();
        }
        let straight = knee(&free, free.character.as_ref().unwrap());
        // Tuck (down key): motors drive the knees towards lower + 2 rad (clamped by the joint limits).
        let mut w = World::from_level(&level);
        w.spawn_character(1, [600.0, 0.0]);
        for _ in 0..20 {
            w.set_character_keys(false, false, false, true);
            w.step();
        }
        let tucked = knee(&w, w.character.as_ref().unwrap());
        assert!(tucked > straight + 0.5, "knee {tucked} vs {straight}");
        assert_eq!(w.character.as_ref().unwrap().pose, 4);
        w.set_character_keys(false, false, false, false);
        assert_eq!(w.character.as_ref().unwrap().pose, 0);
    }

    #[test]
    fn soft_hip_and_shoulder_breaks_leave_hinged_stumps() {
        let level = parse("<levelXML><info v='1.99' x='600' y='0'/></levelXML>").unwrap();
        let mut w = World::from_level(&level);
        w.spawn_character(1, [600.0, 0.0]);
        let mut d = w.character.take().unwrap();
        for limb in [Limb::Hip1, Limb::Shoulder2] {
            let id = d.joints[&limb];
            w.b2_mut().destroy_joint(id);
            d.break_side_effects(&mut w, limb, 1.0);
        }
        assert_eq!(d.sev.len(), 2);
        let mass_before = w.b2().body(d.bodies[&Part::UpperLeg1]).mass;
        assert!(mass_before > 0.0);
        for _ in 0..60 {
            w.step();
        }
        for s in &d.sev {
            let p = w.b2().body(*s).xf.position;
            assert!(p.x.is_finite() && p.y.is_finite());
        }
        // a hard break (above the snap limit) leaves no stump
        let n = d.sev.len();
        let id = d.joints[&Limb::Hip2];
        w.b2_mut().destroy_joint(id);
        d.break_side_effects(&mut w, Limb::Hip2, 10000.0);
        assert_eq!(d.sev.len(), n);
    }

    #[test]
    fn soft_waist_and_neck_breaks_expose_chains_and_smashes_tear_them() {
        let level = parse("<levelXML><info v='1.99' x='600' y='0'/></levelXML>").unwrap();
        let mut w = World::from_level(&level);
        w.spawn_character(1, [600.0, 0.0]);
        let mut d = w.character.take().unwrap();
        let waist = d.break_limit(&w, Limb::Waist);
        let id = d.joints[&Limb::Waist];
        w.b2_mut().destroy_joint(id);
        d.broken.push(Limb::Waist);
        d.break_side_effects(&mut w, Limb::Waist, waist + 5.0);
        let n = d.intestines.as_ref().expect("intestines exposed").bodies.len();
        assert!((1..=9).contains(&n), "{n}");
        let neck = d.break_limit(&w, Limb::Neck);
        let id = d.joints[&Limb::Neck];
        w.b2_mut().destroy_joint(id);
        d.broken.push(Limb::Neck);
        d.break_side_effects(&mut w, Limb::Neck, neck + 1.0);
        assert!(d.spine.is_some());
        for _ in 0..30 {
            w.step();
        }
        d.smash_chest(&mut w);
        assert!(d.intestines.as_ref().unwrap().joints.last().unwrap().is_none(), "chest end torn");
        assert!(d.spine.as_ref().unwrap().joints[0].is_none(), "spine neck end torn");
        // a plain chest smash on a fresh doll builds the chain then tears its chest end
        w.spawn_character(1, [600.0, 0.0]);
        let mut d2 = w.character.take().unwrap();
        d2.smash_chest(&mut w);
        assert!(d2.intestines.is_some());
    }

    #[test]
    fn break_limits_follow_the_originals_mass_ratios() {
        let level = parse("<levelXML><info v='1.99' x='600' y='0'/></levelXML>").unwrap();
        let mut w = World::from_level(&level);
        w.spawn_character(1, [600.0, 0.0]);
        let d = w.character.take().unwrap();
        // Base character masses are the originals' DEF_* constants times a scale ratio close to 1.
        let neck = d.break_limit(&w, Limb::Neck);
        assert!((50.0..=130.0).contains(&neck), "neck limit {neck}");
        for limb in [Limb::Waist, Limb::Shoulder1, Limb::Elbow1, Limb::Hip1, Limb::Knee1] {
            assert!(d.break_limit(&w, limb) > 10.0, "{limb:?}");
        }
    }

    #[test]
    fn joint_load_grows_with_impact_speed_and_nothing_breaks_in_a_normal_fall() {
        // Peak (reaction force / break limit) over the whole fall, and the number of broken joints.
        let drop = |height: f32| {
            let level = parse(&format!(
                "<levelXML><info v='1.99' x='600' y='{height}'/><shapes>\
                 <sh t='0' p0='600' p1='3000' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>"
            ))
            .unwrap();
            let mut w = World::from_level(&level);
            w.spawn_character(1, [600.0, height]);
            let mut peak = 0.0f64;
            for _ in 0..300 {
                w.step();
                let d = w.character.as_ref().unwrap();
                for (&limb, &id) in &d.joints {
                    if let Some((f, _)) = w.b2().revolute_reaction(id) {
                        peak = peak.max(f / d.break_limit(&w, limb));
                    }
                }
            }
            (peak, w.character.as_ref().unwrap().broken.len())
        };
        let (short, short_broken) = drop(2500.0);
        let (long, _) = drop(-3500.0);
        assert_eq!(short_broken, 0);
        assert!(long > short, "load {long} should exceed {short}");
    }

    #[test]
    fn hands_grab_what_they_touch_while_space_is_held_and_let_go_on_release() {
        // Where does the left lower arm start? Put a small static ball right on it.
        let probe = parse("<levelXML><info v='1.99' x='600' y='0'/></levelXML>").unwrap();
        let mut w0 = World::from_level(&probe);
        w0.spawn_character(1, [600.0, 0.0]);
        let tip = w0.character.as_ref().unwrap().position_px(&w0, Part::LowerArm1).unwrap();
        let level = parse(&format!(
            "<levelXML><info v='1.99' x='600' y='0'/><shapes>\
             <sh t='1' p0='{}' p1='{}' p2='30' p3='30' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1' p12='0'/></shapes></levelXML>",
            tip[0], tip[1]
        ))
        .unwrap();
        let mut idle = World::from_level(&level);
        idle.spawn_character(1, [600.0, 0.0]);
        for _ in 0..5 {
            idle.step();
        }
        assert!(idle.character.as_ref().unwrap().grips.iter().all(Option::is_none), "grabbed without holding space");
        let mut w = World::from_level(&level);
        w.spawn_character(1, [600.0, 0.0]);
        w.set_character_grab(true);
        for _ in 0..5 {
            w.step();
        }
        assert!(w.character.as_ref().unwrap().grips.iter().any(Option::is_some), "hand did not grab the ball it overlaps");
        w.set_character_grab(false);
        assert!(w.character.as_ref().unwrap().grips.iter().all(Option::is_none));
    }
}

