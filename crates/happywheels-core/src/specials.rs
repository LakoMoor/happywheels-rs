//! Physics for the original's "special" objects. Each entry below was transcribed from the
//! original's own `createBody` source (dumped to analysis/specials-src.json), not guessed.
//! Only simple rigid-body specials are covered so far; multi-body ones (Van, Table, Chair, …),
//! joints and scripted behaviour are TODO.

use box2d::{math::Vec2, shape::{Geometry, ShapeDef}};

use crate::level::Special;

const PS: f64 = 62.5;

/// Joint between the level (fixed world) and the special's body.
#[derive(Debug, Clone, Copy)]
pub enum JointSpec {
    /// Pin at a world point (px) with optional angle limits (radians, relative to the creation pose).
    Revolute { anchor: [f64; 2], limits: Option<(f64, f64)> },
    /// Prismatic along a world axis, with translation limits (metres) — `(0, 0)` locks it.
    Prismatic { axis: [f64; 2], limits: (f64, f64) },
}

/// Wind zone of the Fan special: `center` of the sensor box (metres) and the fan angle (radians).
#[derive(Debug, Clone, Copy)]
pub struct FanSpec {
    pub center: [f64; 2],
    pub angle: f64,
}

/// Boost pad (`Boost`): power and the force direction `(sin(a), -cos(a))`, a = (rotation + 90)°.
#[derive(Debug, Clone, Copy)]
pub struct BoostSpec {
    pub power: f64,
    pub angle: f64,
}

/// `Jet`: impulse `power·accel` along the body's up axis every step, `accel` ramping by `accel_step` (starting at `accel0`), stops after
/// `fire_total` steps; smashes (explodes) when a contact impulse exceeds `round(100·mass)`.
#[derive(Debug, Clone, Copy)]
pub struct JetSpec {
    pub power: f64,
    pub fire_total: f64,
    pub accel0: f64,
    pub accel_step: f64,
    pub fixed_rotation: bool,
}

/// A body welded to the special's main body: origin at `local` (px, in the main body's frame), same angle.
pub struct Attached {
    pub local: [f64; 2],
    pub shapes: Vec<Geometry>,
    pub density: f64,
    pub friction: f64,
    pub restitution: f64,
}

/// A rigid body made of one or more shapes, ready to be inserted into the world.
pub struct SimpleBody {
    /// Shapes with the body's main material (body-local metres).
    pub shape: Vec<Geometry>,
    /// Body origin / angle override (px, radians); default is the special's own `x,y` / `angle`.
    pub pos: Option<[f64; 2]>,
    pub angle: Option<f64>,
    /// Extra shapes fixed in the level (world metres, density 0).
    pub statics: Vec<ShapeDef>,
    pub joint: Option<JointSpec>,
    /// More shapes on the same body with their own density: `(shapes, density)` (same friction/restitution).
    pub more: Vec<(Vec<Geometry>, f64)>,
    /// Separate bodies welded to this one (locked revolute joint at their centre), e.g. the Van's tires.
    pub attached: Vec<Attached>,
    pub density: f64,
    pub friction: f64,
    pub restitution: f64,
    /// Fixed in the level (immovable2 / level-body shape).
    pub fixed: bool,
    pub sleeping: bool,
    /// Fan: wind zone (the first static is its sensor).
    pub fan: Option<FanSpec>,
    /// Boost pad: force per unit mass of the pad (the first static is its sensor).
    pub boost: Option<BoostSpec>,
    /// Jet engine: thrust and burn parameters.
    pub jet: Option<JetSpec>,
    /// Filter group index of the main shapes (Paddle uses 15).
    pub group: i16,
}

/// Box `(half_w, half_h, centre_x, centre_y)` in px, local to the body.
fn bx(hw: f64, hh: f64, cx: f64, cy: f64) -> Geometry {
    ShapeDef::oriented_box(hw / PS, hh / PS, Vec2::new(cx / PS, cy / PS), 0.0).geometry
}

fn boxes(parts: &[(f64, f64, f64, f64)]) -> Vec<Geometry> {
    parts.iter().map(|&(hw, hh, cx, cy)| bx(hw, hh, cx, cy)).collect()
}

fn ball(r_px: f64) -> Geometry {
    Geometry::Circle { local_position: Vec2::default(), radius: r_px / PS }
}

fn num(s: &Special, k: &str) -> Option<f64> {
    s.num(k).map(f64::from)
}

/// `None` = not a simple special (yet), or non-interactive artwork.
pub fn simple_body(s: &Special) -> Option<SimpleBody> {
    if s.flag("interactive") == Some(false) {
        return None;
    }
    let sleeping = s.flag("sleeping").unwrap_or(false);
    let fixed = s.flag("immovable2").unwrap_or(false);
    let cuboid = |hw: f64, hh: f64| vec![bx(hw, hh, 0.0, 0.0)];
    let (shape, density, friction, restitution, fixed) = match s.kind {
        // IBeam: box w×h, density 75, friction .3, restitution .1
        3 => (cuboid(num(s, "shapeWidth")? / 2.0, num(s, "shapeHeight")? / 2.0), 75.0, 0.3, 0.1, fixed),
        // Log: box w×h, density 30, friction .7, restitution .2
        4 => (cuboid(num(s, "shapeWidth")? / 2.0, num(s, "shapeHeight")? / 2.0), 30.0, 0.7, 0.2, fixed),
        // TrashCan (intact can only; spill/crush states not ported): box 45.94×76.18 px (clip `shapes.block`), density .75, friction .3, restitution .1
        26 if s.flag("interactive") != Some(false) => (cuboid(45.94 / 2.0, 76.18 / 2.0), 0.75, 0.3, 0.1, false),
        // FoodItem: polygon from the clip's p0_* guides (watermelon/pumpkin hexagons, pineapple quad), density 2, friction .3, restitution .1
        32 if s.flag("interactive") != Some(false) => {
            let pts: &[(f64, f64)] = match num(s, "foodItemType").unwrap_or(1.0) as i32 {
                2 => &[(-11.9, -12.85), (12.25, -14.05), (20.05, 0.2), (13.0, 13.05), (-9.1, 14.05), (-20.1, 0.6)],
                3 => &[(-6.85, -16.95), (5.65, -17.2), (6.45, 20.1), (-6.85, 19.7)],
                _ => &[(-12.0, -12.4), (9.15, -12.7), (18.1, 1.5), (7.65, 13.3), (-12.5, 13.0), (-20.2, 1.05)],
            };
            (vec![Geometry::Polygon(pts.iter().map(|&(x, y)| Vec2::new(x / PS, y / PS)).collect())], 2.0, 0.3, 0.1, false)
        }
        // Meteor: circle of diameter shapeWidth, density 75, friction .5, restitution .1
        11 => (vec![ball(num(s, "shapeWidth")? / 2.0)], 75.0, 0.5, 0.1, fixed),
        // SoccerBall: circle r=10, density .5, friction .1, restitution .5
        10 => (vec![ball(10.0)], 0.5, 0.1, 0.5, false),
        // Mine: box 25×2 plus the 2×1 trigger box at (0,-2.5) (a solid shape whose Result detonates it), density 1, friction 1, restitution .1
        2 => (boxes(&[(12.5, 1.0, 0.0, 0.0), (1.0, 0.5, 0.0, -2.5)]), 1.0, 1.0, 0.1, false),
        // Bottle: box 10×29, density 2, friction .3, restitution .1
        20 => (cuboid(5.0, 14.5), 2.0, 0.3, 0.1, false),
        // TV: box 55×40, density 2, friction .3, restitution .1
        21 => (cuboid(27.5, 20.0), 2.0, 0.3, 0.1, false),
        // Boombox: box 48×28, density 2, friction .3, restitution .1
        22 => (cuboid(24.0, 14.0), 2.0, 0.3, 0.1, false),
        // Table: top + two legs, density 5, friction .1, restitution .2
        1 => (boxes(&[(78.0, 3.0, 0.0, -3.0), (3.0, 25.0, -48.0, 25.0), (3.0, 25.0, 48.0, 25.0)]), 5.0, 0.1, 0.2, false),
        // Chair: seat + two legs + back (back leans to the side given by `reverse`), same material as Table
        19 => {
            let sign = if s.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
            let shape = boxes(&[(21.0, 3.0, 0.0, -3.0), (3.0, 16.0, -18.0, 16.0), (3.0, 16.0, 18.0, 16.0), (3.0, 25.0, -18.0 * sign, -31.0)]);
            (shape, 5.0, 0.1, 0.2, false)
        }
        // Glass: box w×h, density 2, friction .3, restitution .1 (shattering is behaviour, not modelled)
        18 => (cuboid(num(s, "shapeWidth")? / 2.0, num(s, "shapeHeight")? / 2.0), 2.0, 0.3, 0.1, false),
        // Rail: two static boxes (w × 9) stacked at local (0, ∓4.5), μ.3 e.1 (the upper one carries
        // material bit 4, which only matters for character grinding behaviour — not modelled).
        27 => {
            let half = num(s, "shapeWidth")? / 2.0;
            (boxes(&[(half, 4.5, 0.0, -4.5), (half, 4.5, 0.0, 4.5)]), 1.0, 0.3, 0.1, true)
        }
        // FinishLine: static box 400×40 at (x,y), μ.3 e.1. The win zone (x±200, y-200..y) is behaviour for
        // when characters exist.
        9 => (cuboid(200.0, 20.0), 1.0, 0.3, 0.1, true),
        // Toilet: back box 20×47 at (-21s,-16) ρ2, base box 21×15 at (3s,31.5) ρ4, 5-vertex bowl ρ2; μ.3 e.1.
        // (Breaking behaviour on impact is not modelled.)
        24 => {
            let sg = if s.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
            let v = |x: f64, y: f64| Vec2::new(x * sg / PS, y / PS);
            let mut bowl = vec![v(-25.0, 4.5), v(31.0, 4.5), v(25.0, 20.0), v(3.5, 30.5), v(-22.0, 17.0)];
            if sg < 0.0 {
                bowl = vec![bowl[1], bowl[0], bowl[4], bowl[3], bowl[2]];
            }
            return Some(SimpleBody {
                shape: vec![bx(10.0, 23.5, -21.0 * sg, -16.0)],
                pos: None,
                angle: None,
                statics: Vec::new(),
                joint: None,
                attached: Vec::new(),
                more: vec![(vec![bx(10.5, 7.5, 3.0 * sg, 31.5)], 4.0), (vec![Geometry::Polygon(bowl)], 2.0)],
                density: 2.0,
                friction: 0.3,
                restitution: 0.1,
                fixed: false,
                sleeping,
                group: 0,
                fan: None, boost: None, jet: None,
            });
        }
        _ => return composite(s),
    };
    Some(SimpleBody { shape, pos: None, angle: None, statics: Vec::new(), joint: None, attached: Vec::new(), more: Vec::new(), density, friction, restitution, fixed, sleeping, group: 0, fan: None, boost: None, jet: None })
}

/// Specials with extra fixed geometry and/or a joint to the level (transcribed from the original).
fn composite(s: &Special) -> Option<SimpleBody> {
    let (x, y) = (num(s, "x")?, num(s, "y")?);
    let angle = num(s, "angle").unwrap_or(0.0).to_radians();
    let sleeping = s.flag("sleeping").unwrap_or(false);
    let fixed_box = |hw: f64, hh: f64, c: [f64; 2], a: f64| ShapeDef::oriented_box(hw / PS, hh / PS, Vec2::new(c[0] / PS, c[1] / PS), a);
    match s.kind {
        // WreckingBall: body at (x + ropeLength, y), angle -90°; stem box 36×15 at (0,-80) + ball r=75
        // (ρ500 μ.1 e.1); fixed anchor box 36×32 at (x, y-16); pin at (x,y), limits [-45°, 225°].
        7 => {
            let rope = num(s, "ropeLength")?;
            Some(SimpleBody {
                shape: vec![bx(18.0, 7.5, 0.0, -80.0), ball(75.0)],
                pos: Some([x + rope, y]),
                angle: Some(-std::f64::consts::FRAC_PI_2),
                statics: vec![fixed_box(18.0, 16.0, [x, y - 16.0], 0.0)],
                attached: Vec::new(), more: Vec::new(),
                joint: Some(JointSpec::Revolute {
                    anchor: [x, y],
                    limits: Some((-0.25 * std::f64::consts::PI, 1.25 * std::f64::consts::PI)),
                }),
                density: 500.0,
                friction: 0.1,
                restitution: 0.1,
                fixed: false,
                sleeping: false,
                group: 0,
                fan: None, boost: None, jet: None,
            })
        }
        // SpringBox: pad 350×22 at (0,-9) + two feet 26×18 at (±120,11) (ρ200 μ.5 e.1); fixed base 350×32
        // at body-local (0,4); prismatic to the level along (sin a, -cos a), locked (limits 0..0) — the
        // launching motor is runtime behaviour, not modelled.
        5 => {
            let (sin, cos) = angle.sin_cos();
            Some(SimpleBody {
                shape: boxes(&[(175.0, 11.0, 0.0, -9.0), (13.0, 9.0, -120.0, 11.0), (13.0, 9.0, 120.0, 11.0)]),
                pos: None,
                angle: None,
                statics: vec![fixed_box(175.0, 16.0, [x - 4.0 * sin, y + 4.0 * cos], angle)],
                attached: Vec::new(), more: Vec::new(),
                joint: Some(JointSpec::Prismatic { axis: [sin, -cos], limits: (0.0, 0.0) }),
                density: 200.0,
                friction: 0.5,
                restitution: 0.1,
                fixed: false,
                sleeping,
                group: 0,
                fan: None, boost: None, jet: None,
            })
        }
        // Spikes: solid base box (halfWidth = 7.5·numSpikes) × 20 at local (0,35), ρ3 μ.3 e.1, fixed when
        // immovable2 (default true). The 50-tall impale sensor and the impaling joints are not modelled.
        6 => {
            let half = 7.5 * num(s, "numSpikes").unwrap_or(1.0);
            Some(SimpleBody {
                shape: boxes(&[(half, 10.0, 0.0, 35.0)]),
                pos: None,
                angle: None,
                statics: Vec::new(),
                joint: None,
                attached: Vec::new(), more: Vec::new(),
                density: 3.0,
                friction: 0.3,
                restitution: 0.1,
                fixed: s.flag("immovable2").unwrap_or(true),
                sleeping,
                group: 0,
                fan: None, boost: None, jet: None,
            })
        }
        // Van (transcribed from createBody/createJoints): body box 132×116 (ρ10 μ.3 e.1, category 8); two tire
        // bodies at local (±58, 48) with a 20×45 box (ρ10 μ1 e.3), each welded to the van by a revolute joint
        // locked at limits 0..0 (anchor = the tire's centre).
        0 => {
            let tire = |x: f64| Attached { local: [x, 48.0], shapes: vec![bx(10.0, 22.5, 0.0, 0.0)], density: 10.0, friction: 1.0, restitution: 0.3 };
            Some(SimpleBody {
                shape: vec![bx(66.0, 58.0, 0.0, 0.0)],
                pos: None,
                angle: None,
                statics: Vec::new(),
                joint: None,
                attached: vec![tire(-58.0), tire(58.0)],
                more: Vec::new(),
                density: 10.0,
                friction: 0.3,
                restitution: 0.1,
                fixed: false,
                sleeping,
                group: 0,
                fan: None, boost: None, jet: None,
            })
        }
        // Paddle: pad box 332×30 at (0,-5) ρ200 μ.5 e.1 (group 15); fixed level box 350×40 at the origin (group 0)
        // and a fixed blocker 262.5×10 at local (0,15) (group 15); pin to the level at local (151·dir, -5), locked.
        // The swinging behaviour (timer/spring) is runtime and not modelled.
        35 => {
            let dir = if s.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
            let (sin, cos) = angle.sin_cos();
            let world = |lx: f64, ly: f64| [x + lx * cos - ly * sin, y + lx * sin + ly * cos];
            let mut blocker = fixed_box(131.25, 5.0, world(0.0, 15.0), angle);
            blocker.filter.group_index = 15;
            Some(SimpleBody {
                shape: vec![bx(166.0, 15.0, 0.0, -5.0)],
                pos: None,
                angle: None,
                statics: vec![fixed_box(175.0, 20.0, [x, y], angle), blocker],
                attached: Vec::new(),
                more: Vec::new(),
                joint: Some(JointSpec::Revolute { anchor: world(151.0 * dir, -5.0), limits: Some((0.0, 0.0)) }),
                density: 200.0,
                friction: 0.5,
                restitution: 0.1,
                fixed: false,
                sleeping: false,
                group: 15,
                fan: None, boost: None, jet: None,
            })
        }
        // Buildings: one static box on the level body; Building1 (floorWidth·300) × (165·floors + 100),
        // Building2 (500·floorWidth + 226) × (180·floors + 120), anchored at their top-left corner (x, y), μ.3 e.1.
        13 | 14 => {
            let (fw, nf) = (num(s, "floorWidth")?, num(s, "numFloors")?);
            let (w, h) = if s.kind == 13 { (300.0 * fw, 165.0 * nf + 100.0) } else { (500.0 * fw + 226.0, 180.0 * nf + 120.0) };
            Some(SimpleBody {
                shape: Vec::new(),
                pos: None,
                angle: None,
                statics: vec![fixed_box(0.5 * w, 0.5 * h, [x + 0.5 * w, y + 0.5 * h], 0.0)],
                attached: Vec::new(),
                more: Vec::new(),
                joint: None,
                density: 0.0,
                friction: 0.3,
                restitution: 0.1,
                fixed: true,
                sleeping: false,
                group: 0,
                fan: None, boost: None, jet: None,
            })
        }
        // Jet: box (15.5 × 19 px, both grown by (power-1)/9), density 3, friction .5, restitution .1, optionally fixed rotation.
        28 => {
            let power = num(s, "power").unwrap_or(1.0);
            let grow = (power - 1.0) / 9.0;
            let (w, h) = (15.5 + 15.5 * grow, 19.0 + 19.0 * grow);
            let fire_time = num(s, "fireTime").unwrap_or(0.0);
            let accel_time = num(s, "accelTime").unwrap_or(0.0);
            Some(SimpleBody {
                shape: vec![bx(0.5 * w, 0.5 * h, 0.0, 0.0)],
                pos: None,
                angle: None,
                statics: Vec::new(),
                attached: Vec::new(),
                more: Vec::new(),
                joint: None,
                density: 3.0,
                friction: 0.5,
                restitution: 0.1,
                fixed: false,
                sleeping,
                group: 0,
                fan: None,
                boost: None,
                jet: Some(JetSpec {
                    power,
                    fire_total: if fire_time > 0.0 { 30.0 * fire_time } else { f64::INFINITY },
                    accel0: if accel_time > 0.0 { 0.0 } else { 1.0 },
                    accel_step: if accel_time > 0.0 { 1.0 / (30.0 * accel_time) } else { 0.0 },
                    fixed_rotation: s.flag("fixedRotation").unwrap_or(false),
                }),
            })
        }
        // Boost: sensor box (half-extents 90·numPanels × 90) at (x,y) rotated by the pad's angle (group -20, category 8). Behaviour: every
        // body seen in the sensor's PERSIST events gets force `power·mass` along `(sin a, -cos a)` (a = (rotation+90)°) each frame.
        12 => {
            let panels = num(s, "numPanels").unwrap_or(2.0);
            let mut sensor = fixed_box(90.0 * panels, 90.0, [x, y], angle);
            sensor.is_sensor = true;
            sensor.filter.group_index = -20;
            Some(SimpleBody {
                shape: Vec::new(),
                pos: None,
                angle: None,
                statics: vec![sensor],
                attached: Vec::new(),
                more: Vec::new(),
                joint: None,
                density: 0.0,
                friction: 0.3,
                restitution: 0.1,
                fixed: true,
                sleeping: false,
                group: 0,
                fan: None,
                boost: Some(BoostSpec { power: num(s, "boostPower").unwrap_or(20.0), angle: angle + std::f64::consts::FRAC_PI_2 }),
                jet: None,
            })
        }
        // Fan: wind sensor box 300×500 centred 300 px along the fan's up axis (group -20), floor box 300×50 at
        // 25 px below it, and a 7-vertex half-disc (r 150) on top, all static.
        8 => {
            let (sin, cos) = angle.sin_cos();
            let mut sensor = fixed_box(150.0, 250.0, [x + 300.0 * sin, y - 300.0 * cos], angle);
            sensor.is_sensor = true;
            sensor.filter.group_index = -20;
            let floor = fixed_box(150.0, 25.0, [x + 25.0 * sin, y - 25.0 * cos], angle);
            let pts = [(150.0, 0.0), (125.0, 84.5), (65.0, 137.0), (0.0, 150.0), (-65.0, 137.0), (-125.0, 84.5), (-150.0, 0.0)];
            let dome = ShapeDef::new(Geometry::Polygon(
                pts.iter()
                    .map(|&(px, py)| {
                        let (vx, vy) = (px / PS, py / PS);
                        Vec2::new(cos * vx - sin * vy + x / PS, sin * vx + cos * vy + y / PS)
                    })
                    .collect(),
            ));
            Some(SimpleBody {
                shape: Vec::new(),
                pos: None,
                angle: None,
                statics: vec![sensor, floor, dome],
                attached: Vec::new(),
                more: Vec::new(),
                joint: None,
                density: 0.0,
                friction: 0.3,
                restitution: 0.1,
                fixed: true,
                sleeping: false,
                group: 0,
                fan: Some(FanSpec { center: [(x + 300.0 * sin) / PS, (y - 300.0 * cos) / PS], angle }),
                boost: None, jet: None,
            })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn special(kind: usize, props: &[(&str, &str)]) -> Special {
        let names = crate::level::special_property_names(kind);
        let mut v = vec![None; names.len()];
        for (k, val) in props {
            let i = names.iter().position(|n| n == k).unwrap_or_else(|| panic!("{k} not a property of kind {kind}: {names:?}"));
            v[i] = Some((*val).to_string());
        }
        Special { kind, props: v, texts: Default::default() }
    }

    #[test]
    fn property_names_come_from_the_original() {
        assert_eq!(crate::level::special_property_names(3), ["x", "y", "shapeWidth", "shapeHeight", "angle", "immovable2", "sleeping"]);
        assert_eq!(crate::level::special_property_names(10), ["x", "y"]);
    }

    #[test]
    fn ibeam_is_a_heavy_box_and_log_a_light_grippy_one() {
        let ib = simple_body(&special(3, &[("shapeWidth", "200"), ("shapeHeight", "20")])).unwrap();
        let lg = simple_body(&special(4, &[("shapeWidth", "200"), ("shapeHeight", "20"), ("immovable2", "t")])).unwrap();
        assert_eq!((ib.density, ib.friction, ib.restitution, ib.fixed), (75.0, 0.3, 0.1, false));
        assert_eq!((lg.density, lg.friction, lg.restitution, lg.fixed), (30.0, 0.7, 0.2, true));
    }

    #[test]
    fn non_interactive_and_unknown_specials_are_skipped() {
        assert!(simple_body(&special(20, &[("interactive", "f")])).is_none());
        assert!(simple_body(&special(25, &[])).is_none()); // HomingMine: scripted, not done yet
        assert!(simple_body(&special(26, &[("interactive", "f")])).is_none());
    }
}
