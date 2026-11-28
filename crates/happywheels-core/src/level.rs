//! Happy Wheels level XML (`<levelXML><info/><shapes><sh t p0..pN/>…`).
//!
//! Properties are stored by index (`p0..pN`) exactly as in the file; the index→name
//! mapping in [`property_names`] was read from the running original.

use std::collections::BTreeMap;

use anyhow::{bail, Context, Result};

/// Names of the original's special types, in `specialList` order (kind index → name, minus "Ref").
pub const SPECIAL_NAMES: [&str; 36] = [
    "Van", "Table", "Mine", "IBeam", "Log", "SpringBox", "Spikes", "WreckingBall", "Fan", "FinishLine",
    "SoccerBall", "Meteor", "Boost", "Building1", "Building2", "HarpoonGun", "TextBox", "NPCharacter",
    "Glass", "Chair", "Bottle", "TV", "Boombox", "SignPost", "Toilet", "HomingMine", "TrashCan", "Rail",
    "Jet", "ArrowGun", "Chain", "Token", "FoodItem", "Cannon", "BladeWeapon", "Paddle",
];

/// Ordered `pN` property names of a special type, read from the running original
/// (`getFullProperties()`, dumped to data/original-properties.json).
pub fn special_property_names(kind: usize) -> Vec<String> {
    use std::sync::OnceLock;
    static DATA: OnceLock<serde_json::Value> = OnceLock::new();
    let data = DATA.get_or_init(|| serde_json::from_str(include_str!("../data/original-properties.json")).expect("embedded property dump"));
    let Some(name) = SPECIAL_NAMES.get(kind) else { return Vec::new() };
    data["specials"][format!("{name}Ref")]
        .as_array()
        .map(|a| a.iter().map(|p| p["name"].as_str().unwrap_or("").to_owned()).collect())
        .unwrap_or_default()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeKind {
    Rectangle,
    Circle,
    Triangle,
    Polygon,
    Art,
}

impl ShapeKind {
    pub fn from_index(t: usize) -> Result<Self> {
        Ok(match t {
            0 => Self::Rectangle,
            1 => Self::Circle,
            2 => Self::Triangle,
            3 => Self::Polygon,
            4 => Self::Art,
            _ => bail!("unknown shape type {t}"),
        })
    }
}

/// Property order of `<sh pN>`, VERIFIED against the original via `getFullProperties()`
/// (see analysis/original-properties.json). Circle adds `innerCutout`.
pub fn property_names(kind: ShapeKind) -> &'static [&'static str] {
    const BASE: [&str; 12] = [
        "x", "y", "shapeWidth", "shapeHeight", "angle", "immovable", "sleeping", "density",
        "color", "outlineColor", "opacity", "collision",
    ];
    const CIRCLE: [&str; 13] = [
        "x", "y", "shapeWidth", "shapeHeight", "angle", "immovable", "sleeping", "density",
        "color", "outlineColor", "opacity", "collision", "innerCutout",
    ];
    match kind {
        ShapeKind::Circle => &CIRCLE,
        _ => &BASE,
    }
}

#[derive(Debug, Clone)]
pub struct Shape {
    pub kind: ShapeKind,
    /// `i='f'`: non-interactive artwork.
    pub interactive: bool,
    /// Raw `p0..pN` values by index.
    pub props: Vec<String>,
    /// Polygon/art vertices (`x_y` pairs; art may carry bezier handles).
    pub verts: Vec<[f32; 2]>,
    /// Art only: per vertex `[in.x, in.y, out.x, out.y]` bezier handles (`x_y_inx_iny_outx_outy`, all zero = straight).
    pub handles: Vec<[f32; 4]>,
    /// Art only: `f='t'` on the `v` element — the path is closed and filled.
    pub complete_fill: bool,
}

impl Shape {
    fn index_of(&self, name: &str) -> Option<usize> {
        property_names(self.kind).iter().position(|n| *n == name)
    }
    pub fn num(&self, name: &str) -> Option<f32> {
        self.props.get(self.index_of(name)?)?.parse().ok()
    }

    /// Factor on a polygon's or art shape's vertices: `shapeWidth` / `shapeHeight` are pixel sizes of the box around the
    /// vertices *and the origin* (the original's `setDefaultDimensions`), the factor limited to 0.1..10 like its setters.
    pub fn vertex_scale(&self) -> [f32; 2] {
        [("shapeWidth", 0), ("shapeHeight", 1)].map(|(name, axis)| {
            let (lo, hi) = self.verts.iter().fold((0.0f32, 0.0f32), |(lo, hi), v| (lo.min(v[axis]), hi.max(v[axis])));
            match self.num(name) {
                Some(size) if size != 0.0 && hi > lo => (size / (hi - lo)).clamp(0.1, 10.0),
                _ => 1.0,
            }
        })
    }
    pub fn flag(&self, name: &str) -> Option<bool> {
        match self.props.get(self.index_of(name)?)?.as_str() {
            "t" => Some(true),
            "f" => Some(false),
            _ => None,
        }
    }
}

/// `<j>`: pin (t=0) or sliding (t=1) joint. Body refs: "-1" = world, "s<i>" = special i,
/// "g<i>" = group i, bare "<i>" = shape (verified against the original's loader and `createXML`).
#[derive(Debug, Clone, PartialEq)]
pub struct Joint {
    pub sliding: bool,
    pub pos: [f32; 2],
    pub body1: BodyRef,
    pub body2: BodyRef,
    pub limit: bool,
    pub motor: bool,
    pub collide_self: bool,
    /// Pin: `ua`/`la` (angle limits). Slider: `ul`/`ll` (translation limits).
    pub upper: f32,
    pub lower: f32,
    /// Pin: `tq` (torque). Slider: `fo` (force).
    pub force: f32,
    /// `sp`: motor speed.
    pub speed: f32,
    /// Slider axis angle `a`.
    pub axis_angle: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyRef {
    World,
    Shape(usize),
    Group(usize),
    Special(usize),
}

impl BodyRef {
    fn parse(s: &str) -> Self {
        // VERIFIED (level loader / createXML): "s<i>" = special i, "g<i>" = group i, bare "<i>" = shape i.
        if let Some(n) = s.strip_prefix('g') {
            n.parse().map_or(Self::World, Self::Group)
        } else if let Some(n) = s.strip_prefix('s') {
            n.parse().map_or(Self::World, Self::Special)
        } else {
            s.parse().map_or(Self::World, Self::Shape)
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct Info {
    pub version: f32,
    pub start: [f32; 2],
    pub character: i32,
    pub background: i32,
    /// `bgc`: background colour 0xRRGGBB (default white, as the loader does when the attribute is missing).
    pub background_color: u32,
}

#[derive(Debug, Clone, Default)]
pub struct Level {
    pub info: Info,
    pub shapes: Vec<Shape>,
    pub special_count: usize,
    pub joint_count: usize,
    pub joints: Vec<Joint>,
    pub specials: Vec<Special>,
    pub groups: Vec<Group>,
    pub triggers: Vec<Trigger>,
    /// The level built into the original (id 1) instead of a `levelXML` one.
    pub builtin: bool,
}

impl Level {
    /// The built-in level: it has no XML, its content comes from `World::level1`.
    pub fn level1() -> Self {
        let info = Info { start: crate::physics::LEVEL1_START, background_color: 0xFF_FFFF, ..Info::default() };
        Level { info, builtin: true, ..Level::default() }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Special {
    /// Index into the original's special list (0 = VanRef, 1 = TableRef, …).
    pub kind: usize,
    /// Raw `pN` attributes, by index (missing = None).
    pub props: Vec<Option<String>>,
    /// `<pN><![CDATA[..]]></pN>` text properties (e.g. text box captions).
    pub texts: BTreeMap<usize, String>,
}

impl Special {
    fn raw(&self, name: &str) -> Option<&str> {
        let i = special_property_names(self.kind).iter().position(|n| n == name)?;
        self.props.get(i)?.as_deref()
    }
    pub fn num(&self, name: &str) -> Option<f32> {
        self.raw(name)?.parse().ok()
    }
    /// Boolean property (`t`/`f`); `None` when absent.
    pub fn flag(&self, name: &str) -> Option<bool> {
        match self.raw(name)? {
            "t" => Some(true),
            "f" => Some(false),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct VehicleProps {
    pub space_action: i32,
    pub shift_action: i32,
    pub ctrl_action: i32,
    pub acceleration: f32,
    pub leaning: f32,
    pub pose: i32,
    pub lock_joints: bool,
}

#[derive(Debug, Clone, Default)]
pub struct Group {
    pub pos: [f32; 2],
    pub angle: f32,
    pub offset: [f32; 2],
    pub sleeping: bool,
    pub foreground: bool,
    pub opacity: f32,
    pub immovable: bool,
    pub fixed_rotation: bool,
    /// Present when `v='t'` (a controllable vehicle/character group).
    pub vehicle: Option<VehicleProps>,
    pub shapes: Vec<Shape>,
    pub specials: Vec<Special>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetKind {
    Shape,
    Special,
    Group,
    Joint,
    Trigger,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TriggerTarget {
    pub kind: TargetKind,
    pub index: usize,
    /// `<a i=… p0…/>` actions (action-trigger type only).
    pub actions: Vec<Vec<String>>,
}

#[derive(Debug, Clone, Default)]
pub struct Trigger {
    pub pos: [f32; 2],
    pub size: [f32; 2],
    pub angle: f32,
    pub triggered_by: i32,
    /// 1 = action trigger, 2 = sound trigger, 3 = level victory.
    pub type_index: i32,
    pub repeat_type: i32,
    pub start_disabled: bool,
    pub repeat_interval: Option<f32>,
    pub delay: Option<f32>,
    pub sound_index: Option<usize>,
    pub sound_location: i32,
    pub sound_volume: f32,
    pub sound_pan: f32,
    pub targets: Vec<TriggerTarget>,
}

type VertData = (Vec<[f32; 2]>, Vec<[f32; 4]>, bool);
type VertCache = BTreeMap<(bool, i64), VertData>;

fn parse_shape(sh: roxmltree::Node, vert_cache: &mut VertCache) -> Result<Shape> {
    let t = sh.attribute("t").context("shape without t")?.parse::<usize>()?;
    let interactive = sh.attribute("i") != Some("f");
    // Non-interactive polygons are stored as type 3 but behave as art (type 4).
    let mut kind = ShapeKind::from_index(t)?;
    if kind == ShapeKind::Polygon && !interactive {
        kind = ShapeKind::Art;
    }
    let mut props = Vec::new();
    while let Some(v) = sh.attribute(format!("p{}", props.len()).as_str()) {
        props.push(v.to_owned());
    }
    crate::editor::limit_attributes(false, t, props.iter_mut().enumerate());
    let mut data: VertData = (Vec::new(), Vec::new(), false);
    if matches!(kind, ShapeKind::Polygon | ShapeKind::Art) {
        if let Some(v) = sh.children().find(|n| n.has_tag_name("v")) {
            let id = v.attribute("id").and_then(|s| s.parse().ok()).unwrap_or(-1);
            let key = (kind == ShapeKind::Art, id);
            if let Some(n) = v.attribute("n").and_then(|s| s.parse::<usize>().ok()) {
                let (mut list, mut handles) = (Vec::new(), Vec::new());
                for i in 0..n {
                    let s = v.attribute(format!("v{i}").as_str()).unwrap_or("0_0");
                    let p: Vec<f32> = s.split('_').map(|x| x.parse::<f32>().unwrap_or(0.0)).collect();
                    let at = |k: usize| p.get(k).copied().unwrap_or(0.0);
                    list.push([at(0), at(1)]);
                    handles.push([at(2), at(3), at(4), at(5)]);
                }
                vert_cache.insert(key, (list, handles, v.attribute("f") == Some("t")));
            }
            data = vert_cache.get(&key).cloned().unwrap_or(data);
        }
    }
    let (verts, handles, complete_fill) = data;
    Ok(Shape { kind, interactive, props, verts, handles, complete_fill })
}

fn parse_special(sp: roxmltree::Node) -> Result<Special> {
    let kind = sp.attribute("t").context("special without t")?.parse::<usize>()?;
    let mut props = Vec::new();
    let mut texts = BTreeMap::new();
    for child in sp.children().filter(|c| c.is_element()) {
        if let Some(i) = child.tag_name().name().strip_prefix('p').and_then(|n| n.parse::<usize>().ok()) {
            texts.insert(i, child.text().unwrap_or("").to_owned());
        }
    }
    let mut i = 0;
    while i < 64 {
        match sp.attribute(format!("p{i}").as_str()) {
            Some(v) => props.push(Some(v.to_owned())),
            None if texts.contains_key(&i) || i < props.len() + 1 && sp.attributes().any(|a| a.name().strip_prefix('p').and_then(|n| n.parse::<usize>().ok()).is_some_and(|n| n > i)) => props.push(None),
            None => break,
        }
        i += 1;
    }
    crate::editor::limit_attributes(true, kind, props.iter_mut().enumerate().filter_map(|(i, p)| Some((i, p.as_mut()?))));
    Ok(Special { kind, props, texts })
}

/// [`parse`] with every shape's and special's parameters as written: for replaying goldens that were recorded from the original's
/// classes with sizes its level loader would have limited (a floor wider than 5000 px, a 20 px thick log).
pub fn parse_unlimited(xml: &str) -> Result<Level> {
    let mut level = parse(xml)?;
    let doc = roxmltree::Document::parse(xml).context("level XML")?;
    let mut shapes = doc.descendants().filter(|n| n.has_tag_name("sh"));
    let mut specials = doc.descendants().filter(|n| n.has_tag_name("sp"));
    for shape in &mut level.shapes {
        if let Some(node) = shapes.next() {
            for (i, prop) in shape.props.iter_mut().enumerate() {
                if let Some(raw) = node.attribute(format!("p{i}").as_str()) {
                    *prop = raw.to_owned();
                }
            }
        }
    }
    for special in &mut level.specials {
        if let Some(node) = specials.next() {
            for (i, prop) in special.props.iter_mut().enumerate() {
                *prop = node.attribute(format!("p{i}").as_str()).map(str::to_owned);
            }
        }
    }
    Ok(level)
}

pub fn parse(xml: &str) -> Result<Level> {
    let doc = roxmltree::Document::parse(xml).context("level XML")?;
    let root = doc.root_element();
    if root.tag_name().name() != "levelXML" {
        bail!("root element is <{}>, expected <levelXML>", root.tag_name().name());
    }
    let f = |n: roxmltree::Node, a: &str| n.attribute(a).and_then(|v| v.parse::<f32>().ok());
    let flag = |n: roxmltree::Node, a: &str| n.attribute(a) == Some("t");
    let mut level = Level::default();
    if let Some(info) = root.children().find(|n| n.has_tag_name("info")) {
        level.info = Info {
            version: f(info, "v").unwrap_or(0.0),
            start: [f(info, "x").unwrap_or(0.0), f(info, "y").unwrap_or(0.0)],
            character: f(info, "c").unwrap_or(0.0) as i32,
            background: f(info, "bg").unwrap_or(0.0) as i32,
            background_color: info.attribute("bgc").and_then(|v| v.parse::<i64>().ok()).map_or(0xFF_FFFF, |v| v as u32 & 0xFF_FFFF),
        };
    }
    // Vertex lists are shared by id: later shapes may carry only `<v id=…/>`.
    let mut vert_cache = VertCache::new();
    if let Some(shapes) = root.children().find(|n| n.has_tag_name("shapes")) {
        for sh in shapes.children().filter(|n| n.has_tag_name("sh")) {
            level.shapes.push(parse_shape(sh, &mut vert_cache)?);
        }
    }
    if let Some(sps) = root.children().find(|n| n.has_tag_name("specials")) {
        for sp in sps.children().filter(|n| n.has_tag_name("sp")) {
            level.specials.push(parse_special(sp)?);
        }
    }
    if let Some(gs) = root.children().find(|n| n.has_tag_name("groups")) {
        for g in gs.children().filter(|n| n.has_tag_name("g")) {
            let mut group = Group {
                pos: [f(g, "x").unwrap_or(0.0), f(g, "y").unwrap_or(0.0)],
                angle: f(g, "r").unwrap_or(0.0),
                offset: [f(g, "ox").unwrap_or(0.0), f(g, "oy").unwrap_or(0.0)],
                sleeping: flag(g, "s"),
                foreground: flag(g, "f"),
                opacity: f(g, "o").unwrap_or(100.0),
                immovable: flag(g, "im"),
                fixed_rotation: flag(g, "fr"),
                ..Default::default()
            };
            if flag(g, "v") {
                group.vehicle = Some(VehicleProps {
                    space_action: f(g, "sb").unwrap_or(0.0) as i32,
                    shift_action: f(g, "sh").unwrap_or(0.0) as i32,
                    ctrl_action: f(g, "ct").unwrap_or(0.0) as i32,
                    acceleration: f(g, "a").unwrap_or(0.0),
                    leaning: f(g, "l").unwrap_or(0.0),
                    pose: f(g, "cp").unwrap_or(0.0) as i32,
                    lock_joints: flag(g, "lo"),
                });
            }
            for c in g.children().filter(|c| c.is_element()) {
                match c.tag_name().name() {
                    "sh" => group.shapes.push(parse_shape(c, &mut vert_cache)?),
                    "sp" => group.specials.push(parse_special(c)?),
                    _ => {}
                }
            }
            level.groups.push(group);
        }
    }
    if let Some(js) = root.children().find(|n| n.has_tag_name("joints")) {
        for j in js.children().filter(|n| n.has_tag_name("j")) {
            let sliding = j.attribute("t") == Some("1");
            // NB: the original writes `ul ='…'` / `ua ='…'` with a space before `=`; XML allows it.
            let (upper, lower, force) = if sliding {
                (f(j, "ul"), f(j, "ll"), f(j, "fo"))
            } else {
                (f(j, "ua"), f(j, "la"), f(j, "tq"))
            };
            level.joints.push(Joint {
                sliding,
                upper: upper.unwrap_or(0.0),
                lower: lower.unwrap_or(0.0),
                force: force.unwrap_or(0.0),
                speed: f(j, "sp").unwrap_or(0.0),
                axis_angle: f(j, "a").unwrap_or(0.0),
                pos: [f(j, "x").unwrap_or(0.0), f(j, "y").unwrap_or(0.0)],
                body1: BodyRef::parse(j.attribute("b1").unwrap_or("-1")),
                body2: BodyRef::parse(j.attribute("b2").unwrap_or("-1")),
                limit: flag(j, "l"),
                motor: flag(j, "m"),
                collide_self: flag(j, "c"),
            });
        }
    }
    if let Some(ts) = root.children().find(|n| n.has_tag_name("triggers")) {
        for t in ts.children().filter(|n| n.has_tag_name("t")) {
            let mut trig = Trigger {
                pos: [f(t, "x").unwrap_or(0.0), f(t, "y").unwrap_or(0.0)],
                size: [f(t, "w").unwrap_or(0.0), f(t, "h").unwrap_or(0.0)],
                angle: f(t, "a").unwrap_or(0.0),
                triggered_by: f(t, "b").unwrap_or(0.0) as i32,
                type_index: f(t, "t").unwrap_or(0.0) as i32,
                repeat_type: f(t, "r").unwrap_or(0.0) as i32,
                start_disabled: flag(t, "sd"),
                repeat_interval: f(t, "i"),
                delay: f(t, "d"),
                sound_index: t.attribute("s").and_then(|s| s.parse().ok()),
                sound_location: f(t, "l").unwrap_or(1.0) as i32,
                sound_volume: f(t, "v").unwrap_or(1.0),
                sound_pan: f(t, "p").unwrap_or(0.0),
                targets: Vec::new(),
            };
            for c in t.children().filter(|c| c.is_element()) {
                let kind = match c.tag_name().name() {
                    "sh" => TargetKind::Shape,
                    "sp" => TargetKind::Special,
                    "g" => TargetKind::Group,
                    "j" => TargetKind::Joint,
                    "t" => TargetKind::Trigger,
                    _ => continue,
                };
                let actions = c
                    .children()
                    .filter(|a| a.has_tag_name("a"))
                    .map(|a| {
                        std::iter::once(a.attribute("i").unwrap_or("0").to_owned())
                            .chain((0..).map_while(|n| a.attribute(format!("p{n}").as_str()).map(str::to_owned)))
                            .collect()
                    })
                    .collect();
                trig.targets.push(TriggerTarget { kind, index: c.attribute("i").and_then(|v| v.parse().ok()).unwrap_or(0), actions });
            }
            level.triggers.push(trig);
        }
    }
    level.special_count = level.specials.len();
    level.joint_count = level.joints.len();
    Ok(level)
}

#[cfg(test)]
mod tests {
    use super::*;

    const XML: &str = "<levelXML><info v='1.99' x='100' y='-50' c='2' bg='0'/><shapes>\
        <sh t='0' p0='0' p1='200' p2='1000' p3='20' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>\
        <sh t='1' p0='50' p1='0' p2='50' p3='50' p4='0' p5='f' p6='f' p7='1' p8='16711680' p9='-1' p10='100' p11='1' p12='0'/>\
        <sh t='3'><v f='t' id='1' n='3' v0='0_0' v1='10_0' v2='0_10'/></sh>\
        <sh t='3'><v f='t' id='1'/></sh></shapes><specials><sp t='0'/></specials></levelXML>";

    #[test]
    fn parses_shapes_and_shared_vertices() {
        let l = parse(XML).unwrap();
        assert_eq!(l.info.start, [100.0, -50.0]);
        assert_eq!(l.shapes.len(), 4);
        assert_eq!(l.shapes[0].num("shapeWidth"), Some(1000.0));
        assert_eq!(l.shapes[0].flag("immovable"), Some(true));
        assert_eq!(l.shapes[1].num("shapeWidth"), Some(50.0));
        assert_eq!(l.shapes[3].verts.len(), 3);
        assert_eq!(l.special_count, 1);
    }

    #[test]
    fn parses_joints() {
        let l = parse("<levelXML><info v='1'/><joints><j t='0' x='5' y='6' b1='0' b2='-1' l='t' ua ='1' la ='0' m='f' tq='1' sp='0' c='f'/><j t='1' x='0' y='0' b1='g2' b2='3' a='0' l='f' ul ='1' ll ='0' m='t' fo='1' sp='0' c='t'/></joints></levelXML>").unwrap();
        assert_eq!(l.joints.len(), 2);
        assert_eq!(l.joints[0].body1, BodyRef::Shape(0));
        assert_eq!(l.joints[0].body2, BodyRef::World);
        assert!(l.joints[0].limit && !l.joints[0].sliding);
        assert_eq!((l.joints[0].upper, l.joints[0].lower, l.joints[0].force), (1.0, 0.0, 1.0));
        assert_eq!((l.joints[1].upper, l.joints[1].force, l.joints[1].axis_angle), (1.0, 1.0, 0.0));
        assert_eq!(l.joints[1].body1, BodyRef::Group(2));
        assert_eq!(l.joints[1].body2, BodyRef::Shape(3));
        assert!(l.joints[1].sliding && l.joints[1].motor && l.joints[1].collide_self);
    }

    #[test]
    fn parses_groups_specials_triggers() {
        let l = parse("<levelXML><info v='1'/>\
            <specials><sp t='16' p0='10' p1='20' p3='1'><p2><![CDATA[hello]]></p2></sp></specials>\
            <groups><g x='5' y='6' r='90' ox='1' oy='2' s='f' f='t' o='50' im='f' fr='t' v='t' sb='1' sh='2' ct='3' a='4' l='5' cp='6' lo='t'>\
              <sh t='0' p0='0' p1='0' p2='10' p3='10' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/><sp t='0' p0='1'/></g></groups>\
            <triggers><t x='0' y='0' w='100' h='50' a='0' b='4' t='1' r='3' sd='t' i='2' d='5'><sh i='0'><a i='1' p0='9'/></sh><g i='0'/></t></triggers></levelXML>").unwrap();
        let sp = &l.specials[0];
        assert_eq!(sp.kind, 16);
        assert_eq!(sp.props[0].as_deref(), Some("10"));
        assert_eq!(sp.props[2], None);
        assert_eq!(sp.props[3].as_deref(), Some("1"));
        assert_eq!(sp.texts.get(&2).map(String::as_str), Some("hello"));
        let g = &l.groups[0];
        assert_eq!((g.pos, g.angle, g.opacity, g.foreground, g.fixed_rotation), ([5.0, 6.0], 90.0, 50.0, true, true));
        assert_eq!(g.vehicle.as_ref().unwrap().acceleration, 4.0);
        assert!(g.vehicle.as_ref().unwrap().lock_joints);
        assert_eq!((g.shapes.len(), g.specials.len()), (1, 1));
        let t = &l.triggers[0];
        assert_eq!((t.size, t.type_index, t.repeat_interval, t.delay), ([100.0, 50.0], 1, Some(2.0), Some(5.0)));
        assert!(t.start_disabled);
        assert_eq!(t.targets.len(), 2);
        assert_eq!(t.targets[0].kind, TargetKind::Shape);
        assert_eq!(t.targets[0].actions, vec![vec!["1".to_string(), "9".to_string()]]);
        assert_eq!(t.targets[1].kind, TargetKind::Group);
    }

    #[test]
    fn rejects_wrong_root() {
        assert!(parse("<x/>").is_err());
    }
}

#[cfg(test)]
mod limit_tests {
    /// The running original loads `p4='8'` spikes as 20; shapes are held to their limits the same way.
    #[test]
    fn loaded_parameters_are_held_to_the_originals_limits() {
        let level = super::parse(
            "<levelXML><info v='1.99'/><shapes><sh t='1' p0='0' p1='0' p2='2' p3='300' p4='0' p5='t' p6='f' p7='0' p8='0' p9='-1' p10='100' p11='9'/></shapes><specials><sp t='6' p0='1000' p1='400' p2='0' p3='t' p4='8' p5='f'/></specials></levelXML>",
        )
        .unwrap();
        assert_eq!(level.specials[0].num("numSpikes"), Some(20.0));
        assert_eq!(level.specials[0].num("x"), Some(1000.0));
        // width and height of a circle follow the last one set (5 px at least); density 0.1, collision 7
        let props = &level.shapes[0].props;
        assert_eq!((props[2].as_str(), props[3].as_str(), props[7].as_str(), props[11].as_str()), ("300", "300", "0.1", "7"));
    }
}
