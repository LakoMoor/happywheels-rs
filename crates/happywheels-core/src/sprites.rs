//! Sprite drawing for the vehicle/ragdoll parts. The original keeps one MovieClip per body part (`Character<N>V111`:
//! name, position, rotation, scale and nested children down to texture leaves; dumped by `analysis/clip-dump.js` into
//! `data/clip-c<N>.json`). Each frame the part clip takes the pose of its Box2D body. Here a part is drawn as
//! `G = body_xf · rest_body_xf⁻¹ · rest_part_world` followed by the clip's own nested transforms; textures come from the
//! `@0.3x` atlas (frame sizes relative to `origSize` give the pixel ratio).

use std::{collections::HashMap, path::Path};

use box2d::world::BodyId;
use image::{Rgba, RgbaImage};
use serde::Deserialize;

use crate::physics::World;

#[derive(Debug, Deserialize)]
struct Node {
    n: String,
    x: f64,
    y: f64,
    r: f64,
    sx: f64,
    sy: f64,
    t: Option<String>,
    k: Vec<Node>,
    /// Children of the clip at its other frames (`gotoAndStop(n)`, n ≥ 2) for multi-frame parts.
    #[serde(default)]
    alt: HashMap<String, Vec<Node>>,
}

#[derive(Debug, Deserialize)]
struct AtlasFrame {
    frame: Rect,
    #[serde(rename = "spriteSourceSize")]
    sprite_source: Rect,
    #[serde(rename = "sourceSize")]
    source: Size,
    #[serde(rename = "origSize")]
    orig: Option<Size>,
}

#[derive(Debug, Deserialize, Clone, Copy)]
struct Rect {
    x: f64,
    y: f64,
    w: f64,
    h: f64,
}

#[derive(Debug, Deserialize, Clone, Copy)]
struct Size {
    w: f64,
    h: f64,
}

#[derive(Debug, Deserialize)]
struct AtlasFile {
    frames: HashMap<String, AtlasFrame>,
}

/// 2×3 affine matrix `[a b tx; c d ty]` stored as `[a, b, c, d, tx, ty]` with x' = a·x + c·y + tx.
type M = [f64; 6];

fn mul(p: M, q: M) -> M {
    [
        p[0] * q[0] + p[2] * q[1],
        p[1] * q[0] + p[3] * q[1],
        p[0] * q[2] + p[2] * q[3],
        p[1] * q[2] + p[3] * q[3],
        p[0] * q[4] + p[2] * q[5] + p[4],
        p[1] * q[4] + p[3] * q[5] + p[5],
    ]
}

fn local(x: f64, y: f64, rot: f64, sx: f64, sy: f64) -> M {
    let (s, c) = rot.sin_cos();
    [c * sx, s * sx, -s * sy, c * sy, x, y]
}

fn invert(m: M) -> Option<M> {
    let det = m[0] * m[3] - m[1] * m[2];
    if det.abs() < 1e-12 {
        return None;
    }
    let (a, b, c, d) = (m[3] / det, -m[1] / det, -m[2] / det, m[0] / det);
    Some([a, b, c, d, -(a * m[4] + c * m[5]), -(b * m[4] + d * m[5])])
}

fn body_matrix(world: &World, h: BodyId) -> M {
    let [x, y] = world.body_center_px(h).unwrap_or([0.0, 0.0]);
    local(f64::from(x), f64::from(y), f64::from(world.body_angle(h)), 1.0, 1.0)
}

struct Binding {
    /// Index path to the part clip (one entry for top-level parts, more inside nested clips such as `kid`/`girl`).
    path: Vec<usize>,
    body: BodyId,
    /// `rest_body⁻¹` composed with the part's rest matrix in world px (including the clip's 0.5 scale).
    rest: M,
}

pub struct Sprites {
    clip: Vec<Node>,
    /// Atlas pages (a character has one, or several in `a`/`b`/`c` subfolders): frames and pixels.
    pages: Vec<(HashMap<String, AtlasFrame>, RgbaImage)>,
    bindings: Vec<Binding>,
    /// Clip-vs-physics layout offset (px) used only by `bind_in` (the c10 daughter's frame / son's basket, whose clips do not follow the COM rule).
    shift: [f64; 2],
    /// City backdrop layers (bottom → top, with their parallax multiplier): baked buildings (`drawBuilding`).
    city: Vec<(f64, Vec<Building>)>,
    /// Extra node names hidden for the current `draw` (Santa's `ski`/`stem` children after those parts broke off).
    hidden: std::cell::RefCell<Vec<&'static str>>,
    /// `wound` overlays of the chest / pelvis are drawn only while this is set (see `Ragdoll::part_wound`).
    show_wound: std::cell::Cell<bool>,
}

/// One baked `drawBuilding` result: the top `W` rows drawn once (and again `dup` px to the right) and `k` tiled rows below them.
struct Building {
    x: f64,
    y: f64,
    blur: f64,
    top: RgbaImage,
    tile: RgbaImage,
    dup: Option<f64>,
    top_rows: f64,
    bottom: f64,
}

/// Node names that are hidden at rest (broken/gore states) and not drawn.
const HIDDEN: [&str; 6] = ["broken", "wound", "helmet", "kid", "metalShards", "shape"];

impl Node {
    fn name_is(&self, n: &str) -> bool {
        self.n == n
    }
}

impl Sprites {
    /// Loads the clip dump and atlas of character `index`. `assets` is the original asset root.
    pub fn open(assets: &Path, index: u32) -> Option<Self> {
        let clip: Vec<Node> = serde_json::from_str(match index {
            1 => include_str!("../data/clip-c1.json"),
            2 => include_str!("../data/clip-c2.json"),
            3 => include_str!("../data/clip-c3.json"),
            11 => include_str!("../data/clip-c11.json"),
            10 => include_str!("../data/clip-c10.json"),
            8 => include_str!("../data/clip-c8.json"),
            9 => include_str!("../data/clip-c9.json"),
            7 => include_str!("../data/clip-c7.json"),
            6 => include_str!("../data/clip-c6.json"),
            5 => include_str!("../data/clip-c5.json"),
            4 => include_str!("../data/clip-c4.json"),
            _ => return None,
        })
        .ok()?;
        let dir = assets.join(format!("animate/character{index}"));
        let mut folders = vec![(dir.clone(), format!("character{index}"))];
        for sub in ["a", "b", "c"] {
            folders.push((dir.join(sub), sub.to_string()));
        }
        let mut pages = Vec::new();
        for (folder, stem) in folders {
            let (Ok(json), Ok(img)) = (std::fs::read_to_string(folder.join(format!("{stem}@0.3x.json"))), image::open(folder.join(format!("{stem}@0.3x.png")))) else { continue };
            let Ok(atlas) = serde_json::from_str::<AtlasFile>(&json) else { continue };
            pages.push((atlas.frames, img.to_rgba8()));
        }
        if pages.is_empty() {
            return None;
        }
        // Atlas `game/l` holds the runtime special-element art (spikes, ...).
        for g in ["l", "d"] {
            let game = assets.join(format!("animate/game/{g}"));
            if let (Ok(json), Ok(img)) = (std::fs::read_to_string(game.join(format!("{g}@0.3x.json"))), image::open(game.join(format!("{g}@0.3x.png")))) {
                if let Ok(atlas) = serde_json::from_str::<AtlasFile>(&json) {
                    pages.push((atlas.frames, img.to_rgba8()));
                }
            }
        }
        // Some clips reuse textures that live in another character's atlas (the dirt bike's `metalShards` use `fat-lady-asset-44`): load those pages too.
        let mut missing = Vec::new();
        Self::textures(&clip, &mut missing);
        missing.retain(|t| !pages.iter().any(|(f, _)| f.contains_key(t)));
        for other in (1..=11).filter(|&o| o != index) {
            if missing.is_empty() {
                break;
            }
            let dir = assets.join(format!("animate/character{other}"));
            let mut tries = vec![(dir.clone(), format!("character{other}"))];
            tries.extend(["a", "b", "c"].map(|sub| (dir.join(sub), sub.to_string())));
            for (folder, stem) in tries {
                let Ok(json) = std::fs::read_to_string(folder.join(format!("{stem}@0.3x.json"))) else { continue };
                let Ok(atlas) = serde_json::from_str::<AtlasFile>(&json) else { continue };
                if !missing.iter().any(|t| atlas.frames.contains_key(t)) {
                    continue;
                }
                let Ok(img) = image::open(folder.join(format!("{stem}@0.3x.png"))) else { continue };
                missing.retain(|t| !atlas.frames.contains_key(t));
                pages.push((atlas.frames, img.to_rgba8()));
            }
        }
        Some(Self { clip, pages, bindings: Vec::new(), shift: [0.0, 0.0], city: Vec::new(), hidden: Default::default(), show_wound: Default::default() })
    }

    /// Every texture name used anywhere in `nodes` (including alternate frames).
    fn textures(nodes: &[Node], out: &mut Vec<String>) {
        for n in nodes {
            if let Some(t) = &n.t {
                if !out.contains(t) {
                    out.push(t.clone());
                }
            }
            Self::textures(&n.k, out);
            for alt in n.alt.values() {
                Self::textures(alt, out);
            }
        }
    }

    /// Binds clip part `name` to `body`. The original's `paint` puts the clip at the body's centre of mass with the body's angle, so
    /// no rest layout is needed (`world` and `start` are kept for call-site compatibility).
    pub fn bind(&mut self, _world: &World, name: &str, body: BodyId, _start: [f32; 2]) {
        self.bind_origin(name, body);
    }

    /// Rest-offset binding for a part nested inside clip `parent` (assumed unrotated and unscaled) — the c10 daughter's frame and son's basket sit
    /// at their clip's rest layout, not at their body's centre of mass; `shift` comes from [`set_shift`](Self::set_shift).
    pub fn bind_in(&mut self, world: &World, parent: &str, name: &str, body: BodyId, start: [f32; 2]) {
        let Some(pi) = self.clip.iter().position(|c| c.n == parent) else { return };
        let Some(ci) = self.clip[pi].k.iter().position(|c| c.n == name) else { return };
        let (p, c) = (&self.clip[pi], &self.clip[pi].k[ci]);
        let Some(inv) = invert(body_matrix(world, body)) else { return };
        let origin = local(f64::from(start[0]) - self.shift[0], f64::from(start[1]) - self.shift[1], 0.0, 0.5, 0.5);
        let world_part = mul(origin, mul(local(p.x, p.y, 0.0, 1.0, 1.0), local(c.x, c.y, c.r, c.sx, c.sy)));
        self.bindings.push(Binding { path: vec![pi, ci], body, rest: mul(inv, world_part) });
    }

    /// Measures the clip-vs-physics layout offset from top-level clip `name` and its body (call before `bind_in`).
    fn set_shift(&mut self, world: &World, name: &str, body: BodyId, start: [f32; 2]) {
        let (Some(c), Some(p)) = (self.clip.iter().find(|c| c.n == name), world.body_center_px(body)) else { return };
        self.shift = [f64::from(start[0]) + c.x * 0.5 - f64::from(p[0]), f64::from(start[1]) + c.y * 0.5 - f64::from(p[1])];
    }

    /// Parallax backdrop of the "green hills" level backdrop (`bg='1'`; clips `green_sky_backdrop`, `green_backdrop3/2/1`, bottom to top). The layers
    /// are plain clip trees drawn with [`draw_backdrop`](Self::draw_backdrop); `assets` is the original asset root.
    pub fn open_backdrop(assets: &Path) -> Option<Self> {
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        let layers = [
            include_str!("../data/backdrop-green_sky_backdrop.json"),
            include_str!("../data/backdrop-green_backdrop3.json"),
            include_str!("../data/backdrop-green_backdrop2.json"),
            include_str!("../data/backdrop-green_backdrop1.json"),
        ];
        let clip: Vec<Node> = layers
            .iter()
            .map(|s| serde_json::from_str::<File>(s).ok().map(|f| Node { n: "layer".into(), x: 0.0, y: 0.0, r: 0.0, sx: 1.0, sy: 1.0, t: None, k: f.k, alt: HashMap::new() }))
            .collect::<Option<_>>()?;
        let mut pages = Vec::new();
        for i in 1..=3 {
            let stem = format!("backgrounds_atlas_{i}");
            let folder = assets.join("animate/backgrounds").join(&stem);
            let (Ok(json), Ok(img)) = (std::fs::read_to_string(folder.join(format!("{stem}@0.3x.json"))), image::open(folder.join(format!("{stem}@0.3x.png")))) else { continue };
            let Ok(atlas) = serde_json::from_str::<AtlasFile>(&json) else { continue };
            pages.push((atlas.frames, img.to_rgba8()));
        }
        if pages.is_empty() {
            return None;
        }
        Some(Self { clip, pages, bindings: Vec::new(), shift: [0.0, 0.0], city: Vec::new(), hidden: Default::default(), show_wound: Default::default() })
    }

    /// City backdrop (`bg='2'`): `city_bd1` (fixed sky/clouds) below two layers of baked buildings (`backdrop2`: multiplier .25, blur 5;
    /// `backdrop1`: .5, blur 2), as in `createBackDrops` / `MZ` / `Mg` (deobfuscated.js 48633–48690, 49604).
    pub fn open_city_backdrop(assets: &Path) -> Option<Self> {
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        let load = |s: &str| serde_json::from_str::<File>(s).ok();
        let (l1, l2, sky) = (load(include_str!("../data/backdrop-city-backdrop1.json"))?, load(include_str!("../data/backdrop-city-backdrop2.json"))?, load(include_str!("../data/backdrop-city-city_bd1.json"))?);
        let mut s = Self::open_backdrop(assets)?;
        s.clip = vec![Node { n: "layer".into(), x: 0.0, y: 0.0, r: 0.0, sx: 1.0, sy: 1.0, t: None, k: sky.k, alt: HashMap::new() }];
        // (W, k, duplicate offset) per building b1..b4
        let near = [(90.0, 88.0, Some(5000.0)), (140.0, 90.0, Some(5000.0)), (60.0, 102.0, Some(5000.0)), (160.0, 101.0, Some(5000.0))];
        let far = [(90.0, 88.0, None), (40.0, 50.0, Some(2500.0)), (46.0, 50.0, Some(2500.0)), (15.0, 50.0, Some(2500.0))];
        let b_far = s.bake_buildings(&l2.k, &far, 5.0, 0.25);
        let b_near = s.bake_buildings(&l1.k, &near, 2.0, 0.5);
        s.city = vec![(0.25, b_far), (0.5, b_near)];
        Some(s)
    }

    /// Right/bottom extent of everything drawn under `node` (clip units).
    fn extent(&self, node: &Node, m: M, out: &mut (f64, f64)) {
        let m = mul(m, local(node.x, node.y, node.r, node.sx, node.sy));
        if let Some(f) = node.t.as_ref().and_then(|t| self.pages.iter().find_map(|(fr, _)| fr.get(t))) {
            let (w, h) = (f.orig.map_or(f.source.w, |o| o.w), f.orig.map_or(f.source.h, |o| o.h));
            for (x, y) in [(0.0, 0.0), (w, 0.0), (0.0, h), (w, h)] {
                out.0 = out.0.max(m[0] * x + m[2] * y + m[4]);
                out.1 = out.1.max(m[1] * x + m[3] * y + m[5]);
            }
        }
        for k in &node.k {
            self.extent(k, m, out);
        }
    }

    /// `drawBuilding` for b1..b4 of a layer: rasterise the node, keep its top `W` rows and a `k`-row strip to tile downwards.
    fn bake_buildings(&self, nodes: &[Node], params: &[(f64, f64, Option<f64>); 4], blur: f64, mult: f64) -> Vec<Building> {
        let bottom = mult * 10000.0 + (1.0 - mult) * 500.0;
        let mut out = Vec::new();
        for (name, &(w, k, dup)) in ["b1", "b2", "b3", "b4"].iter().zip(params) {
            let Some(n) = nodes.iter().find(|c| c.n == *name) else { continue };
            let mut ext = (0.0, 0.0);
            for c in &n.k {
                self.extent(c, local(0.0, 0.0, 0.0, 1.0, 1.0), &mut ext);
            }
            let width = ext.0.ceil() as u32 + 2 * blur as u32 + 2;
            let render = |dy: f64, rows: u32| {
                let mut img = RgbaImage::new(width, rows);
                for c in &n.k {
                    self.draw_node(&mut img, c, local(blur, dy, 0.0, 1.0, 1.0));
                }
                blur_horizontal(&img, blur as f32)
            };
            let top = render(blur, (w + blur) as u32);
            let tile = render(-w, k as u32);
            out.push(Building { x: n.x, y: n.y, blur, top, tile, dup, top_rows: w, bottom });
        }
        out
    }

    /// Draws the baked city buildings (call after the fixed `city_bd1` layer).
    fn draw_city(&self, img: &mut RgbaImage, camera: [f32; 2]) {
        let (cx, cy) = (f64::from(img.width()) / 2.0 - f64::from(camera[0]), f64::from(img.height()) / 2.0 - f64::from(camera[1]));
        for (m, buildings) in &self.city {
            let (ox, oy) = ((cx * m).round(), (cy * m).round());
            for b in buildings {
                let (rx, ry) = (ox + b.x - b.blur, oy + b.y - b.blur);
                let xs: Vec<f64> = std::iter::once(0.0).chain(b.dup).collect();
                for dx in xs {
                    composite(img, &b.top, rx + dx, ry);
                    let mut y = ry + b.top_rows + b.blur;
                    let end = oy + b.bottom.min(1e9);
                    while y < end && y < f64::from(img.height()) {
                        if y + f64::from(b.tile.height()) > 0.0 {
                            composite(img, &b.tile, rx + dx, y);
                        }
                        y += f64::from(b.tile.height());
                    }
                }
            }
        }
    }

    /// Draws the backdrop layers behind everything. `adjustBackDrops`: a layer with multiplier m sits at `round(container · m)`, where the world
    /// container is shifted by `(width/2 − camera.x, height/2 − camera.y)`; the sky (multiplier 0) stays fixed to the screen. Layer blur
    /// (3/6/7 px in the original, baked into render textures) is not applied.
    pub fn draw_backdrop(&self, img: &mut RgbaImage, camera: [f32; 2]) {
        const MULTIPLIERS: [f64; 4] = [0.0, 0.01, 0.05, 0.1];
        let (cx, cy) = (f64::from(img.width()) / 2.0 - f64::from(camera[0]), f64::from(img.height()) / 2.0 - f64::from(camera[1]));
        // Flash `BlurFilter` strengths of the layers (sky has none); blur is applied as a gaussian with the variance of `quality` (3) box passes.
        const BLUR: [f32; 4] = [0.0, 7.0, 6.0, 3.0];
        for ((layer, m), blur) in self.clip.iter().zip(MULTIPLIERS).zip(BLUR) {
            let g = local((cx * m).round(), (cy * m).round(), 0.0, 1.0, 1.0);
            if blur == 0.0 {
                for child in &layer.k {
                    self.draw_node(img, child, g);
                }
                continue;
            }
            let mut tmp = RgbaImage::new(img.width(), img.height());
            for child in &layer.k {
                self.draw_node(&mut tmp, child, g);
            }
            let sigma = (3.0 * (blur * blur - 1.0) / 12.0).sqrt();
            let tmp = imageproc::filter::gaussian_blur_f32(&tmp, sigma);
            for (x, y, p) in tmp.enumerate_pixels() {
                if p[3] == 0 {
                    continue;
                }
                let a = f32::from(p[3]) / 255.0;
                let d = img.get_pixel_mut(x, y);
                for i in 0..3 {
                    d[i] = (f32::from(p[i]) * a + f32::from(d[i]) * (1.0 - a)) as u8;
                }
            }
        }
        if !self.city.is_empty() {
            self.draw_city(img, camera);
        }
    }

    /// Binds a part whose clip origin is the body origin (ragdoll limbs, wheels): the clip takes the body's position and angle, like the original.
    pub fn bind_origin(&mut self, name: &str, body: BodyId) {
        let Some(part) = self.clip.iter().position(|c| c.name_is(name)) else { return };
        let c = &self.clip[part];
        self.bindings.push(Binding { path: vec![part], body, rest: local(0.0, 0.0, 0.0, 0.5 * c.sx, 0.5 * c.sy) });
    }

    /// [`bind_origin`](Self::bind_origin) for a part nested one level down inside clip `parent` (`kid`, `girl`).
    pub fn bind_origin_in(&mut self, parent: &str, name: &str, body: BodyId) {
        let Some(pi) = self.clip.iter().position(|c| c.n == parent) else { return };
        let Some(ci) = self.clip[pi].k.iter().position(|c| c.n == name) else { return };
        let c = &self.clip[pi].k[ci];
        self.bindings.push(Binding { path: vec![pi, ci], body, rest: local(0.0, 0.0, 0.0, 0.5 * c.sx, 0.5 * c.sy) });
    }

    /// Depth-first path to the first clip called `name` (skipping `shapeGuide`).
    fn find_path(nodes: &[Node], name: &str, prefix: &mut Vec<usize>) -> Option<Vec<usize>> {
        for (i, n) in nodes.iter().enumerate() {
            if n.n == "shapeGuide" {
                continue;
            }
            prefix.push(i);
            if n.n == name {
                return Some(prefix.clone());
            }
            if let Some(p) = Self::find_path(&n.k, name, prefix) {
                return Some(p);
            }
            prefix.pop();
        }
        None
    }

    /// [`bind_origin`](Self::bind_origin) for a part found anywhere in the clip tree (the pogo man is one assembled skeleton).
    pub fn bind_origin_deep(&mut self, name: &str, body: BodyId) {
        let Some(path) = Self::find_path(&self.clip, name, &mut Vec::new()) else { return };
        let mut node = &self.clip[path[0]];
        for &i in &path[1..] {
            node = &node.k[i];
        }
        let rest = local(0.0, 0.0, 0.0, 0.5 * node.sx, 0.5 * node.sy);
        self.bindings.push(Binding { path, body, rest });
    }

    /// Binds the eleven limbs of `doll` to the clips named like the original's parts, inside `parent` when given.
    fn bind_ragdoll(&mut self, doll: &crate::character::Ragdoll, parent: Option<&str>) {
        use crate::character::Part;
        let pairs = [
            (Part::Chest, "chest"),
            (Part::Head, "head"),
            (Part::Pelvis, "pelvis"),
            (Part::UpperArm1, "upperArm1"),
            (Part::UpperArm2, "upperArm2"),
            (Part::LowerArm1, "lowerArm1"),
            (Part::LowerArm2, "lowerArm2"),
            (Part::UpperLeg1, "upperLeg1"),
            (Part::UpperLeg2, "upperLeg2"),
            (Part::LowerLeg1, "lowerLeg1"),
            (Part::LowerLeg2, "lowerLeg2"),
        ];
        for (p, n) in pairs {
            if let Some(&b) = doll.bodies.get(&p) {
                match parent {
                    Some("*") => self.bind_origin_deep(n, b),
                    Some(par) => self.bind_origin_in(par, n, b),
                    None => self.bind_origin(n, b),
                }
            }
        }
    }

    /// Binds all parts of the rider and its vehicle (call right after `spawn_bike`, before the first step).
    pub fn bind_vehicle_rider(&mut self, world: &World, start: [f32; 2]) {
        use crate::vehicle::Vehicle;
        let Some(doll) = world.character.as_ref() else { return };
        if !matches!(world.bike.as_ref(), Some(Vehicle::Pogo(_))) {
            self.bind_ragdoll(doll, None);
        }
        match world.bike.as_ref() {
            Some(Vehicle::Bike(b)) => {
                if let Some(k) = b.kid.as_ref() {
                    self.bind_ragdoll(&k.doll, Some("kid"));
                }
            }
            Some(Vehicle::Dirt(d)) => {
                if let Some(g) = d.girl.as_ref() {
                    self.bind_ragdoll(&g.doll, Some("girl"));
                }
            }
            _ => {}
        }
        if let Some(Vehicle::Heli(h)) = world.bike.as_ref() {
            let (copter, magnet) = (h.copter, h.magnet);
            self.bind(world, "copter", copter, start);
            self.bind(world, "copterFront", copter, start);
            self.bind(world, "magnet", magnet, start);
        }
        if let Some(Vehicle::Pogo(p)) = world.bike.as_ref() {
            // The pogo man is one assembled skeleton clip whose origin is the chest guide (the intact `frame` clip at (199, 262) stands for the guide's frameShape at (28.85, 107.45), which fixes the layout shift).
            self.bind_ragdoll(doll, Some("*"));
            let (frame, rod) = (p.frame, p.rod);
            self.bind(world, "frame", frame, start);
            self.bind(world, "rod", rod, start);
        }
        if let Some(Vehicle::Santa(s)) = world.bike.as_ref() {
            self.bind_ragdoll(&s.elf1.doll, Some("elf1"));
            self.bind_ragdoll(&s.elf2.doll, Some("elf2"));
            let (sleigh, boxes) = (s.sleigh, s.boxes.clone());
            self.bind(world, "sleigh", sleigh, start);
            for (i, b) in boxes.into_iter().enumerate() {
                self.bind_origin(&format!("box{}", i + 1), b);
            }
        }
        if let Some(Vehicle::Family(f)) = world.bike.as_ref() {
            let (d, s) = (&f.daughter, &f.son);
            self.bind_ragdoll(&d.doll, Some("daughter"));
            self.bind_ragdoll(&s.doll, Some("son"));
            let (frame, wheel, gear, basket) = (d.frame, d.wheel, d.gear, s.basket);
            self.bind_origin_in("daughter", "wheel", wheel);
            self.bind_origin_in("daughter", "gear", gear);
            if let Some(d) = Some(&f.dad) {
                self.set_shift(world, "backWheel", d.back_wheel, start);
            }
            self.bind_in(world, "daughter", "frame", frame, start);
            self.bind_in(world, "son", "basket", basket, start);
        }
        let bike = match world.bike.as_ref() {
            Some(Vehicle::Bike(b)) => Some(b),
            Some(Vehicle::Family(f)) => Some(&f.dad),
            _ => None,
        };
        if let Some(bike) = bike {
            let (frame, back, front, gear) = (bike.frame, bike.back_wheel, bike.front_wheel, bike.gear);
            // `fork`/`seat` top-level clips are the frameSmash pieces; the intact frame draws them inside `frame`.
            self.bind(world, "frame", frame, start);
            self.bind_origin("backWheel", back);
            self.bind_origin("frontWheel", front);
            self.bind_origin("gear", gear);
        }
        match world.bike.as_ref() {
            Some(Vehicle::Dirt(d)) => {
                let (frame, back, front) = (d.frame, d.back_wheel, d.front_wheel);
                self.bind(world, "frame", frame, start);
                self.bind_origin("backWheel", back);
                self.bind_origin("frontWheel", front);
            }
            Some(Vehicle::Mower(m)) => {
                let (body, back, front) = (m.body, m.back_wheel, m.front_wheel);
                self.bind(world, "mower", body, start);
                self.bind_origin("backWheel", back);
                self.bind_origin("frontWheel", front);
            }
            Some(Vehicle::Grinder(g)) => {
                let (frame, back, front) = (g.frame, g.back_wheel, g.front_wheel);
                self.bind(world, "frame", frame, start);
                self.bind_origin("backWheel", back);
                self.bind_origin("frontWheel", front);
            }
            Some(Vehicle::Cart(c)) => {
                let (body, shaft, back, front) = (c.body, c.shaft, c.back_wheel, c.front_wheel);
                self.bind(world, "main", body, start);
                self.bind(world, "shaft", shaft, start);
                let boxes = c.groceries.clone();
                for (i, b) in boxes.into_iter().enumerate() {
                    self.bind_origin(&format!("box{i}"), b);
                }
                self.bind_origin("backWheel", back);
                self.bind_origin("frontWheel", front);
            }
            _ => {}
        }
        if let Some(Vehicle::Segway(s)) = world.bike.as_ref() {
            let (frame, wheel) = (s.frame, s.wheel);
            self.bind(world, "frame", frame, start);
            self.bind(world, "wheelCover", frame, start);
            self.bind_origin("wheel", wheel);
        }
        if let Some(Vehicle::Chair(c)) = world.bike.as_ref() {
            let (chair, big, small) = (c.chair, c.big_wheel, c.small_wheel);
            // `handle`/`fueltank`/`jet`/`metalShards` top-level clips are the chairSmash pieces.
            self.bind(world, "chair", chair, start);
            self.bind_origin("bigWheel", big);
            self.bind_origin("smallWheel", small);
        }
    }

    /// True if `body` is drawn as a sprite (the polygon fallback skips it).
    pub fn covers(&self, body: BodyId) -> bool {
        self.bindings.iter().any(|b| b.body == body)
    }

    /// Draws every bound part plus the debris a smash leaves (chunks, brain, heart, helmet); `camera_offset` translates world px to image px.
    /// True when special element `kind` has sprite art here (and so replaces the grey fallback).
    pub fn covers_special(&self, kind: usize) -> bool {
        let tex = match kind {
            6 => "spike-clean.png",
            3 => "i-beam-body.png",
            4 => "log-full.png",
            _ => return false,
        };
        self.pages.iter().any(|(f, _)| f.contains_key(tex))
    }

    /// Spikes (`Spikes.setupMC`, `paint`): clip at the body's centre of mass; `spikes` row at (−halfWidth, −25) and `base` (`spike-platform`, stretched
    /// to 15·n−2) at (0, 35), both lowered by the body's local-centre y; one `spike-clean` per 15 px. NOT verified: clips with fewer than 20 spikes
    /// (the library clip has 20 children; the original's `setupMC` adds more only above 20).
    pub fn draw_specials(&self, img: &mut RgbaImage, level: &crate::level::Level, world: &World, camera_offset: (f32, f32)) {
        let cam = local(f64::from(camera_offset.0), f64::from(camera_offset.1), 0.0, 1.0, 1.0);
        for (i, sp) in level.specials.iter().enumerate() {
            if !matches!(sp.kind, 3 | 4 | 6) {
                continue;
            }
            let Some(body) = world.special_bodies.get(i).copied().flatten() else { continue };
            if world.body_center_px(body).is_none() {
                continue;
            }
            // IBeamMC / LogMC: one texture, the clip is stretched to shapeWidth × shapeHeight (`mc.width/height`); clip bounds 400×32 / 36×400.
            if sp.kind != 6 {
                let (tex, cw, ch) = if sp.kind == 3 { ("i-beam-body.png", 400.0, 32.0) } else { ("log-full.png", 36.0, 400.0) };
                let (sw, sh) = (f64::from(sp.num("shapeWidth").unwrap_or(cw as f32)), f64::from(sp.num("shapeHeight").unwrap_or(ch as f32)));
                let m = mul(cam, mul(body_matrix(world, body), local(0.0, 0.0, 0.0, sw / cw, sh / ch)));
                self.blit(img, tex, mul(m, local(-cw / 2.0, -ch / 2.0, 0.0, 0.5, 0.5)));
                continue;
            }
            let n = sp.num("numSpikes").unwrap_or(3.0).max(1.0) as usize;
            let lc_y = world.b2().body(body).sweep.local_center.y * 62.5;
            let m = mul(cam, body_matrix(world, body));
            let half = 7.5 * n as f64;
            for k in 0..n {
                self.blit(img, "spike-clean.png", mul(m, mul(local(-half + 15.0 * k as f64, -25.0 - lc_y, 0.0, 1.0, 1.0), local(0.0, 0.0, 0.0, 0.333, 0.333))));
            }
            let width = 15.0 * n as f64 - 2.0;
            self.blit(img, "spike-platform.png", mul(m, mul(local(0.0, 35.0 - lc_y, 0.0, 1.0, 1.0), local(-width / 2.0, -10.0, 0.0, width / 300.0, 1.0))));
        }
    }

    pub fn draw(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32)) {
        {
            let mut h = self.hidden.borrow_mut();
            h.clear();
            if let Some(crate::vehicle::Vehicle::Santa(s)) = world.bike.as_ref() {
                if s.wheels_gone {
                    h.push("ski");
                }
                if s.stem_gone {
                    h.push("stem");
                }
            }
        }
        for b in &self.bindings {
            self.draw_binding(img, world, camera_offset, b);
        }
        let bike_pieces = match world.bike.as_ref() {
            Some(crate::vehicle::Vehicle::Bike(b)) => Some(&b.pieces),
            Some(crate::vehicle::Vehicle::Family(f)) => Some(&f.dad.pieces),
            _ => None,
        };
        if let Some(pieces) = bike_pieces {
            for (&body, name) in pieces.iter().zip(["fork", "brokenFrame", "seat"]) {
                self.draw_loose(img, world, camera_offset, name, body);
            }
        }
        match world.bike.as_ref() {
            // `DirtBike::pieces` follow the shape-target order [4, 3, 2, 5, 1, 0] of the creation order fork, tank, engine, middle, rear, seat.
            Some(crate::vehicle::Vehicle::Dirt(d)) => {
                for (&body, name) in d.pieces.iter().zip(["rear", "middle", "engine", "seat", "tank", "fork"]) {
                    self.draw_loose(img, world, camera_offset, name, body);
                }
            }
            // chairSmash debris: the rod is whichever debris body is neither the tank nor the jet (those two get cleared when they blow up).
            Some(crate::vehicle::Vehicle::Chair(c)) => {
                for &body in &c.debris {
                    let name = if Some(body) == c.tank {
                        "fueltank"
                    } else if Some(body) == c.jet {
                        "jet"
                    } else {
                        "handle"
                    };
                    self.draw_loose(img, world, camera_offset, name, body);
                }
            }
            Some(crate::vehicle::Vehicle::Heli(h)) => {
                for (&body, name) in h.debris.iter().zip(&h.debris_names) {
                    self.draw_loose(img, world, camera_offset, name, body);
                }
            }
            Some(crate::vehicle::Vehicle::Mower(m)) => {
                if let Some(halves) = m.halves {
                    for (body, name) in halves.into_iter().zip(["mowerFront", "mowerRear"]) {
                        self.draw_loose(img, world, camera_offset, name, body);
                    }
                }
            }
            // `mainSmash` pieces are the body shapes newest first: seat-side order is verified in the cart_smash screenshot.
            Some(crate::vehicle::Vehicle::Cart(c)) => {
                for (&body, name) in c.pieces.iter().zip(["mainRear", "mainSeat", "mainBase", "mainFront"]) {
                    self.draw_loose(img, world, camera_offset, name, body);
                }
            }
            Some(crate::vehicle::Vehicle::Santa(s)) => {
                for (&body, name) in s.debris.iter().zip(&s.debris_names) {
                    self.draw_loose(img, world, camera_offset, name, body);
                }
            }
            Some(crate::vehicle::Vehicle::Pogo(p)) => {
                for (&body, name) in p.pieces.iter().zip(["brokenframe1", "brokenframe2", "spring"]) {
                    self.draw_loose(img, world, camera_offset, name, body);
                }
            }
            _ => {}
        }
        if let Some(doll) = world.character.as_ref() {
            self.draw_doll_extras(img, world, camera_offset, doll, None);
        }
        self.draw_shards(img, world, camera_offset);
        if let Some(crate::vehicle::Vehicle::Family(f)) = world.bike.as_ref() {
            for (&body, name) in f.daughter.debris.iter().zip(["fork", "brokenFrame", "seat"]) {
                self.draw_loose_in(img, world, camera_offset, "daughter", name, body);
            }
        }
        for name in ["kid", "girl", "daughter", "son", "elf1", "elf2"] {
            if let Some(doll) = Self::npc_doll(world, name) {
                self.draw_doll_extras(img, world, camera_offset, doll, Some(name));
            }
        }
    }

    /// Sprite particles of the shard bursts (`createPointBurst`): the particle's frame of its debris clip, centred on the particle, natural clip scale.
    fn draw_shards(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32)) {
        for p in &world.blood.sprites {
            let (name, parent, _) = p.kind.clip();
            let nodes = match parent {
                Some(par) => self.clip.iter().find(|c| c.n == par).map(|c| &c.k),
                None => Some(&self.clip),
            };
            let Some(node) = nodes.and_then(|n| n.iter().find(|c| c.n == name)) else { continue };
            let children = if p.frame > 0 { node.alt.get(&(p.frame + 1).to_string()).unwrap_or(&node.k) } else { &node.k };
            let mut ext = (0.0, 0.0);
            for c in children {
                self.extent(c, local(0.0, 0.0, 0.0, 1.0, 1.0), &mut ext);
            }
            let m = local(p.pos[0] - ext.0 / 2.0 + f64::from(camera_offset.0), p.pos[1] - ext.1 / 2.0 + f64::from(camera_offset.1), 0.0, 1.0, 1.0);
            for c in children {
                self.draw_node(img, c, m);
            }
        }
    }

    /// The ragdoll of the NPC clip `name` (`kid`, `girl`, `daughter`, `son`, `elf1`, `elf2`) of the current vehicle.
    fn npc_doll<'a>(world: &'a World, name: &str) -> Option<&'a crate::character::Ragdoll> {
        use crate::vehicle::Vehicle;
        match (world.bike.as_ref()?, name) {
            (Vehicle::Bike(b), "kid") => b.kid.as_ref().map(|k| &k.doll),
            (Vehicle::Dirt(d), "girl") => d.girl.as_ref().map(|g| &g.doll),
            (Vehicle::Family(f), "daughter") => Some(&f.daughter.doll),
            (Vehicle::Family(f), "son") => Some(&f.son.doll),
            (Vehicle::Santa(s), "elf1") => Some(&s.elf1.doll),
            (Vehicle::Santa(s), "elf2") => Some(&s.elf2.doll),
            _ => None,
        }
    }

    /// Debris of a ragdoll (chunks, brain, heart, spine / intestine chains, feet, helmet, severed stumps), inside clip `parent` for NPCs.
    fn draw_doll_extras(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32), doll: &crate::character::Ragdoll, parent: Option<&str>) {
        let mut extras: Vec<(&str, BodyId)> = doll.chunks.iter().zip(&doll.chunk_names).map(|(&b, n)| (n.as_str(), b)).collect();
        extras.extend(doll.brain.map(|b| ("brain", b)));
        extras.extend(doll.heart.map(|b| ("heart", b)));
        let spine_names: Vec<String> = (1..=doll.spine.as_ref().map_or(0, |c| c.bodies.len())).map(|i| format!("spine{i}")).collect();
        let gut_names: Vec<String> = (1..=doll.intestines.as_ref().map_or(0, |c| c.bodies.len())).map(|i| format!("intestine{i}")).collect();
        extras.extend(doll.spine.iter().flat_map(|c| c.bodies.iter()).zip(&spine_names).map(|(&b, n)| (n.as_str(), b)));
        extras.extend(doll.intestines.iter().flat_map(|c| c.bodies.iter()).zip(&gut_names).map(|(&b, n)| (n.as_str(), b)));
        extras.extend(doll.helmet.map(|b| ("helmet", b)));
        extras.extend(doll.feet[0].map(|b| ("foot1", b)));
        extras.extend(doll.feet[1].map(|b| ("foot2", b)));
        for (&body, limb) in doll.sev.iter().zip(&doll.sev_limbs) {
            use crate::character::Limb;
            let name = match limb {
                Limb::Shoulder1 => "upperArm3",
                Limb::Shoulder2 => "upperArm4",
                Limb::Hip1 => "upperLeg3",
                _ => "upperLeg4",
            };
            extras.push((name, body));
        }
        for (name, body) in extras {
            match parent {
                Some(par) => self.draw_loose_in(img, world, camera_offset, par, name, body),
                None => self.draw_loose(img, world, camera_offset, name, body),
            }
        }
    }
    /// [`draw_loose`](Self::draw_loose) for a clip nested in `parent`.
    fn draw_loose_in(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32), parent: &str, name: &str, body: BodyId) {
        let Some(pi) = self.clip.iter().position(|c| c.n == parent) else { return };
        let Some(ci) = self.clip[pi].k.iter().position(|c| c.n == name) else { return };
        let c = &self.clip[pi].k[ci];
        let b = Binding { path: vec![pi, ci], body, rest: local(0.0, 0.0, 0.0, 0.5 * c.sx, 0.5 * c.sy) };
        self.draw_binding(img, world, camera_offset, &b);
    }

    /// Draws top-level clip `name` on a body created after the bindings (debris, break pieces).
    fn draw_loose(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32), name: &str, body: BodyId) {
        let Some(part) = self.clip.iter().position(|c| c.n == name) else { return };
        let c = &self.clip[part];
        let b = Binding { path: vec![part], body, rest: local(0.0, 0.0, 0.0, 0.5 * c.sx, 0.5 * c.sy) };
        self.draw_binding(img, world, camera_offset, &b);
    }

    fn draw_binding(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32), b: &Binding) {
        if world.body_center_px(b.body).is_none() {
            return;
        }
        // `skiSmash` on an intact sleigh re-centres the clip (`sleighMC.inner` follows the new local centre): keep it fixed to the body origin.
        let recentre = match world.bike.as_ref() {
            Some(crate::vehicle::Vehicle::Santa(s)) if s.sleigh == b.body => {
                let lc = world.b2().body(b.body).sweep.local_center;
                local((s.sleigh_lc0.x - lc.x) * 62.5, (s.sleigh_lc0.y - lc.y) * 62.5, 0.0, 1.0, 1.0)
            }
            _ => local(0.0, 0.0, 0.0, 1.0, 1.0),
        };
        let g = mul(local(f64::from(camera_offset.0), f64::from(camera_offset.1), 0.0, 1.0, 1.0), mul(body_matrix(world, b.body), mul(recentre, b.rest)));
        let mut node = &self.clip[b.path[0]];
        for &i in &b.path[1..] {
            node = &node.k[i];
        }
        // Top-level rider parts switch frames when limbs break (`gotoAndStop`); NPC / nested parts stay on frame 1.
        let owner = match b.path.len() {
            1 => world.character.as_ref(),
            2 => Self::npc_doll(world, &self.clip[b.path[0]].n),
            _ => None,
        };
        let mut frame = owner.map_or(1, |d| d.part_frame(&node.n));
        if b.path.len() == 1 && node.n == "box3" && matches!(world.bike.as_ref(), Some(crate::vehicle::Vehicle::Cart(c)) if c.cola_open) {
            frame = 2;
        }
        let children = if frame > 1 { node.alt.get(&frame.to_string()).unwrap_or(&node.k) } else { &node.k };
        self.show_wound.set(owner.is_some_and(|d| d.part_wound(&node.n)));
        for child in children {
            self.draw_node(img, child, g);
        }
        self.show_wound.set(false);
    }

    fn draw_node(&self, img: &mut RgbaImage, n: &Node, parent: M) {
        if (HIDDEN.contains(&n.n.as_str()) && !(n.n == "wound" && self.show_wound.get())) || self.hidden.borrow().contains(&n.n.as_str()) {
            return;
        }
        let m = mul(parent, local(n.x, n.y, n.r, n.sx, n.sy));
        if let Some(t) = &n.t {
            self.blit(img, t, m);
        }
        for k in &n.k {
            self.draw_node(img, k, m);
        }
    }

    fn blit(&self, img: &mut RgbaImage, name: &str, m: M) {
        let Some((f, image)) = self.pages.iter().find_map(|(frames, image)| frames.get(name).map(|f| (f, image))) else { return };
        // Leaf textures are authored at 3× (scale .33); frame pixels are `ratio` of the original source pixels.
        let ratio = f.orig.map_or(1.0, |o| f.source.w / o.w);
        let ratio_y = f.orig.map_or(1.0, |o| f.source.h / o.h);
        let (w, h) = (f.sprite_source.w, f.sprite_source.h);
        // texture px (small atlas) → source px (÷ratio) → clip transform
        let full = mul(m, local(f.sprite_source.x / ratio, f.sprite_source.y / ratio_y, 0.0, 1.0 / ratio, 1.0 / ratio_y));
        let Some(inv) = invert(full) else { return };
        let corners = [(0.0, 0.0), (w, 0.0), (0.0, h), (w, h)].map(|(x, y)| (full[0] * x + full[2] * y + full[4], full[1] * x + full[3] * y + full[5]));
        let min_x = corners.iter().map(|c| c.0).fold(f64::MAX, f64::min).floor().max(0.0) as i64;
        let max_x = corners.iter().map(|c| c.0).fold(f64::MIN, f64::max).ceil().min(f64::from(img.width())) as i64;
        let min_y = corners.iter().map(|c| c.1).fold(f64::MAX, f64::min).floor().max(0.0) as i64;
        let max_y = corners.iter().map(|c| c.1).fold(f64::MIN, f64::max).ceil().min(f64::from(img.height())) as i64;
        for py in min_y..max_y {
            for px in min_x..max_x {
                let (sx, sy) = (px as f64 + 0.5, py as f64 + 0.5);
                let u = inv[0] * sx + inv[2] * sy + inv[4];
                let v = inv[1] * sx + inv[3] * sy + inv[5];
                if u < 0.0 || v < 0.0 || u >= w || v >= h {
                    continue;
                }
                let src = Self::sample(image, f.frame.x + u, f.frame.y + v);
                if src[3] == 0 {
                    continue;
                }
                let dst = img.get_pixel_mut(px as u32, py as u32);
                let a = f32::from(src[3]) / 255.0;
                let da = f32::from(dst[3]) / 255.0;
                let out = a + da * (1.0 - a);
                for i in 0..3 {
                    dst[i] = ((f32::from(src[i]) * a + f32::from(dst[i]) * da * (1.0 - a)) / out) as u8;
                }
                dst[3] = (out * 255.0).round() as u8;
            }
        }
    }

    fn sample(image: &RgbaImage, x: f64, y: f64) -> Rgba<u8> {
        let (x, y) = (x - 0.5, y - 0.5);
        let (x0, y0) = (x.floor(), y.floor());
        let (fx, fy) = ((x - x0) as f32, (y - y0) as f32);
        let get = |xx: f64, yy: f64| *image.get_pixel((xx.max(0.0) as u32).min(image.width() - 1), (yy.max(0.0) as u32).min(image.height() - 1));
        let (p00, p10, p01, p11) = (get(x0, y0), get(x0 + 1.0, y0), get(x0, y0 + 1.0), get(x0 + 1.0, y0 + 1.0));
        let mut out = [0u8; 4];
        for i in 0..4 {
            let top = f32::from(p00[i]) * (1.0 - fx) + f32::from(p10[i]) * fx;
            let bot = f32::from(p01[i]) * (1.0 - fx) + f32::from(p11[i]) * fx;
            out[i] = (top * (1.0 - fy) + bot * fy) as u8;
        }
        Rgba(out)
    }
}

/// Horizontal-only gaussian with the variance of three box passes of width `blur` (`BlurFilter(blur, 0, 3)`).
fn blur_horizontal(img: &RgbaImage, blur: f32) -> RgbaImage {
    let sigma = (3.0 * (blur * blur - 1.0) / 12.0).sqrt();
    if sigma < 0.3 {
        return img.clone();
    }
    let r = (sigma * 3.0).ceil() as i32;
    let kernel: Vec<f32> = (-r..=r).map(|i| (-(i * i) as f32 / (2.0 * sigma * sigma)).exp()).collect();
    let sum: f32 = kernel.iter().sum();
    let mut out = RgbaImage::new(img.width(), img.height());
    for y in 0..img.height() {
        for x in 0..img.width() {
            let mut acc = [0.0f32; 4];
            for (k, w) in kernel.iter().enumerate() {
                let sx = (x as i32 + k as i32 - r).clamp(0, img.width() as i32 - 1) as u32;
                let p = img.get_pixel(sx, y);
                let a = f32::from(p[3]);
                for c in 0..3 {
                    acc[c] += f32::from(p[c]) * a * w;
                }
                acc[3] += a * w;
            }
            let a = acc[3] / sum;
            if a > 0.0 {
                out.put_pixel(x, y, Rgba([(acc[0] / acc[3]) as u8, (acc[1] / acc[3]) as u8, (acc[2] / acc[3]) as u8, a as u8]));
            }
        }
    }
    out
}

/// Alpha-composites `src` onto `dst` with its top-left corner at (`x`, `y`).
fn composite(dst: &mut RgbaImage, src: &RgbaImage, x: f64, y: f64) {
    let (x0, y0) = (x.round() as i64, y.round() as i64);
    for (sx, sy, p) in src.enumerate_pixels() {
        if p[3] == 0 {
            continue;
        }
        let (dx, dy) = (x0 + i64::from(sx), y0 + i64::from(sy));
        if dx < 0 || dy < 0 || dx >= i64::from(dst.width()) || dy >= i64::from(dst.height()) {
            continue;
        }
        let d = dst.get_pixel_mut(dx as u32, dy as u32);
        let a = f32::from(p[3]) / 255.0;
        for c in 0..3 {
            d[c] = (f32::from(p[c]) * a + f32::from(d[c]) * (1.0 - a)) as u8;
        }
        d[3] = 255;
    }
}
