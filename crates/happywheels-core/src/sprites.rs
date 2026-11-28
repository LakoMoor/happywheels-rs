//! Sprite drawing for the vehicle/ragdoll parts. The original keeps one MovieClip per body part (`Character<N>V111`:
//! name, position, rotation, scale and nested children down to texture leaves; dumped by `analysis/clip-dump.js` into
//! `data/clip-c<N>.json`). Each frame the part clip takes the pose of its Box2D body. Here a part is drawn as
//! `G = body_xf · rest_body_xf⁻¹ · rest_part_world` followed by the clip's own nested transforms; textures come from the
//! `@0.6x` atlas (frame sizes relative to `origSize` give the pixel ratio).

use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

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
    /// Pixi skew (radians): Flash exports some rotations as skew.
    #[serde(default)]
    kx: f64,
    #[serde(default)]
    ky: f64,
    k: Vec<Node>,
    /// Children of the clip at its other frames (`gotoAndStop(n)`, n ≥ 2) for multi-frame parts.
    #[serde(default)]
    alt: HashMap<String, Vec<Node>>,
    /// Invisible when the original has just built the character (`data/runtime-clips.json`).
    #[serde(skip)]
    hidden: bool,
}

/// What the original changes in a character's clips at run time, dumped from its display tree: the layer order of the parts
/// (`addChildAt` in `createMovieClips`), nodes hidden at start and the positions it gives the `inner` nodes of vehicle bodies.
#[derive(Debug, Deserialize)]
struct RuntimeClip {
    order: Vec<String>,
    hidden: Vec<String>,
    pos: HashMap<String, [f64; 2]>,
}

fn apply_runtime(nodes: &mut [Node], prefix: &str, runtime: &RuntimeClip) {
    for n in nodes {
        let path = if prefix.is_empty() { n.n.clone() } else { format!("{prefix}/{}", n.n) };
        if runtime.hidden.contains(&path) {
            n.hidden = true;
        }
        if let Some(&[x, y]) = runtime.pos.get(&path) {
            (n.x, n.y) = (x, y);
        }
        apply_runtime(&mut n.k, &path, runtime);
        // the same instances stay hidden on the clip's other frames (`chest.neck`)
        for frame in n.alt.values_mut() {
            apply_runtime(frame, &path, runtime);
        }
    }
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

/// Asset files shared by desktop filesystem loading and browser-provided files.
#[derive(Clone)]
pub enum SpriteAssets {
    Directory(PathBuf),
    Memory(std::rc::Rc<HashMap<PathBuf, Vec<u8>>>),
}

impl SpriteAssets {
    pub fn memory(files: HashMap<PathBuf, Vec<u8>>) -> Self {
        Self::Memory(std::rc::Rc::new(files))
    }

    fn root(&self) -> &Path {
        match self { Self::Directory(path) => path, Self::Memory(_) => Path::new("") }
    }

    fn read(&self, path: &Path) -> Option<Vec<u8>> {
        match self { Self::Directory(_) => std::fs::read(path).ok(), Self::Memory(files) => files.get(path).cloned() }
    }

    fn string(&self, path: &Path) -> Option<String> {
        String::from_utf8(self.read(path)?).ok()
    }

    fn image(&self, path: &Path) -> Option<image::DynamicImage> {
        image::load_from_memory(&self.read(path)?).ok()
    }
}

#[derive(Clone)]
struct Binding {
    /// Placement the original computes itself (gore chain clips): position px, rotation, stretch of the `inner` node.
    pose: Option<([f64; 2], f64, f64)>,
    /// Draw only this nested clip (as its own layer), or everything but it.
    only: Option<&'static str>,
    without: Option<&'static str>,
    /// Index path to the part clip (one entry for top-level parts, more inside nested clips such as `kid`/`girl`).
    path: Vec<usize>,
    body: BodyId,
    /// `rest_body⁻¹` composed with the part's rest matrix in world px (including the clip's 0.5 scale).
    rest: M,
}

/// Atlas size used. The original defaults to the full-size one; this renderer samples bilinearly without mipmaps, so it takes the 0.6× one,
/// which at the 2× view is close to one texel per pixel.
const ATLAS: &str = "@0.6x";

/// One drawn texture: its name and where the untrimmed texture's top-left, top-right and bottom-left corners land.
pub type DrawnTexture = (String, [[f64; 2]; 3]);

/// A blurred backdrop layer as rendered when its offset was `at`.
struct Blurred {
    at: (i64, i64),
    image: RgbaImage,
}

pub struct Sprites {
    blurred: std::cell::RefCell<Vec<Option<Blurred>>>,
    /// When set, every texture drawn is recorded here (compared with the original's display list in tests).
    pub trace: std::cell::RefCell<Option<Vec<DrawnTexture>>>,
    clip: Vec<Node>,
    /// Image pixels per world pixel (0 = unset = 1).
    pub view: std::cell::Cell<f64>,
    /// Layer of each part clip (`name`, or `npc/name`), bottom first.
    layers: HashMap<String, usize>,
    /// Hidden node names to show while the current binding is drawn.
    shown: std::cell::RefCell<Vec<&'static str>>,
    /// Parts collected by `draw` to be painted in layer order.
    queue: std::cell::RefCell<Option<Vec<Binding>>>,
    /// Atlas pages (a character has one, or several in `a`/`b`/`c` subfolders): frames and pixels.
    pages: Vec<(HashMap<String, AtlasFrame>, RgbaImage)>,
    bindings: Vec<Binding>,
    /// Clip-vs-physics layout offset (px) used only by `bind_in` (the c10 daughter's frame / son's basket, whose clips do not follow the COM rule).
    /// City backdrop layers (bottom → top, with their parallax multiplier): baked buildings (`drawBuilding`).
    city: Vec<(f64, Vec<Building>)>,
    /// Extra node names hidden for the current `draw` (Santa's `ski`/`stem` children after those parts broke off).
    hidden: std::cell::RefCell<Vec<&'static str>>,
    /// `wound` overlays of the chest / pelvis are drawn only while this is set (see `Ragdoll::part_wound`).
    show_wound: std::cell::Cell<bool>,
    /// `targetMaskHolder` mask of the body being drawn: screen → mower-frame matrix and the rect's top `y` (px); pixels outside x ±100, y..y+100 are skipped.
    grind_mask: std::cell::Cell<Option<(M, f64)>>,
    /// Asset root (kept for the lazily opened NPC clips) and those clips (`NPCSprite1..12` as top-level nodes `npc1..12`).
    assets: SpriteAssets,
    npc: std::cell::OnceCell<Option<Box<Sprites>>>,
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
    /// The node's own transform with y position and scales as given (`rotation` and `skew` as pixi composes them).
    fn matrix_with(&self, x: f64, y: f64, rot: f64, sx: f64, sy: f64) -> M {
        let (a, b) = (rot + self.ky, rot - self.kx);
        [a.cos() * sx, a.sin() * sx, -b.sin() * sy, b.cos() * sy, x, y]
    }

    fn matrix(&self) -> M {
        self.matrix_with(self.x, self.y, self.r, self.sx, self.sy)
    }

    fn name_is(&self, n: &str) -> bool {
        self.n == n
    }
}

impl Sprites {
    /// Loads the clip dump and atlas of character `index`. `assets` is the original asset root.
    pub fn open(assets: &Path, index: u32) -> Option<Self> {
        Self::open_from(&SpriteAssets::Directory(assets.to_path_buf()), index)
    }

    pub fn open_from(source: &SpriteAssets, index: u32) -> Option<Self> {
        let assets = source.root();
        let mut clip: Vec<Node> = serde_json::from_str(match index {
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
        // `christmas_bell` (library clip, 0.5 scale) on the four balls of Santa's rear strap.
        if index == 8 {
            for i in 1..=4 {
                clip.push(serde_json::from_str(&format!(r#"{{"n":"bell{i}","x":0,"y":0,"r":0,"sx":1,"sy":1,"px":0,"py":0,"t":null,"vis":true,"k":[{{"n":"","x":-5,"y":-4,"r":0,"sx":0.333,"sy":0.333,"px":0,"py":0,"t":"santa-asset-21.png","vis":true,"k":[]}}]}}"#)).ok()?);
            }
        }
        let mut runtimes: HashMap<String, RuntimeClip> = serde_json::from_str(include_str!("../data/runtime-clips.json")).ok()?;
        let runtime = runtimes.remove(&index.to_string())?;
        apply_runtime(&mut clip, "", &runtime);
        let layers = runtime.order.into_iter().enumerate().map(|(layer, name)| (name, layer)).collect();
        let dir = assets.join(format!("animate/character{index}"));
        let mut folders = vec![(dir.clone(), format!("character{index}"))];
        for sub in ["a", "b", "c"] {
            folders.push((dir.join(sub), sub.to_string()));
        }
        // (the heart's art is shared by all characters)
        folders.push((assets.join("animate/character-shared"), "character-shared".to_string()));
        let mut pages = Vec::new();
        for (folder, stem) in folders {
            let (Some(json), Some(img)) = (source.string(&folder.join(format!("{stem}{ATLAS}.json"))), source.image(&folder.join(format!("{stem}{ATLAS}.png")))) else { continue };
            let Ok(atlas) = serde_json::from_str::<AtlasFile>(&json) else { continue };
            pages.push((atlas.frames, img.to_rgba8()));
        }
        if pages.is_empty() {
            return None;
        }
        // Atlas `game/l` holds the runtime special-element art (spikes, ...).
        for g in ["l", "d", "r", "o", "c", "a", "n", "p", "q"] {
            let game = assets.join(format!("animate/game/{g}"));
            if let (Some(json), Some(img)) = (source.string(&game.join(format!("{g}{ATLAS}.json"))), source.image(&game.join(format!("{g}{ATLAS}.png")))) {
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
                let Some(json) = source.string(&folder.join(format!("{stem}{ATLAS}.json"))) else { continue };
                let Ok(atlas) = serde_json::from_str::<AtlasFile>(&json) else { continue };
                if !missing.iter().any(|t| atlas.frames.contains_key(t)) {
                    continue;
                }
                let Some(img) = source.image(&folder.join(format!("{stem}{ATLAS}.png"))) else { continue };
                missing.retain(|t| !atlas.frames.contains_key(t));
                pages.push((atlas.frames, img.to_rgba8()));
            }
        }
        Some(Self { blurred: Default::default(), view: Default::default(), trace: Default::default(), layers, shown: Default::default(), queue: Default::default(), clip, pages, bindings: Vec::new(), city: Vec::new(), hidden: Default::default(), show_wound: Default::default(), grind_mask: Default::default(), assets: source.clone(), npc: Default::default() })
    }

    /// Clip set of the NPC ragdolls: `NPCSprite1..12` (dumped from the oracle) with the atlas pages that hold their textures (game atlases and the
    /// character atlases).
    fn open_npc(source: &SpriteAssets) -> Option<Self> {
        let assets = source.root();
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        let sources = [
            include_str!("../data/game-NPCSprite1.json"),
            include_str!("../data/game-NPCSprite2.json"),
            include_str!("../data/game-NPCSprite3.json"),
            include_str!("../data/game-NPCSprite4.json"),
            include_str!("../data/game-NPCSprite5.json"),
            include_str!("../data/game-NPCSprite6.json"),
            include_str!("../data/game-NPCSprite7.json"),
            include_str!("../data/game-NPCSprite8.json"),
            include_str!("../data/game-NPCSprite9.json"),
            include_str!("../data/game-NPCSprite10.json"),
            include_str!("../data/game-NPCSprite11.json"),
            include_str!("../data/game-NPCSprite12.json"),
        ];
        let mut clip = Vec::new();
        for (i, s) in sources.iter().enumerate() {
            let f: File = serde_json::from_str(s).ok()?;
            clip.push(Node { n: format!("npc{}", i + 1), x: 0.0, y: 0.0, r: 0.0, sx: 1.0, sy: 1.0, t: None, k: f.k, alt: HashMap::new(), hidden: false, kx: 0.0, ky: 0.0 });
        }
        let mut missing = Vec::new();
        Self::textures(&clip, &mut missing);
        let mut folders: Vec<(PathBuf, String)> = Vec::new();
        for g in ["d", "e", "f", "g", "h", "i", "j", "k", "l"] {
            folders.push((assets.join(format!("animate/game/{g}")), g.to_string()));
        }
        for c in 1..=11 {
            let dir = assets.join(format!("animate/character{c}"));
            folders.push((dir.clone(), format!("character{c}")));
            for sub in ["a", "b", "c"] {
                folders.push((dir.join(sub), sub.to_string()));
            }
        }
        let mut pages = Vec::new();
        for (folder, stem) in folders {
            if missing.is_empty() {
                break;
            }
            let Some(json) = source.string(&folder.join(format!("{stem}{ATLAS}.json"))) else { continue };
            let Ok(atlas) = serde_json::from_str::<AtlasFile>(&json) else { continue };
            if !missing.iter().any(|t| atlas.frames.contains_key(t)) {
                continue;
            }
            let Some(img) = source.image(&folder.join(format!("{stem}{ATLAS}.png"))) else { continue };
            missing.retain(|t| !atlas.frames.contains_key(t));
            pages.push((atlas.frames, img.to_rgba8()));
        }
        Some(Self { blurred: Default::default(), view: Default::default(), trace: Default::default(), layers: Default::default(), shown: Default::default(), queue: Default::default(), clip, pages, bindings: Vec::new(), city: Vec::new(), hidden: Default::default(), show_wound: Default::default(), grind_mask: Default::default(), assets: source.clone(), npc: Default::default() })
    }

    /// Draws the NPC of special `sp` (frame 1 of every part, `shape` guides hidden): each part clip sits at its body's centre of mass with the body's
    /// angle and scale ±0.5 (`createMovieClips`; x flipped by `reverse`). Wounds, limb frames and detached parts are not drawn.
    fn draw_npc(&self, img: &mut RgbaImage, sp: &crate::level::Special, bodies: [BodyId; 11], world: &World, cam: M) {
        const NAMES: [&str; 11] = ["chest", "head", "pelvis", "upperArm1", "upperArm2", "lowerArm1", "lowerArm2", "upperLeg1", "upperLeg2", "lowerLeg1", "lowerLeg2"];
        let idx = (sp.num("charIndex").unwrap_or(1.0).max(1.0) as usize).min(self.clip.len()) - 1;
        let flip = if sp.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
        let root = &self.clip[idx];
        // draw order of `createMovieClips`: arm2, arm1, head, chest, pelvis, leg2, leg1
        for name in ["upperArm2", "lowerArm2", "upperArm1", "lowerArm1", "head", "chest", "pelvis", "upperLeg2", "lowerLeg2", "upperLeg1", "lowerLeg1"] {
            let Some(bi) = NAMES.iter().position(|n| *n == name) else { continue };
            let body = bodies[bi];
            if world.body_center_px(body).is_none() {
                continue;
            }
            let Some(path) = Self::find_path(&root.k, name, &mut Vec::new()) else { continue };
            let mut node = &root.k[path[0]];
            for &i in &path[1..] {
                node = &node.k[i];
            }
            let m = mul(cam, mul(body_matrix(world, body), local(0.0, 0.0, 0.0, 0.5 * flip * node.sx, 0.5 * node.sy)));
            self.set_grind_mask(world, body, cam);
            for child in &node.k {
                self.draw_node(img, child, m);
            }
            self.grind_mask.set(None);
        }
    }

    /// Sets (or clears) the `targetMaskHolder` clip for drawing `body` under camera matrix `cam`.
    fn set_grind_mask(&self, world: &World, body: BodyId, cam: M) {
        let mask = match world.bike.as_ref() {
            Some(crate::vehicle::Vehicle::Mower(m)) => m.blade_mask(world, body),
            _ => None,
        };
        self.grind_mask.set(mask.and_then(|(c, a, y)| invert(mul(cam, local(c[0], c[1], a, 1.0, 1.0))).map(|inv| (inv, y))));
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

    /// Parallax backdrop of the "green hills" level backdrop (`bg='1'`; clips `green_sky_backdrop`, `green_backdrop3/2/1`, bottom to top). The layers
    /// are plain clip trees drawn with [`draw_backdrop`](Self::draw_backdrop); `assets` is the original asset root.
    pub fn open_backdrop(assets: &Path) -> Option<Self> {
        Self::open_backdrop_from(&SpriteAssets::Directory(assets.to_path_buf()))
    }

    pub fn open_backdrop_from(source: &SpriteAssets) -> Option<Self> {
        let assets = source.root();
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
            .map(|s| serde_json::from_str::<File>(s).ok().map(|f| Node { n: "layer".into(), x: 0.0, y: 0.0, r: 0.0, sx: 1.0, sy: 1.0, t: None, k: f.k, alt: HashMap::new(), hidden: false, kx: 0.0, ky: 0.0 }))
            .collect::<Option<_>>()?;
        let mut pages = Vec::new();
        for i in 1..=3 {
            let stem = format!("backgrounds_atlas_{i}");
            let folder = assets.join("animate/backgrounds").join(&stem);
            let (Some(json), Some(img)) = (source.string(&folder.join(format!("{stem}{ATLAS}.json"))), source.image(&folder.join(format!("{stem}{ATLAS}.png")))) else { continue };
            let Ok(atlas) = serde_json::from_str::<AtlasFile>(&json) else { continue };
            pages.push((atlas.frames, img.to_rgba8()));
        }
        if pages.is_empty() {
            return None;
        }
        Some(Self { blurred: Default::default(), view: Default::default(), trace: Default::default(), layers: Default::default(), shown: Default::default(), queue: Default::default(), clip, pages, bindings: Vec::new(), city: Vec::new(), hidden: Default::default(), show_wound: Default::default(), grind_mask: Default::default(), assets: source.clone(), npc: Default::default() })
    }

    /// City backdrop (`bg='2'`): `city_bd1` (fixed sky/clouds) below two layers of baked buildings (`backdrop2`: multiplier .25, blur 5;
    /// `backdrop1`: .5, blur 2), as in `createBackDrops` / `MZ` / `Mg` (deobfuscated.js 48633–48690, 49604).
    pub fn open_city_backdrop(assets: &Path) -> Option<Self> {
        Self::open_city_backdrop_from(&SpriteAssets::Directory(assets.to_path_buf()))
    }

    pub fn open_city_backdrop_from(source: &SpriteAssets) -> Option<Self> {
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        let load = |s: &str| serde_json::from_str::<File>(s).ok();
        let (l1, l2, sky) = (load(include_str!("../data/backdrop-city-backdrop1.json"))?, load(include_str!("../data/backdrop-city-backdrop2.json"))?, load(include_str!("../data/backdrop-city-city_bd1.json"))?);
        let mut s = Self::open_backdrop_from(source)?;
        s.clip = vec![Node { n: "layer".into(), x: 0.0, y: 0.0, r: 0.0, sx: 1.0, sy: 1.0, t: None, k: sky.k, alt: HashMap::new(), hidden: false, kx: 0.0, ky: 0.0 }];
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
        for (index, ((layer, m), blur)) in self.clip.iter().zip(MULTIPLIERS).zip(BLUR).enumerate() {
            let g = local((cx * m).round(), (cy * m).round(), 0.0, 1.0, 1.0);
            if blur == 0.0 {
                for child in &layer.k {
                    self.draw_node(img, child, g);
                }
                continue;
            }
            // Blurring a whole layer costs more than the rest of the frame, so each one is kept, rendered with a margin, and reused
            // until the camera has moved the layer by more than that margin.
            const MARGIN: i64 = 64;
            let (ox, oy) = ((cx * m).round() as i64, (cy * m).round() as i64);
            let size = (img.width() + 2 * MARGIN as u32, img.height() + 2 * MARGIN as u32);
            let mut cache = self.blurred.borrow_mut();
            if cache.len() <= index {
                cache.resize_with(index + 1, || None);
            }
            let fits = |c: &Blurred| c.image.dimensions() == size && (c.at.0 - ox).abs() <= MARGIN && (c.at.1 - oy).abs() <= MARGIN;
            if !cache[index].as_ref().is_some_and(fits) {
                let mut tmp = RgbaImage::new(size.0, size.1);
                let shifted = local((ox + MARGIN) as f64, (oy + MARGIN) as f64, 0.0, 1.0, 1.0);
                for child in &layer.k {
                    self.draw_node(&mut tmp, child, shifted);
                }
                // premultiplied, as Flash blurs: otherwise transparent black bleeds into the edges as a dark halo
                for p in tmp.pixels_mut() {
                    for i in 0..3 {
                        p[i] = (u16::from(p[i]) * u16::from(p[3]) / 255) as u8;
                    }
                }
                let sigma = (3.0 * (blur * blur - 1.0) / 12.0).sqrt();
                cache[index] = Some(Blurred { at: (ox, oy), image: imageproc::filter::gaussian_blur_f32(&tmp, sigma) });
            }
            let Some(layer) = cache[index].as_ref() else { continue };
            let (dx, dy) = (MARGIN - (ox - layer.at.0), MARGIN - (oy - layer.at.1));
            for (x, y, d) in img.enumerate_pixels_mut() {
                let p = layer.image.get_pixel((i64::from(x) + dx) as u32, (i64::from(y) + dy) as u32);
                if p[3] == 0 {
                    continue;
                }
                let a = f32::from(p[3]) / 255.0;
                for i in 0..3 {
                    d[i] = (f32::from(p[i]) + f32::from(d[i]) * (1.0 - a)).min(255.0) as u8;
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
        self.bindings.push(Binding { pose: None, only: None, without: None, path: vec![part], body, rest: local(0.0, 0.0, 0.0, 0.5 * c.sx, 0.5 * c.sy) });
    }

    /// [`bind_origin`](Self::bind_origin) for a part nested one level down inside clip `parent` (`kid`, `girl`).
    pub fn bind_origin_in(&mut self, parent: &str, name: &str, body: BodyId) {
        let Some(pi) = self.clip.iter().position(|c| c.n == parent) else { return };
        let Some(ci) = self.clip[pi].k.iter().position(|c| c.n == name) else { return };
        let c = &self.clip[pi].k[ci];
        self.bindings.push(Binding { pose: None, only: None, without: None, path: vec![pi, ci], body, rest: local(0.0, 0.0, 0.0, 0.5 * c.sx, 0.5 * c.sy) });
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
        self.bindings.push(Binding { pose: None, only: None, without: None, path, body, rest });
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
                    self.bind_origin_in("kid", "seat", k.seat);
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
            // `addChildAt` at one index: the last bell ends up lowest
            for (i, &b) in s.strap_balls.iter().skip(12).take(4).enumerate().rev() {
                self.bind_origin(&format!("bell{}", i + 1), b);
            }
        }
        if let Some(Vehicle::Family(f)) = world.bike.as_ref() {
            let (d, s) = (&f.daughter, &f.son);
            self.bind_ragdoll(&d.doll, Some("daughter"));
            self.bind_ragdoll(&s.doll, Some("son"));
            let (frame, wheel, gear, basket) = (d.frame, d.wheel, d.gear, s.basket);
            self.bind_origin_in("daughter", "wheel", wheel);
            self.bind_origin_in("daughter", "gear", gear);
            self.bind_origin_in("daughter", "frame", frame);
            self.bind_origin_in("son", "basket", basket);
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
                self.bind_origin("bladecover", body);
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
                // `createMovieClips` lifts `main.inner.cart` into its own layer above the near lower arm.
                if let Some(mut front) = self.bindings.last().cloned() {
                    if let Some(main) = self.bindings.last_mut() {
                        main.without = Some("cart");
                    }
                    front.only = Some("cart");
                    self.bindings.push(front);
                }
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
            self.bind_origin("wheelCover", s.stand);
            self.bind_origin("wheel", wheel);
        }
        if let Some(Vehicle::Chair(c)) = world.bike.as_ref() {
            let (chair, big, small) = (c.chair, c.big_wheel, c.small_wheel);
            // `handle`/`fueltank`/`jet`/`metalShards` top-level clips are the chairSmash pieces.
            self.bind(world, "chair", chair, start);
            self.bind_origin("bigWheel", big);
            self.bind_origin("smallWheel", small);
            // `paint` keeps the intact jet at the big wheel's centre with the chair's angle (+ `jetAngle`).
            self.bind_origin("jet", big);
        }
    }

    /// True if `body` is drawn as a sprite (the polygon fallback skips it).
    pub fn covers(&self, body: BodyId) -> bool {
        self.bindings.iter().any(|b| b.body == body)
    }

    /// Draws every bound part plus the debris a smash leaves (chunks, brain, heart, helmet); `camera_offset` translates world px to image px.
    /// True when special element `kind` has sprite art here (and so replaces the grey fallback).
    pub fn covers_special(&self, kind: usize) -> bool {
        if kind == 17 {
            return self.npc.get_or_init(|| Self::open_npc(&self.assets).map(Box::new)).is_some();
        }
        if kind == 18 {
            return true;
        }
        let tex = match kind {
            6 => "spike-clean.png",
            3 => "i-beam-body.png",
            4 => "log-full.png",
            1 => "table-mc-top.png",
            2 => "mine-base-1.png",
            5 => "springbox-mc-pad.png",
            10 => "soccer-ball.png",
            7 => "wrecking-ball-ball.png",
            0 => "van-body.png",
            27 => "rail.png",
            11 => "meteor.png",
            31 => "token-outer-ring.png",
            23 => "post.png",
            8 => "fan-1522.png",
            9 => "finish-line-asset-1480.png",
            34 => "blade-weapon-1654.png",
            15 => "harpoon-gun-base-2.png",
            28 => "jet-ref.png",
            35 => "paddle-paddle.png",
            12 => "boost-1718.png",
            24 => "toilet-ref-base.png",
            19 => "chair-top-asset.png",
            33 => "cannon-asset-1541.png",
            26 => "trash-can-body.png",
            20 => "bottle-1724.png",
            21 => "tv-damage-0.png",
            22 => "boombox-1683.png",
            30 => "link-5.png",
            32 => "food-item-asset-1411.png",
            25 => "homing-mine-jet.png",
            29 => "arrow-asset-1650.png",
            _ => return false,
        };
        self.pages.iter().any(|(f, _)| f.contains_key(tex))
    }

    /// Spikes (`Spikes.setupMC`, `paint`): clip at the body's centre of mass; `spikes` row at (−halfWidth, −25) and `base` (`spike-platform`, stretched
    /// to 15·n−2) at (0, 35), both lowered by the body's local-centre y; one `spike-clean` per 15 px. NOT verified: clips with fewer than 20 spikes
    /// (the library clip has 20 children; the original's `setupMC` adds more only above 20).
    /// Chunk bodies of smashed food items: the `m<n>` child of the fruit clip's `chunks` at the body's pose (`createBody` of the chunk class).
    fn draw_food_chunks(&self, img: &mut RgbaImage, world: &World, cam: M) {
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        static CLIPS: std::sync::OnceLock<[Vec<Node>; 3]> = std::sync::OnceLock::new();
        let clips = CLIPS.get_or_init(|| {
            let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
            [f(include_str!("../data/game-WatermelonMC.json")), f(include_str!("../data/game-PumpkinMC.json")), f(include_str!("../data/game-PineappleMC.json"))]
        });
        for &(b, ty, n) in world.food_chunks() {
            if world.body_center_px(b).is_none() {
                continue;
            }
            let name = format!("m{n}");
            let Some(m) = clips[usize::from(ty) % 3].iter().find(|c| c.n == "chunks").and_then(|c| c.k.iter().find(|k| k.n == name)) else { continue };
            let pose = mul(cam, mul(body_matrix(world, b), local(0.0, 0.0, m.r, m.sx, m.sy)));
            for k in &m.k {
                self.draw_node(img, k, pose);
            }
        }
    }

    /// Pieces of shattered boomboxes and TVs (`BoomboxPiece<n>MC` / `TVPiece<n>MC` at the piece body's pose).
    fn draw_boombox_pieces(&self, img: &mut RgbaImage, world: &World, cam: M) {
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        static PIECES: std::sync::OnceLock<[Vec<Node>; 5]> = std::sync::OnceLock::new();
        let pieces = PIECES.get_or_init(|| {
            let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
            [
                f(include_str!("../data/game-BoomboxPiece1MC.json")),
                f(include_str!("../data/game-BoomboxPiece2MC.json")),
                f(include_str!("../data/game-TVPiece1MC.json")),
                f(include_str!("../data/game-TVPiece2MC.json")),
                f(include_str!("../data/game-TVPiece3MC.json")),
            ]
        });
        for (b, n) in world.boombox_pieces() {
            if world.body_center_px(b).is_none() {
                continue;
            }
            for k in &pieces[usize::from(n - 1)] {
                self.draw_node(img, k, mul(cam, body_matrix(world, b)));
            }
        }
    }

    /// Pieces of broken chairs (`chair_leg`, `chair_back_broken`, `chair_top` at the piece body's pose; the seat mirrors with `reverse`).
    fn draw_chair_pieces(&self, img: &mut RgbaImage, world: &World, cam: M) {
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        static PARTS: std::sync::OnceLock<[Vec<Node>; 3]> = std::sync::OnceLock::new();
        let parts = PARTS.get_or_init(|| {
            let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
            [f(include_str!("../data/game-chair_leg.json")), f(include_str!("../data/game-chair_back_broken.json")), f(include_str!("../data/game-chair_top.json"))]
        });
        for (b, kind, mirror) in world.chair_pieces() {
            if world.body_center_px(b).is_none() {
                continue;
            }
            for k in &parts[usize::from(kind)] {
                self.draw_node(img, k, mul(cam, mul(body_matrix(world, b), local(0.0, 0.0, 0.0, mirror, 1.0))));
            }
        }
    }

    /// Triangular pieces of shattered glass panes: a 0x66CCFF fill at alpha .35 (vertices in the body's frame, px).
    fn draw_glass_pieces(&self, img: &mut RgbaImage, world: &World, cam: M) {
        for (b, tri) in world.glass_pieces() {
            if world.body_center_px(b).is_none() {
                continue;
            }
            let bd = world.b2().body(b);
            let m = mul(cam, local(bd.xf.position.x * 62.5, bd.xf.position.y * 62.5, bd.sweep.a, 1.0, 1.0));
            // blood left by impaled flesh: `GlassBloodMC.inner` (`glass-blood.png`, 0.25, offset (-61,-60)) at the impact point (not clipped to the triangle yet)
            for &(x, y, rot, mirror) in world.glass_blood(b) {
                self.blit(img, "glass-blood.png", mul(m, mul(local(x, y, rot, mirror, 1.0), local(-61.0, -60.0, 0.0, 0.25, 0.25))));
            }
            let pts = tri.map(|(x, y)| (m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5]));
            let (x0, x1) = (pts.iter().map(|p| p.0).fold(f64::MAX, f64::min).floor().max(0.0) as i64, pts.iter().map(|p| p.0).fold(f64::MIN, f64::max).ceil().min(f64::from(img.width())) as i64);
            let (y0, y1) = (pts.iter().map(|p| p.1).fold(f64::MAX, f64::min).floor().max(0.0) as i64, pts.iter().map(|p| p.1).fold(f64::MIN, f64::max).ceil().min(f64::from(img.height())) as i64);
            let edge = |a: (f64, f64), b: (f64, f64), p: (f64, f64)| (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0);
            for py in y0..y1 {
                for px in x0..x1 {
                    let p = (px as f64 + 0.5, py as f64 + 0.5);
                    let (e0, e1, e2) = (edge(pts[0], pts[1], p), edge(pts[1], pts[2], p), edge(pts[2], pts[0], p));
                    if (e0 >= 0.0 && e1 >= 0.0 && e2 >= 0.0) || (e0 <= 0.0 && e1 <= 0.0 && e2 <= 0.0) {
                        let dst = img.get_pixel_mut(px as u32, py as u32);
                        for (c, s) in dst.0.iter_mut().zip([0x66_u8, 0xCC, 0xFF]) {
                            *c = (f64::from(*c) * 0.65 + f64::from(s) * 0.35).round() as u8;
                        }
                    }
                }
            }
        }
    }

    /// Pieces of broken toilets (`ToiletBasePiece1/2MC`, `ToiletBackPiece1/2MC`) at the piece body's origin, mirrored with `reverse`.
    fn draw_toilet_pieces(&self, img: &mut RgbaImage, world: &World, cam: M) {
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        static PARTS: std::sync::OnceLock<[Vec<Node>; 4]> = std::sync::OnceLock::new();
        let parts = PARTS.get_or_init(|| {
            let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
            [
                f(include_str!("../data/game-ToiletBasePiece1MC.json")),
                f(include_str!("../data/game-ToiletBasePiece2MC.json")),
                f(include_str!("../data/game-ToiletBackPiece1MC.json")),
                f(include_str!("../data/game-ToiletBackPiece2MC.json")),
            ]
        });
        for (b, kind, mirror) in world.toilet_pieces() {
            let Some([x, y]) = world.body_position_px(b) else { continue };
            let m = mul(cam, local(f64::from(x), f64::from(y), f64::from(world.body_angle(b)), mirror, 1.0));
            for k in &parts[usize::from(kind)] {
                self.draw_node(img, k, m);
            }
        }
    }

    /// The two halves of a broken log (`LogHalf1` upper, `LogHalf2` lower), stretched like the log, at the half body's origin.
    fn draw_log_halves(&self, img: &mut RgbaImage, world: &World, cam: M) {
        for (b, which, sx, sy) in world.log_halves() {
            let Some([x, y]) = world.body_position_px(b) else { continue };
            let m = mul(cam, mul(local(f64::from(x), f64::from(y), f64::from(world.body_angle(b)), 1.0, 1.0), local(0.0, 0.0, 0.0, sx, sy)));
            let (tex, ox, oy) = if which == 0 { ("log-half-1.png", -18.0, -100.0) } else { ("log-half-2.png", -18.0, -112.0) };
            self.blit(img, tex, mul(m, local(ox, oy, 0.0, 0.5, 0.5)));
        }
    }

    /// Arrows (`ArrowMC`) and harpoons (`HarpoonMC`) of the guns at their body's centre of mass and angle.
    fn draw_arrows(&self, img: &mut RgbaImage, world: &World, cam: M) {
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        static CLIPS: std::sync::OnceLock<[Vec<Node>; 2]> = std::sync::OnceLock::new();
        let clips = CLIPS.get_or_init(|| {
            let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
            [f(include_str!("../data/game-ArrowMC.json")), f(include_str!("../data/game-HarpoonMC.json"))]
        });
        for (b, harpoon) in world.arrow_bodies() {
            if world.body_center_px(b).is_none() {
                continue;
            }
            for k in &clips[usize::from(harpoon)] {
                self.draw_node(img, k, mul(cam, body_matrix(world, b)));
            }
            if harpoon {
                // blood left on the shaft by impaled flesh (`HarpoonBloodMC` frames, drawn in the harpoon's local px)
                static BLOOD: std::sync::OnceLock<Vec<Vec<Node>>> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                let frames = BLOOD.get_or_init(|| serde_json::from_str::<Frames>(include_str!("../data/game-HarpoonBloodMC.json")).map(|f| f.frames).unwrap_or_default());
                for &(frame, ..) in world.harpoon_blood(b) {
                    for k in frames.get(frame as usize).into_iter().flatten() {
                        self.draw_node(img, k, mul(cam, body_matrix(world, b)));
                    }
                }
            }
        }
    }

    pub fn draw_specials(&self, img: &mut RgbaImage, level: &crate::level::Level, world: &World, camera_offset: (f32, f32)) {
        let cam = self.camera(camera_offset);
        self.draw_food_chunks(img, world, cam);
        self.draw_boombox_pieces(img, world, cam);
        self.draw_chair_pieces(img, world, cam);
        self.draw_glass_pieces(img, world, cam);
        self.draw_toilet_pieces(img, world, cam);
        self.draw_log_halves(img, world, cam);
        self.draw_arrows(img, world, cam);
        for (i, sp) in level.specials.iter().enumerate() {
            if sp.kind == 0 {
                self.draw_van(img, sp, world.special_parts.get(i).map(Vec::as_slice).unwrap_or(&[]), world, cam);
                continue;
            }
            if matches!(sp.kind, 1 | 2 | 5 | 7 | 10) {
                self.draw_clip_special(img, sp, world.special_bodies.get(i).copied().flatten(), world, cam);
                continue;
            }
            // Fan (`blades.play()`: one of 6 frames per step) and finish line: static clips at the level position/angle.
            if matches!(sp.kind, 8 | 9) {
                static CLIPS: std::sync::OnceLock<[Vec<Node>; 2]> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                let clips = CLIPS.get_or_init(|| {
                    let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
                    [f(include_str!("../data/game-FanMC.json")), f(include_str!("../data/game-FinishLineMC.json"))]
                });
                let m = mul(cam, local(f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians(), 1.0, 1.0));
                for n in &clips[usize::from(sp.kind == 9)] {
                    if n.n == "blades" && sp.kind == 8 {
                        let f = (world.frame() % 6) as usize + 1;
                        let kids = if f > 1 { n.alt.get(&f.to_string()).unwrap_or(&n.k) } else { &n.k };
                        let bm = mul(m, n.matrix());
                        for k in kids {
                            self.draw_node(img, k, bm);
                        }
                    } else {
                        self.draw_node(img, n, m);
                    }
                }
                continue;
            }
            // BladeWeapon: `BladeWeaponMC.gotoAndStop(bladeWeaponType)` (one texture per frame) at the weapon body's origin with its angle (`paint`
            // with zero offsets), mirrored when `reverse`; a non-interactive blade stays at its level position. The blood stamps (`paintBlood`: `BladeBloodMC`
            // clipped by `BladeWeaponMaskMC`, whose mask is vector art that the clip dump cannot read) are not drawn.
            if sp.kind == 34 {
                static FRAMES: std::sync::OnceLock<Vec<Vec<Node>>> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                let frames = FRAMES.get_or_init(|| serde_json::from_str::<Frames>(include_str!("../data/game-BladeWeaponMC.json")).map(|f| f.frames).unwrap_or_default());
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let pose = match world.special_bodies.get(i).copied().flatten() {
                    Some(b) if world.b2().body(b).alive => {
                        let bd = world.b2().body(b);
                        local(bd.xf.position.x * 62.5, bd.xf.position.y * 62.5, bd.sweep.a, 1.0, 1.0)
                    }
                    Some(_) => continue,
                    None => local(x, y, a, 1.0, 1.0),
                };
                let flip = if sp.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
                let m = mul(cam, mul(pose, local(0.0, 0.0, 0.0, flip, 1.0)));
                let frame = (sp.num("bladeWeaponType").unwrap_or(1.0).max(1.0) as usize).min(frames.len().max(1)) - 1;
                let nodes = frames.get(frame).map(Vec::as_slice).unwrap_or(&[]);
                for n in nodes {
                    self.draw_node(img, n, m);
                }
                // paintBlood: `BladeBloodMC` stamps clipped by `BladeWeaponMaskMC`. The mask is vector art the clip dump cannot read, so the blade's
                // own texture silhouette stands in for it (blood is multiplied by the blade sprite's alpha, which also covers the handle).
                let stamps = world.special_bodies.get(i).copied().flatten().map_or(&[][..], |b| world.blade_blood(b));
                if !stamps.is_empty() {
                    static BLOOD: std::sync::OnceLock<Vec<Node>> = std::sync::OnceLock::new();
                    #[derive(Deserialize)]
                    struct Clip {
                        k: Vec<Node>,
                    }
                    let blood = BLOOD.get_or_init(|| serde_json::from_str::<Clip>(include_str!("../data/game-BladeBloodMC.json")).map(|c| c.k).unwrap_or_default());
                    let mut mask = RgbaImage::new(img.width(), img.height());
                    let mut paint = RgbaImage::new(img.width(), img.height());
                    for n in nodes {
                        self.draw_node(&mut mask, n, m);
                    }
                    for &(sx, sy, rot, mirror) in stamps {
                        let sm = mul(m, local(sx, sy, rot, mirror, 1.0));
                        for n in blood {
                            self.draw_node(&mut paint, n, sm);
                        }
                    }
                    for (x, y, p) in paint.enumerate_pixels() {
                        let a = u32::from(p[3]) * u32::from(mask.get_pixel(x, y)[3]) / 255;
                        if a == 0 {
                            continue;
                        }
                        let d = img.get_pixel_mut(x, y);
                        for c in 0..3 {
                            d[c] = ((u32::from(p[c]) * a + u32::from(d[c]) * (255 - a)) / 255) as u8;
                        }
                    }
                }
                continue;
            }
            // HarpoonGun / ArrowGun: static base clip at the level position/angle; `turret` is rotated by (aim angle − base angle) (`paint`). Firing frames,
            // the harpoon gun's light and the loaded harpoon's disappearance are not drawn.
            if matches!(sp.kind, 15 | 29) {
                static GUNS: std::sync::OnceLock<[Vec<Node>; 2]> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                let guns = GUNS.get_or_init(|| {
                    let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
                    [f(include_str!("../data/game-HarpoonGunMC.json")), f(include_str!("../data/game-ArrowGunMC.json"))]
                });
                let harpoon = sp.kind == 15;
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let turret = world.gun_turret_angle(x, y, a, harpoon).map_or(0.0, |t| t - a);
                let m = mul(cam, local(x, y, a, 1.0, 1.0));
                for n in &guns[usize::from(!harpoon)] {
                    if n.n == "turret" {
                        let tm = mul(m, n.matrix_with(n.x, n.y, turret, n.sx, n.sy));
                        for k in &n.k {
                            if k.n != "light" {
                                self.draw_node(img, k, tm);
                            }
                        }
                    } else {
                        self.draw_node(img, n, m);
                    }
                }
                continue;
            }
            // Jet (`jet_mc` scaled 1 + (power−1)/9 at the body's COM; flames only while burning, scaled .95·accel + .05) and HomingMine (`HomingMineMC` at the
            // mine body's COM; its four thruster flames scale with the thrust impulse, which is not tracked, so they stay hidden).
            if matches!(sp.kind, 25 | 28) {
                static CLIPS: std::sync::OnceLock<[Vec<Node>; 2]> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                let clips = CLIPS.get_or_init(|| {
                    let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
                    [f(include_str!("../data/game-jet_mc.json")), f(include_str!("../data/game-HomingMineMC.json"))]
                });
                let Some(body) = world.special_bodies.get(i).copied().flatten().filter(|&b| world.body_center_px(b).is_some()) else { continue };
                let pose = mul(cam, body_matrix(world, body));
                if sp.kind == 28 {
                    let s = 1.0 + (f64::from(sp.num("power").unwrap_or(1.0)) - 1.0) / 9.0;
                    let m = mul(pose, local(0.0, 0.0, 0.0, s, s));
                    let (accel, burning) = world.jet_state(body).unwrap_or((1.0, false));
                    for n in &clips[0] {
                        if n.n == "flames" {
                            if burning {
                                let fs = 0.95 * accel + 0.05;
                                for k in &n.k {
                                    self.draw_node(img, k, mul(m, n.matrix_with(n.x, n.y, n.r, fs, fs)));
                                }
                            }
                        } else {
                            self.draw_node(img, n, m);
                        }
                    }
                } else {
                    for n in clips[1].iter().filter(|n| !n.n.starts_with("jet")) {
                        self.draw_node(img, n, pose);
                    }
                }
                continue;
            }
            // Paddle: `PaddleBaseMC` static at the level position/angle, `PaddMC` at the pad body's centre of mass; `reverse` mirrors the nubs, arrow and timer x.
            // The timer text, the `glow` and the moving-arrow state are not drawn.
            if sp.kind == 35 {
                static CLIPS: std::sync::OnceLock<[Vec<Node>; 2]> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                let clips = CLIPS.get_or_init(|| {
                    let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
                    [f(include_str!("../data/game-PaddleBaseMC.json")), f(include_str!("../data/game-PaddMC.json"))]
                });
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let rev = sp.flag("reverse").unwrap_or(false);
                let place = |s: &Self, img: &mut RgbaImage, clip: &[Node], m: M| {
                    for n in clip {
                        let mx = if rev && matches!(n.n.as_str(), "nub" | "timer" | "arrow") { -1.0 } else { 1.0 };
                        let nm = mul(m, n.matrix_with(n.x * mx, n.y, n.r, n.sx, n.sy));
                        if let Some(tex) = &n.t {
                            s.blit(img, tex, nm);
                        }
                        for k in n.k.iter().filter(|k| k.n != "glow") {
                            s.draw_node(img, k, nm);
                        }
                    }
                };
                place(self, img, &clips[0], mul(cam, local(x, y, a, 1.0, 1.0)));
                let pad = world.special_bodies.get(i).copied().flatten().filter(|&b| world.body_center_px(b).is_some());
                place(self, img, &clips[1], mul(cam, pad.map_or_else(|| local(x, y, a, 1.0, 1.0), |b| body_matrix(world, b))));
                continue;
            }
            // Boost pad: `numPanels` `BoostPanelMC` (18 frames) 180 px apart centred on (x, y), rotated by the level angle; panel k starts at frame
            // `1 − 4k (mod 18)` and plays one frame per step (`gotoAndPlay`). Frame phase vs. the original's timeline start is not verified.
            if sp.kind == 12 {
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                static PANEL: std::sync::OnceLock<Vec<Vec<Node>>> = std::sync::OnceLock::new();
                let panel = PANEL.get_or_init(|| serde_json::from_str::<Frames>(include_str!("../data/game-BoostPanelMC.json")).map(|f| f.frames).unwrap_or_default());
                if panel.is_empty() {
                    continue;
                }
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let num = sp.num("numPanels").unwrap_or(2.0).max(1.0) as i64;
                let (mut e, mut start) = (-90.0 * num as f64, 1_i64);
                for _ in 0..num {
                    let f = ((start - 1 + i64::from(world.frame())).rem_euclid(18)) as usize;
                    let m = mul(cam, mul(local(x, y, a, 1.0, 1.0), local(e, 0.0, 0.0, 1.0, 1.0)));
                    for n in &panel[f.min(panel.len() - 1)] {
                        self.draw_node(img, n, m);
                    }
                    e += 180.0;
                    start -= 4;
                    if start <= 0 {
                        start += 18;
                    }
                }
                continue;
            }
            // Chain: link k (`link0MC` for even k, `link1MC` for odd) scaled 1 + (linkScale − 1)/9·2 at each link body's COM (non-interactive chains have
            // no bodies and are not drawn here). FoodItem: the whole-fruit child of `WatermelonMC`/`PumpkinMC`/`PineappleMC` at the body's COM (the
            // `chunks` stay hidden until a smash, which is not drawn: the body is simply gone).
            if matches!(sp.kind, 30 | 32) {
                static CLIPS: std::sync::OnceLock<[Vec<Node>; 5]> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                let clips = CLIPS.get_or_init(|| {
                    let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
                    [
                        f(include_str!("../data/game-link0MC.json")),
                        f(include_str!("../data/game-link1MC.json")),
                        f(include_str!("../data/game-WatermelonMC.json")),
                        f(include_str!("../data/game-PumpkinMC.json")),
                        f(include_str!("../data/game-PineappleMC.json")),
                    ]
                });
                if sp.kind == 30 {
                    let s = 1.0 + (f64::from(sp.num("linkScale").unwrap_or(1.0)) - 1.0) / 9.0 * 2.0;
                    for (k, &b) in world.special_parts.get(i).into_iter().flatten().enumerate() {
                        if world.body_center_px(b).is_none() {
                            continue;
                        }
                        let m = mul(cam, mul(body_matrix(world, b), local(0.0, 0.0, 0.0, s, s)));
                        for n in &clips[k % 2] {
                            self.draw_node(img, n, m);
                        }
                    }
                } else if let Some(b) = world.special_bodies.get(i).copied().flatten().filter(|&b| world.body_center_px(b).is_some()) {
                    let ty = (sp.num("foodItemType").unwrap_or(1.0).max(1.0) as usize).min(3) - 1;
                    for n in clips[2 + ty].iter().filter(|n| n.n != "chunks" && n.n != "shapes") {
                        self.draw_node(img, n, mul(cam, body_matrix(world, b)));
                    }
                }
                continue;
            }
            // Bottle (frame `bottleType`), TV and boombox (frame 1; the other frames are damage states): one texture clip at the body's COM, or at the
            // level position when not interactive. Broken pieces/shards and the later damage frames are not drawn.
            if matches!(sp.kind, 20..=22) {
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                static CLIPS: std::sync::OnceLock<[Vec<Vec<Node>>; 3]> = std::sync::OnceLock::new();
                let clips = CLIPS.get_or_init(|| {
                    let f = |s: &str| serde_json::from_str::<Frames>(s).map(|f| f.frames).unwrap_or_default();
                    [f(include_str!("../data/game-bottle.json")), f(include_str!("../data/game-tv.json")), f(include_str!("../data/game-BoomboxMC.json"))]
                });
                let frames = &clips[sp.kind - 20];
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let pose = match world.special_bodies.get(i).copied().flatten() {
                    Some(b) if world.body_center_px(b).is_some() => body_matrix(world, b),
                    Some(_) => continue,
                    None => local(x, y, a, 1.0, 1.0),
                };
                let f = match sp.kind {
                    20 => (sp.num("bottleType").unwrap_or(1.0).max(1.0) as usize).min(frames.len().max(1)) - 1,
                    21 | 22 => world.special_bodies.get(i).copied().flatten().map_or(1, |b| world.boombox_frame(b)).clamp(1, frames.len().max(1)) - 1,
                    _ => 0,
                };
                for n in frames.get(f).map(Vec::as_slice).unwrap_or(&[]) {
                    self.draw_node(img, n, mul(cam, pose));
                }
                continue;
            }
            // Toilet (`ToiletMC.container` frame 1, x flipped by `reverse`) and trash can (`can` + `lid` when it contains trash; `shapes` — the spilled trash —
            // stays hidden) at the body's COM, or the level position when not interactive. Break pieces / spill / squash frames are not drawn.
            if matches!(sp.kind, 24 | 26) {
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                static TOILET: std::sync::OnceLock<Vec<Node>> = std::sync::OnceLock::new();
                static CAN: std::sync::OnceLock<Vec<Vec<Node>>> = std::sync::OnceLock::new();
                let toilet = TOILET.get_or_init(|| serde_json::from_str::<File>(include_str!("../data/game-ToiletMC.json")).map(|f| f.k).unwrap_or_default());
                let cans = CAN.get_or_init(|| serde_json::from_str::<Frames>(include_str!("../data/game-TrashCanMC.json")).map(|f| f.frames).unwrap_or_default());
                let can = cans.first().map_or(&[][..], Vec::as_slice);
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let pose = match world.special_bodies.get(i).copied().flatten() {
                    Some(b) if world.body_center_px(b).is_some() => body_matrix(world, b),
                    Some(_) => continue,
                    None => local(x, y, a, 1.0, 1.0),
                };
                if sp.kind == 24 {
                    let flip = if sp.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
                    let m = mul(cam, mul(pose, local(0.0, 0.0, 0.0, flip, 1.0)));
                    let (back, base) = world.toilet_parts(world.special_bodies.get(i).copied().flatten());
                    for c in toilet {
                        for n in c.k.iter().filter(|n| (n.n != "back" || back) && (n.n != "base" || base)) {
                            self.draw_node(img, n, m);
                        }
                    }
                } else {
                    let trash = sp.flag("containsTrash").unwrap_or(true);
                    let (frame, lid, hollow) = world.special_bodies.get(i).copied().flatten().and_then(|b| world.can_look(b)).unwrap_or((1, trash, !trash));
                    if frame > 1 {
                        // crushed: the squashed texture of that frame (no lid, no can)
                        for n in cans.get(frame - 1).map(Vec::as_slice).unwrap_or(&[]).iter().filter(|n| n.t.is_some()) {
                            self.draw_node(img, n, mul(cam, pose));
                        }
                    } else {
                        for n in can.iter().filter(|n| n.n == "can" || (n.n == "lid" && lid)) {
                            // `createHollowTrashCan`: `mc.can.y -= 13`
                            let up = if hollow && n.n == "can" { -13.0 } else { 0.0 };
                            self.draw_node(img, n, mul(mul(cam, pose), local(0.0, up, 0.0, 1.0, 1.0)));
                        }
                    }
                }
                continue;
            }
            // Chair: `ChairMC.container` frame 1 (top, two legs, back) x-flipped by `reverse`, at the body's COM or the level position when not interactive.
            // The broken-back frame and the breaking pieces are not drawn.
            if sp.kind == 19 {
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                static CHAIR: std::sync::OnceLock<Vec<Node>> = std::sync::OnceLock::new();
                let chair = CHAIR.get_or_init(|| serde_json::from_str::<File>(include_str!("../data/game-ChairMC.json")).map(|f| f.k).unwrap_or_default());
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let pose = match world.special_bodies.get(i).copied().flatten() {
                    Some(b) if world.body_center_px(b).is_some() => body_matrix(world, b),
                    Some(_) => continue,
                    None => local(x, y, a, 1.0, 1.0),
                };
                let flip = if sp.flag("reverse").unwrap_or(false) { -1.0 } else { 1.0 };
                let m = mul(cam, mul(pose, local(0.0, 0.0, 0.0, flip, 1.0)));
                for n in chair {
                    self.draw_node(img, n, m);
                }
                continue;
            }
            // Cannon: `CannonBaseMC` (frame `cannonType`) static at the level position/angle; `CannonMuzzleMC` (same frame) scaled 1 + muzzleScale/20 at the muzzle
            // body's local point (0, 2 px) with its angle. The star/loading meter animation is not drawn (`meter.bar` hidden, star frame 1).
            if sp.kind == 33 {
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                static CLIPS: std::sync::OnceLock<[Vec<Vec<Node>>; 2]> = std::sync::OnceLock::new();
                let clips = CLIPS.get_or_init(|| {
                    let f = |s: &str| serde_json::from_str::<Frames>(s).map(|f| f.frames).unwrap_or_default();
                    [f(include_str!("../data/game-CannonBaseMC.json")), f(include_str!("../data/game-CannonMuzzleMC.json"))]
                });
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let ty = (sp.num("cannonType").unwrap_or(1.0).max(1.0) as usize).saturating_sub(1);
                let Some(muzzle) = world.special_bodies.get(i).copied().flatten().filter(|&b| world.body_center_px(b).is_some()) else { continue };
                let d = 1.0 + f64::from(sp.num("muzzleScale").unwrap_or(1.0)) / 20.0;
                let bd = world.b2().body(muzzle);
                let o = bd.world_point(box2d::math::Vec2::new(0.0, 2.0 / 62.5));
                let mm = mul(cam, local(o.x * 62.5, o.y * 62.5, bd.sweep.a, d, d));
                let bm = mul(cam, local(x, y, a, 1.0, 1.0));
                for (clip, m) in [(&clips[1], mm), (&clips[0], bm)] {
                    let f = ty.min(clip.len().saturating_sub(1));
                    for n in clip.get(f).map(Vec::as_slice).unwrap_or(&[]) {
                        if n.n == "meter" {
                            continue;
                        }
                        self.draw_node(img, n, m);
                    }
                }
                continue;
            }
            // NPCharacter: see `draw_npc` (clips and atlases are opened on first use).
            if sp.kind == 17 {
                let Some(chest) = world.special_bodies.get(i).copied().flatten() else { continue };
                let Some(bodies) = world.npc_bodies(chest) else { continue };
                if let Some(npc) = self.npc.get_or_init(|| Self::open_npc(&self.assets).map(Box::new)) {
                    npc.draw_npc(img, sp, bodies, world, cam);
                }
                continue;
            }
            // Glass: a vector rectangle (fill 0x66CCFF, alpha .35) of shapeWidth × shapeHeight at the pane body's centre of mass; a shattered pane's body is
            // gone, so nothing is drawn then. Shatter particles (`GlassShardsMC`) are not drawn.
            if sp.kind == 18 {
                let Some(body) = world.special_bodies.get(i).copied().flatten().filter(|&b| world.body_center_px(b).is_some()) else { continue };
                let (w, h) = (f64::from(sp.num("shapeWidth").unwrap_or(10.0)), f64::from(sp.num("shapeHeight").unwrap_or(100.0)));
                let m = mul(cam, body_matrix(world, body));
                let Some(inv) = invert(m) else { continue };
                let corners = [(-w / 2.0, -h / 2.0), (w / 2.0, -h / 2.0), (-w / 2.0, h / 2.0), (w / 2.0, h / 2.0)].map(|(x, y)| (m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5]));
                let (x0, x1) = (corners.iter().map(|c| c.0).fold(f64::MAX, f64::min).floor().max(0.0) as i64, corners.iter().map(|c| c.0).fold(f64::MIN, f64::max).ceil().min(f64::from(img.width())) as i64);
                let (y0, y1) = (corners.iter().map(|c| c.1).fold(f64::MAX, f64::min).floor().max(0.0) as i64, corners.iter().map(|c| c.1).fold(f64::MIN, f64::max).ceil().min(f64::from(img.height())) as i64);
                for py in y0..y1 {
                    for px in x0..x1 {
                        let (sx, sy) = (px as f64 + 0.5, py as f64 + 0.5);
                        let (u, v) = (inv[0] * sx + inv[2] * sy + inv[4], inv[1] * sx + inv[3] * sy + inv[5]);
                        if u.abs() <= w / 2.0 && v.abs() <= h / 2.0 {
                            let dst = img.get_pixel_mut(px as u32, py as u32);
                            for (c, s) in dst.0.iter_mut().zip([0x66_u8, 0xCC, 0xFF]) {
                                *c = (f64::from(*c) * 0.65 + f64::from(s) * 0.35).round() as u8;
                            }
                        }
                    }
                }
                continue;
            }
            // SignPost: static `SignMC` frame `signPostType` (a single sign texture per frame) with `PostMC` behind it when `signPost` is set.
            if sp.kind == 23 {
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                static SIGNS: std::sync::OnceLock<Vec<Vec<Node>>> = std::sync::OnceLock::new();
                static POST: std::sync::OnceLock<Vec<Node>> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                let signs = SIGNS.get_or_init(|| serde_json::from_str::<Frames>(include_str!("../data/game-SignMC.json")).map(|f| f.frames).unwrap_or_default());
                let post = POST.get_or_init(|| serde_json::from_str::<File>(include_str!("../data/game-PostMC.json")).map(|f| f.k).unwrap_or_default());
                let m = mul(cam, local(f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians(), 1.0, 1.0));
                if sp.flag("signPost").unwrap_or(true) {
                    for n in post {
                        self.draw_node(img, n, m);
                    }
                }
                let frame = (sp.num("signPostType").unwrap_or(1.0).max(1.0) as usize).min(signs.len().max(1)) - 1;
                for n in signs.get(frame).map(Vec::as_slice).unwrap_or(&[]) {
                    self.draw_node(img, n, m);
                }
                continue;
            }
            // Token: `CoinMC.container.gotoAndStop(tokenType)` at the level position; gone once collected (its sensor body is destroyed).
            if sp.kind == 31 {
                static COIN: std::sync::OnceLock<Vec<Node>> = std::sync::OnceLock::new();
                #[derive(Deserialize)]
                struct File {
                    k: Vec<Node>,
                }
                let coin = COIN.get_or_init(|| serde_json::from_str::<File>(include_str!("../data/game-CoinMC.json")).map(|f| f.k).unwrap_or_default());
                if world.special_bodies.get(i).copied().flatten().is_none_or(|b| world.body_center_px(b).is_none()) {
                    continue;
                }
                let frame = sp.num("tokenType").unwrap_or(1.0).max(1.0) as usize;
                let m = mul(cam, local(f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), 0.0, 1.0, 1.0));
                for container in coin {
                    let kids = if frame > 1 { container.alt.get(&frame.to_string()).unwrap_or(&container.k) } else { &container.k };
                    for k in kids {
                        self.draw_node(img, k, m);
                    }
                }
                continue;
            }
            // Rail: static `RailMC` (rail.png, 100×18) at the level position/angle, `mc.width = shapeWidth` (x scale only).
            if sp.kind == 27 {
                let (x, y, a) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
                let sw = f64::from(sp.num("shapeWidth").unwrap_or(250.0));
                self.blit(img, "rail.png", mul(cam, mul(local(x, y, a, sw / 100.0, 1.0), local(-50.0, -9.0, 0.0, 0.5, 0.5))));
                continue;
            }
            // Meteor: `meteor.png` (clip 407×401 at (−205, −201)), scaled by shapeWidth / 400, at the body's centre of mass.
            if sp.kind == 11 {
                let Some(body) = world.special_bodies.get(i).copied().flatten().filter(|&b| world.body_center_px(b).is_some()) else { continue };
                let s = f64::from(sp.num("shapeWidth").unwrap_or(400.0)) / 400.0;
                self.blit(img, "meteor.png", mul(cam, mul(body_matrix(world, body), mul(local(0.0, 0.0, 0.0, s, s), local(-205.0, -201.0, 0.0, 1.0, 1.0)))));
                continue;
            }
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
            // Blood bitmap (`bmd`, 15n−2 × 50, centred on the clip like the row): one `spike_blood` frame per impale at the snapped spike x.
            {
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                static BLOOD: std::sync::OnceLock<Vec<Vec<Node>>> = std::sync::OnceLock::new();
                let frames = BLOOD.get_or_init(|| serde_json::from_str::<Frames>(include_str!("../data/game-spike_blood.json")).map(|f| f.frames).unwrap_or_default());
                for &(x, frame) in world.spike_blood(body) {
                    let sm = mul(m, local(x - width / 2.0, -25.0 - lc_y, 0.0, 1.0, 1.0));
                    for n in frames.get(usize::from(frame).saturating_sub(1)).map(Vec::as_slice).unwrap_or(&[]) {
                        self.draw_node(img, n, sm);
                    }
                }
            }
        }
    }

    /// Van (`createMovieClips`/`paint`): `VanBodyMC` at the van body's centre of mass, one `VanTireMC` per tire body; a van without bodies (not
    /// interactive) is one static clip at the level position with the tires at (±58, 48).
    fn draw_van(&self, img: &mut RgbaImage, sp: &crate::level::Special, parts: &[BodyId], world: &World, cam: M) {
        use std::sync::OnceLock;
        static CLIPS: OnceLock<[Vec<Node>; 2]> = OnceLock::new();
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        let clips = CLIPS.get_or_init(|| {
            let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
            [f(include_str!("../data/game-VanBodyMC.json")), f(include_str!("../data/game-VanTireMC.json"))]
        });
        let draw = |s: &Self, img: &mut RgbaImage, clip: &[Node], m: M| {
            for n in clip {
                s.draw_node(img, n, m);
            }
        };
        if parts.len() == 3 && parts.iter().all(|&b| world.body_center_px(b).is_some()) {
            for (&b, clip) in parts[1..].iter().zip([&clips[1], &clips[1]]).chain([(&parts[0], &clips[0])]) {
                draw(self, img, clip, mul(cam, body_matrix(world, b)));
            }
        } else {
            let (x, y, ang) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
            let m = mul(cam, local(x, y, ang, 1.0, 1.0));
            for tx in [-58.0, 58.0] {
                draw(self, img, &clips[1], mul(m, local(tx, 48.0, 0.0, 1.0, 1.0)));
            }
            draw(self, img, &clips[0], m);
        }
    }

    /// Table / mine / spring box: the library clip tree (`data/game-*MC.json`, dumped from the oracle) at the body's centre of mass (a table without a
    /// body stays at its level position). Spring box: `pad` follows the body lowered by its local-centre y (`pad.y = −localCenter.y`), `base` sits at the
    /// level position and angle (original moves it to the foreground). The timer text / glow / arrows' animation and the launch are not drawn.
    fn draw_clip_special(&self, img: &mut RgbaImage, sp: &crate::level::Special, body: Option<BodyId>, world: &World, cam: M) {
        use std::sync::OnceLock;
        static CLIPS: OnceLock<[Vec<Node>; 5]> = OnceLock::new();
        #[derive(Deserialize)]
        struct File {
            k: Vec<Node>,
        }
        let clips = CLIPS.get_or_init(|| {
            let f = |s: &str| serde_json::from_str::<File>(s).map(|f| f.k).unwrap_or_default();
            [f(include_str!("../data/game-TableMC.json")), f(include_str!("../data/game-MineMC.json")), f(include_str!("../data/game-SpringBoxMC.json")), f(include_str!("../data/game-SoccerBallMC.json")), f(include_str!("../data/game-WreckingBallMC.json"))]
        });
        let (x, y, ang) = (f64::from(sp.num("x").unwrap_or(0.0)), f64::from(sp.num("y").unwrap_or(0.0)), f64::from(sp.num("angle").unwrap_or(0.0)).to_radians());
        let live = body.filter(|&b| world.body_center_px(b).is_some());
        let pose = match live {
            Some(b) => body_matrix(world, b),
            None => local(x, y, ang, 1.0, 1.0),
        };
        let clip = &clips[match sp.kind { 1 => 0, 2 => 1, 5 => 2, 10 => 3, _ => 4 }];
        if sp.kind == 7 {
            // WreckingBall: `ball` at the body's centre of mass, `rope` at the pivot (x, y); both rotated by the ball's angle; `base.y = L − 136.5`,
            // stem stretched to L − 120 (clip height 214.5).
            let Some(b) = live else { return };
            let len = f64::from(sp.num("ropeLength").unwrap_or(350.0));
            let a = f64::from(world.body_angle(b));
            for n in clip {
                match n.n.as_str() {
                    "ball" => {
                        for k in &n.k {
                            self.draw_node(img, k, mul(cam, body_matrix(world, b)));
                        }
                    }
                    "rope" => {
                        let m = mul(cam, local(x, y, a, 1.0, 1.0));
                        for k in &n.k {
                            match k.n.as_str() {
                                "base" => self.draw_node_at(img, k, m, Some(len - 136.5), None),
                                "stem" => self.draw_node_at(img, k, m, None, Some((len - 120.0) / 214.5)),
                                _ => self.draw_node(img, k, m),
                            }
                        }
                    }
                    _ => {}
                }
            }
            return;
        }
        if sp.kind != 5 {
            for n in clip {
                self.draw_node(img, n, mul(cam, pose));
            }
            return;
        }
        let lc_y = live.map_or(0.0, |b| world.b2().body(b).sweep.local_center.y * 62.5);
        for n in clip {
            match n.n.as_str() {
                "pad" => {
                    for k in &n.k {
                        self.draw_node(img, k, mul(cam, mul(pose, local(0.0, -lc_y, 0.0, 1.0, 1.0))));
                    }
                }
                "base" => self.draw_node(img, n, mul(cam, local(x, y, ang, 1.0, 1.0))),
                _ => {}
            }
        }
    }

    pub fn draw(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32)) {
        {
            let mut h = self.hidden.borrow_mut();
            h.clear();
            if matches!(world.bike.as_ref(), Some(crate::vehicle::Vehicle::Heli(heli)) if heli.blade_smashed) {
                h.push("propeller");
            }
            if let Some(crate::vehicle::Vehicle::Santa(s)) = world.bike.as_ref() {
                if s.wheels_gone {
                    h.push("ski");
                }
                if s.stem_gone {
                    h.push("stem");
                }
            }
        }
        *self.queue.borrow_mut() = Some(Vec::new());
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
        let mut parts = self.queue.borrow_mut().take().unwrap_or_default();
        let sleigh = self.layers.get("sleigh").copied().unwrap_or(usize::MAX);
        // (layer, place within it): 0 below the clip (strokes), 1 the clip, 2 and 3 above it
        let key = |b: &Binding| match b.only {
            Some(_) => self.layers.get("lowerArm1").map_or((usize::MAX, 2), |&l| (l, 2)),
            None if self.clip[b.path[0]].n.starts_with("bell") => (sleigh, 3),
            None => (self.layer(&b.path), 1),
        };
        parts.sort_by_key(key);
        let mut lines = world.bike.as_ref().map_or_else(Vec::new, |v| v.strokes(world));
        lines.extend(world.character.iter().flat_map(|d| d.gore_strokes(world, None)));
        for name in ["kid", "girl", "daughter", "son", "elf1", "elf2"] {
            lines.extend(Self::npc_doll(world, name).into_iter().flat_map(|d| d.gore_strokes(world, Some(name))));
        }
        let mut strokes: Vec<_> = lines.into_iter().map(|s| ((self.layers.get(s.layer.as_ref()).copied().unwrap_or(usize::MAX), if s.above { 2 } else { 0 }), s)).collect();
        strokes.sort_by_key(|s| s.0);
        let mut strokes = strokes.into_iter().peekable();
        for b in &parts {
            while let Some((_, s)) = strokes.next_if(|s| s.0 < key(b)) {
                self.draw_stroke(img, camera_offset, &s);
            }
            self.draw_binding(img, world, camera_offset, b);
        }
        for (_, s) in strokes {
            self.draw_stroke(img, camera_offset, &s);
        }
        self.draw_shards(img, world, camera_offset);
    }

    /// An antialiased polyline with flat ends, `width` world px thick. Traced segment by segment as `("line <colour> <width>", [from, to, _])`.
    fn draw_stroke(&self, img: &mut RgbaImage, camera_offset: (f32, f32), stroke: &crate::vehicle::Stroke) {
        if let Some(trace) = self.trace.borrow_mut().as_mut() {
            let m = self.camera((0.0, 0.0));
            for s in stroke.points.windows(2) {
                let at = |p: [f64; 2]| [m[0] * p[0] + m[4], m[3] * p[1] + m[5]];
                trace.push((format!("line {:06x} {}", stroke.color, stroke.width), [at(s[0]), at(s[1]), [0.0, 0.0]]));
            }
        }
        let m = self.camera(camera_offset);
        let at = |p: [f64; 2]| (m[0] * p[0] + m[4], m[3] * p[1] + m[5]);
        let half = stroke.width * m[0] / 2.0;
        let color = [(stroke.color >> 16) as u8, (stroke.color >> 8) as u8, stroke.color as u8];
        // coverage first, so the joints between segments are not blended twice
        let mut covered: HashMap<(u32, u32), f64> = HashMap::new();
        for s in stroke.points.windows(2) {
            let (a, b) = (at(s[0]), at(s[1]));
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len = dx.hypot(dy);
            let pad = half + 1.0;
            let xs = (a.0.min(b.0) - pad).floor().max(0.0) as u32..(a.0.max(b.0) + pad).ceil().clamp(0.0, f64::from(img.width())) as u32;
            for x in xs {
                let ys = (a.1.min(b.1) - pad).floor().max(0.0) as u32..(a.1.max(b.1) + pad).ceil().clamp(0.0, f64::from(img.height())) as u32;
                for y in ys {
                    let (px, py) = (f64::from(x) + 0.5 - a.0, f64::from(y) + 0.5 - a.1);
                    let t = if len > 0.0 { ((px * dx + py * dy) / (len * len)).clamp(0.0, 1.0) } else { 0.0 };
                    let cover = (half + 0.5 - (px - t * dx).hypot(py - t * dy)).clamp(0.0, 1.0);
                    if cover > 0.0 {
                        let c = covered.entry((x, y)).or_insert(0.0);
                        *c = c.max(cover);
                    }
                }
            }
        }
        for ((x, y), a) in covered {
            let d = img.get_pixel_mut(x, y);
            for i in 0..3 {
                d[i] = (f64::from(color[i]) * a + f64::from(d[i]) * (1.0 - a)).round() as u8;
            }
        }
    }

    /// World px → image px for a view whose world-space offset is `offset`.
    fn camera(&self, offset: (f32, f32)) -> M {
        let s = if self.view.get() > 0.0 { self.view.get() } else { 1.0 };
        local(f64::from(offset.0) * s, f64::from(offset.1) * s, 0.0, s, s)
    }

    fn layer(&self, path: &[usize]) -> usize {
        let top = &self.clip[path[0]];
        let mut node = top;
        for &i in &path[1..] {
            node = &node.k[i];
        }
        let key = if matches!(top.n.as_str(), "kid" | "girl" | "daughter" | "son" | "elf1" | "elf2") && path.len() > 1 {
            format!("{}/{}", top.n, top.k[path[1]].n)
        } else {
            node.n.clone()
        };
        self.layers.get(&key).copied().unwrap_or(usize::MAX)
    }

    /// Sprite particles of the shard bursts (`createPointBurst`): the particle's frame of its debris clip, centred on the particle, natural clip scale.
    fn draw_shards(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32)) {
        for p in &world.blood.sprites {
            if let crate::blood::ShardKind::Food(ty) = p.kind {
                #[derive(Deserialize)]
                struct Frames {
                    frames: Vec<Vec<Node>>,
                }
                static FRAMES: std::sync::OnceLock<[Vec<Vec<Node>>; 10]> = std::sync::OnceLock::new();
                let frames = FRAMES.get_or_init(|| {
                    let f = |s: &str| serde_json::from_str::<Frames>(s).map(|f| f.frames).unwrap_or_default();
                    [f(include_str!("../data/game-WatermelonParticlesMC.json")), f(include_str!("../data/game-PumpkinParticlesMC.json")), f(include_str!("../data/game-PineappleParticlesMC.json")), f(include_str!("../data/game-BoomboxShardsMC.json")), f(include_str!("../data/game-TVShardsMC.json")), f(include_str!("../data/game-GlassShardsMC.json")), f(include_str!("../data/game-ToiletParticlesMC.json")), f(include_str!("../data/game-WoodChipsMC.json")), f(include_str!("../data/game-Blood.json")), f(include_str!("../data/game-WaterMC.json"))]
                });
                let Some(children) = frames[usize::from(ty) % 10].get(p.frame) else { continue };
                let mut ext = (0.0, 0.0);
                for c in children {
                    self.extent(c, local(0.0, 0.0, 0.0, 1.0, 1.0), &mut ext);
                }
                let m = mul(self.camera(camera_offset), local(p.pos[0] - ext.0 / 2.0, p.pos[1] - ext.1 / 2.0, 0.0, 1.0, 1.0));
                for c in children {
                    self.draw_node(img, c, m);
                }
                continue;
            }
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
            let m = mul(self.camera(camera_offset), local(p.pos[0] - ext.0 / 2.0, p.pos[1] - ext.1 / 2.0, 0.0, 1.0, 1.0));
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
        for (spine, stem) in [(true, "spine"), (false, "intestine")] {
            for (i, body, p, angle, stretch) in doll.chain_clips(world, spine) {
                self.draw_posed(img, world, camera_offset, parent, &format!("{stem}{}", i + 1), body, ([p.x * 62.5, p.y * 62.5], angle, stretch));
            }
        }
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
    /// Draws clip `name` (inside `parent` for sub-characters) at a placement of its own rather than on `body`.
    #[allow(clippy::too_many_arguments)]
    fn draw_posed(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32), parent: Option<&str>, name: &str, body: BodyId, pose: ([f64; 2], f64, f64)) {
        let mut path = Vec::new();
        let mut nodes = &self.clip;
        for part in parent.into_iter().chain([name]) {
            let Some(i) = nodes.iter().position(|c| c.n == part) else { return };
            path.push(i);
            nodes = &nodes[i].k;
        }
        let mut node = &self.clip[path[0]];
        for &i in &path[1..] {
            node = &node.k[i];
        }
        let rest = local(0.0, 0.0, 0.0, 0.5 * node.sx, 0.5 * node.sy);
        self.draw_binding(img, world, camera_offset, &Binding { pose: Some(pose), only: None, without: None, path, body, rest });
    }

    /// [`draw_loose`](Self::draw_loose) for a clip nested in `parent`.
    fn draw_loose_in(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32), parent: &str, name: &str, body: BodyId) {
        let Some(pi) = self.clip.iter().position(|c| c.n == parent) else { return };
        let Some(ci) = self.clip[pi].k.iter().position(|c| c.n == name) else { return };
        let c = &self.clip[pi].k[ci];
        let b = Binding { pose: None, only: None, without: None, path: vec![pi, ci], body, rest: local(0.0, 0.0, 0.0, 0.5 * c.sx, 0.5 * c.sy) };
        self.draw_binding(img, world, camera_offset, &b);
    }

    /// Draws top-level clip `name` on a body created after the bindings (debris, break pieces).
    fn draw_loose(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32), name: &str, body: BodyId) {
        let Some(part) = self.clip.iter().position(|c| c.n == name) else { return };
        let c = &self.clip[part];
        let b = Binding { pose: None, only: None, without: None, path: vec![part], body, rest: local(0.0, 0.0, 0.0, 0.5 * c.sx, 0.5 * c.sy) };
        self.draw_binding(img, world, camera_offset, &b);
    }

    fn draw_binding(&self, img: &mut RgbaImage, world: &World, camera_offset: (f32, f32), b: &Binding) {
        if world.body_center_px(b.body).is_none() {
            return;
        }
        if let Some(queue) = self.queue.borrow_mut().as_mut() {
            queue.push(b.clone());
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
        if let Some(crate::vehicle::Vehicle::Grinder(g)) = world.bike.as_ref() {
            if b.path.len() == 1 && !g.ejected {
                let held = match self.clip[b.path[0]].n.as_str() {
                    "lowerLeg1" => g.feet_held()[0],
                    "lowerLeg2" => g.feet_held()[1],
                    _ => false,
                };
                if held {
                    return;
                }
            }
        }
        let mut placed = body_matrix(world, b.body);
        if let Some(crate::vehicle::Vehicle::Chair(c)) = world.bike.as_ref() {
            if b.body == c.big_wheel && self.clip[b.path[0]].n == "jet" {
                if c.chair_smashed {
                    return;
                }
                placed = local(placed[4], placed[5], f64::from(world.body_angle(c.chair)) + c.jet_angle, 1.0, 1.0);
            }
        }
        if let (Some(crate::vehicle::Vehicle::Santa(s)), [top, part]) = (world.bike.as_ref(), b.path.as_slice()) {
            let elf = match self.clip[*top].n.as_str() {
                "elf1" => Some(&s.elf1),
                "elf2" => Some(&s.elf2),
                _ => None,
            };
            let leg = match self.clip[*top].k[*part].n.as_str() {
                "lowerLeg1" => Some((0, true)),
                "lowerLeg2" => Some((1, true)),
                "upperLeg1" => Some((0, false)),
                "upperLeg2" => Some((1, false)),
                _ => None,
            };
            if let Some(([x, y], angle)) = elf.zip(leg).and_then(|(elf, (side, lower))| elf.leg_paint(world, side, lower)) {
                placed = local(x * 62.5, y * 62.5, angle, 1.0, 1.0);
            }
        }
        let mut node = &self.clip[b.path[0]];
        for &i in &b.path[1..] {
            node = &node.k[i];
        }
        // These wheel clips only follow their body's position (the chair's wheels are ordinary body clips); `paint` turns its `inner` node, the rest of the clip stays level.
        let spin = (b.path.len() == 1 && matches!(node.n.as_str(), "wheel" | "frontWheel" | "backWheel") && node.k.iter().any(|c| c.n == "inner")).then(|| {
            let angle = f64::from(world.body_angle(b.body));
            placed = local(placed[4], placed[5], 0.0, 1.0, 1.0);
            angle
        });
        if let Some(([x, y], angle, _)) = b.pose {
            placed = local(x, y, angle, 1.0, 1.0);
        }
        let stretch = b.pose.map(|p| p.2);
        let g = mul(self.camera(camera_offset), mul(placed, mul(recentre, b.rest)));
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
        let hands_open = owner.or_else(|| world.character.as_ref().filter(|_| matches!(world.bike.as_ref(), Some(crate::vehicle::Vehicle::Pogo(_))))).is_some_and(|d| d.hands_open);
        // (the pogo man's parts sit deeper in one assembled skeleton)
        let pogo = matches!(world.bike.as_ref(), Some(crate::vehicle::Vehicle::Pogo(_)));
        self.reveal(world, owner.or_else(|| world.character.as_ref().filter(|_| pogo)));
        // a body the mower is grinding is clipped by `targetMaskHolder`'s rect
        self.set_grind_mask(world, b.body, self.camera(camera_offset));
        if let Some(name) = b.only {
            if let Some((child, parent)) = Self::nested(children, name, g) {
                self.draw_node(img, child, parent);
            }
        } else {
            if let Some(name) = b.without {
                self.hidden.borrow_mut().push(name);
            }
            for child in children {
                if hands_open && child.n == "hand" {
                    if let Some(open) = child.alt.get("2") {
                        let m = mul(g, child.matrix());
                        for k in open {
                            self.draw_node(img, k, m);
                        }
                        continue;
                    }
                }
                match world.bike.as_ref() {
                    Some(crate::vehicle::Vehicle::Pogo(p)) if node.n == "frame" && child.n == "spring" && !p.frame_smashed => {
                        self.draw_node_at(img, child, g, None, Some(p.spring_scale(world)));
                        continue;
                    }
                    // the moped's front rim turns with the frame, not with the wheel
                    Some(crate::vehicle::Vehicle::Dirt(d)) if node.n == "frontWheel" && child.n == "rim" => {
                        self.draw_node_turned(img, child, g, f64::from(world.body_angle(d.frame)));
                        continue;
                    }
                    _ => {}
                }
                match (spin, stretch) {
                    (Some(angle), _) if child.n == "inner" => self.draw_node_turned(img, child, g, angle),
                    (_, Some(sx)) if child.n == "inner" => {
                        let m = mul(g, child.matrix_with(child.x, child.y, child.r, sx, child.sy));
                        for k in &child.k {
                            self.draw_node(img, k, m);
                        }
                    }
                    _ => self.draw_node(img, child, g),
                }
            }
            if let Some(name) = b.without {
                self.hidden.borrow_mut().retain(|&n| n != name);
            }
        }
        self.grind_mask.set(None);
        self.show_wound.set(false);
        self.shown.borrow_mut().clear();
    }

    /// The node called `name` under `nodes`, with the transform of its parent.
    fn nested<'a>(nodes: &'a [Node], name: &str, parent: M) -> Option<(&'a Node, M)> {
        nodes.iter().find_map(|n| if n.n == name { Some((n, parent)) } else { Self::nested(&n.k, name, mul(parent, n.matrix())) })
    }

    /// Hidden clips the original makes visible in the current state (its head `helmet` is drawn until it comes off).
    fn reveal(&self, world: &World, owner: Option<&crate::character::Ragdoll>) {
        use crate::vehicle::Vehicle;
        let mut shown = self.shown.borrow_mut();
        shown.clear();
        if let Some(doll) = owner {
            if doll.helmet_on || doll.helmet_geo.is_empty() {
                shown.push("helmet");
            }
            if doll.broken.contains(&crate::character::Limb::Neck) {
                shown.push("neck");
            }
        }
        match world.bike.as_ref() {
            Some(Vehicle::Chair(c)) if c.firing => shown.push("flames"),
            Some(Vehicle::Heli(h)) => {
                if h.magnetized {
                    shown.push("lights");
                }
                if h.blade_smashed {
                    shown.push("brokenPropeller");
                }
            }
            _ => {}
        }
    }

    /// `draw_node` with the node's y and/or vertical scale replaced (runtime-resized clips).
    fn draw_node_at(&self, img: &mut RgbaImage, n: &Node, parent: M, y: Option<f64>, sy: Option<f64>) {
        let m = mul(parent, n.matrix_with(n.x, y.unwrap_or(n.y), n.r, n.sx, sy.unwrap_or(n.sy)));
        for k in &n.k {
            self.draw_node(img, k, m);
        }
    }

    fn draw_node(&self, img: &mut RgbaImage, n: &Node, parent: M) {
        self.draw_node_turned(img, n, parent, n.r);
    }

    /// [`draw_node`](Self::draw_node) with the node's rotation replaced.
    fn draw_node_turned(&self, img: &mut RgbaImage, n: &Node, parent: M, r: f64) {
        let name = n.n.as_str();
        let shown = (name == "wound" && self.show_wound.get()) || self.shown.borrow().contains(&name);
        if ((n.hidden || HIDDEN.contains(&name)) && !shown) || self.hidden.borrow().contains(&name) {
            return;
        }
        let m = mul(parent, n.matrix_with(n.x, n.y, r, n.sx, n.sy));
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
        if let Some(trace) = self.trace.borrow_mut().as_mut() {
            let at = |x: f64, y: f64| [m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5]];
            let (ow, oh) = (f.source.w / ratio, f.source.h / ratio_y);
            trace.push((name.to_string(), [at(0.0, 0.0), at(ow, 0.0), at(0.0, oh)]));
        }
        let (w, h) = (f.sprite_source.w, f.sprite_source.h);
        // texture px (small atlas) → source px (÷ratio) → clip transform
        let full = mul(m, local(f.sprite_source.x / ratio, f.sprite_source.y / ratio_y, 0.0, 1.0 / ratio, 1.0 / ratio_y));
        let Some(inv) = invert(full) else { return };
        let corners = [(0.0, 0.0), (w, 0.0), (0.0, h), (w, h)].map(|(x, y)| (full[0] * x + full[2] * y + full[4], full[1] * x + full[3] * y + full[5]));
        let min_x = corners.iter().map(|c| c.0).fold(f64::MAX, f64::min).floor().max(0.0) as i64;
        let max_x = corners.iter().map(|c| c.0).fold(f64::MIN, f64::max).ceil().min(f64::from(img.width())) as i64;
        let min_y = corners.iter().map(|c| c.1).fold(f64::MAX, f64::min).floor().max(0.0) as i64;
        let max_y = corners.iter().map(|c| c.1).fold(f64::MIN, f64::max).ceil().min(f64::from(img.height())) as i64;
        let clip = self.grind_mask.get();
        for py in min_y..max_y {
            for px in min_x..max_x {
                let (sx, sy) = (px as f64 + 0.5, py as f64 + 0.5);
                if let Some((to_mower, top)) = clip {
                    let mx = to_mower[0] * sx + to_mower[2] * sy + to_mower[4];
                    let my = to_mower[1] * sx + to_mower[3] * sy + to_mower[5];
                    if !(-100.0..=100.0).contains(&mx) || !(top..=top + 100.0).contains(&my) {
                        continue;
                    }
                }
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
