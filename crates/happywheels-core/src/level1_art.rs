//! Artwork of the built-in level (`Level1`): the clip tree (`data/level1-art.json`, transforms only, `analysis/level1-art.py`) with the
//! vector paths of `level1.shapes.txt` and the `level_N.png` rasters read from the user's copy of the original assets.

use std::collections::HashMap;
use std::path::Path;

use image::{Rgba, RgbaImage};
use serde::Deserialize;

use crate::physics::World;
use crate::sprites::SpriteAssets;

/// Affine transform `[a, b, c, d, tx, ty]`.
type M = [f64; 6];

#[derive(Debug, Deserialize)]
struct Node {
    #[serde(default)]
    n: String,
    x: f64,
    y: f64,
    sx: Option<f64>,
    sy: Option<f64>,
    #[serde(default)]
    r: f64,
    #[serde(default)]
    kx: f64,
    #[serde(default)]
    ky: f64,
    /// `setColorTransform` as a 4×5 colour matrix (offsets in 0..1).
    matrix: Option<Vec<f64>>,
    /// Line of `level1.shapes.txt`.
    shape: Option<usize>,
    #[serde(default)]
    k: Vec<Node>,
}

impl Node {
    fn matrix_at(&self, x: f64, y: f64, r: f64) -> M {
        let (sx, sy) = (self.sx.unwrap_or(1.0), self.sy.unwrap_or(1.0));
        // Pixi: the x axis turns by rotation + skew.y, the y axis by rotation - skew.x
        [(r + self.ky).cos() * sx, (r + self.ky).sin() * sx, -(r - self.kx).sin() * sy, (r - self.kx).cos() * sy, x, y]
    }
}

/// One `f` command group of a shape line: every contour is filled on its own. The `h` (hole) command of the file does nothing in
/// the original (`Graphics.addHole` does not exist there), so outlines meant as holes stay filled.
struct Fill {
    color: [f64; 4],
    contours: Vec<Vec<[f64; 2]>>,
}

struct Shape {
    fills: Vec<Fill>,
    /// Lower and upper corner.
    bounds: [f64; 4],
}

fn grow(b: &mut [f64; 4], p: [f64; 2]) {
    *b = [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])];
}

const NOWHERE: [f64; 4] = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];

/// The PixiAnimate draw commands used by the level: `f` colour alpha, `m`, `l`, `q`, `c` (close), `h` (ignored).
fn parse_shape(line: &str) -> Option<Shape> {
    let mut words = line.split_whitespace();
    let mut fills: Vec<Fill> = Vec::new();
    let mut contour: Vec<[f64; 2]> = Vec::new();
    let mut bounds = NOWHERE;
    let finish = |fills: &mut Vec<Fill>, contour: &mut Vec<[f64; 2]>| {
        if contour.len() > 2 {
            if let Some(f) = fills.last_mut() {
                f.contours.push(std::mem::take(contour));
            }
        }
        contour.clear();
    };
    while let Some(op) = words.next() {
        let mut number = || words.next()?.parse::<f64>().ok();
        match op {
            "f" => {
                finish(&mut fills, &mut contour);
                let hex = words.next()?.strip_prefix('#')?;
                let full: String = if hex.len() == 3 { hex.chars().flat_map(|c| [c, c]).collect() } else { hex.to_string() };
                let rgb = u32::from_str_radix(&full, 16).ok()?;
                let alpha = words.next()?.parse::<f64>().ok()?;
                let channel = |s: u32| f64::from((rgb >> s) & 255) / 255.0;
                fills.push(Fill { color: [channel(16), channel(8), channel(0), alpha], contours: Vec::new() });
            }
            "m" => {
                finish(&mut fills, &mut contour);
                let p = [number()?, number()?];
                grow(&mut bounds, p);
                contour.push(p);
            }
            "l" => {
                let p = [number()?, number()?];
                grow(&mut bounds, p);
                contour.push(p);
            }
            "q" => {
                let (c, p) = ([number()?, number()?], [number()?, number()?]);
                let from = *contour.last()?;
                grow(&mut bounds, c);
                grow(&mut bounds, p);
                for i in 1..=8 {
                    let t = f64::from(i) / 8.0;
                    let (a, b, d) = ((1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t), t * t);
                    contour.push([a * from[0] + b * c[0] + d * p[0], a * from[1] + b * c[1] + d * p[1]]);
                }
            }
            "c" => finish(&mut fills, &mut contour),
            "h" => {}
            _ => return None,
        }
    }
    finish(&mut fills, &mut contour);
    Some(Shape { fills, bounds })
}

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

fn apply(m: M, p: [f64; 2]) -> [f64; 2] {
    [m[0] * p[0] + m[2] * p[1] + m[4], m[1] * p[0] + m[3] * p[1] + m[5]]
}

/// Whether the transformed box touches the image.
fn visible(img: &RgbaImage, m: M, b: [f64; 4]) -> bool {
    let mut out = NOWHERE;
    for p in [[b[0], b[1]], [b[2], b[1]], [b[0], b[3]], [b[2], b[3]]] {
        grow(&mut out, apply(m, p));
    }
    out[2] > 0.0 && out[3] > 0.0 && out[0] < f64::from(img.width()) && out[1] < f64::from(img.height())
}

/// `source-over` on straight alpha (the target may be a transparent layer).
fn blend(d: &mut Rgba<u8>, color: [f64; 3], alpha: f64) {
    let under = f64::from(d[3]) / 255.0 * (1.0 - alpha);
    let out = alpha + under;
    if out <= 0.0 {
        return;
    }
    for i in 0..3 {
        d[i] = ((color[i] * 255.0 * alpha + f64::from(d[i]) * under) / out).round().clamp(0.0, 255.0) as u8;
    }
    d[3] = (out * 255.0).round() as u8;
}

/// Samples per pixel side.
const SAMPLES: usize = 2;

/// Multisampling like the original's: every fill of a shape covers or misses each sample, so that outlines sharing an edge leave no
/// seam between them; the samples are averaged onto the image at the end.
struct Canvas {
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    /// Premultiplied colour and alpha per sample.
    samples: Vec<[f32; 4]>,
}

impl Canvas {
    /// Covers the part of `bounds` (image coordinates) inside the image.
    fn new(img: &RgbaImage, bounds: [f64; 4]) -> Option<Self> {
        let x = bounds[0].floor().max(0.0) as usize;
        let y = bounds[1].floor().max(0.0) as usize;
        let width = (bounds[2].ceil().max(0.0) as usize).min(img.width() as usize).checked_sub(x)?;
        let height = (bounds[3].ceil().max(0.0) as usize).min(img.height() as usize).checked_sub(y)?;
        (width > 0 && height > 0).then(|| Canvas { x, y, width, height, samples: vec![[0.0; 4]; width * height * SAMPLES * SAMPLES] })
    }

    /// Fills the outline (image coordinates).
    fn fill(&mut self, contour: &[[f64; 2]], color: [f64; 3], alpha: f64) {
        let mut bounds = NOWHERE;
        contour.iter().for_each(|&p| grow(&mut bounds, p));
        let n = SAMPLES as f64;
        let (row_width, rows) = (self.width * SAMPLES, self.height * SAMPLES);
        // sample (i, j) sits at (x + (i + 0.5) / n, y + (j + 0.5) / n)
        let first = |v: f64, origin: usize| ((v - origin as f64) * n - 0.5).ceil().max(0.0) as usize;
        let (from_row, to_row) = (first(bounds[1], self.y), first(bounds[3], self.y).min(rows));
        let edges: Vec<([f64; 2], [f64; 2])> = (0..contour.len()).map(|i| (contour[i], contour[(i + 1) % contour.len()])).filter(|(a, b)| a[1] != b[1]).collect();
        let a = alpha as f32;
        let source = [color[0] as f32 * a, color[1] as f32 * a, color[2] as f32 * a, a];
        let mut crossings: Vec<f64> = Vec::new();
        for row in from_row..to_row {
            let at = self.y as f64 + (row as f64 + 0.5) / n;
            crossings.clear();
            for (a, b) in &edges {
                if (a[1] <= at) != (b[1] <= at) {
                    crossings.push(a[0] + (at - a[1]) / (b[1] - a[1]) * (b[0] - a[0]));
                }
            }
            crossings.sort_by(f64::total_cmp);
            for span in crossings.chunks_exact(2) {
                let (from, to) = (first(span[0], self.x).min(row_width), first(span[1], self.x).min(row_width));
                for s in &mut self.samples[row * row_width + from..row * row_width + to.max(from)] {
                    *s = std::array::from_fn(|i| source[i] + s[i] * (1.0 - source[3]));
                }
            }
        }
    }

    fn resolve(&self, img: &mut RgbaImage) {
        let row_width = self.width * SAMPLES;
        for j in 0..self.height {
            for i in 0..self.width {
                let mut sum = [0.0f32; 4];
                for sy in 0..SAMPLES {
                    for s in &self.samples[(j * SAMPLES + sy) * row_width + i * SAMPLES..][..SAMPLES] {
                        sum = std::array::from_fn(|c| sum[c] + s[c]);
                    }
                }
                if sum[3] > 0.0 {
                    let color = std::array::from_fn(|c| f64::from(sum[c] / sum[3]));
                    blend(img.get_pixel_mut((self.x + i) as u32, (self.y + j) as u32), color, f64::from(sum[3]) / (SAMPLES * SAMPLES) as f64);
                }
            }
        }
    }
}

/// Nearest-texel blit of a raster placed by `m` (scale and translation only).
fn blit(img: &mut RgbaImage, raster: &RgbaImage, m: M) {
    let (w, h) = (f64::from(raster.width()), f64::from(raster.height()));
    if m[0] <= 0.0 || m[3] <= 0.0 || !visible(img, m, [0.0, 0.0, w, h]) {
        return;
    }
    let range = |from: f64, to: f64, limit: u32| ((from - 0.5).ceil().max(0.0) as u32, ((to - 0.5).ceil().max(0.0) as u32).min(limit));
    let (x0, x1) = range(m[4], m[4] + w * m[0], img.width());
    let (y0, y1) = range(m[5], m[5] + h * m[3], img.height());
    let columns: Vec<u32> = (x0..x1).map(|x| (((f64::from(x) + 0.5 - m[4]) / m[0]) as u32).min(raster.width() - 1)).collect();
    for y in y0..y1 {
        let v = (((f64::from(y) + 0.5 - m[5]) / m[3]) as u32).min(raster.height() - 1);
        for (x, &u) in (x0..x1).zip(&columns) {
            let p = raster.get_pixel(u, v);
            match p[3] {
                0 => {}
                255 => img.put_pixel(x, y, *p),
                a => blend(img.get_pixel_mut(x, y), [f64::from(p[0]) / 255.0, f64::from(p[1]) / 255.0, f64::from(p[2]) / 255.0], f64::from(a) / 255.0),
            }
        }
    }
}

/// `Level1.BM_RASTERS`: raster number and position for the `bm` placeholders, in their order in `backGround`.
const BM_RASTERS: [(u32, f64, f64); 10] = [
    (1, 0.0, 282.0),
    (2, 990.25, 282.71),
    (3, 2496.35, -771.39),
    (4, 4933.75, -910.6),
    (6, 7417.9, -939.29),
    (5, 7334.31, -2168.45),
    (7, 9610.05, 46.69),
    (9, 10753.0, 9764.03),
    (10, 12191.4, -291.73),
    (11, 14913.21, -47.12),
];

pub(crate) struct Level1Art {
    layers: Vec<Node>,
    shapes: HashMap<usize, Shape>,
    rasters: HashMap<u32, RgbaImage>,
}

impl Level1Art {
    pub(crate) fn open(source: &SpriteAssets) -> Option<Self> {
        let folder = source.root().join("animate/level1");
        let layers: Vec<Node> = serde_json::from_str(include_str!("../data/level1-art.json")).ok()?;
        let text = source.string(&folder.join("shapes/level1.shapes.txt"))?;
        let shapes: HashMap<usize, Shape> = text.lines().enumerate().filter_map(|(i, l)| Some((i, parse_shape(l)?))).collect();
        let load = |n: u32| Some((n, source.image(&folder.join(Path::new(&format!("level_{n}.png"))))?.to_rgba8()));
        let rasters = BM_RASTERS.iter().map(|r| r.0).chain([8]).map(load).collect::<Option<_>>()?;
        Some(Self { layers, shapes, rasters })
    }

    fn layer(&self, name: &str) -> Option<&Node> {
        self.layers.iter().find(|n| n.n == name)
    }

    fn node(&self, img: &mut RgbaImage, node: &Node, m: M, colour: Option<&[f64]>) {
        let colour = node.matrix.as_deref().or(colour);
        if let Some(shape) = node.shape.and_then(|i| self.shapes.get(&i)) {
            let mut bounds = NOWHERE;
            let b = shape.bounds;
            for p in [[b[0], b[1]], [b[2], b[1]], [b[0], b[3]], [b[2], b[3]]] {
                grow(&mut bounds, apply(m, p));
            }
            if let Some(mut canvas) = Canvas::new(img, bounds) {
                for f in &shape.fills {
                    let mut c = [f.color[0], f.color[1], f.color[2]];
                    if let Some(t) = colour {
                        c = std::array::from_fn(|i| (t[5 * i] * f.color[0] + t[5 * i + 1] * f.color[1] + t[5 * i + 2] * f.color[2] + t[5 * i + 4]).clamp(0.0, 1.0));
                    }
                    for contour in &f.contours {
                        let placed: Vec<[f64; 2]> = contour.iter().map(|&p| apply(m, p)).collect();
                        canvas.fill(&placed, c, f.color[3]);
                    }
                }
                canvas.resolve(img);
            }
        }
        for k in &node.k {
            self.node(img, k, mul(m, k.matrix_at(k.x, k.y, k.r)), colour);
        }
    }

    /// One of the parallax layers (`backdrop4`, `backdrop2`, `backdrop3`), placed by `m`.
    pub(crate) fn draw_backdrop_layer(&self, img: &mut RgbaImage, name: &str, m: M) {
        for k in self.layer(name).into_iter().flat_map(|l| &l.k) {
            self.node(img, k, mul(m, k.matrix_at(k.x, k.y, k.r)), None);
        }
    }

    /// The tiled `chasm` (raster 8 at half size: four tiles 800 px to the right, then four on the spot).
    fn chasm(&self, img: &mut RgbaImage, view: M, x: f64, y: f64) {
        let Some(raster) = self.rasters.get(&8) else { return };
        let step = f64::from(raster.height()) / 2.0;
        for dx in [800.0, 0.0] {
            for i in 0..4 {
                blit(img, raster, mul(view, [0.5, 0.0, 0.0, 0.5, x + dx, y + f64::from(i) * step]));
            }
        }
    }

    /// `backGround` (behind the characters): rasters, and the clips that follow the level's bodies. `view` maps world px to the image.
    pub(crate) fn draw_background(&self, img: &mut RgbaImage, world: &World, view: M) {
        let Some(background) = self.layer("backGround") else { return };
        let clips = world.level1_clips();
        let mut raster = BM_RASTERS.iter();
        for k in &background.k {
            match k.n.as_str() {
                "bm" => {
                    if let Some((&(_, x, y), image)) = raster.next().and_then(|r| Some((r, self.rasters.get(&r.0)?))) {
                        blit(img, image, mul(view, [0.5, 0.0, 0.0, 0.5, x, y]));
                    }
                }
                "chasm" => self.chasm(img, view, k.x, k.y),
                name => match clips.iter().find(|c| c.name == name) {
                    Some(c) if !c.visible => {}
                    Some(c) => {
                        let m = mul(view, k.matrix_at(c.position[0], c.position[1], c.angle));
                        // the gears turn their drawing (`inner`) by the guide's rotation inside the clip
                        match c.inner {
                            Some(turn) => {
                                for part in &k.k {
                                    let r = if part.n == "inner" { turn } else { part.r };
                                    self.node(img, part, mul(m, part.matrix_at(part.x, part.y, r)), None);
                                }
                            }
                            None => self.node(img, k, m, None),
                        }
                    }
                    None => self.node(img, k, mul(view, k.matrix_at(k.x, k.y, k.r)), None),
                },
            }
        }
    }

    /// `foreGround` (in front of everything).
    pub(crate) fn draw_foreground(&self, img: &mut RgbaImage, view: M) {
        for k in self.layer("foreGround").into_iter().flat_map(|l| &l.k) {
            self.node(img, k, mul(view, k.matrix_at(k.x, k.y, k.r)), None);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hole_commands_do_nothing_and_outlines_sharing_an_edge_leave_no_seam() {
        // a square, an outline inside it marked as a hole, and a third one continuing the square from x = 8.25
        let shape = parse_shape("f #f00 1 m 0 0 l 8.25 0 l 8.25 8 l 0 8 c m 2 2 l 6 2 l 6 6 l 2 6 h m 8.25 0 l 10.5 0 l 10.5 8 l 8.25 8 c").unwrap();
        let mut img = RgbaImage::new(12, 8);
        let mut canvas = Canvas::new(&img, shape.bounds).unwrap();
        for f in &shape.fills {
            for contour in &f.contours {
                canvas.fill(contour, [f.color[0], f.color[1], f.color[2]], f.color[3]);
            }
        }
        canvas.resolve(&mut img);
        assert_eq!(img.get_pixel(1, 1).0, [255, 0, 0, 255]);
        assert_eq!(img.get_pixel(4, 4)[3], 255);
        assert_eq!(img.get_pixel(8, 1)[3], 255);
        assert_eq!(img.get_pixel(10, 1)[3], 128);
    }

}
