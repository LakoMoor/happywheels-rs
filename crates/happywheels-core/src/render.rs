//! CPU renderer for a level snapshot into an RGBA frame (shown by the existing
//! wgpu full-screen-texture path). Colour/outline semantics follow the UNVERIFIED
//! property table in `level.rs`.

use image::{Rgba, RgbaImage};
use imageproc::{drawing::{draw_filled_circle_mut, draw_polygon_mut}, point::Point};

use crate::{level::{Level, ShapeKind}, physics::{DrawShape, World}};

fn colour(shape: &crate::level::Shape) -> Rgba<u8> {
    let c = shape.num("color").unwrap_or(0.0) as u32;
    let a = (shape.num("opacity").unwrap_or(100.0).clamp(0.0, 100.0) * 2.55) as u8;
    Rgba([(c >> 16) as u8, (c >> 8) as u8, c as u8, a.max(1)])
}

/// Draw the level with the camera centred on the start point (or origin).
pub fn render_level(level: &Level, world: &World, width: u32, height: u32) -> RgbaImage {
    render_level_at(level, world, width, height, level.info.start)
}

/// Same as [`render_level`] with the camera centred on `camera` (pixels), e.g. the rider's chest.
pub fn render_level_at(level: &Level, world: &World, width: u32, height: u32, camera: [f32; 2]) -> RgbaImage {
    render_level_sprites(level, world, width, height, camera, None)
}

/// [`render_level_at`] with sprite-drawn bodies (see [`crate::sprites`]); other bodies keep the polygon fallback.
pub fn render_level_sprites(level: &Level, world: &World, width: u32, height: u32, camera: [f32; 2], sprites: Option<&crate::sprites::Sprites>) -> RgbaImage {
    render_level_scene(level, world, width, height, camera, sprites, None)
}

/// [`render_level_sprites`] with a parallax backdrop (`Sprites::open_backdrop`) drawn behind the level.
pub fn render_level_scene(level: &Level, world: &World, width: u32, height: u32, camera: [f32; 2], sprites: Option<&crate::sprites::Sprites>, backdrop: Option<&crate::sprites::Sprites>) -> RgbaImage {
    let bg = level.info.background_color;
    let mut img = RgbaImage::from_pixel(width, height, Rgba([(bg >> 16) as u8, (bg >> 8) as u8, bg as u8, 255]));
    if let Some(b) = backdrop {
        b.draw_backdrop(&mut img, camera);
    }
    let (cx, cy) = (camera[0], camera[1]);
    let to_screen = |x: f32, y: f32| ((x - cx) + width as f32 / 2.0, (y - cy) + height as f32 / 2.0);
    for (i, s) in level.shapes.iter().enumerate() {
        if !world.shape_visible.get(i).copied().unwrap_or(true) { continue; }
        let col = colour(s);
        let frozen = world.shape_frozen_pose.get(i).copied().flatten();
        let (px, py) = match frozen.map(|p| p.0).or_else(|| world.position_px(i)) {
            Some([x, y]) => (x, y),
            None => (s.num("x").unwrap_or(0.0), s.num("y").unwrap_or(0.0)),
        };
        let angle = match frozen.map(|p| p.1) {
            Some(angle) => angle,
            None => match world.shape_bodies.get(i).copied().flatten().filter(|&h| h != world.level_body()) {
            Some(h) => world.body_angle(h),
            None => s.num("angle").unwrap_or(0.0).to_radians(),
            },
        };
        let [ox, oy] = world.shape_render_offset.get(i).copied().unwrap_or([0.0, 0.0]);
        let (sin, cos) = angle.sin_cos();
        draw_shape(&mut img, s, (px + ox * cos - oy * sin, py + ox * sin + oy * cos), angle, col, &to_screen);
    }
    // Specials modelled as rigid bodies: draw each shape in grey.
    for (si, h) in world.special_parts.iter().enumerate().flat_map(|(si, v)| v.iter().map(move |h| (si, h))) {
        if sprites.is_some_and(|s| level.specials.get(si).is_some_and(|sp| s.covers_special(sp.kind))) {
            continue;
        }
        let col = Rgba([0x80, 0x70, 0x60, 255]);
        for sh in world.body_draw_shapes(*h) {
            match sh {
                DrawShape::Poly(pts) => {
                    let pts: Vec<_> = pts.iter().map(|p| to_screen(p[0], p[1])).collect();
                    poly(&mut img, &pts, col);
                }
                DrawShape::Circle { center, radius } => {
                    let (sx, sy) = to_screen(center[0], center[1]);
                    draw_filled_circle_mut(&mut img, (sx as i32, sy as i32), radius as i32, col);
                }
            }
        }
    }
    // NPC ragdolls (and what is left of them after smashes).
    let npc_art = sprites.is_some_and(|s| s.covers_special(17));
    let npc_main = if npc_art { world.npc_main_bodies() } else { Vec::new() };
    for h in world.npc_draw_bodies().into_iter().filter(|h| !(npc_main.contains(h) || sprites.is_some() && world.is_piece_with_art(*h))) {
        let col = Rgba([0x70, 0x50, 0x90, 255]);
        for sh in world.body_draw_shapes(h) {
            match sh {
                DrawShape::Poly(pts) => {
                    let pts: Vec<_> = pts.iter().map(|p| to_screen(p[0], p[1])).collect();
                    poly(&mut img, &pts, col);
                }
                DrawShape::Circle { center, radius } => {
                    let (sx, sy) = to_screen(center[0], center[1]);
                    draw_filled_circle_mut(&mut img, (sx as i32, sy as i32), radius as i32, col);
                }
            }
        }
    }
    // Player ragdoll and bike.
    if let Some(doll) = &world.character {
        let mut bodies: Vec<_> = doll.bodies.values().map(|h| (*h, Rgba([0x20, 0x30, 0x80, 255]))).collect();
        let red = Rgba([0x90, 0x20, 0x20, 255]);
        bodies.extend(doll.chunks.iter().chain(doll.brain.iter()).chain(doll.heart.iter()).chain(doll.feet.iter().flatten()).chain(doll.helmet.iter()).chain(doll.sev.iter()).chain(doll.intestines.iter().flat_map(|c| c.bodies.iter())).chain(doll.spine.iter().flat_map(|c| c.bodies.iter())).chain(doll.ligaments.iter().flatten().flat_map(|c| c.bodies.iter())).map(|h| (*h, red)));
        if let Some(bike) = &world.bike {
            let grey = Rgba([0x55, 0x55, 0x55, 255]);
            bodies.extend(bike.bodies().into_iter().map(|h| (h, grey)));
        }
        for (h, col) in bodies {
            if sprites.is_some_and(|s| s.covers(h)) {
                continue;
            }
            for sh in world.body_draw_shapes(h) {
                match sh {
                    DrawShape::Poly(pts) => {
                        let pts: Vec<_> = pts.iter().map(|p| to_screen(p[0], p[1])).collect();
                        poly(&mut img, &pts, col);
                    }
                    DrawShape::Circle { center, radius } => {
                        let (sx, sy) = to_screen(center[0], center[1]);
                        draw_filled_circle_mut(&mut img, (sx as i32, sy as i32), radius as i32, col);
                    }
                }
            }
        }
    }
    if let Some(sp) = sprites {
        sp.draw_specials(&mut img, level, world, (width as f32 / 2.0 - cx, height as f32 / 2.0 - cy));
        sp.draw(&mut img, world, (width as f32 / 2.0 - cx, height as f32 / 2.0 - cy));
    }
    // Blood: one dark red segment per particle (last → current position), like `BloodParticle.step`.
    for p in &world.blood.particles {
        let (x0, y0) = to_screen(p.last[0] as f32, p.last[1] as f32);
        let (x1, y1) = to_screen(p.cur[0] as f32, p.cur[1] as f32);
        imageproc::drawing::draw_line_segment_mut(&mut img, (x0, y0), (x1, y1), Rgba([0x99, 0x00, 0x00, 255]));
        imageproc::drawing::draw_line_segment_mut(&mut img, (x0, y0 + 1.0), (x1, y1 + 1.0), Rgba([0x99, 0x00, 0x00, 255]));
    }
    // Mower sparks: one yellow segment per spark (tail → head).
    for s in &world.blood.sparks {
        let (x0, y0) = to_screen(s.seg[0][0] as f32, s.seg[0][1] as f32);
        let (x1, y1) = to_screen(s.seg[1][0] as f32, s.seg[1][1] as f32);
        imageproc::drawing::draw_line_segment_mut(&mut img, (x0, y0), (x1, y1), Rgba([0xFF, 0xFF, s.blue, 255]));
    }
    // Groups: children follow the group body (child coords assumed world-space at start).
    for (gi, g) in level.groups.iter().enumerate() {
        if !world.group_visible.get(gi).copied().unwrap_or(true) { continue; }
        let Some(&h) = world.group_bodies.get(gi) else { continue };
        let frozen = world.group_frozen_pose.get(gi).copied().flatten();
        let Some([bx, by]) = frozen.map(|p| p.0).or_else(|| world.body_position_px(h)) else { continue };
        let ba = frozen.map_or_else(|| world.body_angle(h), |p| p.1);
        for s in &g.shapes {
            // verified frame: local = (child.x + offset.x, child.y + offset.y), child angle as-is
            let lx = s.props.first().and_then(|v| v.parse::<f32>().ok()).unwrap_or(0.0) + g.offset[0];
            let ly = s.props.get(1).and_then(|v| v.parse::<f32>().ok()).unwrap_or(0.0) + g.offset[1];
            let (sb, cb) = ba.sin_cos();
            let (px, py) = (bx + lx * cb - ly * sb, by + lx * sb + ly * cb);
            let angle = ba + s.num("angle").unwrap_or(0.0).to_radians();
            draw_shape(&mut img, s, (px, py), angle, colour(s), &to_screen);
        }
    }
    img
}

/// A level shape as `getFlatSprite` draws it: fill `color` (≥ 0; −1 = none), a 1 px outline in `outlineColor` (≥ 0) and the whole sprite at `opacity`;
/// fully transparent or colourless shapes are invisible.
fn draw_shape(img: &mut RgbaImage, s: &crate::level::Shape, (px, py): (f32, f32), angle: f32, _col: Rgba<u8>, to_screen: &dyn Fn(f32, f32) -> (f32, f32)) {
    let rgb = |v: f32| [(v as i64 >> 16) as u8, (v as i64 >> 8) as u8, v as i64 as u8];
    let color = s.num("color").unwrap_or(0.0);
    let outline = s.num("outlineColor").unwrap_or(-1.0);
    let alpha = (s.num("opacity").unwrap_or(100.0) / 100.0).clamp(0.0, 1.0);
    if alpha == 0.0 || (color < 0.0 && outline < 0.0) {
        return;
    }
    let fill = (color >= 0.0).then(|| rgb(color));
    let line = (outline >= 0.0).then(|| rgb(outline));
    let (sin, cos) = angle.sin_cos();
    let rot = |x: f32, y: f32| to_screen(px + x * cos - y * sin, py + x * sin + y * cos);
    let pts: Vec<(f32, f32)> = match s.kind {
        ShapeKind::Circle => {
            let (sx, sy) = to_screen(px, py);
            let r = s.num("shapeWidth").unwrap_or(0.0) / 2.0;
            paint(img, Outline::Circle((sx, sy), r), fill, line, alpha);
            return;
        }
        ShapeKind::Rectangle => {
            let (w, h) = (s.num("shapeWidth").unwrap_or(0.0) / 2.0, s.num("shapeHeight").unwrap_or(0.0) / 2.0);
            vec![rot(-w, -h), rot(w, -h), rot(w, h), rot(-w, h)]
        }
        ShapeKind::Triangle => {
            let (sx, sy) = (s.num("shapeWidth").unwrap_or(100.0) / 100.0, s.num("shapeHeight").unwrap_or(300.0) / 300.0);
            [(0.0, -200.0), (50.0, 100.0), (-50.0, 100.0)].iter().map(|&(x, y)| rot(x * sx, y * sy)).collect()
        }
        ShapeKind::Art if !s.verts.is_empty() => {
            let (sx, sy) = (s.num("shapeWidth").unwrap_or(100.0) / 100.0, s.num("shapeHeight").unwrap_or(100.0) / 100.0);
            let path = art_path(&s.verts, &s.handles, s.complete_fill);
            let pts: Vec<(f32, f32)> = path.iter().map(|v| rot(v[0] * sx, v[1] * sy)).collect();
            // `ArtShape.getFlatSprite`: only a closed path (`completeFill`) is filled; an open one is a stroke, black when no outline colour is set.
            let line = line.or((!s.complete_fill).then_some([0, 0, 0]));
            paint(img, Outline::Poly(pts, s.complete_fill), if s.complete_fill { fill } else { None }, line, alpha);
            return;
        }
        ShapeKind::Polygon | ShapeKind::Art => {
            let (sx, sy) = (s.num("shapeWidth").unwrap_or(100.0) / 100.0, s.num("shapeHeight").unwrap_or(100.0) / 100.0);
            s.verts.iter().map(|v| rot(v[0] * sx, v[1] * sy)).collect()
        }
    };
    paint(img, Outline::Poly(pts, true), fill, line, alpha);
}

enum Outline {
    /// Points and whether the outline closes back to the first point.
    Poly(Vec<(f32, f32)>, bool),
    Circle((f32, f32), f32),
}

/// Draws fill then hairline outline into a scratch layer over the shape's bounding box and composites it with `alpha`.
/// Flattens an art path (`addBezierCommands`: a straight line when both handles are zero, else a cubic with controls `prev.out + prev` and
/// `cur.in + cur`; a closed path adds the last → first segment).
fn art_path(verts: &[[f32; 2]], handles: &[[f32; 4]], closed: bool) -> Vec<[f32; 2]> {
    let mut out = vec![verts[0]];
    let n = verts.len();
    let segments = if closed { n } else { n - 1 };
    for i in 0..segments {
        let (a, b) = (i, (i + 1) % n);
        let ha = handles.get(a).copied().unwrap_or_default();
        let hb = handles.get(b).copied().unwrap_or_default();
        let (p0, p3) = (verts[a], verts[b]);
        let (c1, c2) = ([p0[0] + ha[2], p0[1] + ha[3]], [p3[0] + hb[0], p3[1] + hb[1]]);
        if ha[2] == 0.0 && ha[3] == 0.0 && hb[0] == 0.0 && hb[1] == 0.0 {
            out.push(p3);
        } else {
            for k in 1..=16 {
                let t = k as f32 / 16.0;
                let u = 1.0 - t;
                let w = [u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t];
                out.push([w[0] * p0[0] + w[1] * c1[0] + w[2] * c2[0] + w[3] * p3[0], w[0] * p0[1] + w[1] * c1[1] + w[2] * c2[1] + w[3] * p3[1]]);
            }
        }
    }
    out
}

fn paint(img: &mut RgbaImage, shape: Outline, fill: Option<[u8; 3]>, line: Option<[u8; 3]>, alpha: f32) {
    let (min, max) = match &shape {
        Outline::Poly(p, _) if p.len() >= 2 => (
            (p.iter().map(|q| q.0).fold(f32::MAX, f32::min), p.iter().map(|q| q.1).fold(f32::MAX, f32::min)),
            (p.iter().map(|q| q.0).fold(f32::MIN, f32::max), p.iter().map(|q| q.1).fold(f32::MIN, f32::max)),
        ),
        Outline::Circle(c, r) => ((c.0 - r, c.1 - r), (c.0 + r, c.1 + r)),
        _ => return,
    };
    let (x0, y0) = ((min.0.floor() as i64 - 1).max(0), (min.1.floor() as i64 - 1).max(0));
    let (x1, y1) = ((max.0.ceil() as i64 + 2).min(i64::from(img.width())), (max.1.ceil() as i64 + 2).min(i64::from(img.height())));
    if x1 <= x0 || y1 <= y0 {
        return;
    }
    let mut layer = RgbaImage::new((x1 - x0) as u32, (y1 - y0) as u32);
    let (ox, oy) = (x0 as f32, y0 as f32);
    match &shape {
        Outline::Poly(p, closed) => {
            let local: Vec<(f32, f32)> = p.iter().map(|q| (q.0 - ox, q.1 - oy)).collect();
            if let Some(f) = fill {
                poly(&mut layer, &local, Rgba([f[0], f[1], f[2], 255]));
            }
            if let Some(l) = line {
                let segments = if *closed { local.len() } else { local.len() - 1 };
                for i in 0..segments {
                    imageproc::drawing::draw_line_segment_mut(&mut layer, local[i], local[(i + 1) % local.len()], Rgba([l[0], l[1], l[2], 255]));
                }
            }
        }
        Outline::Circle(c, r) => {
            let (cx, cy) = ((c.0 - ox) as i32, (c.1 - oy) as i32);
            if let Some(f) = fill {
                draw_filled_circle_mut(&mut layer, (cx, cy), *r as i32, Rgba([f[0], f[1], f[2], 255]));
            }
            if let Some(l) = line {
                imageproc::drawing::draw_hollow_circle_mut(&mut layer, (cx, cy), *r as i32, Rgba([l[0], l[1], l[2], 255]));
            }
        }
    }
    for (lx, ly, px) in layer.enumerate_pixels() {
        if px[3] == 0 {
            continue;
        }
        let a = alpha * f32::from(px[3]) / 255.0;
        let d = img.get_pixel_mut(x0 as u32 + lx, y0 as u32 + ly);
        for i in 0..3 {
            d[i] = (f32::from(px[i]) * a + f32::from(d[i]) * (1.0 - a)).round() as u8;
        }
        d[3] = 255;
    }
}

fn poly(img: &mut RgbaImage, pts: &[(f32, f32)], col: Rgba<u8>) {
    let mut p: Vec<Point<i32>> = pts.iter().map(|&(x, y)| Point::new(x as i32, y as i32)).collect();
    p.dedup();
    if p.len() > 1 && p.first() == p.last() {
        p.pop();
    }
    if p.len() >= 3 {
        draw_polygon_mut(img, &p, col);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn draws_floor_and_ball() {
        let level = crate::level::parse(
            "<levelXML><info v='1' x='0' y='400'/><shapes>\
             <sh t='0' p0='0' p1='500' p2='400' p3='20' p4='0' p5='t' p6='f' p7='1' p8='255' p9='-1' p10='100' p11='1'/>\
             <sh t='1' p0='0' p1='300' p2='60' p3='60' p4='0' p5='t' p6='f' p7='1' p8='16711680' p9='-1' p10='100' p11='1' p12='0'/>\
             </shapes></levelXML>",
        ).unwrap();
        let w = World::from_level(&level);
        let img = render_level(&level, &w, 200, 400);
        assert_eq!(img.get_pixel(100, 300), &Rgba([0, 0, 255, 255])); // floor centre
        assert_eq!(img.get_pixel(100, 100), &Rgba([255, 0, 0, 255])); // ball centre
    }
}
