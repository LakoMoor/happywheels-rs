//! Shapes: `b2PolygonShape` / `b2CircleShape` construction, OBB, centroid, mass, AABBs
//! (box2d-pretty.js ~1810-2330, same operation order).
use crate::collision::{Aabb, CircleShape, PolygonShape};
use crate::math::*;
use crate::settings::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Filter {
    pub category_bits: u16,
    pub mask_bits: u16,
    pub group_index: i16,
}

impl Default for Filter {
    fn default() -> Self {
        Self { category_bits: 1, mask_bits: 0xFFFF, group_index: 0 }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MassData {
    pub mass: f64,
    pub center: Vec2,
    pub i: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Obb {
    pub r: Mat22,
    pub center: Vec2,
    pub extents: Vec2,
}

/// Shape definition (`b2ShapeDef` family). Polygon vertices are in body-local metres, CCW.
#[derive(Debug, Clone)]
pub enum Geometry {
    Circle { local_position: Vec2, radius: f64 },
    Polygon(Vec<Vec2>),
}

#[derive(Debug, Clone)]
pub struct ShapeDef {
    pub geometry: Geometry,
    pub friction: f64,
    pub restitution: f64,
    pub density: f64,
    pub is_sensor: bool,
    pub filter: Filter,
    pub user_data: u64,
}

impl ShapeDef {
    pub fn new(geometry: Geometry) -> Self {
        Self { geometry, friction: 0.2, restitution: 0.0, density: 0.0, is_sensor: false, filter: Filter::default(), user_data: 0 }
    }
    /// `SetAsBox(hx, hy)`.
    pub fn boxed(hx: f64, hy: f64) -> Self {
        Self::new(Geometry::Polygon(vec![Vec2::new(-hx, -hy), Vec2::new(hx, -hy), Vec2::new(hx, hy), Vec2::new(-hx, hy)]))
    }
    /// `SetAsOrientedBox(hx, hy, center, angle)`.
    pub fn oriented_box(hx: f64, hy: f64, center: Vec2, angle: f64) -> Self {
        let m = Mat22::from_angle(angle);
        let v = [Vec2::new(-hx, -hy), Vec2::new(hx, -hy), Vec2::new(hx, hy), Vec2::new(-hx, hy)]
            .map(|p| Vec2::new(center.x + (m.col1.x * p.x + m.col2.x * p.y), center.y + (m.col1.y * p.x + m.col2.y * p.y)));
        Self::new(Geometry::Polygon(v.to_vec()))
    }
    pub fn circle(radius: f64) -> Self {
        Self::new(Geometry::Circle { local_position: Vec2::default(), radius })
    }
}

#[derive(Debug, Clone)]
pub enum ShapeKind {
    Circle(CircleShape),
    Polygon { poly: PolygonShape, obb: Obb, core: Vec<Vec2> },
}

#[derive(Debug, Clone)]
pub struct Shape {
    pub kind: ShapeKind,
    pub friction: f64,
    pub restitution: f64,
    pub density: f64,
    pub is_sensor: bool,
    pub filter: Filter,
    pub user_data: u64,
    pub material: i32,
    pub sweep_radius: f64,
}

fn compute_centroid(v: &[Vec2]) -> Vec2 {
    let mut c = Vec2::default();
    let mut area = 0.0;
    let inv3 = 1.0 / 3.0;
    let n = v.len();
    for i in 0..n {
        let a = v[i];
        let b = if i + 1 < n { v[i + 1] } else { v[0] };
        let (e1x, e1y) = (a.x - 0.0, a.y - 0.0);
        let e2x = b.x - 0.0;
        let tri = 0.5 * (e1x * (b.y - 0.0) - e1y * e2x);
        area += tri;
        c.x += tri * inv3 * (0.0 + a.x + b.x);
        c.y += tri * inv3 * (0.0 + a.y + b.y);
    }
    c.x *= 1.0 / area;
    c.y *= 1.0 / area;
    c
}

fn compute_obb(v: &[Vec2]) -> Obb {
    let n = v.len();
    let mut p: Vec<Vec2> = v.to_vec();
    p.push(v[0]);
    let mut min_area = f64::MAX;
    let mut obb = Obb::default();
    for i in 1..=n {
        let root = p[i - 1];
        let mut ux = p[i].x - root.x;
        let mut uy = p[i].y - root.y;
        let l = (ux * ux + uy * uy).sqrt();
        uy /= l;
        ux /= l;
        let (vx, vy) = (-uy, ux);
        let (mut lx, mut ly, mut hx, mut hy) = (f64::MAX, f64::MAX, -f64::MAX, -f64::MAX);
        for q in v.iter().take(n) {
            let dx = q.x - root.x;
            let dy = q.y - root.y;
            let px = ux * dx + uy * dy;
            let py = vx * dx + vy * dy;
            if px < lx {
                lx = px;
            }
            if py < ly {
                ly = py;
            }
            if px > hx {
                hx = px;
            }
            if py > hy {
                hy = py;
            }
        }
        let area = (hx - lx) * (hy - ly);
        if area < 0.95 * min_area {
            min_area = area;
            obb.r.col1 = Vec2::new(ux, uy);
            obb.r.col2 = Vec2::new(vx, vy);
            let cx = 0.5 * (lx + hx);
            let cy = 0.5 * (ly + hy);
            obb.center = Vec2::new(root.x + (obb.r.col1.x * cx + obb.r.col2.x * cy), root.y + (obb.r.col1.y * cx + obb.r.col2.y * cy));
            obb.extents = Vec2::new(0.5 * (hx - lx), 0.5 * (hy - ly));
        }
    }
    obb
}

impl Shape {
    pub fn new(def: &ShapeDef) -> Self {
        let kind = match &def.geometry {
            Geometry::Circle { local_position, radius } => ShapeKind::Circle(CircleShape { local_position: *local_position, radius: *radius }),
            Geometry::Polygon(v) => {
                let n = v.len();
                let vertices = v.clone();
                let mut normals = Vec::with_capacity(n);
                for i in 0..n {
                    let j = if i + 1 < n { i + 1 } else { 0 };
                    let h = vertices[j].x - vertices[i].x;
                    let d = vertices[j].y - vertices[i].y;
                    let u = (h * h + d * d).sqrt();
                    normals.push(Vec2::new(d / u, -h / u));
                }
                let centroid = compute_centroid(v);
                let obb = compute_obb(&vertices);
                let mut core = Vec::with_capacity(n);
                for i in 0..n {
                    let a = if i >= 1 { i - 1 } else { n - 1 };
                    let o = i;
                    let (p, m) = (normals[a].x, normals[a].y);
                    let (f, _s) = (normals[o].x, normals[o].y);
                    let vx = vertices[i].x - centroid.x;
                    let vy = vertices[i].y - centroid.y;
                    let g = p * vx + m * vy - TOI_SLOP;
                    let w = f * vx + _s * vy - TOI_SLOP;
                    let x = 1.0 / (p * _s - m * f);
                    core.push(Vec2::new(x * (_s * g - m * w) + centroid.x, x * (p * w - f * g) + centroid.y));
                }
                ShapeKind::Polygon { poly: PolygonShape { vertices, normals, centroid }, obb, core }
            }
        };
        Shape {
            kind,
            friction: def.friction,
            restitution: def.restitution,
            density: def.density,
            is_sensor: def.is_sensor,
            filter: def.filter,
            user_data: def.user_data,
            material: 0,
            sweep_radius: 0.0,
        }
    }

    pub fn compute_aabb(&self, xf: &XForm) -> Aabb {
        match &self.kind {
            ShapeKind::Circle(c) => {
                let r = &xf.r;
                let x = xf.position.x + (r.col1.x * c.local_position.x + r.col2.x * c.local_position.y);
                let y = xf.position.y + (r.col1.y * c.local_position.x + r.col2.y * c.local_position.y);
                Aabb { lower: Vec2::new(x - c.radius, y - c.radius), upper: Vec2::new(x + c.radius, y + c.radius) }
            }
            ShapeKind::Polygon { obb, .. } => {
                let r = &xf.r;
                let i1 = obb.r.col1;
                let i2 = obb.r.col2;
                let mut s = Mat22 {
                    col1: Vec2::new(r.col1.x * i1.x + r.col2.x * i1.y, r.col1.y * i1.x + r.col2.y * i1.y),
                    col2: Vec2::new(r.col1.x * i2.x + r.col2.x * i2.y, r.col1.y * i2.x + r.col2.y * i2.y),
                };
                for c in [&mut s.col1, &mut s.col2] {
                    c.x = c.x.abs();
                    c.y = c.y.abs();
                }
                let e = obb.extents;
                let hx = s.col1.x * e.x + s.col2.x * e.y;
                let hy = s.col1.y * e.x + s.col2.y * e.y;
                let c = obb.center;
                let cx = xf.position.x + (r.col1.x * c.x + r.col2.x * c.y);
                let cy = xf.position.y + (r.col1.y * c.x + r.col2.y * c.y);
                Aabb { lower: Vec2::new(cx - hx, cy - hy), upper: Vec2::new(cx + hx, cy + hy) }
            }
        }
    }

    pub fn compute_swept_aabb(&self, a: &XForm, b: &XForm) -> Aabb {
        match &self.kind {
            ShapeKind::Circle(c) => {
                let p = |xf: &XForm| {
                    let r = &xf.r;
                    (
                        xf.position.x + (r.col1.x * c.local_position.x + r.col2.x * c.local_position.y),
                        xf.position.y + (r.col1.y * c.local_position.x + r.col2.y * c.local_position.y),
                    )
                };
                let (x1, y1) = p(a);
                let (x2, y2) = p(b);
                Aabb {
                    lower: Vec2::new((if x1 < x2 { x1 } else { x2 }) - c.radius, (if y1 < y2 { y1 } else { y2 }) - c.radius),
                    upper: Vec2::new((if x1 > x2 { x1 } else { x2 }) + c.radius, (if y1 > y2 { y1 } else { y2 }) + c.radius),
                }
            }
            ShapeKind::Polygon { .. } => {
                let (s1, s2) = (self.compute_aabb(a), self.compute_aabb(b));
                Aabb {
                    lower: Vec2::new(
                        if s1.lower.x < s2.lower.x { s1.lower.x } else { s2.lower.x },
                        if s1.lower.y < s2.lower.y { s1.lower.y } else { s2.lower.y },
                    ),
                    upper: Vec2::new(
                        if s1.upper.x > s2.upper.x { s1.upper.x } else { s2.upper.x },
                        if s1.upper.y > s2.upper.y { s1.upper.y } else { s2.upper.y },
                    ),
                }
            }
        }
    }

    pub fn compute_mass(&self) -> MassData {
        match &self.kind {
            ShapeKind::Circle(c) => {
                let mass = self.density * PI * c.radius * c.radius;
                MassData {
                    mass,
                    center: c.local_position,
                    i: mass * (0.5 * c.radius * c.radius + (c.local_position.x * c.local_position.x + c.local_position.y * c.local_position.y)),
                }
            }
            ShapeKind::Polygon { poly, .. } => {
                let (mut cx, mut cy, mut area, mut i) = (0.0, 0.0, 0.0, 0.0);
                let k_inv3 = 1.0 / 3.0;
                let n = poly.vertices.len();
                for a in 0..n {
                    let p1 = poly.vertices[a];
                    let p2 = if a + 1 < n { poly.vertices[a + 1] } else { poly.vertices[0] };
                    let (e1x, e1y) = (p1.x - 0.0, p1.y - 0.0);
                    let (e2x, e2y) = (p2.x - 0.0, p2.y - 0.0);
                    let d = e1x * e2y - e1y * e2x;
                    let tri = 0.5 * d;
                    area += tri;
                    cx += tri * k_inv3 * (0.0 + p1.x + p2.x);
                    cy += tri * k_inv3 * (0.0 + p1.y + p2.y);
                    i += d
                        * (k_inv3 * (0.25 * (e1x * e1x + e2x * e1x + e2x * e2x) + (0.0 * e1x + 0.0 * e2x))
                            + 0.0
                            + (k_inv3 * (0.25 * (e1y * e1y + e2y * e1y + e2y * e2y) + (0.0 * e1y + 0.0 * e2y)) + 0.0));
                }
                let mass = self.density * area;
                MassData { mass, center: Vec2::new(cx * (1.0 / area), cy * (1.0 / area)), i: self.density * i }
            }
        }
    }

    /// `UpdateSweepRadius(center)` — distance of the farthest core vertex from the body's local center.
    pub fn update_sweep_radius(&mut self, c: Vec2) {
        match &self.kind {
            ShapeKind::Circle(s) => {
                let dx = s.local_position.x - c.x;
                let dy = s.local_position.y - c.y;
                self.sweep_radius = (dx * dx + dy * dy).sqrt() + s.radius - TOI_SLOP;
            }
            ShapeKind::Polygon { core, .. } => {
                self.sweep_radius = 0.0;
                for v in core {
                    let dx = v.x - c.x;
                    let dy = v.y - c.y;
                    let d = (dx * dx + dy * dy).sqrt();
                    if d > self.sweep_radius {
                        self.sweep_radius = d;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_mass_centroid_and_inertia() {
        let mut d = ShapeDef::boxed(1.0, 0.5);
        d.density = 2.0;
        let s = Shape::new(&d);
        let m = s.compute_mass();
        assert!((m.mass - 4.0).abs() < 1e-12);
        assert!(m.center.x.abs() < 1e-12 && m.center.y.abs() < 1e-12);
        // I = m (w² + h²) / 12 with w = 2, h = 1
        assert!((m.i - 4.0 * 5.0 / 12.0).abs() < 1e-12, "{}", m.i);
    }

    #[test]
    fn circle_mass_and_aabb() {
        let mut d = ShapeDef::circle(0.5);
        d.density = 1.0;
        let s = Shape::new(&d);
        let m = s.compute_mass();
        assert!((m.mass - PI * 0.25).abs() < 1e-12);
        let xf = XForm { position: Vec2::new(3.0, 4.0), r: Mat22::identity() };
        let a = s.compute_aabb(&xf);
        assert_eq!((a.lower.x, a.upper.y), (2.5, 4.5));
    }

    #[test]
    fn oriented_box_aabb_rotates() {
        let s = Shape::new(&ShapeDef::boxed(1.0, 0.5));
        let xf = XForm { position: Vec2::default(), r: Mat22::from_angle(PI / 2.0) };
        let a = s.compute_aabb(&xf);
        assert!((a.upper.x - 0.5).abs() < 1e-12 && (a.upper.y - 1.0).abs() < 1e-12, "{a:?}");
    }
}
