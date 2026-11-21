//! `b2Distance` (GJK on core shapes) and `b2TimeOfImpact` (box2d-pretty.js ~2395-2780).
use crate::collision::{CircleShape, PolygonShape};
use crate::math::*;
use crate::settings::*;
use crate::shape::{Shape, ShapeKind};
use crate::world::Sweep;

const MIN: f64 = 5e-324; // Number.MIN_VALUE

/// A shape as seen by GJK: polygons use their *core* vertices, circles collapse to a point.
enum Gjk<'a> {
    Poly { core: &'a [Vec2] },
    Point(Vec2),
}

impl Gjk<'_> {
    fn first_vertex(&self, xf: &XForm) -> Vec2 {
        match self {
            Gjk::Poly { core } => mul_x(xf, core[0]),
            Gjk::Point(p) => *p,
        }
    }
    fn support(&self, xf: &XForm, dx: f64, dy: f64) -> Vec2 {
        match self {
            Gjk::Poly { core } => {
                let r = &xf.r;
                let s = dx * r.col1.x + dy * r.col1.y;
                let a = dx * r.col2.x + dy * r.col2.y;
                let mut best = 0;
                let mut best_v = core[0].x * s + core[0].y * a;
                for (i, c) in core.iter().enumerate().skip(1) {
                    let d = c.x * s + c.y * a;
                    if d > best_v {
                        best = i;
                        best_v = d;
                    }
                }
                let c = core[best];
                Vec2::new(xf.position.x + (r.col1.x * c.x + r.col2.x * c.y), xf.position.y + (r.col1.y * c.x + r.col2.y * c.y))
            }
            Gjk::Point(p) => *p,
        }
    }
}

fn process_two(x1: &mut Vec2, x2: &mut Vec2, p1: &mut [Vec2; 3], p2: &mut [Vec2; 3], pts: &mut [Vec2; 3]) -> usize {
    let (s, a) = (pts[0], pts[1]);
    let (o, h) = (p1[0], p1[1]);
    let (l, d) = (p2[0], p2[1]);
    let c = -a.x;
    let u = -a.y;
    let mut p = s.x - a.x;
    let mut m = s.y - a.y;
    let f = (p * p + m * m).sqrt();
    p /= f;
    m /= f;
    let mut t = c * p + u * m;
    if t <= 0.0 || f < MIN {
        *x1 = h;
        *x2 = d;
        p1[0] = h;
        p2[0] = d;
        pts[0] = a;
        1
    } else {
        t /= f;
        x1.x = h.x + t * (o.x - h.x);
        x1.y = h.y + t * (o.y - h.y);
        x2.x = d.x + t * (l.x - d.x);
        x2.y = d.y + t * (l.y - d.y);
        2
    }
}

fn process_three(x1: &mut Vec2, x2: &mut Vec2, p1: &mut [Vec2; 3], p2: &mut [Vec2; 3], pts: &mut [Vec2; 3]) -> usize {
    let (s, a, o) = (pts[0], pts[1], pts[2]);
    let (h, l, d) = (p1[0], p1[1], p1[2]);
    let (c, u, p) = (p2[0], p2[1], p2[2]);
    let (m, f, _x, v, y, g) = (s.x, s.y, a.x, a.y, o.x, o.y);
    let w = _x - m;
    let x = v - f;
    let b = y - m;
    let t = g - f;
    let aa = y - _x;
    let cc = g - v;
    let ss = -(m * b + f * t);
    let ee = y * b + g * t;
    let oo = -(_x * aa + v * cc);
    let kk = y * aa + g * cc;
    if ee <= 0.0 && kk <= 0.0 {
        *x1 = d;
        *x2 = p;
        p1[0] = d;
        p2[0] = p;
        pts[0] = o;
        return 1;
    }
    let mm = w * t - x * b;
    let rr = mm * (m * v - f * _x);
    let ii = mm * (_x * g - v * y);
    if ii <= 0.0 && oo >= 0.0 && kk >= 0.0 && oo + kk > 0.0 {
        let q = oo / (oo + kk);
        x1.x = l.x + q * (d.x - l.x);
        x1.y = l.y + q * (d.y - l.y);
        x2.x = u.x + q * (p.x - u.x);
        x2.y = u.y + q * (p.y - u.y);
        p1[0] = d;
        p2[0] = p;
        pts[0] = o;
        return 2;
    }
    let ll = mm * (y * f - g * m);
    if ll <= 0.0 && ss >= 0.0 && ee >= 0.0 && ss + ee > 0.0 {
        let q = ss / (ss + ee);
        x1.x = h.x + q * (d.x - h.x);
        x1.y = h.y + q * (d.y - h.y);
        x2.x = c.x + q * (p.x - c.x);
        x2.y = c.y + q * (p.y - c.y);
        p1[1] = d;
        p2[1] = p;
        pts[1] = o;
        return 2;
    }
    let mut j = ii + ll + rr;
    j = 1.0 / j;
    let bb = ii * j;
    let dd = ll * j;
    let ff = 1.0 - bb - dd;
    x1.x = bb * h.x + dd * l.x + ff * d.x;
    x1.y = bb * h.y + dd * l.y + ff * d.y;
    x2.x = bb * c.x + dd * u.x + ff * p.x;
    x2.y = bb * c.y + dd * u.y + ff * p.y;
    3
}

fn distance_generic(x1: &mut Vec2, x2: &mut Vec2, s1: &Gjk, xf1: &XForm, s2: &Gjk, xf2: &XForm) -> f64 {
    let mut p1 = [Vec2::default(); 3];
    let mut p2 = [Vec2::default(); 3];
    let mut pts = [Vec2::default(); 3];
    let mut count = 0usize;
    *x1 = s1.first_vertex(xf1);
    *x2 = s2.first_vertex(xf2);
    let mut p = 0.0;
    for _ in 0..20 {
        let f = x2.x - x1.x;
        let g_ = x2.y - x1.y;
        let v = s1.support(xf1, f, g_);
        let y = s2.support(xf2, -f, -g_);
        p = f * f + g_ * g_;
        let gx = y.x - v.x;
        let gy = y.y - v.y;
        if p - (f * gx + g_ * gy) <= 0.01 * p {
            if count == 0 {
                *x1 = v;
                *x2 = y;
            }
            return p.sqrt();
        }
        match count {
            0 => {
                p1[0] = v;
                p2[0] = y;
                pts[0] = Vec2::new(gx, gy);
                *x1 = p1[0];
                *x2 = p2[0];
                count += 1;
            }
            1 => {
                p1[1] = v;
                p2[1] = y;
                pts[1] = Vec2::new(gx, gy);
                count = process_two(x1, x2, &mut p1, &mut p2, &mut pts);
            }
            _ => {
                p1[2] = v;
                p2[2] = y;
                pts[2] = Vec2::new(gx, gy);
                count = process_three(x1, x2, &mut p1, &mut p2, &mut pts);
            }
        }
        if count == 3 {
            return 0.0;
        }
        let mut x = -f64::MAX;
        for q in pts.iter().take(count) {
            x = x.max(q.x * q.x + q.y * q.y);
        }
        if count == 3 || p <= 100.0 * MIN * x {
            let f = x2.x - x1.x;
            let g_ = x2.y - x1.y;
            return (f * f + g_ * g_).sqrt();
        }
    }
    p.sqrt()
}

fn world_center(c: &CircleShape, xf: &XForm) -> Vec2 {
    let r = &xf.r;
    Vec2::new(
        xf.position.x + (r.col1.x * c.local_position.x + r.col2.x * c.local_position.y),
        xf.position.y + (r.col1.y * c.local_position.x + r.col2.y * c.local_position.y),
    )
}

fn distance_cc(x1: &mut Vec2, x2: &mut Vec2, c1: &CircleShape, xf1: &XForm, c2: &CircleShape, xf2: &XForm) -> f64 {
    let p1 = world_center(c1, xf1);
    let p2 = world_center(c2, xf2);
    let mut dx = p2.x - p1.x;
    let mut dy = p2.y - p1.y;
    let dist_sq = dx * dx + dy * dy;
    let r1 = c1.radius - TOI_SLOP;
    let r2 = c2.radius - TOI_SLOP;
    let rs = r1 + r2;
    if dist_sq > rs * rs {
        let dist = (dx * dx + dy * dy).sqrt();
        dx /= dist;
        dy /= dist;
        let w = dist - rs;
        x1.x = p1.x + r1 * dx;
        x1.y = p1.y + r1 * dy;
        x2.x = p2.x - r2 * dx;
        x2.y = p2.y - r2 * dy;
        return w;
    }
    if dist_sq > MIN * MIN {
        let d = (dx * dx + dy * dy).sqrt();
        dx /= d;
        dy /= d;
        x1.x = p1.x + r1 * dx;
        x1.y = p1.y + r1 * dy;
        *x2 = *x1;
        return 0.0;
    }
    *x1 = p1;
    *x2 = *x1;
    0.0
}

fn distance_pc(x1: &mut Vec2, x2: &mut Vec2, poly: &[Vec2], xfp: &XForm, c: &CircleShape, xfc: &XForm) -> f64 {
    let point = Gjk::Point(world_center(c, xfc));
    let ident = XForm::default_identity();
    let mut u = distance_generic(x1, x2, &Gjk::Poly { core: poly }, xfp, &point, &ident);
    let p = c.radius - TOI_SLOP;
    if u > p {
        u -= p;
        let mut m = x2.x - x1.x;
        let mut f = x2.y - x1.y;
        let l = (m * m + f * f).sqrt();
        m /= l;
        f /= l;
        x2.x -= p * m;
        x2.y -= p * f;
    } else {
        u = 0.0;
        *x2 = *x1;
    }
    u
}

trait Ident {
    fn default_identity() -> Self;
}
impl Ident for XForm {
    fn default_identity() -> Self {
        XForm { position: Vec2::default(), r: Mat22::identity() }
    }
}

pub fn distance(x1: &mut Vec2, x2: &mut Vec2, s1: &Shape, xf1: &XForm, s2: &Shape, xf2: &XForm) -> f64 {
    match (&s1.kind, &s2.kind) {
        (ShapeKind::Circle(a), ShapeKind::Circle(b)) => distance_cc(x1, x2, a, xf1, b, xf2),
        (ShapeKind::Polygon { core, .. }, ShapeKind::Circle(c)) => distance_pc(x1, x2, core, xf1, c, xf2),
        (ShapeKind::Circle(c), ShapeKind::Polygon { core, .. }) => distance_pc(x2, x1, core, xf2, c, xf1),
        (ShapeKind::Polygon { core: a, .. }, ShapeKind::Polygon { core: b, .. }) => {
            distance_generic(x1, x2, &Gjk::Poly { core: a }, xf1, &Gjk::Poly { core: b }, xf2)
        }
    }
}

pub fn sweep_xform(s: &Sweep, t: f64) -> XForm {
    let mut xf = XForm::default();
    if 1.0 - s.t0 > MIN {
        let n = (t - s.t0) / (1.0 - s.t0);
        xf.position.x = (1.0 - n) * s.c0.x + n * s.c.x;
        xf.position.y = (1.0 - n) * s.c0.y + n * s.c.y;
        xf.r.set_angle((1.0 - n) * s.a0 + n * s.a);
    } else {
        xf.position = s.c;
        xf.r.set_angle(s.a);
    }
    let r = xf.r;
    xf.position.x -= r.col1.x * s.local_center.x + r.col2.x * s.local_center.y;
    xf.position.y -= r.col1.y * s.local_center.x + r.col2.y * s.local_center.y;
    xf
}

pub fn time_of_impact(s1: &Shape, sw1: &Sweep, s2: &Shape, sw2: &Sweep) -> f64 {
    let r1 = s1.sweep_radius;
    let r2 = s2.sweep_radius;
    let t0 = sw1.t0;
    let d = sw1.c.x - sw1.c0.x;
    let u = sw1.c.y - sw1.c0.y;
    let p = sw2.c.x - sw2.c0.x;
    let m = sw2.c.y - sw2.c0.y;
    let f = sw1.a - sw1.a0;
    let w2 = sw2.a - sw2.a0;
    let mut v = 0.0;
    let mut iter = 0;
    let mut target = 0.0;
    let (mut p1, mut p2) = (Vec2::default(), Vec2::default());
    loop {
        let t = (1.0 - v) * t0 + v;
        let xf1 = sweep_xform(sw1, t);
        let xf2 = sweep_xform(sw2, t);
        let dist = distance(&mut p1, &mut p2, s1, &xf1, s2, &xf2);
        if iter == 0 {
            target = if dist > 2.0 * TOI_SLOP {
                1.5 * TOI_SLOP
            } else {
                let lo = 0.05 * TOI_SLOP;
                let hi = dist - 0.5 * TOI_SLOP;
                if lo > hi {
                    lo
                } else {
                    hi
                }
            };
        }
        if dist - target < 0.05 * TOI_SLOP || iter == 20 {
            break;
        }
        let mut x = p2.x - p1.x;
        let mut b = p2.y - p1.y;
        let o = (x * x + b * b).sqrt();
        x /= o;
        b /= o;
        let k = x * (d - p) + b * (u - m) + (if f < 0.0 { -f } else { f }) * r1 + (if w2 < 0.0 { -w2 } else { w2 }) * r2;
        if k == 0.0 {
            v = 1.0;
            break;
        }
        let pn = v + (dist - target) / k;
        if !(0.0..=1.0).contains(&pn) {
            v = 1.0;
            break;
        }
        if pn < (1.0 + 100.0 * MIN) * v {
            break;
        }
        v = pn;
        iter += 1;
    }
    v
}

#[allow(dead_code)]
fn _unused(_: &PolygonShape) {}
