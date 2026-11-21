//! Contact manifolds: `b2CollideCircles`, `b2CollidePolygonAndCircle`, `b2CollidePolygons`
//! (transcribed from box2d-pretty.js lines ~1400-1760, same operation order).
use crate::math::*;
use crate::settings::*;

/// `b2ContactID`: four feature bytes, packed into `key` (low byte = reference edge).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ContactId {
    pub reference_edge: u8,
    pub incident_edge: u8,
    pub incident_vertex: u8,
    pub flip: u8,
}

impl ContactId {
    pub fn key(&self) -> u32 {
        self.reference_edge as u32 | (self.incident_edge as u32) << 8 | (self.incident_vertex as u32) << 16 | (self.flip as u32) << 24
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ManifoldPoint {
    pub local_point1: Vec2,
    pub local_point2: Vec2,
    pub separation: f64,
    pub normal_impulse: f64,
    pub tangent_impulse: f64,
    pub id: ContactId,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Manifold {
    pub points: [ManifoldPoint; MAX_MANIFOLD_POINTS],
    pub normal: Vec2,
    pub point_count: usize,
}

#[derive(Debug, Clone)]
pub struct PolygonShape {
    pub vertices: Vec<Vec2>,
    pub normals: Vec<Vec2>,
    pub centroid: Vec2,
}

impl PolygonShape {
    pub fn count(&self) -> usize {
        self.vertices.len()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CircleShape {
    pub local_position: Vec2,
    pub radius: f64,
}

#[derive(Clone, Copy, Default)]
struct ClipVertex {
    v: Vec2,
    id: ContactId,
}

fn clip_segment_to_line(out: &mut [ClipVertex; 2], input: &[ClipVertex; 2], n: Vec2, offset: f64) -> usize {
    let mut count = 0;
    let (a, b) = (input[0].v, input[1].v);
    let d1 = dot(n, a) - offset;
    let d2 = dot(n, b) - offset;
    if d1 <= 0.0 {
        out[count] = input[0];
        count += 1;
    }
    if d2 <= 0.0 {
        out[count] = input[1];
        count += 1;
    }
    if d1 * d2 < 0.0 {
        let u = d1 / (d1 - d2);
        out[count].v = Vec2::new(a.x + u * (b.x - a.x), a.y + u * (b.y - a.y));
        out[count].id = if d1 > 0.0 { input[0].id } else { input[1].id };
        count += 1;
    }
    count
}

fn edge_separation(poly1: &PolygonShape, xf1: &XForm, edge1: usize, poly2: &PolygonShape, xf2: &XForm) -> f64 {
    let n = poly1.normals[edge1];
    let r = &xf1.r;
    let nx = r.col1.x * n.x + r.col2.x * n.y;
    let ny = r.col1.y * n.x + r.col2.y * n.y;
    let r2 = &xf2.r;
    let px = r2.col1.x * nx + r2.col1.y * ny;
    let py = r2.col2.x * nx + r2.col2.y * ny;
    let mut idx = 0;
    let mut min_dot = f64::MAX;
    for (i, v) in poly2.vertices.iter().enumerate() {
        let d = v.x * px + v.y * py;
        if d < min_dot {
            min_dot = d;
            idx = i;
        }
    }
    let v1 = poly1.vertices[edge1];
    let v1x = xf1.position.x + (r.col1.x * v1.x + r.col2.x * v1.y);
    let v1y = xf1.position.y + (r.col1.y * v1.x + r.col2.y * v1.y);
    let v2 = poly2.vertices[idx];
    let v2x = xf2.position.x + (r2.col1.x * v2.x + r2.col2.x * v2.y);
    let v2y = xf2.position.y + (r2.col1.y * v2.x + r2.col2.y * v2.y);
    (v2x - v1x) * nx + (v2y - v1y) * ny
}

/// Returns (best edge, separation); a separation > 0 means a separating axis was found early.
fn find_max_separation(poly1: &PolygonShape, xf1: &XForm, poly2: &PolygonShape, xf2: &XForm) -> (usize, f64) {
    let count = poly1.count();
    let c2 = poly2.centroid;
    let mut dx = xf2.position.x + (xf2.r.col1.x * c2.x + xf2.r.col2.x * c2.y);
    let mut dy = xf2.position.y + (xf2.r.col1.y * c2.x + xf2.r.col2.y * c2.y);
    let c1 = poly1.centroid;
    dx -= xf1.position.x + (xf1.r.col1.x * c1.x + xf1.r.col2.x * c1.y);
    dy -= xf1.position.y + (xf1.r.col1.y * c1.x + xf1.r.col2.y * c1.y);
    let dlx = dx * xf1.r.col1.x + dy * xf1.r.col1.y;
    let dly = dx * xf1.r.col2.x + dy * xf1.r.col2.y;
    let mut best = 0;
    let mut max_dot = -f64::MAX;
    for (i, n) in poly1.normals.iter().enumerate() {
        let d = n.x * dlx + n.y * dly;
        if d > max_dot {
            max_dot = d;
            best = i;
        }
    }
    let s = edge_separation(poly1, xf1, best, poly2, xf2);
    if s > 0.0 {
        return (best, s);
    }
    let prev = if best >= 1 { best - 1 } else { count - 1 };
    let s_prev = edge_separation(poly1, xf1, prev, poly2, xf2);
    if s_prev > 0.0 {
        return (prev, s_prev);
    }
    let next = if best + 1 < count { best + 1 } else { 0 };
    let s_next = edge_separation(poly1, xf1, next, poly2, xf2);
    if s_next > 0.0 {
        return (next, s_next);
    }
    let (inc, mut edge, mut best_sep);
    if s_prev > s && s_prev > s_next {
        inc = -1i32;
        edge = prev;
        best_sep = s_prev;
    } else if s_next > s {
        inc = 1;
        edge = next;
        best_sep = s_next;
    } else {
        return (best, s);
    }
    loop {
        let e = if inc == -1 {
            if edge >= 1 {
                edge - 1
            } else {
                count - 1
            }
        } else if edge + 1 < count {
            edge + 1
        } else {
            0
        };
        let sep = edge_separation(poly1, xf1, e, poly2, xf2);
        if sep > 0.0 {
            return (e, sep);
        }
        if !(sep > best_sep) {
            break;
        }
        edge = e;
        best_sep = sep;
    }
    (edge, best_sep)
}

fn find_incident_edge(c: &mut [ClipVertex; 2], poly1: &PolygonShape, xf1: &XForm, edge1: usize, poly2: &PolygonShape, xf2: &XForm) {
    let n = poly1.normals[edge1];
    let a = &xf1.r;
    let u = a.col1.x * n.x + a.col2.x * n.y;
    let p = a.col1.y * n.x + a.col2.y * n.y;
    let b = &xf2.r;
    let nx = b.col1.x * u + b.col1.y * p;
    let ny = b.col2.x * u + b.col2.y * p;
    let mut idx = 0;
    let mut min_dot = f64::MAX;
    for (i, n2) in poly2.normals.iter().enumerate() {
        let d = nx * n2.x + ny * n2.y;
        if d < min_dot {
            min_dot = d;
            idx = i;
        }
    }
    let count = poly2.count();
    let i1 = idx;
    let i2 = if i1 + 1 < count { i1 + 1 } else { 0 };
    for (k, vi) in [(0usize, i1), (1, i2)] {
        let v = poly2.vertices[vi];
        c[k].v = Vec2::new(xf2.position.x + (b.col1.x * v.x + b.col2.x * v.y), xf2.position.y + (b.col1.y * v.x + b.col2.y * v.y));
        c[k].id = ContactId {
            reference_edge: edge1 as u8,
            incident_edge: vi as u8,
            incident_vertex: k as u8,
            flip: 0,
        };
    }
}

pub fn collide_polygons(m: &mut Manifold, poly1: &PolygonShape, xf1: &XForm, poly2: &PolygonShape, xf2: &XForm) {
    m.point_count = 0;
    let (edge1, sep1) = find_max_separation(poly1, xf1, poly2, xf2);
    if sep1 > 0.0 {
        return;
    }
    let (edge2, sep2) = find_max_separation(poly2, xf2, poly1, xf1);
    if sep2 > 0.0 {
        return;
    }
    let (ref_poly, inc_poly, xf_ref, xf_inc, edge, flip);
    if sep2 > 0.98 * sep1 + 0.001 {
        (ref_poly, inc_poly, xf_ref, xf_inc, edge, flip) = (poly2, poly1, *xf2, *xf1, edge2, 1u8);
    } else {
        (ref_poly, inc_poly, xf_ref, xf_inc, edge, flip) = (poly1, poly2, *xf1, *xf2, edge1, 0u8);
    }
    let mut incident = [ClipVertex::default(); 2];
    find_incident_edge(&mut incident, ref_poly, &xf_ref, edge, inc_poly, &xf_inc);
    let count = ref_poly.count();
    let v11 = ref_poly.vertices[edge];
    let v12 = if edge + 1 < count { ref_poly.vertices[edge + 1] } else { ref_poly.vertices[0] };
    let mut side = mul_mv(&xf_ref.r, v12 - v11);
    side.normalize();
    let normal = cross_vf(side, 1.0);
    let v11 = mul_x(&xf_ref, v11);
    let v12 = mul_x(&xf_ref, v12);
    let front_offset = dot(normal, v11);
    let side_offset1 = -dot(side, v11);
    let side_offset2 = dot(side, v12);
    let mut clip1 = [ClipVertex::default(); 2];
    let mut clip2 = [ClipVertex::default(); 2];
    if clip_segment_to_line(&mut clip1, &incident, side.negative(), side_offset1) < 2 {
        return;
    }
    if clip_segment_to_line(&mut clip2, &clip1, side, side_offset2) < 2 {
        return;
    }
    m.normal = if flip != 0 { normal.negative() } else { normal };
    let mut n = 0;
    for cv in clip2.iter().take(MAX_MANIFOLD_POINTS) {
        let sep = dot(normal, cv.v) - front_offset;
        if sep <= 0.0 {
            let p = &mut m.points[n];
            p.separation = sep;
            p.local_point1 = mul_xt(xf1, cv.v);
            p.local_point2 = mul_xt(xf2, cv.v);
            p.id = cv.id;
            p.id.flip = flip;
            n += 1;
        }
    }
    m.point_count = n;
}

pub fn collide_circles(m: &mut Manifold, c1: &CircleShape, xf1: &XForm, c2: &CircleShape, xf2: &XForm) {
    m.point_count = 0;
    let a = c1.local_position;
    let mut p1x = xf1.position.x + (xf1.r.col1.x * a.x + xf1.r.col2.x * a.y);
    let mut p1y = xf1.position.y + (xf1.r.col1.y * a.x + xf1.r.col2.y * a.y);
    let b = c2.local_position;
    let mut p2x = xf2.position.x + (xf2.r.col1.x * b.x + xf2.r.col2.x * b.y);
    let mut p2y = xf2.position.y + (xf2.r.col1.y * b.x + xf2.r.col2.y * b.y);
    let dx = p2x - p1x;
    let dy = p2y - p1y;
    let dist_sq = dx * dx + dy * dy;
    let (r1, r2) = (c1.radius, c2.radius);
    let radius = r1 + r2;
    if dist_sq > radius * radius {
        return;
    }
    let separation;
    if dist_sq < 5e-324 {
        separation = -radius;
        m.normal = Vec2::new(0.0, 1.0);
    } else {
        let dist = dist_sq.sqrt();
        separation = dist - radius;
        let inv = 1.0 / dist;
        m.normal = Vec2::new(inv * dx, inv * dy);
    }
    m.point_count = 1;
    let p = &mut m.points[0];
    p.id = ContactId::default();
    p.separation = separation;
    p1x += r1 * m.normal.x;
    p1y += r1 * m.normal.y;
    p2x -= r2 * m.normal.x;
    p2y -= r2 * m.normal.y;
    let mx = 0.5 * (p1x + p2x);
    let my = 0.5 * (p1y + p2y);
    let tx = mx - xf1.position.x;
    let ty = my - xf1.position.y;
    p.local_point1 = Vec2::new(tx * xf1.r.col1.x + ty * xf1.r.col1.y, tx * xf1.r.col2.x + ty * xf1.r.col2.y);
    let tx = mx - xf2.position.x;
    let ty = my - xf2.position.y;
    p.local_point2 = Vec2::new(tx * xf2.r.col1.x + ty * xf2.r.col1.y, tx * xf2.r.col2.x + ty * xf2.r.col2.y);
}

pub fn collide_polygon_and_circle(m: &mut Manifold, poly: &PolygonShape, xf1: &XForm, circle: &CircleShape, xf2: &XForm) {
    m.point_count = 0;
    let c = circle.local_position;
    let cx = xf2.position.x + (xf2.r.col1.x * c.x + xf2.r.col2.x * c.y);
    let cy = xf2.position.y + (xf2.r.col1.y * c.x + xf2.r.col2.y * c.y);
    let dx = cx - xf1.position.x;
    let dy = cy - xf1.position.y;
    let lx = dx * xf1.r.col1.x + dy * xf1.r.col1.y;
    let ly = dx * xf1.r.col2.x + dy * xf1.r.col2.y;
    let mut normal_index = 0usize;
    let mut separation = -f64::MAX;
    let radius = circle.radius;
    let count = poly.count();
    for i in 0..count {
        let d = Vec2::new(lx - poly.vertices[i].x, ly - poly.vertices[i].y);
        let s = poly.normals[i].x * d.x + poly.normals[i].y * d.y;
        if s > radius {
            return;
        }
        if s > separation {
            separation = s;
            normal_index = i;
        }
    }
    let local_to_world = |p: &mut ManifoldPoint, nrm: Vec2, sep: f64| {
        let px = cx - radius * nrm.x;
        let py = cy - radius * nrm.y;
        let tx = px - xf1.position.x;
        let ty = py - xf1.position.y;
        p.local_point1 = Vec2::new(tx * xf1.r.col1.x + ty * xf1.r.col1.y, tx * xf1.r.col2.x + ty * xf1.r.col2.y);
        let tx = px - xf2.position.x;
        let ty = py - xf2.position.y;
        p.local_point2 = Vec2::new(tx * xf2.r.col1.x + ty * xf2.r.col1.y, tx * xf2.r.col2.x + ty * xf2.r.col2.y);
        p.separation = sep;
    };
    if separation < 5e-324 {
        m.point_count = 1;
        let n = poly.normals[normal_index];
        m.normal = Vec2::new(xf1.r.col1.x * n.x + xf1.r.col2.x * n.y, xf1.r.col1.y * n.x + xf1.r.col2.y * n.y);
        let nrm = m.normal;
        let p = &mut m.points[0];
        p.id = ContactId { reference_edge: 0, incident_edge: normal_index as u8, incident_vertex: NULL_FEATURE, flip: 0 };
        local_to_world(p, nrm, separation - radius);
        return;
    }
    let v1i = normal_index;
    let v2i = if v1i + 1 < count { v1i + 1 } else { 0 };
    let v1 = poly.vertices[v1i];
    let v2 = poly.vertices[v2i];
    let mut ex = v2.x - v1.x;
    let mut ey = v2.y - v1.y;
    let length = (ex * ex + ey * ey).sqrt();
    ex /= length;
    ey /= length;
    let dx = lx - v1.x;
    let dy = ly - v1.y;
    let u = dx * ex + dy * ey;
    let mut id = ContactId::default();
    let (qx, qy);
    if u <= 0.0 {
        qx = v1.x;
        qy = v1.y;
        id.incident_edge = NULL_FEATURE;
        id.incident_vertex = v1i as u8;
    } else if u >= length {
        qx = v2.x;
        qy = v2.y;
        id.incident_edge = NULL_FEATURE;
        id.incident_vertex = v2i as u8;
    } else {
        qx = ex * u + v1.x;
        qy = ey * u + v1.y;
        id.incident_edge = normal_index as u8;
        id.incident_vertex = 0;
    }
    let mut dx = lx - qx;
    let mut dy = ly - qy;
    let dist = (dx * dx + dy * dy).sqrt();
    dx /= dist;
    dy /= dist;
    if dist > radius {
        return;
    }
    m.point_count = 1;
    m.normal = Vec2::new(xf1.r.col1.x * dx + xf1.r.col2.x * dy, xf1.r.col1.y * dx + xf1.r.col2.y * dy);
    let nrm = m.normal;
    let p = &mut m.points[0];
    local_to_world(p, nrm, dist - radius);
    id.reference_edge = 0;
    id.flip = 0;
    p.id = id;
}

pub fn test_overlap(a: &Aabb, b: &Aabb) -> bool {
    let d1x = b.lower.x - a.upper.x;
    let d1y = b.lower.y - a.upper.y;
    let d2x = a.lower.x - b.upper.x;
    let d2y = a.lower.y - b.upper.y;
    !(d1x > 0.0 || d1y > 0.0 || d2x > 0.0 || d2y > 0.0)
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Aabb {
    pub lower: Vec2,
    pub upper: Vec2,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn boxp(hx: f64, hy: f64) -> PolygonShape {
        PolygonShape {
            vertices: vec![Vec2::new(-hx, -hy), Vec2::new(hx, -hy), Vec2::new(hx, hy), Vec2::new(-hx, hy)],
            normals: vec![Vec2::new(0.0, -1.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0), Vec2::new(-1.0, 0.0)],
            centroid: Vec2::default(),
        }
    }
    fn at(x: f64, y: f64) -> XForm {
        XForm { position: Vec2::new(x, y), r: Mat22::identity() }
    }

    #[test]
    fn stacked_boxes_make_two_points() {
        let mut m = Manifold::default();
        collide_polygons(&mut m, &boxp(1.0, 1.0), &at(0.0, 0.0), &boxp(1.0, 1.0), &at(0.0, -1.99));
        assert_eq!(m.point_count, 2);
        assert!((m.points[0].separation + 0.01).abs() < 1e-12, "{:?}", m.points[0].separation);
    }

    #[test]
    fn separated_boxes_have_no_contact() {
        let mut m = Manifold::default();
        collide_polygons(&mut m, &boxp(1.0, 1.0), &at(0.0, 0.0), &boxp(1.0, 1.0), &at(5.0, 0.0));
        assert_eq!(m.point_count, 0);
    }

    #[test]
    fn circle_on_box_and_circle_circle() {
        let mut m = Manifold::default();
        let c = CircleShape { local_position: Vec2::default(), radius: 0.5 };
        collide_polygon_and_circle(&mut m, &boxp(1.0, 1.0), &at(0.0, 0.0), &c, &at(0.0, -1.45));
        assert_eq!(m.point_count, 1);
        assert!((m.points[0].separation + 0.05).abs() < 1e-12);
        collide_circles(&mut m, &c, &at(0.0, 0.0), &c, &at(0.9, 0.0));
        assert_eq!(m.point_count, 1);
        assert!((m.points[0].separation + 0.1).abs() < 1e-12);
    }
}
