//! Compares the Rust port against the ORIGINAL Box2D (run through the oracle:
//! analysis/golden-box2d-scene.js → tests/golden/box2d-scene.json). Must match to ~machine precision.
use box2d::math::Vec2;
use box2d::shape::ShapeDef;
use box2d::world::{BodyDef, World};

fn scene() -> (World, Vec<usize>) {
    let mut w = World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true);
    let g = w.ground();
    for (hx, hy, c, a) in [(20.0, 1.0, Vec2::new(0.0, 5.0), 0.0), (4.0, 0.3, Vec2::new(8.0, 3.0), -0.35)] {
        let mut f = ShapeDef::oriented_box(hx, hy, c, a);
        f.friction = 0.6;
        f.restitution = 0.1;
        w.create_shape(g, &f);
    }
    let mut ids = Vec::new();
    let mut add = |w: &mut World, x: f64, y: f64, ang: f64, mut d: ShapeDef| {
        d.density = 1.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        let b = w.create_body(&BodyDef { position: Vec2::new(x, y), angle: ang, ..Default::default() });
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        ids.push(b);
    };
    add(&mut w, 0.0, 0.0, 0.0, ShapeDef::circle(0.5));
    add(&mut w, 3.0, 0.0, 0.3, ShapeDef::boxed(0.5, 0.5));
    add(&mut w, 6.0, 0.0, 0.0, ShapeDef::boxed(0.6, 0.4));
    add(&mut w, 6.05, -1.5, 0.1, ShapeDef::boxed(0.5, 0.5));
    add(&mut w, 8.0, -1.0, 0.0, ShapeDef::circle(0.4));
    add(&mut w, -4.0, 0.0, 0.8, ShapeDef::boxed(1.0, 0.25));
    (w, ids)
}

#[test]
fn scene_matches_original_box2d() {
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/box2d-scene.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root).unwrap()).unwrap();
    let (mut w, ids) = scene();
    let mut worst = vec![0.0f64; ids.len()];
    let mut first_bad: Option<(usize, usize, f64)> = None;
    for (n, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.step(1.0 / 30.0, 10);
        for (i, &b) in ids.iter().enumerate() {
            let g = &frame[i];
            let body = w.body(b);
            let e = (body.xf.position.x - g[0].as_f64().unwrap())
                .abs()
                .max((body.xf.position.y - g[1].as_f64().unwrap()).abs())
                .max((body.sweep.a - g[2].as_f64().unwrap()).abs());
            worst[i] = worst[i].max(e);
            if e > 1e-9 && first_bad.is_none() {
                first_bad = Some((n, i, e));
            }
        }
    }
    println!("per-body max error: {worst:?}; first divergence (frame, body, err): {first_bad:?}");
    assert!(first_bad.is_none(), "diverged from the original: {first_bad:?}");
}

#[test]
#[ignore]
fn trace_body(
) {
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/box2d-scene.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root).unwrap()).unwrap();
    let (mut w, ids) = scene();
    let which: usize = std::env::var("B").ok().and_then(|s| s.parse().ok()).unwrap_or(2);
    for (n, frame) in v["frames"].as_array().unwrap().iter().enumerate().take(30) {
        w.step(1.0 / 30.0, 10);
        let b = w.body(ids[which]);
        let g = &frame[which];
        if n == 22 || n == 23 {
            for c in w.debug_contacts(ids[which]) {
                println!("  contact after step {}: {c}", n + 1);
            }
        }
        if n >= 18 {
            println!("f{n}: ours ({:.6},{:.6},{:.6}) v=({:.4},{:.4}) | orig ({:.6},{:.6},{:.6})", b.xf.position.x, b.xf.position.y, b.sweep.a, b.linear_velocity.x, b.linear_velocity.y, g[0].as_f64().unwrap(), g[1].as_f64().unwrap(), g[2].as_f64().unwrap());
        }
    }
}

#[test]
#[ignore]
fn debug_collide_at_step_24() {
    use box2d::collision::*;
    use box2d::shape::ShapeKind;
    let (mut w, ids) = scene();
    for _ in 0..23 {
        w.step(1.0 / 30.0, 10);
    }
    let b2 = ids[2];
    let ground = w.ground();
    let sb = w.body(b2).shapes[0];
    let sg = w.body(ground).shapes[1];
    let (ShapeKind::Polygon { poly: pb, .. }, ShapeKind::Polygon { poly: pg, .. }) = (&w.c.shapes[sb].shape.kind, &w.c.shapes[sg].shape.kind) else { panic!() };
    let (xb, xg) = (w.body(b2).xf, w.body(ground).xf);
    println!("box xf {:?}\nramp verts {:?}", xb, pg.vertices);
    let mut m = Manifold::default();
    // contact shape order: which is shape1? try both
    collide_polygons(&mut m, pg, &xg, pb, &xb);
    println!("ramp,box -> {} points sep {:?}", m.point_count, m.points[0].separation);
    collide_polygons(&mut m, pb, &xb, pg, &xg);
    println!("box,ramp -> {} points", m.point_count);
}

fn ragdoll_scene() -> (World, Vec<usize>) {
    use box2d::joints::RevoluteJointDef;
    use box2d::shape::{Filter, Geometry};
    let guides: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../happywheels-core/data/character-guides.json")).unwrap(),
    )
    .unwrap();
    let g = &guides["c1"];
    let get = |n: &str| -> [f64; 5] {
        let a = g[n].as_array().unwrap();
        [0, 1, 2, 3, 4].map(|i| a[i].as_f64().unwrap())
    };
    const S: f64 = 62.5;
    const CS: f64 = 125.0;
    let mut w = World::new(Vec2::new(-2500.0 / S, -2500.0 / S), Vec2::new(3400.0 / S, 3000.0 / S), Vec2::new(0.0, 10.0), true);
    let ground = w.ground();
    let mut fd = ShapeDef::oriented_box(1000.0 / S, 25.0 / S, Vec2::new(0.0, 425.0 / S), 0.0);
    fd.friction = 1.0;
    fd.restitution = 0.1;
    fd.filter = Filter { category_bits: 8, mask_bits: 0xFFFF, group_index: 0 };
    w.create_shape(ground, &fd);
    let names = ["chestShape", "headShape", "pelvisShape", "upperArm1Shape", "upperArm2Shape", "upperLeg1Shape", "upperLeg2Shape", "lowerArm1Shape", "lowerArm2Shape", "lowerLeg1Shape", "lowerLeg2Shape"];
    let mut ids = Vec::new();
    for n in names {
        let [x, y, rot, sx, sy] = get(n);
        let b = w.create_body(&BodyDef { position: Vec2::new(x / CS, y / CS), angle: rot / (180.0 / std::f64::consts::PI), allow_sleep: false, ..Default::default() });
        let mut d = if n == "headShape" {
            ShapeDef::new(Geometry::Circle { local_position: Vec2::default(), radius: sx * 5.0 / CS })
        } else {
            ShapeDef::boxed(sx * 5.0 / CS, sy * 5.0 / CS)
        };
        d.density = 1.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter = Filter { category_bits: 260, mask_bits: 270, group_index: -1 };
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        ids.push(b);
    }
    let idx = |n: &str| ids[names.iter().position(|x| *x == n).unwrap()];
    let joints = [
        ("headAnchor", "chestShape", "headShape", -20.0, 20.0),
        ("pelvisAnchor", "chestShape", "pelvisShape", -5.0, 5.0),
        ("upperArmAnchor", "chestShape", "upperArm1Shape", -180.0, 60.0),
        ("upperArmAnchor", "chestShape", "upperArm2Shape", -180.0, 60.0),
        ("lowerArm1Anchor", "upperArm1Shape", "lowerArm1Shape", -160.0, 0.0),
        ("lowerArm2Anchor", "upperArm2Shape", "lowerArm2Shape", -160.0, 0.0),
        ("upperLegAnchor", "pelvisShape", "upperLeg1Shape", -150.0, 10.0),
        ("upperLegAnchor", "pelvisShape", "upperLeg2Shape", -150.0, 10.0),
        ("lowerLeg1Anchor", "upperLeg1Shape", "lowerLeg1Shape", 0.0, 150.0),
        ("lowerLeg2Anchor", "upperLeg2Shape", "lowerLeg2Shape", 0.0, 150.0),
    ];
    let deg = 180.0 / std::f64::consts::PI;
    for (anchor, p, c, lo, hi) in joints {
        let a = get(anchor);
        let (bp, bc) = (idx(p), idx(c));
        let mut d = RevoluteJointDef::initialize(w.body(bp), bp, w.body(bc), bc, Vec2::new(a[0] / CS, a[1] / CS));
        let rel = w.body(bc).sweep.a - w.body(bp).sweep.a;
        d.enable_limit = true;
        d.max_motor_torque = 4.0;
        d.lower_angle = lo / deg - rel;
        d.upper_angle = hi / deg - rel;
        w.create_revolute_joint(&d);
    }
    (w, ids)
}

#[test]
fn ragdoll_matches_original_box2d() {
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/ragdoll.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root).unwrap()).unwrap();
    let (mut w, ids) = ragdoll_scene();
    let mut worst = vec![0.0f64; ids.len()];
    let mut first: Option<(usize, usize, f64)> = None;
    for (n, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.step(1.0 / 30.0, 10);
        for (i, &b) in ids.iter().enumerate() {
            let g = &frame[i];
            let body = w.body(b);
            let e = (body.xf.position.x * 62.5 - g[0].as_f64().unwrap()).abs().max((body.xf.position.y * 62.5 - g[1].as_f64().unwrap()).abs());
            worst[i] = worst[i].max(e);
            if e > 1e-6 && first.is_none() {
                first = Some((n, i, e));
            }
        }
    }
    println!("ragdoll per-part max error (px): {worst:?}; first divergence: {first:?}");
    assert!(first.is_none(), "ragdoll diverged from the original: {first:?}");
}

/// A body with three shapes of different materials (the Van's body + tires welded into one) dropped on a
/// floor: verifies multi-shape mass, several simultaneous contacts and the resulting (genuinely asymmetric)
/// sequential-impulse response against the original.
#[test]
fn multi_shape_body_matches_original_box2d() {
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/multishape.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root).unwrap()).unwrap();
    let mut w = World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true);
    let ground = w.create_body(&BodyDef::default());
    let mut f = ShapeDef::oriented_box(32.0, 0.16, Vec2::new(8.0, 8.0), 0.0);
    f.friction = 1.0;
    f.restitution = 0.1;
    w.create_shape(ground, &f);
    let b = w.create_body(&BodyDef { position: Vec2::new(8.0, 3.2), ..Default::default() });
    let ps = 62.5;
    for (hw, hh, cx, cy, fr, re) in [(66.0, 58.0, 0.0, 0.0, 0.3, 0.1), (10.0, 22.5, -58.0, 48.0, 1.0, 0.3), (10.0, 22.5, 58.0, 48.0, 1.0, 0.3)] {
        let mut d = ShapeDef::oriented_box(hw / ps, hh / ps, Vec2::new(cx / ps, cy / ps), 0.0);
        d.density = 10.0;
        d.friction = fr;
        d.restitution = re;
        w.create_shape(b, &d);
    }
    w.set_mass_from_shapes(b);
    assert!((w.body(b).mass - v["mass"].as_f64().unwrap()).abs() < 1e-12);
    let mut worst = 0.0f64;
    for (n, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.step(1.0 / 30.0, 10);
        let body = w.body(b);
        let got = [body.xf.position.x, body.xf.position.y, body.sweep.a];
        for k in 0..3 {
            worst = worst.max((got[k] - frame[k].as_f64().unwrap()).abs());
        }
        assert!(worst < 1e-9, "diverged at frame {n}: worst {worst}");
    }
    println!("multi-shape worst error {worst:e}");
}

#[test]
fn destroying_a_shape_or_body_removes_contacts_and_frees_the_stack() {
    let mut w = World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true);
    let g = w.create_body(&BodyDef::default());
    let floor = w.create_shape(g, &ShapeDef::oriented_box(20.0, 0.5, Vec2::new(0.0, 5.0), 0.0));
    let mut ids = Vec::new();
    for k in 0..2 {
        let b = w.create_body(&BodyDef { position: Vec2::new(0.0, 4.0 - 1.1 * f64::from(k)), ..Default::default() });
        let mut d = ShapeDef::boxed(0.5, 0.5);
        d.density = 1.0;
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        ids.push(b);
    }
    for _ in 0..30 {
        w.step(1.0 / 30.0, 10);
    }
    assert!(w.body(ids[0]).xf.position.y > 3.9);
    // removing the floor shape lets both boxes fall; destroying the lower box lets the upper one fall onto nothing
    w.destroy_shape(floor);
    for _ in 0..30 {
        w.step(1.0 / 30.0, 10);
    }
    assert!(w.body(ids[0]).xf.position.y > 6.0, "box did not fall: {}", w.body(ids[0]).xf.position.y);
    w.destroy_body(ids[1]);
    assert!(!w.body(ids[1]).alive && w.body(ids[1]).shapes.is_empty());
    for _ in 0..5 {
        w.step(1.0 / 30.0, 10);
    }
}

#[test]
fn aabb_query_finds_shapes_in_the_box() {
    use box2d::collision::Aabb;
    let mut w = World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true);
    let mut near = Vec::new();
    for x in [0.0, 1.0, 20.0] {
        let b = w.create_body(&BodyDef { position: Vec2::new(x, 0.0), ..Default::default() });
        let mut d = ShapeDef::boxed(0.5, 0.5);
        d.density = 1.0;
        near.push(w.create_shape(b, &d));
    }
    let q = Aabb { lower: Vec2::new(-2.0, -2.0), upper: Vec2::new(2.0, 2.0) };
    let mut hit = w.query(&q, 30);
    hit.sort();
    assert_eq!(hit, vec![near[0], near[1]]);
    assert_eq!(w.query(&q, 1).len(), 1);
}

#[test]
fn distance_joints_match_original_box2d() {
    use box2d::joints::DistanceJointDef;
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/distance.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root).unwrap()).unwrap();
    let mut w = World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true);
    let ground = w.create_body(&BodyDef::default());
    let mut mk = |x: f64, y: f64| {
        let b = w.create_body(&BodyDef { position: Vec2::new(x, y), ..Default::default() });
        let mut d = ShapeDef::boxed(0.4, 0.3);
        d.density = 1.0;
        d.friction = 0.3;
        d.restitution = 0.1;
        d.filter.group_index = -5;
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        b
    };
    let (a, b, c) = (mk(3.0, 0.0), mk(8.0, 0.0), mk(8.0, 3.0));
    for (b1, b2, p1, p2, hz, dr) in [(ground, a, (0.0, 0.0), (3.0, 0.0), 0.0, 0.0), (ground, b, (8.0, -2.0), (8.0, 0.0), 2.0, 0.3), (b, c, (8.0, 0.0), (8.0, 3.0), 0.0, 0.0)] {
        let mut d = DistanceJointDef::initialize(w.body(b1), b1, w.body(b2), b2, Vec2::new(p1.0, p1.1), Vec2::new(p2.0, p2.1));
        d.frequency_hz = hz;
        d.damping_ratio = dr;
        w.create_distance_joint(&d);
    }
    let mut worst = 0.0f64;
    for frame in v["frames"].as_array().unwrap() {
        w.step(1.0 / 30.0, 10);
        for (i, id) in [a, b, c].iter().enumerate() {
            let body = w.body(*id);
            for (k, got) in [body.xf.position.x, body.xf.position.y, body.sweep.a].iter().enumerate() {
                worst = worst.max((got - frame[i][k].as_f64().unwrap()).abs());
            }
        }
    }
    println!("distance worst {worst:e}");
    assert!(worst < 1e-9, "distance joints diverge: {worst}");
}

#[test]
fn gear_joint_matches_original_box2d() {
    use box2d::joints::RevoluteJointDef;
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/gear.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root).unwrap()).unwrap();
    let mut w = World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true);
    let frame = w.create_body(&BodyDef { position: Vec2::new(5.0, 0.0), ..Default::default() });
    let mut d = ShapeDef::boxed(1.0, 0.3);
    d.density = 2.0;
    d.friction = 0.3;
    d.restitution = 0.1;
    d.filter.group_index = -5;
    w.create_shape(frame, &d);
    w.set_mass_from_shapes(frame);
    let mut circle = |x: f64, y: f64, r: f64, den: f64| {
        let b = w.create_body(&BodyDef { position: Vec2::new(x, y), ..Default::default() });
        let mut d = ShapeDef::circle(r);
        d.density = den;
        d.friction = 1.0;
        d.restitution = 0.3;
        d.filter.group_index = -5;
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        b
    };
    let (a, g) = (circle(4.0, 0.6, 0.5, 5.0), circle(6.2, 0.4, 0.3, 2.0));
    let mut rj = |b2, x: f64, y: f64, motor: bool| {
        let mut d = RevoluteJointDef::initialize(w.body(frame), frame, w.body(b2), b2, Vec2::new(x, y));
        d.max_motor_torque = 20.0;
        if motor {
            d.enable_motor = true;
            d.motor_speed = 4.0;
        }
        w.create_revolute_joint(&d)
    };
    let (ja, jg) = (rj(a, 4.0, 0.6, true), rj(g, 6.2, 0.4, false));
    let gj = w.create_gear_joint(ja, jg, 1.0).unwrap();
    w.flip_gear_ratio(gj);
    let mut worst = 0.0f64;
    for frame_data in v["frames"].as_array().unwrap() {
        w.step(1.0 / 30.0, 10);
        for (i, id) in [frame, a, g].iter().enumerate() {
            let body = w.body(*id);
            for (k, got) in [body.xf.position.x, body.xf.position.y, body.sweep.a].iter().enumerate() {
                worst = worst.max((got - frame_data[i][k].as_f64().unwrap()).abs());
            }
        }
    }
    println!("gear worst {worst:e}");
    assert!(worst < 1e-9, "gear joint diverges: {worst}");
}

/// Rope (max-length) joint, type 7: a static anchor → ball → ball chain flung by an impulse.
#[test]
fn rope_joint_matches_original_box2d() {
    use box2d::joints::DistanceJointDef;
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/rope.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root).unwrap()).unwrap();
    let mut w = World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true);
    let anchor = w.create_body(&BodyDef::default());
    let mut mk = |x: f64, y: f64, r: f64| {
        let b = w.create_body(&BodyDef { position: Vec2::new(x, y), ..Default::default() });
        let mut d = ShapeDef::circle(r);
        d.density = 3.0;
        d.friction = 0.5;
        d.restitution = 0.2;
        d.filter.group_index = -5;
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        b
    };
    let (a, c) = (mk(6.5, -0.5, 0.4), mk(8.0, 0.0, 0.5));
    let mut rope = |b1, b2, p1: Vec2, p2: Vec2, len: f64| {
        let mut d = DistanceJointDef::initialize(w.body(b1), b1, w.body(b2), b2, p1, p2);
        d.length = len;
        d.collide_connected = true;
        w.create_rope_joint(&d);
    };
    rope(anchor, a, Vec2::new(5.0, -3.0), Vec2::new(6.5, -0.5), 3.0);
    rope(a, c, Vec2::new(6.5, -0.5), Vec2::new(8.0, 0.0), 2.0);
    let center = w.body(c).sweep.c;
    w.body_mut(c).apply_impulse(Vec2::new(6.0, -4.0), center);
    let mut worst = 0.0f64;
    for frame_data in v["frames"].as_array().unwrap() {
        w.step(1.0 / 30.0, 10);
        for (i, id) in [a, c].iter().enumerate() {
            let body = w.body(*id);
            for (k, got) in [body.xf.position.x, body.xf.position.y, body.sweep.a].iter().enumerate() {
                worst = worst.max((got - frame_data[i][k].as_f64().unwrap()).abs());
            }
        }
    }
    println!("rope worst {worst:e}");
    assert!(worst < 1e-9, "rope joint diverges: {worst}");
}

/// Post-solve `Result` events (per-contact-point normal impulses): same count, order, impulses and positions
/// as the original's contact listener on a small pile.
#[test]
fn contact_results_match_original_box2d() {
    use box2d::world::ContactEventKind;
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/result.json");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root).unwrap()).unwrap();
    let mut w = World::new(Vec2::new(-40.0, -40.0), Vec2::new(60.0, 50.0), Vec2::new(0.0, 10.0), true);
    let ground = w.ground();
    let mut gd = ShapeDef::oriented_box(20.0, 1.0, Vec2::new(0.0, 5.0), 0.0);
    gd.friction = 0.6;
    w.create_shape(ground, &gd);
    let mut ids = Vec::new();
    let mut add = |w: &mut World, x: f64, y: f64, a: f64, circle: bool| {
        let b = w.create_body(&BodyDef { position: Vec2::new(x, y), angle: a, ..Default::default() });
        let mut d = if circle { ShapeDef::circle(0.5) } else { ShapeDef::boxed(0.6, 0.4) };
        d.density = if circle { 1.0 } else { 2.0 };
        d.friction = 0.5;
        d.restitution = if circle { 0.4 } else { 0.1 };
        w.create_shape(b, &d);
        w.set_mass_from_shapes(b);
        ids.push(b);
    };
    add(&mut w, 0.0, 0.0, 0.3, false);
    add(&mut w, 0.5, -2.0, 0.0, false);
    add(&mut w, -0.3, -4.0, 0.0, true);
    add(&mut w, 3.0, -1.0, 1.0, false);
    w.c.report_results = true;
    let index = |w: &World, s: usize| -> i64 { ids.iter().position(|&b| b == w.c.shapes[s].body).map_or(-1, |i| i as i64) };
    let mut worst = 0.0f64;
    for (n, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.c.events.clear();
        w.step(1.0 / 30.0, 10);
        let got: Vec<_> = w.c.events.iter().filter(|e| e.kind == ContactEventKind::Result).collect();
        let want = frame.as_array().unwrap();
        assert_eq!(got.len(), want.len(), "frame {n}: result count");
        for (g, e) in got.iter().zip(want) {
            assert_eq!(index(&w, g.shape1), e[0].as_i64().unwrap(), "frame {n}: shape1");
            assert_eq!(index(&w, g.shape2), e[1].as_i64().unwrap(), "frame {n}: shape2");
            worst = worst.max((g.normal_impulse - e[2].as_f64().unwrap()).abs());
            worst = worst.max((g.position.x - e[3].as_f64().unwrap()).abs());
            worst = worst.max((g.position.y - e[4].as_f64().unwrap()).abs());
        }
    }
    assert!(worst < 1e-9, "contact results diverge: {worst}");
}
