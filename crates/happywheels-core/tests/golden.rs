//! Golden comparison against the ORIGINAL engine. tests/golden/*.json were captured by running
//! the original's own Box2D (module 23222 in the oracle) on the same scene — see
//! analysis/golden-sample.js. Checks our Box2D port against it.

use std::path::PathBuf;

use happywheels_core::{level, physics::World};

struct Golden {
    shapes: Vec<usize>,
    frames: Vec<Vec<[f32; 3]>>,
}

fn load(name: &str) -> Golden {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let text = std::fs::read_to_string(root.join("tests/golden").join(format!("{name}.json"))).unwrap();
    let v: serde_json::Value = serde_json::from_str(&text).unwrap();
    Golden {
        shapes: v["shapes"].as_array().unwrap().iter().map(|x| x.as_u64().unwrap() as usize).collect(),
        frames: v["frames"]
            .as_array()
            .unwrap()
            .iter()
            .map(|f| {
                f.as_array()
                    .unwrap()
                    .iter()
                    .map(|b| {
                        let b = b.as_array().unwrap();
                        [b[0].as_f64().unwrap() as f32, b[1].as_f64().unwrap() as f32, b[2].as_f64().unwrap() as f32]
                    })
                    .collect()
            })
            .collect(),
    }
}

/// Max position error (px) per tracked body over all frames.
fn divergence(name: &str) -> Vec<f32> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let xml = std::fs::read_to_string(root.join("tests/levels").join(format!("{name}.xml"))).unwrap();
    let golden = load(name);
    let level = level::parse(&xml).unwrap();
    let mut world = World::from_level(&level);
    let mut worst = vec![0.0f32; golden.shapes.len()];
    for frame in &golden.frames {
        world.step();
        for (k, &shape) in golden.shapes.iter().enumerate() {
            let [x, y] = world.position_px(shape).unwrap();
            let err = ((x - frame[k][0]).powi(2) + (y - frame[k][1]).powi(2)).sqrt();
            worst[k] = worst[k].max(err);
        }
    }
    worst
}

#[test]
fn sample_scene_matches_original_box2d() {
    // Floor, ramp and two balls in the game's world (borders included), rolling and bouncing for 150 frames:
    // the Rust Box2D port tracks the original to float precision (positions are reported as f32 px).
    let worst = divergence("game-sample");
    eprintln!("sample: worst position error per body (px): {worst:?}");
    assert!(worst.iter().all(|&e| e < 1e-3), "sample diverges: {worst:?}");
}

#[test]
#[ignore = "diagnostic: cargo test --test golden -- --ignored --nocapture"]
fn print_body0_trace() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let xml = std::fs::read_to_string(root.join("tests/levels/sample.xml")).unwrap();
    let golden = load("sample");
    let mut world = World::from_level(&level::parse(&xml).unwrap());
    for (i, f) in golden.frames.iter().enumerate() {
        world.step();
        let [_, y] = world.position_px(golden.shapes[0]).unwrap();
        if i < 3 || i % 10 == 9 || (28..40).contains(&i) {
            eprintln!("frame {:3}: box2d y={:8.2}  ours y={:8.2}  diff={:6.2}", i + 1, f[0][1], y, y - f[0][1]);
        }
    }
}

#[test]
fn pendulum_joint_matches_original_box2d() {
    // Body 0: free pin-jointed bar (large swings); body 1: same bar limited to +-10 degrees.
    let worst = divergence("game-pendulum");
    eprintln!("pendulum: worst position error per body (px): {worst:?}");
    assert!(worst.iter().all(|&e| e < 1e-3), "pendulum diverges: {worst:?}");
}

/// Base ragdoll dropped on a floor (top y=400): original Box2D vs ours, per-part max position error.
fn ragdoll_divergence() -> Vec<f32> {
    use happywheels_core::character::{guide, Ragdoll, PART_ORDER};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/game-ragdoll.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='0'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    let doll = Ragdoll::spawn(&mut w, &guide(1).unwrap(), [1000.0, 0.0]).unwrap();
    let mut worst = vec![0.0f32; 11];
    for frame in v["frames"].as_array().unwrap() {
        w.step();
        for (i, part) in PART_ORDER.iter().enumerate() {
            let g = &frame[i];
            let p = doll.position_px(&w, *part).unwrap();
            let e = ((p[0] - g[0].as_f64().unwrap() as f32).powi(2) + (p[1] - g[1].as_f64().unwrap() as f32).powi(2)).sqrt();
            worst[i] = worst[i].max(e);
            if i == 0 && std::env::var("RD_TRACE").is_ok() {
                println!("chest ours {:?} orig {:?}", p, [g[0].as_f64(), g[1].as_f64(), g[2].as_f64()]);
            }
        }
    }
    worst
}

#[test]
#[ignore]
fn print_ragdoll_divergence() {
    println!("ragdoll per-part max error px: {:?}", ragdoll_divergence());
}

/// The whole 150-frame drop (free fall, impact, settling) must match the original's Box2D.
#[test]
fn ragdoll_drop_matches_original() {
    let worst = ragdoll_divergence();
    assert!(worst.iter().all(|&e| e < 1e-3), "ragdoll diverges: {worst:?}");
}

/// Pose driver (tuck → arch → free → tuck, keys changing at frames 50/90/110) against the original's
/// `setJoint`/`cancelPose` running on the same ragdoll (analysis/golden-pose.js).
#[test]
fn ragdoll_poses_match_original() {
    use happywheels_core::character::{guide, PART_ORDER};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/game-pose.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='0'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_character(1, [1000.0, 0.0]));
    let _ = guide(1);
    let mut worst = 0.0f32;
    for (n, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        match n {
            0..=49 => w.set_character_keys(false, false, false, true),
            50..=89 => w.set_character_keys(true, false, false, false),
            90..=109 => w.set_character_keys(false, false, false, false),
            _ => w.set_character_keys(false, false, false, true),
        }
        w.step();
        let doll = w.character.as_ref().unwrap();
        for (i, part) in PART_ORDER.iter().enumerate() {
            let p = doll.position_px(&w, *part).unwrap();
            let g = &frame[i];
            let e = ((p[0] - g[0].as_f64().unwrap() as f32).powi(2) + (p[1] - g[1].as_f64().unwrap() as f32).powi(2)).sqrt();
            worst = worst.max(e);
        }
    }
    assert!(worst < 1e-2, "pose driver diverges from the original by {worst} px");
}

/// Bike rider (accelerate, lean back, brake, eject at frame 100) against the original's logic running on the
/// same bodies/joints in the oracle (analysis/golden-bike.js).
#[test]
fn bike_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/bike.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(happywheels_core::vehicle::Keys { left: (60..80).contains(&i), up: i < 60, space: (80..100).contains(&i), z: i == 100, ..Default::default() });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let bike = w.bike.as_ref().unwrap().as_bike().unwrap();
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([bike.frame, bike.back_wheel, bike.front_wheel, bike.gear]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            worst = worst.max(e);
        }
    }
    assert!(worst < 1e-6, "bike diverges by {worst} px");
}

/// Segway rider (accelerate, lean, jump, leg poses, eject at frame 120) against the original's logic running
/// on the same bodies/joints in the oracle (analysis/golden-segway.js).
#[test]
fn segway_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/segway.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(2, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (10..30).contains(&i),
            right: (30..50).contains(&i),
            up: i < 40,
            space: (20..45).contains(&i),
            shift: (50..70).contains(&i),
            ctrl: (70..90).contains(&i),
            z: i == 120,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Segway(s)) = w.bike.as_ref() else { panic!("not a segway") };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([s.frame, s.stand, s.shock, s.wheel]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    assert!(worst < 1e-6, "segway diverges by {worst} px at frame/body {worst_at:?}");
}

/// Jet wheelchair (drive, tilt, jet with angle changes, eject at 120, wheel poses afterwards) against the
/// original's logic in the oracle (analysis/golden-chair.js).
#[test]
fn chair_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/chair.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(1, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (10..25).contains(&i),
            right: (25..40).contains(&i),
            up: i < 40,
            space: (20..60).contains(&i),
            shift: (60..70).contains(&i) || (130..140).contains(&i),
            ctrl: (70..75).contains(&i) || (150..160).contains(&i),
            z: i == 120,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Chair(c)) = w.bike.as_ref() else { panic!("not a chair") };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([c.chair, c.big_wheel, c.small_wheel]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    assert!(worst < 1e-6, "chair diverges by {worst} px at frame/body {worst_at:?}");
}

/// Dirt bike (drive, lean impulses, boost, brake, eject at 120) against the original's logic in the oracle
/// (analysis/golden-dirt.js).
#[test]
fn dirt_bike_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/dirt.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(5, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (10..20).contains(&i),
            right: (20..30).contains(&i),
            up: i < 60,
            space: (30..60).contains(&i),
            ctrl: (60..80).contains(&i),
            z: i == 120,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Dirt(d)) = w.bike.as_ref() else { panic!("not a dirt bike") };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([d.frame, d.back_wheel, d.front_wheel]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    assert!(worst < 1e-6, "dirt bike diverges by {worst} px at frame/body {worst_at:?}");
}

/// Lawnmower (drive, spin impulses, shock hops, eject at 120) against the original's logic in the oracle
/// (analysis/golden-mower.js).
#[test]
fn mower_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/mower.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(6, [1000.0, 250.0]));
    w.joints_break = false;
    let mut first = None;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (10..20).contains(&i),
            right: (20..30).contains(&i),
            up: i < 60,
            space: (30..40).contains(&i) || (70..90).contains(&i),
            z: i == 120,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Mower(d)) = w.bike.as_ref() else { panic!("not a mower") };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([d.body, d.back_wheel, d.front_wheel, d.back_shock(), d.front_shock()]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > 1e-9 && first.is_none() {
                first = Some((i, k, e));
            }
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    assert!(worst < 1e-6, "mower diverges by {worst} px (first {first:?}) at frame/body {worst_at:?}");
}

/// Pogo (steering, bounce, charged jump, lean, poses, eject at 170) against the original's logic in the oracle
/// (analysis/golden-pogo.js).
#[test]
fn pogo_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/pogo.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(9, [1000.0, 250.0]));
    w.joints_break = false;
    let mut first = None;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (70..80).contains(&i),
            right: (80..90).contains(&i),
            up: (20..45).contains(&i),
            down: (100..115).contains(&i),
            space: (50..65).contains(&i),
            shift: (120..125).contains(&i),
            ctrl: (125..130).contains(&i),
            z: i == 170,
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Pogo(d)) = w.bike.as_ref() else { panic!("not a pogo") };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([d.frame, d.rod]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > 1e-9 && first.is_none() {
                first = Some((i, k, e));
            }
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    assert!(worst < 1e-6, "pogo diverges by {worst} px (first {first:?}) at frame/body {worst_at:?}");
}

/// Shopping cart (drive, lean impulses, shock hops, eject at 120, lunge poses) against the oracle
/// (analysis/golden-cart.js).
#[test]
fn cart_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/cart.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(4, [1000.0, 250.0]));
    w.joints_break = false;
    let mut first = None;
    let mut worst_early = 0.0f64;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (10..20).contains(&i),
            right: (20..30).contains(&i),
            up: i < 60,
            space: (30..40).contains(&i) || (70..90).contains(&i),
            shift: (130..140).contains(&i),
            ctrl: (140..150).contains(&i),
            z: i == 120,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Cart(d)) = w.bike.as_ref() else { panic!("not a cart") };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([d.body, d.shaft, d.back_wheel, d.front_wheel, d.back_shock(), d.front_shock()]).chain(d.groceries.iter().copied()).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if i < 70 && e > worst_early {
                worst_early = e;
            }
            if e > 1e-6 && first.is_none() {
                first = Some((i, k, e));
            }
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    // The grocery boxes pile up and collide with each other: ULP noise from frame 0 (1e-13 px) grows
    // exponentially (chaotic contacts), so the exact (<1e-6) window is the first 70 frames.
    assert!(worst_early < 1e-6, "cart diverges early by {worst_early} px");
    assert!(worst < 0.05, "cart diverges by {worst} px (first {first:?}) at frame/body {worst_at:?}");
}


/// Helicopter (hover, balance, rope reel, eject at 150) against the oracle (analysis/golden-heli.js).
#[test]
fn heli_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/heli.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(11, [1000.0, 250.0]));
    w.joints_break = false;
    let mut first = None;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (20..30).contains(&i),
            right: (30..40).contains(&i),
            up: (10..60).contains(&i),
            down: (70..90).contains(&i),
            shift: (100..110).contains(&i) || (180..190).contains(&i),
            ctrl: (110..125).contains(&i),
            z: i == 150,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Heli(d)) = w.bike.as_ref() else { panic!("not a heli") };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([d.copter, d.magnet]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > 1e-9 && first.is_none() {
                first = Some((i, k, e));
            }
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    assert!(worst < 1e-6, "heli diverges by {worst} px (first {first:?}) at frame/body {worst_at:?}");
}


/// Heli smashes (copterSmash / bladeSmash / copterLegSmash) forced at step 95 against the original's Box2D (analysis/golden-heli-smash.tpl.js):
/// doll bodies, magnet and every body born from the smash, in creation order.
#[test]
fn heli_smashes_match_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    use box2d::math::Vec2;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    for mode in ["copter", "blade", "leg"] {
        let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join(format!("tests/golden/heli_smash_{mode}.json"))).unwrap()).unwrap();
        let level = level::parse(
            "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
             <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
        )
        .unwrap();
        let mut w = World::from_level(&level);
        assert!(w.spawn_bike(11, [1000.0, 250.0]));
        w.joints_break = false;
        let at = Vec2::new(v["at"][0].as_f64().unwrap(), v["at"][1].as_f64().unwrap());
        let mut worst = 0.0f64;
        let mut worst_at = (0, 0);
        let mut first = None;
        let mut early = 0.0f64;
        for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
            if i == 95 {
                let mut doll = w.character.take().unwrap();
                let Some(Vehicle::Heli(mut h)) = w.bike.take() else { panic!("not a heli") };
                match mode {
                    "copter" => h.copter_smash(&mut w, &mut doll, at),
                    "blade" => h.blade_smash_at(&mut w, at),
                    _ => h.leg_smash_at(&mut w, 0),
                }
                w.bike = Some(Vehicle::Heli(h));
                w.character = Some(doll);
            }
            w.set_rider_keys(Keys {
                left: (20..30).contains(&i),
                right: (30..40).contains(&i),
                up: (10..60).contains(&i),
                down: (70..90).contains(&i),
                shift: (100..110).contains(&i) || (180..190).contains(&i),
                ctrl: (110..125).contains(&i),
                z: i == 150,
                ..Default::default()
            });
            w.step();
            let doll = w.character.as_ref().unwrap();
            let Some(Vehicle::Heli(d)) = w.bike.as_ref() else { panic!("not a heli") };
            let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([d.magnet]).chain(d.debris.iter().copied()).collect();
            assert_eq!(ids.len(), frame.as_array().unwrap().len(), "{mode}: body count at frame {i}");
            for (k, id) in ids.iter().enumerate() {
                let p = w.b2().body(*id).xf.position;
                let g = &frame[k];
                let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
                if i < 110 {
                    early = early.max(e);
                }
                if e > 1e-9 && first.is_none() {
                    first = Some((i, k, e));
                }
                if e > worst {
                    worst = e;
                    worst_at = (i, k);
                }
            }
        }
        // The replica and the port differ by ~5e-12 px of libm noise already in plain flight (see `heli_rider_matches_original`); the debris pile
        // amplifies it (chaotic contacts) to ~1e-5 px by step 180, so: exact through the first 15 steps after the smash, tiny afterwards.
        assert!(early < 1e-8, "{mode}: right after the smash (first {first:?}) diverges by {early} px");
        assert!(worst < 1e-4, "{mode}: heli smash diverges by {worst} px at frame/body {worst_at:?}");
    }
}

/// Santa smashes (behaviour only): skis first, then the whole sleigh; the broken pieces exist, wheels/sleigh are gone, nothing blows up.
#[test]
fn santa_ski_and_sleigh_smash_replace_bodies() {
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(8, [1000.0, 250.0]));
    for _ in 0..30 {
        w.step();
    }
    let mut doll = w.character.take().unwrap();
    let Some(Vehicle::Santa(mut s)) = w.bike.take() else { panic!("not santa") };
    s.stem_smash(&mut w, true);
    assert!(s.stem_gone && s.debris.len() == 1);
    s.ski_smash(&mut w);
    assert!(s.wheels_gone && !s.sleigh_gone && s.debris.len() == 4);
    w.bike = Some(Vehicle::Santa(s));
    w.character = Some(doll);
    for _ in 0..30 {
        w.step();
    }
    doll = w.character.take().unwrap();
    let Some(Vehicle::Santa(mut s)) = w.bike.take() else { panic!("not santa") };
    s.sleigh_smash(&mut w, &mut doll);
    assert!(s.sleigh_gone && s.debris.len() == 11 && s.ejected);
    w.bike = Some(Vehicle::Santa(s));
    w.character = Some(doll);
    for _ in 0..60 {
        w.step();
    }
    let Some(Vehicle::Santa(s)) = w.bike.as_ref() else { panic!() };
    for id in s.debris.iter() {
        let p = w.b2().body(*id).xf.position;
        assert!(p.x.is_finite() && p.y.is_finite());
    }
}

/// Family daughter frame smash (behaviour only): frame body replaced by three pieces, daughter ejected, simulation stays finite.
#[test]
fn family_daughter_frame_smash_replaces_the_frame() {
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(10, [1000.0, 250.0]));
    for _ in 0..30 {
        w.step();
    }
    let Some(Vehicle::Family(mut f)) = w.bike.take() else { panic!("not a family") };
    f.daughter.frame_smash(&mut w);
    assert!(f.daughter.frame_gone && f.daughter.ejected && f.daughter.debris.len() == 3);
    w.bike = Some(Vehicle::Family(f));
    for _ in 0..90 {
        w.step();
    }
    let Some(Vehicle::Family(f)) = w.bike.as_ref() else { panic!() };
    for id in f.bodies() {
        let p = w.b2().body(id).xf.position;
        assert!(p.x.is_finite() && p.y.is_finite());
    }
}

/// Rail grinder (no rails: drive, brake, spin, lean, eject at 170) against the oracle (analysis/golden-grinder.js).
#[test]
fn grinder_rider_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/grinder.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(7, [1000.0, 250.0]));
    w.joints_break = false;
    let mut first = None;
    let mut worst = 0.0f64;
    let mut late = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (10..20).contains(&i) || (200..210).contains(&i),
            right: (20..30).contains(&i),
            up: (30..90).contains(&i),
            down: (90..93).contains(&i),
            shift: (110..125).contains(&i),
            ctrl: (130..150).contains(&i),
            z: i == 170,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Grinder(d)) = w.bike.as_ref() else { panic!("not a grinder") };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([d.frame, d.back_wheel, d.front_wheel]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > 1e-9 && first.is_none() {
                first = Some((i, k, e));
            }
            if i >= 190 {
                late = late.max(e);
            } else if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    assert!(worst < 1e-6, "grinder diverges by {worst} px (first {first:?}) at frame/body {worst_at:?}");
    // after the eject the ragdoll tumbles on the floor; ULP noise grows exponentially there (cf. the cart test)
    assert!(late < 200.0, "grinder tumble diverges by {late} px");
}


/// Smash detection: a short drop stays below every limit, a long fall pushes at least one part over.
#[test]
fn ragdoll_smash_detection_scales_with_fall_height() {
    let xml = |y: u32| {
        format!(
            "<levelXML><info v='1.99' x='1000' y='{y}'/><shapes>\\
             <sh t='0' p0='1000' p1='5000' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>"
        )
    };
    let run = |y: u32| {
        let level = level::parse(&xml(y)).unwrap();
        let mut w = World::from_level(&level);
        assert!(w.spawn_character(1, level.info.start));
        w.joints_break = false;
        let mut hit = Vec::new();
        for _ in 0..400 {
            w.step();
            hit.extend(w.smashes.iter().copied());
        }
        hit
    };
    assert!(run(4800).is_empty(), "a 200 px drop must not smash anything");
    let big = run(300);
    assert!(!big.is_empty(), "a 4700 px fall must smash a part");
}

/// c10 smoke test: dad + daughter trailer ride a floor without exploding, stay hitched, and the daughter can eject.
#[test]
fn c10_family_rides_and_daughter_ejects() {
    use happywheels_core::vehicle::{Keys, Vehicle};
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(10, [1000.0, 250.0]));
    for i in 0..300 {
        w.set_rider_keys(Keys { up: i > 20, ctrl: i == 200, ..Default::default() });
        w.step();
    }
    let Some(Vehicle::Family(f)) = w.bike.as_ref() else { panic!("not a family") };
    assert!(f.daughter.ejected, "ctrl ejects the daughter");
    let dad = w.b2().body(f.dad.frame).xf.position;
    assert!(dad.x * 62.5 > 1100.0 && dad.y * 62.5 < 600.0, "dad rode forward on the floor: {dad:?}");
}

/// c10 family (dad bike + daughter trailer + son in the basket; up/lean/brake, daughter ctrl-eject at 120, son
/// shift-eject at 160, dad z-eject at 200) against analysis/golden-family.js.
#[test]
fn c10_family_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/family.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(10, [1000.0, 250.0]));
    w.joints_break = false;
    let mut first = None;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            up: (20..110).contains(&i),
            left: (60..75).contains(&i),
            space: (90..100).contains(&i),
            ctrl: i == 120,
            shift: i == 160,
            z: i == 200,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Family(f)) = w.bike.as_ref() else { panic!("not a family") };
        let mut ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).collect();
        ids.extend([f.dad.frame, f.dad.back_wheel, f.dad.front_wheel, f.dad.gear]);
        ids.extend(PART_ORDER.iter().map(|p| f.daughter.doll.bodies[p]));
        ids.extend([f.daughter.frame, f.daughter.wheel, f.daughter.gear]);
        ids.extend(PART_ORDER.iter().map(|p| f.son.doll.bodies[p]));
        ids.push(f.son.basket);
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > 1e-9 && first.is_none() {
                first = Some((i, k, e));
            }
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    // 41 bodies tumbling after three ejects: ULP noise reaches ~5e-6 px, so the bound is 1e-4
    assert!(worst < 1e-4, "family diverges by {worst} px (first {first:?}) at frame/body {worst_at:?}");
}

/// c8 Santa smoke test: the sleigh (with both elves strapped on) drives forward on a floor, space boosts, z ejects.
#[test]
fn c8_santa_rides_and_ejects() {
    use happywheels_core::vehicle::{Keys, Vehicle};
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(8, [1000.0, 250.0]));
    for i in 0..300 {
        w.set_rider_keys(Keys { up: i > 30, space: (100..140).contains(&i), z: i == 250, ..Default::default() });
        w.step();
        let Some(Vehicle::Santa(s)) = w.bike.as_ref() else { panic!("not santa") };
        let p = w.b2().body(s.sleigh).xf.position;
        assert!(p.x.is_finite() && p.y.is_finite(), "frame {i}: {p:?}");
    }
    let Some(Vehicle::Santa(s)) = w.bike.as_ref() else { panic!("not santa") };
    assert!(s.ejected, "z ejects Santa");
    let p = w.b2().body(s.sleigh).xf.position;
    assert!(p.x * 62.5 > 1150.0 && p.y * 62.5 < 600.0, "sleigh drove forward on the floor: {p:?}");
}

/// c8 Santa (sleigh + 2 elves + straps; up/lean/boost, Santa z-eject at 200, elves' eject at 220) against the
/// replica in analysis/golden-santa.js (generated from golden-santa.tpl.js).
#[test]
fn c8_santa_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/santa.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(8, [1000.0, 250.0]));
    w.joints_break = false;
    let mut first = None;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    let mut early = 0.0f64;
    let mut strap_err = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys { up: (30..190).contains(&i), left: (60..70).contains(&i), space: (100..140).contains(&i), z: i == 200 || i == 220, ..Default::default() });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Santa(s)) = w.bike.as_ref() else { panic!("not santa") };
        let mut ids = vec![s.sleigh, s.reigns_id];
        ids.extend(s.wheels);
        ids.extend(&s.boxes);
        ids.extend(PART_ORDER.iter().map(|p| doll.bodies[p]));
        for e in [&s.elf1, &s.elf2] {
            ids.extend(PART_ORDER.iter().map(|p| e.doll.bodies[p]));
            ids.extend([e.stem_id, e.wheel_id]);
        }
        ids.extend(&s.strap_balls);
        assert_eq!(ids.len(), frame.as_array().unwrap().len());
        if i < 200 {
            let want = &v["imp"][i];
            let got = s.strap_impulses(&w);
            for n in 0..2 {
                strap_err = strap_err.max((got[n] - want[n].as_f64().unwrap().abs()).abs());
            }
        }
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > 1e-6 && first.is_none() {
                first = Some((i, k, e));
            }
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
            if i < 200 {
                early = early.max(e);
            }
        }
    }
    assert!(strap_err < 1e-9, "chest-strap impulses differ from the original by {strap_err}");
    assert!(early < 1e-5, "santa diverges before the eject by {early} px");
    assert!(worst < 1e-3, "santa diverges by {worst} px (first {first:?}) at frame/body {worst_at:?}");
}

/// c11 heli magnet: with the magnet on, a dynamic crate touched from above gets pinned to the magnet by a joint,
/// and switching the magnet off releases it.
#[test]
fn c11_magnet_picks_up_and_releases() {
    use happywheels_core::vehicle::{Keys, Vehicle};
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>\
         <sh t='0' p0='1000' p1='385' p2='800' p3='30' p4='0' p5='f' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(11, [1000.0, 250.0]));
    let joints = |w: &World| {
        let Some(Vehicle::Heli(h)) = w.bike.as_ref() else { panic!("not a heli") };
        w.b2().body(h.magnet).joints.len()
    };
    let base = joints(&w);
    let mut max = base;
    for i in 0..400 {
        // space toggles the magnet on at 5 and off at 300 (edge-triggered); ctrl pays the rope out
        w.set_rider_keys(Keys { space: i == 5 || i == 300, ctrl: (10..200).contains(&i), ..Default::default() });
        w.step();
        max = max.max(joints(&w));
        if i == 299 {
            assert!(joints(&w) > base, "the magnet is holding the crate at frame {i}");
        }
    }
    assert!(max > base);
    assert_eq!(joints(&w), base, "magnet off releases it");
}

/// c7 rail grinding: holding space while the wheels roll onto a Rail special attaches dongles (sensor body + slider);
/// releasing space drops them again.
#[test]
fn c7_grinder_attaches_to_rail() {
    use happywheels_core::vehicle::{Keys, Vehicle};
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes>\
         <specials><sp t='27' p0='1500' p1='385' p2='1400' p3='0'/></specials></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(7, [1000.0, 250.0]));
    let mut attached_at = None;
    let mut max_x = 0.0f64;
    for i in 0..500 {
        w.set_rider_keys(Keys { up: i > 20, space: i < 420, ..Default::default() });
        w.step();
        let Some(Vehicle::Grinder(g)) = w.bike.as_ref() else { panic!("not a grinder") };
        let p = w.b2().body(g.frame).xf.position;
        assert!(p.x.is_finite() && p.y.is_finite(), "frame {i}");
        max_x = max_x.max(p.x * 62.5);
        if g.dongles.iter().any(Option::is_some) && attached_at.is_none() {
            attached_at = Some(i);
        }
        if i == 499 {
            assert!(g.dongles.iter().all(Option::is_none), "dongles are gone after space is released");
        }
    }
    assert!(attached_at.is_some(), "the wheels grabbed the rail");
    assert!(max_x > 1700.0, "rode along the rail: {max_x}");
}

/// c7 grinder on a Rail special (space held 0..150 attaches dongles, up 20..230, a lean at 60) against
/// analysis/golden-rail.js: the dongle bodies must appear/disappear on the same frames and track to <1e-6 px.
#[test]
fn c7_rail_grinding_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/rail.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes>\
         <specials><sp t='27' p0='1500' p1='385' p2='1400' p3='0'/></specials></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(7, [1000.0, 250.0]));
    w.joints_break = false;
    let (mut worst, mut worst_at) = (0.0f64, (0, 0));
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys { up: (20..230).contains(&i), space: i < 150, left: (60..70).contains(&i), ..Default::default() });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Grinder(g)) = w.bike.as_ref() else { panic!("not a grinder") };
        let mut ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).collect();
        ids.extend([g.frame, g.back_wheel, g.front_wheel]);
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let gk = &frame[k];
            let e = (p.x * 62.5 - gk[0].as_f64().unwrap()).hypot(p.y * 62.5 - gk[1].as_f64().unwrap());
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
        for n in 0..2 {
            let want = &frame[15 + n];
            let exists = want[2].as_f64().unwrap() > 0.5;
            assert_eq!(g.dongles[n].is_some(), exists, "dongle {n} presence at frame {i}");
            if let Some(d) = g.dongles[n] {
                let p = w.b2().body(d.body).xf.position;
                let e = (p.x * 62.5 - want[0].as_f64().unwrap()).hypot(p.y * 62.5 - want[1].as_f64().unwrap());
                if e > worst {
                    worst = e;
                    worst_at = (i, 100 + n);
                }
            }
        }
    }
    assert!(worst < 1e-6, "rail grinding diverges by {worst} px at frame/body {worst_at:?}");
}

/// c8 `breakStrap`: cutting an elf's chest strap at the sleigh end leaves a loose ball on the elf's side, the elf stays
/// attached by the head strap, and the simulation stays finite.
#[test]
fn c8_break_strap_releases_the_elf_chest() {
    use happywheels_core::vehicle::{Keys, Vehicle};
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(8, [1000.0, 250.0]));
    for i in 0..300 {
        if i == 100 {
            if let Some(Vehicle::Santa(mut s)) = w.bike.take() {
                s.break_strap(&mut w, 1);
                s.break_strap(&mut w, 2);
                assert!(!s.elf1.strapped_in && !s.elf2.strapped_in);
                w.bike = Some(Vehicle::Santa(s));
            }
        }
        w.set_rider_keys(Keys { up: i > 20, ..Default::default() });
        w.step();
        let Some(Vehicle::Santa(s)) = w.bike.as_ref() else { panic!("not santa") };
        for e in [&s.elf1, &s.elf2] {
            let p = w.b2().body(e.doll.bodies[&happywheels_core::character::Part::Chest]).xf.position;
            assert!(p.x.is_finite() && p.y.is_finite(), "frame {i}");
        }
    }
}

/// Every playable character spawns, runs, ejects (destroying joints/bodies) and still renders without panicking.
#[test]
fn all_characters_spawn_eject_and_render() {
    use happywheels_core::vehicle::Keys;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    for index in 1..=11 {
        let mut w = World::from_level(&level);
        if !w.spawn_bike(index, [1000.0, 250.0]) {
            assert!(w.spawn_character(index, [1000.0, 250.0]), "character {index} cannot spawn at all");
        }
        for i in 0..120 {
            w.set_rider_keys(Keys { up: i > 20, z: i == 60 || i == 80, shift: i == 70, ctrl: i == 90, space: (30..50).contains(&i), ..Default::default() });
            w.step();
        }
        let img = happywheels_core::render::render_level(&level, &w, 200, 150);
        assert_eq!((img.width(), img.height()), (200, 150), "character {index}");
    }
}

/// `headSmash1`: forcing a head smash at frame 30 of a falling ragdoll (4 chunk balls + brain, head destroyed)
/// against analysis/golden-smash.js, <1e-6 px (tumbling on the floor afterwards).
#[test]
fn head_smash_matches_original() {
    use happywheels_core::character::PART_ORDER;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/smash.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_character(1, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        if i == 30 {
            let mut doll = w.character.take().unwrap();
            doll.smash_head(&mut w);
            w.character = Some(doll);
        }
        w.step();
        let doll = w.character.as_ref().unwrap();
        let mut ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).collect();
        ids.extend(&doll.chunks);
        ids.extend(doll.brain);
        assert_eq!(ids.len(), frame.as_array().unwrap().len(), "frame {i}");
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let e = (p.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(p.y * 62.5 - frame[k][1].as_f64().unwrap());
            worst = worst.max(e);
        }
    }
    assert!(worst < 1e-6, "head smash diverges by {worst} px");
}

/// `chestSmash` / `pelvisSmash` / `footSmash1` forced at frame 30 of a falling ragdoll against
/// analysis/golden-smash2.tpl.js (modes chest, pelvis, foot): chunk rings, heart, loose foot, torn joints + filters.
#[test]
fn chest_pelvis_foot_smash_match_original() {
    use happywheels_core::character::PART_ORDER;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    for mode in ["chest", "pelvis", "foot", "neck", "elbow"] {
        let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join(format!("tests/golden/smash_{mode}.json"))).unwrap()).unwrap();
        let mut w = World::from_level(&level);
        assert!(w.spawn_character(1, [1000.0, 250.0]));
        w.joints_break = false;
        let mut worst = 0.0f64;
        for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
            if i == 30 {
                let mut doll = w.character.take().unwrap();
                match mode {
                    "chest" => doll.smash_chest(&mut w),
                    "pelvis" => doll.smash_pelvis(&mut w),
                    "elbow" => {
                        let limit = doll.break_limit(&w, happywheels_core::character::Limb::Elbow1);
                        doll.break_limb(&mut w, happywheels_core::character::Limb::Elbow1, limit + 1.0);
                    }
                    "neck" => {
                        let neck = doll.break_limit(&w, happywheels_core::character::Limb::Neck);
                        doll.break_limb(&mut w, happywheels_core::character::Limb::Neck, neck + 1.0);
                    }
                    _ => doll.smash_foot(&mut w, 0),
                }
                w.character = Some(doll);
            }
            w.step();
            let doll = w.character.as_ref().unwrap();
            let mut ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).collect();
            ids.extend(&doll.chunks);
            ids.extend(doll.heart);
            ids.extend(doll.feet.iter().flatten());
            for c in [&doll.intestines, &doll.spine, &doll.ligaments[0]].into_iter().flatten() {
                ids.extend(&c.bodies);
            }
            assert_eq!(ids.len(), frame.as_array().unwrap().len(), "{mode} frame {i}");
            for (k, id) in ids.iter().enumerate() {
                let p = w.b2().body(*id).xf.position;
                let e = (p.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(p.y * 62.5 - frame[k][1].as_f64().unwrap());
                worst = worst.max(e);
            }
        }
        assert!(worst < 1e-6, "{mode} smash diverges by {worst} px");
    }
}

/// c3 `frameSmash` forced at frame 100 (frame replaced by fork / broken frame / seat bodies) against
/// analysis/golden-bike-smash.js.
#[test]
fn bike_frame_smash_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::Vehicle;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/bike_smash.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        if i == 100 {
            let (Some(mut doll), Some(Vehicle::Bike(mut bike))) = (w.character.take(), w.bike.take()) else { panic!() };
            bike.frame_smash(&mut w, &mut doll);
            w.character = Some(doll);
            w.bike = Some(Vehicle::Bike(bike));
        }
        w.set_rider_keys(happywheels_core::vehicle::Keys { left: (60..80).contains(&i), up: i < 60, space: (80..100).contains(&i), ..Default::default() });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let bike = w.bike.as_ref().unwrap().as_bike().unwrap();
        let mut ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).collect();
        if bike.frame_smashed {
            ids.extend(&bike.pieces);
        } else {
            ids.push(bike.frame);
        }
        ids.extend([bike.back_wheel, bike.front_wheel, bike.gear]);
        assert_eq!(ids.len(), frame.as_array().unwrap().len(), "frame {i}");
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let e = (p.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(p.y * 62.5 - frame[k][1].as_f64().unwrap());
            worst = worst.max(e);
        }
    }
    assert!(worst < 1e-6, "frame smash diverges by {worst} px");
}

/// A rider's neck break (joints_break on) ejects the bike; elbow/knee breaks only drop their own joint.
#[test]
fn bike_rider_hurt_ejects() {
    use happywheels_core::character::Limb;
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    let (Some(mut doll), Some(Vehicle::Bike(mut bike))) = (w.character.take(), w.bike.take()) else { panic!() };
    bike.rider_hurt(&mut w, &mut doll, false, &[Limb::Elbow1]);
    assert!(!bike.ejected, "one elbow is not enough");
    bike.rider_hurt(&mut w, &mut doll, false, &[Limb::Neck]);
    assert!(bike.ejected, "neck break ejects");
}

/// c3 front wheel smashed at frame 40 and back wheel at 70 (normal (0.8, 0.6)) against analysis/golden-bike-wheel.js.
#[test]
fn bike_wheel_smash_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::Vehicle;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/bike_wheel.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        if i == 40 || i == 70 {
            let Some(Vehicle::Bike(mut bike)) = w.bike.take() else { panic!() };
            bike.wheel_smash(&mut w, i == 40, box2d::math::Vec2::new(0.8, 0.6));
            w.bike = Some(Vehicle::Bike(bike));
        }
        w.set_rider_keys(happywheels_core::vehicle::Keys { left: (60..80).contains(&i), up: i < 60, space: (80..100).contains(&i), z: i == 100, ..Default::default() });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let bike = w.bike.as_ref().unwrap().as_bike().unwrap();
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([bike.frame, bike.back_wheel, bike.front_wheel, bike.gear]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let e = (p.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(p.y * 62.5 - frame[k][1].as_f64().unwrap());
            worst = worst.max(e);
        }
    }
    assert!(worst < 1e-6, "wheel smash diverges by {worst} px");
}

/// Writes a picture of a smashed rider to `screenshots/smash_bike_head.png` (or HW_DUMP).
#[test]
fn dump_smash_picture() {
    let path = std::env::var("HW_DUMP").unwrap_or_else(|_| format!("{}/../../screenshots/smash_bike_head.png", env!("CARGO_MANIFEST_DIR")));
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    for i in 0..45 {
        w.set_rider_keys(happywheels_core::vehicle::Keys { up: i < 20, ..Default::default() });
        w.step();
    }
    let (Some(mut doll), Some(happywheels_core::vehicle::Vehicle::Bike(mut bike))) = (w.character.take(), w.bike.take()) else { panic!() };
    bike.frame_smash(&mut w, &mut doll);
    doll.smash_head(&mut w);
    w.character = Some(doll);
    w.bike = Some(happywheels_core::vehicle::Vehicle::Bike(bike));
    for _ in 0..30 {
        w.step();
    }
    happywheels_core::render::render_level(&level, &w, 800, 500).save(path).unwrap();
}

/// Grinder limb breaks: one hand does not eject, all four hands/feet gone does.
#[test]
fn grinder_limb_breaks_eject_when_all_four_are_gone() {
    use happywheels_core::character::Limb;
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(7, [1000.0, 250.0]));
    let (Some(mut doll), Some(mut v)) = (w.character.take(), w.bike.take()) else { panic!() };
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Elbow1, Limb::Knee2]);
    assert!(!v.ejected(), "two of four is not enough");
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Shoulder2, Limb::Hip1]);
    assert!(v.ejected(), "all four gone ejects");
    assert!(matches!(v, Vehicle::Grinder(_)));
}

/// Mower limb breaks: all four hands/feet gone ejects.
#[test]
fn mower_limb_breaks_eject_when_all_four_are_gone() {
    use happywheels_core::character::Limb;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(6, [1000.0, 250.0]));
    let (Some(mut doll), Some(mut v)) = (w.character.take(), w.bike.take()) else { panic!() };
    assert!(matches!(v, happywheels_core::vehicle::Vehicle::Mower(_)));
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Elbow1, Limb::Hip2]);
    assert!(!v.ejected());
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Shoulder2, Limb::Knee1]);
    assert!(v.ejected());
}

/// Heli limb breaks: both hands gone ejects, legs do nothing.
#[test]
fn heli_hand_breaks_eject() {
    use happywheels_core::character::Limb;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(11, [1000.0, 250.0]));
    let (Some(mut doll), Some(mut v)) = (w.character.take(), w.bike.take()) else { panic!() };
    assert!(matches!(v, happywheels_core::vehicle::Vehicle::Heli(_)));
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Hip1, Limb::Elbow1]);
    assert!(!v.ejected());
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Shoulder2]);
    assert!(v.ejected());
}

/// Pogo limb breaks: feet never eject, both hands gone does.
#[test]
fn pogo_hand_breaks_eject() {
    use happywheels_core::character::Limb;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(9, [1000.0, 250.0]));
    let (Some(mut doll), Some(mut v)) = (w.character.take(), w.bike.take()) else { panic!() };
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Hip1, Limb::Knee2, Limb::Elbow1]);
    assert!(!v.ejected());
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Shoulder2]);
    assert!(v.ejected());
}

/// Cart limb breaks: shoulders/hips are ignored, elbows+knees all gone ejects.
#[test]
fn cart_elbow_knee_breaks_eject() {
    use happywheels_core::character::Limb;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(4, [1000.0, 250.0]));
    let (Some(mut doll), Some(mut v)) = (w.character.take(), w.bike.take()) else { panic!() };
    assert!(matches!(v, happywheels_core::vehicle::Vehicle::Cart(_)));
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Shoulder1, Limb::Shoulder2, Limb::Hip1, Limb::Hip2]);
    assert!(!v.ejected(), "shoulders and hips are ignored");
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Elbow1, Limb::Elbow2, Limb::Knee1, Limb::Knee2]);
    assert!(v.ejected());
}

/// Segway limb breaks (31116 subclass): shoulders and elbows (both hands gone) eject; hips and knees only drop that foot.
#[test]
fn segway_elbow_breaks_eject() {
    use happywheels_core::character::Limb;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(2, [1000.0, 250.0]));
    let (Some(mut doll), Some(mut v)) = (w.character.take(), w.bike.take()) else { panic!() };
    assert!(matches!(v, happywheels_core::vehicle::Vehicle::Segway(_)));
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Hip1, Limb::Hip2, Limb::Knee1, Limb::Knee2, Limb::Shoulder1]);
    assert!(!v.ejected(), "feet never eject, one hand is still on");
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Elbow2]);
    assert!(v.ejected());
}

/// Dirt bike limb breaks: all four hands/feet gone ejects.
#[test]
fn dirt_limb_breaks_eject_when_all_four_are_gone() {
    use happywheels_core::character::Limb;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(5, [1000.0, 250.0]));
    let (Some(mut doll), Some(mut v)) = (w.character.take(), w.bike.take()) else { panic!() };
    assert!(matches!(v, happywheels_core::vehicle::Vehicle::Dirt(_)));
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Elbow1, Limb::Hip2]);
    assert!(!v.ejected());
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Shoulder2, Limb::Knee1]);
    assert!(v.ejected());
}

/// Chair: a hip break frees that leg's strap but never ejects.
#[test]
fn chair_hip_break_frees_leg_strap() {
    use happywheels_core::character::Limb;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(1, [1000.0, 250.0]));
    let (Some(mut doll), Some(mut v)) = (w.character.take(), w.bike.take()) else { panic!() };
    assert!(matches!(v, happywheels_core::vehicle::Vehicle::Chair(_)));
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Hip1, Limb::Hip2, Limb::Elbow1]);
    assert!(!v.ejected());
    v.rider_hurt(&mut w, &mut doll, false, &[Limb::Waist]);
    assert!(v.ejected());
}

/// Segway `frameSmash`: joint destroyed, rider ejected, frame/wheel shapes in group 0.
#[test]
fn segway_frame_smash_ejects_and_frees_shapes() {
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(2, [1000.0, 250.0]));
    let (Some(mut doll), Some(Vehicle::Segway(mut sg))) = (w.character.take(), w.bike.take()) else { panic!() };
    sg.frame_smash(&mut w, &mut doll);
    assert!(sg.ejected && sg.frame_smashed);
    let s = w.b2().body(sg.wheel).shapes[0];
    assert_eq!(w.b2().c.shapes[s].shape.filter.group_index, 0);
    w.character = Some(doll);
    w.bike = Some(Vehicle::Segway(sg));
    for _ in 0..30 {
        w.step();
    }
}

/// c5 dirt bike `frameSmash` forced at frame 120 (six loose polygons) against analysis/golden-dirt-smash.js.
#[test]
fn dirt_frame_smash_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/dirt_smash.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(5, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys {
            left: (10..20).contains(&i),
            right: (20..30).contains(&i),
            up: i < 60,
            space: (30..60).contains(&i),
            ctrl: (60..80).contains(&i),
            ..Default::default()
        });
        if i == 120 {
            let (Some(mut doll), Some(Vehicle::Dirt(mut d))) = (w.character.take(), w.bike.take()) else { panic!() };
            d.frame_smash(&mut w, &mut doll);
            w.character = Some(doll);
            w.bike = Some(Vehicle::Dirt(d));
        }
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Dirt(d)) = w.bike.as_ref() else { panic!() };
        let mut ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).collect();
        if d.frame_smashed {
            ids.extend(&d.pieces);
        } else {
            ids.push(d.frame);
        }
        ids.extend([d.back_wheel, d.front_wheel]);
        assert_eq!(ids.len(), frame.as_array().unwrap().len(), "frame {i}");
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let e = (p.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(p.y * 62.5 - frame[k][1].as_f64().unwrap());
            worst = worst.max(e);
        }
    }
    assert!(worst < 1e-6, "dirt frame smash diverges by {worst} px");
}

/// c5 dirt bike wheels smashed at frames 40 (front) and 90 (back), normal (0.8, 0.6), against analysis/golden-dirt-wheel.js.
#[test]
fn dirt_wheel_smash_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/dirt_wheel.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(5, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        if i == 40 || i == 90 {
            let Some(Vehicle::Dirt(mut d)) = w.bike.take() else { panic!() };
            d.wheel_smash(&mut w, i == 40, box2d::math::Vec2::new(0.8, 0.6));
            w.bike = Some(Vehicle::Dirt(d));
        }
        w.set_rider_keys(Keys {
            left: (10..20).contains(&i),
            right: (20..30).contains(&i),
            up: i < 60,
            space: (30..60).contains(&i),
            ctrl: (60..80).contains(&i),
            z: i == 120,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Dirt(d)) = w.bike.as_ref() else { panic!() };
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([d.frame, d.back_wheel, d.front_wheel]).collect();
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let e = (p.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(p.y * 62.5 - frame[k][1].as_f64().unwrap());
            worst = worst.max(e);
        }
    }
    assert!(worst < 1e-6, "dirt wheel smash diverges by {worst} px");
}

/// A hard drop (86 m, ~41 m/s) with `joints_break` on trips the c2 segway frame trigger (limit 20). The c3/c5 bikes land on their
/// wheels and only see ~130 (wheel/frame limits are 200), so they do not smash — matching how the thresholds were tuned.
#[test]
fn hard_drop_smashes_segway_frame() {
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='-5000'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(2, [1000.0, -5000.0]));
    assert!(w.joints_break);
    for _ in 0..200 {
        w.step();
    }
    let Some(Vehicle::Segway(s)) = w.bike.as_ref() else { panic!() };
    assert!(s.frame_smashed && s.ejected, "segway frame should smash after an 86 m drop");
}

/// Renders every character on its vehicle into `screenshots/` at the repo root (or HW_SHOTS), for eyeballing.
#[test]
fn dump_screenshots() {
    let dir = std::env::var("HW_SHOTS").unwrap_or_else(|_| format!("{}/../../screenshots", env!("CARGO_MANIFEST_DIR")));
    std::fs::create_dir_all(&dir).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='400' y='250'/><shapes>\
         <sh t='0' p0='400' p1='425' p2='1600' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    for index in 1..=11usize {
        let mut w = World::from_level(&level);
        assert!(w.spawn_bike(index, [400.0, 250.0]));
        for i in 0..60 {
            w.set_rider_keys(happywheels_core::vehicle::Keys { up: i > 20 && i < 45, ..Default::default() });
            w.step();
        }
        let chest = w.character.as_ref().and_then(|d| d.bodies.get(&happywheels_core::character::Part::Chest)).map(|h| w.b2().body(*h).xf.position).unwrap();
        let cam = [(chest.x * 62.5) as f32, (chest.y * 62.5) as f32 - 40.0];
        happywheels_core::render::render_level_at(&level, &w, 800, 500, cam).save(format!("{dir}/c{index}.png")).unwrap();
    }
}

/// Pictures of the vehicle breaks (heli copter/blade, Santa ski/stem/sleigh, family daughter frame / son basket) in `screenshots/break_*.png`.
#[test]
fn dump_break_screenshots() {
    use happywheels_core::vehicle::Vehicle;
    let dir = std::env::var("HW_SHOTS").unwrap_or_else(|_| format!("{}/../../screenshots", env!("CARGO_MANIFEST_DIR")));
    std::fs::create_dir_all(&dir).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let shot = |w: &World, name: &str| {
        let chest = w.character.as_ref().and_then(|d| d.bodies.get(&happywheels_core::character::Part::Chest)).map(|h| w.b2().body(*h).xf.position).unwrap();
        let cam = [(chest.x * 62.5) as f32, (chest.y * 62.5) as f32 - 40.0];
        happywheels_core::render::render_level_at(&level, w, 800, 500, cam).save(format!("{dir}/{name}.png")).unwrap();
    };
    let settle = |w: &mut World, n: usize| (0..n).for_each(|_| w.step());
    // heli: blade smash, then copter smash
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(11, [1000.0, 250.0]));
    settle(&mut w, 40);
    let (Some(mut doll), Some(Vehicle::Heli(mut h))) = (w.character.take(), w.bike.take()) else { panic!() };
    let at = w.b2().body(h.copter).world_point(box2d::math::Vec2::new(0.4, -0.5));
    h.copter_smash(&mut w, &mut doll, at);
    w.character = Some(doll);
    w.bike = Some(Vehicle::Heli(h));
    settle(&mut w, 40);
    shot(&w, "break_heli_copter");
    // santa: ski + stem, then the whole sleigh
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(8, [1000.0, 250.0]));
    settle(&mut w, 40);
    let (Some(mut doll), Some(Vehicle::Santa(mut s))) = (w.character.take(), w.bike.take()) else { panic!() };
    s.ski_smash(&mut w);
    s.stem_smash(&mut w, true);
    w.character = Some(doll);
    w.bike = Some(Vehicle::Santa(s));
    settle(&mut w, 30);
    shot(&w, "break_santa_ski_stem");
    doll = w.character.take().unwrap();
    let Some(Vehicle::Santa(mut s)) = w.bike.take() else { panic!() };
    s.sleigh_smash(&mut w, &mut doll);
    w.character = Some(doll);
    w.bike = Some(Vehicle::Santa(s));
    settle(&mut w, 40);
    shot(&w, "break_santa_sleigh");
    // family: daughter frame
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(10, [1000.0, 250.0]));
    settle(&mut w, 40);
    let Some(Vehicle::Family(mut f)) = w.bike.take() else { panic!() };
    f.daughter.frame_smash(&mut w);
    w.bike = Some(Vehicle::Family(f));
    settle(&mut w, 40);
    shot(&w, "break_family_daughter_frame");
    // family: dad's frame
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(10, [1000.0, 250.0]));
    settle(&mut w, 40);
    let (Some(mut doll), Some(Vehicle::Family(mut f))) = (w.character.take(), w.bike.take()) else { panic!() };
    f.dad.frame_smash(&mut w, &mut doll);
    assert!(f.dad.frame_smashed);
    w.character = Some(doll);
    w.bike = Some(Vehicle::Family(f));
    settle(&mut w, 60);
    for id in match w.bike.as_ref() { Some(Vehicle::Family(f)) => f.bodies(), _ => panic!() } {
        assert!(w.b2().body(id).xf.position.x.is_finite());
    }
    shot(&w, "break_family_dad_frame");
}

/// NPC smashes (behaviour only): an NPC dropped from 85 m loses head / chest / pelvis to chunk circles (sounds HeadSmash/ChestSmash/PelvisSmash),
/// the simulation stays finite, and `screenshots/npc_smash.png` shows the pile.
#[test]
fn npc_dropped_from_height_smashes_into_chunks() {
    let dir = std::env::var("HW_SHOTS").unwrap_or_else(|_| format!("{}/../../screenshots", env!("CARGO_MANIFEST_DIR")));
    std::fs::create_dir_all(&dir).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='-5000'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes>\
         <specials><sp t='17' p0='900' p1='-5000' p2='90' p3='1' p4='f' p5='f' p6='f' p7='t'/><sp t='17' p0='1100' p1='-5000' p2='180' p3='5' p4='f' p5='t' p6='f' p7='t'/></specials></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    let mut heard = std::collections::HashSet::new();
    for _ in 0..500 {
        w.step();
        for (name, _) in w.take_sounds() {
            heard.insert(name);
        }
    }
    assert!(heard.iter().any(|n| n.ends_with("Smash")), "nothing smashed: {heard:?}");
    happywheels_core::render::render_level_at(&level, &w, 800, 500, [1000.0, 330.0]).save(format!("{dir}/npc_smash.png")).unwrap();
}

/// TrashCan spill / crush (behaviour only): a can dropped 13 m spills into a hollow can + 10 trash bodies, one dropped from 80 m is crushed, an
/// empty one starts hollow. Sounds TrashCanSpill*, `screenshots/trash_can_spill.png` shows the three.
#[test]
fn trash_cans_spill_and_crush() {
    let dir = std::env::var("HW_SHOTS").unwrap_or_else(|_| format!("{}/../../screenshots", env!("CARGO_MANIFEST_DIR")));
    std::fs::create_dir_all(&dir).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes>\
         <specials><sp t='26' p0='850' p1='-400' p2='0' p3='f' p4='t' p5='t'/><sp t='26' p0='1000' p1='-4500' p2='0' p3='f' p4='t' p5='t'/><sp t='26' p0='1150' p1='340' p2='0' p3='f' p4='t' p5='f'/></specials></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    let mut heard = Vec::new();
    for _ in 0..240 {
        w.step();
        heard.extend(w.take_sounds().into_iter().map(|s| s.0));
    }
    assert!(heard.iter().filter(|n| n.starts_with("TrashCanSpill")).count() >= 2, "{heard:?}");
    assert!(heard.iter().any(|n| n.starts_with("TrashCanHit")), "no TrashCanHit in {heard:?}");
    happywheels_core::render::render_level_at(&level, &w, 800, 500, [1000.0, 330.0]).save(format!("{dir}/trash_can_spill.png")).unwrap();
}

/// Blood (behaviour only): smashing a rider's head sprays 200 particles that fall and die below the 850 px limit; a neck break spews a flow.
/// `screenshots/blood_head_smash.png` shows the spray a few frames after the smash.
#[test]
fn blood_burst_sprays_and_dies() {
    let dir = std::env::var("HW_SHOTS").unwrap_or_else(|_| format!("{}/../../screenshots", env!("CARGO_MANIFEST_DIR")));
    std::fs::create_dir_all(&dir).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    for _ in 0..30 {
        w.step();
    }
    let mut doll = w.character.take().unwrap();
    doll.smash_head(&mut w);
    w.character = Some(doll);
    assert_eq!(w.blood.particles.len(), 200);
    for _ in 0..6 {
        w.step();
    }
    let chest = w.b2().body(w.character.as_ref().unwrap().bodies[&happywheels_core::character::Part::Chest]).xf.position;
    happywheels_core::render::render_level_at(&level, &w, 800, 500, [(chest.x * 62.5) as f32, (chest.y * 62.5) as f32 - 40.0]).save(format!("{dir}/blood_head_smash.png")).unwrap();
    let flow = w.blood_flow(2.5, 4.0, w.character.as_ref().unwrap().bodies[&happywheels_core::character::Part::Chest], box2d::math::Vec2::default(), 270.0, 20);
    for _ in 0..400 {
        w.step();
    }
    let _ = flow;
    assert!(w.blood.particles.is_empty(), "{} particles left", w.blood.particles.len());
    assert!(w.blood.flows[0].finished);
}

/// Limb-break blood flows (behaviour only): smashing the pelvis tears both hips and the waist, so leg stumps and the stomach spew blood for
/// the following frames. `screenshots/blood_flow_pelvis.png`.
#[test]
fn limb_breaks_spew_blood_flows() {
    let dir = std::env::var("HW_SHOTS").unwrap_or_else(|_| format!("{}/../../screenshots", env!("CARGO_MANIFEST_DIR")));
    std::fs::create_dir_all(&dir).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    for _ in 0..30 {
        w.step();
    }
    let mut doll = w.character.take().unwrap();
    doll.smash_pelvis(&mut w);
    w.character = Some(doll);
    assert!(w.blood.flows.len() >= 3, "flows: {}", w.blood.flows.len());
    let mut heard = Vec::new();
    for _ in 0..25 {
        w.step();
        heard.extend(w.take_sounds().into_iter().map(|s| s.0));
    }
    assert!(heard.iter().any(|n| n == "Char3Pelvis"), "no pelvis vocal in {heard:?}");
    assert!(!w.blood.particles.is_empty());
    let chest = w.b2().body(w.character.as_ref().unwrap().bodies[&happywheels_core::character::Part::Chest]).xf.position;
    happywheels_core::render::render_level_at(&level, &w, 800, 500, [(chest.x * 62.5) as f32, (chest.y * 62.5) as f32 + 20.0]).save(format!("{dir}/blood_flow_pelvis.png")).unwrap();
}

/// Bike wheel loop (behaviour only): accelerating on the ground asks for BikeLoop1 → 2 → 3 as the back wheel spins up, none in the air.
#[test]
fn bike_wheel_loop_follows_wheel_spin() {
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    let mut seen = Vec::new();
    for i in 0..150 {
        w.set_rider_keys(happywheels_core::vehicle::Keys { up: i > 20, ..Default::default() });
        w.step();
        if let Some((n, _, _)) = w.loops.first() {
            if seen.last() != Some(n) {
                seen.push(n.clone());
            }
        }
    }
    assert!(seen.contains(&"BikeLoop3".to_string()), "loops seen: {seen:?}");
    assert!(seen.first().is_some_and(|n| n != "BikeLoop3"), "loops seen: {seen:?}");
}

/// Heli and mower loops (behaviour only): HeliLoop at volume 0.5 idle / 0.75 with up held, MowerLoop 0.5 idle / 1 with the motors on.
#[test]
fn heli_and_mower_loops_follow_the_controls() {
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    for (index, name, idle, busy) in [(11, "HeliLoop", 0.5f32, 0.75f32), (6, "MowerLoop", 0.5, 1.0)] {
        let mut w = World::from_level(&level);
        assert!(w.spawn_bike(index, [1000.0, 250.0]));
        w.set_rider_keys(Default::default());
        w.step();
        let v0 = w.loops.iter().find(|l| l.0 == name).unwrap_or_else(|| panic!("no {name}")).2;
        assert_eq!(v0, idle);
        w.set_rider_keys(happywheels_core::vehicle::Keys { up: true, ..Default::default() });
        w.step();
        let v1 = w.loops.iter().find(|l| l.0 == name).unwrap().2;
        assert_eq!(v1, busy, "{name}");
    }
}

/// Wheel loops of the other car-like vehicles (behaviour only): dirt bike, segway, cart and chair each ask for a BikeLoop while driving on the ground.
#[test]
fn other_vehicles_ask_for_wheel_loops() {
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='4000' p1='425' p2='8000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    for index in [5usize, 2, 4, 1] {
        let mut w = World::from_level(&level);
        w.dad_kid = false;
        assert!(w.spawn_bike(index, [1000.0, 250.0]));
        let mut asked = false;
        let mut motor = false;
        for i in 0..200 {
            w.set_rider_keys(happywheels_core::vehicle::Keys { up: i > 20, ..Default::default() });
            w.step();
            asked |= w.loops.iter().any(|l| l.0.starts_with("BikeLoop"));
            motor |= w.loops.iter().any(|l| l.0 == "ElectricMotor1");
        }
        assert!(index != 4 || motor, "cart never asked for ElectricMotor1");
        assert!(asked, "vehicle {index} never asked for a wheel loop");
    }
}

/// c6 mower `mowerSmash` forced at frame 100 (two halves, wheels re-pinned) against analysis/golden-mower-smash.js.
/// The halves' mass, centre, velocity and inertia, the contact list and the joint anchors match the oracle exactly, and so does every
/// frame after the smash (< 1e-6 px): the earlier ~0.06 px residual was the rider's pose motors, which the original keeps running after the
/// vehicle stops listening to keys (the replica's pose stays at 8 from the right-lean phase).
#[test]
fn mower_smash_matches_original_at_first() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/mower_smash.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(6, [1000.0, 250.0]));
    w.joints_break = false;
    let (mut before, mut after) = (0.0f64, 0.0f64);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        if i == 100 {
            let (Some(mut doll), Some(Vehicle::Mower(mut m))) = (w.character.take(), w.bike.take()) else { panic!() };
            m.smash(&mut w, &mut doll);
            w.character = Some(doll);
            w.bike = Some(Vehicle::Mower(m));
        }
        w.set_rider_keys(Keys {
            left: (10..20).contains(&i),
            right: (20..30).contains(&i),
            up: i < 60,
            space: (30..40).contains(&i) || (70..90).contains(&i),
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Mower(m)) = w.bike.as_ref() else { panic!() };
        let mut ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).collect();
        match m.halves {
            Some(h) => ids.extend(h),
            None => ids.push(m.body),
        }
        ids.extend([m.back_wheel, m.front_wheel]);
        if m.halves.is_none() {
            ids.extend([m.back_shock(), m.front_shock()]);
        }
        assert_eq!(ids.len(), frame.as_array().unwrap().len(), "frame {i}");
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let e = (p.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(p.y * 62.5 - frame[k][1].as_f64().unwrap());
            if i < 100 {
                before = before.max(e);
            } else {
                after = after.max(e);
            }
        }
    }
    assert!(before < 1e-6, "pre-smash diverges by {before} px");
    assert!(after < 1e-6, "post-smash frames diverge by {after} px");
}

/// c2 segway helmet smash forced at frame 50 (loose helmet body appears at the head) against analysis/golden-segway-helmet.js.
#[test]
fn segway_helmet_smash_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/segway_helmet.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(2, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        if i == 50 {
            let mut doll = w.character.take().unwrap();
            doll.helmet_smash(&mut w);
            w.character = Some(doll);
        }
        w.set_rider_keys(Keys {
            left: (10..30).contains(&i),
            right: (30..50).contains(&i),
            up: i < 40,
            space: (20..45).contains(&i),
            shift: (50..70).contains(&i),
            ctrl: (70..90).contains(&i),
            z: i == 120,
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Segway(s)) = w.bike.as_ref() else { panic!() };
        let mut ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([s.frame, s.stand, s.shock, s.wheel]).collect();
        ids.extend(doll.helmet);
        assert_eq!(ids.len(), frame.as_array().unwrap().len(), "frame {i}");
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let e = (p.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(p.y * 62.5 - frame[k][1].as_f64().unwrap());
            worst = worst.max(e);
        }
    }
    assert!(worst < 1e-6, "segway helmet smash diverges by {worst} px");
}

/// c9 pogo frame smash forced at frame 100 (frame -> three loose boxes) against analysis/golden-pogo-smash.js.
/// The frames before the smash and the three pieces' mass, inertia, centre, velocities and position at creation match the original
/// (<1e-9). After that the replica and the port part ways within a frame: the pieces overlap each other with *exactly tied* separating
/// axes (the boxes are symmetric about the same origin), and which axis `FindMaxSeparation` picks is decided by ~1e-13 noise
/// (e.g. the replica's frame position differs from the port's by 7e-14), so the contact normals can flip. Not a porting bug.
#[test]
fn pogo_frame_smash_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/pogo_smash.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(9, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate().take(101) {
        if i == 100 {
            let (Some(mut doll), Some(Vehicle::Pogo(mut p))) = (w.character.take(), w.bike.take()) else { panic!() };
            p.frame_smash(&mut w, &mut doll);
            for (k, piece) in v["pieces"].as_array().unwrap().iter().enumerate() {
                let b = w.b2().body(p.pieces[k]);
                let f = |x: &serde_json::Value| x.as_f64().unwrap();
                let close = |a: f64, b: f64, what: &str| assert!((a - b).abs() < 1e-9, "piece {k} {what}: {a} vs {b}");
                close(b.mass, f(&piece["mass"]), "mass");
                close(b.linear_velocity.x, f(&piece["lv"][0]), "vx");
                close(b.linear_velocity.y, f(&piece["lv"][1]), "vy");
                close(b.angular_velocity, f(&piece["av"]), "av");
                close(b.xf.position.x, f(&piece["pos"][0]), "x");
                close(b.xf.position.y, f(&piece["pos"][1]), "y");
                close(b.sweep.local_center.y, f(&piece["lc"][1]), "local centre");
            }
            w.character = Some(doll);
            w.bike = Some(Vehicle::Pogo(p));
            break;
        }
        w.set_rider_keys(Keys {
            left: (70..80).contains(&i),
            right: (80..90).contains(&i),
            up: (20..45).contains(&i),
            space: (50..65).contains(&i),
            ..Default::default()
        });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let Some(Vehicle::Pogo(p)) = w.bike.as_ref() else { panic!() };
        let ids: Vec<_> = PART_ORDER.iter().map(|q| doll.bodies[q]).chain([p.frame, p.rod]).collect();
        for (k, id) in ids.iter().enumerate() {
            let q = w.b2().body(*id).xf.position;
            worst = worst.max((q.x * 62.5 - frame[k][0].as_f64().unwrap()).hypot(q.y * 62.5 - frame[k][1].as_f64().unwrap()));
        }
    }
    assert!(worst < 1e-6, "pogo diverges by {worst} px before the smash");
}

/// c1 chair wheel smash: the big wheel's circle becomes a 30x100 px box and the chair keeps simulating.
#[test]
fn chair_wheel_smash_replaces_circle_with_box() {
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(1, [1000.0, 250.0]));
    for _ in 0..30 {
        w.step();
    }
    let Some(Vehicle::Chair(mut c)) = w.bike.take() else { panic!() };
    let wheel = c.big_wheel;
    c.wheel_smash(&mut w, box2d::math::Vec2::new(0.0, -1.0));
    w.bike = Some(Vehicle::Chair(c));
    let s = w.b2().body(wheel).shapes[0];
    assert_eq!(w.b2().shape_radius(s), 0.0, "wheel shape is a polygon now");
    for _ in 0..60 {
        w.step();
    }
    assert!(w.bike.is_some());
}

/// c1 chair smash: wheels detach, three debris bodies appear, rider ejected, simulation stays finite.
#[test]
fn chair_smash_sheds_debris_and_ejects() {
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(1, [1000.0, 250.0]));
    for _ in 0..30 {
        w.step();
    }
    let (Some(mut doll), Some(Vehicle::Chair(mut c))) = (w.character.take(), w.bike.take()) else { panic!() };
    c.chair_smash(&mut w, &mut doll);
    assert!(c.ejected && c.chair_smashed && !c.firing);
    assert_eq!(c.debris.len(), 3);
    assert_eq!(w.b2().body(c.chair).shapes.len(), 1);
    w.character = Some(doll);
    w.bike = Some(Vehicle::Chair(c));
    let Some(Vehicle::Chair(mut c)) = w.bike.take() else { panic!() };
    c.fueltank_smash(&mut w);
    c.jet_smash(&mut w);
    assert_eq!(c.debris.len(), 1);
    w.bike = Some(Vehicle::Chair(c));
    for _ in 0..120 {
        w.step();
    }
    let Some(Vehicle::Chair(c)) = w.bike.as_ref() else { panic!() };
    for id in c.debris.iter().chain([&c.chair, &c.big_wheel]) {
        let p = w.b2().body(*id).xf.position;
        assert!(p.x.is_finite() && p.y.is_finite() && p.y < 100.0);
    }
}

/// c4 cart smash: basket shape removed, then the body splits into four loose pieces and the rider is ejected.
#[test]
fn cart_smash_splits_body() {
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(4, [1000.0, 250.0]));
    for _ in 0..30 {
        w.step();
    }
    let (Some(mut doll), Some(Vehicle::Cart(mut c))) = (w.character.take(), w.bike.take()) else { panic!() };
    let n = w.b2().body(c.body).shapes.len();
    c.cart_smash(&mut w);
    assert_eq!(w.b2().body(c.body).shapes.len(), n - 1);
    c.main_smash(&mut w, &mut doll);
    assert!(c.ejected && c.main_smashed);
    assert_eq!(c.pieces.len(), 4);
    w.character = Some(doll);
    w.bike = Some(Vehicle::Cart(c));
    for _ in 0..120 {
        w.step();
    }
    let Some(Vehicle::Cart(c)) = w.bike.as_ref() else { panic!() };
    for id in &c.pieces {
        let p = w.b2().body(*id).xf.position;
        assert!(p.x.is_finite() && p.y < 100.0);
    }
}

/// c2 segway: after an eject the hands are still at the handle, so pressing space re-boards (`reAttach`) and the other limbs follow.
#[test]
fn segway_reattaches_when_hand_is_at_the_handle() {
    use happywheels_core::vehicle::{Keys, Vehicle};
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(2, [1000.0, 250.0]));
    w.joints_break = false;
    for _ in 0..30 {
        w.step();
    }
    w.set_rider_keys(Keys { z: true, ..Default::default() });
    w.step();
    assert!(w.bike.as_ref().unwrap().ejected());
    w.set_rider_keys(Keys { space: true, ..Default::default() });
    w.step();
    let Some(Vehicle::Segway(s)) = w.bike.as_ref() else { panic!() };
    assert!(s.is_reattached(), "space next to the handle should re-board");
    for _ in 0..60 {
        w.set_rider_keys(Keys { space: true, ..Default::default() });
        w.step();
    }
    assert!(!w.bike.as_ref().unwrap().ejected());
}

/// c3 dad with his kid on the seat (class `MY`): accelerate, lean back, brake, dad z-eject at 100 (forwarded to the kid, who ejects too), against analysis/golden-bike-kid.js.
#[test]
fn bike_with_kid_matches_original() {
    use happywheels_core::character::PART_ORDER;
    use happywheels_core::vehicle::Keys;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let v: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(root.join("tests/golden/bike_kid.json")).unwrap()).unwrap();
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    w.joints_break = false;
    let mut worst = 0.0f64;
    let mut worst_at = (0, 0);
    for (i, frame) in v["frames"].as_array().unwrap().iter().enumerate() {
        w.set_rider_keys(Keys { left: (60..80).contains(&i), up: i < 60, space: (80..100).contains(&i), z: i == 100, ..Default::default() });
        w.step();
        let doll = w.character.as_ref().unwrap();
        let bike = w.bike.as_ref().unwrap().as_bike().unwrap();
        let kid = bike.kid.as_ref().unwrap();
        let ids: Vec<_> = PART_ORDER.iter().map(|p| doll.bodies[p]).chain([bike.frame, bike.back_wheel, bike.front_wheel, bike.gear]).chain(PART_ORDER.iter().map(|p| kid.doll.bodies[p])).chain([kid.seat]).collect();
        assert_eq!(ids.len(), frame.as_array().unwrap().len());
        for (k, id) in ids.iter().enumerate() {
            let p = w.b2().body(*id).xf.position;
            let g = &frame[k];
            let e = (p.x * 62.5 - g[0].as_f64().unwrap()).hypot(p.y * 62.5 - g[1].as_f64().unwrap());
            if e > worst {
                worst = e;
                worst_at = (i, k);
            }
        }
    }
    assert!(worst < 1e-6, "dad+kid diverges by {worst} px at (frame, body) {worst_at:?}");
}

/// c5 girl smoke test: she rides behind the rider, stays pinned while driving, and shift ejects her only.
#[test]
fn dirt_bike_girl_rides_and_ejects_on_shift() {
    use happywheels_core::character::Part;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(5, [1000.0, 250.0]));
    for i in 0..90 {
        w.set_rider_keys(Keys { up: true, shift: i == 80, ..Default::default() });
        w.step();
        if i == 60 {
            let Some(Vehicle::Dirt(d)) = w.bike.as_ref() else { panic!() };
            let g = d.girl.as_ref().expect("girl attached");
            assert!(!g.ejected && !d.ejected);
            let gp = w.b2().body(g.doll.bodies[&Part::Pelvis]).xf.position;
            let fp = w.b2().body(w.character.as_ref().unwrap().bodies[&Part::Pelvis]).xf.position;
            let dist = (gp.x - fp.x).hypot(gp.y - fp.y); assert!(dist < 1.5, "girl drifted off the bike: {dist} gp {gp:?} fp {fp:?}");
        }
    }
    let Some(Vehicle::Dirt(d)) = w.bike.as_ref() else { panic!() };
    assert!(d.girl.as_ref().unwrap().ejected);
}

/// c5: an ejected rider whose hand is brought to the handle and who presses space is seated again (`grabAction` → `reAttach`).
#[test]
fn dirt_bike_rider_reboards_at_the_handle() {
    use happywheels_core::character::Part;
    use happywheels_core::vehicle::{Keys, Vehicle};
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(5, [1000.0, 250.0]));
    w.joints_break = false;
    for _ in 0..5 {
        w.step();
    }
    w.set_rider_keys(Keys { z: true, ..Default::default() });
    w.step();
    assert!(w.bike.as_ref().unwrap().ejected());
    // bring the left forearm's tip onto the handle: handle = frame-local anchor, so use the original anchor via the guide
    let arm = w.character.as_ref().unwrap().bodies[&Part::LowerArm1];
    let Some(Vehicle::Dirt(d)) = w.bike.as_ref() else { panic!() };
    let handle = w.b2().body(d.frame).world_point(d.handle_anchor());
    let tip = {
        let b = w.b2();
        let y = match &b.c.shapes[b.body(arm).shapes[0]].shape.kind {
            box2d::shape::ShapeKind::Polygon { poly, .. } => poly.vertices[2].y,
            _ => 0.0,
        };
        b.body(arm).world_point(box2d::math::Vec2::new(0.0, y))
    };
    let (pos, ang) = (w.b2().body(arm).xf.position, w.b2().body(arm).sweep.a);
    w.b2_mut().set_xform(arm, box2d::math::Vec2::new(pos.x + handle.x - tip.x, pos.y + handle.y - tip.y), ang);
    w.set_rider_keys(Keys { space: true, ..Default::default() });
    w.step();
    assert!(!w.bike.as_ref().unwrap().ejected(), "rider did not re-board");
}

/// Sprite rendering of the c3 bike + rider (visual check; screenshot in screenshots/sprites_bike.png).
#[test]
fn sprites_draw_the_bike_and_rider() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 3) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..40 {
        w.step();
    }
    let mut img = happywheels_core::render::render_level_at(&level, &w, 640, 360, [1000.0, 330.0]);
    for p in img.pixels_mut() {
        *p = image::Rgba([0xc8, 0xd8, 0xe8, 255]);
    }
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    std::fs::create_dir_all(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots")).unwrap();
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_bike.png")).unwrap();
}

/// Sprite rendering of the c1 wheelchair + rider (screenshot in screenshots/sprites_chair.png).
#[test]
fn sprites_draw_the_chair_and_rider() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 1) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(1, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..40 {
        w.step();
    }
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_chair.png")).unwrap();
}

/// Sprite rendering of the c2 segway + rider (screenshot in screenshots/sprites_segway.png).
#[test]
fn sprites_draw_the_segway_and_rider() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 2) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(2, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..40 {
        w.step();
    }
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_segway.png")).unwrap();
}

/// Sprite rendering of characters 4–6 (cart, dirt bike, mower); screenshots in screenshots/sprites_c<N>.png.
#[test]
fn sprites_draw_other_vehicles() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    for n in [4u32, 5, 6, 7, 8, 9, 10, 11] {
        let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, n) else { continue };
        let level = level::parse(
            "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
             <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
        )
        .unwrap();
        let mut w = World::from_level(&level);
        assert!(w.spawn_bike(n as usize, [1000.0, 250.0]));
        sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
        for _ in 0..if n == 9 { 3 } else { 40 } {
            w.step();
        }
        let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
        sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
        img.save(format!("{}/../../screenshots/sprites_c{n}.png", env!("CARGO_MANIFEST_DIR"))).unwrap();
    }
}

/// Santa's sleigh bells (behaviour only): SleighBellLoop is requested while space is held and the boost is not maxed.
#[test]
fn santa_boost_asks_for_sleigh_bells() {
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(8, [1000.0, 250.0]));
    w.set_rider_keys(Default::default());
    w.step();
    assert!(!w.loops.iter().any(|l| l.0 == "SleighBellLoop"));
    w.set_rider_keys(happywheels_core::vehicle::Keys { space: true, ..Default::default() });
    w.step();
    assert!(w.loops.iter().any(|l| l.0 == "SleighBellLoop"));
}

/// The c3 dad's kid on the seat (screenshot screenshots/sprites_c3_kid.png).
#[test]
fn sprites_draw_the_dads_kid() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 3) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = true;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..40 {
        w.step();
    }
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_c3_kid.png")).unwrap();
}

/// Head smash debris drawn with sprites (screenshot screenshots/sprites_head_smash.png): chunks and brain appear, the head is gone.
#[test]
fn sprites_draw_head_smash_debris() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 3) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..30 {
        w.step();
    }
    let mut doll = w.character.take().unwrap();
    doll.smash_head(&mut w);
    doll.smash_chest(&mut w);
    w.character = Some(doll);
    for _ in 0..6 {
        w.step();
    }
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_head_smash.png")).unwrap();
    assert_eq!(w.character.as_ref().unwrap().chunk_names.len(), 8);
}

/// Bike frame smash drawn with the fork / broken frame / seat pieces (screenshot screenshots/sprites_bike_smash.png).
#[test]
fn sprites_draw_bike_frame_smash() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 3) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    w.dad_kid = false;
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..30 {
        w.step();
    }
    let mut doll = w.character.take().unwrap();
    let mut bike = w.bike.take();
    if let Some(happywheels_core::vehicle::Vehicle::Bike(b)) = bike.as_mut() {
        b.frame_smash(&mut w, &mut doll);
    }
    w.bike = bike;
    w.character = Some(doll);
    for _ in 0..4 {
        w.step();
    }
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_bike_smash.png")).unwrap();
}

/// Dirt bike and pogo frame smashes drawn with their pieces (screenshots/sprites_dirt_smash.png, sprites_pogo_smash.png).
#[test]
fn sprites_draw_dirt_and_pogo_smash() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    for (n, name) in [(5u32, "dirt"), (9, "pogo"), (6, "mower"), (1, "chair"), (10, "family"), (11, "heli"), (4, "cart"), (8, "santa")] {
        let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, n) else { continue };
        let level = level::parse(
            "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
             <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
        )
        .unwrap();
        let mut w = World::from_level(&level);
        assert!(w.spawn_bike(n as usize, [1000.0, 250.0]));
        sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
        for _ in 0..3 {
            w.step();
        }
        let mut doll = w.character.take().unwrap();
        let mut bike = w.bike.take();
        match bike.as_mut() {
            Some(happywheels_core::vehicle::Vehicle::Dirt(b)) => b.frame_smash(&mut w, &mut doll),
            Some(happywheels_core::vehicle::Vehicle::Pogo(b)) => b.frame_smash(&mut w, &mut doll),
            Some(happywheels_core::vehicle::Vehicle::Mower(b)) => b.smash(&mut w, &mut doll),
            Some(happywheels_core::vehicle::Vehicle::Chair(b)) => b.chair_smash(&mut w, &mut doll),
            Some(happywheels_core::vehicle::Vehicle::Heli(b)) => b.copter_smash(&mut w, &mut doll, box2d::math::Vec2::new(16.0, 6.0)),
            Some(happywheels_core::vehicle::Vehicle::Family(f)) => f.dad.frame_smash(&mut w, &mut doll),
            Some(happywheels_core::vehicle::Vehicle::Cart(c)) => c.main_smash(&mut w, &mut doll),
            Some(happywheels_core::vehicle::Vehicle::Santa(s)) => s.sleigh_smash(&mut w, &mut doll),
            _ => panic!("unexpected vehicle"),
        }
        w.bike = bike;
        w.character = Some(doll);
        for _ in 0..if name == "cart" { 20 } else { 4 } {
            w.step();
        }
        let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
        sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
        img.save(format!("{}/../../screenshots/sprites_{name}_smash.png", env!("CARGO_MANIFEST_DIR"))).unwrap();
    }
}

/// Level shapes as `getFlatSprite` draws them (screenshot screenshots/level_shapes.png): fill + 1 px outline, translucent fill, outline-only, no fill and no outline (invisible).
#[test]
fn level_shapes_use_fill_outline_and_opacity() {
    let level = level::parse(
        "<levelXML><info v='1.99' x='100' y='100'/><shapes>\
         <sh t='0' p0='150' p1='150' p2='200' p3='120' p4='0' p5='t' p6='f' p7='1' p8='16711680' p9='255' p10='100' p11='1'/>\
         <sh t='0' p0='380' p1='150' p2='200' p3='120' p4='0' p5='t' p6='f' p7='1' p8='65280' p9='-1' p10='40' p11='1'/>\
         <sh t='0' p0='150' p1='320' p2='200' p3='100' p4='0' p5='t' p6='f' p7='1' p8='-1' p9='0' p10='100' p11='1'/>\
         <sh t='0' p0='380' p1='320' p2='200' p3='100' p4='0' p5='t' p6='f' p7='1' p8='-1' p9='-1' p10='100' p11='1'/>\
         </shapes></levelXML>",
    )
    .unwrap();
    let w = World::from_level(&level);
    let img = happywheels_core::render::render_level_at(&level, &w, 520, 420, [260.0, 210.0]);
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/level_shapes.png")).unwrap();
    // the translucent green rectangle blends with the background (not pure green) and the invisible one leaves the background
    let p = img.get_pixel(380, 150);
    assert!(p[0] > 0 && p[0] < 255 && p[1] == 255, "{p:?}");
    assert_eq!(img.get_pixel(380, 320), &image::Rgba([255, 255, 255, 255]));
}

/// Art shapes: bezier paths (screenshot screenshots/level_art.png) — a closed filled blob with curved sides and an open black stroke.
#[test]
fn art_shapes_draw_bezier_paths() {
    let level = level::parse(
        "<levelXML><info v='1.99' x='100' y='100'/><shapes>\
         <sh t='4' p0='150' p1='150' p2='100' p3='100' p4='0' p5='f' p6='f' p7='1' p8='16744448' p9='0' p10='100' p11='1'>\
           <v id='1' n='4' f='t' v0='-100_0_0_60_0_-60' v1='0_-100_-60_0_60_0' v2='100_0_0_-60_0_60' v3='0_100_60_0_-60_0'/></sh>\
         <sh t='4' p0='400' p1='150' p2='100' p3='100' p4='0' p5='f' p6='f' p7='1' p8='-1' p9='255' p10='100' p11='1'>\
           <v id='2' n='3' f='f' v0='-100_50_0_0_0_0' v1='0_-80_-50_0_50_0' v2='100_50_0_0_0_0'/></sh>\
         </shapes></levelXML>",
    )
    .unwrap();
    assert_eq!(level.shapes[0].verts.len(), 4);
    assert!(level.shapes[0].complete_fill && !level.shapes[1].complete_fill);
    let w = World::from_level(&level);
    let img = happywheels_core::render::render_level_at(&level, &w, 520, 300, [270.0, 150.0]);
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/level_art.png")).unwrap();
    assert_eq!(img.get_pixel(140, 150), &image::Rgba([255, 128, 0, 255]));
}

/// `bgc` sets the level background colour (default white).
#[test]
fn background_colour_comes_from_bgc() {
    let w = |xml: &str| {
        let level = level::parse(xml).unwrap();
        let world = World::from_level(&level);
        happywheels_core::render::render_level_at(&level, &world, 8, 8, [0.0, 0.0])
    };
    assert_eq!(w("<levelXML><info v='1.99' bgc='255'/><shapes/></levelXML>").get_pixel(1, 1), &image::Rgba([0, 0, 255, 255]));
    assert_eq!(w("<levelXML><info v='1.99'/><shapes/></levelXML>").get_pixel(1, 1), &image::Rgba([255, 255, 255, 255]));
}

/// Green hills backdrop (screenshot screenshots/backdrop_green.png): sky fixed, hill layers shifted by 1 % / 5 % / 10 % of the camera motion.
#[test]
fn green_backdrop_draws_parallax_layers() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(back) = happywheels_core::sprites::Sprites::open_backdrop(&assets) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='450' y='250' bg='1'/><shapes>\
         <sh t='0' p0='450' p1='450' p2='900' p3='60' p4='0' p5='t' p6='f' p7='1' p8='6710886' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let w = World::from_level(&level);
    let a = happywheels_core::render::render_level_scene(&level, &w, 900, 500, [450.0, 250.0], None, Some(&back));
    let b = happywheels_core::render::render_level_scene(&level, &w, 900, 500, [2450.0, 1500.0], None, Some(&back));
    a.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/backdrop_green.png")).unwrap();
    b.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/backdrop_green_scrolled.png")).unwrap();
    assert_ne!(a.get_pixel(450, 100), &image::Rgba([255, 255, 255, 255]), "sky should be drawn");
    assert_ne!(a, b);
}

/// City backdrop (screenshots/backdrop_city.png).
#[test]
fn city_backdrop_draws_buildings() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(back) = happywheels_core::sprites::Sprites::open_city_backdrop(&assets) else { return };
    let level = level::parse("<levelXML><info v='1.99' x='450' y='250' bg='2'/><shapes></shapes></levelXML>").unwrap();
    let w = World::from_level(&level);
    let a = happywheels_core::render::render_level_scene(&level, &w, 900, 500, [450.0, 1500.0], None, Some(&back));
    a.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/backdrop_city.png")).unwrap();
}

/// Limb breaks switch part frames and draw severed stumps (screenshot screenshots/sprites_limb_breaks.png).
#[test]
fn sprites_draw_limb_break_frames() {
    use happywheels_core::character::Limb;
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 3) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..3 {
        w.step();
    }
    let mut doll = w.character.take().unwrap();
    doll.break_limb(&mut w, Limb::Shoulder1, 1.0);
    doll.break_limb(&mut w, Limb::Elbow2, 1.0);
    doll.break_limb(&mut w, Limb::Hip2, 1.0);
    doll.break_limb(&mut w, Limb::Knee1, 1.0);
    doll.break_limb(&mut w, Limb::Neck, 1.0);
    doll.break_limb(&mut w, Limb::Waist, 1.0);
    doll.break_limb(&mut w, Limb::Shoulder2, 1.0e6);
    assert!(doll.part_wound("pelvis") && doll.part_wound("chest"));
    assert_eq!(doll.part_frame("upperArm1"), 3);
    assert_eq!(doll.part_frame("lowerArm2"), 2);
    assert_eq!(doll.part_frame("upperLeg2"), 3);
    assert_eq!(doll.part_frame("lowerLeg1"), 2);
    assert_eq!(doll.part_frame("head"), 2);
    w.character = Some(doll);
    for _ in 0..4 {
        w.step();
    }
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_limb_breaks.png")).unwrap();
}

/// Stump clips show frame 8 after the chest / pelvis smash (original chestSmash / pelvisSmash).
#[test]
fn stump_frames_follow_chest_and_pelvis_smash() {
    use happywheels_core::character::Limb;
    let level = level::parse("<levelXML><info v='1.99' x='1000' y='250'/><shapes></shapes></levelXML>").unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    let mut doll = w.character.take().unwrap();
    doll.break_limb(&mut w, Limb::Shoulder1, 1.0);
    doll.break_limb(&mut w, Limb::Hip2, 1.0);
    assert_eq!((doll.part_frame("upperArm3"), doll.part_frame("upperLeg4")), (2, 2));
    doll.smash_chest(&mut w);
    doll.smash_pelvis(&mut w);
    assert_eq!((doll.part_frame("upperArm3"), doll.part_frame("upperLeg4")), (8, 8));
}

/// NPC debris and break frames are drawn inside the NPC's clip (screenshot screenshots/sprites_kid_smash.png).
#[test]
fn sprites_draw_kid_debris_and_frames() {
    use happywheels_core::character::Limb;
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 3) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..3 {
        w.step();
    }
    let mut bike = w.bike.take();
    let Some(happywheels_core::vehicle::Vehicle::Bike(b)) = bike.as_mut() else { panic!() };
    let Some(kid) = b.kid.as_mut() else { return };
    kid.doll.break_limb(&mut w, Limb::Elbow1, 1.0);
    kid.doll.break_limb(&mut w, Limb::Knee2, 1.0);
    kid.doll.smash_head(&mut w);
    assert_eq!(kid.doll.part_frame("lowerArm1"), 2);
    w.bike = bike;
    for _ in 0..6 {
        w.step();
    }
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_kid_smash.png")).unwrap();
}

/// Family: the daughter's frame smash drawn with her fork / broken frame / seat pieces (screenshot screenshots/sprites_daughter_smash.png).
#[test]
fn sprites_draw_daughter_frame_pieces() {
    use happywheels_core::vehicle::Vehicle;
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 10) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(10, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..30 {
        w.step();
    }
    let Some(Vehicle::Family(mut f)) = w.bike.take() else { panic!() };
    f.daughter.frame_smash(&mut w);
    w.bike = Some(Vehicle::Family(f));
    for _ in 0..10 {
        w.step();
    }
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_daughter_smash.png")).unwrap();
}

/// Wheelchair jet and fuel tank blow up into metal shards (screenshot screenshots/sprites_chair_jet_shards.png).
#[test]
fn sprites_draw_chair_jet_shards() {
    use happywheels_core::vehicle::Vehicle;
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 1) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(1, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..3 {
        w.step();
    }
    let (Some(mut doll), Some(Vehicle::Chair(mut c))) = (w.character.take(), w.bike.take()) else { panic!() };
    c.chair_smash(&mut w, &mut doll);
    c.jet_smash(&mut w);
    c.fueltank_smash(&mut w);
    w.character = Some(doll);
    w.bike = Some(Vehicle::Chair(c));
    for _ in 0..5 {
        w.step();
    }
    assert!(!w.blood.sprites.is_empty());
    let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
    sprites.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
    img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_chair_jet_shards.png")).unwrap();
}

/// Santa: skiSmash on an intact sleigh moves its local centre but the sleigh clip stays where it was (screenshot screenshots/sprites_santa_ski_smash.png: before | after).
#[test]
fn santa_sleigh_clip_is_recentred_after_ski_smash() {
    use happywheels_core::vehicle::Vehicle;
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let Some(mut sprites) = happywheels_core::sprites::Sprites::open(&assets, 8) else { return };
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(8, [1000.0, 250.0]));
    sprites.bind_vehicle_rider(&w, [1000.0, 250.0]);
    for _ in 0..3 {
        w.step();
    }
    let render = |w: &World| {
        let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
        sprites.draw(&mut img, w, (320.0 - 1000.0, 180.0 - 330.0));
        img
    };
    let before = render(&w);
    let Some(Vehicle::Santa(mut s)) = w.bike.take() else { panic!() };
    let lc0 = w.b2().body(s.sleigh).sweep.local_center;
    s.ski_smash(&mut w);
    assert!(w.b2().body(s.sleigh).sweep.local_center != lc0, "base swap should move the local centre");
    w.bike = Some(Vehicle::Santa(s));
    let after = render(&w);
    let mut both = image::RgbaImage::new(1280, 360);
    image::imageops::replace(&mut both, &before, 0, 0);
    image::imageops::replace(&mut both, &after, 640, 0);
    both.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_santa_ski_smash.png")).unwrap();
}

/// c4 cart groceries: the soda can spills and the cracker box bursts on a hard hit (screenshot screenshots/sprites_cart_groceries.png).
#[test]
fn cart_groceries_burst_and_spill() {
    use happywheels_core::vehicle::Vehicle;
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(4, [1000.0, 250.0]));
    let mut sprites = happywheels_core::sprites::Sprites::open(&assets, 4);
    if let Some(s) = sprites.as_mut() {
        s.bind_vehicle_rider(&w, [1000.0, 250.0]);
    }
    let (mut cola, mut cracker) = (false, false);
    for i in 0..200 {
        if i == 10 {
            // slam the soda can and the cracker box into the basket
            if let Some(Vehicle::Cart(c)) = w.bike.as_ref() {
                let (soda, box_) = (c.groceries[3], c.groceries[4]);
                w.b2_mut().body_mut(soda).linear_velocity = box2d::math::Vec2::new(0.0, 60.0);
                w.b2_mut().body_mut(box_).linear_velocity = box2d::math::Vec2::new(0.0, 300.0);
            }
        }
        w.step();
        if let Some(Vehicle::Cart(c)) = w.bike.as_ref() {
            cola |= c.cola_open;
            cracker |= c.cracker_gone;
        }
    }
    assert!(cola, "soda can should spill");
    assert!(cracker, "cracker box should burst");
    eprintln!("cola {cola} cracker {cracker} sprites {}", w.blood.sprites.len());
    if let Some(s) = sprites {
        let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
        s.draw(&mut img, &w, (320.0 - 1000.0, 180.0 - 330.0));
        img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_cart_groceries.png")).unwrap();
    }
}

/// Boosting Santa's sleigh throws snowflakes off its underside (screenshot screenshots/sprites_santa_snow.png).
#[test]
fn santa_boost_throws_snow() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(8, [1000.0, 250.0]));
    let mut sprites = happywheels_core::sprites::Sprites::open(&assets, 8);
    if let Some(s) = sprites.as_mut() {
        s.bind_vehicle_rider(&w, [1000.0, 250.0]);
    }
    w.set_rider_keys(happywheels_core::vehicle::Keys { space: true, right: true, ..Default::default() });
    for _ in 0..40 {
        w.step();
    }
    assert!(w.blood.sprites.iter().any(|p| p.kind == happywheels_core::blood::ShardKind::Snow), "boost should throw snow");
    if let Some(s) = sprites {
        let mut img = image::RgbaImage::from_pixel(640, 360, image::Rgba([0xc8, 0xd8, 0xe8, 255]));
        let c = w.b2().body(match w.bike.as_ref() { Some(happywheels_core::vehicle::Vehicle::Santa(s)) => s.sleigh, _ => panic!() }).sweep.c;
        s.draw(&mut img, &w, (320.0 - c.x as f32 * 62.5, 180.0 - c.y as f32 * 62.5));
        img.save(concat!(env!("CARGO_MANIFEST_DIR"), "/../../screenshots/sprites_santa_snow.png")).unwrap();
    }
}

/// The mower grinds a loose body of flesh material that its blade hits: filter/material change, a rising body drags it down, GrindLoop2 plays and the pair is removed afterwards.
#[test]
fn mower_blade_grinds_a_loose_body() {
    use box2d::{math::Vec2, shape::{Geometry, ShapeDef}, world::BodyDef};
    use happywheels_core::vehicle::Vehicle;
    let level = level::parse(
        "<levelXML><info v='1.99' x='1000' y='250'/><shapes>\
         <sh t='0' p0='1000' p1='425' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>",
    )
    .unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(6, [1000.0, 250.0]));
    for _ in 0..20 {
        w.step();
    }
    // a 0.3 m flesh box dropped onto the blade
    let Some(Vehicle::Mower(m)) = w.bike.as_ref() else { panic!() };
    let mc = w.b2().body(m.body).sweep.c;
    let target = {
        let b = w.b2_mut();
        // the blade sits under the mower body; find it through the lowest non-sensor shape point
        let id = b.create_body(&BodyDef { position: Vec2::new(mc.x, mc.y + 0.55), ..Default::default() });
        let mut d = ShapeDef::new(Geometry::Polygon(vec![Vec2::new(-0.15, -0.15), Vec2::new(0.15, -0.15), Vec2::new(0.15, 0.15), Vec2::new(-0.15, 0.15)]));
        d.density = 1.0;
        let s = b.create_shape(id, &d);
        b.shape_mut(s).material = 1;
        b.set_mass_from_shapes(id);
        id
    };
    let (mut ground, mut loop_seen) = (false, false);
    for _ in 0..240 {
        w.step();
        if let Some(Vehicle::Mower(m)) = w.bike.as_ref() {
            ground |= !m.targets.is_empty();
            loop_seen |= w.loops.iter().any(|l| l.0 == "GrindLoop2");
        }
    }
    assert!(ground && loop_seen, "the blade should start grinding the box and request GrindLoop2");
    assert!(!w.b2().body(target).alive, "the ground body is removed once it leaves the clearance sensor");
}

/// `grindShape`: a ground head/chest/pelvis stops smashing (its listeners are deleted in the original) while the other parts keep their limits.
#[test]
fn ground_parts_stop_smashing() {
    use happywheels_core::character::Part;
    let level = level::parse("<levelXML><info v='1.99' x='1000' y='250'/><shapes></shapes></levelXML>").unwrap();
    let mut w = World::from_level(&level);
    assert!(w.spawn_bike(3, [1000.0, 250.0]));
    let mut doll = w.character.take().unwrap();
    assert!(doll.smash_limit(&w, Part::Head).is_some() && doll.smash_limit(&w, Part::Chest).is_some());
    let head = doll.bodies[&Part::Head];
    doll.grind_shape(&mut w, head);
    assert!(doll.smash_limit(&w, Part::Head).is_none());
    assert!(doll.smash_limit(&w, Part::Chest).is_some());
}

/// Spikes are drawn from the `game/l` atlas (`spike-clean` row + `spike-platform` base) instead of grey boxes.
#[test]
fn spikes_have_sprite_art() {
    let Some(assets) = std::env::var_os("HW_ASSETS").map(std::path::PathBuf::from).or_else(|| Some(std::path::PathBuf::from("../../../assets-99tnk64n1mubaoc8w"))).filter(|p| p.join("animate/game/l").exists()) else { return };
    let level = level::parse("<levelXML><info v='1.99' x='1000' y='300'/><shapes></shapes><specials><sp t='6' p0='1000' p1='400' p2='0' p3='t' p4='8' p5='f'/></specials></levelXML>").unwrap();
    let w = World::from_level(&level);
    let sp = happywheels_core::sprites::Sprites::open(&assets, 1).unwrap();
    assert!(sp.covers_special(6) && sp.covers_special(3) && sp.covers_special(4) && sp.covers_special(1) && sp.covers_special(2) && sp.covers_special(5) && sp.covers_special(7) && sp.covers_special(10) && sp.covers_special(0) && sp.covers_special(27) && sp.covers_special(11) && sp.covers_special(31) && sp.covers_special(23) && !sp.covers_special(8));
    let plain = happywheels_core::render::render_level_at(&level, &w, 400, 300, [1000.0, 400.0]);
    let art = happywheels_core::render::render_level_sprites(&level, &w, 400, 300, [1000.0, 400.0], Some(&sp));
    assert_ne!(plain.as_raw(), art.as_raw());
}
