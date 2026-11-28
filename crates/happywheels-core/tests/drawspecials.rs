//! What is drawn for each special object with default properties, after one step, against the original's display list
//! (`tests/golden/draw-specials.json`, `analysis/golden-specials.py`). Needs the original assets.

use happywheels_core::{physics::World, render::render_level_view, sprites::Sprites};

type Drawn = (String, [[f64; 2]; 3]);
/// `[texture, top left, top right, bottom left]`
type Listed = (String, [f64; 2], [f64; 2], [f64; 2]);

#[derive(serde::Deserialize)]
struct Case {
    kind: usize,
    name: String,
    xml: String,
    list: Option<Vec<Listed>>,
}

fn drawn(assets: &std::path::Path, xml: &str, steps: usize) -> Vec<Drawn> {
    let level = happywheels_core::level::parse(xml).unwrap();
    let mut world = World::from_level(&level);
    assert!(world.spawn_bike(3, level.info.start));
    let mut sprites = Sprites::open(assets, 3).unwrap();
    sprites.bind_vehicle_rider(&world, level.info.start);
    world.set_rider_keys(Default::default());
    // the fan's blades are a looping clip (6 frames, driven by the step counter here, by the display there): compare frame 1
    for _ in 0..steps {
        world.step();
    }
    *sprites.trace.borrow_mut() = Some(Vec::new());
    let camera = [1300.0, 250.0];
    render_level_view(&level, &world, 900, 500, 1.0, camera, Some(&sprites), None);
    // screen -> level px
    let list = sprites.trace.borrow_mut().take().unwrap();
    list.into_iter().filter(|d| !d.0.starts_with("line ")).map(|(n, q)| (n, q.map(|p| [p[0] + f64::from(camera[0]) - 450.0, p[1] + f64::from(camera[1]) - 250.0]))).collect()
}

/// Still different: the finish line's posts are drawn once, not twice (9); the homing mine's jets and light are not animated (25); chain links are layered
/// otherwise (30).
const KNOWN: [usize; 3] = [9, 25, 30];

#[test]
fn specials_are_drawn_like_the_original() {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    if !assets.is_dir() {
        return;
    }
    let cases: Vec<Case> = serde_json::from_str(include_str!("../../../tests/golden/draw-specials.json")).unwrap();
    assert_eq!(cases.len(), 36);
    let mut report = Vec::new();
    let mut differing = Vec::new();
    for case in cases {
        let Some(original) = case.list else { continue };
        let original: Vec<Drawn> = original.into_iter().map(|(n, a, b, c)| (n, [a, b, c])).collect();
        // as in the capture: the rider is there (turrets aim at them) and what an empty level shows is taken away
        let steps = if case.kind == 8 { 6 } else { 1 };
        let mut port = drawn(&assets, &case.xml, steps);
        let empty = case.xml.split("<specials>").next().unwrap().to_string() + "</levelXML>";
        for (name, quad) in drawn(&assets, &empty, steps) {
            if let Some(j) = port.iter().position(|(n, q)| *n == name && q.iter().flatten().zip(quad.iter().flatten()).all(|(a, b)| (a - b).abs() < 0.6)) {
                port.remove(j);
            }
        }
        let off = |a: &[[f64; 2]; 3], b: &[[f64; 2]; 3]| a.iter().flatten().zip(b.iter().flatten()).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
        if std::env::var("DUMP").is_ok_and(|d| d.split(',').any(|k| k == case.kind.to_string())) {
            for (side, list) in [("original", &original), ("here", &port)] {
                for (n, q) in list {
                    println!("{} {side}: {n} [{:.1},{:.1}] [{:.1},{:.1}] [{:.1},{:.1}]", case.kind, q[0][0], q[0][1], q[1][0], q[1][1], q[2][0], q[2][1]);
                }
            }
        }
        let mut used = vec![false; port.len()];
        let mut wrong = Vec::new();
        let mut previous = None;
        // `?` and `pixiid_*` are generated textures (blood bitmaps, timer texts), named only there
        for (name, quad) in original.iter().filter(|d| d.0 != "?" && !d.0.starts_with("pixiid_")) {
            let nearest = (0..port.len()).filter(|&j| !used[j] && &port[j].0 == name).min_by(|&a, &b| off(&port[a].1, quad).total_cmp(&off(&port[b].1, quad)));
            match nearest {
                Some(j) => {
                    used[j] = true;
                    // the jet's flame length is random there (.95·accel + .1·random of 63 px)
                    if off(&port[j].1, quad) > if name == "jet-flame.png" { 4.0 } else { 1.0 } {
                        wrong.push(format!("{name} {:.1} px off", off(&port[j].1, quad)));
                    }
                    if previous.is_some_and(|p| p > j) {
                        wrong.push(format!("{name} in the wrong layer"));
                    }
                    previous = Some(j);
                }
                None => wrong.push(format!("{name} missing at {:?}", quad[0])),
            }
        }
        let extra: Vec<_> = port.iter().zip(&used).filter(|(_, &u)| !u).map(|(d, _)| d.0.as_str()).collect();
        if !extra.is_empty() {
            wrong.push(format!("only here: {} {:?}", extra.len(), &extra[..extra.len().min(5)]));
        }
        if !wrong.is_empty() {
            wrong.truncate(5);
            differing.push(case.kind);
            report.push(format!("{} {} ({} drawn there): {}", case.kind, case.name, original.len(), wrong.join("; ")));
        }
    }
    assert_eq!(differing, KNOWN, "{}", report.join("\n"));
}
