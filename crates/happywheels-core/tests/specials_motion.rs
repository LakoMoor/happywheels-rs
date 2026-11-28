//! How the bodies of a level with one special object move, against the original (`tests/golden/specials-motion.json`,
//! `analysis/golden-specials-motion.py`): the rider (character 3, no keys) is there as in the capture.

use happywheels_core::physics::World;
use std::collections::BTreeMap;

#[derive(serde::Deserialize)]
struct Case {
    kind: usize,
    name: String,
    xml: String,
    /// Frame -> `[x, y, angle]` of every body with mass.
    snaps: BTreeMap<usize, Vec<[f64; 3]>>,
}

/// Still different, with the first frame that is: once the homing mine has torn the rider apart (frame 46) the pieces drift from
/// frame 48 (25).
const KNOWN: [(usize, usize); 1] = [(25, 60)];

#[test]
fn specials_move_like_the_original() {
    let cases: Vec<Case> = serde_json::from_str(include_str!("../../../tests/golden/specials-motion.json")).unwrap();
    assert_eq!(cases.len(), 36);
    let mut report = Vec::new();
    let mut differing = Vec::new();
    for case in cases {
        let level = happywheels_core::level::parse(&case.xml).unwrap();
        let mut world = World::from_level(&level);
        assert!(world.spawn_bike(3, level.info.start));
        let mut frame = 0;
        for (&at, original) in &case.snaps {
            while frame < at {
                world.set_rider_keys(Default::default());
                world.step();
                frame += 1;
            }
            let here: Vec<[f64; 3]> = world.b2().c.bodies.iter().filter(|b| b.alive && b.inv_mass > 0.0).map(|b| [b.sweep.c.x * 62.5, b.sweep.c.y * 62.5, b.sweep.a]).collect();
            // bodies have no common names: pair each of the original's with the nearest one here
            let mut used = vec![false; here.len()];
            let mut worst = 0.0f64;
            for o in original {
                let off = |h: &[f64; 3]| (h[0] - o[0]).abs().max((h[1] - o[1]).abs()).max((h[2] - o[2]).abs() * 10.0);
                match (0..here.len()).filter(|&j| !used[j]).min_by(|&a, &b| off(&here[a]).total_cmp(&off(&here[b]))) {
                    Some(j) => {
                        used[j] = true;
                        worst = worst.max(off(&here[j]));
                    }
                    None => worst = f64::INFINITY,
                }
            }
            if std::env::var("DUMP").is_ok_and(|d| d.split(',').any(|k| k == format!("{}:{at}", case.kind))) {
                let near = |a: &[f64; 3], b: &[f64; 3]| (a[0] - b[0]).abs() < 0.01 && (a[1] - b[1]).abs() < 0.01 && (a[2] - b[2]).abs() < 0.001;
                println!("{} there only: {:?}", case.kind, original.iter().filter(|o| !here.iter().any(|h| near(h, o))).collect::<Vec<_>>());
                println!("{} here only: {:?}", case.kind, here.iter().filter(|h| !original.iter().any(|o| near(h, o))).collect::<Vec<_>>());
            }
            if here.len() != original.len() || worst > 0.01 {
                differing.push((case.kind, at));
                report.push(format!("{} {} frame {at}: {} bodies there, {} here, worst {worst:.3}", case.kind, case.name, original.len(), here.len()));
                break;
            }
        }
    }
    assert_eq!(differing, KNOWN, "{}", report.join("\n"));
}
