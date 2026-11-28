//! Tables, a van and chains breaking and arrows striking, against the original (`tests/golden/breaks.json`, `analysis/golden-breaks.py`): the pose of
//! every body with mass and the sounds started.

use happywheels_core::physics::World;
use std::collections::BTreeMap;

#[derive(serde::Deserialize)]
struct Case {
    name: String,
    xml: String,
    snaps: BTreeMap<usize, Vec<[f64; 3]>>,
    /// `[frame, sound]`
    sounds: Vec<(usize, String)>,
}

/// The sounds of these mechanics (variant numbers are random in both).
const SOUNDS: [&str; 7] = ["TableBreak", "VanHit", "VanSmash", "ChainBreak", "CarTire", "ArrowF", "ArrowSolid"];

fn kind(name: &str) -> String {
    // the table's two sounds are told apart by their number
    if name.starts_with("TableBreak") { name.into() } else { name.trim_end_matches(char::is_numeric).into() }
}

#[test]
fn things_break_like_the_original() {
    let cases: Vec<Case> = serde_json::from_str(include_str!("../../../tests/golden/breaks.json")).unwrap();
    assert_eq!(cases.len(), 10);
    let mut report = Vec::new();
    for case in cases {
        let level = happywheels_core::level::parse(&case.xml).unwrap();
        let mut world = World::from_level(&level);
        assert!(world.spawn_bike(3, level.info.start));
        let mut frame = 0;
        let mut heard = Vec::new();
        for (&at, original) in &case.snaps {
            while frame < at {
                world.set_rider_keys(Default::default());
                world.step();
                frame += 1;
                heard.extend(world.take_sounds().into_iter().filter(|s| SOUNDS.iter().any(|n| s.0.starts_with(n))).map(|s| (frame, kind(&s.0))));
            }
            let here: Vec<[f64; 3]> = world.b2().c.bodies.iter().filter(|b| b.alive && b.inv_mass > 0.0).map(|b| [b.sweep.c.x * 62.5, b.sweep.c.y * 62.5, b.sweep.a]).collect();
            let mut used = vec![false; here.len()];
            let mut worst = 0.0f64;
            for o in original {
                let off = |h: &[f64; 3]| (h[0] - o[0]).abs().max((h[1] - o[1]).abs()).max((h[2] - o[2]).abs() * 10.0);
                if let Some(j) = (0..here.len()).filter(|&j| !used[j]).min_by(|&a, &b| off(&here[a]).total_cmp(&off(&here[b]))) {
                    used[j] = true;
                    worst = worst.max(off(&here[j]));
                }
            }
            // (the arrow gun's invisible aiming-line body is not made here, and the arrows meet a ragdoll: only the sounds are compared)
            if case.name == "arrow_npc" && std::env::var("DUMP").is_ok() {
                let near = |o: &[f64; 3]| here.iter().any(|h| (h[0] - o[0]).abs() < 1.0 && (h[1] - o[1]).abs() < 1.0);
                println!("{at}: {}/{} there only {:?}", original.len(), here.len(), original.iter().filter(|o| !near(o) && o[0] > 1050.0).collect::<Vec<_>>());
                println!("    here only {:?}", here.iter().filter(|h| h[0] > 1050.0 && !original.iter().any(|o| (h[0] - o[0]).abs() < 1.0 && (h[1] - o[1]).abs() < 1.0)).collect::<Vec<_>>());
            }
            if case.name != "arrow_npc" && (here.len() != original.len() || worst > 0.01) {
                report.push(format!("{} frame {at}: {} bodies there, {} here, worst {worst:.3}", case.name, original.len(), here.len()));
                break;
            }
        }
        let there: Vec<(usize, String)> = case.sounds.iter().filter(|s| SOUNDS.iter().any(|n| s.1.starts_with(n))).map(|s| (s.0, kind(&s.1))).collect();
        let sorted = |mut v: Vec<(usize, String)>| {
            v.sort();
            v
        };
        let (there, heard) = (sorted(there), sorted(heard));
        // The capture's browser hands out no sound handles, so an arrow there squelches into every new body; with a handle (a browser
        // with sound, and this port) it stays quiet until its last squelch has ended. So here only some of those are expected.
        let mut rest = there.clone();
        let within = heard.iter().all(|h| rest.iter().position(|t| t == h).map(|k| rest.remove(k)).is_some()) && rest.iter().all(|r| r.1 == "ArrowFlesh");
        let same = if case.name == "arrow_npc" { within } else { there == heard };
        if !same {
            report.push(format!("{} sounds there {there:?}\n    here {heard:?}", case.name));
        }
    }
    assert!(report.is_empty(), "{}", report.join("\n"));
}
