//! Integration harness over tests/levels/*.xml. Golden snapshots from the ORIGINAL are not
//! available yet (see docs/PORT.md); until then these guard the properties a golden diff needs:
//! every level parses, and the simulation is bit-for-bit deterministic across runs.

use std::path::PathBuf;

use happywheels_core::{level, physics::World};

fn level_files() -> Vec<PathBuf> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/levels");
    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "xml"))
        .collect();
    files.sort();
    files
}

fn run(xml: &str, steps: usize) -> Vec<Option<[f32; 2]>> {
    let level = level::parse(xml).unwrap();
    let mut world = World::from_level(&level);
    for _ in 0..steps {
        world.step();
    }
    (0..level.shapes.len()).map(|i| world.position_px(i)).collect()
}

#[test]
fn sample_levels_exist_and_parse() {
    let files = level_files();
    assert!(files.len() >= 2, "expected sample levels, found {files:?}");
    for f in files {
        let xml = std::fs::read_to_string(&f).unwrap();
        level::parse(&xml).unwrap_or_else(|e| panic!("{}: {e:#}", f.display()));
    }
}

#[test]
fn simulation_is_deterministic() {
    for f in level_files() {
        let xml = std::fs::read_to_string(&f).unwrap();
        let (a, b) = (run(&xml, 120), run(&xml, 120));
        assert_eq!(a, b, "{} is not deterministic", f.display());
    }
}

#[test]
fn nothing_explodes_or_falls_out_of_the_world() {
    for f in level_files() {
        let xml = std::fs::read_to_string(&f).unwrap();
        for (i, p) in run(&xml, 300).into_iter().enumerate() {
            if let Some([x, y]) = p {
                assert!(x.is_finite() && y.is_finite() && y.abs() < 1.0e5, "{} shape {i}: ({x}, {y})", f.display());
            }
        }
    }
}
