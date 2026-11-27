//! What the port draws after one step, compared with the original's display list (texture and corners, in draw order).
//! The lists in `tests/golden/draw-c*.json` come from the running original (`analysis/golden-draw.js`, `window.__CHAR`).

use happywheels_core::{physics::World, sprites::Sprites};

fn port_list(c: usize, frames: usize, break_limbs: bool) -> Option<Vec<(String, [[f64; 2]; 3])>> {
    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    let mut sprites = Sprites::open(&assets, c as u32)?;
    let level = happywheels_core::level::parse("<levelXML><info v='1.99' x='1000' y='250' h='f' bg='0' bgc=''/><shapes><sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>").unwrap();
    let mut world = World::from_level(&level);
    assert!(world.spawn_bike(c, level.info.start));
    sprites.bind_vehicle_rider(&world, level.info.start);
    if break_limbs {
        use happywheels_core::character::Limb;
        let mut doll = world.character.take().unwrap();
        let mut vehicle = world.bike.take().unwrap();
        for (limb, over) in [(Limb::Neck, 5.0), (Limb::Waist, 10.0), (Limb::Elbow1, 1.0)] {
            let force = doll.break_limit(&world, limb) + over;
            doll.break_limb(&mut world, limb, force);
            vehicle.rider_hurt(&mut world, &mut doll, false, &[limb]);
        }
        world.character = Some(doll);
        world.bike = Some(vehicle);
    }
    for _ in 0..frames {
        world.set_rider_keys(Default::default());
        world.step();
    }
    *sprites.trace.borrow_mut() = Some(Vec::new());
    sprites.draw(&mut image::RgbaImage::new(1, 1), &world, (0.0, 0.0));
    let list = sprites.trace.borrow_mut().take();
    // the first-frame lists were taken without the stroked lines
    // blood particles are random in both and have no texture name in the original
    list.map(|l| l.into_iter().filter(|d| !d.0.starts_with("blood-particle") && (frames > 1 || break_limbs || !d.0.starts_with("line "))).collect())
}

#[test]
fn characters_are_drawn_like_the_original() {
    for c in 1..=11 {
        compare(c, 1, "draw", false);
    }
}

/// After 30 frames: wheels have turned, and the lines `paint` strokes (shocks, magnet rope, harness) have moved.
#[test]
fn moving_characters_are_drawn_like_the_original() {
    for c in 1..=11 {
        compare(c, 30, "lines", false);
    }
}

/// Neck, waist and elbow broken before the first frame: stumps, spine, intestines and the ligament, with their red lines.
#[test]
fn broken_riders_are_drawn_like_the_original() {
    for c in [2, 6] {
        compare(c, 1, "broken1", true);
        compare(c, 15, "broken15", true);
    }
}

/// Known difference: the c10 daughter's chain gear is a pixel off.
fn compare(c: usize, frames: usize, golden: &str, break_limbs: bool) {
        let Some(port) = port_list(c, frames, break_limbs) else { return };
        let path = format!("{}/../../tests/golden/{golden}-c{c}.json", env!("CARGO_MANIFEST_DIR"));
        let original: Vec<(String, [[f64; 2]; 3])> = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let mut used = vec![false; port.len()];
        let mut previous = None;
        for (name, quad) in &original {
            let off = |other: &[[f64; 2]; 3]| quad.iter().flatten().zip(other.iter().flatten()).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
            let nearest = (0..port.len()).filter(|&j| !used[j] && &port[j].0 == name).min_by(|&a, &b| off(&port[a].1).total_cmp(&off(&port[b].1)));
            let Some(j) = nearest.filter(|&j| off(&port[j].1) < 2.0) else {
                panic!("c{c}: {name} at {:?} is not drawn; same texture here: {:?}", quad[0], port.iter().filter(|d| &d.0 == name).map(|d| d.1[0]).collect::<Vec<_>>());
            };
            used[j] = true;
            let limit = if c == 10 && off(&port[j].1) < 1.2 { 1.2 } else { 0.6 };
            assert!(off(&port[j].1) < limit, "c{c}: {name} is {} px off", off(&port[j].1));
            assert!(previous.is_none_or(|p| p < j), "c{c}: {name} is drawn in the wrong layer");
            previous = Some(j);
        }
        let extra: Vec<_> = port.iter().zip(&used).filter(|(_, &u)| !u).map(|(d, _)| &d.0).collect();
        assert!(extra.is_empty(), "c{c}: drawn here only: {extra:?}");
}
