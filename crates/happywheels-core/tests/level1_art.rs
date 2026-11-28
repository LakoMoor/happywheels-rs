//! The built-in level drawn with the original artwork, as a screenshot to put next to the original's frame (`analysis/level1-draw.js`).
//! Needs the original assets; without them the test does nothing.
use happywheels_core::level::Level;
use happywheels_core::physics::World;
use happywheels_core::render::render_level_view;
use happywheels_core::sprites::{SpriteAssets, Sprites};

fn assets() -> Option<std::path::PathBuf> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    path.join("animate/level1").is_dir().then_some(path)
}

#[test]
fn dump_level1() {
    let Some(assets) = assets() else { return };
    let level = Level::level1();
    let backdrop = Sprites::open_level_backdrop(&SpriteAssets::Directory(assets.clone()), &level).expect("level 1 artwork");
    let mut world = World::for_level(&level);
    assert!(world.spawn_bike(3, level.info.start));
    let mut sprites = Sprites::open(&assets, 3).expect("character artwork");
    sprites.bind_vehicle_rider(&world, level.info.start);
    for _ in 0..60 {
        world.step();
    }
    let shots = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../screenshots");
    let shot = |name: &str, camera: [f32; 2]| {
        let img = render_level_view(&level, &world, 1800, 1000, 2.0, camera, Some(&sprites), Some(&backdrop));
        // the ground under the start is the level's own art, not the plain background colour
        assert_ne!(img.get_pixel(900, 990).0, [255, 255, 255, 255], "{name}");
        img.save(shots.join(name)).unwrap();
    };
    shot("level1_start.png", world.view_center_px().expect("camera"));
    for (name, at) in [("level1_bridge.png", [4600.0, 150.0]), ("level1_gears.png", [5700.0, 600.0]), ("level1_pistons.png", [8400.0, 350.0]), ("level1_mine.png", [300.0, 250.0])] {
        shot(name, at);
    }
}
