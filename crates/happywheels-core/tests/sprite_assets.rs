use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

use happywheels_core::{
    level,
    physics::World,
    render,
    sprites::{SpriteAssets, Sprites},
};

fn collect_atlases(root: &Path, dir: &Path, files: &mut HashMap<PathBuf, Vec<u8>>) {
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            collect_atlases(root, &path, files);
        } else if path.to_string_lossy().contains("@0.3x.") {
            files.insert(
                path.strip_prefix(root).unwrap().to_path_buf(),
                std::fs::read(path).unwrap(),
            );
        }
    }
}

#[test]
fn memory_atlases_render_identically_to_disk() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    if !root.join("animate/character3").exists() {
        eprintln!("Original atlases unavailable; skipping asset-dependent comparison");
        return;
    }
    let mut files = HashMap::new();
    collect_atlases(&root, &root.join("animate"), &mut files);
    let memory = SpriteAssets::memory(files);
    let level = level::parse(include_str!("../../../tests/levels/sample.xml")).unwrap();
    let mut world = World::from_level(&level);
    assert!(world.spawn_bike(3, level.info.start));
    let mut disk_sprites = Sprites::open(&root, 3).unwrap();
    let mut memory_sprites = Sprites::open_from(&memory, 3).unwrap();
    disk_sprites.bind_vehicle_rider(&world, level.info.start);
    memory_sprites.bind_vehicle_rider(&world, level.info.start);
    let disk = render::render_level_sprites(
        &level,
        &world,
        640,
        360,
        level.info.start,
        Some(&disk_sprites),
    );
    let from_memory = render::render_level_sprites(
        &level,
        &world,
        640,
        360,
        level.info.start,
        Some(&memory_sprites),
    );
    assert_eq!(disk, from_memory);
    let without_art = render::render_level(&level, &world, 640, 360);
    assert_ne!(disk, without_art, "comparison must exercise artwork");
}
