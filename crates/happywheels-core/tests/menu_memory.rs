use std::{collections::HashMap, path::PathBuf};
use happywheels_core::{AssetRoot, Menu};

#[test]
fn menu_memory_matches_directory_assets() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    if !root.exists() { eprintln!("Original assets unavailable; skipping"); return; }
    let names = ["animate/game-ui/game_ui_atlas_1/game_ui_atlas_1.json", "animate/game-ui/game_ui_atlas_1/game_ui_atlas_1.png", "animate/game-ui/game_ui_atlas_3/game_ui_atlas_3.json", "animate/game-ui/game_ui_atlas_3/game_ui_atlas_3.png", "image/character_images/character_images.json", "image/character_images/character_images.png", "fonts/ClarendonLTStd-Bold.otf", "fonts/HelveticaNeueLTStd-Roman.otf"];
    let files: HashMap<PathBuf, Vec<u8>> = names.iter().map(|name| (PathBuf::from(name), std::fs::read(root.join(name)).unwrap())).collect();
    let mut disk = Menu::load(&AssetRoot::new(&root).unwrap()).unwrap();
    let mut memory = Menu::load_memory(&files).unwrap();
    assert_eq!(disk.render(900, 500).unwrap(), memory.render(900, 500).unwrap());
}
