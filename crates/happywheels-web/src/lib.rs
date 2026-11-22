#[cfg(target_arch = "wasm32")]
mod renderer;

#[cfg(target_arch = "wasm32")]
mod browser {
    use happywheels_core::{
        character::Part,
        level::Level,
        physics::World,
        render,
        sprites::{SpriteAssets, Sprites},
        vehicle::Keys,
    };
    use std::{collections::HashMap, path::PathBuf};
    use wasm_bindgen::prelude::*;
    use web_sys::HtmlCanvasElement;

    use crate::renderer::Renderer;

    const SAMPLE: &str = include_str!("../../../tests/levels/sample.xml");

    #[wasm_bindgen]
    pub struct AssetPack {
        files: HashMap<PathBuf, Vec<u8>>,
    }

    #[wasm_bindgen]
    impl AssetPack {
        #[wasm_bindgen(constructor)]
        pub fn new() -> Self {
            Self {
                files: HashMap::new(),
            }
        }

        pub fn add(&mut self, path: &str, bytes: &[u8]) -> Result<(), JsValue> {
            // Only normalized relative atlas paths are accepted.
            let path = std::path::Path::new(path);
            if !path.starts_with("animate")
                || path
                    .components()
                    .any(|part| !matches!(part, std::path::Component::Normal(_)))
            {
                return Err(JsValue::from_str("Invalid asset path"));
            }
            self.files.insert(path.to_path_buf(), bytes.to_vec());
            Ok(())
        }
    }

    #[wasm_bindgen]
    pub struct Game {
        level: Level,
        world: World,
        renderer: Renderer,
        canvas: HtmlCanvasElement,
        keys: [bool; 8],
        last_time: Option<f64>,
        accumulator: f64,
        character: usize,
        assets: Option<SpriteAssets>,
        sprites: Option<Sprites>,
        backdrop: Option<Sprites>,
    }

    fn world_for(level: &Level, character: usize) -> World {
        let mut world = World::from_level(level);
        if !world.spawn_bike(character, level.info.start) {
            world.spawn_character(character, level.info.start);
        }
        world
    }

    fn js_error(error: impl std::fmt::Display) -> JsValue {
        JsValue::from_str(&error.to_string())
    }

    #[wasm_bindgen]
    impl Game {
        pub async fn create(canvas: HtmlCanvasElement) -> Result<Game, JsValue> {
            let level = happywheels_core::level::parse(SAMPLE).map_err(js_error)?;
            let world = world_for(&level, 3);
            let frame = render::render_level(
                &level,
                &world,
                canvas.width().max(1),
                canvas.height().max(1),
            );
            let renderer = Renderer::new(canvas.clone(), &frame)
                .await
                .map_err(js_error)?;
            Ok(Self {
                level,
                world,
                renderer,
                canvas,
                keys: [false; 8],
                last_time: None,
                accumulator: 0.0,
                character: 3,
                assets: None,
                sprites: None,
                backdrop: None,
            })
        }

        /// Load locally selected original atlas files without uploading them anywhere.
        pub fn set_assets(&mut self, pack: AssetPack) -> Result<(), JsValue> {
            let assets = SpriteAssets::memory(pack.files);
            let mut sprites =
                Sprites::open_from(&assets, self.character as u32).ok_or_else(|| {
                    JsValue::from_str(
                        "The selected folder does not contain the current character's atlas",
                    )
                })?;
            sprites.bind_vehicle_rider(&self.world, self.level.info.start);
            let backdrop = match self.level.info.background {
                1 => Sprites::open_backdrop_from(&assets),
                2 => Sprites::open_city_backdrop_from(&assets),
                _ => None,
            };
            self.assets = Some(assets);
            self.sprites = Some(sprites);
            self.backdrop = backdrop;
            Ok(())
        }

        /// Replace the current level only after it parses successfully.
        pub fn load_level(&mut self, xml: &str, character: usize) -> Result<(), JsValue> {
            if !(1..=11).contains(&character) {
                return Err(JsValue::from_str("Character must be between 1 and 11"));
            }
            let level = happywheels_core::level::parse(xml).map_err(js_error)?;
            let world = world_for(&level, character);
            self.level = level;
            self.world = world;
            self.character = character;
            self.sprites = self.assets.as_ref().and_then(|assets| {
                let mut sprites = Sprites::open_from(assets, character as u32)?;
                sprites.bind_vehicle_rider(&self.world, self.level.info.start);
                Some(sprites)
            });
            self.backdrop =
                self.assets
                    .as_ref()
                    .and_then(|assets| match self.level.info.background {
                        1 => Sprites::open_backdrop_from(assets),
                        2 => Sprites::open_city_backdrop_from(assets),
                        _ => None,
                    });
            self.keys = [false; 8];
            self.accumulator = 0.0;
            self.last_time = None;
            Ok(())
        }

        pub fn key(&mut self, index: usize, pressed: bool) {
            if let Some(key) = self.keys.get_mut(index) {
                *key = pressed;
            }
        }

        pub fn clear_keys(&mut self) {
            self.keys = [false; 8];
        }

        pub fn frame(&mut self, now_ms: f64) -> Result<(), JsValue> {
            // A hidden tab must not accumulate minutes of simulation to catch up on return.
            let elapsed = self
                .last_time
                .map_or(0.0, |last| (now_ms - last).clamp(0.0, 250.0));
            self.last_time = Some(now_ms);
            self.accumulator += elapsed;
            while self.accumulator >= 1000.0 / 30.0 {
                let k = self.keys;
                self.world
                    .drive(f32::from(u8::from(k[1])) - f32::from(u8::from(k[0])));
                self.world.set_rider_keys(Keys {
                    left: k[0],
                    right: k[1],
                    up: k[2],
                    down: k[3],
                    space: k[4],
                    z: k[5],
                    shift: k[6],
                    ctrl: k[7],
                });
                self.world.step();
                self.world.take_sounds();
                self.accumulator -= 1000.0 / 30.0;
            }
            let (w, h) = (self.canvas.width().max(1), self.canvas.height().max(1));
            let camera = self
                .world
                .character
                .as_ref()
                .and_then(|d| d.bodies.get(&Part::Chest))
                .and_then(|&b| self.world.body_position_px(b))
                .unwrap_or(self.level.info.start);
            let frame = render::render_level_scene(
                &self.level,
                &self.world,
                w,
                h,
                camera,
                self.sprites.as_ref(),
                self.backdrop.as_ref(),
            );
            self.renderer.resize(w, h);
            self.renderer.upload(&frame).map_err(js_error)?;
            self.renderer.render();
            Ok(())
        }
    }
}
