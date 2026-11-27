#[cfg(target_arch = "wasm32")]
mod renderer;

#[cfg(any(target_arch = "wasm32", test))]
fn in_mouse_trigger(point: [f32; 2], position: [f32; 2], size: [f32; 2]) -> bool {
    (point[0] - position[0]).abs() < size[0].abs() * 0.5
        && (point[1] - position[1]).abs() < size[1].abs() * 0.5
}

#[cfg(any(target_arch = "wasm32", test))]
fn elapsed_ms(last: Option<f64>, now: f64, paused: bool) -> f64 {
    last.map_or(0.0, |previous| if paused { 0.0 } else { (now - previous).clamp(0.0, 250.0) })
}

#[cfg(test)]
mod tests {
    #[test]
    fn mouse_trigger_hit_area_is_screen_aligned() {
        assert!(super::in_mouse_trigger([110.0, 110.0], [100.0, 100.0], [40.0, 30.0]));
        assert!(!super::in_mouse_trigger([121.0, 100.0], [100.0, 100.0], [40.0, 30.0]));
        assert!(!super::in_mouse_trigger([100.0, 116.0], [100.0, 100.0], [40.0, 30.0]));
    }

    #[test]
    fn paused_clock_does_not_accumulate_elapsed_time() {
        assert_eq!(super::elapsed_ms(Some(10.0), 1000.0, true), 0.0);
        assert_eq!(super::elapsed_ms(None, 1000.0, false), 0.0);
        assert_eq!(super::elapsed_ms(Some(1000.0), 1016.0, false), 16.0);
        assert_eq!(super::elapsed_ms(Some(10.0), 1000.0, false), 250.0);
    }
}

#[cfg(target_arch = "wasm32")]
mod browser {
    use happywheels_core::{
        level::Level,
        physics::World,
        render,
        sprites::{SpriteAssets, Sprites},
        vehicle::Keys,
        Menu, MenuAction,
    };
    use std::{collections::HashMap, path::PathBuf};
    use wasm_bindgen::prelude::*;
    use web_sys::HtmlCanvasElement;

    use crate::renderer::Renderer;

    const SAMPLE: &str = include_str!("../../../tests/levels/sample.xml");

    #[wasm_bindgen]
    pub fn audio_table() -> String {
        include_str!("../../happywheels-core/data/audiosprite.json").to_string()
    }

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
            if !(path.starts_with("animate") || path.starts_with("image") || path.starts_with("fonts"))
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
        particle_time: f64,
        /// The original's "60 fps" option (`maxFPS = 60`, `update60fps`): the simulation still runs at 30 Hz, bodies are drawn between the last two steps.
        interpolate: bool,
        prev_poses: Vec<happywheels_core::Pose>,
        character: usize,
        assets: Option<SpriteAssets>,
        sprites: Option<Sprites>,
        backdrop: Option<Sprites>,
        menu: Option<Menu>,
        menu_open: bool,
        paused: bool,
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
                particle_time: 0.0,
                interpolate: false,
                prev_poses: Vec::new(),
                character: 3,
                assets: None,
                sprites: None,
                backdrop: None,
                menu: None,
                menu_open: false,
                paused: false,
            })
        }

        /// Load locally selected original atlas files without uploading them anywhere.
        pub fn set_assets(&mut self, pack: AssetPack) -> Result<(), JsValue> {
            let menu = Menu::load_memory(&pack.files).ok();
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
            self.menu = menu;
            Ok(())
        }

        pub fn open_menu(&mut self) -> bool {
            if let Some(menu) = &mut self.menu { menu.reset(); }
            self.menu_open = self.menu.is_some();
            self.keys = [false; 8];
            self.menu_open
        }

        pub fn menu_click(&mut self, x: f32, y: f32) -> i32 {
            if !self.menu_open { return 0; }
            match self.menu.as_mut().and_then(|menu| menu.handle_click(x, y)) {
                Some(MenuAction::CharacterSelected(index)) => i32::from(index),
                Some(MenuAction::LoadLevel) => -1,
                Some(MenuAction::Exit) => { self.menu_open = false; -2 },
                None => 0,
            }
        }

        pub fn close_menu(&mut self) { self.menu_open = false; self.last_time = None; }

        /// Replace the current level only after it parses successfully.
        pub fn load_level(&mut self, xml: &str, character: usize) -> Result<(), JsValue> {
            let level = happywheels_core::level::parse(xml).map_err(js_error)?;
            self.replace_level(level, character)
        }

        /// Recreate the current level and rider, retaining the loaded asset pack.
        pub fn restart(&mut self, character: usize) -> Result<(), JsValue> {
            self.replace_level(self.level.clone(), character)
        }

        fn replace_level(&mut self, level: Level, character: usize) -> Result<(), JsValue> {
            if !(1..=11).contains(&character) {
                return Err(JsValue::from_str("Character must be between 1 and 11"));
            }
            let world = world_for(&level, character);
            self.level = level;
            self.world = world;
            self.prev_poses.clear();
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
            self.menu_open = false;
            self.paused = false;
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

        /// Discard wall-clock time spent while the page is hidden or suspended.
        pub fn reset_clock(&mut self) {
            self.last_time = None;
        }

        pub fn level_complete(&self) -> bool {
            self.world.level_complete
        }

        pub fn set_paused(&mut self, paused: bool) {
            self.paused = paused;
            self.last_time = None;
            self.keys = [false; 8];
        }

        /// Mouse triggers are transparent, unrotated rectangles in the original session's 900×500 screen space.
        /// `use60FPS` option: draw the simulation interpolated between its 30 Hz steps (the physics itself is unchanged).
        pub fn set_interpolation(&mut self, on: bool) {
            self.interpolate = on;
        }

        /// `bloodSetting` (0 off, 1 bitmap particles, 2 lines, 3/4 metaball) for worlds created from now on; call `restart` to apply it.
        pub fn set_blood_setting(&mut self, setting: u8) {
            happywheels_core::blood::set_blood_setting(setting.min(4));
        }

        /// Maximum number of blood/shard particles (the options slider, 0..=2000); applies to worlds created from now on (Restart / level load).
        pub fn set_max_particles(&mut self, max: u32) {
            happywheels_core::blood::set_max_particles(max as usize);
        }

        pub fn pointer(&mut self, x: f32, y: f32, clicked: bool) {
            if self.paused || self.menu_open { return; }
            let mut topmost = None;
            for (i, trigger) in self.level.triggers.iter().enumerate() {
                if trigger.triggered_by != 6 { continue; }
                let inside = crate::in_mouse_trigger([x, y], trigger.pos, trigger.size);
                if inside && self.world.mouse_trigger_enabled(i) {
                    topmost = Some(i);
                } else if !inside {
                    self.world.release_trigger(i);
                }
            }
            if clicked {
                if let Some(i) = topmost { self.world.click_trigger(i); }
            }
        }

        pub fn frame(&mut self, now_ms: f64) -> Result<String, JsValue> {
            if self.menu_open {
                self.last_time = None;
                let (w, h) = (self.canvas.width().max(1), self.canvas.height().max(1));
                let frame = self.menu.as_mut().ok_or_else(|| JsValue::from_str("Menu assets unavailable"))?.render(w, h).map_err(js_error)?;
                self.renderer.resize(w, h);
                self.renderer.upload(&frame).map_err(js_error)?;
                self.renderer.render();
                return Ok("[[],[],[0,0],[]]".into());
            }
            // A hidden tab must not accumulate minutes of simulation to catch up on return.
            let elapsed = crate::elapsed_ms(self.last_time, now_ms, self.paused);
            self.last_time = Some(now_ms);
            self.accumulator += elapsed;
            let mut sounds = Vec::new();
            let mut trigger_sounds = Vec::new();
            let mut scaled_sounds = Vec::new();
            // 60 fps (`run60fps`): physics keeps its 30 Hz tick, the particles move every display frame by dt = 30 / maxFPS
            self.world.set_external_particles(self.interpolate);
            while !self.paused && self.accumulator >= 1000.0 / 30.0 {
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
                self.prev_poses = self.world.b2().pose_snapshot();
                self.world.step();
                sounds.extend(self.world.take_sounds());
                trigger_sounds.extend(self.world.take_trigger_sounds());
                scaled_sounds.extend(self.world.take_scaled_sounds());
                self.accumulator -= 1000.0 / 30.0;
            }
            if self.interpolate && !self.paused {
                // the original's ticker is capped at maxFPS = 60 and steps the particles with dt = 0.5; faster displays must not add steps
                self.particle_time += elapsed;
                while self.particle_time >= 1000.0 / 60.0 {
                    self.world.step_particles(0.5);
                    self.particle_time -= 1000.0 / 60.0;
                }
            }
            let (w, h) = (self.canvas.width().max(1), self.canvas.height().max(1));
            let blend = (self.interpolate && !self.paused && !self.prev_poses.is_empty()).then(|| (self.accumulator / (1000.0 / 30.0)).clamp(0.0, 1.0));
            let saved = blend.map(|b| self.world.b2_mut().blend_poses(&self.prev_poses, b));
            let camera = self.world.camera_focus_px().unwrap_or(self.level.info.start);
            let frame = render::render_level_scene(
                &self.level,
                &self.world,
                w,
                h,
                camera,
                self.sprites.as_ref(),
                self.backdrop.as_ref(),
            );
            if let Some(saved) = saved {
                self.world.b2_mut().restore_poses(&saved);
            }
            self.renderer.resize(w, h);
            self.renderer.upload(&frame).map_err(js_error)?;
            self.renderer.render();
            let loops = if self.paused { &[][..] } else { &self.world.loops[..] };
            serde_json::to_string(&(sounds, loops, camera, trigger_sounds, scaled_sounds)).map_err(js_error)
        }
    }
}
