#![allow(clippy::too_many_arguments)] // blit/text helpers take explicit geometry
use ab_glyph::{FontArc, PxScale};
use anyhow::{Context, Result};
use image::{imageops, Rgba, RgbaImage};
use imageproc::drawing::{draw_hollow_rect_mut, draw_text_mut, text_size};

use crate::{
    assets::AssetRoot,
    atlas::{Atlas, Rect as AtlasRect},
};

const LOGICAL_WIDTH: u32 = 900;
const LOGICAL_HEIGHT: u32 = 500;
const BLUE: Rgba<u8> = Rgba([0, 105, 165, 255]);
const RED: Rgba<u8> = Rgba([170, 70, 80, 255]);
const WHITE: Rgba<u8> = Rgba([240, 240, 240, 255]);
const DISABLED: Rgba<u8> = Rgba([85, 95, 110, 180]);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuAction {
    Exit,
    CharacterSelected(u8),
    LoadLevel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Scene {
    Main,
    CharacterSelect,
    Selected(u8),
}

#[derive(Debug, Clone, Copy)]
struct Button {
    rect: AtlasRect,
    action: ButtonAction,
    enabled: bool,
}

#[derive(Debug, Clone, Copy)]
enum ButtonAction {
    Play,
    Back,
    Exit,
    LoadLevel,
    Character(u8),
    Unavailable,
}

#[derive(Debug, Clone, Copy)]
struct Presentation {
    scale: f32,
    offset_x: f32,
    offset_y: f32,
}

pub struct Menu {
    scene: Scene,
    ui: Atlas,
    ui_pixels: RgbaImage,
    ui_parts: Atlas,
    ui_parts_pixels: RgbaImage,
    faces: Atlas,
    face_pixels: RgbaImage,
    title_font: FontArc,
    body_font: FontArc,
    buttons: Vec<Button>,
    presentation: Presentation,
    /// A button was released: the original plays `MenuSelect`.
    pressed: bool,
}

impl Menu {
    pub fn reset(&mut self) { self.scene = Scene::Main; }
    pub fn load_memory(files: &std::collections::HashMap<std::path::PathBuf, Vec<u8>>) -> Result<Self> {
        let read = |name: &str| files.get(std::path::Path::new(name)).with_context(|| format!("missing {name}"));
        let (ui, ui_pixels) = Atlas::from_bytes(read("animate/game-ui/game_ui_atlas_1/game_ui_atlas_1.json")?, read("animate/game-ui/game_ui_atlas_1/game_ui_atlas_1.png")?)?;
        let (ui_parts, ui_parts_pixels) = Atlas::from_bytes(read("animate/game-ui/game_ui_atlas_3/game_ui_atlas_3.json")?, read("animate/game-ui/game_ui_atlas_3/game_ui_atlas_3.png")?)?;
        let (faces, face_pixels) = Atlas::from_bytes(read("image/character_images/character_images.json")?, read("image/character_images/character_images.png")?)?;
        let title_font = FontArc::try_from_vec(read("fonts/ClarendonLTStd-Bold.otf")?.clone()).map_err(|e| anyhow::anyhow!("title font: {e:?}"))?;
        let body_font = FontArc::try_from_vec(read("fonts/HelveticaNeueLTStd-Roman.otf")?.clone()).map_err(|e| anyhow::anyhow!("body font: {e:?}"))?;
        Ok(Self { scene: Scene::Main, ui, ui_pixels, ui_parts, ui_parts_pixels, faces, face_pixels, title_font, body_font, buttons: Vec::new(), presentation: Presentation { scale: 1.0, offset_x: 0.0, offset_y: 0.0 }, pressed: false })
    }
    pub fn load(assets: &AssetRoot) -> Result<Self> {
        let ui = assets.atlas("animate/game-ui/game_ui_atlas_1/game_ui_atlas_1.json")?;
        let ui_pixels = ui.load_pixels()?;
        let ui_parts = assets.atlas("animate/game-ui/game_ui_atlas_3/game_ui_atlas_3.json")?;
        let ui_parts_pixels = ui_parts.load_pixels()?;
        let faces = assets.atlas("image/character_images/character_images.json")?;
        let face_pixels = faces.load_pixels()?;
        let title_font = assets.font("fonts/ClarendonLTStd-Bold.otf")?;
        let body_font = assets.font("fonts/HelveticaNeueLTStd-Roman.otf")?;
        Ok(Self {
            scene: Scene::Main,
            ui,
            ui_pixels,
            ui_parts,
            ui_parts_pixels,
            faces,
            face_pixels,
            title_font,
            body_font,
            buttons: Vec::new(),
            pressed: false,
            presentation: Presentation {
                scale: 1.0,
                offset_x: 0.0,
                offset_y: 0.0,
            },
        })
    }

    pub fn selected_character(&self) -> Option<u8> {
        match self.scene {
            Scene::Selected(character) => Some(character),
            _ => None,
        }
    }

    /// Whether a click since the last call hit a button (the app then plays the `MenuSelect` sound).
    pub fn take_press(&mut self) -> bool {
        std::mem::take(&mut self.pressed)
    }

    pub fn handle_click(&mut self, x: f32, y: f32) -> Option<MenuAction> {
        let logical_x = (x - self.presentation.offset_x) / self.presentation.scale;
        let logical_y = (y - self.presentation.offset_y) / self.presentation.scale;
        let button = self.buttons.iter().find(|button| {
            button.enabled
                && logical_x >= button.rect.x as f32
                && logical_y >= button.rect.y as f32
                && logical_x < (button.rect.x + button.rect.w) as f32
                && logical_y < (button.rect.y + button.rect.h) as f32
        })?;
        self.pressed = true;
        match button.action {
            ButtonAction::Play => {
                self.scene = Scene::CharacterSelect;
                None
            }
            ButtonAction::Back => {
                self.scene = Scene::Main;
                None
            }
            ButtonAction::Exit => Some(MenuAction::Exit),
            ButtonAction::LoadLevel => Some(MenuAction::LoadLevel),
            ButtonAction::Character(character) => {
                self.scene = Scene::Selected(character);
                Some(MenuAction::CharacterSelected(character))
            }
            ButtonAction::Unavailable => None,
        }
    }

    pub fn render(&mut self, width: u32, height: u32) -> Result<RgbaImage> {
        let mut canvas = RgbaImage::new(LOGICAL_WIDTH, LOGICAL_HEIGHT);
        draw_background(&mut canvas);
        self.buttons.clear();
        match self.scene {
            Scene::Main => self.draw_main(&mut canvas)?,
            Scene::CharacterSelect => self.draw_character_select(&mut canvas)?,
            Scene::Selected(character) => self.draw_selected(&mut canvas, character)?,
        }

        let scale = (width as f32 / LOGICAL_WIDTH as f32)
            .min(height as f32 / LOGICAL_HEIGHT as f32)
            .max(f32::EPSILON);
        let scaled_width = ((LOGICAL_WIDTH as f32 * scale).round() as u32).max(1);
        let scaled_height = ((LOGICAL_HEIGHT as f32 * scale).round() as u32).max(1);
        let offset_x = (width.saturating_sub(scaled_width)) / 2;
        let offset_y = (height.saturating_sub(scaled_height)) / 2;
        self.presentation = Presentation {
            scale,
            offset_x: offset_x as f32,
            offset_y: offset_y as f32,
        };

        let scaled = imageops::resize(
            &canvas,
            scaled_width,
            scaled_height,
            imageops::FilterType::Lanczos3,
        );
        let mut output = RgbaImage::new(width.max(1), height.max(1));
        imageops::overlay(&mut output, &scaled, offset_x.into(), offset_y.into());
        Ok(output)
    }

    fn draw_main(&mut self, canvas: &mut RgbaImage) -> Result<()> {
        self.sprite(
            canvas,
            &self.ui_parts,
            &self.ui_parts_pixels,
            "game-ui-asset-38_nophone.png",
            70,
            0,
            100,
            236,
        )?;
        self.sprite(
            canvas,
            &self.ui_parts,
            &self.ui_parts_pixels,
            "game-ui-asset-36.png",
            0,
            190,
            290,
            290,
        )?;
        self.sprite(
            canvas,
            &self.ui_parts,
            &self.ui_parts_pixels,
            "game-ui-asset-27.png",
            80,
            90,
            180,
            129,
        )?;
        self.sprite(
            canvas,
            &self.ui,
            &self.ui_pixels,
            "game-ui-asset-6.png",
            380,
            18,
            500,
            225,
        )?;

        let entries = [
            ("PLAY", ButtonAction::Play, true, BLUE),
            ("BROWSE LEVELS", ButtonAction::Unavailable, false, BLUE),
            ("LOAD LEVEL/REPLAY", ButtonAction::LoadLevel, true, BLUE),
            ("LEVEL EDITOR", ButtonAction::Unavailable, false, BLUE),
            ("OPTIONS", ButtonAction::Unavailable, false, RED),
            ("CREDITS", ButtonAction::Unavailable, false, RED),
            ("EXIT", ButtonAction::Exit, true, RED),
        ];
        for (index, (label, action, enabled, color)) in entries.into_iter().enumerate() {
            let y = 255 + index as u32 * 32;
            self.text_right(
                canvas,
                label,
                875,
                y as i32,
                30.0,
                if enabled { color } else { DISABLED },
                &self.title_font,
            );
            self.buttons.push(Button {
                rect: AtlasRect {
                    x: 565,
                    y,
                    w: 315,
                    h: 32,
                },
                action,
                enabled,
            });
        }
        self.text(canvas, "v 1.99.1-r", 8, 482, 10.0, WHITE, &self.body_font);
        Ok(())
    }

    fn draw_character_select(&mut self, canvas: &mut RgbaImage) -> Result<()> {
        self.sprite(
            canvas,
            &self.ui,
            &self.ui_pixels,
            "game-ui-108.png",
            175,
            20,
            550,
            81,
        )?;
        for character in 1..=11u8 {
            let column = u32::from(character - 1) % 6;
            let row = u32::from(character - 1) / 6;
            let x = 93 + column * 122;
            let y = 145 + row * 135;
            self.sprite(
                canvas,
                &self.faces,
                &self.face_pixels,
                &format!("char{character}.png"),
                x,
                y,
                72,
                72,
            )?;
            draw_hollow_rect_mut(
                canvas,
                imageproc::rect::Rect::at(x as i32 - 4, y as i32 - 4).of_size(80, 80),
                Rgba([210, 210, 210, 255]),
            );
            self.buttons.push(Button {
                rect: AtlasRect {
                    x: x - 4,
                    y: y - 4,
                    w: 80,
                    h: 80,
                },
                action: ButtonAction::Character(character),
                enabled: true,
            });
        }
        self.text(
            canvas,
            "Select your character",
            285,
            435,
            20.0,
            WHITE,
            &self.body_font,
        );
        self.text(canvas, "BACK", 20, 465, 22.0, BLUE, &self.title_font);
        self.buttons.push(Button {
            rect: AtlasRect {
                x: 10,
                y: 455,
                w: 100,
                h: 35,
            },
            action: ButtonAction::Back,
            enabled: true,
        });
        Ok(())
    }

    fn draw_selected(&mut self, canvas: &mut RgbaImage, character: u8) -> Result<()> {
        self.sprite(
            canvas,
            &self.faces,
            &self.face_pixels,
            &format!("char{character}.png"),
            370,
            65,
            160,
            160,
        )?;
        self.text_center(
            canvas,
            &format!("Character {character} selected"),
            260,
            32.0,
            WHITE,
            &self.title_font,
        );
        self.text_center(
            canvas,
            "Level loading and physics are not ported yet.",
            315,
            20.0,
            WHITE,
            &self.body_font,
        );
        self.text_center(canvas, "BACK", 405, 26.0, BLUE, &self.title_font);
        self.buttons.push(Button {
            rect: AtlasRect {
                x: 380,
                y: 390,
                w: 140,
                h: 55,
            },
            action: ButtonAction::Back,
            enabled: true,
        });
        Ok(())
    }

    fn sprite(
        &self,
        canvas: &mut RgbaImage,
        atlas: &Atlas,
        pixels: &RgbaImage,
        name: &str,
        x: u32,
        y: u32,
        width: u32,
        height: u32,
    ) -> Result<()> {
        let sprite = atlas.resize_frame(pixels, name, width, height)?;
        imageops::overlay(canvas, &sprite, x.into(), y.into());
        Ok(())
    }

    fn text(
        &self,
        canvas: &mut RgbaImage,
        text: &str,
        x: i32,
        y: i32,
        size: f32,
        color: Rgba<u8>,
        font: &FontArc,
    ) {
        draw_text_mut(canvas, color, x, y, PxScale::from(size), font, text);
    }

    fn text_right(
        &self,
        canvas: &mut RgbaImage,
        text: &str,
        right: u32,
        y: i32,
        size: f32,
        color: Rgba<u8>,
        font: &FontArc,
    ) {
        let (width, _) = text_size(PxScale::from(size), font, text);
        self.text(
            canvas,
            text,
            right.saturating_sub(width) as i32,
            y,
            size,
            color,
            font,
        );
    }

    fn text_center(
        &self,
        canvas: &mut RgbaImage,
        text: &str,
        y: u32,
        size: f32,
        color: Rgba<u8>,
        font: &FontArc,
    ) {
        let (width, _) = text_size(PxScale::from(size), font, text);
        self.text(
            canvas,
            text,
            LOGICAL_WIDTH.saturating_sub(width) as i32 / 2,
            y as i32,
            size,
            color,
            font,
        );
    }
}

fn draw_background(canvas: &mut RgbaImage) {
    for (x, y, pixel) in canvas.enumerate_pixels_mut() {
        let horizontal = x as f32 / LOGICAL_WIDTH as f32;
        let vertical = y as f32 / LOGICAL_HEIGHT as f32;
        let light = 0.55 + 0.25 * (1.0 - horizontal) + 0.15 * (1.0 - vertical);
        *pixel = Rgba([
            (38.0 * light) as u8,
            (42.0 * light) as u8,
            (65.0 * light) as u8,
            255,
        ]);
    }
}
