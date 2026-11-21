use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

use anyhow::{bail, Context, Result};
use image::{imageops::FilterType, RgbaImage};
use serde::Deserialize;

#[derive(Debug, Clone)]
pub struct Atlas {
    image_path: PathBuf,
    frames: BTreeMap<String, Frame>,
    width: u32,
    height: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rect {
    pub x: u32,
    pub y: u32,
    pub w: u32,
    pub h: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub frame: Rect,
    pub sprite_source: Rect,
    pub source_size: (u32, u32),
    pub rotated: bool,
}

impl Atlas {
    pub fn open(json_path: impl AsRef<Path>) -> Result<Self> {
        let json_path = json_path.as_ref();
        let text = fs::read_to_string(json_path)
            .with_context(|| format!("reading atlas {}", json_path.display()))?;
        let file: AtlasFile = serde_json::from_str(&text)
            .with_context(|| format!("parsing atlas {}", json_path.display()))?;
        let image_path = json_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join(file.meta.image);
        let (width, height) = image::image_dimensions(&image_path)
            .with_context(|| format!("reading atlas image {}", image_path.display()))?;
        if (width, height) != (file.meta.size.w, file.meta.size.h) {
            bail!(
                "atlas {} declares {}x{}, but its image is {}x{}",
                json_path.display(),
                file.meta.size.w,
                file.meta.size.h,
                width,
                height
            );
        }
        for (name, frame) in &file.frames {
            let rect = frame.frame;
            if rect.x + rect.w > width || rect.y + rect.h > height {
                bail!(
                    "frame {name} in {} lies outside its atlas",
                    json_path.display()
                );
            }
        }
        Ok(Self {
            image_path,
            frames: file.frames,
            width,
            height,
        })
    }

    pub fn frame(&self, name: &str) -> Result<&Frame> {
        self.frames
            .get(name)
            .with_context(|| format!("frame {name} is missing from {}", self.image_path.display()))
    }

    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }

    pub fn load_pixels(&self) -> Result<RgbaImage> {
        image::open(&self.image_path)
            .with_context(|| format!("decoding atlas image {}", self.image_path.display()))
            .map(|image| image.to_rgba8())
    }

    pub fn dimensions(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    pub fn resize_frame(
        &self,
        pixels: &RgbaImage,
        name: &str,
        width: u32,
        height: u32,
    ) -> Result<RgbaImage> {
        let frame = self.frame(name)?;
        if frame.rotated {
            bail!("rotated frame {name} is not supported by the menu renderer yet");
        }
        if width == 0 || height == 0 {
            bail!("cannot render frame {name} at zero size");
        }
        let rect = frame.frame;
        let cropped = image::imageops::crop_imm(pixels, rect.x, rect.y, rect.w, rect.h).to_image();
        Ok(image::imageops::resize(
            &cropped,
            width,
            height,
            FilterType::Lanczos3,
        ))
    }
}

#[derive(Deserialize)]
struct AtlasFile {
    frames: BTreeMap<String, Frame>,
    meta: Meta,
}

#[derive(Deserialize)]
struct Meta {
    image: String,
    size: Size,
}

#[derive(Deserialize)]
struct Size {
    w: u32,
    h: u32,
}

impl<'de> Deserialize<'de> for Frame {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct RawFrame {
            frame: Rect,
            #[serde(default)]
            rotated: bool,
            #[serde(rename = "spriteSourceSize")]
            sprite_source: Rect,
            #[serde(rename = "sourceSize")]
            source_size: Size,
        }

        let raw = RawFrame::deserialize(deserializer)?;
        Ok(Self {
            frame: raw.frame,
            sprite_source: raw.sprite_source,
            source_size: (raw.source_size.w, raw.source_size.h),
            rotated: raw.rotated,
        })
    }
}

impl<'de> Deserialize<'de> for Rect {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct RawRect {
            x: u32,
            y: u32,
            w: u32,
            h: u32,
        }

        let raw = RawRect::deserialize(deserializer)?;
        Ok(Self {
            x: raw.x,
            y: raw.y,
            w: raw.w,
            h: raw.h,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_original_texture_packer_fields() {
        let json = r#"{
            "frames": {
                "game-ui-asset-6.png": {
                    "frame": {"x":1062,"y":1526,"w":945,"h":424},
                    "rotated": false,
                    "trimed": false,
                    "spriteSourceSize": {"x":0,"y":0,"w":945,"h":424},
                    "sourceSize": {"w":945,"h":424}
                }
            },
            "meta": {
                "image": "game_ui_atlas_1.png",
                "format": "RGBA8888",
                "size": {"w":2048,"h":2048},
                "scale": 1
            }
        }"#;
        let atlas: AtlasFile = serde_json::from_str(json).expect("valid original atlas JSON");
        let frame = &atlas.frames["game-ui-asset-6.png"];
        assert_eq!(
            frame.frame,
            Rect {
                x: 1062,
                y: 1526,
                w: 945,
                h: 424
            }
        );
        assert_eq!(frame.source_size, (945, 424));
        assert!(!frame.rotated);
        assert_eq!(atlas.meta.size.w, 2048);
    }
}
