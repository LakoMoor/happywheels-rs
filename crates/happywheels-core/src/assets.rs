use std::{
    fs,
    path::{Path, PathBuf},
};

use ab_glyph::FontArc;
use anyhow::{anyhow, bail, Context, Result};
use serde::Serialize;

use crate::{atlas::Atlas, shapes};

#[derive(Debug, Clone)]
pub struct AssetRoot {
    root: PathBuf,
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct ContentReport {
    pub atlases: usize,
    pub atlas_frames: usize,
    pub shape_files: usize,
    pub shape_paths: usize,
    pub fonts: usize,
}

impl AssetRoot {
    pub fn new(root: impl Into<PathBuf>) -> Result<Self> {
        let root = root.into();
        if !root.is_dir() {
            bail!("asset directory {} does not exist", root.display());
        }
        Ok(Self { root })
    }

    pub fn path(&self) -> &Path {
        &self.root
    }

    pub fn atlas(&self, relative: &str) -> Result<Atlas> {
        Atlas::open(self.root.join(relative))
    }

    pub fn font(&self, relative: &str) -> Result<FontArc> {
        let path = self.root.join(relative);
        let bytes = fs::read(&path).with_context(|| format!("reading font {}", path.display()))?;
        FontArc::try_from_vec(bytes)
            .map_err(|error| anyhow!("decoding font {}: {error:?}", path.display()))
    }

    pub fn check_content(&self) -> Result<ContentReport> {
        let animate = self.root.join("animate");
        let mut atlas_paths = Vec::new();
        collect_extensions(&animate, "json", &mut atlas_paths)?;
        atlas_paths.sort();

        let mut report = ContentReport {
            atlases: 0,
            atlas_frames: 0,
            shape_files: 0,
            shape_paths: 0,
            fonts: 0,
        };
        for path in atlas_paths {
            let atlas = Atlas::open(&path)?;
            report.atlases += 1;
            report.atlas_frames += atlas.frame_count();
        }

        let shapes_dir = self.root.join("shapes");
        let mut shape_paths = Vec::new();
        collect_extensions(&shapes_dir, "txt", &mut shape_paths)?;
        shape_paths.sort();
        for path in shape_paths {
            let text = fs::read_to_string(&path)
                .with_context(|| format!("reading shapes {}", path.display()))?;
            report.shape_files += 1;
            report.shape_paths += shapes::parse(&text)?.len();
        }

        let fonts_dir = self.root.join("fonts");
        let mut font_paths = Vec::new();
        collect_extensions(&fonts_dir, "otf", &mut font_paths)?;
        for path in font_paths {
            let relative = path
                .strip_prefix(&self.root)
                .map_err(|error| anyhow!(error))?;
            self.font(&relative.to_string_lossy())?;
            report.fonts += 1;
        }
        Ok(report)
    }
}

fn collect_extensions(directory: &Path, extension: &str, output: &mut Vec<PathBuf>) -> Result<()> {
    let entries = fs::read_dir(directory)
        .with_context(|| format!("reading asset directory {}", directory.display()))?;
    for entry in entries {
        let entry = entry.with_context(|| format!("reading entry in {}", directory.display()))?;
        let path = entry.path();
        if path.is_dir() {
            collect_extensions(&path, extension, output)?;
        } else if path.extension().is_some_and(|value| value == extension) {
            output.push(path);
        }
    }
    Ok(())
}
