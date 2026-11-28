//! The level editor in the browser: the core's [`Editor`] with a window onto the canvas. The page supplies the toolbar and
//! the property fields; the window itself is drawn like the game.

use happywheels_core::{
    editor::{schema, Document, Value},
    editor_session::{Editor, Preview, Tool, View},
    sprites::{SpriteAssets, Sprites},
};
use image::RgbaImage;

pub struct EditorUi {
    editor: Editor,
    view: View,
    preview: Preview,
    /// Special art without any rider bound to it.
    sprites: Option<Sprites>,
    /// The window must be drawn again.
    dirty: bool,
}

impl EditorUi {
    /// Open the editor on a level, or on an empty one when it cannot be read.
    pub fn new(xml: &str, width: u32, height: u32) -> EditorUi {
        let editor = Editor::new(Document::parse(xml).unwrap_or_default());
        let camera = [editor.doc.info.x, editor.doc.info.y];
        EditorUi { editor, view: View { camera, zoom: 2.0, width, height }, preview: Preview::default(), sprites: None, dirty: true }
    }

    pub fn use_assets(&mut self, assets: &SpriteAssets) {
        self.sprites = Sprites::open_from(assets, 1);
        self.preview = Preview::default();
        self.dirty = true;
    }

    pub fn redraw(&mut self) {
        self.dirty = true;
    }

    /// The window, when it changed since the last call.
    pub fn frame(&mut self, width: u32, height: u32) -> Option<RgbaImage> {
        if (width, height) != (self.view.width, self.view.height) {
            (self.view.width, self.view.height) = (width, height);
            self.dirty = true;
        }
        std::mem::take(&mut self.dirty).then(|| self.editor.render(self.view, &mut self.preview, self.sprites.as_ref()))
    }

    /// `kind`: 0 down, 1 move, 2 up; position in canvas element pixels.
    pub fn pointer(&mut self, kind: u8, x: f64, y: f64, shift: bool) {
        // whole canvas pixels, so that saved levels stay tidy
        let p = self.view.canvas([x, y]).map(f64::round);
        // the view's zoom 2 is the original's 100%
        self.editor.snap = 10.0 / self.view.zoom;
        match kind {
            0 => self.editor.pointer_down(p, shift),
            1 => self.editor.pointer_move(p),
            // hovering matters only to the NPC poser
            3 => {
                self.dirty |= self.editor.is_posing() && self.editor.pose_move(p);
                return;
            }
            _ => self.editor.pointer_up(p),
        }
        self.dirty = true;
    }

    pub fn double_click(&mut self, x: f64, y: f64) {
        self.editor.double_click(self.view.canvas([x, y]));
        self.dirty = true;
    }

    pub fn modifiers(&mut self, shift: bool, alt: bool) {
        (self.editor.shift, self.editor.alt) = (shift, alt);
    }

    pub fn pan(&mut self, dx: f64, dy: f64) {
        self.view.camera = [(self.view.camera[0] + dx / self.view.zoom).clamp(0.0, 20_000.0), (self.view.camera[1] + dy / self.view.zoom).clamp(0.0, 10_000.0)];
        self.dirty = true;
    }

    pub fn zoom(&mut self, factor: f64, x: f64, y: f64) {
        self.view.zoom_by(factor, [x, y]);
        self.dirty = true;
    }

    pub fn key(&mut self, code: u32, shift: bool, ctrl: bool) -> bool {
        let used = match code {
            // + and - zoom about the middle
            61 | 171 | 187 | 189 | 173 => {
                let factor = if matches!(code, 189 | 173) { 0.5 } else { 2.0 };
                self.zoom(factor, f64::from(self.view.width) / 2.0, f64::from(self.view.height) / 2.0);
                true
            }
            // paste without shift lands in the middle of the window
            86 if !shift => {
                self.editor.paste(None);
                if let Some(c) = self.editor.selection_centre() {
                    self.editor.nudge([self.view.camera[0] - c[0], self.view.camera[1] - c[1]]);
                }
                true
            }
            _ => self.editor.key(code, shift, ctrl),
        };
        self.dirty |= used;
        used
    }

    pub fn command(&mut self, name: &str) {
        let e = &mut self.editor;
        match name {
            "undo" => e.undo(),
            "redo" => e.redo(),
            "copy" => e.copy(),
            "paste" => {
                self.key(86, false, false);
            }
            "delete" => e.delete(),
            "group" => e.group(),
            "ungroup" => e.ungroup(),
            "link" => e.link_targets(),
            "up" => e.change_depth(true),
            "down" => e.change_depth(false),
            "finish" => e.finish_drawing(false),
            "start" => {
                [e.doc.info.x, e.doc.info.y] = self.view.camera.map(f64::round);
            }
            "new" => *self = EditorUi { sprites: self.sprites.take(), ..EditorUi::new("", self.view.width, self.view.height) },
            _ => return,
        }
        self.dirty = true;
    }

    /// `tool` 0..8; `shape`, `special`, `sliding` are the options of the placing tools.
    pub fn tool(&mut self, tool: usize, shape: usize, special: usize, sliding: bool) {
        let e = &mut self.editor;
        e.set_tool(Tool::ALL[tool.min(7)]);
        e.shape_kind = shape.min(2);
        e.special_kind = special.min(schema().specials.len() - 1);
        e.sliding = sliding;
        self.dirty = true;
    }

    pub fn set_property(&mut self, name: &str, value: &str) {
        self.editor.set_property(name, value);
        self.dirty = true;
    }

    /// Level settings: `character`, `forceCharacter`, `hideVehicle`, `backdrop`, `background`.
    pub fn set_info(&mut self, name: &str, value: &str) {
        let info = &mut self.editor.doc.info;
        let n: f64 = value.parse().unwrap_or(0.0);
        match name {
            "character" => info.character = n.clamp(1.0, 11.0),
            "forceCharacter" => info.force_character = value == "true",
            "hideVehicle" => info.hide_vehicle = value == "true",
            "backdrop" => info.backdrop = n.clamp(0.0, 2.0),
            "background" => info.background = n.clamp(0.0, 16_777_215.0),
            _ => return,
        }
        self.dirty = true;
    }

    pub fn load(&mut self, xml: &str) -> Result<(), String> {
        let doc = Document::parse(xml).map_err(|e| format!("{e:#}"))?;
        self.editor = Editor::new(doc);
        self.view.camera = [self.editor.doc.info.x, self.editor.doc.info.y];
        self.dirty = true;
        Ok(())
    }

    /// The character the level insists on.
    pub fn forced_character(&self) -> Option<usize> {
        let info = &self.editor.doc.info;
        info.force_character.then(|| (info.character as usize).clamp(1, 11))
    }

    pub fn xml(&self) -> String {
        self.editor.document().to_xml()
    }

    /// `what`: 0 choose action `value`, 1 add an action, 2 remove it, 3 set parameter `param`, 4 remove the target.
    pub fn trigger_edit(&mut self, what: u8, target: usize, action: usize, param: usize, value: f64) {
        let e = &mut self.editor;
        match what {
            0 => e.set_action(target, action, value.max(0.0) as usize),
            1 => e.add_action(target),
            2 => e.remove_action(target, action),
            3 => e.set_action_param(target, action, param, value),
            _ => e.remove_target(target),
        }
        self.dirty = true;
    }

    /// What the page shows beside the window.
    pub fn state(&self) -> String {
        let e = &self.editor;
        let value = |v: &Value| match v {
            Value::Null => serde_json::Value::Bool(false),
            Value::Bool(b) => (*b).into(),
            Value::Num(n) => (*n).into(),
            Value::Text(s) => s.clone().into(),
        };
        let info = &e.doc.info;
        serde_json::json!({
            "tool": Tool::ALL.iter().position(|&t| t == e.tool),
            "selected": e.selection.len(),
            "editingGroup": e.editing_group(),
            "editingVertices": e.vertex_edit.as_ref().map(|v| v.selected.len()),
            "targets": e.trigger_targets().iter().map(|t| serde_json::json!({ "label": t.label, "choices": t.choices, "actions": t.actions })).collect::<Vec<_>>(),
            "properties": e.properties().iter().map(|(n, v)| (n.clone(), value(v))).collect::<Vec<_>>(),
            "info": { "character": info.character, "forceCharacter": info.force_character, "hideVehicle": info.hide_vehicle, "backdrop": info.backdrop, "background": info.background },
        })
        .to_string()
    }

    pub fn special_names() -> String {
        serde_json::to_string(&schema().specials.iter().map(|c| c.name.trim_end_matches("Ref")).collect::<Vec<_>>()).unwrap_or_default()
    }
}
