//! The arrow tool's NPC poser (`openPoser`): double-click an NPC, drag its limbs, double-click again.

use super::{Editor, Point, Ref, Tool};
use crate::editor::Value;
use crate::poser::{PROPERTIES, Poser};

/// The NPC being posed.
pub struct Posing {
    pub special: usize,
    pub poser: Poser,
    /// The segment being dragged.
    pub drag: Option<usize>,
    /// The pointer is over the far arm: chest and pelvis turn translucent.
    pub see_through: bool,
}

/// Where the poser's clip holder stands on the canvas.
pub struct Holder {
    pub at: Point,
    /// Degrees.
    pub angle: f64,
    pub scale: [f64; 2],
    pub char_index: u32,
}

impl Editor {
    pub(super) fn open_poser(&mut self, special: usize) -> bool {
        let Some(item) = self.doc.specials.get(special).filter(|i| i.spec().name == "NPCharacterRef") else { return false };
        let number = |name| item.get(name).map_or(0.0, Value::number);
        let Some(poser) = Poser::new(number("charIndex") as u32, PROPERTIES.map(number)) else { return false };
        self.selection.clear();
        self.drag = None;
        self.series = None;
        self.posing = Some(Posing { special, poser, drag: None, see_through: false });
        true
    }

    pub fn holder(&self) -> Option<Holder> {
        let item = self.doc.specials.get(self.posing.as_ref()?.special)?;
        let number = |name| item.get(name).map_or(0.0, Value::number);
        let flip = if item.get("reverse").is_some_and(Value::truthy) { -1.0 } else { 1.0 };
        Some(Holder { at: [number("x"), number("y")], angle: number("angle"), scale: [0.5 * flip, 0.5], char_index: number("charIndex") as u32 })
    }

    /// A canvas point in the coordinates of the posed clip.
    fn in_holder(&self, p: Point) -> Option<Point> {
        let h = self.holder()?;
        let (s, c) = h.angle.to_radians().sin_cos();
        let (x, y) = (p[0] - h.at[0], p[1] - h.at[1]);
        Some([(x * c + y * s) / h.scale[0], (y * c - x * s) / h.scale[1]])
    }

    pub(super) fn pose_down(&mut self, p: Point) {
        let Some(local) = self.in_holder(p) else { return };
        let Some(posing) = self.posing.as_mut() else { return };
        posing.drag = posing.poser.pick(local);
        self.pose_move(p);
    }

    /// Dragging solves the limb towards the pointer; hovering only changes what is see-through.
    pub fn pose_move(&mut self, p: Point) -> bool {
        let Some(local) = self.in_holder(p) else { return false };
        let Some(posing) = self.posing.as_mut() else { return false };
        match posing.drag {
            Some(which) => {
                posing.poser.drag(which, local);
                true
            }
            None => {
                // segments 3 and 4 are the far arm
                let over = matches!(posing.poser.pick(local), Some(3 | 4));
                std::mem::replace(&mut posing.see_through, over) != over
            }
        }
    }

    /// Releasing stores the angles of the dragged segment and of its parent, as one undo step.
    pub(super) fn pose_up(&mut self) {
        let Some(posing) = self.posing.as_mut() else { return };
        let Some(which) = posing.drag.take() else { return };
        let special = posing.special;
        let angles: Vec<(&str, f64)> = [Some(which), Poser::parent(which)].into_iter().flatten().map(|i| (PROPERTIES[i], posing.poser.angle(i))).collect();
        let Some(item) = self.doc.specials.get(special) else { return };
        if angles.iter().all(|&(name, a)| item.get(name).map(Value::number) == Some(a)) {
            return;
        }
        self.checkpoint();
        for (name, a) in angles {
            self.doc.specials[special].set(name, Value::Num(a));
        }
    }

    pub(super) fn close_poser(&mut self) {
        self.posing = None;
    }

    /// Tool changes, undo and redo close the poser.
    pub fn is_posing(&self) -> bool {
        self.posing.is_some() && self.tool == Tool::Arrow
    }

    /// The NPC the poser stands in for, which the window does not draw.
    pub(super) fn posed_special(&self) -> Option<Ref> {
        self.posing.as_ref().map(|p| Ref::Special(p.special))
    }
}
