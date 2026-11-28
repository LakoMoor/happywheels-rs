//! Editing inside a group (the original's group canvas, opened by a double click): the members are laid out where the group
//! shows them, the usual tools work on them alone, and closing turns them back into the group's own coordinates.

use std::borrow::Cow;

use super::{add, from_group, number, position, turn, Editor, Point, Tool};
use crate::editor::{Document, Item, Ref, Value};

pub struct GroupEdit {
    /// The group's index in the outer document.
    pub group: usize,
    /// The document as it was before opening.
    outer: Document,
    undo: Vec<Document>,
    redo: Vec<Document>,
}

/// Store a value past the input field's limits (the original's `…Unbound` setters).
pub(super) fn set_unbound(item: &mut Item, name: &str, value: f64) {
    if let Some(i) = item.spec().props.iter().position(|p| p.0 == name) {
        item.values[i] = Value::Num(value);
    }
}

/// `rebuild`: the members go back into group coordinates, the group itself staying put. A group left empty disappears, and
/// one left without shapes loses its joints (it can no longer take any).
fn merged(edit: &GroupEdit, inner: &Document) -> (Document, bool) {
    let mut doc = edit.outer.clone();
    // level settings can be changed while inside
    doc.info = inner.info.clone();
    let g = doc.groups[edit.group].clone();
    let local = |item: &Item| -> Item {
        let mut item = item.clone();
        item.grouped = true;
        let world = position(&item);
        let p: Point = add(turn([world[0] - g.x, world[1] - g.y], -g.angle), [-g.offset[0], -g.offset[1]]);
        set_unbound(&mut item, "x", p[0]);
        set_unbound(&mut item, "y", p[1]);
        if item.get("angle").is_some() {
            let a = number(&item, "angle") - g.angle;
            set_unbound(&mut item, "angle", a);
        }
        item
    };
    doc.groups[edit.group].shapes = inner.shapes.iter().map(local).collect();
    doc.groups[edit.group].specials = inner.specials.iter().map(local).collect();
    let empty = inner.shapes.is_empty() && inner.specials.is_empty();
    if inner.shapes.is_empty() {
        let this = Some(Ref::Group(edit.group));
        for j in &mut doc.joints {
            j.body1 = j.body1.filter(|&b| Some(b) != this);
            j.body2 = j.body2.filter(|&b| Some(b) != this);
        }
    }
    (doc, empty)
}

impl Editor {
    /// The group being edited, if any.
    pub fn editing_group(&self) -> Option<usize> {
        self.group_edit.as_ref().map(|e| e.group)
    }

    /// The whole level, also while a group is open.
    pub fn document(&self) -> Cow<'_, Document> {
        let Some(edit) = &self.group_edit else { return Cow::Borrowed(&self.doc) };
        let (doc, empty) = merged(edit, &self.doc);
        if !empty {
            return Cow::Owned(doc);
        }
        let mut closed = Editor::new(doc);
        closed.remove(&[Ref::Group(edit.group)]);
        Cow::Owned(closed.doc)
    }

    /// The outer level without the open group, to draw behind its members.
    pub(super) fn surroundings(&self) -> Option<&Document> {
        self.group_edit.as_ref().map(|e| &e.outer)
    }

    pub fn open_group(&mut self, index: usize) {
        if self.group_edit.is_some() || index >= self.doc.groups.len() {
            return;
        }
        let outer = self.doc.clone();
        let g = &outer.groups[index];
        let world = |item: &Item| -> Item {
            let mut item = item.clone();
            let p = from_group(g, position(&item));
            set_unbound(&mut item, "x", p[0]);
            set_unbound(&mut item, "y", p[1]);
            if item.get("angle").is_some() {
                let a = number(&item, "angle") + g.angle;
                set_unbound(&mut item, "angle", a);
            }
            item
        };
        let inner = Document { info: outer.info.clone(), shapes: g.shapes.iter().map(world).collect(), specials: g.specials.iter().map(world).collect(), ..Document::default() };
        self.doc = inner;
        self.group_edit = Some(Box::new(GroupEdit { group: index, outer, undo: std::mem::take(&mut self.undo), redo: std::mem::take(&mut self.redo) }));
        self.selection.clear();
        self.vertex_edit = None;
        self.series = None;
        self.drag = None;
        if matches!(self.tool, Tool::Joint | Tool::Trigger) {
            self.tool = Tool::Arrow;
        }
    }

    /// Return to the level; whatever was done inside is one undo step.
    pub fn close_group(&mut self) {
        let Some(edit) = self.group_edit.take() else { return };
        let (doc, empty) = merged(&edit, &self.doc);
        let changed = !self.undo.is_empty();
        let index = edit.group;
        self.undo = edit.undo;
        self.redo = edit.redo;
        self.vertex_edit = None;
        self.series = None;
        self.drag = None;
        if !changed {
            // nothing was touched: keep the very numbers there were
            let info = self.doc.info.clone();
            self.doc = edit.outer;
            self.doc.info = info;
            self.selection = vec![Ref::Group(index)];
            return;
        }
        self.doc = edit.outer;
        self.checkpoint();
        self.doc = doc;
        self.selection = vec![Ref::Group(index)];
        if empty {
            self.selection.clear();
            self.remove(&[Ref::Group(index)]);
        }
    }
}
