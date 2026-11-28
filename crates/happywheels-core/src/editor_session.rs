//! Editing a [`Document`] the way the original editor's tools do: selection, moving, sizing, turning, depth, copy and paste,
//! groups, placing objects, undo. Coordinates are the level's canvas pixels (20000 × 10000).
//!
//! Differences from the original editor: no on-canvas handles, vertex or curve editing, NPC poser, or editing inside a group;
//! polygons and art are drawn with straight edges only; special objects are picked by their unrotated bounds turned with them.

use crate::editor::{Document, Group, Item, Joint, Ref, Target, Trigger, Value, Vertices};
use image::{Rgba, RgbaImage};
use imageproc::drawing::{draw_hollow_circle_mut, draw_line_segment_mut};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    Arrow,
    Shape,
    Polygon,
    Art,
    Joint,
    Special,
    Text,
    Trigger,
}

impl Tool {
    /// In the order of the original's number keys 1..8.
    pub const ALL: [Tool; 8] = [Tool::Arrow, Tool::Shape, Tool::Polygon, Tool::Art, Tool::Joint, Tool::Special, Tool::Text, Tool::Trigger];
}

/// Bounds of a freshly placed special relative to its position: left, top, width, height (dumped from the original editor;
/// text boxes and bottles, which it could not measure headless, are estimates).
const SPECIAL_BOUNDS: [[f64; 4]; 36] = [
    [-81.0, -59.0, 162.0, 129.0], [-78.0, -6.0, 155.84, 55.95], [-13.0, -2.0, 25.0, 3.0], [-200.0, -16.0, 400.0, 32.0], [-18.0, -200.0, 36.0, 400.0],
    [-175.0, -20.0, 350.0, 40.0], [-150.0, -24.97, 300.0, 69.95], [-424.79, -24.99, 850.0, 450.0], [-150.0, -550.0, 300.37, 699.85], [-201.0, -230.0, 401.6, 250.6],
    [-10.0, -10.0, 19.98, 19.98], [-205.0, -201.0, 407.0, 401.0], [-183.0, -93.0, 365.81, 185.99], [0.0, 0.0, 300.0, 595.0], [0.0, 0.0, 726.0, 660.0],
    [-25.0, -137.05, 50.0, 153.3], [0.0, 0.0, 160.0, 20.0], [-9.24, -34.38, 21.34, 129.49], [-5.0, -50.0, 10.0, 100.0], [-21.0, -52.9, 42.05, 87.92],
    [-5.0, -17.0, 10.0, 34.0], [-27.0, -20.05, 55.0, 40.5], [-24.0, -16.05, 48.0, 32.0], [-30.0, -18.0, 60.0, 160.0], [-24.45, -43.0, 63.46, 77.93],
    [-12.0, -12.0, 23.0, 23.0], [-27.2, -42.4, 53.0, 77.4], [-125.0, -9.0, 250.0, 18.0], [-8.0, -10.0, 16.5, 24.25], [-16.0, -86.0, 30.97, 94.3],
    [-5.0, -3.5, 9.0, 186.0], [-21.0, -20.0, 40.8, 40.8], [-18.7, -15.0, 41.0, 30.5], [-41.0, -193.0, 82.47, 244.49], [-13.0, -43.0, 25.97, 86.25],
    [-175.0, -20.0, 350.0, 40.0],
];

/// Special types the original can always put into a group (I-beam, text box, sign post, blade); the others only as
/// non-interactive scenery.
const GROUPABLE: [usize; 4] = [3, 16, 23, 34];
/// Special types a joint attaches to besides those with an "interactive" property.
const JOINABLE: [usize; 5] = [0, 1, 3, 4, 28];

const TEXT: usize = 16;
const JOINT_RADIUS: f64 = 10.0;
/// How near a click closes a polygon onto its first vertex (`SNAP_DISTANCE`).
const SNAP: f64 = 5.0;
/// Art outlines; polygons stop at the physics engine's limit.
const MAX_VERTICES: usize = 100;
const MAX_POLYGON_VERTICES: usize = 10;
/// Shortest edge of a polygon.
const MIN_EDGE: f64 = 5.0;
const UNDO_LIMIT: usize = 200;

type Point = [f64; 2];

fn turn(p: Point, degrees: f64) -> Point {
    let (s, c) = degrees.to_radians().sin_cos();
    [p[0] * c - p[1] * s, p[0] * s + p[1] * c]
}

fn add(a: Point, b: Point) -> Point {
    [a[0] + b[0], a[1] + b[1]]
}

fn number(item: &Item, name: &str) -> f64 {
    item.get(name).map_or(0.0, Value::number)
}

fn position(item: &Item) -> Point {
    [number(item, "x"), number(item, "y")]
}

fn set_position(item: &mut Item, p: Point) {
    item.set("x", Value::Num(p[0]));
    item.set("y", Value::Num(p[1]));
}

fn rect(l: f64, t: f64, w: f64, h: f64) -> Vec<Point> {
    vec![[l, t], [l + w, t], [l + w, t + h], [l, t + h]]
}

/// The object's outline around its own position, unrotated.
fn local_outline(item: &Item) -> Vec<Point> {
    let (w, h) = (number(item, "shapeWidth"), number(item, "shapeHeight"));
    if item.special {
        let [l, t, bw, bh] = SPECIAL_BOUNDS[item.class];
        if item.class == TEXT {
            let size = number(item, "fontSize");
            let text = match item.get("caption") {
                Some(Value::Text(s)) => s.clone(),
                _ => String::new(),
            };
            let longest = text.lines().map(|l| l.chars().count()).max().unwrap_or(1).max(1);
            return rect(0.0, 0.0, longest as f64 * size * 0.55, text.lines().count().max(1) as f64 * size * 1.2);
        }
        // sized specials scale their bounds with their width and height
        let (sx, sy) = match (item.get("shapeWidth"), item.get("shapeHeight")) {
            (Some(_), Some(_)) => (w / bw.max(1.0), h / bh.max(1.0)),
            _ => (1.0, 1.0),
        };
        return rect(l * sx, t * sy, bw * sx, bh * sy);
    }
    match item.class {
        1 => (0..24).map(|i| turn([w / 2.0, 0.0], f64::from(i) * 15.0)).collect(),
        2 => vec![[0.0, -h * 2.0 / 3.0], [w / 2.0, h / 3.0], [-w / 2.0, h / 3.0]],
        3 | 4 => {
            let Some(v) = item.vertices.as_ref() else { return rect(-w / 2.0, -h / 2.0, w, h) };
            let scale = [0, 1].map(|axis| {
                let (lo, hi) = v.points.iter().fold((0.0f64, 0.0f64), |(lo, hi), p| (lo.min(p[0][axis]), hi.max(p[0][axis])));
                if hi > lo { number(item, ["shapeWidth", "shapeHeight"][axis]) / (hi - lo) } else { 1.0 }
            });
            v.points.iter().map(|p| [p[0][0] * scale[0], p[0][1] * scale[1]]).collect()
        }
        _ => rect(-w / 2.0, -h / 2.0, w, h),
    }
}

fn item_outline(item: &Item) -> Vec<Point> {
    let (at, angle) = (position(item), number(item, "angle"));
    local_outline(item).into_iter().map(|p| add(at, turn(p, angle))).collect()
}

/// Where a point of a group's contents lands on the canvas.
fn from_group(g: &Group, p: Point) -> Point {
    add([g.x, g.y], turn(add(g.offset, p), g.angle))
}

fn bounds(points: &[Point]) -> [f64; 4] {
    points.iter().fold([f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY], |b, p| [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])])
}

fn inside(polygon: &[Point], p: Point) -> bool {
    let mut odd = false;
    for i in 0..polygon.len() {
        let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
            odd = !odd;
        }
    }
    odd
}

/// What a click at the polygon tool's cursor does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cursor {
    Add,
    /// Close the outline.
    Complete,
    Nothing,
    /// The point lies straight on past the last edge: it replaces the last vertex.
    ReplaceLast,
}

/// The polygon tool's cursor (its `ENTER_FRAME` handler): where the next vertex would go, keeping the outline convex.
/// Points are relative to the first vertex; `snap` is how near the first vertex counts as closing.
pub fn polygon_cursor(vertices: &[Point], mouse: Point, shift: bool, snap: f64) -> (Point, Cursor) {
    // JavaScript's `Math.round`
    let round = |v: f64| (v + 0.5).floor();
    let mut next = mouse.map(round);
    if shift {
        next = next.map(|v| 10.0 * round(0.1 * v));
    }
    let sub = |a: Point, b: Point| [a[0] - b[0], a[1] - b[1]];
    let dot = |a: Point, b: Point| a[0] * b[0] + a[1] * b[1];
    let length = |a: Point| a[0].hypot(a[1]);
    let unit = |a: Point| {
        let l = length(a);
        if l > 0.0 { [a[0] / l, a[1] / l] } else { a }
    };
    let n = vertices.len();
    if n < 2 {
        let Some(&first) = vertices.first() else { return (next, Cursor::Add) };
        let d = sub(next, first);
        return match length(d) {
            0.0 => (next, Cursor::Nothing),
            l if l < MIN_EDGE => (add(first, unit(d).map(|v| v * MIN_EDGE)).map(round), Cursor::Add),
            _ => (next, Cursor::Add),
        };
    }
    let (first, second, before, last) = (vertices[0], vertices[1], vertices[n - 2], vertices[n - 1]);
    let (along, closing, step) = (unit(sub(last, before)), unit(sub(first, last)), sub(next, last));
    let (forward, towards) = (dot(step, along), dot(step, closing));
    let (outwards, past) = (dot(step, [along[1], -along[0]]), dot(step, [-closing[1], closing[0]]));
    let mut state = Cursor::Add;
    if forward < 0.0 && towards < 0.0 {
        (next, state) = (last, Cursor::Nothing);
    } else if forward >= 0.0 && outwards > 0.0 {
        (next, state) = (add(last, along.map(|v| v * forward)).map(round), Cursor::ReplaceLast);
    } else if (towards >= 0.0 && past > 0.0) || n == MAX_POLYGON_VERTICES {
        (next, state) = (first, Cursor::Complete);
    }
    if state != Cursor::Complete && state != Cursor::Nothing {
        let edge = length(sub(next, last));
        if edge == 0.0 {
            state = Cursor::Nothing;
        } else if edge < MIN_EDGE {
            (next, state) = (last, Cursor::Nothing);
        }
        if length(sub(first, next)) < snap {
            (next, state) = (first, Cursor::Complete);
        } else if n > 2 {
            let opening = unit(sub(first, second));
            if dot(sub(next, first), [-opening[1], opening[0]]) > 0.0 {
                (next, state) = (first, Cursor::Complete);
            }
        }
    }
    if n == 2 && state == Cursor::Complete {
        (next, state) = (last, Cursor::Nothing);
    }
    (next, state)
}

/// What a click would do, for the front end's pointer handling.
#[derive(Clone, Copy, Debug)]
enum Drag {
    Move { last: Point, moved: bool },
    Box { from: Point, to: Point },
}

#[derive(Clone, Debug)]
enum Copied {
    Item(Item),
    Group(Group),
    Joint(Joint),
    Trigger(Trigger),
}

pub struct Editor {
    pub doc: Document,
    pub selection: Vec<Ref>,
    pub tool: Tool,
    /// Shape tool: 0 rectangle, 1 circle, 2 triangle.
    pub shape_kind: usize,
    pub special_kind: usize,
    /// Joint tool: sliding instead of pin.
    pub sliding: bool,
    /// Vertices of the polygon or art being drawn, on the canvas.
    pub drawing: Vec<Point>,
    /// How near the first vertex a click closes an outline, in canvas pixels (5 at the original's 100% zoom).
    pub snap: f64,
    undo: Vec<Document>,
    redo: Vec<Document>,
    drag: Option<Drag>,
    clipboard: Vec<(Ref, Copied)>,
    /// The last edit, so that a run of arrow-key steps is one undo step.
    series: Option<(u8, Vec<Ref>)>,
}

impl Default for Editor {
    fn default() -> Editor {
        Editor::new(Document::default())
    }
}

impl Editor {
    pub fn new(doc: Document) -> Editor {
        Editor { doc, selection: Vec::new(), tool: Tool::Arrow, shape_kind: 0, special_kind: 0, sliding: false, drawing: Vec::new(), snap: SNAP, undo: Vec::new(), redo: Vec::new(), drag: None, clipboard: Vec::new(), series: None }
    }

    pub fn set_tool(&mut self, tool: Tool) {
        if tool != self.tool {
            self.tool = tool;
            self.drawing.clear();
            self.selection.clear();
            self.drag = None;
            self.series = None;
        }
    }

    fn checkpoint(&mut self) {
        self.undo.push(self.doc.clone());
        if self.undo.len() > UNDO_LIMIT {
            self.undo.remove(0);
        }
        self.redo.clear();
        self.series = None;
    }

    /// One undo step for consecutive edits of the same kind on the same selection.
    fn checkpoint_series(&mut self, kind: u8) {
        let key = (kind, self.selection.clone());
        if self.series.as_ref() != Some(&key) {
            self.checkpoint();
            self.series = Some(key);
        }
    }

    pub fn undo(&mut self) {
        if let Some(d) = self.undo.pop() {
            self.redo.push(std::mem::replace(&mut self.doc, d));
            self.selection.clear();
            self.series = None;
        }
    }

    pub fn redo(&mut self) {
        if let Some(d) = self.redo.pop() {
            self.undo.push(std::mem::replace(&mut self.doc, d));
            self.selection.clear();
            self.series = None;
        }
    }

    /// The outline of anything on the canvas.
    pub fn outline(&self, r: Ref) -> Vec<Point> {
        let d = &self.doc;
        match r {
            Ref::Shape(i) => d.shapes.get(i).map(item_outline),
            Ref::Special(i) => d.specials.get(i).map(item_outline),
            Ref::Group(i) => d.groups.get(i).map(|g| {
                let points: Vec<Point> = g.shapes.iter().chain(&g.specials).flat_map(item_outline).map(|p| from_group(g, p)).collect();
                if points.is_empty() {
                    return rect(g.x - JOINT_RADIUS, g.y - JOINT_RADIUS, 2.0 * JOINT_RADIUS, 2.0 * JOINT_RADIUS);
                }
                let [l, t, r, b] = bounds(&points);
                rect(l, t, r - l, b - t)
            }),
            Ref::Joint(i) => d.joints.get(i).map(|j| rect(j.x - JOINT_RADIUS, j.y - JOINT_RADIUS, 2.0 * JOINT_RADIUS, 2.0 * JOINT_RADIUS)),
            Ref::Trigger(i) => d.triggers.get(i).map(|t| rect(-t.width / 2.0, -t.height / 2.0, t.width, t.height).into_iter().map(|p| add([t.x, t.y], turn(p, t.angle))).collect()),
        }
        .unwrap_or_default()
    }

    fn all(&self) -> impl Iterator<Item = Ref> {
        let d = &self.doc;
        // bottom to top, as the original stacks its layers
        (0..d.shapes.len())
            .map(Ref::Shape)
            .chain((0..d.specials.len()).map(Ref::Special))
            .chain((0..d.groups.len()).map(Ref::Group))
            .chain((0..d.joints.len()).map(Ref::Joint))
            .chain((0..d.triggers.len()).map(Ref::Trigger))
    }

    fn hits(&self, r: Ref, p: Point) -> bool {
        match r {
            Ref::Group(i) => self.doc.groups.get(i).is_some_and(|g| {
                g.shapes.iter().chain(&g.specials).any(|x| inside(&item_outline(x).into_iter().map(|q| from_group(g, q)).collect::<Vec<_>>(), p))
            }),
            _ => inside(&self.outline(r), p),
        }
    }

    /// Everything under `p`, topmost first.
    pub fn under(&self, p: Point) -> Vec<Ref> {
        let mut hit: Vec<Ref> = self.all().filter(|&r| self.hits(r, p)).collect();
        hit.reverse();
        hit
    }

    /// The selection rectangle being dragged.
    pub fn selection_box(&self) -> Option<[Point; 2]> {
        match self.drag {
            Some(Drag::Box { from, to }) => Some([from, to]),
            _ => None,
        }
    }

    pub fn pointer_down(&mut self, p: Point, shift: bool) {
        self.series = None;
        match self.tool {
            Tool::Arrow => match self.under(p).first() {
                Some(&r) => {
                    if let Some(i) = self.selection.iter().position(|&s| s == r) {
                        if shift {
                            self.selection.remove(i);
                            return;
                        }
                    } else {
                        if !shift {
                            self.selection.clear();
                        }
                        self.selection.push(r);
                    }
                    self.drag = Some(Drag::Move { last: p, moved: false });
                }
                None => {
                    if !shift {
                        self.selection.clear();
                    }
                    self.drag = Some(Drag::Box { from: p, to: p });
                }
            },
            Tool::Shape => self.place_shape(p),
            Tool::Special => self.place_special(self.special_kind, p),
            Tool::Text => self.place_special(TEXT, p),
            Tool::Joint => self.place_joint(p),
            Tool::Trigger => self.place_trigger(p),
            Tool::Polygon | Tool::Art => self.add_vertex(p, shift),
        }
    }

    pub fn pointer_move(&mut self, p: Point) {
        match self.drag {
            Some(Drag::Move { last, moved }) => {
                let d = [p[0] - last[0], p[1] - last[1]];
                if d == [0.0, 0.0] {
                    return;
                }
                if !moved {
                    self.checkpoint();
                }
                self.shift_selected(d);
                self.drag = Some(Drag::Move { last: p, moved: true });
            }
            Some(Drag::Box { from, .. }) => self.drag = Some(Drag::Box { from, to: p }),
            None => {}
        }
    }

    pub fn pointer_up(&mut self, p: Point) {
        self.pointer_move(p);
        if let Some(Drag::Box { from, to }) = self.drag {
            let [l, t, r, b] = bounds(&[from, to]);
            // the box takes what lies entirely inside it
            let found: Vec<Ref> = self
                .all()
                .filter(|&x| {
                    let o = self.outline(x);
                    !o.is_empty() && o.iter().all(|q| q[0] >= l && q[0] <= r && q[1] >= t && q[1] <= b)
                })
                .filter(|x| !self.selection.contains(x))
                .collect();
            self.selection.extend(found);
        }
        self.drag = None;
    }

    /// The original editor's keys (JavaScript key codes). Returns false for keys it does not use.
    pub fn key(&mut self, code: u32, shift: bool, ctrl: bool) -> bool {
        let step = if shift { 10.0 } else { 1.0 };
        match code {
            49..=56 => self.set_tool(Tool::ALL[(code - 49) as usize]),
            8 | 46 if !self.drawing.is_empty() => {
                self.drawing.pop();
            }
            8 | 46 => self.delete(),
            13 => self.finish_drawing(false),
            27 => {
                self.drawing.clear();
                self.selection.clear();
            }
            37 => self.nudge([-step, 0.0]),
            39 => self.nudge([step, 0.0]),
            38 if ctrl => self.change_depth(true),
            40 if ctrl => self.change_depth(false),
            38 => self.nudge([0.0, -step]),
            40 => self.nudge([0.0, step]),
            65 => self.resize(-step, 0.0),
            68 => self.resize(step, 0.0),
            87 => self.resize(0.0, step),
            83 => self.resize(0.0, -step),
            90 if ctrl => self.undo(),
            89 if ctrl => self.redo(),
            90 => self.rotate(-step),
            88 => self.rotate(step),
            67 => self.copy(),
            86 => self.paste(if shift { None } else { Some([0.0, 0.0]) }),
            71 if shift => self.ungroup(),
            71 => self.group(),
            _ => return false,
        }
        true
    }

    fn shift_selected(&mut self, d: Point) {
        for &r in &self.selection {
            match r {
                Ref::Shape(i) => {
                    if let Some(x) = self.doc.shapes.get_mut(i) {
                        set_position(x, add(position(x), d));
                    }
                }
                Ref::Special(i) => {
                    if let Some(x) = self.doc.specials.get_mut(i) {
                        set_position(x, add(position(x), d));
                    }
                }
                Ref::Group(i) => {
                    if let Some(g) = self.doc.groups.get_mut(i) {
                        (g.x, g.y) = (g.x + d[0], g.y + d[1]);
                    }
                }
                Ref::Joint(i) => {
                    if let Some(j) = self.doc.joints.get_mut(i) {
                        (j.x, j.y) = (j.x + d[0], j.y + d[1]);
                    }
                }
                Ref::Trigger(i) => {
                    if let Some(t) = self.doc.triggers.get_mut(i) {
                        (t.x, t.y) = (t.x + d[0], t.y + d[1]);
                    }
                }
            }
        }
    }

    pub fn nudge(&mut self, d: Point) {
        if !self.selection.is_empty() {
            self.checkpoint_series(0);
            self.shift_selected(d);
        }
    }

    pub fn resize(&mut self, dw: f64, dh: f64) {
        if self.selection.is_empty() {
            return;
        }
        self.checkpoint_series(1);
        for &r in &self.selection {
            let item = match r {
                Ref::Shape(i) => self.doc.shapes.get_mut(i),
                Ref::Special(i) => self.doc.specials.get_mut(i),
                Ref::Trigger(i) => {
                    if let Some(t) = self.doc.triggers.get_mut(i) {
                        t.width = (t.width + dw).clamp(5.0, 5000.0);
                        t.height = (t.height + dh).clamp(5.0, 5000.0);
                    }
                    None
                }
                _ => None,
            };
            if let Some(item) = item {
                for (name, d) in [("shapeWidth", dw), ("shapeHeight", dh)] {
                    if d != 0.0 && item.get(name).is_some() {
                        let v = number(item, name) + d;
                        item.set(name, Value::Num(v));
                    }
                }
            }
        }
    }

    pub fn rotate(&mut self, degrees: f64) {
        if self.selection.is_empty() {
            return;
        }
        self.checkpoint_series(2);
        let wrap = |a: f64| if a > 180.0 { a - 360.0 } else if a < -180.0 { a + 360.0 } else { a };
        for &r in &self.selection {
            match r {
                Ref::Shape(i) => {
                    if let Some(x) = self.doc.shapes.get_mut(i) {
                        let a = number(x, "angle") + degrees;
                        x.set("angle", Value::Num(a));
                    }
                }
                Ref::Special(i) => {
                    if let Some(x) = self.doc.specials.get_mut(i) {
                        let a = number(x, "angle") + degrees;
                        x.set("angle", Value::Num(a));
                    }
                }
                Ref::Group(i) => {
                    if let Some(g) = self.doc.groups.get_mut(i) {
                        g.angle = wrap(g.angle + degrees).round();
                    }
                }
                Ref::Trigger(i) => {
                    if let Some(t) = self.doc.triggers.get_mut(i) {
                        t.angle = wrap(t.angle + degrees);
                    }
                }
                Ref::Joint(i) => {
                    if let Some(j) = self.doc.joints.get_mut(i).filter(|j| j.prismatic) {
                        j.axis_angle = wrap(j.axis_angle + degrees);
                    }
                }
            }
        }
    }

    /// Renumber every reference after lists changed; `None` removes it.
    fn remap(&mut self, map: impl Fn(Ref) -> Option<Ref>) {
        for j in &mut self.doc.joints {
            j.body1 = j.body1.and_then(&map);
            j.body2 = j.body2.and_then(&map);
        }
        for t in &mut self.doc.triggers {
            t.targets = std::mem::take(&mut t.targets).into_iter().filter_map(|x| map(x.target).map(|target| Target { target, actions: x.actions })).collect();
        }
        self.selection = std::mem::take(&mut self.selection).into_iter().filter_map(map).collect();
    }

    /// Remove `gone` from their lists and renumber what pointed at the others.
    fn remove(&mut self, gone: &[Ref]) {
        fn keep<T>(list: &mut Vec<T>, make: fn(usize) -> Ref, gone: &[Ref]) -> Vec<Option<usize>> {
            let mut next = 0;
            let table: Vec<Option<usize>> = (0..list.len())
                .map(|i| {
                    (!gone.contains(&make(i))).then(|| {
                        next += 1;
                        next - 1
                    })
                })
                .collect();
            let mut i = 0;
            list.retain(|_| {
                i += 1;
                table[i - 1].is_some()
            });
            table
        }
        let shapes = keep(&mut self.doc.shapes, Ref::Shape, gone);
        let specials = keep(&mut self.doc.specials, Ref::Special, gone);
        let groups = keep(&mut self.doc.groups, Ref::Group, gone);
        let joints = keep(&mut self.doc.joints, Ref::Joint, gone);
        let triggers = keep(&mut self.doc.triggers, Ref::Trigger, gone);
        self.remap(|r| {
            let at = |table: &[Option<usize>], i: usize| table.get(i).copied().flatten();
            Some(match r {
                Ref::Shape(i) => Ref::Shape(at(&shapes, i)?),
                Ref::Special(i) => Ref::Special(at(&specials, i)?),
                Ref::Group(i) => Ref::Group(at(&groups, i)?),
                Ref::Joint(i) => Ref::Joint(at(&joints, i)?),
                Ref::Trigger(i) => Ref::Trigger(at(&triggers, i)?),
            })
        });
    }

    pub fn delete(&mut self) {
        if !self.selection.is_empty() {
            self.checkpoint();
            let gone = std::mem::take(&mut self.selection);
            self.remove(&gone);
        }
    }

    /// Move the selection one place up or down within its layer.
    pub fn change_depth(&mut self, up: bool) {
        if self.selection.is_empty() {
            return;
        }
        self.checkpoint();
        let mut selected = self.selection.clone();
        let index = |r: Ref| match r {
            Ref::Shape(i) | Ref::Special(i) | Ref::Group(i) | Ref::Joint(i) | Ref::Trigger(i) => i,
        };
        // the one nearest the target end goes first
        selected.sort_by_key(|&r| if up { usize::MAX - index(r) } else { index(r) });
        for k in 0..selected.len() {
            let r = selected[k];
            let i = index(r);
            let other = if up { i + 1 } else { i.wrapping_sub(1) };
            let same = |n: usize| match r {
                Ref::Shape(_) => Ref::Shape(n),
                Ref::Special(_) => Ref::Special(n),
                Ref::Group(_) => Ref::Group(n),
                Ref::Joint(_) => Ref::Joint(n),
                Ref::Trigger(_) => Ref::Trigger(n),
            };
            let length = match r {
                Ref::Shape(_) => self.doc.shapes.len(),
                Ref::Special(_) => self.doc.specials.len(),
                Ref::Group(_) => self.doc.groups.len(),
                Ref::Joint(_) => self.doc.joints.len(),
                Ref::Trigger(_) => self.doc.triggers.len(),
            };
            if other >= length || selected.contains(&same(other)) {
                continue;
            }
            match r {
                Ref::Shape(_) => self.doc.shapes.swap(i, other),
                Ref::Special(_) => self.doc.specials.swap(i, other),
                Ref::Group(_) => self.doc.groups.swap(i, other),
                Ref::Joint(_) => self.doc.joints.swap(i, other),
                Ref::Trigger(_) => self.doc.triggers.swap(i, other),
            }
            self.remap(|x| Some(if x == r { same(other) } else if x == same(other) { r } else { x }));
            selected[k] = same(other);
        }
    }

    pub fn copy(&mut self) {
        self.clipboard = self
            .selection
            .iter()
            .filter_map(|&r| {
                Some((
                    r,
                    match r {
                        Ref::Shape(i) => Copied::Item(self.doc.shapes.get(i)?.clone()),
                        Ref::Special(i) => Copied::Item(self.doc.specials.get(i)?.clone()),
                        Ref::Group(i) => Copied::Group(self.doc.groups.get(i)?.clone()),
                        Ref::Joint(i) => Copied::Joint(self.doc.joints.get(i)?.clone()),
                        Ref::Trigger(i) => Copied::Trigger(self.doc.triggers.get(i)?.clone()),
                    },
                ))
            })
            .collect();
    }

    /// Paste the copies, shifted by `by` (the original's "paste in place" is no shift) or, with `None`, kept where they were
    /// for the front end to centre on screen with [`Editor::nudge`].
    pub fn paste(&mut self, by: Option<Point>) {
        if self.clipboard.is_empty() {
            return;
        }
        self.checkpoint();
        let d = by.unwrap_or([0.0, 0.0]);
        let copies = self.clipboard.clone();
        let mut placed: Vec<(Ref, Ref)> = Vec::new();
        // bodies first, so that joints and triggers can point at their copies
        let mut order: Vec<&(Ref, Copied)> = copies.iter().collect();
        order.sort_by_key(|(_, c)| match c {
            Copied::Item(_) | Copied::Group(_) => 0,
            Copied::Joint(_) => 1,
            Copied::Trigger(_) => 2,
        });
        for (from, copy) in order {
            let new = match copy.clone() {
                Copied::Item(mut x) => {
                    let to = add(position(&x), d);
                    set_position(&mut x, to);
                    if x.special {
                        self.doc.specials.push(x);
                        Ref::Special(self.doc.specials.len() - 1)
                    } else {
                        self.doc.shapes.push(x);
                        Ref::Shape(self.doc.shapes.len() - 1)
                    }
                }
                Copied::Group(mut g) => {
                    (g.x, g.y) = (g.x + d[0], g.y + d[1]);
                    self.doc.groups.push(g);
                    Ref::Group(self.doc.groups.len() - 1)
                }
                Copied::Joint(mut j) => {
                    let moved = |b: Option<Ref>| b.map(|b| placed.iter().find(|p| p.0 == b).map_or(b, |p| p.1));
                    (j.x, j.y, j.body1, j.body2) = (j.x + d[0], j.y + d[1], moved(j.body1), moved(j.body2));
                    self.doc.joints.push(j);
                    Ref::Joint(self.doc.joints.len() - 1)
                }
                Copied::Trigger(mut t) => {
                    (t.x, t.y) = (t.x + d[0], t.y + d[1]);
                    for x in &mut t.targets {
                        if let Some(p) = placed.iter().find(|p| p.0 == x.target) {
                            x.target = p.1;
                        }
                    }
                    self.doc.triggers.push(t);
                    Ref::Trigger(self.doc.triggers.len() - 1)
                }
            };
            placed.push((*from, new));
        }
        self.selection = placed.into_iter().map(|p| p.1).collect();
        self.series = None;
    }

    /// The centre of the selection's bounds.
    pub fn selection_centre(&self) -> Option<Point> {
        let points: Vec<Point> = self.selection.iter().flat_map(|&r| self.outline(r)).collect();
        if points.is_empty() {
            return None;
        }
        let [l, t, r, b] = bounds(&points);
        Some([(l + r) / 2.0, (t + b) / 2.0])
    }

    fn groupable(&self, r: Ref) -> bool {
        match r {
            Ref::Shape(i) => i < self.doc.shapes.len(),
            Ref::Special(i) => self.doc.specials.get(i).is_some_and(|s| GROUPABLE.contains(&s.class) || s.get("interactive").is_some_and(|v| !v.truthy())),
            _ => false,
        }
    }

    /// `RefGroup.build`: the group sits at the centre of its members' bounds, which keep their canvas coordinates.
    pub fn group(&mut self) {
        let members: Vec<Ref> = self.selection.iter().copied().filter(|&r| self.groupable(r)).collect();
        if members.is_empty() {
            return;
        }
        self.checkpoint();
        let [l, t, r, b] = bounds(&members.iter().flat_map(|&m| self.outline(m)).collect::<Vec<_>>());
        let centre = [(l + r) / 2.0, (t + b) / 2.0];
        let mut group = Group { x: centre[0], y: centre[1], offset: [-centre[0], -centre[1]], opacity: 100.0, ..Group::default() };
        for &m in &members {
            let (list, i) = match m {
                Ref::Shape(i) => (&self.doc.shapes, i),
                Ref::Special(i) => (&self.doc.specials, i),
                _ => continue,
            };
            let mut item = list[i].clone();
            item.grouped = true;
            if item.special {
                group.specials.push(item);
            } else {
                group.shapes.push(item);
            }
        }
        self.selection.clear();
        self.remove(&members);
        self.doc.groups.push(group);
        self.selection = vec![Ref::Group(self.doc.groups.len() - 1)];
    }

    /// `breakApart`.
    pub fn ungroup(&mut self) {
        let broken: Vec<Ref> = self.selection.iter().copied().filter(|r| matches!(r, Ref::Group(i) if *i < self.doc.groups.len())).collect();
        if broken.is_empty() {
            return;
        }
        self.checkpoint();
        let mut released = Vec::new();
        for &r in &broken {
            let Ref::Group(i) = r else { continue };
            let g = self.doc.groups[i].clone();
            for mut item in g.shapes.clone().into_iter().chain(g.specials.clone()) {
                item.grouped = false;
                let to = from_group(&g, position(&item));
                set_position(&mut item, to);
                if item.get("angle").is_some() {
                    let a = number(&item, "angle") + g.angle;
                    item.set("angle", Value::Num(a));
                }
                released.push(item);
            }
        }
        self.selection.clear();
        self.remove(&broken);
        for item in released {
            if item.special {
                self.doc.specials.push(item);
                self.selection.push(Ref::Special(self.doc.specials.len() - 1));
            } else {
                self.doc.shapes.push(item);
                self.selection.push(Ref::Shape(self.doc.shapes.len() - 1));
            }
        }
    }

    fn place_shape(&mut self, p: Point) {
        let Some(mut item) = Item::new(false, self.shape_kind.min(2), false) else { return };
        self.checkpoint();
        set_position(&mut item, p);
        self.doc.shapes.push(item);
    }

    fn place_special(&mut self, class: usize, p: Point) {
        let Some(mut item) = Item::new(true, class, false) else { return };
        self.checkpoint();
        set_position(&mut item, p);
        self.doc.specials.push(item);
    }

    fn joinable(&self, r: Ref) -> bool {
        match r {
            Ref::Shape(i) => self.doc.shapes.get(i).is_some_and(|s| s.interactive),
            Ref::Special(i) => self.doc.specials.get(i).is_some_and(|s| match s.get("interactive") {
                Some(v) => v.truthy(),
                None => JOINABLE.contains(&s.class),
            }),
            Ref::Group(i) => i < self.doc.groups.len(),
            _ => false,
        }
    }

    /// The joint takes the two topmost joinable objects under it (`identifyBodies`).
    fn place_joint(&mut self, p: Point) {
        self.checkpoint();
        let mut joint = Joint::new(self.sliding);
        (joint.x, joint.y) = (p[0], p[1]);
        let mut bodies = self.under(p).into_iter().filter(|&r| self.joinable(r));
        joint.body1 = bodies.next();
        joint.body2 = bodies.next();
        self.doc.joints.push(joint);
    }

    fn place_trigger(&mut self, p: Point) {
        self.checkpoint();
        self.doc.triggers.push(Trigger {
            x: p[0],
            y: p[1],
            width: 100.0,
            height: 100.0,
            angle: 0.0,
            triggered_by: 1.0,
            kind: 1,
            repeat: 1.0,
            interval: Value::Num(1.0),
            start_disabled: false,
            delay: 0.0,
            sound: 0,
            sound_location: 1.0,
            panning: 0.0,
            volume: 1.0,
            targets: Vec::new(),
        });
    }

    fn add_vertex(&mut self, p: Point, shift: bool) {
        let mut p = [p[0].round(), p[1].round()];
        let Some(&first) = self.drawing.first() else {
            self.drawing.push(p);
            return;
        };
        if self.tool == Tool::Polygon {
            let local: Vec<Point> = self.drawing.iter().map(|v| [v[0] - first[0], v[1] - first[1]]).collect();
            let (next, what) = polygon_cursor(&local, [p[0] - first[0], p[1] - first[1]], shift, self.snap);
            p = add(first, next);
            match what {
                Cursor::Add => self.drawing.push(p),
                Cursor::ReplaceLast => {
                    self.drawing.pop();
                    self.drawing.push(p);
                }
                Cursor::Complete if convex(&self.drawing) => self.finish_drawing(true),
                Cursor::Complete | Cursor::Nothing => {}
            }
        } else if self.drawing.len() > 2 && (first[0] - p[0]).hypot(first[1] - p[1]) <= self.snap {
            // a click on the first vertex closes the outline
            self.finish_drawing(true);
        } else if self.drawing.last() != Some(&p) && self.drawing.len() < MAX_VERTICES {
            self.drawing.push(p);
        }
    }

    pub fn finish_drawing(&mut self, closed: bool) {
        let class = match self.tool {
            Tool::Polygon => 3,
            Tool::Art => 4,
            _ => return,
        };
        if self.drawing.len() < 2 || (class == 3 && (self.drawing.len() < 3 || !convex(&self.drawing))) {
            return;
        }
        self.checkpoint();
        let points = std::mem::take(&mut self.drawing);
        // `recenter`: the shape sits at the middle of the box around its vertices and its first-click origin
        let origin = points[0];
        let local: Vec<Point> = points.iter().map(|p| [p[0] - origin[0], p[1] - origin[1]]).collect();
        let (mut lo, mut hi) = ([0.0f64; 2], [0.0f64; 2]);
        for p in &local {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let middle = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
        let used = self.doc.shapes.iter().chain(self.doc.groups.iter().flat_map(|g| &g.shapes)).filter_map(|s| s.vertices.as_ref().map(|v| v.id)).max();
        let Some(mut item) = Item::new(false, class, false) else { return };
        item.vertices = Some(Vertices { id: used.map_or(0, |i| i + 1), filled: closed || class == 3, points: local.iter().map(|p| [[p[0] - middle[0], p[1] - middle[1]], [0.0; 2], [0.0; 2]]).collect() });
        set_position(&mut item, add(origin, middle));
        for (axis, name) in ["shapeWidth", "shapeHeight"].into_iter().enumerate() {
            // the natural size, which the setter turns into a scale of 1
            let (lo, hi) = item.vertices.iter().flat_map(|v| &v.points).fold((0.0f64, 0.0f64), |(lo, hi), p| (lo.min(p[0][axis]), hi.max(p[0][axis])));
            if let Some(i) = item.spec().props.iter().position(|q| q.0 == name) {
                item.values[i] = Value::Num((hi - lo).round());
            }
        }
        self.doc.shapes.push(item);
    }

    /// The editable properties shared by the whole selection, with the first object's values.
    pub fn properties(&self) -> Vec<(String, Value)> {
        let lists: Vec<Vec<(String, Value)>> = self.selection.iter().map(|&r| self.properties_of(r)).collect();
        let Some((first, rest)) = lists.split_first() else { return Vec::new() };
        first.iter().filter(|(name, _)| rest.iter().all(|l| l.iter().any(|(n, _)| n == name))).cloned().collect()
    }

    fn properties_of(&self, r: Ref) -> Vec<(String, Value)> {
        let num = Value::Num;
        let flag = Value::Bool;
        let named = |list: Vec<(&str, Value)>| list.into_iter().map(|(n, v)| (n.to_string(), v)).collect::<Vec<_>>();
        match r {
            Ref::Shape(i) | Ref::Special(i) => {
                let list = if matches!(r, Ref::Shape(_)) { &self.doc.shapes } else { &self.doc.specials };
                list.get(i).map_or_else(Vec::new, |x| x.spec().props.iter().zip(&x.values).map(|(p, v)| (p.0.clone(), v.clone())).collect())
            }
            Ref::Group(i) => self.doc.groups.get(i).map_or_else(Vec::new, |g| {
                let mut list = vec![("x", num(g.x)), ("y", num(g.y)), ("angle", num(g.angle)), ("sleeping", flag(g.sleeping)), ("foreground", flag(g.foreground)), ("opacity", num(g.opacity)), ("immovable", flag(g.immovable)), ("fixedRotation", flag(g.fixed_rotation)), ("vehicle", flag(g.vehicle.is_some()))];
                if let Some(v) = &g.vehicle {
                    list.extend([("spaceAction", num(v.space_action)), ("shiftAction", num(v.shift_action)), ("ctrlAction", num(v.ctrl_action)), ("acceleration", num(v.acceleration)), ("leaningStrength", num(v.leaning_strength)), ("characterPose", num(v.character_pose)), ("lockJoints", flag(v.lock_joints))]);
                }
                named(list)
            }),
            Ref::Joint(i) => self.doc.joints.get(i).map_or_else(Vec::new, |j| {
                let mut list = vec![("x", num(j.x)), ("y", num(j.y)), ("limit", flag(j.limit))];
                if j.prismatic {
                    list.extend([("axisAngle", num(j.axis_angle)), ("upperLimit", num(j.upper)), ("lowerLimit", num(j.lower)), ("motor", flag(j.motor)), ("force", num(j.force))]);
                } else {
                    list.extend([("upperAngle", num(j.upper)), ("lowerAngle", num(j.lower)), ("motor", flag(j.motor)), ("torque", num(j.force))]);
                }
                list.extend([("speed", num(j.speed)), ("collideSelf", flag(j.collide_self)), ("vehicleControlled", flag(j.vehicle_controlled))]);
                named(list)
            }),
            Ref::Trigger(i) => self.doc.triggers.get(i).map_or_else(Vec::new, |t| {
                named(vec![("x", num(t.x)), ("y", num(t.y)), ("shapeWidth", num(t.width)), ("shapeHeight", num(t.height)), ("angle", num(t.angle)), ("triggeredBy", num(t.triggered_by)), ("typeIndex", num(f64::from(t.kind))), ("repeatType", num(t.repeat)), ("repeatInterval", num(t.interval.number())), ("startDisabled", flag(t.start_disabled)), ("triggerDelay", num(t.delay)), ("soundEffect", num(t.sound as f64)), ("soundLocation", num(t.sound_location)), ("panning", num(t.panning)), ("volume", num(t.volume))])
            }),
        }
    }

    /// Set a property of everything selected, from the text of an input field (`t`/`f` or `true`/`false` for switches).
    pub fn set_property(&mut self, name: &str, text: &str) {
        if self.selection.is_empty() {
            return;
        }
        let value = match text {
            _ if name == "caption" => Value::Text(text.into()),
            "t" | "true" => Value::Bool(true),
            "f" | "false" => Value::Bool(false),
            _ => match text.trim().parse::<f64>() {
                Ok(n) if n.is_finite() => Value::Num(n),
                _ => return,
            },
        };
        self.checkpoint();
        let (n, on) = (value.number(), value.truthy());
        for &r in &self.selection {
            match r {
                Ref::Shape(i) => {
                    if let Some(x) = self.doc.shapes.get_mut(i) {
                        x.set(name, value.clone());
                    }
                }
                Ref::Special(i) => {
                    if let Some(x) = self.doc.specials.get_mut(i) {
                        x.set(name, value.clone());
                    }
                }
                Ref::Group(i) => {
                    let Some(g) = self.doc.groups.get_mut(i) else { continue };
                    match name {
                        "x" => g.x = n,
                        "y" => g.y = n,
                        "angle" => g.angle = n.clamp(-180.0, 180.0).round(),
                        "sleeping" => g.sleeping = on,
                        "foreground" => g.foreground = on,
                        "opacity" => g.opacity = n.clamp(0.0, 100.0).round(),
                        "immovable" => g.immovable = on,
                        "fixedRotation" => g.fixed_rotation = on,
                        "vehicle" => g.vehicle = on.then(|| crate::editor::Vehicle { acceleration: 1.0, ..Default::default() }),
                        _ => {
                            if let Some(v) = g.vehicle.as_mut() {
                                match name {
                                    "spaceAction" => v.space_action = n.clamp(0.0, 3.0),
                                    "shiftAction" => v.shift_action = n.clamp(0.0, 3.0),
                                    "ctrlAction" => v.ctrl_action = n.clamp(0.0, 3.0),
                                    "acceleration" => v.acceleration = n.clamp(1.0, 10.0),
                                    "leaningStrength" => v.leaning_strength = n.clamp(0.0, 10.0),
                                    "characterPose" => v.character_pose = n.clamp(0.0, 3.0),
                                    "lockJoints" => v.lock_joints = on,
                                    _ => {}
                                }
                            }
                        }
                    }
                }
                Ref::Joint(i) => {
                    let Some(j) = self.doc.joints.get_mut(i) else { continue };
                    match name {
                        "x" => j.x = n,
                        "y" => j.y = n,
                        "limit" => j.limit = on,
                        "motor" => j.motor = on,
                        "axisAngle" => j.axis_angle = n.clamp(-180.0, 180.0),
                        "upperAngle" | "upperLimit" => j.set_upper(n),
                        "lowerAngle" | "lowerLimit" => j.set_lower(n),
                        "torque" | "force" => j.set_force(n),
                        "speed" => j.set_speed(n),
                        "collideSelf" => j.collide_self = on,
                        "vehicleControlled" => j.vehicle_controlled = on,
                        _ => {}
                    }
                }
                Ref::Trigger(i) => {
                    let Some(t) = self.doc.triggers.get_mut(i) else { continue };
                    match name {
                        "x" => t.x = n,
                        "y" => t.y = n,
                        "shapeWidth" => t.width = n.clamp(5.0, 5000.0).round(),
                        "shapeHeight" => t.height = n.clamp(5.0, 5000.0).round(),
                        "angle" => t.angle = n.clamp(-180.0, 180.0),
                        "triggeredBy" => t.triggered_by = n.round().clamp(1.0, 6.0),
                        "typeIndex" => t.kind = n.round().clamp(1.0, 3.0) as u8,
                        "repeatType" => t.repeat = n.round().clamp(1.0, 4.0),
                        "repeatInterval" => t.interval = Value::Num(n.clamp(0.1, 30.0)),
                        "startDisabled" => t.start_disabled = on,
                        "triggerDelay" => t.delay = n.clamp(0.0, 30.0),
                        "soundEffect" => t.sound = n.round().clamp(0.0, 325.0) as i64,
                        "soundLocation" => t.sound_location = n.round().clamp(1.0, 2.0),
                        "panning" => t.panning = n.clamp(-1.0, 1.0),
                        "volume" => t.volume = n.clamp(0.0, 1.0),
                        _ => {}
                    }
                }
            }
        }
    }

    /// Make every selected trigger act on (or listen to) every selected other object, with each target's first action.
    pub fn link_targets(&mut self) {
        let triggers: Vec<usize> = self.selection.iter().filter_map(|r| if let Ref::Trigger(i) = r { Some(*i) } else { None }).collect();
        if triggers.is_empty() || self.selection.len() < 2 {
            return;
        }
        self.checkpoint();
        // with only triggers selected, the first drives the rest
        let (drivers, targets): (Vec<usize>, Vec<Ref>) = if self.selection.iter().all(|r| matches!(r, Ref::Trigger(_))) {
            (vec![triggers[0]], triggers[1..].iter().map(|&i| Ref::Trigger(i)).collect())
        } else {
            (triggers, self.selection.iter().copied().filter(|r| !matches!(r, Ref::Trigger(_))).collect())
        };
        for i in drivers {
            for &target in &targets {
                let Ok(counts) = self.doc.action_counts(target) else { continue };
                let actions = counts.map_or_else(Vec::new, |c| vec![crate::editor::Action { index: 0, params: vec![Some(Value::Num(0.0)); c.first().copied().unwrap_or(0)] }]);
                if let Some(t) = self.doc.triggers.get_mut(i) {
                    if !t.targets.iter().any(|x| x.target == target) {
                        t.targets.push(Target { target, actions });
                    }
                }
            }
        }
    }
}

/// The editor's window onto the canvas: the original's is 900 × 500 view pixels.
#[derive(Clone, Copy, Debug)]
pub struct View {
    /// Canvas point at the middle of the window.
    pub camera: Point,
    /// Screen pixels per canvas pixel.
    pub zoom: f64,
    pub width: u32,
    pub height: u32,
}

impl View {
    pub fn canvas(&self, screen: Point) -> Point {
        [(screen[0] - f64::from(self.width) / 2.0) / self.zoom + self.camera[0], (screen[1] - f64::from(self.height) / 2.0) / self.zoom + self.camera[1]]
    }

    pub fn screen(&self, canvas: Point) -> (f32, f32) {
        (((canvas[0] - self.camera[0]) * self.zoom + f64::from(self.width) / 2.0) as f32, ((canvas[1] - self.camera[1]) * self.zoom + f64::from(self.height) / 2.0) as f32)
    }

    /// Zoom about a screen point, which stays on the same canvas point.
    pub fn zoom_by(&mut self, factor: f64, at: Point) {
        let keep = self.canvas(at);
        self.zoom = (self.zoom * factor).clamp(0.05, 8.0);
        let moved = self.canvas(at);
        self.camera = [self.camera[0] + keep[0] - moved[0], self.camera[1] + keep[1] - moved[1]];
    }
}

/// The level as the game would draw it, kept until the document changes.
#[derive(Default)]
pub struct Preview {
    built: Option<(String, crate::level::Level, crate::physics::World)>,
}

impl Editor {
    /// The window: the level drawn by the game's renderer (nothing simulated), and over it what only the editor shows —
    /// the start point, joints, triggers, the selection and the outline being drawn.
    pub fn render(&self, view: View, preview: &mut Preview, sprites: Option<&crate::sprites::Sprites>) -> RgbaImage {
        let xml = self.doc.to_xml();
        if preview.built.as_ref().is_none_or(|b| b.0 != xml) {
            preview.built = crate::level::parse(&xml).ok().map(|level| {
                let world = crate::physics::World::from_level(&level);
                (xml, level, world)
            });
        }
        let mut img = match &preview.built {
            Some((_, level, world)) => crate::render::render_level_view(level, world, view.width, view.height, view.zoom as f32, [view.camera[0] as f32, view.camera[1] as f32], sprites, None),
            None => RgbaImage::from_pixel(view.width, view.height, Rgba([255, 255, 255, 255])),
        };
        let outline = |img: &mut RgbaImage, points: &[Point], closed: bool, colour: Rgba<u8>| {
            let n = points.len();
            for i in 0..n.saturating_sub(usize::from(!closed)) {
                let ((ax, ay), (bx, by)) = (view.screen(points[i]), view.screen(points[(i + 1) % n]));
                // two pixels wide: the page shows the image smaller than it is
                for (dx, dy) in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)] {
                    draw_line_segment_mut(img, (ax + dx, ay + dy), (bx + dx, by + dy), colour);
                }
            }
        };
        let ring = |img: &mut RgbaImage, at: Point, radius: f64, colour: Rgba<u8>| {
            let (x, y) = view.screen(at);
            // a dark rim keeps the mark visible on anything of its own colour
            for (d, c) in [(-1, Rgba([0, 0, 0, 255])), (0, colour), (1, colour), (2, colour), (3, Rgba([255, 255, 255, 255]))] {
                draw_hollow_circle_mut(img, (x as i32, y as i32), ((radius * view.zoom) as i32 - d).max(1), c);
            }
        };
        let grey = Rgba([120, 120, 120, 255]);
        outline(&mut img, &rect(0.0, 0.0, 20_000.0, 10_000.0), true, grey);
        let green = Rgba([30, 150, 60, 255]);
        for (i, t) in self.doc.triggers.iter().enumerate() {
            outline(&mut img, &self.outline(Ref::Trigger(i)), true, green);
            // arms to what an activating (or object-listening) trigger points at
            if t.kind == 1 || t.triggered_by == 4.0 {
                for target in &t.targets {
                    let points = self.outline(target.target);
                    if !points.is_empty() {
                        let [l, top, r, b] = bounds(&points);
                        outline(&mut img, &[[t.x, t.y], [(l + r) / 2.0, (top + b) / 2.0]], false, green);
                    }
                }
            }
        }
        let red = Rgba([200, 40, 40, 255]);
        for j in &self.doc.joints {
            ring(&mut img, [j.x, j.y], JOINT_RADIUS, red);
            if j.prismatic {
                let d = turn([40.0, 0.0], j.axis_angle);
                outline(&mut img, &[[j.x - d[0], j.y - d[1]], [j.x + d[0], j.y + d[1]]], false, red);
            }
        }
        let start = [self.doc.info.x, self.doc.info.y];
        ring(&mut img, start, 20.0, Rgba([230, 140, 0, 255]));
        outline(&mut img, &[[start[0] - 28.0, start[1]], [start[0] + 28.0, start[1]]], false, Rgba([230, 140, 0, 255]));
        outline(&mut img, &[[start[0], start[1] - 28.0], [start[0], start[1] + 28.0]], false, Rgba([230, 140, 0, 255]));
        let blue = Rgba([20, 110, 255, 255]);
        for &r in &self.selection {
            let points = self.outline(r);
            outline(&mut img, &points, true, blue);
            if !points.is_empty() {
                let [l, t, r, b] = bounds(&points);
                outline(&mut img, &rect(l - 3.0, t - 3.0, r - l + 6.0, b - t + 6.0), true, blue);
            }
        }
        if let Some([from, to]) = self.selection_box() {
            let [l, t, r, b] = bounds(&[from, to]);
            outline(&mut img, &rect(l, t, r - l, b - t), true, blue);
        }
        outline(&mut img, &self.drawing, false, blue);
        for &p in &self.drawing {
            ring(&mut img, p, SNAP, blue);
        }
        img
    }
}

fn convex(points: &[Point]) -> bool {
    let n = points.len();
    n >= 3
        && (0..n).all(|i| {
            let (a, b, c) = (points[i], points[(i + 1) % n], points[(i + 2) % n]);
            let u = [b[0] - a[0], b[1] - a[1]];
            let v = [c[0] - b[0], c[1] - b[1]];
            // the original rejects any turn to the left (clockwise outlines on screen)
            v[0] * u[1] - v[1] * u[0] <= 0.0
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn click(e: &mut Editor, p: Point) {
        e.pointer_down(p, false);
        e.pointer_up(p);
    }

    fn two_boxes() -> Editor {
        let mut e = Editor::default();
        e.set_tool(Tool::Shape);
        click(&mut e, [1000.0, 1000.0]);
        click(&mut e, [1050.0, 1000.0]);
        e
    }

    #[test]
    fn joints_take_the_two_bodies_under_them_and_follow_deletion() {
        let mut e = two_boxes();
        e.set_tool(Tool::Joint);
        click(&mut e, [1025.0, 1000.0]);
        assert_eq!((e.doc.joints[0].body1, e.doc.joints[0].body2), (Some(Ref::Shape(1)), Some(Ref::Shape(0))));
        e.set_tool(Tool::Arrow);
        click(&mut e, [960.0, 1000.0]);
        assert_eq!(e.selection, [Ref::Shape(0)]);
        e.key(8, false, false);
        assert_eq!((e.doc.shapes.len(), e.doc.joints[0].body1, e.doc.joints[0].body2), (1, Some(Ref::Shape(0)), None));
        e.undo();
        assert_eq!((e.doc.shapes.len(), e.doc.joints[0].body2), (2, Some(Ref::Shape(0))));
    }

    #[test]
    fn grouping_keeps_everything_where_it_was() {
        let mut e = two_boxes();
        e.set_tool(Tool::Arrow);
        e.pointer_down([800.0, 800.0], false);
        e.pointer_up([1300.0, 1200.0]);
        assert_eq!(e.selection.len(), 2);
        let before: Vec<Vec<Point>> = e.selection.iter().map(|&r| e.outline(r)).collect();
        e.group();
        let g = &e.doc.groups[0];
        assert_eq!(([g.x, g.y], g.offset, g.shapes.len(), e.doc.shapes.len()), ([1025.0, 1000.0], [-1025.0, -1000.0], 2, 0));
        e.ungroup();
        let after: Vec<Vec<Point>> = e.selection.iter().map(|&r| e.outline(r)).collect();
        assert_eq!(before, after);
    }

    #[test]
    fn grouping_matches_the_original_editor() {
        // numbers from `RefGroup.build` and `breakApart` run in the original
        let xml = "<levelXML><info v='1.99' x='300' y='5100' c='1' f='f' h='f' bg='0' bgc='16777215'/><shapes>\
            <sh t='0' p0='1000' p1='1000' p2='200' p3='50' p4='30' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>\
            <sh t='1' p0='1300' p1='1100' p2='80' p3='80' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>\
            <sh t='2' p0='900' p1='1200' p2='100' p3='90' p4='45' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/>\
            </shapes><specials><sp t='3' p0='1100' p1='900' p2='400' p3='32' p4='10' p5='t' p6='f'/></specials></levelXML>";
        let mut e = Editor::new(Document::parse(xml).unwrap());
        e.selection = vec![Ref::Shape(0), Ref::Shape(1), Ref::Shape(2), Ref::Special(0)];
        e.group();
        let near = |a: f64, b: f64, by: f64| assert!((a - b).abs() < by, "{a} against {b}");
        // within a pixel and a half: the original measures its drawn art
        near(e.doc.groups[0].x, 1091.7157, 1.5);
        near(e.doc.groups[0].y, 1053.0410, 1.5);
        // turn and shift the group as the original was, from its exact centre
        let g = &mut e.doc.groups[0];
        (g.x, g.y, g.offset, g.angle) = (1191.715728752538, 1053.0409908366953, [-1091.715728752538, -1053.0409908366953], 40.0);
        e.ungroup();
        let original = [[1155.5515, 953.4555, 70.0], [1321.0861, 1222.8962, 40.0], [950.3895, 1042.3856, 85.0], [1296.4347, 941.1298, 50.0]];
        for (item, want) in e.doc.shapes.iter().chain(&e.doc.specials).zip(original) {
            near(position(item)[0], want[0], 0.001);
            near(position(item)[1], want[1], 0.001);
            near(number(item, "angle"), want[2], 0.001);
        }
    }

    #[test]
    fn the_polygon_cursor_matches_the_original_code() {
        let cases: serde_json::Value = serde_json::from_str(include_str!("../../../tests/golden/polygon-cursor.json")).unwrap();
        let point = |v: &serde_json::Value| [v[0].as_f64().unwrap(), v[1].as_f64().unwrap()];
        for (k, case) in cases.as_array().unwrap().iter().enumerate() {
            let vertices: Vec<Point> = case["verts"].as_array().unwrap().iter().map(point).collect();
            let snap = SNAP * 2f64.powi(-(case["zoom"].as_i64().unwrap() as i32));
            let (next, what) = polygon_cursor(&vertices, point(&case["mouse"]), case["shift"].as_bool().unwrap(), snap);
            let want = [Cursor::Add, Cursor::Complete, Cursor::Nothing, Cursor::ReplaceLast][case["state"].as_u64().unwrap() as usize];
            assert_eq!((next, what), (point(&case["next"]), want), "case {k}: {case}");
        }
    }

    #[test]
    fn keys_move_size_turn_and_copy() {
        let mut e = two_boxes();
        e.set_tool(Tool::Arrow);
        click(&mut e, [960.0, 1000.0]);
        for (code, shift) in [(39, true), (40, false), (68, true), (88, true), (67, false)] {
            e.key(code, shift, false);
        }
        let s = &e.doc.shapes[0];
        assert_eq!((position(s), number(s, "shapeWidth")), ([1010.0, 1001.0], 110.0));
        assert!((number(s, "angle") - 10.0).abs() < 1e-9);
        e.key(86, true, false);
        assert_eq!((e.doc.shapes.len(), e.selection.clone()), (3, vec![Ref::Shape(2)]));
        e.key(40, false, true);
        assert_eq!(e.selection, [Ref::Shape(1)]);
    }

    #[test]
    fn the_window_shows_what_is_edited() {
        let mut e = two_boxes();
        let view = View { camera: [1000.0, 1000.0], zoom: 1.0, width: 400, height: 300 };
        assert_eq!(view.canvas([200.0, 150.0]), [1000.0, 1000.0]);
        let mut preview = Preview::default();
        let before = e.render(view, &mut preview, None);
        // the rectangle's fill, then the same place once it is deleted
        assert_ne!(before.get_pixel(180, 150).0, [255, 255, 255, 255]);
        e.set_tool(Tool::Arrow);
        e.pointer_down([800.0, 800.0], false);
        e.pointer_up([1200.0, 1200.0]);
        e.delete();
        assert_eq!(e.render(view, &mut preview, None).get_pixel(180, 150).0, [255, 255, 255, 255]);
    }

    #[test]
    fn polygons_are_recentred_and_saved() {
        let mut e = Editor::default();
        e.set_tool(Tool::Polygon);
        for p in [[1000.0, 1000.0], [1100.0, 1000.0], [1100.0, 1060.0], [1000.0, 1000.0]] {
            click(&mut e, p);
        }
        let s = &e.doc.shapes[0];
        assert_eq!((position(s), number(s, "shapeWidth"), number(s, "shapeHeight")), ([1050.0, 1030.0], 100.0, 60.0));
        let xml = e.doc.to_xml();
        assert!(xml.contains("v0=\"-50_-30\" v1=\"50_-30\" v2=\"50_30\""), "{xml}");
        assert_eq!(Document::parse(&xml).unwrap().to_xml(), xml);
    }
}
