//! Editing the vertices of a finished polygon or art shape (the original's vertex edit mode, opened by a double click).
//! Vertices are in the shape's own unscaled units; the shape keeps its scale, so its width and height follow the outline.

use super::{bounds, number, position, turn, Editor, Point, MAX_POLYGON_VERTICES, MAX_VERTICES, SNAP};
use crate::editor::{Item, Value};

/// A vertex and its incoming and outgoing curve handles (relative to it).
pub type Vertex = [Point; 3];

/// What the window shows of the edited outline.
#[derive(Default)]
pub struct Marks {
    /// Canvas position of each vertex and whether it is selected.
    pub vertices: Vec<(Point, bool)>,
    /// Curve handles of the selected vertices: vertex, side (1 in, 2 out), canvas position.
    pub handles: Vec<(usize, usize, Point)>,
}

/// How far a new polygon vertex is pushed out of the edge it splits (`ADJUSTMENT_LENGTH`).
const NUDGE: f64 = 0.25;

#[derive(Clone, Copy, Debug)]
enum Drag {
    /// The vertex under the pointer; the other selected ones keep their offsets from it.
    Vertex { index: usize, moved: bool },
    Handle { index: usize, side: usize, moved: bool },
    Box { from: Point, to: Point },
}

#[derive(Clone, Debug)]
pub struct VertexEdit {
    pub shape: usize,
    pub selected: Vec<usize>,
    /// The shape's factor on its vertices, kept exact while editing (the file holds rounded sizes).
    factor: Point,
    drag: Option<Drag>,
    /// The shape already has its new vertex-list id.
    renamed: bool,
}

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}

fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn unit(a: Point) -> Point {
    let l = a[0].hypot(a[1]);
    if l > 0.0 { [a[0] / l, a[1] / l] } else { a }
}

/// Whether the corner at `b` turns the wrong way (or not at all).
fn bent(a: Point, b: Point, c: Point) -> bool {
    let u = unit(sub(b, a));
    dot(unit(sub(c, b)), [u[1], -u[0]]) >= 0.0
}

/// `isConvexB2Vec2`: strictly convex, so three vertices in a line are refused.
pub fn strictly_convex(points: &[Point]) -> bool {
    let n = points.len();
    (0..n).all(|i| !bent(points[i], points[(i + 1) % n], points[(i + 2) % n]))
}

/// `fixConcaveViolations`: push every offending vertex just outside the line between its neighbours.
pub fn fix_concave(points: &mut [Point]) {
    let n = points.len();
    // the original loops until nothing is bent; a bound keeps a degenerate outline from hanging
    for _ in 0..100 {
        let mut moved = false;
        for i in 0..n {
            let (a, b, c) = (points[i], points[(i + 1) % n], points[(i + 2) % n]);
            if bent(a, b, c) {
                moved = true;
                let line = unit(sub(c, a));
                let along = dot(line, sub(b, a));
                points[(i + 1) % n] = [a[0] + line[0] * along + line[1] * NUDGE, a[1] + line[1] * along - line[0] * NUDGE];
            }
        }
        if !moved {
            break;
        }
    }
}

/// `insertVertSelected`: a new vertex after each selected one. Art curves are split at their middle; polygon edges get a
/// point pushed slightly outwards. `selected` is renumbered to keep naming the same vertices.
pub fn insert_vertices(points: &mut Vec<Vertex>, selected: &mut [usize], art: bool, closed: bool) -> bool {
    let limit = if art { MAX_VERTICES } else { MAX_POLYGON_VERTICES };
    let mut added = false;
    for k in 0..selected.len() {
        let n = points.len();
        if n >= limit {
            break;
        }
        let i = selected[k];
        if i + 1 >= n && !closed {
            continue;
        }
        let (a, b) = (points[i], points[(i + 1) % n]);
        let at = if art {
            // the cubic through both vertices' handles, at t = 0.5
            [0, 1].map(|x| (a[0][x] + 3.0 * (a[0][x] + a[2][x]) + 3.0 * (b[0][x] + b[1][x]) + b[0][x]) / 8.0)
        } else {
            let edge = sub(b[0], a[0]);
            let out = unit([edge[1], -edge[0]]);
            [0, 1].map(|x| a[0][x] + 0.5 * edge[x] + out[x] * NUDGE)
        };
        points.insert(i + 1, [at, [0.0; 2], [0.0; 2]]);
        added = true;
        for s in selected.iter_mut() {
            if *s > i {
                *s += 1;
            }
        }
    }
    if added && !art {
        let mut corners: Vec<Point> = points.iter().map(|p| p[0]).collect();
        fix_concave(&mut corners);
        for (p, c) in points.iter_mut().zip(corners) {
            p[0] = c;
        }
    }
    added
}

/// The shape's factor on its vertices (see `level::Shape::vertex_scale`).
fn scale(item: &Item) -> Point {
    let Some(v) = item.vertices.as_ref() else { return [1.0, 1.0] };
    [0, 1].map(|axis| {
        let (lo, hi) = v.points.iter().fold((0.0f64, 0.0f64), |(lo, hi), p| (lo.min(p[0][axis]), hi.max(p[0][axis])));
        let size = number(item, ["shapeWidth", "shapeHeight"][axis]);
        if hi > lo && size != 0.0 { (size / (hi - lo)).clamp(0.1, 10.0) } else { 1.0 }
    })
}

/// A canvas point in the shape's vertex units.
fn to_local(item: &Item, s: Point, p: Point) -> Point {
    let q = turn(sub(p, position(item)), -number(item, "angle"));
    [q[0] / s[0], q[1] / s[1]]
}

fn to_canvas(item: &Item, s: Point, v: Point) -> Point {
    let q = turn([v[0] * s[0], v[1] * s[1]], number(item, "angle"));
    let at = position(item);
    [at[0] + q[0], at[1] + q[1]]
}

fn snapped(p: Point, shift: bool) -> Point {
    let round = |v: f64| (v + 0.5).floor();
    let p = p.map(round);
    if shift { p.map(|v| 10.0 * round(0.1 * v)) } else { p }
}

impl Editor {
    fn edited(&self) -> Option<(&VertexEdit, &Item)> {
        let edit = self.vertex_edit.as_ref()?;
        let item = self.doc.shapes.get(edit.shape)?;
        item.vertices.as_ref()?;
        Some((edit, item))
    }

    pub fn vertex_marks(&self) -> Marks {
        let Some((edit, item)) = self.edited() else { return Default::default() };
        let points = &item.vertices.as_ref().expect("checked").points;
        let marks = points.iter().enumerate().map(|(i, p)| (to_canvas(item, edit.factor, p[0]), edit.selected.contains(&i))).collect();
        let handles = edit
            .selected
            .iter()
            .filter_map(|&i| points.get(i).map(|p| (i, p)))
            .flat_map(|(i, p)| [1, 2].into_iter().filter(|&s| p[s] != [0.0; 2]).map(move |s| (i, s, to_canvas(item, edit.factor, [p[0][0] + p[s][0], p[0][1] + p[s][1]]))))
            .collect();
        Marks { vertices: marks, handles }
    }

    pub fn vertex_box(&self) -> Option<[Point; 2]> {
        match self.vertex_edit.as_ref()?.drag {
            Some(Drag::Box { from, to }) => Some([from, to]),
            _ => None,
        }
    }

    /// Change the edited shape's vertex list as one undo step; the shape keeps its scale and gets a new list id.
    fn change_vertices(&mut self, checkpoint: bool, change: impl FnOnce(&mut Vec<Vertex>, &mut Vec<usize>)) {
        if self.edited().is_none() {
            return;
        }
        if checkpoint {
            self.checkpoint();
        }
        let next_id = self.doc.shapes.iter().chain(self.doc.groups.iter().flat_map(|g| &g.shapes)).filter_map(|s| s.vertices.as_ref().map(|v| v.id)).max().map_or(0, |i| i + 1);
        let Some(edit) = self.vertex_edit.as_mut() else { return };
        let item = &mut self.doc.shapes[edit.shape];
        let factor = edit.factor;
        let Some(vertices) = item.vertices.as_mut() else { return };
        let before = vertices.points.clone();
        change(&mut vertices.points, &mut edit.selected);
        if vertices.points == before {
            return;
        }
        if !edit.renamed {
            // other shapes may share the old list
            vertices.id = next_id;
            edit.renamed = true;
        }
        let extent = [0, 1].map(|axis| {
            let (lo, hi) = vertices.points.iter().fold((0.0f64, 0.0f64), |(lo, hi), p| (lo.min(p[0][axis]), hi.max(p[0][axis])));
            hi - lo
        });
        for (axis, name) in ["shapeWidth", "shapeHeight"].into_iter().enumerate() {
            if let Some(i) = item.spec().props.iter().position(|q| q.0 == name) {
                // the original's getter: `Math.round(scaleX * defaultWidth)`
                item.values[i] = Value::Num((factor[axis] * extent[axis] + 0.5).floor());
            }
        }
    }

    pub(super) fn open_vertex_edit(&mut self, shape: usize) {
        self.selection.clear();
        self.series = None;
        let factor = scale(&self.doc.shapes[shape]);
        self.vertex_edit = Some(VertexEdit { shape, selected: Vec::new(), factor, drag: None, renamed: false });
    }

    pub fn close_vertex_edit(&mut self) {
        if let Some(edit) = self.vertex_edit.take() {
            if edit.shape < self.doc.shapes.len() {
                self.selection = vec![crate::editor::Ref::Shape(edit.shape)];
            }
        }
    }

    /// The vertex or handle under a canvas point.
    fn vertex_at(&self, p: Point) -> Option<(usize, Option<usize>)> {
        let Marks { vertices: marks, handles } = self.vertex_marks();
        let near = |q: Point| (q[0] - p[0]).hypot(q[1] - p[1]) <= self.snap;
        marks.iter().position(|m| near(m.0)).map(|i| (i, None)).or_else(|| handles.iter().find(|h| near(h.2)).map(|h| (h.0, Some(h.1))))
    }

    pub(super) fn vertex_pointer_down(&mut self, p: Point, shift: bool) {
        let hit = self.vertex_at(p);
        let Some(edit) = self.vertex_edit.as_mut() else { return };
        edit.drag = match hit {
            Some((i, None)) => match edit.selected.iter().position(|&s| s == i) {
                Some(at) if shift => {
                    edit.selected.remove(at);
                    edit.selected.last().map(|&last| Drag::Vertex { index: last, moved: false })
                }
                found => {
                    if found.is_none() {
                        if !shift {
                            edit.selected.clear();
                        }
                        edit.selected.push(i);
                    }
                    Some(Drag::Vertex { index: i, moved: false })
                }
            },
            Some((index, Some(side))) => Some(Drag::Handle { index, side, moved: false }),
            None => {
                if !shift {
                    edit.selected.clear();
                }
                Some(Drag::Box { from: p, to: p })
            }
        };
    }

    pub(super) fn vertex_pointer_move(&mut self, p: Point) {
        let Some((edit, item)) = self.edited() else { return };
        let (drag, shift, alt, snap) = (edit.drag, self.shift, self.alt, self.snap);
        let local = to_local(item, edit.factor, p);
        let art = item.class == 4;
        let points = &item.vertices.as_ref().expect("checked").points;
        match drag {
            Some(Drag::Vertex { index, moved }) => {
                let Some(from) = points.get(index).map(|v| v[0]) else { return };
                let to = snapped(local, shift);
                let by = sub(to, from);
                if by == [0.0, 0.0] {
                    return;
                }
                let mut corners: Vec<Point> = points.iter().map(|v| v[0]).collect();
                for &s in &edit.selected {
                    if let Some(c) = corners.get_mut(s) {
                        *c = [c[0] + by[0], c[1] + by[1]].map(|v| (v + 0.5).floor());
                    }
                }
                // a polygon vertex refuses to go where the outline would stop being convex
                if !art && !strictly_convex(&corners) {
                    return;
                }
                self.change_vertices(!moved, |points, _| {
                    for (v, c) in points.iter_mut().zip(corners) {
                        v[0] = c;
                    }
                });
                if let Some(e) = self.vertex_edit.as_mut() {
                    e.drag = Some(Drag::Vertex { index, moved: true });
                }
            }
            Some(Drag::Handle { index, side, moved }) => {
                let Some(vertex) = points.get(index).map(|v| v[0]) else { return };
                let mut handle = snapped(sub(local, vertex), shift);
                if handle[0].hypot(handle[1]) < snap {
                    handle = [0.0; 2];
                }
                self.change_vertices(!moved, |points, _| {
                    points[index][side] = handle;
                    // the other handle mirrors it unless Alt is held
                    if !alt {
                        points[index][3 - side] = [-handle[0], -handle[1]];
                    }
                });
                if let Some(e) = self.vertex_edit.as_mut() {
                    e.drag = Some(Drag::Handle { index, side, moved: true });
                }
            }
            Some(Drag::Box { from, .. }) => {
                if let Some(e) = self.vertex_edit.as_mut() {
                    e.drag = Some(Drag::Box { from, to: p });
                }
            }
            None => {}
        }
    }

    pub(super) fn vertex_pointer_up(&mut self, p: Point) {
        self.vertex_pointer_move(p);
        let marks = self.vertex_marks().vertices;
        let Some(edit) = self.vertex_edit.as_mut() else { return };
        if let Some(Drag::Box { from, to }) = edit.drag {
            let [l, t, r, b] = bounds(&[from, to]);
            for (i, (q, _)) in marks.iter().enumerate() {
                if q[0] >= l && q[0] <= r && q[1] >= t && q[1] <= b && !edit.selected.contains(&i) {
                    edit.selected.push(i);
                }
            }
        }
        edit.drag = None;
    }

    /// A double click on a selected art vertex gives it handles; on nothing it leaves the mode.
    pub(super) fn vertex_double_click(&mut self, p: Point) {
        match self.vertex_at(p) {
            None => self.close_vertex_edit(),
            Some((i, None)) => {
                let Some((edit, item)) = self.edited() else { return };
                if item.class != 4 || !edit.selected.contains(&i) {
                    return;
                }
                self.change_vertices(true, |points, _| {
                    let n = points.len();
                    let (before, after, at) = (points[(i + n - 1) % n][0], points[(i + 1) % n][0], points[i][0]);
                    let along = if before == after { sub(before, at) } else { sub(before, after) };
                    let length = (along[0].hypot(along[1]) * 0.25).max(5.0);
                    let along = unit(along).map(|v| (v * length + 0.5).floor());
                    let neg = |h: Point| [-h[0], -h[1]];
                    let [_, one, two] = &mut points[i];
                    if *one == [0.0; 2] {
                        *one = if *two != [0.0; 2] { neg(*two) } else { along };
                    }
                    if *two == [0.0; 2] {
                        *two = if *one != [0.0; 2] { neg(*one) } else { along };
                    }
                });
            }
            Some(_) => {}
        }
    }

    /// Keys of the vertex edit mode: Backspace removes, Enter inserts, arrows move.
    pub(super) fn vertex_key(&mut self, code: u32, shift: bool, ctrl: bool) -> bool {
        let Some((_, item)) = self.edited() else { return false };
        let art = item.class == 4;
        let closed = !art || item.vertices.as_ref().is_some_and(|v| v.filled);
        match code {
            8 | 46 => self.change_vertices(true, |points, selected| {
                let fewest = if art { 2 } else { 3 };
                let mut gone = selected.clone();
                gone.sort_unstable_by(|a, b| b.cmp(a));
                for i in gone {
                    if points.len() <= fewest {
                        break;
                    }
                    points.remove(i);
                    selected.retain(|&s| s != i);
                    selected.iter_mut().filter(|s| **s > i).for_each(|s| *s -= 1);
                }
            }),
            13 => self.change_vertices(true, |points, selected| {
                insert_vertices(points, selected, art, closed);
            }),
            37..=40 => {
                // one screen pixel (ten with Shift) at the original's zoom
                let step = if shift { 10.0 } else { 1.0 } * self.snap / SNAP;
                let by = match code {
                    37 => [-step, 0.0],
                    38 => [0.0, -step],
                    39 => [step, 0.0],
                    _ => [0.0, step],
                };
                self.change_vertices(true, |points, selected| {
                    let mut moved = points.clone();
                    for &s in selected.iter() {
                        if let Some(v) = moved.get_mut(s) {
                            v[0] = [v[0][0] + by[0], v[0][1] + by[1]];
                        }
                    }
                    if art || strictly_convex(&moved.iter().map(|v| v[0]).collect::<Vec<_>>()) {
                        *points = moved;
                    }
                });
            }
            27 => self.close_vertex_edit(),
            90 if ctrl => self.undo(),
            89 if ctrl => self.redo(),
            49..=56 => return false,
            _ => {}
        }
        true
    }
}
