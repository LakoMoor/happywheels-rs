//! The editor's NPC poser (module 7151): nine limb segments, dragged with cyclic-coordinate-descent inverse kinematics.
//! The segment geometry of the twelve to sixteen character types comes from the running original (`data/npc-poser.json`,
//! `analysis/poser.js`); angles are radians, positions are in the coordinates of the posed clip (half size in the level).

use box2d::fdlibm::{cos, sin};

/// Segments in the order used everywhere here: the property each one sets.
pub const PROPERTIES: [&str; 9] = ["neckAngle", "shoulder1Angle", "elbow1Angle", "shoulder2Angle", "elbow2Angle", "hip1Angle", "knee1Angle", "hip2Angle", "knee2Angle"];

/// `matchRealCharacter` sets the upper limbs before the lower ones.
const MATCH_ORDER: [usize; 9] = [0, 1, 3, 2, 4, 5, 7, 6, 8];

const PARENT: [Option<usize>; 9] = [None, None, Some(1), None, Some(3), None, Some(5), None, Some(7)];

#[derive(Debug, Clone, Copy)]
struct Segment {
    at: [f64; 2],
    length: f64,
    reference: f64,
    offset: f64,
    difference: f64,
    min: f64,
    max: f64,
    angle: f64,
}

/// One draggable clip: its place in the draw order and its bounds `[x, y, width, height]` in its own coordinates.
#[derive(Debug, Clone, Copy)]
pub struct PoserClip {
    pub order: i64,
    pub bounds: [f64; 4],
    pub scale: [f64; 2],
}

#[derive(Debug, Clone)]
pub struct Poser {
    segments: [Segment; 9],
    clips: [PoserClip; 9],
}

#[derive(serde::Deserialize)]
struct Guide {
    segments: Vec<[f64; 8]>,
    clips: Vec<[f64; 7]>,
}

impl Poser {
    /// The poser of an NPC of `char_index` (1..=16) holding `angles` (degrees, in the order of [`PROPERTIES`]).
    pub fn new(char_index: u32, angles: [f64; 9]) -> Option<Self> {
        let guides: std::collections::HashMap<String, Guide> = serde_json::from_str(include_str!("../data/npc-poser.json")).ok()?;
        let guide = guides.get(&char_index.to_string())?;
        let mut segments = [Segment { at: [0.0; 2], length: 0.0, reference: 0.0, offset: 0.0, difference: 0.0, min: 0.0, max: 0.0, angle: 0.0 }; 9];
        let mut clips = [PoserClip { order: 0, bounds: [0.0; 4], scale: [1.0; 2] }; 9];
        for i in 0..9 {
            let [x, y, length, reference, offset, difference, min, max] = *guide.segments.get(i)?;
            segments[i] = Segment { at: [x, y], length, reference, offset, difference, min, max, angle: reference };
            let [order, bx, by, w, h, sx, sy] = *guide.clips.get(i)?;
            clips[i] = PoserClip { order: order as i64, bounds: [bx, by, w, h], scale: [sx, sy] };
        }
        let mut poser = Poser { segments, clips };
        for i in MATCH_ORDER {
            let model = angles[i] * (std::f64::consts::PI / 180.0);
            let s = poser.segments[i];
            let angle = match PARENT[i] {
                Some(p) => model + poser.segments[p].angle + poser.segments[p].difference,
                None => model + s.reference,
            };
            poser.set_angle(i, angle);
        }
        Some(poser)
    }

    fn end(&self, i: usize) -> [f64; 2] {
        let s = &self.segments[i];
        [cos(s.angle) * s.length + s.at[0], sin(s.angle) * s.length + s.at[1]]
    }

    /// The `angle` setter: clamped to the joint's range; the child follows the end point and keeps its bend.
    fn set_angle(&mut self, i: usize, mut angle: f64) {
        let s = self.segments[i];
        let base = PARENT[i].map_or(s.reference, |p| self.segments[p].angle + self.segments[p].difference);
        let relative = angle - base;
        if relative < s.min {
            angle = base + s.min;
        }
        if relative > s.max {
            angle = base + s.max;
        }
        let turned = s.angle - angle;
        self.segments[i].angle = angle;
        let end = self.end(i);
        for (child, parent) in PARENT.into_iter().enumerate() {
            if parent == Some(i) {
                self.segments[child].at = end;
                self.set_angle(child, self.segments[child].angle - turned);
            }
        }
    }

    /// `IKSolver.solve`: bend segment `which` and its parent so that the end of `which` reaches `target`.
    pub fn drag(&mut self, which: usize, target: [f64; 2]) {
        let near = |p: [f64; 2]| (target[0] - p[0]).abs() < 1.0 && (target[1] - p[1]).abs() < 1.0;
        let unit = |v: [f64; 2]| {
            let length = (v[0] * v[0] + v[1] * v[1]).sqrt();
            if length < f64::from_bits(1) {
                return v;
            }
            let inverse = 1.0 / length;
            [v[0] * inverse, v[1] * inverse]
        };
        for _ in 0..100 {
            let mut next = Some(which);
            while let Some(i) = next {
                let from = self.segments[i].at;
                let end = self.end(which);
                if near(end) {
                    return;
                }
                let wanted = unit([target[0] - from[0], target[1] - from[1]]);
                let have = unit([end[0] - from[0], end[1] - from[1]]);
                let dot = wanted[0] * have[0] + wanted[1] * have[1];
                if dot < 0.9999 {
                    let cross = wanted[0] * have[1] - wanted[1] * have[0];
                    if cross > 0.0 {
                        self.set_angle(i, self.segments[i].angle - box2d::fdlibm::acos(dot));
                    } else if cross < 0.0 {
                        self.set_angle(i, self.segments[i].angle + box2d::fdlibm::acos(dot));
                    }
                }
                next = PARENT[i];
            }
            if near(self.end(which)) {
                return;
            }
        }
    }

    /// `modelAngleDegrees`: what a segment's property gets.
    pub fn angle(&self, i: usize) -> f64 {
        let s = &self.segments[i];
        let model = match PARENT[i] {
            Some(p) => s.angle - self.segments[p].angle - self.segments[p].difference,
            None => s.angle + s.difference - s.offset,
        };
        model * (180.0 / std::f64::consts::PI)
    }

    /// The segment's parent, whose property a drag also stores.
    pub fn parent(i: usize) -> Option<usize> {
        PARENT.get(i).copied().flatten()
    }

    /// Where `drawSegment` puts the segment's clip: position and rotation in whole degrees.
    pub fn placement(&self, i: usize) -> ([f64; 2], f64) {
        let s = &self.segments[i];
        let degrees = (s.angle + s.difference - s.offset) * (180.0 / std::f64::consts::PI);
        (s.at, (degrees + 0.5).floor())
    }

    pub fn clip(&self, i: usize) -> PoserClip {
        self.clips[i]
    }

    /// The topmost clip whose bounds contain `point`.
    pub fn pick(&self, point: [f64; 2]) -> Option<usize> {
        (0..9)
            .filter(|&i| {
                let (at, degrees) = self.placement(i);
                let turn = degrees * std::f64::consts::PI / 180.0;
                let (dx, dy) = (point[0] - at[0], point[1] - at[1]);
                let clip = self.clips[i];
                let local = [(dx * cos(turn) + dy * sin(turn)) / clip.scale[0], (dy * cos(turn) - dx * sin(turn)) / clip.scale[1]];
                let [x, y, w, h] = clip.bounds;
                local[0] >= x && local[0] <= x + w && local[1] >= y && local[1] <= y + h
            })
            .max_by_key(|&i| self.clips[i].order)
    }
}
