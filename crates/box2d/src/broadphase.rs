//! Classic Box2D 2.0.1 sweep-and-prune broadphase with quantised endpoints and a buffered pair manager
//! (box2d-pretty.js ~509-1256). Pair *ordering* (which decides contact creation order, and therefore
//! solver order) is reproduced exactly, so the structure is kept as in the original.
use crate::collision::Aabb;
use crate::math::*;
use std::collections::HashMap;

pub const MAX_PROXIES: usize = 1024;
pub const NULL_PROXY: u16 = u16::MAX;
const INVALID: i32 = u16::MAX as i32;
const USHRT_MAX: f64 = 65535.0;

#[derive(Clone, Copy, Default)]
struct Bound {
    value: u16,
    proxy_id: u16,
    stabbing: u16,
}

impl Bound {
    fn is_lower(&self) -> bool {
        self.value & 1 == 0
    }
    fn is_upper(&self) -> bool {
        self.value & 1 == 1
    }
}

#[derive(Clone, Copy)]
struct Proxy {
    lower: [usize; 2],
    upper: [usize; 2],
    overlap_count: i32,
    time_stamp: u16,
    next: u16,
    user_data: Option<usize>,
}

#[derive(Clone, Copy, Default)]
struct BoundValues {
    lower: [u16; 2],
    upper: [u16; 2],
}

#[derive(Clone, Copy, Default)]
struct Pair {
    id1: u16,
    id2: u16,
    status: u8,
    user_data: Option<usize>,
}

const BUFFERED: u8 = 1;
const REMOVED: u8 = 2;
const FINAL: u8 = 4;

/// What the pair manager reports to the world during `commit` (in buffer order).
pub enum PairEvent {
    /// A new overlapping pair `(user_data1, user_data2)`; the returned value is stored with the pair.
    Added(usize, usize),
    /// A final pair went away; carries the value stored when it was added.
    Removed(usize, usize, Option<usize>),
}

pub struct BroadPhase {
    world_lower: Vec2,
    world_upper: Vec2,
    q: Vec2,
    proxies: Vec<Proxy>,
    bounds: [Vec<Bound>; 2],
    proxy_count: usize,
    free_proxy: u16,
    time_stamp: u16,
    query_results: Vec<u16>,
    pairs: HashMap<(u16, u16), Pair>,
    pair_buffer: Vec<(u16, u16)>,
}

impl BroadPhase {
    pub fn new(lower: Vec2, upper: Vec2) -> Self {
        let mut proxies = Vec::with_capacity(MAX_PROXIES);
        for i in 0..MAX_PROXIES {
            proxies.push(Proxy {
                lower: [0; 2],
                upper: [0; 2],
                overlap_count: INVALID,
                time_stamp: 0,
                next: if i + 1 < MAX_PROXIES { (i + 1) as u16 } else { NULL_PROXY },
                user_data: None,
            });
        }
        let blank = vec![Bound::default(); 2 * MAX_PROXIES];
        Self {
            world_lower: lower,
            world_upper: upper,
            q: Vec2::new(USHRT_MAX / (upper.x - lower.x), USHRT_MAX / (upper.y - lower.y)),
            proxies,
            bounds: [blank.clone(), blank],
            proxy_count: 0,
            free_proxy: 0,
            time_stamp: 1,
            query_results: Vec::new(),
            pairs: HashMap::new(),
            pair_buffer: Vec::new(),
        }
    }

    pub fn in_range(&self, a: &Aabb) -> bool {
        let mut dx = a.lower.x - self.world_upper.x;
        let mut dy = a.lower.y - self.world_upper.y;
        let ex = self.world_lower.x - a.upper.x;
        let ey = self.world_lower.y - a.upper.y;
        dx = dx.max(ex);
        dy = dy.max(ey);
        dx.max(dy) < 0.0
    }

    /// `QueryAABB`: user data of every proxy whose quantized box overlaps `a`, in the original's order.
    pub fn query_aabb(&self, a: &Aabb) -> Vec<usize> {
        // `QueryAABB` walks the x axis, then the y axis, and lists a proxy when the second walk meets it again: the order matters to
        // callers that take only the first few
        let q = self.compute_bounds(a);
        let count = 2 * self.proxy_count;
        let walk = |axis: usize| {
            let bounds = &self.bounds[axis];
            let li = Self::binary_search(bounds, count, q.lower[axis]);
            let ui = Self::binary_search(bounds, count, q.upper[axis]);
            let mut met: Vec<u16> = bounds[li..ui].iter().filter(|b| b.is_lower()).map(|b| b.proxy_id).collect();
            if li > 0 {
                let mut i = li - 1;
                let mut stabbing = bounds[i].stabbing;
                while stabbing != 0 {
                    let b = bounds[i];
                    if b.is_lower() && li <= self.proxies[b.proxy_id as usize].upper[axis] {
                        met.push(b.proxy_id);
                        stabbing -= 1;
                    }
                    if i == 0 {
                        break;
                    }
                    i -= 1;
                }
            }
            met
        };
        let mut on_x = vec![false; self.proxies.len()];
        for id in walk(0) {
            on_x[id as usize] = true;
        }
        walk(1).into_iter().filter(|&id| on_x[id as usize]).filter_map(|id| self.proxies[id as usize].user_data).collect()
    }

    fn compute_bounds(&self, a: &Aabb) -> BoundValues {
        let clamp_lo = |v: f64, lo: f64, hi: f64| v.min(hi).max(lo);
        let lx = clamp_lo(a.lower.x, self.world_lower.x, self.world_upper.x);
        let ly = clamp_lo(a.lower.y, self.world_lower.y, self.world_upper.y);
        let ux = clamp_lo(a.upper.x, self.world_lower.x, self.world_upper.x);
        let uy = clamp_lo(a.upper.y, self.world_lower.y, self.world_upper.y);
        let t = |v: f64| (v as i64) as u32;
        BoundValues {
            lower: [
                (t(self.q.x * (lx - self.world_lower.x)) & 65534) as u16,
                (t(self.q.y * (ly - self.world_lower.y)) & 65534) as u16,
            ],
            upper: [
                ((t(self.q.x * (ux - self.world_lower.x)) & 65535) | 1) as u16,
                ((t(self.q.y * (uy - self.world_lower.y)) & 65535) | 1) as u16,
            ],
        }
    }

    fn test_overlap(&self, b: &BoundValues, p: &Proxy) -> bool {
        for n in 0..2 {
            let bounds = &self.bounds[n];
            if b.lower[n] > bounds[p.upper[n]].value {
                return false;
            }
            if b.upper[n] < bounds[p.lower[n]].value {
                return false;
            }
        }
        true
    }

    fn binary_search(bounds: &[Bound], count: usize, value: u16) -> usize {
        let mut low: i64 = 0;
        let mut high: i64 = count as i64 - 1;
        while low <= high {
            let mid = (low + high) / 2;
            let v = bounds[mid as usize].value;
            if v > value {
                high = mid - 1;
            } else if v < value {
                low = mid + 1;
            } else {
                return mid as usize;
            }
        }
        low as usize
    }

    fn increment_overlap_count(&mut self, id: u16) {
        let ts = self.time_stamp;
        let p = &mut self.proxies[id as usize];
        if p.time_stamp < ts {
            p.time_stamp = ts;
            p.overlap_count = 1;
        } else {
            p.overlap_count = 2;
            self.query_results.push(id);
        }
    }

    fn increment_time_stamp(&mut self) {
        if self.time_stamp == u16::MAX {
            for p in &mut self.proxies {
                p.time_stamp = 0;
            }
            self.time_stamp = 1;
        } else {
            self.time_stamp += 1;
        }
    }

    /// Returns (lower index, upper index) and bumps overlap counts like the original `Query`.
    fn query(&mut self, lower_value: u16, upper_value: u16, axis: usize, count: usize) -> (usize, usize) {
        let li = Self::binary_search(&self.bounds[axis], count, lower_value);
        let ui = Self::binary_search(&self.bounds[axis], count, upper_value);
        for i in li..ui {
            let b = self.bounds[axis][i];
            if b.is_lower() {
                self.increment_overlap_count(b.proxy_id);
            }
        }
        if li > 0 {
            let mut i = li as i64 - 1;
            let mut s = self.bounds[axis][i as usize].stabbing;
            while s != 0 {
                let b = self.bounds[axis][i as usize];
                if b.is_lower() && li <= self.proxies[b.proxy_id as usize].upper[axis] {
                    self.increment_overlap_count(b.proxy_id);
                    s -= 1;
                }
                i -= 1;
            }
        }
        (li, ui)
    }

    /// Debug: raw quantised bounds `[x lower, x upper, y lower, y upper]` of every live proxy id (index into the bound arrays included).
    pub fn debug_proxy(&self, id: u16) -> [(usize, u16); 4] {
        let p = &self.proxies[id as usize];
        [
            (p.lower[0], self.bounds[0][p.lower[0]].value),
            (p.upper[0], self.bounds[0][p.upper[0]].value),
            (p.lower[1], self.bounds[1][p.lower[1]].value),
            (p.upper[1], self.bounds[1][p.upper[1]].value),
        ]
    }

    fn add_buffered_pair(&mut self, id1: u16, id2: u16) {
        let (a, b) = if id1 > id2 { (id2, id1) } else { (id1, id2) };
        let pair = self.pairs.entry((a, b)).or_insert(Pair { id1: a, id2: b, status: 0, user_data: None });
        if pair.status & BUFFERED == 0 {
            pair.status |= BUFFERED;
            self.pair_buffer.push((a, b));
        }
        pair.status &= !REMOVED;
    }

    fn remove_buffered_pair(&mut self, id1: u16, id2: u16) {
        let (a, b) = if id1 > id2 { (id2, id1) } else { (id1, id2) };
        if let Some(pair) = self.pairs.get_mut(&(a, b)) {
            if pair.status & BUFFERED == 0 {
                pair.status |= BUFFERED;
                self.pair_buffer.push((a, b));
            }
            pair.status |= REMOVED;
        }
    }

    /// `b2PairManager.Commit`: runs the callbacks in buffer order.
    pub fn commit(&mut self, cb: &mut dyn FnMut(PairEvent) -> Option<usize>) {
        let buffer = std::mem::take(&mut self.pair_buffer);
        let mut removed = Vec::new();
        for (a, b) in buffer {
            let Some(mut pair) = self.pairs.get(&(a, b)).copied() else { continue };
            pair.status &= !BUFFERED;
            let u1 = self.proxies[pair.id1 as usize].user_data.unwrap_or(usize::MAX);
            let u2 = self.proxies[pair.id2 as usize].user_data.unwrap_or(usize::MAX);
            if pair.status & REMOVED != 0 {
                if pair.status & FINAL != 0 {
                    cb(PairEvent::Removed(u1, u2, pair.user_data));
                }
                removed.push((a, b));
                self.pairs.insert((a, b), pair);
            } else if pair.status & FINAL == 0 {
                pair.user_data = cb(PairEvent::Added(u1, u2));
                pair.status |= FINAL;
                self.pairs.insert((a, b), pair);
            } else {
                self.pairs.insert((a, b), pair);
            }
        }
        for k in removed {
            self.pairs.remove(&k);
        }
    }

    pub fn create_proxy(&mut self, aabb: &Aabb, user_data: usize, cb: &mut dyn FnMut(PairEvent) -> Option<usize>) -> u16 {
        let id = self.free_proxy;
        self.free_proxy = self.proxies[id as usize].next;
        self.proxies[id as usize].overlap_count = 0;
        self.proxies[id as usize].user_data = Some(user_data);
        let bound_count = 2 * self.proxy_count;
        let bv = self.compute_bounds(aabb);
        for axis in 0..2 {
            let (l, d) = self.query(bv.lower[axis], bv.upper[axis], axis, bound_count);
            let b = &mut self.bounds[axis];
            b.copy_within(d..bound_count, d + 2);
            b.copy_within(l..d, l + 1);
            let d = d + 1;
            b[l] = Bound { value: bv.lower[axis], proxy_id: id, stabbing: if l == 0 { 0 } else { b[l - 1].stabbing } };
            b[d] = Bound { value: bv.upper[axis], proxy_id: id, stabbing: b[d - 1].stabbing };
            for n in l..d {
                b[n].stabbing += 1;
            }
            for n in l..bound_count + 2 {
                let bd = self.bounds[axis][n];
                let p = &mut self.proxies[bd.proxy_id as usize];
                if bd.is_lower() {
                    p.lower[axis] = n;
                } else {
                    p.upper[axis] = n;
                }
            }
        }
        self.proxy_count += 1;
        let results = std::mem::take(&mut self.query_results);
        for r in results {
            self.add_buffered_pair(id, r);
        }
        self.commit(cb);
        self.increment_time_stamp();
        id
    }

    pub fn destroy_proxy(&mut self, id: u16, cb: &mut dyn FnMut(PairEvent) -> Option<usize>) {
        let p = self.proxies[id as usize];
        let s = 2 * self.proxy_count;
        for axis in 0..2 {
            let (l, d) = (p.lower[axis], p.upper[axis]);
            let (lv, uv) = (self.bounds[axis][l].value, self.bounds[axis][d].value);
            let b = &mut self.bounds[axis];
            b.copy_within(l + 1..d, l);
            b.copy_within(d + 1..s, d - 1);
            for v in l..s - 2 {
                let bd = self.bounds[axis][v];
                let pp = &mut self.proxies[bd.proxy_id as usize];
                if bd.is_lower() {
                    pp.lower[axis] = v;
                } else {
                    pp.upper[axis] = v;
                }
            }
            for g in l..d - 1 {
                self.bounds[axis][g].stabbing -= 1;
            }
            self.query(lv, uv, axis, s - 2);
        }
        let results = std::mem::take(&mut self.query_results);
        for r in results {
            self.remove_buffered_pair(id, r);
        }
        self.commit(cb);
        self.increment_time_stamp();
        let pr = &mut self.proxies[id as usize];
        pr.user_data = None;
        pr.overlap_count = INVALID;
        pr.next = self.free_proxy;
        self.free_proxy = id;
        self.proxy_count -= 1;
    }

    /// `MoveProxy`: updates endpoints, buffering pair additions/removals (call `commit` afterwards).
    pub fn move_proxy(&mut self, id: u16, aabb: &Aabb) {
        if id == NULL_PROXY || id as usize >= MAX_PROXIES {
            return;
        }
        let count = 2 * self.proxy_count;
        let new = self.compute_bounds(aabb);
        let p0 = self.proxies[id as usize];
        let mut old = BoundValues::default();
        for i in 0..2 {
            old.lower[i] = self.bounds[i][p0.lower[i]].value;
            old.upper[i] = self.bounds[i][p0.upper[i]].value;
        }
        for i in 0..2 {
            let g = self.proxies[id as usize].lower[i];
            let w = self.proxies[id as usize].upper[i];
            let x = new.lower[i];
            let b = new.upper[i];
            let t = x as i32 - self.bounds[i][g].value as i32;
            self.bounds[i][g].value = x;
            let a_ = b as i32 - self.bounds[i][w].value as i32;
            self.bounds[i][w].value = b;
            if t < 0 {
                let mut s = g;
                while s > 0 && x < self.bounds[i][s - 1].value {
                    let o = self.bounds[i][s - 1];
                    let c = o.proxy_id;
                    self.bounds[i][s - 1].stabbing += 1;
                    if o.is_upper() {
                        let sp = self.proxies[c as usize];
                        if self.test_overlap(&new, &sp) {
                            self.add_buffered_pair(id, c);
                        }
                        self.proxies[c as usize].upper[i] += 1;
                        self.bounds[i][s].stabbing += 1;
                    } else {
                        self.proxies[c as usize].lower[i] += 1;
                        self.bounds[i][s].stabbing -= 1;
                    }
                    self.proxies[id as usize].lower[i] -= 1;
                    self.bounds[i].swap(s, s - 1);
                    s -= 1;
                }
            }
            if a_ > 0 {
                let mut s = w;
                while s + 1 < count && self.bounds[i][s + 1].value <= b {
                    let h = self.bounds[i][s + 1];
                    let l = h.proxy_id;
                    self.bounds[i][s + 1].stabbing += 1;
                    if h.is_lower() {
                        let d = self.proxies[l as usize];
                        if self.test_overlap(&new, &d) {
                            self.add_buffered_pair(id, l);
                        }
                        self.proxies[l as usize].lower[i] -= 1;
                        self.bounds[i][s].stabbing += 1;
                    } else {
                        self.proxies[l as usize].upper[i] -= 1;
                        self.bounds[i][s].stabbing -= 1;
                    }
                    self.proxies[id as usize].upper[i] += 1;
                    self.bounds[i].swap(s, s + 1);
                    s += 1;
                }
            }
            if t > 0 {
                let mut s = g;
                while s + 1 < count && self.bounds[i][s + 1].value <= x {
                    let h = self.bounds[i][s + 1];
                    let l = h.proxy_id;
                    self.bounds[i][s + 1].stabbing -= 1;
                    if h.is_upper() {
                        let d = self.proxies[l as usize];
                        if self.test_overlap(&old, &d) {
                            self.remove_buffered_pair(id, l);
                        }
                        self.proxies[l as usize].upper[i] -= 1;
                        self.bounds[i][s].stabbing -= 1;
                    } else {
                        self.proxies[l as usize].lower[i] -= 1;
                        self.bounds[i][s].stabbing += 1;
                    }
                    self.proxies[id as usize].lower[i] += 1;
                    self.bounds[i].swap(s, s + 1);
                    s += 1;
                }
            }
            if a_ < 0 {
                let mut s = w;
                while s > 0 && b < self.bounds[i][s - 1].value {
                    let o = self.bounds[i][s - 1];
                    let c = o.proxy_id;
                    self.bounds[i][s - 1].stabbing -= 1;
                    if o.is_lower() {
                        let sp = self.proxies[c as usize];
                        if self.test_overlap(&old, &sp) {
                            self.remove_buffered_pair(id, c);
                        }
                        self.proxies[c as usize].lower[i] += 1;
                        self.bounds[i][s].stabbing -= 1;
                    } else {
                        self.proxies[c as usize].upper[i] += 1;
                        self.bounds[i][s].stabbing += 1;
                    }
                    self.proxies[id as usize].upper[i] -= 1;
                    self.bounds[i].swap(s, s - 1);
                    s -= 1;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn aabb(x0: f64, y0: f64, x1: f64, y1: f64) -> Aabb {
        Aabb { lower: Vec2::new(x0, y0), upper: Vec2::new(x1, y1) }
    }

    #[test]
    fn pairs_appear_and_disappear_in_order() {
        let mut bp = BroadPhase::new(Vec2::new(-100.0, -100.0), Vec2::new(100.0, 100.0));
        let mut log: Vec<String> = Vec::new();
        let mut cb = |e: PairEvent| -> Option<usize> {
            match e {
                PairEvent::Added(a, b) => log.push(format!("+{a}-{b}")),
                PairEvent::Removed(a, b, _) => log.push(format!("-{a}-{b}")),
            }
            Some(0)
        };
        let a = bp.create_proxy(&aabb(0.0, 0.0, 10.0, 10.0), 10, &mut cb);
        let _b = bp.create_proxy(&aabb(5.0, 5.0, 15.0, 15.0), 11, &mut cb);
        let _c = bp.create_proxy(&aabb(50.0, 50.0, 60.0, 60.0), 12, &mut cb);
        bp.move_proxy(a, &aabb(40.0, 40.0, 55.0, 55.0));
        bp.commit(&mut cb);
        bp.move_proxy(a, &aabb(0.0, 0.0, 10.0, 10.0));
        bp.commit(&mut cb);
        // initial overlap a-b is reported when b is created
        assert_eq!(log[0], "+10-11");
        assert!(log.contains(&"-10-11".to_string()) && log.contains(&"+10-12".to_string()), "{log:?}");
        assert!(log.contains(&"-10-12".to_string()), "{log:?}");
        assert_eq!(log.iter().filter(|e| e.as_str() == "+10-11").count(), 2, "{log:?}");
    }
}
