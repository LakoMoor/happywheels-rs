//! The session camera (`Camera` in the original): the view follows the focus body only when it leaves a dead zone whose borders move with the focus
//! velocity, and stays inside the level's `cameraBounds` (user levels: 20000 × 10000 px). The view is 900 × 500 px.

use box2d::math::Vec2;

pub const VIEW: [f64; 2] = [900.0, 500.0];
const PS: f64 = 62.5;
const MIN_SPEED: f64 = 3.0;
const MOVE_INCREMENT: f64 = 5.0;
const UP_INCREMENT: f64 = 1.0;
const TEMP_MAX: f64 = 20.0;

/// The focus body as the camera sees it: identity, origin (m) and linear velocity (m/s).
#[derive(Clone, Copy, Debug)]
pub struct Focus {
    pub body: usize,
    pub position: Vec2,
    pub velocity: Vec2,
}

#[derive(Clone, Copy, Debug)]
pub struct Camera {
    /// `containerObj.x/y`: minus the top-left corner of the view in world px.
    pub container: [f64; 2],
    /// left, right, top, bottom borders of the dead zone (screen px).
    pub borders: [f64; 4],
    /// left, right, top, bottom limits of `container`.
    limits: [f64; 4],
    /// How much the view moved in the last step (`screenMotion`), zero when the focus changed.
    pub motion: [f64; 2],
    /// `YParticleLimit`: particles below this are dropped.
    pub particle_limit: f64,
    temp_count: f64,
    stepped: Option<(usize, Option<usize>)>,
}

impl Default for Camera {
    fn default() -> Self {
        // setLimits(0, 20000, 0, 10000)
        Camera { container: [0.0, 0.0], borders: [100.0, 200.0, 100.0, 150.0], limits: [-0.0, -(20000.0 - VIEW[0]), -0.0, -(10000.0 - VIEW[1])], motion: [0.0, 0.0], particle_limit: 850.0, temp_count: TEMP_MAX + 1.0, stepped: None }
    }
}

impl Camera {
    /// `setLimits(left, right, top, bottom)` of the level's camera bounds.
    pub fn set_limits(&mut self, left: f64, right: f64, top: f64, bottom: f64) {
        self.limits = [-left, -(right - VIEW[0]), -top, -(bottom - VIEW[1])];
    }

    /// Centre of the view in world px.
    pub fn center(&self) -> [f64; 2] {
        [VIEW[0] / 2.0 - self.container[0], VIEW[1] / 2.0 - self.container[1]]
    }

    /// `removeSecondFocus` starts easing from the old second focus (the caller keeps passing it as `temp`).
    pub fn start_easing(&mut self) {
        self.temp_count = 0.0;
    }

    pub fn easing(&self) -> bool {
        self.temp_count <= TEMP_MAX
    }

    fn set_borders(&mut self, v: Vec2, dt: f64) {
        let toward = |value: &mut f64, target: f64, up: f64| {
            if *value < target {
                *value += up * dt;
            } else if *value > target {
                *value -= MOVE_INCREMENT * dt;
            }
        };
        let [left, right, top, bottom] = &mut self.borders;
        let (l, r) = if v.x > MIN_SPEED { (100.0, 200.0) } else if v.x < -MIN_SPEED { (600.0, 700.0) } else { (350.0, 450.0) };
        toward(left, l, MOVE_INCREMENT);
        toward(right, r, MOVE_INCREMENT);
        // vertically the borders grow by `upInc` but shrink by `moveIncrement`
        let (t, b) = if v.y > MIN_SPEED { (100.0, 150.0) } else if v.y < -MIN_SPEED { (350.0, 400.0) } else { (300.0, 350.0) };
        toward(top, t, UP_INCREMENT);
        toward(bottom, b, UP_INCREMENT);
    }

    /// `Camera.step(dt)`. `second` is the helicopter's magnet; `temp` the same body after it was let go (while `easing`).
    pub fn step(&mut self, focus: Focus, second: Option<Focus>, temp: Option<Focus>, dt: f64) {
        let dt = dt.clamp(0.0, 1.0);
        let before = self.container;
        self.set_borders(focus.velocity, dt);
        let mut at = [focus.position.x, focus.position.y];
        if let Some(s) = second {
            at[1] = (at[1] + s.position.y) * 0.5;
        } else if let Some(t) = temp.filter(|_| self.easing()) {
            let c = 0.5 - self.temp_count * 0.5 / TEMP_MAX;
            at[1] = at[1] * (1.0 - c) + t.position.y * c;
            // the old second focus is dropped after the step that reaches the end
            self.temp_count = if self.temp_count >= TEMP_MAX { TEMP_MAX + 1.0 + dt } else { self.temp_count + dt };
        }
        self.follow([at[0] * PS, at[1] * PS]);
        self.particle_limit = focus.position.y * PS + 1000.0;
        let now = (focus.body, second.map(|s| s.body));
        self.motion = if self.stepped == Some(now) { [before[0] - self.container[0], before[1] - self.container[1]] } else { [0.0, 0.0] };
        self.stepped = Some(now);
    }

    /// `center`: move only when the point leaves the dead zone, then clamp that side.
    fn follow(&mut self, p: [f64; 2]) {
        let [left, right, top, bottom] = self.borders;
        let [left_limit, right_limit, top_limit, bottom_limit] = self.limits;
        let x = p[0] + self.container[0];
        let y = p[1] + self.container[1];
        if x > right {
            self.container[0] -= x - right;
            if self.container[0] < right_limit {
                self.container[0] = right_limit;
            }
        }
        if x < left {
            self.container[0] += left - x;
            if self.container[0] > left_limit {
                self.container[0] = left_limit;
            }
        }
        if y > bottom {
            self.container[1] -= y - bottom;
            if self.container[1] < bottom_limit {
                self.container[1] = bottom_limit;
            }
        }
        if y < top {
            self.container[1] += top - y;
            if self.container[1] > top_limit {
                self.container[1] = top_limit;
            }
        }
    }
}
