//! Blood particles (the original's `ParticleController` emitters `MP` burst / `MI` flow and `BloodParticle`, bloodSetting 1/2 look: one
//! 0x990000 line segment per particle per frame). Positions are in pixels; velocities arrive in m/s and are scaled by 62.5/30 like
//! `BloodParticle.scaler`; gravity is `gravityIncrement` = 10 m/s² in px/frame². Particles die below `YParticleLimit` (850 px).

use box2d::math::Vec2;

const SCALER: f64 = 62.5 / 30.0;
const GRAVITY: f64 = 10.0 * 62.5 / 900.0;
const Y_LIMIT: f64 = 850.0;
const MAX_PARTICLES: usize = 2000;

#[derive(Clone, Copy)]
pub struct Particle {
    pub cur: [f64; 2],
    pub last: [f64; 2],
}

impl Particle {
    fn new(x: f64, y: f64, vx: f64, vy: f64) -> Self {
        Self { cur: [x + vx * SCALER, y + vy * SCALER], last: [x, y] }
    }

    /// `BloodParticle.step` without drawing: Verlet step, `false` once below the limit.
    fn step(&mut self) -> bool {
        let dx = self.cur[0] - self.last[0];
        let dy = self.cur[1] - self.last[1];
        self.last = self.cur;
        self.cur = [self.cur[0] + dx, self.cur[1] + dy + GRAVITY];
        self.last[1] <= Y_LIMIT
    }
}

/// `MI`: spews one particle per frame from a point on a body until `total` is exceeded (or `stop`ped).
pub struct Flow {
    pub body: usize,
    pub offset: Vec2,
    pub angle: f64,
    pub min_speed: f64,
    pub speed_range: f64,
    pub total: u32,
    pub count: u32,
    pub finished: bool,
}

#[derive(Default)]
pub struct Blood {
    pub particles: Vec<Particle>,
    pub flows: Vec<Flow>,
    rng: u64,
}

impl Blood {
    pub fn rand(&mut self) -> f64 {
        if self.rng == 0 {
            self.rng = 0x9E37_79B9_7F4A_7C15;
        }
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 11) as f64 / (1u64 << 53) as f64
    }

    /// `MP`/`Md.createParticles`: `total` particles around `start` (px), `vel` the source's velocity (m/s).
    pub fn burst(&mut self, start: [f64; 2], vel: Vec2, initial_range: f64, speed_range: f64, total: u32) {
        for _ in 0..total {
            if self.particles.len() >= MAX_PARTICLES {
                return;
            }
            let vx = vel.x + self.rand() * (self.rand() * speed_range - speed_range / 2.0);
            let vy = vel.y + self.rand() * (self.rand() * speed_range - speed_range * 0.75);
            let x = start[0] + self.rand() * initial_range - initial_range / 2.0;
            let y = start[1] + self.rand() * initial_range - initial_range / 2.0;
            self.particles.push(Particle::new(x, y, vx, vy));
        }
    }

    pub fn spawn(&mut self, at: [f64; 2], vel: [f64; 2]) {
        if self.particles.len() < MAX_PARTICLES {
            self.particles.push(Particle::new(at[0], at[1], vel[0], vel[1]));
        }
    }

    /// Advances every particle one frame and drops the dead ones.
    pub fn step_particles(&mut self) {
        self.particles.retain_mut(Particle::step);
    }
}
