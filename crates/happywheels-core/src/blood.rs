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
    /// Sprite flow (`createFlow` of a debris clip) instead of blood.
    pub sprite: Option<ShardKind>,
}

/// A sprite particle (`MA`): one frame of a debris clip flying under gravity (`createPointBurst`).
#[derive(Clone, Copy)]
pub struct SpriteParticle {
    pub kind: ShardKind,
    pub frame: usize,
    /// Centre of the bitmap in world px.
    pub pos: [f64; 2],
    vel: [f64; 2],
}

/// Which debris clip a burst uses: (clip name, enclosing clip, frame count) — `createBMDArray` sources of the original.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ShardKind {
    Basket,
    Copter,
    Cart,
    Crackers,
    Sleigh,
    Mower,
    /// `jetshards` (the `metalShards` clip of the chair / dirt bike).
    Metal,
    /// Cola spilling from the cart's soda can.
    Cola,
    /// Snow thrown off the boosting sleigh (`snowflakes`, class `Mb`: half gravity).
    Snow,
    /// Juice/flesh particles of a smashed food item (`WatermelonParticlesMC` / `PumpkinParticlesMC` / `PineappleParticlesMC`); 3 = `BoomboxShardsMC`, 4 = `TVShardsMC`, 5 = `GlassShardsMC`.
    Food(u8),
}

impl ShardKind {
    pub fn clip(self) -> (&'static str, Option<&'static str>, usize) {
        match self {
            Self::Basket => ("basketPieces", Some("son"), 15),
            Self::Copter => ("copterShards", None, 16),
            Self::Cart => ("cartPieces", None, 10),
            Self::Crackers => ("crackers", None, 11),
            Self::Sleigh => ("sleighShards", None, 8),
            Self::Mower => ("mowerShards", None, 12),
            Self::Metal => ("metalShards", None, 12),
            Self::Cola => ("cola", None, 13),
            Self::Snow => ("snowflakes", None, 7),
            Self::Food(ty) => ("", None, [15, 9, 8, 6, 9, 11][ty as usize % 6]),
        }
    }
}

#[derive(Default)]
pub struct Blood {
    pub particles: Vec<Particle>,
    pub sparks: Vec<Spark>,
    pub sprites: Vec<SpriteParticle>,
    pub flows: Vec<Flow>,
    rng: u64,
}

/// `SparkBurst` particle (`MV`): a one-pixel line in a 0xFFFF00 + random-blue colour that decays by .65 per step and falls under 1/3·62.5/30 px/step².
#[derive(Clone, Copy)]
pub struct Spark {
    pub pos: [f64; 2],
    pub speed: [f64; 2],
    pub gravity: f64,
    /// Segment drawn this step: (from, to).
    pub seg: [[f64; 2]; 2],
    pub blue: u8,
}

impl Spark {
    /// `MV.step`: draw the tail segment, move, decay; `false` once both speeds are below 1 px.
    fn step(&mut self) -> bool {
        let change = self.speed[1] + self.gravity;
        self.seg = [[self.pos[0] - self.speed[0], self.pos[1] - change], self.pos];
        self.pos[1] += change;
        self.pos[0] += self.speed[0];
        self.speed[0] *= 0.65;
        self.speed[1] *= 0.65;
        self.gravity += 1.0 / 3.0 * 62.5 / 30.0;
        self.speed[1].abs() >= 1.0 || self.speed[0].abs() >= 1.0
    }
}

impl Blood {
    /// `createSparkBurstPoint(start (m), vel (px/step), initialRange, speedRange, total)`: the particles' speeds are in px per step already.
    pub fn spark_burst(&mut self, start: Vec2, vel: Vec2, initial_range: f64, speed_range: f64, total: u32) {
        let scaler = 62.5 / 30.0;
        for _ in 0..total {
            let vx = vel.x + self.rand() * (self.rand() * speed_range - speed_range / 2.0);
            let vy = vel.y + self.rand() * (self.rand() * speed_range - speed_range / 2.0);
            let x = start.x * 62.5 + self.rand() * initial_range - initial_range / 2.0;
            let y = start.y * 62.5 + self.rand() * initial_range - initial_range / 2.0;
            let blue = (self.rand() * 255.0).round() as u8;
            self.sparks.push(Spark { pos: [x, y], speed: [vx * scaler, vy * scaler], gravity: 0.0, seg: [[x, y], [x, y]], blue });
        }
    }

    pub fn step_sparks(&mut self) {
        self.sparks.retain_mut(Spark::step);
    }

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

    /// `Mw` point burst of `total` sprite particles (`MA`): random frame, speeds `rand·(rand·speed − speed/2)` (x) and `rand·(rand·speed − ¾·speed)` (y)
    /// in units of 62.5/30 px per step, gravity 62.5/90 px per step².
    pub fn sprite_burst(&mut self, kind: ShardKind, start: [f64; 2], range: f64, speed: f64, total: u32) {
        let frames = kind.clip().2;
        for _ in 0..total {
            if self.particles.len() + self.sprites.len() >= MAX_PARTICLES {
                return;
            }
            let vx = self.rand() * (self.rand() * speed - speed / 2.0);
            let vy = self.rand() * (self.rand() * speed - speed * 0.75);
            let frame = (self.rand() * frames as f64).floor() as usize;
            let x = start[0] + self.rand() * range - range / 2.0;
            let y = start[1] + self.rand() * range - range / 2.0;
            let k = 62.5 / 30.0;
            self.sprites.push(SpriteParticle { kind, frame: frame.min(frames - 1), pos: [x, y], vel: [vx * k, vy * k] });
        }
    }

    /// One particle of a sprite flow (`Mk.createParticle`): velocity in m/s, centred on `at` (px).
    pub fn spawn_sprite(&mut self, kind: ShardKind, at: [f64; 2], vel: [f64; 2]) {
        if self.particles.len() + self.sprites.len() >= MAX_PARTICLES {
            return;
        }
        let frames = kind.clip().2;
        let frame = ((self.rand() * frames as f64).floor() as usize).min(frames - 1);
        let k = 62.5 / 30.0;
        self.sprites.push(SpriteParticle { kind, frame, pos: at, vel: [vel[0] * k, vel[1] * k] });
    }

    /// Advances the sprite particles one step (dropped below `Y_LIMIT`).
    pub fn step_sprites(&mut self) {
        self.sprites.retain_mut(|p| {
            p.pos[1] += p.vel[1];
            p.pos[0] += p.vel[0];
            p.vel[1] += if p.kind == ShardKind::Snow { 62.5 / 180.0 } else { 62.5 / 90.0 };
            p.pos[1] <= Y_LIMIT
        });
    }

    /// Advances every particle one frame and drops the dead ones.
    pub fn step_particles(&mut self) {
        self.particles.retain_mut(Particle::step);
    }
}
