//! Blood particles (the original's `ParticleController` emitters `MP` burst / `MI` flow and `BloodParticle`, bloodSetting 1/2 look: one
//! 0x990000 line segment per particle per frame). Positions are in pixels; velocities arrive in m/s and are scaled by 62.5/30 like
//! `BloodParticle.scaler`; gravity is `gravityIncrement` = 10 m/s² in px/frame². Particles die below `YParticleLimit` (850 px).

use box2d::math::Vec2;

const SCALER: f64 = 62.5 / 30.0;
const GRAVITY: f64 = 10.0 * 62.5 / 900.0;
const Y_LIMIT: f64 = 850.0;
/// `ParticleController.maxParticles` (options slider 0..2000, default 2000), process-wide like the blood setting.
static MAX_PARTICLES_SETTING: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(2000);

/// Sets the particle cap new emissions respect (the options screen's "maximum particles" slider, clamped to 0..=2000).
pub fn set_max_particles(max: usize) {
    MAX_PARTICLES_SETTING.store(max.min(2000), std::sync::atomic::Ordering::Relaxed);
}

fn max_particles() -> usize {
    MAX_PARTICLES_SETTING.load(std::sync::atomic::Ordering::Relaxed)
}
/// `ShardKind::Food` index of the 13-frame `Blood` clip (bloodSetting 1 particles).
pub const BLOOD: u8 = 8;
/// `ShardKind::Food` index of the 13-frame `WaterMC` clip (toilet water sprays).
pub const WATER: u8 = 9;

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
            Self::Food(ty) => ("", None, [15, 9, 8, 6, 9, 11, 5, 13, 13, 13][ty as usize % 10]),
        }
    }
}

/// Process-wide `bloodSetting` used by new worlds: 0 = no blood, 1 = bitmap particles (default), 2 = line particles.
static BLOOD_SETTING: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(1);

/// Sets the `bloodSetting` new worlds start with (values above 2, the original's metaball looks, behave like 2).
pub fn set_blood_setting(setting: u8) {
    BLOOD_SETTING.store(setting, std::sync::atomic::Ordering::Relaxed);
}

impl Default for Blood {
    fn default() -> Self {
        let setting = BLOOD_SETTING.load(std::sync::atomic::Ordering::Relaxed);
        Self { particles: Vec::new(), sparks: Vec::new(), sprites: Vec::new(), flows: Vec::new(), line_blood: setting >= 2, off: setting == 0, setting, max: max_particles(), rng: 0 }
    }
}

pub struct Blood {
    pub particles: Vec<Particle>,
    pub sparks: Vec<Spark>,
    pub sprites: Vec<SpriteParticle>,
    pub flows: Vec<Flow>,
    /// `bloodSetting` 2 look (`BloodParticleLine`: a dark red line per particle); off = the default setting 1 (`Blood` bitmap sprites).
    pub line_blood: bool,
    /// `bloodSetting` 0: `createBloodBurst/Flow` return empty emitters, nothing is spawned.
    pub off: bool,
    /// The raw `bloodSetting` (3 and 4 are the metaball looks: line particles blurred and thresholded, see `render`).
    pub setting: u8,
    /// Particle cap of this world (`maxParticles` when it was created).
    pub max: usize,
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

    /// Live blood particles of either look (lines or `Blood` sprites).
    pub fn live(&self) -> usize {
        self.particles.len() + self.sprites.iter().filter(|s| s.kind == ShardKind::Food(BLOOD)).count()
    }

    /// `MP`/`Md.createParticles`: `total` particles around `start` (px), `vel` the source's velocity (m/s).
    pub fn burst(&mut self, start: [f64; 2], vel: Vec2, initial_range: f64, speed_range: f64, total: u32) {
        if self.off {
            return;
        }
        for _ in 0..total {
            if self.particles.len() + self.sprites.len() >= self.max {
                return;
            }
            let vx = vel.x + self.rand() * (self.rand() * speed_range - speed_range / 2.0);
            let vy = vel.y + self.rand() * (self.rand() * speed_range - speed_range * 0.75);
            let x = start[0] + self.rand() * initial_range - initial_range / 2.0;
            let y = start[1] + self.rand() * initial_range - initial_range / 2.0;
            if self.line_blood {
                self.particles.push(Particle::new(x, y, vx, vy));
            } else {
                self.spawn_sprite(ShardKind::Food(BLOOD), [x, y], [vx, vy]);
            }
        }
    }

    pub fn spawn(&mut self, at: [f64; 2], vel: [f64; 2]) {
        if self.off {
            return;
        }
        if !self.line_blood {
            self.spawn_sprite(ShardKind::Food(BLOOD), at, vel);
        } else if self.particles.len() < self.max {
            self.particles.push(Particle::new(at[0], at[1], vel[0], vel[1]));
        }
    }

    /// `Mw` point burst of `total` sprite particles (`MA`): random frame, speeds `rand·(rand·speed − speed/2)` (x) and `rand·(rand·speed − ¾·speed)` (y)
    /// in units of 62.5/30 px per step, gravity 62.5/90 px per step².
    pub fn sprite_burst(&mut self, kind: ShardKind, start: [f64; 2], range: f64, speed: f64, total: u32) {
        let frames = kind.clip().2;
        for _ in 0..total {
            if self.particles.len() + self.sprites.len() >= self.max {
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
        if self.particles.len() + self.sprites.len() >= self.max {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn with(setting: u8) -> Blood {
        let mut b = Blood { line_blood: setting >= 2, off: setting == 0, ..Blood::default() };
        b.burst([0.0, 0.0], Vec2::new(0.0, 0.0), 10.0, 10.0, 20);
        b.spawn([0.0, 0.0], [1.0, 1.0]);
        b
    }

    #[test]
    fn blood_settings_choose_the_particle_kind() {
        let off = with(0);
        assert_eq!(off.live(), 0, "setting 0 spawns nothing");
        let sprites = with(1);
        assert_eq!((sprites.particles.len(), sprites.live()), (0, 21), "setting 1: bitmap particles");
        let lines = with(2);
        assert_eq!((lines.particles.len(), lines.sprites.len()), (21, 0), "setting 2: lines");
    }
}

#[cfg(test)]
mod max_particle_tests {
    use super::*;

    #[test]
    fn the_particle_cap_limits_new_emissions() {
        let mut b = Blood { max: 25, ..Blood::default() };
        b.burst([0.0, 0.0], Vec2::default(), 5.0, 15.0, 200);
        let n = b.particles.len() + b.sprites.len();
        assert!(n <= 25, "capped at 25, saw {n}");
        let mut c = Blood::default();
        c.burst([0.0, 0.0], Vec2::default(), 5.0, 15.0, 200);
        assert!(c.particles.len() + c.sprites.len() > 25, "default cap lets them through");
    }
}
