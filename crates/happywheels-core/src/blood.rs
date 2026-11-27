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
    fn step(&mut self, dt: f64) -> bool {
        let dx = self.cur[0] - self.last[0];
        let dy = self.cur[1] - self.last[1];
        self.last = self.cur;
        // original: the Verlet delta is not scaled by dt, only the gravity is (`gravityIncrement * dt * dt`)
        self.cur = [self.cur[0] + dx, self.cur[1] + dy + GRAVITY * dt * dt];
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
    /// `_accumulatedDelta`: the emitter only creates its particle on the step where this is 0 (once per 30 Hz tick, whatever dt is).
    pub acc: f64,
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
        Self { particles: Vec::new(), sparks: Vec::new(), sprites: Vec::new(), flows: Vec::new(), line_blood: setting >= 2, off: setting == 0, setting, max: max_particles(), rng: 0, #[cfg(test)] script: Default::default() }
    }
}

pub struct Blood {
    /// Unit tests only: values returned by `rand` before the generator is used (the oracle runs replace `Math.random` the same way).
    #[cfg(test)]
    pub script: std::collections::VecDeque<f64>,
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
    fn step(&mut self, dt: f64) -> bool {
        let change = self.speed[1] + self.gravity;
        self.seg = [[self.pos[0] - self.speed[0], self.pos[1] - change], self.pos];
        self.pos[1] += change * dt;
        self.pos[0] += self.speed[0] * dt;
        // original `speed *= decayRate * dt` (so the decay is 0.325 per half step at 60 fps)
        self.speed[0] *= 0.65 * dt;
        self.speed[1] *= 0.65 * dt;
        self.gravity += 1.0 / 3.0 * 62.5 / 30.0 * dt;
        self.speed[1].abs() >= 1.0 || self.speed[0].abs() >= 1.0
    }
}

impl SpriteParticle {
    /// Pixels per 30 Hz step.
    pub fn velocity(&self) -> [f64; 2] {
        self.vel
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

    pub fn step_sparks(&mut self, dt: f64) {
        self.sparks.retain_mut(|s| s.step(dt));
    }

    pub fn rand(&mut self) -> f64 {
        #[cfg(test)]
        if let Some(v) = self.script.pop_front() {
            return v;
        }
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
    pub fn step_sprites(&mut self, dt: f64) {
        self.sprites.retain_mut(|p| {
            p.pos[1] += p.vel[1] * dt;
            p.pos[0] += p.vel[0] * dt;
            p.vel[1] += if p.kind == ShardKind::Snow { 62.5 / 180.0 } else { 62.5 / 90.0 } * dt;
            // snowflakes only (`Mb`): air drag above 20 px per step, positive directions only
            if p.kind == ShardKind::Snow {
                if p.vel[1] > 20.0 {
                    p.vel[1] *= dt * 0.9;
                }
                if p.vel[0] > 20.0 {
                    p.vel[0] *= dt * 0.9;
                }
            }
            p.pos[1] <= Y_LIMIT
        });
    }

    /// Advances every particle one frame and drops the dead ones.
    pub fn step_particles(&mut self, dt: f64) {
        self.particles.retain_mut(|p| p.step(dt));
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

#[cfg(test)]
mod dt_tests {
    use super::*;

    /// 60 fps: two half-dt steps per 30 Hz tick; the Verlet delta is not scaled (original), so the particle ends further than with one dt = 1 step, and
    /// sparks decay by 0.65·dt per step.
    #[test]
    fn particle_steps_take_dt_like_the_original() {
        let mut p = Particle::new(0.0, 0.0, 1.0, 0.0);
        let first = p.cur;
        assert!(p.step(0.5));
        let dx = p.cur[0] - first[0];
        assert!((dx - (first[0] - 0.0)).abs() < 1e-12, "x delta is not scaled by dt");
        let mut s = Spark { pos: [0.0, 0.0], speed: [10.0, 0.0], gravity: 0.0, seg: [[0.0; 2]; 2], blue: 0 };
        assert!(s.step(0.5));
        assert!((s.speed[0] - 3.25).abs() < 1e-12 && (s.pos[0] - 5.0).abs() < 1e-12);
    }
}

#[cfg(test)]
mod oracle_dt_tests {
    use super::*;

    /// Spark particle of the real `ParticleController` (`analysis/particle-dt.js`: `createSparkBurstPoint`, then `step(dt)` six times): position, speed and
    /// gravity speed after each step must match bit for bit for dt 0.5 (60 fps) and dt 1 (30 fps).
    #[test]
    fn spark_steps_match_the_original_for_dt_half_and_one() {
        for (dt, init, traj) in [(0.5f64, [625.001885513775, 624.7208843890112, -0.8191573294687974, -30.685581768128035, 0.0], [[624.5923068490406, 609.3780935049472, -0.2662261320773592, -9.972814074641612, 0.3472222222222222], [624.4591937830019, 604.5652975787375, -0.08652349292514173, -3.241164574258524, 0.6944444444444444], [624.4159320365393, 603.2919375138305, -0.028120135200671064, -1.0533784866340203, 1.0416666666666665], [624.401871968939, 603.2860816038468, -0.009139043940218096, -0.34234800815605665, 1.3888888888888888], [624.401871968939, 603.2860816038468, -0.009139043940218096, -0.34234800815605665, 1.3888888888888888], [624.401871968939, 603.2860816038468, -0.009139043940218096, -0.34234800815605665, 1.3888888888888888]])] {
            let mut s = Spark { pos: [init[0], init[1]], speed: [init[2], init[3]], gravity: init[4], seg: [[0.0; 2]; 2], blue: 0 };
            let mut alive = true;
            for (n, want) in traj.iter().enumerate() {
                // a spark that died is removed from its emitter, so the original stops stepping it
                if alive {
                    alive = s.step(dt);
                }
                assert_eq!([s.pos[0], s.pos[1], s.speed[0], s.speed[1], s.gravity], *want, "dt {dt} step {n}");
            }
        }
        for (dt, init, traj) in [(1f64, [624.8570984005928, 624.9513147098478, -8.620726175674957, -1.064674092958912, 0.0], [[616.2363722249179, 623.8866406168888, -5.603472014188722, -0.6920381604232928, 0.6944444444444444], [610.6329002107292, 623.88904690091, -3.6422568092226695, -0.4498248042751403, 1.3888888888888888], [606.9906434015065, 624.8281109855237, -2.3674669259947354, -0.29238612277884124, 2.083333333333333], [604.6231764755117, 626.6190581960782, -1.538853501896578, -0.1900509798062468, 2.7777777777777777], [603.0843229736151, 629.2067849940497, -1.0002547762327758, -0.12353313687406042, 3.4722222222222223], [602.0840681973824, 632.5554740793979, -0.6501656045513043, -0.08029653896813928, 4.166666666666667]])] {
            let mut s = Spark { pos: [init[0], init[1]], speed: [init[2], init[3]], gravity: init[4], seg: [[0.0; 2]; 2], blue: 0 };
            let mut alive = true;
            for (n, want) in traj.iter().enumerate() {
                // a spark that died is removed from its emitter, so the original stops stepping it
                if alive {
                    alive = s.step(dt);
                }
                assert_eq!([s.pos[0], s.pos[1], s.speed[0], s.speed[1], s.gravity], *want, "dt {dt} step {n}");
            }
        }
    }
}

#[cfg(test)]
mod oracle_blood_dt_tests {
    use super::*;

    /// Blood particles of the real `ParticleController` (`analysis/blood-dt.js`: `createBloodBurst` of 3, then `step(dt)` six times).
    /// Bitmap particles (bloodSetting 1): `[x, y, speedX, speedY]`; line particles (bloodSetting 2): `[cur.x, cur.y, last.x, last.y]`.
    #[test]
    fn blood_particles_match_the_original_for_dt_half_and_one() {
        let bitmap_1: ([[f64; 4]; 3], [[[f64; 4]; 3]; 6]) = ([[1000.3217577373609, 249.27812050743958, -7.278378012532184, 1.5983639195149677], [998.0136616272387, 247.60041922209783, -3.66159927931242, -18.472208385406493], [999.3873018234037, 249.5479408851359, -3.3067255009222176, -5.429726701928677]], [[[993.0433797248288, 250.87648442695456, -7.278378012532184, 2.292808363959412], [994.3520623479262, 229.12821083669132, -3.66159927931242, -17.77776394096205], [996.0805763224815, 244.11821418320721, -3.3067255009222176, -4.735282257484232]], [[985.7650017122967, 253.16929279091397, -7.278378012532184, 2.9872528084038565], [990.6904630686138, 211.35044689572928, -3.66159927931242, -17.083319496517607], [992.7738508215593, 239.38293192572297, -3.3067255009222176, -4.040837813039787]], [[978.4866236997646, 256.15654559931784, -7.278378012532184, 3.681697252848301], [987.0288637893013, 194.26712739921166, -3.66159927931242, -16.388875052073164], [989.4671253206371, 235.34209411268318, -3.3067255009222176, -3.3463933685953426]], [[971.2082456872324, 259.83824285216616, -7.278378012532184, 4.376141697292746], [983.3672645099889, 177.8782523471385, -3.66159927931242, -15.69443060762872], [986.1603998197149, 231.99570074408786, -3.3067255009222176, -2.651948924150898]], [[963.9298676747003, 264.2143845494589, -7.278378012532184, 5.0705861417371905], [979.7056652306765, 162.18382173950977, -3.66159927931242, -14.999986163184275], [982.8536743187927, 229.34375181993695, -3.3067255009222176, -1.9575044797064536]], [[956.6514896621682, 269.28497069119607, -7.278378012532184, 5.765030586181635], [976.044065951364, 147.1838355763255, -3.66159927931242, -14.30554171873983], [979.5469488178705, 227.3862473402305, -3.3067255009222176, -1.2630600352620092]]]);
        let bitmap_0_5: ([[f64; 4]; 3], [[[f64; 4]; 3]; 6]) = ([[996.4626600440126, 250.19009992266072, 1.6115856397355728, -15.914579174360076], [997.2064423163422, 249.3769492734689, 0.4668876024915419, 5.685732441123656], [998.2818976566662, 246.67730202917008, 0.2184371344926356, -13.619267155246192]], [[[997.2684528638804, 242.23281033548068, 1.6115856397355728, -15.567356952137855], [997.439886117588, 252.21981549403074, 0.4668876024915419, 6.032954663345878], [998.3911162239125, 239.867668451547, 0.2184371344926356, -13.27204493302397]], [[998.0742456837481, 234.44913185941175, 1.6115856397355728, -15.220134729915634], [997.6733299188338, 255.23629282570369, 0.4668876024915419, 6.3801768855681], [998.5003347911588, 233.23164598503502, 0.2184371344926356, -12.924822710801749]], [[998.8800385036159, 226.83906449445394, 1.6115856397355728, -14.872912507693412], [997.9067737200796, 258.4263812684877, 0.4668876024915419, 6.727399107790323], [998.6095533584052, 226.76923462963413, 0.2184371344926356, -12.577600488579527]], [[999.6858313234836, 219.40260824060724, 1.6115856397355728, -14.52569028547119], [998.1402175213254, 261.7900808223829, 0.4668876024915419, 7.074621330012545], [998.7187719256515, 220.48043438534435, 0.2184371344926356, -12.230378266357306]], [[1000.4916241433514, 212.13976309787165, 1.6115856397355728, -14.17846806324897], [998.3736613225713, 265.32739148738915, 0.4668876024915419, 7.421843552234767], [998.8279904928978, 214.3652452521657, 0.2184371344926356, -11.883156044135085]], [[1001.2974169632191, 205.05052906624718, 1.6115856397355728, -13.831245841026748], [998.6071051238171, 269.03831326350655, 0.4668876024915419, 7.7690657744569895], [998.9372090601441, 208.42366723009815, 0.2184371344926356, -11.535933821912863]]]);
        let lines_1: ([[f64; 4]; 3], [[[f64; 4]; 3]; 6]) = ([[1002.1308566959425, 249.5025100228174, 998.9687909782399, 249.78339454922823], [989.9856880427787, 243.23641283260406, 999.3418603451457, 248.07134348964317], [1001.6986290704434, 257.05523133003476, 1002.4086565981154, 252.10431780032812]], [[[1005.2929224136451, 249.91606994085103, 1002.1308566959425, 249.5025100228174], [980.6295157404118, 239.0959266200094, 989.9856880427787, 243.23641283260406], [1000.9886015427714, 262.7005893041859, 1001.6986290704434, 257.05523133003476]], [[1008.4549881313477, 251.02407430332912, 1005.2929224136451, 249.91606994085103], [971.2733434380448, 235.6498848518592, 980.6295157404118, 239.0959266200094], [1000.2785740150995, 269.04039172278146, 1000.9886015427714, 262.7005893041859]], [[1011.6170538490503, 252.82652311025166, 1008.4549881313477, 251.02407430332912], [961.9171711356778, 232.89828752815347, 971.2733434380448, 235.6498848518592], [999.5685464874275, 276.0746385858215, 1000.2785740150995, 269.04039172278146]], [[1014.7791195667529, 255.32341636161865, 1011.6170538490503, 252.82652311025166], [952.5609988333108, 230.8411346488922, 961.9171711356778, 232.89828752815347], [998.8585189597555, 283.803329893306, 999.5685464874275, 276.0746385858215]], [[1017.9411852844555, 258.5147540574301, 1014.7791195667529, 255.32341636161865], [943.2048265309438, 229.47842621407537, 952.5609988333108, 230.8411346488922], [998.1484914320836, 292.22646564523495, 998.8585189597555, 283.803329893306]], [[1021.1032510021581, 262.40053619768594, 1017.9411852844555, 258.5147540574301], [933.8486542285768, 228.810162223703, 943.2048265309438, 229.47842621407537], [997.4384639044116, 301.34404584160836, 998.1484914320836, 292.22646564523495]]]);
        let lines_0_5: ([[f64; 4]; 3], [[[f64; 4]; 3]; 6]) = ([[1009.4043010496658, 243.11990904902993, 1001.5349989724812, 252.1450089949183], [995.6877942570987, 238.95612669748525, 1000.8354172846769, 250.93611652329562], [1013.9456057237686, 252.261612054689, 1001.5899368203711, 252.56626803027464]], [[[1017.2736031268505, 234.26842021425267, 1009.4043010496658, 243.11990904902993], [990.5401712295206, 227.14974798278598, 995.6877942570987, 238.95612669748525], [1026.301274627166, 252.1305671902145, 1013.9456057237686, 252.261612054689]], [[1025.1429052040353, 225.59054249058653, 1017.2736031268505, 234.26842021425267], [985.3925482019425, 215.51698037919783, 990.5401712295206, 227.14974798278598], [1038.6569435305637, 252.1731334368511, 1026.301274627166, 252.1305671902145]], [[1033.01220728122, 217.0862758780315, 1025.1429052040353, 225.59054249058653], [980.2449251743644, 204.0578238867208, 985.3925482019425, 215.51698037919783], [1051.0126124339613, 252.3893107945988, 1038.6569435305637, 252.1731334368511]], [[1040.8815093584049, 208.7556203765876, 1033.01220728122, 217.0862758780315], [975.0973021467863, 192.77227850535488, 980.2449251743644, 204.0578238867208], [1063.368281337359, 252.77909926345762, 1051.0126124339613, 252.3893107945988]], [[1048.7508114355896, 200.5985759862548, 1040.8815093584049, 208.7556203765876], [969.9496791192082, 181.66034423510007, 975.0973021467863, 192.77227850535488], [1075.7239502407565, 253.34249884342756, 1063.368281337359, 252.77909926345762]], [[1056.6201135127744, 192.6151427070331, 1048.7508114355896, 200.5985759862548], [964.8020560916301, 170.72202107595638, 969.9496791192082, 181.66034423510007], [1088.079619144154, 254.07950953450862, 1075.7239502407565, 253.34249884342756]]]);
        for (dt, (init, traj)) in [(1.0, bitmap_1), (0.5, bitmap_0_5)] {
            let sprites = init.iter().map(|p| SpriteParticle { kind: ShardKind::Food(BLOOD), frame: 0, pos: [p[0], p[1]], vel: [p[2], p[3]] }).collect();
            let mut b = Blood { sprites, ..Blood::default() };
            for (n, want) in traj.iter().enumerate() {
                b.step_sprites(dt);
                let got: Vec<[f64; 4]> = b.sprites.iter().map(|p| [p.pos[0], p.pos[1], p.vel[0], p.vel[1]]).collect();
                assert_eq!(got, want.to_vec(), "bitmap dt {dt} step {n}");
            }
        }
        for (dt, (init, traj)) in [(1.0, lines_1), (0.5, lines_0_5)] {
            let particles = init.iter().map(|p| Particle { cur: [p[0], p[1]], last: [p[2], p[3]] }).collect();
            let mut b = Blood { particles, ..Blood::default() };
            for (n, want) in traj.iter().enumerate() {
                b.step_particles(dt);
                let got: Vec<[f64; 4]> = b.particles.iter().map(|p| [p.cur[0], p.cur[1], p.last[0], p.last[1]]).collect();
                assert_eq!(got, want.to_vec(), "lines dt {dt} step {n}");
            }
        }
    }
}
