//! `Math.sin` / `Math.cos` as the oracle browser (Chrome/V8 with the glibc-derived trig, correctly rounded in practice) computes them.
//!
//! Neither `libm` (musl) nor fdlibm 5.3 round like it: they differ in the last bit for 1–3 % of the arguments, and one ulp in a body's
//! rotation matrix makes a chaotic scene diverge (docs/PORT.md 2026-11-25). Chrome's results equal the correctly rounded value in every
//! sample checked (6000 cos values against a 60-digit reference; 20 000 sin/cos pairs in `tests/trigcheck.rs`), so the value is computed
//! in double-double arithmetic (error ~1e-30) and rounded once.

type Dd = (f64, f64);

fn two_sum(a: f64, b: f64) -> Dd {
    let s = a + b;
    let bb = s - a;
    (s, (a - (s - bb)) + (b - bb))
}
fn quick(a: f64, b: f64) -> Dd {
    let s = a + b;
    (s, b - (s - a))
}
fn two_prod(a: f64, b: f64) -> Dd {
    let p = a * b;
    (p, a.mul_add(b, -p))
}
fn add(a: Dd, b: Dd) -> Dd {
    let (s, e) = two_sum(a.0, b.0);
    let (t, f) = two_sum(a.1, b.1);
    let (s, e) = quick(s, e + t);
    quick(s, e + f)
}
fn mul(a: Dd, b: Dd) -> Dd {
    let (p, e) = two_prod(a.0, b.0);
    quick(p, e + (a.0 * b.1 + a.1 * b.0))
}

const PIO2: [f64; 3] = [std::f64::consts::FRAC_PI_2, 6.123_233_995_736_766e-17, -1.497_384_904_859_169_8e-33];

/// (quadrant, reduced argument in [-π/4, π/4]) for |x| below ~1e6; None beyond.
fn reduce(x: f64) -> Option<(i64, Dd)> {
    if !(x.abs() < 1.0e6) {
        return None;
    }
    let n = (x * (2.0 / std::f64::consts::PI)).round();
    let mut r: Dd = (x, 0.0);
    for p in PIO2 {
        let (ph, pl) = two_prod(n, p);
        r = add(r, (-ph, -pl));
    }
    Some((n as i64, r))
}

fn sin_dd(r: Dd) -> Dd {
    let r2 = mul(r, r);
    let mut sum = r;
    let mut term = r;
    let mut k = 1.0f64;
    for _ in 0..14 {
        term = div_small(mul(term, r2), -((k + 1.0) * (k + 2.0)));
        sum = add(sum, term);
        k += 2.0;
    }
    sum
}

fn cos_dd(r: Dd) -> Dd {
    let r2 = mul(r, r);
    let mut term: Dd = (1.0, 0.0);
    let mut sum = term;
    let mut k = 0.0f64;
    for _ in 0..14 {
        term = div_small(mul(term, r2), -((k + 1.0) * (k + 2.0)));
        sum = add(sum, term);
        k += 2.0;
    }
    sum
}

/// Double-double divided by a small exactly representable integer.
fn div_small(a: Dd, d: f64) -> Dd {
    let q1 = a.0 / d;
    let (p, e) = two_prod(q1, d);
    let rem = add(a, (-p, -e));
    let q2 = rem.0 / d;
    quick(q1, q2)
}

pub fn sin(x: f64) -> f64 {
    if x.abs() < 1.0e-8 {
        return x - x * x * x / 6.0;
    }
    let Some((n, r)) = reduce(x) else { return libm::sin(x) };
    let v = match n.rem_euclid(4) {
        0 => sin_dd(r),
        1 => cos_dd(r),
        2 => { let s = sin_dd(r); (-s.0, -s.1) }
        _ => { let c = cos_dd(r); (-c.0, -c.1) }
    };
    v.0 + v.1
}

pub fn cos(x: f64) -> f64 {
    let Some((n, r)) = reduce(x) else { return libm::cos(x) };
    let v = match n.rem_euclid(4) {
        0 => cos_dd(r),
        1 => { let s = sin_dd(r); (-s.0, -s.1) }
        2 => { let c = cos_dd(r); (-c.0, -c.1) }
        _ => sin_dd(r),
    };
    v.0 + v.1
}

/// `Math.acos` as V8 computes it (fdlibm `e_acos.c`).
pub fn acos(x: f64) -> f64 {
    const PI: f64 = std::f64::consts::PI;
    const PIO2_HI: f64 = std::f64::consts::FRAC_PI_2;
    const PIO2_LO: f64 = 6.123_233_995_736_766e-17;
    const P: [f64; 6] = [1.666_666_666_666_666_6e-1, -3.255_658_186_224_009e-1, 2.012_125_321_348_629_3e-1, -4.005_553_450_067_941e-2, 7.915_349_942_898_145e-4, 3.479_331_075_960_212e-5];
    const Q: [f64; 4] = [-2.403_394_911_734_414, 2.020_945_760_233_505_7, -6.882_839_716_054_533e-1, 7.703_815_055_590_194e-2];
    let ratio = |z: f64| {
        let p = z * (P[0] + z * (P[1] + z * (P[2] + z * (P[3] + z * (P[4] + z * P[5])))));
        let q = 1.0 + z * (Q[0] + z * (Q[1] + z * (Q[2] + z * Q[3])));
        p / q
    };
    let high = (x.to_bits() >> 32) as u32;
    let magnitude = high & 0x7fff_ffff;
    let negative = high >> 31 != 0;
    if magnitude >= 0x3ff0_0000 {
        return if x == 1.0 {
            0.0
        } else if x == -1.0 {
            PI + 2.0 * PIO2_LO
        } else {
            f64::NAN
        };
    }
    if magnitude < 0x3fe0_0000 {
        if magnitude <= 0x3c60_0000 {
            return PIO2_HI + PIO2_LO;
        }
        let r = ratio(x * x);
        return PIO2_HI - (x - (PIO2_LO - x * r));
    }
    if negative {
        let z = (1.0 + x) * 0.5;
        let s = z.sqrt();
        let w = ratio(z) * s - PIO2_LO;
        return PI - 2.0 * (s + w);
    }
    let z = (1.0 - x) * 0.5;
    let s = z.sqrt();
    let df = f64::from_bits(s.to_bits() & 0xffff_ffff_0000_0000);
    let c = (z - df * df) / (s + df);
    let w = ratio(z) * s + c;
    2.0 * (df + w)
}
