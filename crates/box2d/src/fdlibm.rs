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
