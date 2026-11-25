//! `box2d::fdlibm::{sin,cos}` against what the oracle browser's `Math.sin/cos` returned (angle, cos, sin as f64 bit patterns; angles in ±10,
//! recorded with `analysis/evaluate.js` in the real game page).
#[test]
fn trig_matches_oracle_browser() {
    let t = include_str!("trig-chrome.txt");
    let mut n = 0;
    for l in t.lines() {
        let v: Vec<u64> = l.split(' ').map(|h| u64::from_str_radix(h, 16).unwrap()).collect();
        let a = f64::from_bits(v[0]);
        assert_eq!(box2d::fdlibm::cos(a).to_bits(), v[1], "cos({a})");
        assert_eq!(box2d::fdlibm::sin(a).to_bits(), v[2], "sin({a})");
        n += 1;
    }
    assert!(n >= 8000);
}
