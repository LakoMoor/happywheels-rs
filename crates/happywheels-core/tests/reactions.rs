//! Joints left and the rider's ejection after limb breaks, for every character, against the running original
//! (`analysis/golden-reactions.js` -> `tests/golden/reactions.json`).
use happywheels_core::character::{Limb, Part};
use happywheels_core::physics::World;
use std::collections::BTreeMap;

const LEVEL: &str = "<levelXML><info v='1.99' x='1000' y='250' h='f' bg='0' bgc=''/><shapes><sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>";

#[derive(serde::Deserialize)]
struct Case {
    seq: Vec<String>,
    steps: Vec<(usize, bool)>,
}

fn limb(name: &str) -> Limb {
    match name {
        "neckBreak" => Limb::Neck,
        "torsoBreak" => Limb::Waist,
        "shoulderBreak1" => Limb::Shoulder1,
        "shoulderBreak2" => Limb::Shoulder2,
        "elbowBreak1" => Limb::Elbow1,
        "elbowBreak2" => Limb::Elbow2,
        "hipBreak1" => Limb::Hip1,
        "hipBreak2" => Limb::Hip2,
        "kneeBreak1" => Limb::Knee1,
        "kneeBreak2" => Limb::Knee2,
        other => panic!("{other}"),
    }
}

#[test]
fn limb_breaks_leave_the_same_joints_as_the_original() {
    let golden: BTreeMap<usize, Vec<Case>> = serde_json::from_str(include_str!("../../../tests/golden/reactions.json")).unwrap();
    let level = happywheels_core::level::parse(LEVEL).unwrap();
    let mut wrong = Vec::new();
    for (&c, cases) in &golden {
        for case in cases {
            let mut world = World::from_level(&level);
            assert!(world.spawn_bike(c, level.info.start));
            let mut doll = world.character.take().unwrap();
            let mut vehicle = world.bike.take().unwrap();
            let mut steps = vec![(world.b2().joint_count(), vehicle.ejected())];
            for name in &case.seq {
                let l = limb(name);
                let force = doll.break_limit(&world, l) + 1.0;
                doll.break_limb(&mut world, l, force);
                vehicle.rider_hurt(&mut world, &mut doll, false, &[l]);
                steps.push((world.b2().joint_count(), vehicle.ejected()));
            }
            if steps != case.steps {
                wrong.push(format!("c{c} {:?}: original {:?}, here {steps:?}", case.seq, case.steps));
            }
        }
    }
    assert!(wrong.is_empty(), "{} differ:\n{}", wrong.len(), wrong.join("\n"));
}

#[test]
fn passengers_limb_breaks_leave_the_same_joints_as_the_original() {
    let golden: BTreeMap<String, Vec<Case>> = serde_json::from_str(include_str!("../../../tests/golden/passengers.json")).unwrap();
    let level = happywheels_core::level::parse(LEVEL).unwrap();
    let mut wrong = Vec::new();
    for (key, cases) in &golden {
        let (c, who) = key.split_once(' ').unwrap();
        for case in cases {
            let mut world = World::from_level(&level);
            assert!(world.spawn_bike(c.parse().unwrap(), level.info.start));
            let mut vehicle = world.bike.take().unwrap();
            let mut steps = vec![(world.b2().joint_count(), false)];
            for name in &case.seq {
                let off = vehicle.break_passenger_limb(&mut world, who, limb(name)).unwrap();
                steps.push((world.b2().joint_count(), off));
            }
            if steps != case.steps {
                wrong.push(format!("{key} {:?}: original {:?}, here {steps:?}", case.seq, case.steps));
            }
        }
    }
    assert!(wrong.is_empty(), "{} differ:\n{}", wrong.len(), wrong.join("\n"));
}

#[derive(serde::Deserialize)]
struct Smashed {
    seq: Vec<String>,
    steps: Vec<(usize, bool, usize)>,
}

/// Zero-filtered (loose gore) shapes on dynamic bodies.
fn loose(world: &World) -> usize {
    let c = &world.b2().c;
    c.bodies.iter().filter(|b| b.alive && b.mass > 0.0).flat_map(|b| &b.shapes).filter(|&&s| {
        let f = c.shapes[s].shape.filter;
        (f.category_bits, f.mask_bits, f.group_index) == (260, 0xFFFF, 0)
    }).count()
}

/// Chest and pelvis smashes on a rider who already lost limbs take the stumps off too.
#[test]
fn smashes_with_stumps_leave_what_the_original_leaves() {
    let golden: BTreeMap<usize, Vec<Smashed>> = serde_json::from_str(include_str!("../../../tests/golden/stumps.json")).unwrap();
    let level = happywheels_core::level::parse(LEVEL).unwrap();
    let mut wrong = Vec::new();
    for (&c, cases) in &golden {
        for case in cases {
            let mut world = World::from_level(&level);
            assert!(world.spawn_bike(c, level.info.start));
            let mut doll = world.character.take().unwrap();
            let mut vehicle = world.bike.take().unwrap();
            let mut steps = vec![(world.b2().joint_count(), vehicle.ejected(), loose(&world))];
            for name in &case.seq {
                if name.ends_with("Smash") {
                    let mut react = |w: &mut World, d: &mut happywheels_core::character::Ragdoll, limb| vehicle.rider_hurt(w, d, false, &[limb]);
                    if name == "chestSmash" {
                        doll.smash_chest_with(&mut world, &mut react);
                    } else {
                        doll.smash_pelvis_with(&mut world, &mut react);
                    }
                    vehicle.rider_hurt(&mut world, &mut doll, true, &[]);
                } else {
                    let l = limb(name);
                    let force = doll.break_limit(&world, l) + 1.0;
                    doll.break_limb(&mut world, l, force);
                    vehicle.rider_hurt(&mut world, &mut doll, false, &[l]);
                }
                steps.push((world.b2().joint_count(), vehicle.ejected(), loose(&world)));
            }
            if steps != case.steps {
                wrong.push(format!("c{c} {:?}: original {:?}, here {steps:?}", case.seq, case.steps));
            }
        }
    }
    assert!(wrong.is_empty(), "{} differ:\n{}", wrong.len(), wrong.join("\n"));
}

#[derive(serde::Deserialize)]
struct Removal {
    seq: [String; 5],
    /// `[joints, bodies, distance joints, ejected]` before and after the removal.
    steps: Vec<(usize, usize, usize, bool)>,
}

/// `removeBody` as the mower calls it on a torn-off upper limb of its rider, for every character and ligament state.
#[test]
fn removing_a_torn_off_limb_leaves_what_the_original_leaves() {
    let golden: BTreeMap<usize, Vec<Removal>> = serde_json::from_str(include_str!("../../../tests/golden/remove-body.json")).unwrap();
    let level = happywheels_core::level::parse(LEVEL).unwrap();
    let mut wrong = Vec::new();
    for (&c, cases) in &golden {
        for case in cases {
            let [root, how, hinge, before, upper] = &case.seq;
            let mut world = World::from_level(&level);
            assert!(world.spawn_bike(c, level.info.start));
            let mut doll = world.character.take().unwrap();
            let mut vehicle = world.bike.take().unwrap();
            let state = |w: &World, ejected: bool| {
                let b = &w.b2().c;
                // joint ids stay in the arena: the live ones are those the bodies still list
                let live: std::collections::BTreeSet<usize> = b.bodies.iter().filter(|b| b.alive).flat_map(|b| b.joints.iter().map(|j| j.0)).collect();
                let distance = live.iter().filter(|&&j| matches!(w.b2().joint(j).kind, box2d::joints::JointKind::Distance(_))).count();
                (w.b2().joint_count(), b.bodies.iter().filter(|b| b.alive).count(), distance, ejected)
            };
            let force = |doll: &happywheels_core::character::Ragdoll, w: &World, l: Limb, hard: bool| if hard { 1e9 } else { doll.break_limit(w, l) + 1.0 };
            for (name, hard) in [(root, how == "N"), (hinge, before == "H")].into_iter().take(if before.is_empty() { 1 } else { 2 }) {
                let l = limb(name);
                let f = force(&doll, &world, l, hard);
                doll.break_limb(&mut world, l, f);
                vehicle.rider_hurt(&mut world, &mut doll, false, &[l]);
            }
            let mut steps = vec![state(&world, vehicle.ejected())];
            let part = match upper.as_str() {
                "upperArm1Body" => Part::UpperArm1,
                "upperArm2Body" => Part::UpperArm2,
                "upperLeg1Body" => Part::UpperLeg1,
                _ => Part::UpperLeg2,
            };
            let body = doll.bodies[&part];
            if let Some(l) = doll.remove_body(&mut world, body) {
                vehicle.rider_hurt(&mut world, &mut doll, false, &[l]);
            }
            world.b2_mut().destroy_body(body);
            steps.push(state(&world, vehicle.ejected()));
            // the two worlds do not hold the same number of static bodies: compare how the counts change
            let shape = |s: &[(usize, usize, usize, bool)]| (s[0].0, s[1].0, s[1].1 as i64 - s[0].1 as i64, s[0].2, s[1].2, s[0].3, s[1].3);
            if shape(&steps) != shape(&case.steps) {
                wrong.push(format!("c{c} {:?}: original {:?}, here {steps:?}", case.seq, case.steps));
            }
        }
    }
    assert!(wrong.is_empty(), "{} differ:\n{}", wrong.len(), wrong.join("\n"));
}
