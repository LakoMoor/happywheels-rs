//! Joints left and the rider's ejection after limb breaks, for every character, against the running original
//! (`analysis/golden-reactions.js` -> `tests/golden/reactions.json`).
use happywheels_core::character::Limb;
use happywheels_core::physics::World;
use std::collections::BTreeMap;

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
    let level = happywheels_core::level::parse("<levelXML><info v='1.99' x='1000' y='250' h='f' bg='0' bgc=''/><shapes><sh t='0' p0='1000' p1='425' p2='2000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes></levelXML>").unwrap();
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
