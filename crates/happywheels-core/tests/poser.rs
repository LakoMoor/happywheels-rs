//! The editor's NPC poser against drags run in the original (`analysis/poser.js`).

use happywheels_core::editor::Document;
use happywheels_core::editor_session::{Editor, Preview, View};
use happywheels_core::poser::{PROPERTIES, Poser};
use happywheels_core::sprites::Sprites;

#[derive(serde::Deserialize)]
struct Run {
    r#type: u32,
    start: [f64; 9],
    steps: Vec<Step>,
}

#[derive(serde::Deserialize)]
struct Step {
    which: usize,
    mouse: [f64; 2],
    angles: [f64; 9],
    at: Vec<[f64; 2]>,
    shown: [f64; 9],
}

#[test]
fn dragging_limbs_gives_the_originals_angles() {
    let runs: Vec<Run> = serde_json::from_str(include_str!("../../../tests/golden/poser.json")).unwrap();
    assert_eq!(runs.len(), 64);
    assert_eq!(PROPERTIES.len(), 9);
    let mut worst = 0.0f64;
    for (n, run) in runs.iter().enumerate() {
        let mut poser = Poser::new(run.r#type, run.start).unwrap();
        for (k, step) in run.steps.iter().enumerate() {
            poser.drag(step.which, step.mouse);
            for i in 0..9 {
                let (at, degrees) = poser.placement(i);
                let off = (poser.angle(i) - step.angles[i]).abs().max((at[0] - step.at[i][0]).abs()).max((at[1] - step.at[i][1]).abs());
                worst = worst.max(off);
                assert!(off < 1e-9, "run {n} type {} step {k} segment {i}: {} here, {} in the original", run.r#type, poser.angle(i), step.angles[i]);
                // the original reads the rotation back from radians
                assert!((degrees - step.shown[i]).abs() < 1e-9, "run {n} step {k} segment {i}: rotation {degrees}, {}", step.shown[i]);
            }
        }
    }
    println!("worst {worst:e}");
}

/// An NPC (mirrored and turned or not) in the editor: double-click, drag the near arm, undo; the window is saved for
/// comparison with the original's frame.
#[test]
fn posing_an_npc_in_the_editor() {
    let xml = "<levelXML><info v='1.99' x='200' y='200' c='1' f='t' h='f' bg='0' bgc='16777215'/><specials><sp t='17' p0='1000' p1='700' p2='0' p3='2' p4='f' p5='f' p6='f' p7='t' p8='0' p9='0' p10='0' p11='0' p12='0' p13='0' p14='0' p15='0' p16='0' p17='f'/></specials></levelXML>";
    let mut editor = Editor::new(Document::parse(xml).unwrap());
    let angle = |e: &Editor, name: &str| e.doc.specials[0].get(name).unwrap().number();
    assert_eq!(editor.doc.specials[0].spec().name, "NPCharacterRef");
    editor.double_click([1000.0, 700.0]);
    assert!(editor.is_posing() && editor.selection.is_empty());

    // grab the near upper arm by its middle (holder coordinates, half size on the canvas) and pull it up and forward
    let poser = &editor.posing.as_ref().unwrap().poser;
    let ([x, y], _) = poser.placement(1);
    let grab = [1000.0 + x * 0.5, 700.0 + (y + 15.0) * 0.5];
    assert_eq!(poser.pick([x, y + 15.0]), Some(1));
    editor.pointer_down(grab, false);
    editor.pointer_move([1060.0, 650.0]);
    editor.pointer_up([1060.0, 650.0]);
    assert!(angle(&editor, "shoulder1Angle") < -45.0, "{}", angle(&editor, "shoulder1Angle"));
    assert_eq!(angle(&editor, "shoulder2Angle"), 0.0);

    let assets = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../assets-99tnk64n1mubaoc8w");
    if assets.join("animate/editor").is_dir() {
        let sprites = Sprites::open(&assets, 2).unwrap();
        let view = View { width: 600, height: 500, zoom: 2.0, camera: [1000.0, 700.0] };
        let posed = editor.render(view, &mut Preview::default(), Some(&sprites));
        posed.save(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../screenshots/editor_poser.png")).unwrap();
    }

    editor.double_click([1000.0, 700.0]);
    assert!(!editor.is_posing());
    let posed = angle(&editor, "shoulder1Angle");
    editor.undo();
    assert_eq!(angle(&editor, "shoulder1Angle"), 0.0);
    editor.redo();
    assert_eq!(angle(&editor, "shoulder1Angle"), posed);
}
