//! The editor document against the original editor: each fixture is a level the original loaded and saved again
//! (`analysis/editor-golden.js`).

use happywheels_core::editor::Document;

#[derive(serde::Deserialize)]
struct Case {
    input: String,
    output: String,
}

fn cases() -> Vec<Case> {
    serde_json::from_str(include_str!("../../../tests/golden/editor.json")).unwrap()
}

fn check(name: &str, input: impl Fn(&Case) -> &str) {
    let mut failed = Vec::new();
    let all = cases();
    for (k, case) in all.iter().enumerate() {
        let saved = match Document::parse(input(case)) {
            Ok(d) => d.to_xml(),
            Err(e) => format!("error: {e:#}"),
        };
        if saved != case.output {
            let line = saved.lines().zip(case.output.lines()).position(|(a, b)| a != b).unwrap_or(0);
            failed.push(format!("case {k} line {line}:\n  port {}\n  orig {}", saved.lines().nth(line).unwrap_or(""), case.output.lines().nth(line).unwrap_or("")));
        }
    }
    assert!(failed.is_empty(), "{name}: {} of {} differ\n{}", failed.len(), all.len(), failed[..failed.len().min(12)].join("\n"));
}

#[test]
fn levels_are_saved_like_the_original_editor() {
    check("input", |c| &c.input);
}
