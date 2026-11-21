use anyhow::{bail, Context, Result};

#[derive(Debug, Clone, PartialEq)]
pub struct ShapePath {
    pub color: String,
    pub opacity: f32,
    /// Optional outline: `s <width> <color> <opacity>` after the fill header.
    pub stroke: Option<Stroke>,
    pub commands: Vec<DrawCommand>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Stroke {
    pub width: f32,
    pub color: String,
    pub opacity: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DrawCommand {
    MoveTo { x: f32, y: f32 },
    LineTo { x: f32, y: f32 },
    QuadraticTo { cx: f32, cy: f32, x: f32, y: f32 },
    Close,
}

pub fn parse(input: &str) -> Result<Vec<ShapePath>> {
    input
        .lines()
        .enumerate()
        .filter(|(_, line)| !line.trim().is_empty())
        .map(|(index, line)| parse_line(line).with_context(|| format!("shape line {}", index + 1)))
        .collect::<Result<Vec<_>>>()
        .map(|lines| lines.into_iter().flatten().collect())
}

fn parse_line(line: &str) -> Result<Vec<ShapePath>> {
    // A path line holds one or more `f ...` headers; tokens like `M0.7` glue
    // the command letter to its first number, so split those up front.
    let mut tokens: Vec<String> = Vec::new();
    let mut expect_style = 0; // number of upcoming tokens that are style values
    for raw in line.split_whitespace() {
        if expect_style > 0 {
            expect_style -= 1;
            tokens.push(raw.to_owned());
            continue;
        }
        let first = raw.chars().next().unwrap();
        if first.is_ascii_alphabetic() && raw.len() > 1 && !raw.chars().nth(1).unwrap().is_ascii_alphabetic() {
            tokens.push(first.to_string());
            tokens.push(raw[1..].to_owned());
        } else {
            tokens.push(raw.to_owned());
        }
        match first {
            'f' if raw.len() == 1 => expect_style = 2,
            's' if raw.len() == 1 => expect_style = 3,
            _ => {}
        }
    }
    let mut it = tokens.iter().map(String::as_str).peekable();
    let mut paths = Vec::new();
    let mut current: Option<ShapePath> = None;
    while let Some(command) = it.next() {
        if command == "f" {
            paths.extend(current.take());
            current = Some(ShapePath {
                color: it.next().context("missing fill color")?.to_owned(),
                opacity: number(it.next())?,
                stroke: None,
                commands: Vec::new(),
            });
            continue;
        }
        let path = current.as_mut().context("expected fill command 'f'")?;
        let commands = &mut path.commands;
        match command {
            "m" | "M" if !is_number(it.peek()) => {} // degenerate empty move (`m c`) in the data
            "m" | "M" => {
                // Extra coordinate pairs after a move are implicit line-tos.
                commands.push(DrawCommand::MoveTo { x: number(it.next())?, y: number(it.next())? });
                while is_number(it.peek()) {
                    commands.push(DrawCommand::LineTo { x: number(it.next())?, y: number(it.next())? });
                }
            }
            "l" | "L" => loop {
                commands.push(DrawCommand::LineTo { x: number(it.next())?, y: number(it.next())? });
                if !is_number(it.peek()) {
                    break;
                }
            },
            "q" | "Q" => loop {
                commands.push(DrawCommand::QuadraticTo {
                    cx: number(it.next())?,
                    cy: number(it.next())?,
                    x: number(it.next())?,
                    y: number(it.next())?,
                });
                if !is_number(it.peek()) {
                    break;
                }
            },
            "s" => {
                path.stroke = Some(Stroke {
                    width: number(it.next())?,
                    color: it.next().context("missing stroke color")?.to_owned(),
                    opacity: number(it.next())?,
                })
            }
            // `h` only ever appears immediately before `c` in the original data.
            "h" => {}
            "c" | "C" => commands.push(DrawCommand::Close),
            other => bail!("unsupported drawing command {other:?}"),
        }
    }
    paths.extend(current);
    Ok(paths)
}

fn is_number(token: Option<&&str>) -> bool {
    token.is_some_and(|t| t.parse::<f32>().is_ok())
}

fn number(token: Option<&str>) -> Result<f32> {
    let token = token.context("missing numeric argument")?;
    let value: f32 = token
        .parse()
        .with_context(|| format!("invalid number {token:?}"))?;
    if !value.is_finite() {
        bail!("non-finite number {token:?}");
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_original_character_outline() {
        let paths = parse("f #f90 1 m -2.3 -15.5 l -3 -15.5 q -1.15 15.5 0 15.45 c")
            .expect("original shape syntax");
        assert_eq!(paths.len(), 1);
        assert_eq!(paths[0].color, "#f90");
        assert_eq!(
            paths[0].commands,
            vec![
                DrawCommand::MoveTo { x: -2.3, y: -15.5 },
                DrawCommand::LineTo { x: -3.0, y: -15.5 },
                DrawCommand::QuadraticTo {
                    cx: -1.15,
                    cy: 15.5,
                    x: 0.0,
                    y: 15.45
                },
                DrawCommand::Close,
            ]
        );
    }

    #[test]
    fn parses_implicit_pairs_stroke_and_multiple_paths() {
        let paths = parse("f 0 0 s 1 #ccc 1 m 0 0 l 1 1 2 2 h c f #fff 1 M3 4 l 5 6 c f #0f0 1 m c").unwrap();
        assert_eq!(paths.len(), 3);
        assert_eq!(paths[0].stroke.as_ref().unwrap().width, 1.0);
        assert_eq!(paths[0].commands.len(), 4);
        assert_eq!(paths[1].commands[0], DrawCommand::MoveTo { x: 3.0, y: 4.0 });
    }

    #[test]
    fn rejects_truncated_curve() {
        assert!(parse("f #fff 1 q 1 2 3").is_err());
    }
}

