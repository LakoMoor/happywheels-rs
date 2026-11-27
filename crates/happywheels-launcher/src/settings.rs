//! What the launcher chooses, and the `happywheels` command line it stands for.

use std::path::PathBuf;
use std::process::Command;

/// Character 1..=11 in the original's order.
pub const CHARACTERS: [&str; 11] = [
    "Wheelchair Guy",
    "Segway Guy",
    "Irresponsible Dad",
    "Effective Shopper",
    "Moped Couple",
    "Lawnmower Man",
    "Explorer Guy",
    "Santa Claus",
    "Pogostick Man",
    "Irresponsible Mom",
    "Helicopter Man",
];
/// `--blood` 0..=4.
pub const BLOOD: [&str; 5] = ["Off", "Bitmap", "Lines", "Metaball", "Metaball+"];
/// `--particles` presets.
pub const PARTICLES: [usize; 4] = [0, 500, 1000, 2000];

#[derive(Clone, Debug)]
pub struct Settings {
    pub character: usize,
    pub level: Option<PathBuf>,
    pub players: usize,
    pub fps60: bool,
    pub blood: usize,
    pub particles: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Self { character: 1, level: None, players: 1, fps60: false, blood: 1, particles: 1000 }
    }
}

impl Settings {
    pub fn args(&self) -> Vec<String> {
        let mut args: Vec<String> = [
            ("--character", self.character.to_string()),
            ("--players", self.players.to_string()),
            ("--blood", self.blood.to_string()),
            ("--particles", self.particles.to_string()),
        ]
        .into_iter()
        .flat_map(|(name, value)| [name.to_string(), value])
        .collect();
        if self.fps60 {
            args.push("--fps60".into());
        }
        if let Some(level) = &self.level {
            args.extend(["--level".into(), level.to_string_lossy().into_owned(), "--window".into()]);
        }
        args
    }

    /// The game binary built into the same directory as the launcher.
    pub fn command(&self) -> Result<Command, String> {
        let me = std::env::current_exe().map_err(|e| e.to_string())?;
        let game = me.with_file_name(format!("happywheels{}", std::env::consts::EXE_SUFFIX));
        if !game.is_file() {
            return Err("нет бинарника игры: соберите `cargo build --release -p happywheels`".into());
        }
        let mut command = Command::new(game);
        command.args(self.args());
        Ok(command)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arguments_follow_the_choices() {
        let mut s = Settings::default();
        assert_eq!(s.args().join(" "), "--character 1 --players 1 --blood 1 --particles 1000");
        s = Settings { character: 11, level: Some("a b.xml".into()), players: 2, fps60: true, blood: 4, particles: 0 };
        assert_eq!(s.args(), ["--character", "11", "--players", "2", "--blood", "4", "--particles", "0", "--fps60", "--level", "a b.xml", "--window"]);
    }
}
