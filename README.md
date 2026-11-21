# Happy Wheels in Rust

Port of Happy Wheels (Flash/JS) to Rust: Box2D 2.0.1 port, level loader, characters, vehicles, sound, sprite rendering. Checked against the original game via golden tests.

Site: https://lakomoor.github.io/happywheels-rs/

```
cargo test
cargo run -p happywheels -- --help
```

Original game assets are **not** included; point the app at your own copy of the assets. See [docs/PORT.md](docs/PORT.md) for status.

![bike](docs/screenshots/sprites_bike.png)
![backdrop](docs/screenshots/backdrop_green_scrolled.png)

## Multiplayer

`happywheels --level level.xml --blood 0|1|2` picks the blood look (0 none, 1 bitmap drops (default), 2 lines).

`happywheels --level level.xml --window --players 2` — local split screen (P1: arrows/Space, P2: WASD/F). Online race: `--host 7777` on one machine, `--join HOST:7777` on the other (same level file).
