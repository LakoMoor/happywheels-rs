//! Launcher: pick a character, a level and the options, then start the `happywheels` binary next to this one.

mod settings;

use gpui::{div, prelude::*, px, rgb, size, App, Application, Bounds, Context, Div, PathPromptOptions, SharedString, Stateful, Window, WindowBounds, WindowOptions};
use settings::{Settings, BLOOD, CHARACTERS, PARTICLES};

const BG: u32 = 0x14161c;
const PANEL: u32 = 0x1d2029;
const BUTTON: u32 = 0x2a2e3a;
const HOVER: u32 = 0x363b4a;
const ACCENT: u32 = 0xd8432f;
const TEXT: u32 = 0xeceef2;
const MUTED: u32 = 0x8b91a1;

struct Launcher {
    settings: Settings,
    status: SharedString,
}

impl Launcher {
    /// A selectable button that applies `change` to the settings.
    fn choice(&self, id: impl Into<gpui::ElementId>, label: impl Into<SharedString>, selected: bool, cx: &mut Context<Self>, change: impl Fn(&mut Settings) + 'static) -> Stateful<Div> {
        button(id, label, selected).on_click(cx.listener(move |this, _, _, cx| {
            change(&mut this.settings);
            cx.notify();
        }))
    }

    fn pick_level(&mut self, cx: &mut Context<Self>) {
        let paths = cx.prompt_for_paths(PathPromptOptions { files: true, directories: false, multiple: false, prompt: Some("Открыть уровень".into()) });
        cx.spawn(async move |this, cx| {
            let picked = match paths.await {
                Ok(Ok(Some(mut paths))) => paths.pop(),
                _ => None,
            };
            if let Some(path) = picked {
                let _ = this.update(cx, |this, cx| {
                    this.settings.level = Some(path);
                    cx.notify();
                });
            }
        })
        .detach();
    }

    fn play(&mut self, cx: &mut Context<Self>) {
        match self.settings.command().and_then(|mut command| command.spawn().map_err(|e| format!("не удалось запустить игру: {e}"))) {
            Ok(mut child) => {
                self.status = "Игра запущена".into();
                let wait = cx.background_executor().spawn(async move { child.wait() });
                cx.spawn(async move |this, cx| {
                    let status = match wait.await {
                        Ok(s) if s.success() => "Игра закрыта".to_string(),
                        Ok(s) => format!("Игра завершилась с ошибкой: {s}"),
                        Err(e) => format!("Ошибка ожидания игры: {e}"),
                    };
                    let _ = this.update(cx, |this, cx| {
                        this.status = status.into();
                        cx.notify();
                    });
                })
                .detach();
            }
            Err(e) => self.status = e.into(),
        }
        cx.notify();
    }
}

fn button(id: impl Into<gpui::ElementId>, label: impl Into<SharedString>, selected: bool) -> Stateful<Div> {
    div()
        .id(id)
        .px_3()
        .py_1()
        .rounded_md()
        .cursor_pointer()
        .bg(rgb(if selected { ACCENT } else { BUTTON }))
        .hover(move |s| s.bg(rgb(if selected { ACCENT } else { HOVER })))
        .child(label.into())
}

fn section(title: &'static str) -> Div {
    div().flex().flex_col().gap_2().child(div().text_sm().text_color(rgb(MUTED)).child(title))
}

fn row() -> Div {
    div().flex().flex_row().flex_wrap().gap_2()
}

impl Render for Launcher {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let s = self.settings.clone();
        let mut characters = section("ПЕРСОНАЖ").w(px(230.0)).p_4().rounded_lg().bg(rgb(PANEL));
        for (i, name) in CHARACTERS.iter().enumerate() {
            let n = i + 1;
            characters = characters.child(self.choice(("character", n), *name, s.character == n, cx, move |s| s.character = n));
        }

        let level_name = s.level.as_ref().and_then(|p| p.file_name()).map_or_else(|| "не выбран".to_string(), |n| n.to_string_lossy().into_owned());
        let level = section("УРОВЕНЬ")
            .child(
                row()
                    .child(self.choice("menu", "Меню игры", s.level.is_none(), cx, |s| s.level = None))
                    .child(button("open", "Открыть XML…", s.level.is_some()).on_click(cx.listener(|this, _, _, cx| this.pick_level(cx)))),
            )
            .child(div().text_sm().text_color(rgb(MUTED)).child(format!("Файл: {level_name}")));

        let players = section("ИГРОКИ").child(
            row()
                .child(self.choice("p1", "Один", s.players == 1, cx, |s| s.players = 1))
                .child(self.choice("p2", "Двое (split screen)", s.players == 2, cx, |s| s.players = 2)),
        );
        let rate = section("ЧАСТОТА").child(
            row()
                .child(self.choice("fps30", "30 к/с", !s.fps60, cx, |s| s.fps60 = false))
                .child(self.choice("fps60", "60 к/с", s.fps60, cx, |s| s.fps60 = true)),
        );
        let mut blood = row();
        for (i, name) in BLOOD.iter().enumerate() {
            blood = blood.child(self.choice(("blood", i), *name, s.blood == i, cx, move |s| s.blood = i));
        }
        let mut particles = row();
        for n in PARTICLES {
            particles = particles.child(self.choice(("particles", n), n.to_string(), s.particles == n, cx, move |s| s.particles = n));
        }

        let options = div()
            .flex()
            .flex_col()
            .flex_1()
            .gap_5()
            .p_4()
            .rounded_lg()
            .bg(rgb(PANEL))
            .child(level)
            .child(players)
            .child(rate)
            .child(section("КРОВЬ").child(blood))
            .child(section("ЧАСТИЦЫ").child(particles))
            .child(div().flex_1())
            .child(div().text_sm().text_color(rgb(MUTED)).child(self.status.clone()))
            .child(button("play", "ИГРАТЬ", true).py_3().text_xl().flex().justify_center().on_click(cx.listener(|this, _, _, cx| this.play(cx))));

        div()
            .flex()
            .flex_col()
            .size_full()
            .gap_4()
            .p_5()
            .bg(rgb(BG))
            .text_color(rgb(TEXT))
            .child(div().text_2xl().child("Happy Wheels · Rust"))
            .child(div().flex().flex_row().flex_1().gap_4().child(characters).child(options))
    }
}

fn main() {
    Application::new().run(|cx: &mut App| {
        cx.on_window_closed(|cx| {
            if cx.windows().is_empty() {
                cx.quit();
            }
        })
        .detach();
        let bounds = Bounds::centered(None, size(px(760.0), px(600.0)), cx);
        let options = WindowOptions { window_bounds: Some(WindowBounds::Windowed(bounds)), ..Default::default() };
        cx.open_window(options, |_, cx| cx.new(|_| Launcher { settings: Settings::default(), status: "".into() })).expect("launcher window");
        cx.activate(true);
    });
}
