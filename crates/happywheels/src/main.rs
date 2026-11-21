mod audio;

use std::{path::PathBuf, sync::Arc};

use anyhow::{bail, Context, Result};
use happywheels_core::{AssetRoot, Menu, MenuAction};
use image::RgbaImage;
use winit::{
    application::ApplicationHandler,
    dpi::LogicalSize,
    event::{ElementState, MouseButton, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    window::{Window, WindowId},
};

struct Renderer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    pipeline: wgpu::RenderPipeline,
    texture_layout: wgpu::BindGroupLayout,
    bind_group: wgpu::BindGroup,
    _texture: wgpu::Texture,
    texture_size: (u32, u32),
}

impl Renderer {
    async fn new(window: Arc<Window>, frame: &RgbaImage) -> Result<Self> {
        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let surface = instance.create_surface(window)?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                compatible_surface: Some(&surface),
                ..Default::default()
            })
            .await?;
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor::default())
            .await?;
        let caps = surface.get_capabilities(&adapter);
        let format = caps
            .formats
            .iter()
            .copied()
            .find(|format| format.is_srgb())
            .or_else(|| caps.formats.first().copied())
            .context("surface has no supported formats")?;
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            width: size.width.max(1),
            height: size.height.max(1),
            present_mode: wgpu::PresentMode::Fifo,
            alpha_mode: caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
            color_space: Default::default(),
        };
        surface.configure(&device, &config);

        let shader = device.create_shader_module(wgpu::include_wgsl!("shader.wgsl"));
        let texture_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("menu texture layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("menu pipeline layout"),
            bind_group_layouts: &[Some(&texture_layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("menu pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(config.format.into())],
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });
        let (_texture, bind_group, texture_size) =
            Self::create_texture(&device, &texture_layout, frame);
        let mut renderer = Self {
            surface,
            device,
            queue,
            config,
            pipeline,
            texture_layout,
            bind_group,
            _texture,
            texture_size,
        };
        renderer.upload(frame)?;
        Ok(renderer)
    }

    fn create_texture(
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
        frame: &RgbaImage,
    ) -> (wgpu::Texture, wgpu::BindGroup, (u32, u32)) {
        let size = wgpu::Extent3d {
            width: frame.width(),
            height: frame.height(),
            depth_or_array_layers: 1,
        };
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("menu frame"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let view = texture.create_view(&Default::default());
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("menu sampler"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("menu bind group"),
            layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
            ],
        });
        (texture, bind_group, (frame.width(), frame.height()))
    }

    fn upload(&mut self, frame: &RgbaImage) -> Result<()> {
        if self.texture_size != frame.dimensions() {
            let (texture, bind_group, size) =
                Self::create_texture(&self.device, &self.texture_layout, frame);
            self._texture = texture;
            self.bind_group = bind_group;
            self.texture_size = size;
        }
        self.queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &self._texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            frame,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(frame.width() * 4),
                rows_per_image: Some(frame.height()),
            },
            wgpu::Extent3d {
                width: frame.width(),
                height: frame.height(),
                depth_or_array_layers: 1,
            },
        );
        Ok(())
    }

    fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.device, &self.config);
    }

    fn render(&mut self) {
        use wgpu::CurrentSurfaceTexture as SurfaceTexture;
        let frame = match self.surface.get_current_texture() {
            SurfaceTexture::Success(frame) | SurfaceTexture::Suboptimal(frame) => frame,
            SurfaceTexture::Outdated | SurfaceTexture::Lost => {
                self.surface.configure(&self.device, &self.config);
                return;
            }
            _ => return,
        };
        let view = frame.texture.create_view(&Default::default());
        let mut encoder = self.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("menu render pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.draw(0..3, 0..1);
        }
        self.queue.submit([encoder.finish()]);
        self.queue.present(frame);
    }
}

struct Sim {
    level: happywheels_core::level::Level,
    world: happywheels_core::physics::World,
    next_step: std::time::Instant,
    drive: f32,
    /// Keys held: left, right, up, down, space, z, shift, ctrl.
    keys: [bool; 8],
    sprites: Option<happywheels_core::sprites::Sprites>,
    /// Parallax backdrop (`bg='1'` green hills only).
    backdrop: Option<happywheels_core::sprites::Sprites>,
    /// Local second player (split screen): its own copy of the level and world, so players race side by side without touching each other.
    p2: Option<Player>,
}

/// A local player beyond the first: own world, sprites and keys (W A S D, F = space, G = Z).
struct Player {
    world: happywheels_core::physics::World,
    sprites: Option<happywheels_core::sprites::Sprites>,
    keys: [bool; 8],
    drive: f32,
}

impl Player {
    fn new(assets: &std::path::Path, level: &happywheels_core::level::Level, character: usize) -> Self {
        let mut world = happywheels_core::physics::World::from_level(level);
        if !world.spawn_bike(character, level.info.start) {
            world.spawn_character(character, level.info.start);
        }
        let sprites = make_sprites(assets, character, &world, level.info.start);
        Self { world, sprites, keys: [false; 8], drive: 0.0 }
    }

    fn camera(&self, level: &happywheels_core::level::Level) -> [f32; 2] {
        use happywheels_core::character::Part;
        self.world.character.as_ref().and_then(|d| d.bodies.get(&Part::Chest)).and_then(|&b| self.world.body_position_px(b)).unwrap_or(level.info.start)
    }

    fn step(&mut self) {
        let k = self.keys;
        self.world.drive(self.drive);
        self.world.set_rider_keys(happywheels_core::vehicle::Keys { left: k[0], right: k[1], up: k[2], down: k[3], space: k[4], z: k[5], shift: k[6], ctrl: k[7] });
        self.world.step();
        self.world.take_sounds();
    }
}

impl Sim {
    /// Camera centre (px): the rider's chest, or the level start when there is no character.
    fn camera(&self) -> [f32; 2] {
        use happywheels_core::character::Part;
        self.world
            .character
            .as_ref()
            .and_then(|d| d.bodies.get(&Part::Chest))
            .and_then(|&b| self.world.body_position_px(b))
            .unwrap_or(self.level.info.start)
    }
}

/// Sprites for the c3 bike + rider (other characters still use the polygon fallback).
fn make_backdrop(assets: &std::path::Path, level: &happywheels_core::level::Level) -> Option<happywheels_core::sprites::Sprites> {
    match level.info.background {
        1 => happywheels_core::sprites::Sprites::open_backdrop(assets),
        2 => happywheels_core::sprites::Sprites::open_city_backdrop(assets),
        _ => None,
    }
}

fn make_sprites(assets: &std::path::Path, character: usize, world: &happywheels_core::physics::World, start: [f32; 2]) -> Option<happywheels_core::sprites::Sprites> {
    let mut s = happywheels_core::sprites::Sprites::open(assets, character as u32)?;
    s.bind_vehicle_rider(world, start);
    Some(s)
}

/// Split screen: the left half shows player 1, the right half player 2 (each at `w/2` wide, own camera).
fn render_split(level: &happywheels_core::level::Level, world: &happywheels_core::physics::World, sprites: Option<&happywheels_core::sprites::Sprites>, p2: &Player, backdrop: Option<&happywheels_core::sprites::Sprites>, w: u32, h: u32) -> RgbaImage {
    use happywheels_core::character::Part;
    let (half, h) = ((w / 2).max(1), h.max(1));
    let cam = world.character.as_ref().and_then(|d| d.bodies.get(&Part::Chest)).and_then(|&b| world.body_position_px(b)).unwrap_or(level.info.start);
    let left = happywheels_core::render::render_level_scene(level, world, half, h, cam, sprites, backdrop);
    let right = happywheels_core::render::render_level_scene(level, &p2.world, half, h, p2.camera(level), p2.sprites.as_ref(), backdrop);
    let mut out = RgbaImage::new(half * 2, h);
    image::imageops::replace(&mut out, &left, 0, 0);
    image::imageops::replace(&mut out, &right, i64::from(half), 0);
    for y in 0..h {
        out.put_pixel(half - 1, y, image::Rgba([0, 0, 0, 255]));
    }
    out
}

/// Online play: two peers each simulate both riders (separate worlds, deterministic Box2D) and exchange the key state of every step over TCP (lockstep).
struct Net {
    stream: std::net::TcpStream,
    /// The host's local keys drive player 1, the client's player 2.
    host: bool,
}

fn pack_keys(k: [bool; 8]) -> u8 {
    k.iter().enumerate().fold(0, |b, (i, &on)| b | (u8::from(on) << i))
}

fn unpack_keys(b: u8) -> [bool; 8] {
    std::array::from_fn(|i| b & (1 << i) != 0)
}

impl Net {
    /// Sends this step's local keys and waits for the peer's; `None` when the connection is gone.
    fn exchange(&mut self, local: [bool; 8]) -> Option<[bool; 8]> {
        use std::io::{Read, Write};
        self.stream.write_all(&[pack_keys(local)]).ok()?;
        let mut b = [0u8; 1];
        self.stream.read_exact(&mut b).ok()?;
        Some(unpack_keys(b[0]))
    }
}

/// Opens the `--host` / `--join` connection, if one was asked for.
fn connect_net(options: &Options) -> Result<Option<Net>> {
    let (stream, host) = match (options.host, &options.join) {
        (Some(port), _) => {
            println!("waiting for a player on port {port} …");
            (std::net::TcpListener::bind(("0.0.0.0", port))?.accept()?.0, true)
        }
        (None, Some(addr)) => (std::net::TcpStream::connect(addr).with_context(|| format!("connecting to {addr}"))?, false),
        _ => return Ok(None),
    };
    stream.set_nodelay(true)?;
    stream.set_read_timeout(Some(std::time::Duration::from_secs(10)))?;
    Ok(Some(Net { stream, host }))
}

fn dir_of(k: [bool; 8]) -> f32 {
    f32::from(u8::from(k[1])) - f32::from(u8::from(k[0]))
}

struct App {
    sim: Option<Sim>,
    menu: Menu,
    window: Option<Arc<Window>>,
    renderer: Option<Renderer>,
    cursor: (f32, f32),
    frame_limit: Option<u32>,
    rendered_frames: u32,
    failed: bool,
    /// Character picked in the menu (1..=11, index of the character guide); the default rider is the wheelchair.
    character: usize,
    /// Sound output (None when no device / no converted wav).
    audio: Option<audio::Audio>,
    assets: PathBuf,
    /// Local players (1 or 2; `--players N`).
    players: usize,
    net: Option<Net>,
}

impl App {
    fn new(menu: Menu, frame_limit: Option<u32>, assets: PathBuf) -> Self {
        Self {
            assets,
            players: 1,
            net: None,
            sim: None,
            audio: audio::Audio::new(),
            character: 1,
            menu,
            window: None,
            renderer: None,
            cursor: (0.0, 0.0),
            frame_limit,
            rendered_frames: 0,
            failed: false,
        }
    }

    fn initialize(&mut self, event_loop: &ActiveEventLoop) -> Result<()> {
        let attributes = Window::default_attributes()
            .with_title("Happy Wheels Rust — menu slice")
            .with_inner_size(LogicalSize::new(1280.0, 720.0));
        let window = Arc::new(event_loop.create_window(attributes)?);
        let frame = self.current_frame(window.inner_size().width, window.inner_size().height)?;
        self.renderer = Some(pollster::block_on(Renderer::new(window.clone(), &frame))?);
        self.window = Some(window);
        Ok(())
    }

    fn current_frame(&mut self, w: u32, h: u32) -> Result<RgbaImage> {
        match &self.sim {
            Some(sim) => Ok(match &sim.p2 {
                None => happywheels_core::render::render_level_scene(&sim.level, &sim.world, w.max(1), h.max(1), sim.camera(), sim.sprites.as_ref(), sim.backdrop.as_ref()),
                Some(p2) => render_split(&sim.level, &sim.world, sim.sprites.as_ref(), p2, sim.backdrop.as_ref(), w, h),
            }),
            None => self.menu.render(w, h),
        }
    }

    fn load_level_dialog(&mut self) -> Result<()> {
        let Some(path) = rfd::FileDialog::new().add_filter("Happy Wheels level (XML)", &["xml", "txt"]).pick_file() else {
            return Ok(());
        };
        let xml = std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
        let level = happywheels_core::level::parse(&xml)?;
        let level2 = happywheels_core::level::parse(&xml)?;
        let mut world = happywheels_core::physics::World::from_level(&level);
        if !world.spawn_bike(self.character, level.info.start) {
            world.spawn_character(self.character, level.info.start);
        }
        let sprites = make_sprites(&self.assets, self.character, &world, level.info.start);
        let backdrop = make_backdrop(&self.assets, &level);
        self.sim = Some(Sim { level, world, next_step: std::time::Instant::now(), drive: 0.0, keys: [false; 8], sprites, backdrop, p2: (self.players > 1).then(|| Player::new(&self.assets, &level2, self.character % 11 + 1)) });
        Ok(())
    }

    fn refresh_frame(&mut self) -> Result<()> {
        let Some(window) = &self.window else {
            return Ok(());
        };
        let size = window.inner_size();
        let frame = self.current_frame(size.width, size.height)?;
        if let Some(renderer) = &mut self.renderer {
            renderer.upload(&frame)?;
        }
        Ok(())
    }

    fn fail(&mut self, event_loop: &ActiveEventLoop, error: anyhow::Error) {
        eprintln!("{error:#}");
        self.failed = true;
        event_loop.exit();
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        if let Err(error) = self.initialize(event_loop) {
            self.fail(event_loop, error);
            return;
        }
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        let Some(sim) = &mut self.sim else { return };
        let now = std::time::Instant::now();
        if now >= sim.next_step {
            let before = sim.world.events.len();
            let mut k = sim.keys;
            let mut drive = sim.drive;
            if let (Some(net), Some(p2)) = (&mut self.net, &mut sim.p2) {
                // lockstep: host keys → player 1, client keys → player 2
                match net.exchange(sim.keys) {
                    Some(remote) => {
                        let (k1, k2) = if net.host { (sim.keys, remote) } else { (remote, sim.keys) };
                        (k, p2.keys) = (k1, k2);
                        drive = dir_of(k1);
                        p2.drive = dir_of(k2);
                    }
                    None => {
                        eprintln!("network peer disconnected");
                        self.net = None;
                    }
                }
            }
            sim.world.drive(drive);
            sim.world.set_rider_keys(happywheels_core::vehicle::Keys { left: k[0], right: k[1], up: k[2], down: k[3], space: k[4], z: k[5], shift: k[6], ctrl: k[7] });
            sim.world.step();
            if let Some(p2) = &mut sim.p2 {
                p2.step();
            }
            for e in &sim.world.events[before..] {
                println!("trigger {} fired at step {}", e.trigger, e.frame);
            }
            let sounds = sim.world.take_sounds();
            if let Some(audio) = &self.audio {
                for (name, at) in sounds {
                    audio.play(&name, at, sim.level.info.start);
                }
                audio.set_loops(&sim.world.loops, sim.level.info.start);
            }
            sim.next_step += std::time::Duration::from_secs_f64(happywheels_core::physics::TIME_STEP);
            if sim.next_step < now {
                sim.next_step = now; // don't spiral after a stall
            }
            if let Err(error) = self.refresh_frame() {
                self.fail(event_loop, error);
                return;
            }
            if let Some(window) = &self.window {
                window.request_redraw();
            }
        }
        if let Some(sim) = &self.sim {
            event_loop.set_control_flow(ControlFlow::WaitUntil(sim.next_step));
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if let Some(renderer) = &mut self.renderer {
                    renderer.resize(size.width, size.height);
                }
                if let Err(error) = self.refresh_frame() {
                    self.fail(event_loop, error);
                } else if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            WindowEvent::KeyboardInput { event: key, .. } if self.sim.is_some() && matches!(
                key.logical_key,
                winit::keyboard::Key::Named(winit::keyboard::NamedKey::ArrowLeft | winit::keyboard::NamedKey::ArrowRight | winit::keyboard::NamedKey::ArrowUp | winit::keyboard::NamedKey::ArrowDown)
            ) => {
                let pressed = key.state == ElementState::Pressed;
                let dir = match key.logical_key {
                    winit::keyboard::Key::Named(winit::keyboard::NamedKey::ArrowLeft) => -1.0,
                    _ => 1.0,
                };
                let slot = match key.logical_key {
                    winit::keyboard::Key::Named(winit::keyboard::NamedKey::ArrowLeft) => Some(0),
                    winit::keyboard::Key::Named(winit::keyboard::NamedKey::ArrowRight) => Some(1),
                    winit::keyboard::Key::Named(winit::keyboard::NamedKey::ArrowUp) => Some(2),
                    winit::keyboard::Key::Named(winit::keyboard::NamedKey::ArrowDown) => Some(3),
                    _ => None,
                };
                if let (Some(sim), Some(slot)) = (&mut self.sim, slot) {
                    sim.keys[slot] = pressed;
                }
                if matches!(key.logical_key, winit::keyboard::Key::Named(winit::keyboard::NamedKey::ArrowUp | winit::keyboard::NamedKey::ArrowDown)) {
                    return;
                }
                if let Some(sim) = &mut self.sim {
                    sim.drive = if pressed { dir } else if sim.drive == dir { 0.0 } else { sim.drive };
                }
            }
            WindowEvent::KeyboardInput { event: key, .. } if self.sim.is_some()
                && matches!(key.logical_key, winit::keyboard::Key::Named(winit::keyboard::NamedKey::Space)) =>
            {
                if let Some(sim) = &mut self.sim {
                    sim.keys[4] = key.state == ElementState::Pressed;
                }
            }
            WindowEvent::KeyboardInput { event: key, .. } if self.sim.as_ref().is_some_and(|s| s.p2.is_some())
                && matches!(&key.logical_key, winit::keyboard::Key::Character(c) if matches!(c.to_ascii_lowercase().as_str(), "a" | "d" | "w" | "s" | "f" | "g")) =>
            {
                let pressed = key.state == ElementState::Pressed;
                if let (Some(Some(p2)), winit::keyboard::Key::Character(c)) = (self.sim.as_mut().map(|s| s.p2.as_mut()), &key.logical_key) {
                    match c.to_ascii_lowercase().as_str() {
                        "a" => {
                            p2.keys[0] = pressed;
                            p2.drive = if pressed { -1.0 } else if p2.drive < 0.0 { 0.0 } else { p2.drive };
                        }
                        "d" => {
                            p2.keys[1] = pressed;
                            p2.drive = if pressed { 1.0 } else if p2.drive > 0.0 { 0.0 } else { p2.drive };
                        }
                        "w" => p2.keys[2] = pressed,
                        "s" => p2.keys[3] = pressed,
                        "f" => p2.keys[4] = pressed,
                        _ => p2.keys[5] = pressed,
                    }
                }
            }
            WindowEvent::KeyboardInput { event: key, .. } if self.sim.is_some()
                && matches!(&key.logical_key, winit::keyboard::Key::Character(c) if c.eq_ignore_ascii_case("z")) =>
            {
                if let Some(sim) = &mut self.sim {
                    sim.keys[5] = key.state == ElementState::Pressed;
                }
            }
            WindowEvent::KeyboardInput { event: key, .. } if self.sim.is_some()
                && matches!(key.logical_key, winit::keyboard::Key::Named(winit::keyboard::NamedKey::Shift | winit::keyboard::NamedKey::Control)) =>
            {
                if let Some(sim) = &mut self.sim {
                    let slot = if matches!(key.logical_key, winit::keyboard::Key::Named(winit::keyboard::NamedKey::Shift)) { 6 } else { 7 };
                    sim.keys[slot] = key.state == ElementState::Pressed;
                }
            }
            WindowEvent::KeyboardInput {
                event: winit::event::KeyEvent { logical_key: winit::keyboard::Key::Named(winit::keyboard::NamedKey::Escape), state: ElementState::Pressed, .. },
                ..
            } if self.sim.is_some() => {
                self.sim = None; // back to the menu
                event_loop.set_control_flow(ControlFlow::Wait);
                if let Err(error) = self.refresh_frame() {
                    self.fail(event_loop, error);
                } else if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.cursor = (position.x as f32, position.y as f32);
            }
            WindowEvent::MouseInput {
                state: ElementState::Pressed,
                button: MouseButton::Left,
                ..
            } if self.sim.is_none() => {
                if let Some(action) = self.menu.handle_click(self.cursor.0, self.cursor.1) {
                    match action {
                        MenuAction::Exit => event_loop.exit(),
                        MenuAction::LoadLevel => {
                            if let Err(error) = self.load_level_dialog() {
                                eprintln!("{error:#}");
                            }
                        }
                        MenuAction::CharacterSelected(character) => {
                            self.character = usize::from(character);
                            println!("selected character: {character}");
                        }
                    }
                }
                if let Err(error) = self.refresh_frame() {
                    self.fail(event_loop, error);
                } else if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            WindowEvent::RedrawRequested => {
                if let Some(renderer) = &mut self.renderer {
                    renderer.render();
                }
                self.rendered_frames += 1;
                if self
                    .frame_limit
                    .is_some_and(|limit| self.rendered_frames >= limit)
                {
                    event_loop.exit();
                }
            }
            _ => {}
        }
    }
}

struct Options {
    assets: PathBuf,
    screenshot: Option<PathBuf>,
    check_assets: bool,
    frame_limit: Option<u32>,
    level: Option<PathBuf>,
    sim_steps: u32,
    level_png: Option<PathBuf>,
    window: bool,
    players: usize,
    host: Option<u16>,
    join: Option<String>,
}

fn parse_options() -> Result<Option<Options>> {
    let default_assets = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../..")
        .join("assets-99tnk64n1mubaoc8w");
    let mut options = Options {
        assets: default_assets,
        screenshot: None,
        check_assets: false,
        frame_limit: None,
        level: None,
        sim_steps: 90,
        level_png: None,
        window: false,
        players: 1,
        host: None,
        join: None,
    };
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--assets" => {
                options.assets = PathBuf::from(args.next().context("--assets needs a path")?)
            }
            "--screenshot" => {
                options.screenshot = Some(PathBuf::from(
                    args.next().context("--screenshot needs a path")?,
                ));
            }
            "--check-assets" => options.check_assets = true,
            "--level" => options.level = Some(PathBuf::from(args.next().context("--level needs a file")?)),
            "--blood" => happywheels_core::blood::set_blood_setting(args.next().context("--blood needs 0, 1 or 2")?.parse::<u8>()?),
            "--players" => options.players = args.next().context("--players needs a number")?.parse::<usize>()?.clamp(1, 2),
            "--host" => options.host = Some(args.next().context("--host needs a port")?.parse()?),
            "--join" => options.join = Some(args.next().context("--join needs HOST:PORT")?),
            "--level-png" => options.level_png = Some(PathBuf::from(args.next().context("--level-png needs a file")?)),
            "--window" => options.window = true,
            "--steps" => options.sim_steps = args.next().context("--steps needs N")?.parse().context("invalid --steps")?,
            "--frames" => {
                let value = args.next().context("--frames needs a positive integer")?;
                let limit: u32 = value.parse().context("invalid --frames value")?;
                if limit == 0 {
                    bail!("--frames must be greater than zero");
                }
                options.frame_limit = Some(limit);
            }
            "--help" | "-h" => {
                println!(
                    "happywheels [--assets DIR] [--check-assets] [--screenshot FILE] [--frames N] [--level FILE.xml [--steps N] [--level-png F] [--window]]"
                );
                return Ok(None);
            }
            unknown => bail!("unknown option {unknown:?}; use --help"),
        }
    }
    Ok(Some(options))
}

fn main() -> Result<()> {
    env_logger::init();
    let Some(options) = parse_options()? else {
        return Ok(());
    };
    let assets = AssetRoot::new(&options.assets)?;
    if options.check_assets {
        let report = assets.check_content()?;
        println!("{}", serde_json::to_string_pretty(&report)?);
    }
    if let Some(path) = options.level.as_ref().filter(|_| !options.window) {
        // Headless simulation: load a level XML, step the (stand-in) physics, print results.
        let xml = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let level = happywheels_core::level::parse(&xml)?;
        println!(
            "level: {} shapes, {} specials, {} joints, start {:?}",
            level.shapes.len(), level.special_count, level.joint_count, level.info.start
        );
        let mut world = happywheels_core::physics::World::from_level(&level);
        if !world.spawn_bike(3, level.info.start) {
            world.spawn_character(1, level.info.start);
        }
        let sprites = make_sprites(&options.assets, 3, &world, level.info.start);
        let mut p2 = (options.players > 1).then(|| Player::new(&options.assets, &level, 4));
        if let Some(mut net) = connect_net(&options)? {
            // Headless lockstep run: scripted keys (host rides right, client rides right from step 10), both peers print the final positions —
            // they must be identical on both machines/processes.
            let mut p2 = Player::new(&options.assets, &level, 4);
            for s in 0..options.sim_steps {
                let mut mine = [false; 8];
                mine[1] = net.host || s >= 10;
                mine[2] = s % 40 < 20;
                let Some(remote) = net.exchange(mine) else { bail!("peer disconnected at step {s}") };
                let (k1, k2) = if net.host { (mine, remote) } else { (remote, mine) };
                let ks = |k: [bool; 8]| happywheels_core::vehicle::Keys { left: k[0], right: k[1], up: k[2], down: k[3], space: k[4], z: k[5], shift: k[6], ctrl: k[7] };
                world.drive(dir_of(k1));
                world.set_rider_keys(ks(k1));
                world.step();
                p2.keys = k2;
                p2.drive = dir_of(k2);
                p2.step();
            }
            use happywheels_core::character::Part;
            let pos = |w: &happywheels_core::physics::World| w.character.as_ref().and_then(|d| d.bodies.get(&Part::Chest)).and_then(|&b| w.body_position_px(b));
            println!("net-final p1 {:?} p2 {:?}", pos(&world), pos(&p2.world));
            return Ok(());
        }
        for _ in 0..options.sim_steps {
            world.step();
            if let Some(p2) = &mut p2 {
                p2.keys[1] = true; // headless demo: player 2 holds right
                p2.drive = 1.0;
                p2.step();
            }
        }
        for e in &world.events {
            println!("trigger {} fired at step {}", e.trigger, e.frame);
        }
        for i in 0..level.shapes.len() {
            if let Some([x, y]) = world.position_px(i) {
                println!("shape {i}: ({x:.1}, {y:.1}) px after {} steps", options.sim_steps);
            }
        }
        if let Some(png) = &options.level_png {
            let backdrop = make_backdrop(&options.assets, &level);
            match &p2 {
                Some(p2) => render_split(&level, &world, sprites.as_ref(), p2, backdrop.as_ref(), 1280, 720),
                None => happywheels_core::render::render_level_scene(&level, &world, 1280, 720, level.info.start, sprites.as_ref(), backdrop.as_ref()),
            }
            .save(png)?;
            println!("saved {}", png.display());
        }
        return Ok(());
    }
    if let Some(path) = options.screenshot {
        let mut menu = Menu::load(&assets)?;
        let frame = menu.render(1280, 720)?;
        frame
            .save(&path)
            .with_context(|| format!("saving screenshot {}", path.display()))?;
        println!("saved {}", path.display());
        return Ok(());
    }

    let menu = Menu::load(&assets)?;
    let mut app = App::new(menu, options.frame_limit, options.assets.clone());
    app.players = options.players;
    if let Some(net) = connect_net(&options)? {
        app.players = 2;
        app.net = Some(net);
    }
    if let (Some(path), true) = (&options.level, options.window) {
        let xml = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let level = happywheels_core::level::parse(&xml)?;
        let level2 = happywheels_core::level::parse(&xml)?;
        let mut world = happywheels_core::physics::World::from_level(&level);
        if !world.spawn_bike(app.character, level.info.start) {
            world.spawn_character(app.character, level.info.start);
        }
        let sprites = make_sprites(&options.assets, app.character, &world, level.info.start);
        let backdrop = make_backdrop(&options.assets, &level);
        app.sim = Some(Sim { level, world, next_step: std::time::Instant::now(), drive: 0.0, keys: [false; 8], sprites, backdrop, p2: (app.players > 1).then(|| Player::new(&options.assets, &level2, app.character % 11 + 1)) });
    }
    let event_loop = EventLoop::new()?;
    event_loop.set_control_flow(ControlFlow::Wait);
    event_loop.run_app(&mut app)?;
    if app.failed {
        bail!("application initialization or frame update failed");
    }
    Ok(())
}

#[cfg(test)]
mod net_tests {
    use super::*;

    #[test]
    fn peers_exchange_keys_in_lockstep() {
        let listener = std::net::TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let port = listener.local_addr().unwrap().port();
        let client = std::thread::spawn(move || {
            let mut net = Net { stream: std::net::TcpStream::connect(("127.0.0.1", port)).unwrap(), host: false };
            (0..5).map(|i| net.exchange(unpack_keys(i + 100)).unwrap()).collect::<Vec<_>>()
        });
        let mut host = Net { stream: listener.accept().unwrap().0, host: true };
        let got: Vec<_> = (0..5).map(|i| host.exchange(unpack_keys(i)).unwrap()).collect();
        let theirs = client.join().unwrap();
        for i in 0..5u8 {
            assert_eq!(pack_keys(got[usize::from(i)]), i + 100, "host receives the client's keys");
            assert_eq!(pack_keys(theirs[usize::from(i)]), i, "client receives the host's keys");
        }
        assert_eq!(dir_of(unpack_keys(0b10)), 1.0);
        assert_eq!(dir_of(unpack_keys(0b01)), -1.0);
    }
}
