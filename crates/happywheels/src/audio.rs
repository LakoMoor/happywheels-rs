//! Sound effects: the original's `audiosprite.big.mp3` (converted once to `assets/audiosprite.wav`, mono 24 kHz) is cut into
//! one-shot voices by the Howler sprite table (`happywheels-core/data/audiosprite.json`: name → [start ms, duration ms]) and mixed
//! on the default output device with cpal. Playback volume falls off with distance from the camera (a simplification of
//! `playAreaSoundInstance`); if there is no output device or no wav the game simply runs silent.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{FromSample, SampleFormat, SizedSample};

const SPRITES: &str = include_str!("../../happywheels-core/data/audiosprite.json");
const WAV_RATE: f64 = 24_000.0;
/// Original cutoffs are 16 × 8 physics metres (62.5 pixels per metre).
const SOUND_X_CUTOFF: f32 = 1000.0;
const SOUND_Y_CUTOFF: f32 = 500.0;

fn area_gain(at: [f32; 2], listener: [f32; 2]) -> f32 {
    let x = (at[0] - listener[0]).abs() / SOUND_X_CUTOFF;
    let y = (at[1] - listener[1]).abs() / SOUND_Y_CUTOFF;
    (1.0 - x.max(y)).clamp(0.0, 1.0)
}
fn area_pan(at: [f32; 2], listener: [f32; 2]) -> f32 {
    ((at[0] - listener[0]) / SOUND_X_CUTOFF).clamp(-1.0, 1.0)
}

fn stereo_gains(pan: f32) -> [f32; 2] {
    let angle = (pan + 1.0) * std::f32::consts::FRAC_PI_4;
    [angle.cos(), angle.sin()]
}
const MAX_VOICES: usize = 24;

/// `Victory` is requested with playSoundItem in the original, independently of position.
fn sound_gain(name: &str, at: [f32; 2], listener: [f32; 2]) -> f32 {
    if name == "Victory" {
        return 1.0;
    }
    area_gain(at, listener)
}

struct Voice {
    end: usize,
    /// Fractional read position in source samples.
    at: f64,
    gain: f32,
    pan: f32,
}

/// A looping voice (`playAreaSoundLoop`): fades in/out over 0.2 s towards `target`, dropped once silent with target 0.
struct LoopVoice {
    name: String,
    start: usize,
    end: usize,
    at: f64,
    gain: f32,
    target: f32,
    pan: f32,
}

struct Mixer {
    samples: Vec<f32>,
    voices: Vec<Voice>,
    loops: Vec<LoopVoice>,
    /// Source samples advanced per output frame.
    step: f64,
    /// Gain change per output frame for a 0.2 s fade.
    fade: f32,
}

pub struct Audio {
    table: HashMap<String, [f64; 2]>,
    mixer: Arc<Mutex<Mixer>>,
    _stream: cpal::Stream,
}

fn wav_path() -> Option<PathBuf> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/audiosprite.wav");
    [root, PathBuf::from("assets/audiosprite.wav")].into_iter().find(|p| p.exists())
}

/// Mono 16-bit PCM wav → f32 samples (walks the RIFF chunks to find `data`).
fn read_wav(bytes: &[u8]) -> Option<Vec<f32>> {
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" {
        return None;
    }
    let mut i = 12;
    while i + 8 <= bytes.len() {
        let size = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().ok()?) as usize;
        if &bytes[i..i + 4] == b"data" {
            let data = &bytes[i + 8..(i + 8 + size).min(bytes.len())];
            return Some(data.chunks_exact(2).map(|b| f32::from(i16::from_le_bytes([b[0], b[1]])) / 32768.0).collect());
        }
        i += 8 + size + (size & 1);
    }
    None
}

fn fill<T: SizedSample + FromSample<f32>>(mixer: &Mutex<Mixer>, out: &mut [T], channels: usize) {
    let Ok(mut m) = mixer.lock() else { return };
    let Mixer { samples, voices, loops, step, fade } = &mut *m;
    for frame in out.chunks_mut(channels) {
        let mut stereo = [0.0f32; 2];
        let mut mono = 0.0f32;
        for voice in voices.iter_mut() {
            let i = voice.at as usize;
            if i + 1 < voice.end {
                let f = (voice.at - i as f64) as f32;
                let v = (samples[i] * (1.0 - f) + samples[i + 1] * f) * voice.gain;
                let gains = stereo_gains(voice.pan);
                stereo[0] += v * gains[0];
                stereo[1] += v * gains[1];
                mono += v;
                voice.at += *step;
            } else {
                voice.at = f64::MAX;
            }
        }
        for l in loops.iter_mut() {
            if l.end > l.start + 1 {
                let i = l.at as usize;
                let f = (l.at - i as f64) as f32;
                let j = if i + 1 < l.end { i + 1 } else { l.start };
                let v = (samples[i] * (1.0 - f) + samples[j] * f) * l.gain;
                let gains = stereo_gains(l.pan);
                stereo[0] += v * gains[0];
                stereo[1] += v * gains[1];
                mono += v;
                l.at += *step;
                if l.at >= l.end as f64 {
                    l.at = l.start as f64 + (l.at - l.end as f64);
                }
            }
            if l.gain < l.target {
                l.gain = (l.gain + *fade).min(l.target);
            } else if l.gain > l.target {
                l.gain = (l.gain - *fade).max(l.target);
            }
        }
        let count = frame.len();
        for (i, c) in frame.iter_mut().enumerate() {
            let v = if count == 1 || i >= 2 { mono } else { stereo[i] };
            *c = T::from_sample(v.clamp(-1.0, 1.0));
        }
    }
    voices.retain(|v| v.at < v.end as f64);
    loops.retain(|l| l.gain > 0.0 || l.target > 0.0);
}

impl Audio {
    pub fn new() -> Option<Self> {
        let samples = read_wav(&std::fs::read(wav_path()?).ok()?)?;
        let table: HashMap<String, [f64; 2]> = serde_json::from_str(SPRITES).ok()?;
        let device = cpal::default_host().default_output_device()?;
        let config = device.default_output_config().ok()?;
        let channels = config.channels() as usize;
        let rate = f64::from(config.sample_rate().0);
        let mixer = Arc::new(Mutex::new(Mixer { samples, voices: Vec::new(), loops: Vec::new(), step: WAV_RATE / rate, fade: (1.0 / (0.2 * rate)) as f32 }));
        let m = mixer.clone();
        let err = |e| log::warn!("audio stream error: {e}");
        let cfg = config.config();
        let stream = match config.sample_format() {
            SampleFormat::F32 => device.build_output_stream(&cfg, move |d: &mut [f32], _| fill(&m, d, channels), err, None),
            SampleFormat::I16 => device.build_output_stream(&cfg, move |d: &mut [i16], _| fill(&m, d, channels), err, None),
            SampleFormat::U16 => device.build_output_stream(&cfg, move |d: &mut [u16], _| fill(&m, d, channels), err, None),
            _ => return None,
        }
        .ok()?;
        stream.play().ok()?;
        Some(Self { table, mixer, _stream: stream })
    }

    /// Syncs the looping voices with `wanted` (sprite name, position px): new ones fade in, missing ones fade out, kept ones follow the distance gain.
    pub fn set_loops(&self, wanted: &[(String, [f32; 2], f32)], listener: [f32; 2]) {
        let Ok(mut m) = self.mixer.lock() else { return };
        for l in &mut m.loops {
            if !wanted.iter().any(|w| w.0 == l.name) {
                l.target = 0.0;
            }
        }
        for (name, at, volume) in wanted {
            let target = area_gain(*at, listener) * volume;
            let pan = area_pan(*at, listener);
            if let Some(l) = m.loops.iter_mut().find(|l| &l.name == name) {
                l.target = target;
                l.pan = pan;
            } else if let Some(&[start_ms, dur_ms]) = self.table.get(name.as_str()) {
                let start = (start_ms * WAV_RATE / 1000.0) as usize;
                let end = (((start_ms + dur_ms) * WAV_RATE / 1000.0) as usize).min(m.samples.len());
                m.loops.push(LoopVoice { name: name.clone(), start, end, at: start as f64, gain: 0.0, target, pan });
            }
        }
    }

    /// Plays sprite `name` heard from `listener` (px) while the sound sits at `at` (px).
    pub fn play(&self, name: &str, at: [f32; 2], listener: [f32; 2]) {
        let Some(&[start_ms, dur_ms]) = self.table.get(name) else { return };
        let gain = sound_gain(name, at, listener);
        if gain <= 0.0 {
            return;
        }
        let Ok(mut m) = self.mixer.lock() else { return };
        if m.voices.len() >= MAX_VOICES {
            m.voices.remove(0);
        }
        let start = (start_ms * WAV_RATE / 1000.0) as usize;
        let end = (((start_ms + dur_ms) * WAV_RATE / 1000.0) as usize).min(m.samples.len());
        let pan = if name == "Victory" { 0.0 } else { area_pan(at, listener) };
        m.voices.push(Voice { end, at: start as f64, gain, pan });
    }
}

#[cfg(test)]
mod tests {
    use super::{area_gain, area_pan, fill, sound_gain, Mixer, Voice};
    use std::sync::Mutex;

    #[test]
    fn victory_is_global_but_effects_are_positional() {
        assert_eq!(sound_gain("Victory", [5000.0, 0.0], [0.0, 0.0]), 1.0);
        assert_eq!(sound_gain("Bleep3", [5000.0, 0.0], [0.0, 0.0]), 0.0);
    }

    #[test]
    fn area_gain_uses_original_rectangular_cutoffs() {
        assert_eq!(area_gain([500.0, 125.0], [0.0, 0.0]), 0.5);
        assert_eq!(area_gain([0.0, 250.0], [0.0, 0.0]), 0.5);
        assert_eq!(area_gain([0.0, 500.0], [0.0, 0.0]), 0.0);
        assert_eq!(area_gain([1000.0, 0.0], [0.0, 0.0]), 0.0);
    }

    #[test]
    fn mixer_pans_stereo_and_preserves_mono() {
        let mixer = |pan| Mutex::new(Mixer { samples: vec![0.5; 3], voices: vec![Voice { end: 3, at: 0.0, gain: 1.0, pan }], loops: vec![], step: 1.0, fade: 0.0 });
        let mut stereo = [0.0f32; 2];
        fill(&mixer(-1.0), &mut stereo, 2);
        assert!((stereo[0] - 0.5).abs() < 1e-6 && stereo[1].abs() < 1e-6);
        fill(&mixer(1.0), &mut stereo, 2);
        assert!(stereo[0].abs() < 1e-6 && (stereo[1] - 0.5).abs() < 1e-6);
        let mut mono = [0.0f32; 1];
        fill(&mixer(1.0), &mut mono, 1);
        assert_eq!(mono[0], 0.5);
        assert_eq!(area_pan([500.0, 0.0], [0.0, 0.0]), 0.5);
    }
}
