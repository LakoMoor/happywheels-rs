// Web Audio mixer for the original Howler audiosprite. Times in the table are ms.
export class SpriteAudio {
  constructor(context, table) {
    this.context = context;
    this.table = table;
    this.buffer = null;
    this.voices = [];
    this.loops = new Map();
  }

  async load(bytes) {
    const buffer = await this.context.decodeAudioData(bytes);
    this.clear();
    this.buffer = buffer;
  }

  clear() {
    for (const voice of this.voices) voice.source.stop();
    for (const voice of this.loops.values()) voice.source.stop();
    this.voices = [];
    this.loops.clear();
  }

  gainAt(at, listener, volume = 1) {
    return Math.max(0, Math.min(1, 1 - Math.hypot(at[0] - listener[0], at[1] - listener[1]) / 2500)) * volume;
  }

  create(name, gain) {
    const sprite = this.table[name];
    if (!this.buffer || !sprite) return null;
    const [startMs, durationMs] = sprite;
    const start = startMs / 1000;
    const duration = Math.min(durationMs / 1000, this.buffer.duration - start);
    if (duration <= 0) return null;
    const source = this.context.createBufferSource();
    const volume = this.context.createGain();
    source.buffer = this.buffer;
    volume.gain.value = gain;
    source.connect(volume);
    volume.connect(this.context.destination);
    return { source, volume, start, duration };
  }

  play(name, at, listener) {
    // Victory uses playSoundItem in the original, not a positional sound instance.
    const gain = name === 'Victory' ? 1 : this.gainAt(at, listener);
    if (gain <= 0) return;
    const voice = this.create(name, gain);
    if (!voice) return;
    while (this.voices.length >= 24) this.voices.shift().source.stop();
    this.voices.push(voice);
    voice.source.onended = () => {
      const index = this.voices.indexOf(voice);
      if (index !== -1) this.voices.splice(index, 1);
      voice.source.disconnect();
      voice.volume.disconnect();
    };
    voice.source.start(0, voice.start, voice.duration);
  }

  syncLoops(wanted, listener) {
    const names = new Set(wanted.map(([name]) => name));
    const now = this.context.currentTime;
    for (const [name, voice] of this.loops) {
      if (names.has(name)) continue;
      voice.volume.gain.cancelScheduledValues(now);
      voice.volume.gain.setValueAtTime(voice.volume.gain.value, now);
      voice.volume.gain.linearRampToValueAtTime(0, now + 0.2);
      voice.source.stop(now + 0.2);
      this.loops.delete(name);
    }
    for (const [name, at, volume] of wanted) {
      const target = this.gainAt(at, listener, volume);
      let voice = this.loops.get(name);
      if (!voice) {
        voice = this.create(name, 0);
        if (!voice) continue;
        voice.source.loop = true;
        voice.source.loopStart = voice.start;
        voice.source.loopEnd = voice.start + voice.duration;
        voice.source.onended = () => { voice.source.disconnect(); voice.volume.disconnect(); };
        voice.source.start(0, voice.start);
        this.loops.set(name, voice);
      }
      voice.volume.gain.cancelScheduledValues(now);
      voice.volume.gain.setValueAtTime(voice.volume.gain.value, now);
      voice.volume.gain.linearRampToValueAtTime(target, now + 0.2);
    }
  }

  update([sounds, loops, listener]) {
    for (const [name, at] of sounds) this.play(name, at, listener);
    this.syncLoops(loops, listener);
  }
}
