import { test } from 'node:test';
import assert from 'node:assert/strict';
import { SpriteAudio } from './audio.mjs';

class Param {
  value = 0;
  cancelScheduledValues() {}
  setValueAtTime(value) { this.value = value; }
  linearRampToValueAtTime(value) { this.value = value; }
}
class Source {
  connect() {}
  disconnect() {}
  start(...args) { this.started = args; }
  stop(...args) { this.stopped = args; }
}
class Context {
  currentTime = 2;
  destination = {};
  async decodeAudioData() { return { duration: 30 }; }
  createBufferSource() { return new Source(); }
  createGain() { return { gain: new Param(), connect() {}, disconnect() {} }; }
  createStereoPanner() { return { pan: new Param(), connect() {}, disconnect() {} }; }
}

test('plays and bounds one-shot voices at sprite offsets with distance attenuation', async () => {
  const audio = new SpriteAudio(new Context(), { Hit: [1000, 500] });
  await audio.load(new ArrayBuffer(0));
  audio.update([[['Hit', [500, 125]]], [], [0, 0]]);
  assert.deepEqual(audio.voices[0].source.started, [0, 1, 0.5]);
  assert.equal(audio.voices[0].volume.gain.value, 0.5);
  assert.equal(audio.voices[0].panner.pan.value, 0.5);
  audio.update([[], [], [1000, 0]]);
  assert.equal(audio.voices[0].panner.pan.value, -0.5);
  audio.play('Hit', [2500, 0], [0, 0]);
  assert.equal(audio.voices.length, 1);
  for (let i = 0; i < 30; i++) audio.play('Hit', [0, 0], [0, 0]);
  assert.equal(audio.voices.length, 24);
});

test('starts, adjusts and fades loop sounds, and clears on reload', async () => {
  const audio = new SpriteAudio(new Context(), { Motor: [2000, 1000] });
  await audio.load(new ArrayBuffer(0));
  audio.syncLoops([['Motor', [0, 0], 0.5]], [0, 0]);
  const loop = audio.loops.get('Motor');
  assert.deepEqual(loop.source.started, [0, 2]);
  assert.equal(loop.source.loopEnd, 3);
  assert.equal(loop.volume.gain.value, 0.5);
  audio.syncLoops([['Motor', [500, 0], 0.5]], [0, 0]);
  assert.equal(loop.volume.gain.value, 0.25);
  assert.equal(loop.panner.pan.value, 0.5);
  audio.syncLoops([], [0, 0]);
  assert.deepEqual(loop.source.stopped, [2.2]);
  assert.equal(audio.loops.size, 0);
});

test('area gain uses separate horizontal and vertical cutoffs', () => {
  const audio = new SpriteAudio(new Context(), {});
  assert.equal(audio.gainAt([0, 250], [0, 0]), 0.5);
  assert.equal(audio.gainAt([0, 500], [0, 0]), 0);
  assert.equal(audio.gainAt([1000, 0], [0, 0]), 0);
});

test('victory is audible independent of its position', async () => {
  const audio = new SpriteAudio(new Context(), { Victory: [1000, 500], Hit: [2000, 500] });
  await audio.load(new ArrayBuffer(0));
  audio.play('Victory', [5000, 0], [0, 0]);
  audio.play('Hit', [5000, 0], [0, 0]);
  assert.equal(audio.voices.length, 1);
  assert.equal(audio.voices[0].volume.gain.value, 1);
});

test('sound triggers use their volume, location and global pan', async () => {
  const audio = new SpriteAudio(new Context(), { Hit: [1000, 500] });
  await audio.load(new ArrayBuffer(0));
  audio.update([[], [], [0, 0], [{name:'Hit', position:[5000, 0], global:true, volume:0.4, pan:-0.5}]]);
  assert.equal(audio.voices[0].volume.gain.value, 0.4);
  assert.equal(audio.voices[0].panner.pan.value, -0.5);
  audio.update([[], [], [0, 0], [{name:'Hit', position:[500, 0], global:false, volume:0.4, pan:0}]]);
  assert.equal(audio.voices[1].volume.gain.value, 0.2);
  audio.update([[], [], [250, 0], []]);
  assert.ok(Math.abs(audio.voices[1].volume.gain.value - 0.3) < 1e-9);
});

test('positional one-shots stop beyond the cutoff but remain alive on its boundary', async () => {
  const audio = new SpriteAudio(new Context(), { Hit: [1000, 500] });
  await audio.load(new ArrayBuffer(0));
  audio.update([[['Hit', [1000, 0]]], [], [0, 0]]);
  const voice = audio.voices[0];
  assert.equal(voice.volume.gain.value, 0);
  audio.update([[], [], [1, 0]]);
  assert.ok(voice.volume.gain.value > 0);
  audio.update([[], [], [-1, 0]]);
  assert.deepEqual(voice.source.stopped, []);
  assert.equal(audio.voices.length, 0);
  audio.update([[], [], [0, 0]]);
  assert.equal(audio.voices.length, 0);
});

test('area loops stop outside the cutoff and restart at the sprite start', async () => {
  const audio = new SpriteAudio(new Context(), { Motor: [2000, 1000] });
  await audio.load(new ArrayBuffer(0));
  audio.syncLoops([['Motor', [1001, 0], 1]], [0, 0]);
  assert.equal(audio.loops.size, 0);
  audio.syncLoops([['Motor', [1000, 0], 1]], [0, 0]);
  const first = audio.loops.get('Motor');
  assert.equal(first.volume.gain.value, 0);
  audio.syncLoops([['Motor', [1001, 0], 1]], [0, 0]);
  assert.deepEqual(first.source.stopped, []);
  assert.equal(audio.loops.size, 0);
  audio.syncLoops([['Motor', [0, 0], 0.5]], [0, 0]);
  const second = audio.loops.get('Motor');
  assert.notEqual(second, first);
  assert.deepEqual(second.source.started, [0, 2]);
  assert.equal(second.volume.gain.value, 0.5);
  audio.syncLoops([['Motor', [0, 0], 0]], [0, 0]);
  assert.deepEqual(second.source.stopped, []);
  assert.equal(audio.loops.size, 0);
});

test('identical loops from different bodies play independently', async () => {
  const audio = new SpriteAudio(new Context(), { MineHover: [2000, 1000] });
  await audio.load(new ArrayBuffer(0));
  audio.syncLoops([['MineHover', [-500, 0], 1, 10], ['MineHover', [500, 0], 1, 11]], [0, 0]);
  assert.equal(audio.loops.size, 2);
  assert.equal(audio.loops.get('10:MineHover').panner.pan.value, -0.5);
  assert.equal(audio.loops.get('11:MineHover').panner.pan.value, 0.5);
  audio.syncLoops([['MineHover', [500, 0], 1, 11]], [0, 0]);
  assert.equal(audio.loops.size, 1);
  assert.ok(audio.loops.has('11:MineHover'));
});

test('master volume is clamped and scaled sounds multiply the positional gain', async () => {
  const audio = new SpriteAudio(new Context(), { Hit: [1000, 500] });
  await audio.load(new ArrayBuffer(0));
  audio.setVolume(2);
  assert.equal(audio.master.gain.value, 1);
  audio.setVolume(0.4);
  assert.equal(audio.master.gain.value, 0.4);
  audio.update([[], [], [0, 0], [], [['Hit', [500, 125], 0.5]]]);
  assert.equal(audio.voices[0].volume.gain.value, 0.25);
});
