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
}

test('plays and bounds one-shot voices at sprite offsets with distance attenuation', async () => {
  const audio = new SpriteAudio(new Context(), { Hit: [1000, 500] });
  await audio.load(new ArrayBuffer(0));
  audio.update([[['Hit', [1250, 0]]], [], [0, 0]]);
  assert.deepEqual(audio.voices[0].source.started, [0, 1, 0.5]);
  assert.equal(audio.voices[0].volume.gain.value, 0.5);
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
  audio.syncLoops([['Motor', [1250, 0], 0.5]], [0, 0]);
  assert.equal(loop.volume.gain.value, 0.25);
  audio.syncLoops([], [0, 0]);
  assert.deepEqual(loop.source.stopped, [2.2]);
  assert.equal(audio.loops.size, 0);
});

test('victory is audible independent of its position', async () => {
  const audio = new SpriteAudio(new Context(), { Victory: [1000, 500], Hit: [2000, 500] });
  await audio.load(new ArrayBuffer(0));
  audio.play('Victory', [5000, 0], [0, 0]);
  audio.play('Hit', [5000, 0], [0, 0]);
  assert.equal(audio.voices.length, 1);
  assert.equal(audio.voices[0].volume.gain.value, 1);
});
