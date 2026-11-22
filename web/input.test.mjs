import { test } from 'node:test';
import assert from 'node:assert/strict';
import { GameInput } from './input.mjs';

test('releases game keys when focus changes and preserves paired modifiers', () => {
  const states = new Map();
  const input = new GameInput({key: (i, down) => states.set(i, down), clear_keys: () => states.clear()});
  assert.equal(input.key('ArrowRight', true, false), true);
  assert.equal(input.key('ArrowRight', false, true), true);
  assert.equal(states.get(1), false);
  assert.equal(input.key('ArrowUp', true, true), false);
  input.key('ShiftLeft', true, false);
  input.key('ShiftRight', true, false);
  input.key('ShiftLeft', false, false);
  assert.equal(states.get(6), true);
  input.key('ShiftRight', false, false);
  assert.equal(states.get(6), false);
  input.key('ArrowLeft', true, false);
  input.clear();
  assert.equal(states.size, 0);
  assert.equal(input.held.size, 0);
});
