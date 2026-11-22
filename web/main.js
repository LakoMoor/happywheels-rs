import init, { Game } from './pkg/happywheels_web.js';

const canvas = document.querySelector('#game');
const status = document.querySelector('#status');
const character = document.querySelector('#character');
for (let n = 1; n <= 11; n++) {
  character.add(new Option(`Character ${n}`, String(n), n === 3, n === 3));
}

const bindings = new Map([
  ['ArrowLeft', 0], ['ArrowRight', 1], ['ArrowUp', 2], ['ArrowDown', 3],
  ['Space', 4], ['KeyZ', 5], ['ShiftLeft', 6], ['ShiftRight', 6],
  ['ControlLeft', 7], ['ControlRight', 7],
]);

try {
  await init();
  const game = await Game.create(canvas);
  status.textContent = 'Running sample level.';
  const held = new Set();
  const updateKey = (event, down) => {
    const index = bindings.get(event.code);
    if (index === undefined) return;
    if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement) return;
    event.preventDefault();
    if (down) held.add(event.code); else held.delete(event.code);
    game.key(index, [...held].some(code => bindings.get(code) === index));
  };
  window.addEventListener('keydown', event => updateKey(event, true));
  window.addEventListener('keyup', event => updateKey(event, false));
  window.addEventListener('blur', () => { held.clear(); game.clear_keys(); });
  document.querySelector('#level').addEventListener('change', async event => {
    const file = event.target.files[0];
    if (!file) return;
    try {
      game.load_level(await file.text(), Number(character.value));
      held.clear();
      status.textContent = `Loaded ${file.name}.`;
    } catch (error) {
      status.textContent = `Could not load level: ${error}`;
    }
  });
  const draw = time => {
    try {
      game.frame(time);
      requestAnimationFrame(draw);
    } catch (error) {
      status.textContent = `Game stopped: ${error}`;
      console.error(error);
    }
  };
  requestAnimationFrame(draw);
} catch (error) {
  status.textContent = `Could not start WebGPU: ${error}`;
  console.error(error);
}
