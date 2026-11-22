import init, { AssetPack, Game, audio_table } from './pkg/happywheels_web.js';
import { SpriteAudio } from './audio.mjs';

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
  const audioContext = new AudioContext();
  const audio = new SpriteAudio(audioContext, JSON.parse(audio_table()));
  const resumeAudio = () => { if (audioContext.state === 'suspended') audioContext.resume().catch(console.error); };
  window.addEventListener('pointerdown', resumeAudio);
  window.addEventListener('keydown', resumeAudio);
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
  document.querySelector('#assets').addEventListener('change', async event => {
    const audioFile = [...event.target.files].find(file => file.name === 'audiosprite.big.mp3')
      || [...event.target.files].find(file => file.name === 'audiosprite.big.ogg');
    const files = [...event.target.files].map(file => {
      const parts = file.webkitRelativePath.split('/');
      const start = parts.indexOf('animate');
      return { file, path: start < 0 ? '' : parts.slice(start).join('/') };
    }).filter(({path}) => /^animate\/(character\d+|game|backgrounds)\/.*@0\.3x\.(json|png)$/.test(path));
    if (!files.length) {
      status.textContent = 'No sprite atlases found; select the original assets folder.';
      return;
    }
    status.textContent = 'Loading artwork…';
    const pack = new AssetPack();
    try {
      for (const {file, path} of files) pack.add(path, new Uint8Array(await file.arrayBuffer()));
      game.set_assets(pack);
      if (audioFile) {
        try {
          await audio.load(await audioFile.arrayBuffer());
          status.textContent = `Loaded ${files.length} local atlas files and sound.`;
        } catch (error) {
          status.textContent = `Artwork loaded; sound could not be decoded: ${error}`;
        }
      } else {
        audio.clear();
        audio.buffer = null;
        status.textContent = `Loaded ${files.length} local atlas files; audio sprite not found.`;
      }
    } catch (error) {
      status.textContent = `Could not load artwork: ${error}`;
    }
  });
  document.querySelector('#level').addEventListener('change', async event => {
    const file = event.target.files[0];
    if (!file) return;
    try {
      game.load_level(await file.text(), Number(character.value));
      audio.clear();
      held.clear();
      status.textContent = `Loaded ${file.name}.`;
    } catch (error) {
      status.textContent = `Could not load level: ${error}`;
    }
  });
  const draw = time => {
    try {
      audio.update(JSON.parse(game.frame(time)));
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
