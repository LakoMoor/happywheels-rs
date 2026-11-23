import init, { AssetPack, Game, audio_table } from './pkg/happywheels_web.js';
import { SpriteAudio } from './audio.mjs';
import { GameInput } from './input.mjs';

const canvas = document.querySelector('#game');
const status = document.querySelector('#status');
const character = document.querySelector('#character');
for (let n = 1; n <= 11; n++) {
  character.add(new Option(`Character ${n}`, String(n), n === 3, n === 3));
}

try {
  await init();
  const game = await Game.create(canvas);
  const audioContext = new AudioContext();
  const audio = new SpriteAudio(audioContext, JSON.parse(audio_table()));
  const resumeAudio = () => { if (audioContext.state === 'suspended') audioContext.resume().catch(console.error); };
  window.addEventListener('pointerdown', resumeAudio);
  window.addEventListener('keydown', resumeAudio);
  status.textContent = 'Running sample level.';
  const input = new GameInput(game);
  let menuOpen = false;
  let paused = false;
  let victoryShown = false;
  const pauseButton = document.querySelector('#pause');
  const setPaused = value => {
    paused = value;
    game.set_paused(value);
    input.clear();
    if (value) audio.clear();
    pauseButton.textContent = value ? 'Resume' : 'Pause';
    pauseButton.setAttribute('aria-pressed', String(value));
  };
  const pointer = (event, clicked) => {
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    if (menuOpen) {
      if (!clicked) return;
      const action = game.menu_click((event.clientX - rect.left) * canvas.width / rect.width, (event.clientY - rect.top) * canvas.height / rect.height);
      if (action > 0) {
        character.value = String(action);
        game.restart(action);
        setPaused(false);
        victoryShown = false;
        menuOpen = false;
        audio.clear();
        status.textContent = `Started character ${action}.`;
      } else if (action === -1) {
        document.querySelector('#level').click();
      } else if (action === -2) {
        menuOpen = false;
      }
    } else {
      game.pointer((event.clientX - rect.left) * 900 / rect.width, (event.clientY - rect.top) * 500 / rect.height, clicked);
    }
  };
  canvas.addEventListener('pointerup', event => { if (event.button === 0) pointer(event, true); });
  canvas.addEventListener('pointermove', event => pointer(event, false));
  canvas.addEventListener('pointerleave', () => { if (!menuOpen) game.pointer(-1e9, -1e9, false); });
  const updateKey = (event, down) => {
    const target = event.target;
    const editing = target instanceof HTMLElement && (target.isContentEditable || target.closest('input, select, textarea, button'));
    if (event.code === 'Escape' && !editing && !menuOpen) {
      if (down && !event.repeat) { setPaused(!paused); status.textContent = paused ? 'Paused.' : 'Resumed.'; }
      event.preventDefault();
      return;
    }
    if (input.key(event.code, down, Boolean(editing) || paused || menuOpen)) event.preventDefault();
  };
  window.addEventListener('keydown', event => updateKey(event, true));
  window.addEventListener('keyup', event => updateKey(event, false));
  window.addEventListener('blur', () => input.clear());
  document.addEventListener('visibilitychange', () => {
    game.reset_clock();
    if (document.hidden) input.clear();
  });
  document.addEventListener('focusin', event => {
    if (event.target instanceof HTMLElement && (event.target.isContentEditable || event.target.closest('input, select, textarea, button'))) input.clear();
  });
  document.querySelector('#assets').addEventListener('change', async event => {
    const audioFile = [...event.target.files].find(file => file.name === 'audiosprite.big.mp3')
      || [...event.target.files].find(file => file.name === 'audiosprite.big.ogg');
    const files = [...event.target.files].map(file => {
      const parts = file.webkitRelativePath.split('/');
      const start = parts.findIndex(part => ['animate', 'image', 'fonts'].includes(part));
      return { file, path: start < 0 ? '' : parts.slice(start).join('/') };
    }).filter(({path}) => /^animate\/(character\d+|game|backgrounds)\/.*@0\.3x\.(json|png)$/.test(path)
      || /^animate\/game-ui\/game_ui_atlas_[13]\/game_ui_atlas_[13]\.(json|png)$/.test(path)
      || /^image\/character_images\/character_images\.(json|png)$/.test(path)
      || /^fonts\/(ClarendonLTStd-Bold|HelveticaNeueLTStd-Roman)\.otf$/.test(path));
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
      setPaused(false);
      menuOpen = false;
      audio.clear();
      victoryShown = false;
      input.clear();
      status.textContent = `Loaded ${file.name}.`;
    } catch (error) {
      status.textContent = `Could not load level: ${error}`;
    }
  });
  document.querySelector('#restart').addEventListener('click', () => {
    try {
      game.set_blood_setting(Number(document.querySelector('#blood').value));
      game.restart(Number(character.value));
      setPaused(false);
      menuOpen = false;
      audio.clear();
      victoryShown = false;
      input.clear();
      status.textContent = 'Level restarted.';
      canvas.focus();
    } catch (error) {
      status.textContent = `Could not restart level: ${error}`;
    }
  });
  pauseButton.addEventListener('click', () => {
    if (menuOpen) return;
    setPaused(!paused);
    status.textContent = paused ? 'Paused.' : 'Resumed.';
    canvas.focus();
  });
  document.querySelector('#menu').addEventListener('click', () => {
    input.clear();
    menuOpen = game.open_menu();
    audio.clear();
    status.textContent = menuOpen ? 'Menu open.' : 'Select the original assets folder to load the menu.';
  });
  const draw = time => {
    try {
      audio.update(JSON.parse(game.frame(time)));
      if (!victoryShown && game.level_complete()) {
        victoryShown = true;
        status.textContent = 'Level complete!';
      }
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
