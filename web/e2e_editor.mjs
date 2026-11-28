// End-to-end browser check of the level editor (not run by `node --test`): serve web/ on :8011, then
// `node web/e2e_editor.mjs OUTDIR`; with HW_ASSETS (and the symlink web/_assets -> $HW_ASSETS) the original art is loaded
// first. Needs desktop Chrome with WebGPU. Drives the editor with real mouse and key events, writes OUTDIR/editor*.png.
import { spawn } from 'node:child_process';
import { mkdirSync, readdirSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const out = process.argv[2];
mkdirSync(out, { recursive: true });
const ch = spawn('/Applications/Google Chrome.app/Contents/MacOS/Google Chrome', ['--headless=new', '--enable-unsafe-webgpu', '--use-angle=metal', '--remote-debugging-port=9333', '--window-size=1100,1500', `--user-data-dir=${out}/chrome-prof`, 'about:blank'], { stdio: 'ignore' });
const sleep = ms => new Promise(r => setTimeout(r, ms));
let targets;
for (let i = 0; i < 40; i++) { try { targets = await (await fetch('http://127.0.0.1:9333/json')).json(); break; } catch { await sleep(250); } }
const ws = new WebSocket(targets.find(t => t.type === 'page').webSocketDebuggerUrl);
await new Promise(r => ws.addEventListener('open', r, { once: true }));
let id = 0; const pending = new Map(); const errors = [];
ws.addEventListener('message', ({ data }) => {
  const m = JSON.parse(data);
  if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
  else if (m.method === 'Runtime.exceptionThrown' || (m.method === 'Runtime.consoleAPICalled' && m.params.type === 'error')) errors.push(JSON.stringify(m.params).slice(0, 400));
});
const send = (method, params = {}) => new Promise(r => { const i = ++id; pending.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
const evaluate = async expression => (await send('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true })).result.result.value;
const status = () => evaluate('document.querySelector("#status").textContent');

await send('Runtime.enable'); await send('Page.enable');
await send('Page.navigate', { url: 'http://localhost:8011/' });
await sleep(3000);

const root = process.env.HW_ASSETS;
if (root) {
  const files = [];
  (function walk(d) { for (const f of readdirSync(d)) { const p = join(d, f); statSync(p).isDirectory() ? walk(p) : files.push(p.slice(root.length + 1)); } })(root);
  await evaluate(`(async()=>{const dt=new DataTransfer();for(const p of ${JSON.stringify(files)}){if(!/\\.(json|png|otf)$/.test(p))continue;const r=await fetch('/_assets/'+p);const f=new File([await r.blob()],p.split('/').pop());Object.defineProperty(f,'webkitRelativePath',{value:'assets/'+p});dt.items.add(f);}const el=document.querySelector('#assets');el.files=dt.files;el.dispatchEvent(new Event('change',{bubbles:true}));return dt.files.length})()`);
  await sleep(15000);
  console.log('STATUS', await status());
}

// the saved file, as the page would download it
await evaluate(`window.__saved = null; URL.createObjectURL = blob => { blob.text().then(t => { window.__saved = t; }); return 'about:blank'; }; HTMLAnchorElement.prototype.click = function () {};`);
const click = selector => evaluate(`document.querySelector(${JSON.stringify(selector)}).click()`);
const save = async () => { await click('[data-command="save"]'); await sleep(200); return evaluate('window.__saved'); };

await evaluate('window.confirm = () => true');
await click('#editor-toggle');
await sleep(500);
await click('[data-command="new"]');
await evaluate('document.querySelector("#game").scrollIntoView()');
const box = await evaluate('(()=>{const r=document.querySelector("#game").getBoundingClientRect();return [r.left,r.top,r.width,r.height]})()');
// positions in the original's 900 x 500 view
const at = (x, y) => ({ x: box[0] + x * box[2] / 900, y: box[1] + y * box[3] / 500 });
const mouse = (type, p, extra = {}) => send('Input.dispatchMouseEvent', { type, ...p, button: 'left', clickCount: 1, ...extra });
const tap = async (x, y) => { await mouse('mouseMoved', at(x, y)); await mouse('mousePressed', at(x, y)); await mouse('mouseReleased', at(x, y)); await sleep(60); };
const drag = async (a, b) => { await mouse('mousePressed', at(...a)); await mouse('mouseMoved', at(...b), { buttons: 1 }); await mouse('mouseReleased', at(...b)); await sleep(60); };
const press = async (keyCode, key, code, modifiers = 0) => {
  const e = { windowsVirtualKeyCode: keyCode, key, code, modifiers };
  await send('Input.dispatchKeyEvent', { type: 'rawKeyDown', ...e });
  await send('Input.dispatchKeyEvent', { type: 'keyUp', ...e });
  await sleep(60);
};
await evaluate('document.querySelector("#game").focus()');
let shot;

// a floor (shape tool, made wide), a polygon, an I-beam, a joint, a trigger
await press(50, '2', 'Digit2'); await tap(450, 420);
await press(49, '1', 'Digit1'); await tap(450, 420);
for (let i = 0; i < 60; i++) await press(68, 'd', 'KeyD', 8);
await press(51, '3', 'Digit3');
for (const p of [[600, 330], [700, 330], [700, 390], [600, 330]]) await tap(...p);
await evaluate('(()=>{const s=document.querySelector("#ed-special");s.value="3";s.dispatchEvent(new Event("change"))})()');
await press(54, '6', 'Digit6'); await tap(250, 150);
await press(53, '5', 'Digit5'); await tap(250, 150);
await press(56, '8', 'Digit8'); await tap(780, 200);
await press(49, '1', 'Digit1'); await tap(780, 200);
// select the trigger and the polygon, then link them
await mouse('mousePressed', at(680, 360), { modifiers: 8 }); await mouse('mouseReleased', at(680, 360), { modifiers: 8 });
await click('[data-command="link"]');
// pick an action with parameters for the linked polygon and change one
await tap(100, 60);
await tap(780, 200);
const choose = (selector, value) => evaluate(`(()=>{const e=document.querySelector(${JSON.stringify(selector)});e.value=${JSON.stringify(value)};e.dispatchEvent(new Event('change'))})()`);
await choose('#ed-targets select', '4');
await choose('#ed-targets input[type=number]', '25');
shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(join(out, 'editor-trigger.png'), Buffer.from(shot.result.data, 'base64'));
await tap(100, 60);
await drag([60, 40], [60, 40]);
await tap(650, 350);
await sleep(500);
shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(join(out, 'editor.png'), Buffer.from(shot.result.data, 'base64'));
const xml = await save();
writeFileSync(join(out, 'editor-level.xml'), xml ?? '');
console.log('XML', xml);

// vertex mode: double click the polygon, drag its lower corner, insert a vertex, leave
const dbl = async (x, y) => { await tap(x, y); await mouse('mousePressed', at(x, y), { clickCount: 2 }); await mouse('mouseReleased', at(x, y), { clickCount: 2 }); await sleep(100); };
await tap(100, 60);
await dbl(680, 345);
await drag([700, 390], [760, 390]);
await press(13, 'Enter', 'Enter');
await sleep(300);
shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(join(out, 'editor-vertices.png'), Buffer.from(shot.result.data, 'base64'));
console.log('VERTICES', await evaluate('document.querySelector("#ed-selected").textContent'));
await dbl(100, 60);
const edited = await save();
console.log('EDITED', edited.match(/<v [^>]*>/)?.[0], edited !== xml);

// group the floor and the polygon, step inside, move the polygon, step out
await tap(100, 60);
await tap(200, 450);
await mouse('mousePressed', at(690, 345), { modifiers: 8 }); await mouse('mouseReleased', at(690, 345), { modifiers: 8 });
await click('[data-command="group"]');
await tap(100, 60);
await dbl(200, 450);
await tap(690, 345);
for (let i = 0; i < 4; i++) await press(38, 'ArrowUp', 'ArrowUp', 8);
await sleep(200);
shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(join(out, 'editor-group.png'), Buffer.from(shot.result.data, 'base64'));
console.log('GROUP', await evaluate('document.querySelector("#ed-selected").textContent'));
await dbl(100, 60);
const grouped = await save();
console.log('GROUPED', grouped.match(/<groups>[\s\S]*<\/groups>/)?.[0]);

await press(84, 't', 'KeyT');
await sleep(2500);
console.log('TEST', await status());
shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(join(out, 'editor-test.png'), Buffer.from(shot.result.data, 'base64'));
await press(84, 't', 'KeyT');
await sleep(500);
console.log('BACK', await status(), (await save()) === grouped);
await press(90, 'z', 'KeyZ', 2);
console.log('UNDO changed', (await save()) !== grouped);
// the NPC poser: place a character, double click it, pull its near arm, leave
await choose('#ed-special', '17');
await press(54, '6', 'Digit6'); await tap(450, 130);
await press(49, '1', 'Digit1'); await tap(100, 60);
await dbl(450, 130);
await mouse('mousePressed', at(450, 135));
for (let i = 1; i <= 5; i++) await mouse('mouseMoved', at(450 + 10 * i, 135 - 7 * i), { buttons: 1 });
await mouse('mouseReleased', at(500, 100));
await sleep(500);
shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(join(out, 'editor-poser.png'), Buffer.from(shot.result.data, 'base64'));
await dbl(100, 60);
console.log('POSE', (await save()).match(/<sp t=.17.[^>]*>/)?.[0]);
await click('#editor-toggle');
await sleep(500);
console.log('CLOSE', await status());
console.log('ERRORS', errors.length, errors.slice(0, 3));
ch.kill(); process.exit(0);
