// End-to-end browser check (not run by `node --test`): HW_ASSETS=/path/to/original/assets, serve web/ on :8011 with a symlink web/_assets -> $HW_ASSETS,
// then `node web/e2e_assets.mjs OUTDIR`; needs desktop Chrome with WebGPU. Writes OUTDIR/web_assets.png and prints the page status line.
import { spawn } from 'node:child_process';
import { readdirSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const root = process.env.HW_ASSETS;
const files = [];
(function walk(d) { for (const f of readdirSync(d)) { const p = join(d, f); statSync(p).isDirectory() ? walk(p) : files.push(p); } })(root);
const ch = spawn('/Applications/Google Chrome.app/Contents/MacOS/Google Chrome', ['--headless=new', '--enable-unsafe-webgpu', '--use-angle=metal', '--remote-debugging-port=9333', '--window-size=1000,900', '--user-data-dir=' + process.argv[2] + '/chrome-prof', 'about:blank'], { stdio: 'ignore' });
const sleep = ms => new Promise(r => setTimeout(r, ms));
let targets;
for (let i = 0; i < 40; i++) { try { targets = await (await fetch('http://127.0.0.1:9333/json')).json(); break; } catch { await sleep(250); } }
const ws = new WebSocket(targets.find(t => t.type === 'page').webSocketDebuggerUrl);
await new Promise(r => ws.addEventListener('open', r, { once: true }));
let id = 0; const pend = new Map();
ws.addEventListener('message', ({ data }) => { const m = JSON.parse(data); if (m.id && pend.has(m.id)) { pend.get(m.id)(m); pend.delete(m.id); } else if (m.method === 'Runtime.consoleAPICalled' || m.method === 'Runtime.exceptionThrown') console.log('PAGE', JSON.stringify(m.params).slice(0, 300)); });
const send = (method, params = {}) => new Promise(r => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
await send('Runtime.enable'); await send('Page.enable'); await send('DOM.enable');
await send('Page.navigate', { url: 'http://localhost:8011/' });
await sleep(4000);
const files2 = files.map(f => f.slice(root.length + 1));
const expr = `(async()=>{const list=${JSON.stringify(files2)};const dt=new DataTransfer();for(const p of list){if(!/\\.(json|png|otf|mp3)$/.test(p))continue;const r=await fetch('/_assets/'+p);const f=new File([await r.blob()],p.split('/').pop());Object.defineProperty(f,'webkitRelativePath',{value:'assets/'+p});dt.items.add(f);}const el=document.querySelector('#assets');el.files=dt.files;el.dispatchEvent(new Event('change',{bubbles:true}));return dt.files.length})()`;
const rr = await send('Runtime.evaluate', { expression: expr, awaitPromise: true });
console.log('INJECTED', JSON.stringify(rr.result.result));
await sleep(15000);
const st = await send('Runtime.evaluate', { expression: 'document.querySelector("#status").textContent' });
console.log('STATUS', st.result.result.value);
const shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(process.argv[2] + '/web_assets.png', Buffer.from(shot.result.data, 'base64'));
ch.kill(); process.exit(0);
