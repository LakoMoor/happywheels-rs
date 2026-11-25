// Browser check of the 60 fps option: serve web/ on :8011, `node web/e2e_fps60.mjs OUTDIR` ticks the checkbox, loads a level and holds ArrowRight;
// writes OUTDIR/web_fps60.png and prints the status line and whether the box stayed checked (needs desktop Chrome with WebGPU).
// End-to-end browser check (not run by `node --test`): HW_ASSETS=/path/to/original/assets, serve web/ on :8011 with a symlink web/_assets -> $HW_ASSETS,
// then `node web/e2e_assets.mjs OUTDIR`; needs desktop Chrome with WebGPU. Writes OUTDIR/web_fps60.png and prints the page status line.
import { spawn } from 'node:child_process';
import { readdirSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
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
await send('Runtime.evaluate',{expression:`(async()=>{document.querySelector('#fps60').click();const dt=new DataTransfer();dt.items.add(new File(["<levelXML><info v='1.99' x='1000' y='300'/><shapes><sh t='0' p0='1000' p1='450' p2='3000' p3='50' p4='0' p5='t' p6='f' p7='1' p8='0' p9='-1' p10='100' p11='1'/></shapes><specials><sp t='6' p0='1400' p1='420' p2='0' p3='t' p4='8' p5='f'/></specials></levelXML>"],'l.xml'));const i=document.querySelector('#level');i.files=dt.files;i.dispatchEvent(new Event('change'));return 1})()`,awaitPromise:true});
await send('Input.dispatchKeyEvent',{type:'rawKeyDown',key:'ArrowRight',code:'ArrowRight',windowsVirtualKeyCode:39});
await sleep(7000);
console.log('STATUS', (await send('Runtime.evaluate',{expression:'document.querySelector("#status").textContent'})).result.result.value);

const st = await send('Runtime.evaluate', { expression: 'document.querySelector("#status").textContent' });
console.log('STATUS', st.result.result.value, 'CHECKED', (await send('Runtime.evaluate',{expression:'document.querySelector("#fps60").checked + " " + localStorage.getItem("hw-fps60")'})).result.result.value);
const shot = await send('Page.captureScreenshot', { format: 'png' });
writeFileSync(process.argv[2] + '/web_fps60.png', Buffer.from(shot.result.data, 'base64'));
ch.kill(); process.exit(0);
