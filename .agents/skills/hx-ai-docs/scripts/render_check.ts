// 打开笔记页, 报渲染错误, 把每张图和组件截成 PNG
import { spawn } from 'node:child_process';
import fs from 'node:fs';

const [url, prefix, theme = 'dark'] = process.argv.slice(2);
if (!url || !prefix) { console.error('usage: node render_check.ts <url> <prefix> [dark|light]'); process.exit(2); }
const chromePath = `${process.env.HOME}/.cache/ms-playwright/chromium-1255/chrome-linux64/chrome`;
const port = 9300 + Math.floor(Math.random() * 500);
fs.mkdirSync('/dev/shm/c', { recursive: true });
const chrome = spawn(chromePath, ['--headless=new', '--no-sandbox', '--disable-gpu', '--hide-scrollbars',
  `--remote-debugging-port=${port}`, `--user-data-dir=/dev/shm/c/rc${Date.now()}`, '--window-size=1280,1000', 'about:blank'],
  { env: { ...process.env, TMPDIR: '/dev/shm/c' }, stdio: 'ignore' });
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

let wsUrl = '';
for (let i = 0; i < 50 && !wsUrl; i++) {
  try { const list = await (await fetch(`http://127.0.0.1:${port}/json`)).json(); wsUrl = list.find((p: any) => p.type === 'page')?.webSocketDebuggerUrl ?? ''; } catch { /* chrome 还没起来 */ }
  if (!wsUrl) await sleep(200);
}
const sock = new WebSocket(wsUrl);
await new Promise((r) => { sock.onopen = r; });
let seq = 0;
const pending = new Map<number, (v: any) => void>();
const logs: string[] = [];
sock.onmessage = (m) => {
  const d = JSON.parse(String(m.data));
  if (d.id && pending.has(d.id)) { pending.get(d.id)!(d.result); pending.delete(d.id); }
  if (d.method === 'Runtime.exceptionThrown') logs.push(d.params.exceptionDetails.text);
  if (d.method === 'Runtime.consoleAPICalled' && d.params.type === 'error') logs.push(d.params.args.map((a: any) => a.value ?? a.description).join(' ').slice(0, 300));
};
const send = (method: string, params = {}) => new Promise<any>((r) => { const id = ++seq; pending.set(id, r); sock.send(JSON.stringify({ id, method, params })); });
const evaluate = async (expression: string) => (await send('Runtime.evaluate', { expression, returnByValue: true })).result.value;

await send('Page.enable'); await send('Runtime.enable');
await send('Page.navigate', { url }); await sleep(8000);
await evaluate(`localStorage.setItem('theme','${theme}');document.documentElement.setAttribute('data-theme','${theme}')`);
await sleep(800);
const page = JSON.parse(await evaluate(`JSON.stringify({
  h1: document.querySelector('h1')?.textContent ?? null,
  boundary: /This page crashed|Try again|Page Not Found/.test(document.body.innerText),
  mermaid: [...document.querySelectorAll('svg[aria-roledescription=error], .mermaid')].filter(e => /Syntax error|error-text|Parse error/.test(e.outerHTML)).length,
  rects: [...document.querySelectorAll('figure, .hxkb-panel')].map(e => { const b = e.getBoundingClientRect(); return [b.x, b.y + scrollY, b.width, b.height]; }),
})`));
let n = 0;
for (const [x, y, width, height] of page.rects) {
  if (width < 50 || height < 30) continue;
  const shot = await send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true, clip: { x, y, width, height, scale: 1 } });
  fs.writeFileSync(`${prefix}-${n++}.png`, Buffer.from(shot.data, 'base64'));
}
const errors = [...(page.boundary ? ['error boundary'] : []), ...(page.mermaid ? [`mermaid x${page.mermaid}`] : []), ...logs];
console.log(JSON.stringify({ h1: page.h1, shots: n, errors }));
chrome.kill(); process.exit(errors.length ? 1 : 0);
