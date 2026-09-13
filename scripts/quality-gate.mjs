#!/usr/bin/env node
/* HXLoLi 站点质量红线 (quality gate)
 *
 * 目的: 把散落在各处的检查命令收敛成一条命令, 让"改了 A 弄坏 B"这件事在提交前暴露,
 * 而不是等读者点开某个页面才发现.
 *
 * 覆盖的静默失效类型:
 *   1. 生成物与源文件脱钩 (侧边栏 / deck 注册表 / tag 索引 / tag 注册表) —— 源改了但没重跑生成器.
 *   2. hxid 重复、非法, 或跨文章链接在移动后失效.
 *   3. tag 不在词表内, 或 tag 注册表 generated 层过期.
 *   4. 中文标点没归一化 (全角标点混用).
 *   5. TS 类型错误新增 (存量错误用 baseline 白名单放行).
 *   6. 演示页引用指向不存在的文件, 或 .tsx 演示页没被注册进 decks.generated.ts.
 *   7. 构建期断链 —— 配置里 onBrokenLinks 是 warn, 构建不会失败, 所以必须自己抓 "Broken link".
 *   8. 侧车 .html 没被发布, 或发布成了 Docusaurus 应用外壳 (软 404).
 *      最后一条正是 dev server 上踩过的坑: .html 侧车由 webpack copy 在客户端构建时枚举,
 *      因此 dev server 启动后才新增的侧车会 404; build 产物则总是正确的. 本红线在 build 产物上断言.
 *
 * Usage:
 *   npm run gate                      # 全量 (静态检查 + 生成物新鲜度 + 类型 + 构建 + 产物断言)
 *   npm run gate -- --fast            # 跳过构建, 用于快速自检
 *   npm run gate -- --json            # 机器可读输出
 *   npm run gate -- --update-baseline # 把当前 TS 错误登记为存量 (人工确认后用)
 *
 * 退出码: 0 全绿; 1 有失败项.
 */
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';

const SITE = path.resolve(import.meta.dirname, '..');
const argv = process.argv.slice(2);
const FAST = argv.includes('--fast');
const JSON_OUT = argv.includes('--json');
// --dev[=URL]: 额外探测一个正在运行的 dev server, 专门抓"侧车 404 但页面看似正常"的软 404
const devArg = argv.find((a) => a === '--dev' || a.startsWith('--dev='));
const DEV_URL = devArg
  ? (devArg.includes('=') ? devArg.split('=').slice(1).join('=') : (process.env.GATE_DEV_URL ?? 'http://127.0.0.1:3000/HXLoLi'))
  : null;
const UPDATE_BASELINE = argv.includes('--update-baseline');
const BASELINE_FILE = path.join(SITE, 'scripts', 'quality-baseline.json');

const C = { reset: '[0m', dim: '[2m', bold: '[1m', red: '[31m', green: '[32m', yellow: '[33m', cyan: '[36m' };
const results = [];

function record(name, status, detail = '') {
  results.push({ name, status, detail });
  if (JSON_OUT) return;
  const icon = status === 'pass' ? C.green + ' PASS ' + C.reset
    : status === 'fail' ? C.red + ' FAIL ' + C.reset
    : C.yellow + ' WARN ' + C.reset;
  process.stdout.write(icon + ' ' + name + '\n');
  if (detail) process.stdout.write('       ' + C.dim + detail + C.reset + '\n');
}

function section(title) {
  if (!JSON_OUT) process.stdout.write('\n' + C.bold + C.cyan + '== ' + title + ' ==' + C.reset + '\n');
}

/** 跑一条命令; 统一返回 {code, out}, 不抛异常. */
function run(cmd, args, opts = {}) {
  const r = spawnSync(cmd, args, {
    cwd: opts.cwd ?? SITE,
    encoding: 'utf8',
    maxBuffer: 128 * 1024 * 1024,
    env: { ...process.env, ...(opts.env ?? {}) },
    shell: false,
  });
  return { code: r.status ?? (r.error ? 1 : 0), out: (r.stdout ?? '') + (r.stderr ?? ''), error: r.error };
}

const read = (rel) => fs.readFileSync(path.join(SITE, rel), 'utf8');
const write = (rel, s) => fs.writeFileSync(path.join(SITE, rel), s);
const exists = (rel) => fs.existsSync(path.join(SITE, rel));

/** 复刻 docusaurus.config.ts 的 stripNumberPrefix: 去掉 "NNN-"/"NNN_"/"NNN." 前缀. */
function stripNumberPrefix(name) {
  if (/^\d+[-_.]\d+/.test(name)) return name;
  const m = /^(\d+)\s*[-_.]+\s*([^-_.\s].*)$/.exec(name);
  return m ? m[2] : name;
}
function stripPathNumberPrefixes(p) {
  return p.split('/').map(stripNumberPrefix).join('/');
}
/** 由 ai-docs 下的目录还原出站点路由 (与构建期 copy 插件的 to 完全同构). */
function docsRouteFor(dirAbs) {
  const rel = path.relative(path.join(SITE, 'ai-docs'), dirAbs).split(path.sep).join('/');
  return stripPathNumberPrefixes(rel);
}

/** 收集 ai-docs 下所有 markdown 文件 (跳过点开头目录). */
function collectMarkdown(dir) {
  const out = [];
  for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
    if (e.name.startsWith('.')) continue;
    const full = path.join(dir, e.name);
    if (e.isDirectory()) out.push(...collectMarkdown(full));
    else if (/\.mdx?$/i.test(e.name)) out.push(full);
  }
  return out;
}

/** 已知的检查跳过项: 这些文件由 dev server / 插件运行期生成, 不参与新鲜度比较. */
const MD_FILES = collectMarkdown(path.join(SITE, 'ai-docs'));

/* ---------- 1. hxid: 唯一性 / 合法性 / 链接可达 ---------- */
section('hxid 身份与跨文章链接');
{
  const r = run('uv', ['run', '.agents/skills/hx-docs-organize/scripts/hx_docs_id.py', 'check']);
  const tail = r.out.trim().split('\n').slice(-1)[0] ?? '';
  record('hxid check (唯一 / 合法 / 链接最新)', r.code === 0 ? 'pass' : 'fail', tail);
}
{
  const r = run('uv', ['run', '.agents/skills/hx-docs-organize/scripts/hx_docs_id.py', 'links']);
  const lines = r.out.trim().split('\n');
  const summary = (lines.find((l) => l.includes('笔记')) ?? '').trim();
  const tail = (lines.slice(-1)[0] ?? '').trim();
  record('hxid links (本地引用可达)', r.code === 0 ? 'pass' : 'fail', summary + ' | ' + tail);
}

/* ---------- 2. tag 词表与注册表 ---------- */
section('tag 规范与注册表');
{
  const r = run('uv', ['run', '.agents/skills/hx-docs-layout/scripts/hxloli_tags.py', 'check']);
  const tail = r.out.trim().split('\n').slice(-1)[0] ?? '';
  record('tag 词表合规', r.code === 0 ? 'pass' : 'fail', tail);
}
{
  // 裸 hxid 链接 [标题](hxid:hx-xxxxxxxx) 在 Markdown 里合法, 但渲染出来是 href="hxid:...",
  // 浏览器无法跳转 (前端也没有任何拦截器)。只有 resolve 之后的 tagged 形态
  // [标题](路径 "hxid:hx-xxxxxxxx") 才同时携带可渲染路径与持久身份。
  const bare = [];
  for (const full of MD_FILES) {
    const rel = path.relative(SITE, full);
    read(rel).split('\n').forEach((line, i) => {
      for (const m of line.matchAll(/\]\(\s*hxid:(hx-[0-9a-f]{8})\s*\)/g)) {
        bare.push(rel + ':' + (i + 1) + ' (' + m[1] + ')');
      }
    });
  }
  record('hxid 链接已 resolve 为可跳转形态', bare.length === 0 ? 'pass' : 'fail',
    bare.length === 0 ? '无裸 hxid: 链接'
      : bare.length + ' 条裸链接渲染后无法跳转: ' + bare.slice(0, 3).join(', ')
        + '  ==> 运行: uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py resolve --write');
}

{
  const rel = 'ai-docs/.hx-tags.toml';
  const before = read(rel);
  const r = run('uv', ['run', '.agents/skills/hx-docs-layout/scripts/hxloli_tags.py', 'generate']);
  const after = read(rel);
  write(rel, before); // 保持检查只读
  // generated_at 是时间戳, 不参与新鲜度比较
  const strip = (s) => s.replace(/^generated_at = ".*"$/m, 'generated_at = "<ts>"');
  if (r.code !== 0) record('tag 注册表 generated 层新鲜', 'fail', 'generate 执行失败');
  else if (strip(before) !== strip(after)) record('tag 注册表 generated 层新鲜', 'fail', '已过期, 运行: uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py generate');
  else record('tag 注册表 generated 层新鲜', 'pass', 'generate 无差异');
}

/* ---------- 3. 中文标点归一化 ---------- */
section('中文标点归一化');
{
  const files = MD_FILES.map((f) => path.relative(SITE, f));
  const r = run('uv', ['run', '.agents/skills/hx-docs-layout/scripts/format_cn_punct.py', '--check', ...files]);
  const needs = r.out.split('\n').filter((l) => l.includes('needs format'))
    .map((l) => l.trim().replace(/^.*needs format:\s*/, ''));
  record('中文标点归一化 (' + files.length + ' 篇)', r.code === 0 ? 'pass' : 'fail',
    r.code === 0 ? '全部已规范'
      : '需格式化: ' + needs.slice(0, 5).join(', ') + (needs.length > 5 ? ' 等 ' + needs.length + ' 篇' : ''));
}

/* ---------- 4. 生成物新鲜度 ---------- */
section('生成物与源文件是否脱钩');
function freshness(name, rel, fixCmd, fixArgs, fixHint, normalize = (s) => s) {
  if (!exists(rel)) return record(name, 'fail', '缺少生成物 ' + rel);
  const before = read(rel);
  const r = run(fixCmd, fixArgs);
  let after = null;
  try { after = read(rel); } catch { after = null; }
  write(rel, before); // 回滚, 保持检查只读
  if (r.code !== 0) return record(name, 'fail', '生成器执行失败: ' + fixCmd + ' ' + fixArgs.join(' '));
  if (after === null) return record(name, 'fail', '生成器未产出 ' + rel);
  if (normalize(before) !== normalize(after)) return record(name, 'fail', '已过期, 运行: ' + fixHint);
  record(name, 'pass', '重新生成无差异');
}
freshness('侧边栏与 ai-docs 同步', 'sidebarsAiDocs.ts', 'node', ['scripts/generateAiDocsSidebar.js'],
  'node scripts/generateAiDocsSidebar.js');
freshness('deck 注册表与 .tsx 演示页同步', 'src/hxdeck/decks.generated.ts', 'node', ['scripts/generate-deck-registry.mjs'],
  'npm run decks');
freshness('tag 索引 (data/aiDocTags.ts) 新鲜', 'data/aiDocTags.ts', 'node', ['scripts/regenerate-tag-index.mjs'],
  'node scripts/regenerate-tag-index.mjs',
  // generatedAt 是时间戳, 不参与新鲜度比较
  (s) => s.replace(/"generatedAt":\s*"[^"]*"/, '"generatedAt": "<ts>"'));

/* ---------- 5. 演示页引用完整性 ---------- */
section('演示页引用完整性');
function scanPptRefs() {
  const refs = [];
  for (const full of MD_FILES) {
    const rel = path.relative(SITE, full);
    for (const line of read(rel).split('\n')) {
      if (!line.includes('#ppt') && !/##PPT\b/.test(line)) continue;
      const m = line.match(/\]\(([^)]+\.(?:html|tsx))\)/);
      if (!m) continue;
      refs.push({ rel, src: m[1], abs: path.resolve(path.dirname(full), decodeURIComponent(m[1])) });
    }
  }
  return refs;
}
{
  const decks = exists('src/hxdeck/decks.generated.ts') ? read('src/hxdeck/decks.generated.ts') : '';
  let html = 0, tsx = 0; const bad = [];
  for (const r of scanPptRefs()) {
    if (!fs.existsSync(r.abs)) { bad.push(r.rel + ' -> ' + r.src + ' (文件不存在)'); continue; }
    if (r.src.endsWith('.html')) { html++; continue; }
    tsx++;
    const key = path.relative(path.join(SITE, 'ai-docs'), r.abs).split(path.sep).join('/');
    if (!decks.includes(key)) bad.push(r.rel + ' -> ' + r.src + ' (未注册进 decks.generated.ts)');
  }
  record('演示页引用可达 (' + html + ' 个 .html 侧车 / ' + tsx + ' 个 .tsx 演示页)',
    bad.length === 0 ? 'pass' : 'fail', bad.length ? bad.slice(0, 5).join('; ') : '全部可达');
}

/* ---------- 6. TypeScript 类型检查 (带存量基线) ---------- */
section('TypeScript 类型检查');
{
  const r = run('npx', ['tsc', '--noEmit']);
  const errors = r.out.split('\n').filter((l) => /error TS\d+/.test(l));
  // 归一成 "文件|TS码": 同一文件同一码出现多次也计数
  const sig = (l) => { const m = l.match(/^([^(]+)\(\d+,\d+\): error (TS\d+)/); return m ? m[1] + '|' + m[2] : l.trim(); };
  const current = errors.map(sig).sort();
  let baseline = { knownErrors: [] };
  if (fs.existsSync(BASELINE_FILE)) {
    try { baseline = JSON.parse(fs.readFileSync(BASELINE_FILE, 'utf8')); } catch { /* 坏基线当空处理 */ }
  }
  if (UPDATE_BASELINE) {
    fs.writeFileSync(BASELINE_FILE, JSON.stringify({ knownErrors: current }, null, 2) + '\n');
    record('TS 类型检查', 'warn', '已把 ' + current.length + ' 条存量错误写入 scripts/quality-baseline.json');
  } else {
    const counts = (arr) => arr.reduce((m, k) => m.set(k, (m.get(k) ?? 0) + 1), new Map());
    const cCur = counts(current), cKnown = counts(baseline.knownErrors ?? []);
    const added = [], fixed = [];
    for (const [k, n] of cCur) { for (let i = n - (cKnown.get(k) ?? 0); i > 0; i--) added.push(k); }
    for (const [k, n] of cKnown) { for (let i = n - (cCur.get(k) ?? 0); i > 0; i--) fixed.push(k); }
    if (added.length) record('TS 类型检查 (无新增错误)', 'fail',
      '新增 ' + added.length + ' 条: ' + [...new Set(added)].slice(0, 4).join(', '));
    else if (fixed.length) record('TS 类型检查 (无新增错误)', 'warn',
      '存量错误少 ' + fixed.length + ' 条, 可运行 --update-baseline 收紧: ' + [...new Set(fixed)].slice(0, 4).join(', '));
    else record('TS 类型检查 (无新增错误)', 'pass', current.length + ' 条存量错误 (与基线一致)');
  }
}

/* ---------- 7. 构建 + 断链 + 侧车产物 ---------- */
if (FAST) {
  section('构建期检查');
  record('构建 / 断链 / 侧车产物', 'warn', '--fast: 已跳过 (发布前务必跑一次全量 npm run gate)');
} else {
  section('构建期检查 (断链 / 侧车产物)');
  fs.rmSync(path.join(SITE, 'build'), { recursive: true, force: true });
  const r = run('npm', ['run', 'build']);
  if (r.code !== 0) {
    const t = r.out.trim().split('\n').slice(-15).join('\n       ');
    record('docusaurus build', 'fail', '构建失败 (exit ' + r.code + '):\n       ' + t);
  } else {
    record('docusaurus build', 'pass', '构建成功');
  }
  // onBrokenLinks 是 "warn": 构建成功也可能带断链, 必须自己抓
  const broken = r.out.split('\n').filter((l) => /Broken (Markdown )?link/i.test(l));
  record('构建产物无断链 (onBrokenLinks 仅 warn)', broken.length === 0 ? 'pass' : 'fail',
    broken.length === 0 ? '未发现 Broken link 告警'
      : broken.length + ' 条断链: ' + broken.slice(0, 3).map((s) => s.trim()).join(' | '));

  // 侧车 .html 是否真的发布成了静态文件 (而不是 Docusaurus 应用外壳 / 软 404)
  const missing = [];
  for (const ref of scanPptRefs()) {
    if (!ref.src.endsWith('.html')) continue;
    const route = docsRouteFor(path.dirname(ref.abs));
    const base = path.basename(ref.abs);
    const candidates = [
      path.join(SITE, 'build', 'knowledge-base', route, base),
      path.join(SITE, 'build', 'knowledge-base', route, base.replace(/\.html?$/i, ''), 'index.html'),
    ];
    const hit = candidates.find((c) => fs.existsSync(c));
    if (!hit) { missing.push(ref.src + ' (未发布)'); continue; }
    const body = fs.readFileSync(hit, 'utf8');
    // 站点外壳只有几 KB 且带 plugin-pages 类名; 真实侧车是自包含的大文件
    if (/plugin-pages/.test(body) || body.length < 4096) {
      missing.push(ref.src + ' (发布成了站点外壳 ' + body.length + 'B —— 侧车内容丢失)');
    }
  }
  record('侧车 .html 已发布为真实静态文件', missing.length === 0 ? 'pass' : 'fail',
    missing.length ? missing.join('; ') : '全部命中且非站点外壳');
}

/* ---------- 8. (可选) 运行中的 dev server 侧车探测 ---------- */
if (DEV_URL) {
  section('dev server 侧车探测');
  const refs = scanPptRefs().filter((r) => r.src.endsWith('.html'));
  const bad = [];
  let checked = 0;
  for (const ref of refs) {
    const route = docsRouteFor(path.dirname(ref.abs));
    const url = DEV_URL.replace(/\/$/, '') + '/knowledge-base/' + route.split('/').map(encodeURIComponent).join('/') + '/' + encodeURIComponent(path.basename(ref.abs));
    try {
      const res = await fetch(url, { redirect: 'follow' });
      const body = await res.text();
      checked++;
      // dev server 上未发布的侧车会被 historyApiFallback 兜成应用外壳: 200 + 几 KB, 看起来"没报错"
      if (/plugin-pages/.test(body) || body.length < 4096) {
        bad.push(path.basename(ref.abs) + ' (软 404: ' + body.length + 'B 应用外壳)');
      }
    } catch (e) {
      bad.push(path.basename(ref.abs) + ' (请求失败: ' + String(e).slice(0, 60) + ')');
    }
  }
  record('dev server 侧车可加载 (' + checked + '/' + refs.length + ' 个)',
    bad.length === 0 ? 'pass' : 'fail',
    bad.length
      ? bad.slice(0, 5).join('; ') + '  ==> dev server 启动后才新增的 .html 侧车不会被 webpack copy 枚举, 重启 dev server 即可'
      : '全部返回真实侧车内容');
}

/* ---------- 汇总 ---------- */
const fails = results.filter((x) => x.status === 'fail');
const warns = results.filter((x) => x.status === 'warn');
if (JSON_OUT) {
  console.log(JSON.stringify({ ok: fails.length === 0, results }, null, 2));
} else {
  process.stdout.write('\n' + C.bold + '== 汇总 ==' + C.reset + '\n');
  process.stdout.write('  ' + C.green + (results.length - fails.length - warns.length) + ' 通过' + C.reset);
  if (warns.length) process.stdout.write('  ' + C.yellow + warns.length + ' 警告' + C.reset);
  if (fails.length) process.stdout.write('  ' + C.red + fails.length + ' 失败' + C.reset);
  process.stdout.write('\n');
  if (fails.length) {
    process.stdout.write('\n' + C.red + C.bold + '红线未通过:' + C.reset + '\n');
    for (const f of fails) process.stdout.write('  ' + C.red + 'x' + C.reset + ' ' + f.name + '\n      ' + C.dim + f.detail + C.reset + '\n');
  } else {
    process.stdout.write('\n' + C.green + 'OK 全部检查通过, 可以提交.' + C.reset + '\n');
  }
}
process.exit(fails.length ? 1 : 0);
