#!/usr/bin/env node
/**
 * dev-edit-server.mjs — 本地所见即所得 (逐块内联编辑) + VS Code 跳转 + .hx-mitemite.md
 *
 * 仅本地开发环境使用 (run.sh 启动), 构建产物不含任何痕迹.
 * 端口: 3310
 *
 * API:
 *   GET  /health                          -> { ok: true }
 *   GET  /api/file?rel=...                -> 读取站点内文件
 *   GET  /api/blocks?rel=...              -> remark 解析出正文块 [{id,kind,start,end,text}]
 *   POST /api/block  {rel,id,text}        -> 用 text 替换 id 对应源码行, 原子写回
 *   POST /api/file   {rel,content}        -> 整体原子写 (兼容)
 *   GET  /api/mitemite?rel=...            -> 读取 .hx-mitemite.md
 *   POST /api/mitemite {rel,content}      -> 写入 .hx-mitemite.md
 *   POST /api/mitemite/answer {rel,seq,hash,answer} -> 只更新某 block 的答案 (题目不可改)
 *   GET  /api/open-in-vscode?rel=...[:line] -> 打开 vscode -g
 *   OPTIONS                               -> CORS
 */
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { spawn, exec } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { unified } from 'unified';
import remarkParse from 'remark-parse';
import remarkGfm from 'remark-gfm';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const PORT = Number(process.env.HX_EDIT_PORT || 3310);

const CORS_HEADERS = {
  'Access-Control-Allow-Origin': '*',
  'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
  'Access-Control-Allow-Headers': 'Content-Type',
  'Access-Control-Max-Age': '86400',
};

function safeResolve(siteRel) {
  if (typeof siteRel !== 'string' || siteRel.length === 0) return null;
  const clean = siteRel.replace(/^\/+/, '').replace(/\\/g, '/');
  const abs = path.resolve(ROOT, clean);
  const rel = path.relative(ROOT, abs);
  if (rel.startsWith('..') || path.isAbsolute(rel)) return null;
  return abs;
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    let data = '';
    req.on('data', (c) => { data += c; if (data.length > 10 * 1024 * 1024) { reject(new Error('body too large')); req.destroy(); } });
    req.on('end', () => resolve(data));
    req.on('error', reject);
  });
}

function json(res, code, obj) {
  res.writeHead(code, { 'Content-Type': 'application/json; charset=utf-8', ...CORS_HEADERS });
  res.end(JSON.stringify(obj));
}

/** mdast 节点 kind -> 人类可读 kind */
function kindOf(type) {
  switch (type) {
    case 'heading': return 'heading';
    case 'paragraph': return 'paragraph';
    case 'list': return 'list';
    case 'blockquote': return 'blockquote';
    case 'table': return 'table';
    case 'code': return 'code';
    case 'mermaid': return 'code'; // remark 视作 code
    case 'thematicBreak': return 'rule';
    case 'html': return 'html';
    default: return type;
  }
}

/** 提取块正文 (供前端展示/匹配) */
function blockText(node, lines) {
  // 简单的纯文本: 从源码行提取并去 md 记号太复杂, 这里返回源码原文
  const s = node.position?.start?.line ?? 1;
  const e = node.position?.end?.line ?? s;
  const src = lines.slice(s - 1, e).join('\n');
  return src;
}

function parseMitemiteBlocks(content) {
  // 解析 .hx-mitemite.md: ## 0xNN hash begin { ... } 中的 **Q**: / **A**: (行级解析, 稳)
  const out = [];
  const re = /^##\s+(0x[0-9A-Fa-f]{2})\s+([0-9a-f]{8})\s+begin\s+\{$/gm;
  let m;
  const starts = [];
  while ((m = re.exec(content)) !== null) starts.push({ seq: m[1], hash: m[2], idx: m.index });
  const lines = content.split('\n');
  const lineOf = (pos) => content.slice(0, pos).split('\n').length;
  for (let k = 0; k < starts.length; k++) {
    const s = starts[k];
    const endIdx = k + 1 < starts.length ? starts[k + 1].idx : content.length;
    const body = content.slice(s.idx, endIdx);
    const bodyLines = body.split('\n');
    let qStart = -1, aStart = -1, closeLine = -1;
    for (let i = 0; i < bodyLines.length; i++) {
      const t = bodyLines[i].trim();
      if (t === '**Q**:' && qStart < 0) qStart = i;
      else if (t === '**A**:' && aStart < 0) aStart = i;
      else if (t === '}' && closeLine < 0 && qStart >= 0 && aStart >= 0) { closeLine = i; break; }
    }
    if (qStart < 0 || aStart < 0) continue;
    const qText = bodyLines.slice(qStart + 1, aStart).join('\n').trim();
    const close = closeLine >= 0 ? closeLine : bodyLines.length;
    const aText = bodyLines.slice(aStart + 1, close).join('\n').trim();
    out.push({
      seq: s.seq, hash: s.hash, q: qText, a: aText,
      startLine: lineOf(s.idx),
      qLine: lineOf(s.idx) + qStart + 1,
      aLine: lineOf(s.idx) + aStart + 1,
    });
  }
  return out;
}

/** 将 VS Code 窗口提到前台 (KWin 6 Wayland 实测方案):
 *  KWin 6 已移除 client.activate(); 唯一可用的是 workspace.activeWindow = win,
 *  且该赋值受 KWin 焦点窃取防护(FocusStealingPrevention)限制 -> 需先写 kwinrc 置 None 并 reconfigure.
 *  回退: xdotool (XWayland 窗口) -> notify-send 确认提示.
 */
function focusVSCode(line, col) {
  // 0) 确保焦点窃取防护关闭 (KWin Wayland 下 workspace.activeWindow= 赋值必需; 幂等)
  try {
    exec('kwriteconfig6 --file kwinrc --group Windows --key FocusStealingPrevention None 2>/dev/null; qdbus6 org.kde.KWin /KWin org.kde.KWin.reconfigure >/dev/null 2>&1; true');
  } catch (e) {}
  // 1) KWin 原生 Wayland: workspace.activeWindow = 目标窗口 (KWin 6 唯一可用赋值; 脚本内重试等待窗口出现)
  try {
    const tmpDir = process.env.TMPDIR || '/tmp';
    const tmp = path.join(tmpDir, 'hx-kwin-activate-code.js');
    const script = [
      'var t0 = Date.now();',
      'var done = false;',
      'function tryActivate() {',
      '  if (done) return;',
      '  const clients = workspace.windowList();',
      '  for (const c of clients) {',
      '    const cls = String(c.resourceClass || "").toLowerCase();',
      '    const nm = String(c.caption || "");',
      '    if (cls.indexOf("code") !== -1 || cls.indexOf("vscode") !== -1 || nm.indexOf("Visual Studio Code") !== -1) {',
      '      try { workspace.activeWindow = c; console.log("HX-KWIN-ACTIVATED " + nm); } catch (e) { console.log("HX-KWIN-ERR " + e); }',
      '      done = true;',
      '      return;',
      '    }',
      '  }',
      '  if (Date.now() - t0 < 5000) { setTimeout(tryActivate, 250); }',
      '}',
      'tryActivate();',
      ''
    ].join('\n');
    fs.writeFileSync(tmp, script, 'utf8');
    exec('qdbus6 org.kde.KWin /Scripting org.kde.kwin.Scripting.loadScript ' + tmp + ' >/dev/null 2>&1; sleep 0.3; qdbus6 org.kde.KWin /Scripting org.kde.kwin.Scripting.start >/dev/null 2>&1; true');
  } catch (e) {}
  // 2) X11/XWayland 回退: xdotool
  try {
    exec('for i in 1 2 3 4 5; do /usr/bin/xdotool search --onlyvisible --class code windowactivate 2>/dev/null && break; /usr/bin/xdotool search --onlyvisible --name Code windowactivate 2>/dev/null && break; sleep 0.4; done; true');
  } catch (e) {}
  // 3) 桌面通知确认 (无论焦点是否成功, 用户都能看到跳转发生)
  try {
    const loc = line ? ' 第' + line + '行' + (col ? '第' + col + '列' : '') : '';
    exec('/usr/bin/notify-send "已跳转到 VS Code' + loc + '" "index.md 已定位, 窗口应已弹出" 2>/dev/null; true');
  } catch (e) {}
}

function md5(s) {
  return crypto.createHash('md5').update(String(s)).digest('hex').slice(0, 8);
}

/** 解析 markdown -> 可编辑块列表 */
function parseBlocks(md) {
  const lines = md.split('\n');
  const tree = unified().use(remarkParse).use(remarkGfm).parse(md);
  const blocks = [];
  const seen = new Set();
  const h1Idx = md.split('\n').findIndex(l => l.startsWith('# ')) + 1;
  for (const node of tree.children) {
    const type = node.type;
    // 跳过 frontmatter (---) 与图片等无正文块
    if (type === 'yaml') continue;
    if (type === 'thematicBreak' && node.position.start.line === 1) continue; // 开头 ---
    // yaml 前置标识: remark 把 --- ... --- 读成 heading, 跳过 H1 之前的所有 heading
    if (type === 'heading' && node.position.start.line < h1Idx) continue;
    const s = node.position?.start?.line ?? 1;
    const e = node.position?.end?.line ?? s;
    if (!(s >= 1 && e >= s)) continue;
    // 文本内容 (文本节点合并)
    const textNodes = [];
    (function walk(n) {
      if (n.type === 'text' || n.type === 'inlineCode') textNodes.push(n.value);
      if (n.children) for (const ch of n.children) walk(ch);
    })(node);
    const text = textNodes.join('');
    const src = lines.slice(s - 1, e).join('\n');
    const kind = kindOf(type);
    // 对代码块/表格等保留原文, 普通段落给纯文本
    const id = md5(type + ':' + s + ':' + e + ':' + (text || src).slice(0, 40));
    if (seen.has(id)) continue;
    seen.add(id);
    blocks.push({ id, kind, start: s, end: e, text: (text || src).slice(0, 2000), source: src.slice(0, 4000) });
  }
  return { blocks, lineCount: lines.length };
}


/* ============================================================
   AI 知识库侧边栏重建 (仅本地开发)
   GET  /api/sidebar/current            -> { ok, tree }        当前 sidebarsAiDocs.ts 树
   POST /api/sidebar/rebuild            -> { ok, diff, tree }  重扫 ai-docs/ 并重写 sidebarsAiDocs.ts
   与 scripts/generateAiDocsSidebar.js 的扫描规则保持一致.
   ============================================================ */

function stripPrefix (name) {
  return String(name).replace(/^\d+[-_]/, '');
}

/** 读取 sidebarsAiDocs.ts 里 aiDocsSidebar 数组 */
function parseCurrentSidebarTree () {
  const file = path.join(ROOT, 'sidebarsAiDocs.ts');
  const src = fs.readFileSync(file, 'utf8');
  const m = src.match(/const aiDocsSidebar = ([\s\S]*?);\n\nexport/);
  if (!m) throw new Error('sidebarsAiDocs.ts: aiDocsSidebar 数组解析失败');
  return JSON.parse(m[1]);
}

/** tag.json 图标/标签 */
function getJsonTagConfig (folderPath) {
  const tagJsonPath = path.join(folderPath, 'tag.json');
  if (fs.existsSync(tagJsonPath)) {
    try {
      const data = JSON.parse(fs.readFileSync(tagJsonPath, 'utf8'));
      return {
        icon: data.icon ? 'icons/' + data.icon : undefined,
        tags: Array.isArray(data.tags) ? data.tags : [],
      };
    } catch (err) {
      console.error('[tag.json] 无法解析:', err);
    }
  }
  return { icon: undefined, tags: [] };
}

const defaultFolderIcon = 'default-icons/ai-folder.svg';
const defaultDocIcon = 'default-icons/ai-doc.svg';

/** 扫描 ai-docs/ 目录生成侧边栏树 (与 generateAiDocsSidebar.js 相同结构) */
function scanAiDocs (dir, relativePath = '') {
  if (!fs.existsSync(dir)) return { items: [], hasIndex: false };
  const entries = fs.readdirSync(dir);
  const items = [];
  let hasIndex = false;

  for (const entry of entries) {
    if (entry.startsWith('.')) continue;
    const fullPath = path.join(dir, entry);
    let stat;
    try { stat = fs.statSync(fullPath); } catch { continue; }

    if (stat.isDirectory()) {
      const cleanLabel = stripPrefix(entry);
      const folderRelative = path.join(relativePath, entry);
      const result = scanAiDocs(fullPath, folderRelative);
      const { icon: tagIcon, tags = [] } = getJsonTagConfig(fullPath);
      const icon = tagIcon ?? (result.items.length > 0 ? defaultFolderIcon : defaultDocIcon);

      if (result.hasIndex) {
        const id = path.posix.join(...folderRelative.split(path.sep).map(stripPrefix), 'index');
        if (result.items.length === 0) {
          items.push({ type: 'doc', id: id.replace(/\\/g, '/'), label: cleanLabel, customProps: { icon: tagIcon ?? defaultDocIcon, tags } });
          continue;
        }
        items.push({
          type: 'category', label: cleanLabel, collapsible: true, collapsed: true,
          items: result.items, customProps: { icon, tags },
          link: { type: 'doc', id: id.replace(/\\/g, '/') },
        });
        continue;
      }

      if (result.items.length > 0 || result.hasIndex) {
        items.push({
          type: 'category', label: cleanLabel, collapsible: true, collapsed: true,
          items: result.items, customProps: { icon, tags },
        });
      }
    } else if (entry === 'index.md' || entry === 'index.mdx') {
      hasIndex = true;
    } else if (entry.endsWith('.md') || entry.endsWith('.mdx')) {
      const id = path.posix.join(...relativePath.split(path.sep).map(stripPrefix), stripPrefix(entry.replace(/\.mdx?$/, '')));
      items.push({ type: 'doc', id, customProps: { icon: defaultDocIcon, tags: [] } });
    }
  }

  return { items, hasIndex };
}

/** 展平树 -> Map<fullPath, node>: doc 用 id; category 用 label 链 (前缀 cat:) */
function flattenSidebar (items, prefix = '') {
  const map = new Map();
  for (const it of items) {
    if (it.type === 'category') {
      const clean = stripPrefix(it.label);
      const key = (prefix ? prefix + '/' : '') + clean;
      map.set('cat:' + key, it);
      if (Array.isArray(it.items)) {
        const sub = flattenSidebar(it.items, key);
        for (const [k, v] of sub) map.set(k, v);
      }
    } else if (it.type === 'doc') {
      let key = String(it.id || it.label || '').replace(/\.mdx?$/, '');
      // 去掉尾部 /index, 与前端 href 对齐 (index.md 路由不带 /index)
      if (key.endsWith('/index')) key = key.slice(0, -'/index'.length);
      map.set('doc:' + key, it);
    } else {
      map.set('item:' + (prefix ? prefix + '/' : '') + stripPrefix(String(it.label || it.id)), it);
    }
  }
  return map;
}

/** diff 两棵树: added/changed/removed = 完整 key 数组 (去掉类型前缀, 保留层级路径) */
function diffSidebarTrees (oldTree, newTree) {
  const oldMap = flattenSidebar(oldTree || []);
  const newMap = flattenSidebar(newTree || []);
  const sig = (n) => JSON.stringify({ l: n.label, c: n.collapsed, custom: n.customProps || null, sub: Array.isArray(n.items) ? n.items.map((i) => i.type + ':' + String(i.id || i.label)) : null });
  const bare = (key) => key.replace(/^(cat|doc|item):/, '');
  const added = [], changed = [], removed = [];
  for (const [key, node] of newMap) {
    if (!oldMap.has(key)) added.push(bare(key));
    else if (sig(oldMap.get(key)) !== sig(node)) changed.push(bare(key));
  }
  for (const key of oldMap.keys()) if (!newMap.has(key)) removed.push(bare(key));
  return { added, changed, removed, hasChanges: added.length > 0 || changed.length > 0 || removed.length > 0 };
}

/** 将树写回 sidebarsAiDocs.ts (原子) */
function writeSidebarTree (tree) {
  const file = path.join(ROOT, 'sidebarsAiDocs.ts');
  const content = '// 由 scripts/generateAiDocsSidebar.js 自动生成\n// 请勿手动编辑 — 运行 node scripts/generateAiDocsSidebar.js 以更新\n\nconst aiDocsSidebar = ' + JSON.stringify(tree, null, 2) + ';\n\nexport default { aiDocsSidebar };\n';
  const tmp = file + '.hx-tmp-' + process.pid;
  fs.writeFileSync(tmp, content, 'utf8');
  fs.renameSync(tmp, file);
  return content;
}


const server = http.createServer(async (req, res) => {
  const u = new URL(req.url, 'http://localhost');
  const p = u.pathname;

  if (req.method === 'OPTIONS') {
    res.writeHead(204, CORS_HEADERS);
    return res.end();
  }

  if (p === '/health') return json(res, 200, { ok: true, root: ROOT });

  if (p === '/api/file' && req.method === 'GET') {
    const abs = safeResolve(u.searchParams.get('rel') || '');
    if (!abs) return json(res, 400, { error: 'invalid path' });
    try {
      const content = fs.readFileSync(abs, 'utf8');
      return json(res, 200, { ok: true, content });
    } catch (e) { return json(res, 404, { error: String(e.message || e) }); }
  }

  // 解析正文块
  if (p === '/api/blocks' && req.method === 'GET') {
    const abs = safeResolve(u.searchParams.get('rel') || '');
    if (!abs) return json(res, 400, { error: 'invalid path' });
    try {
      const md = fs.readFileSync(abs, 'utf8');
      const { blocks, lineCount } = parseBlocks(md);
      return json(res, 200, { ok: true, blocks, lineCount });
    } catch (e) { return json(res, 500, { error: String(e.message || e) }); }
  }

  // 替换单个块
  if (p === '/api/block' && req.method === 'POST') {
    try {
      const body = JSON.parse(await readBody(req));
      const abs = safeResolve(body.rel);
      if (!abs) return json(res, 400, { error: 'invalid path' });
      const md = fs.readFileSync(abs, 'utf8');
      const { blocks } = parseBlocks(md);
      const target = blocks.find((b) => b.id === body.id);
      if (!target) return json(res, 404, { error: 'block id not found (file changed?)' });
      const lines = md.split('\n');
      const text = String(body.text ?? '').replace(/\r\n/g, '\n');
      const newTextLines = text.split('\n');
      // 替换 [start-1, end) 区间
      const before = lines.slice(0, target.start - 1);
      const after = lines.slice(target.end);
      const next = before.concat(newTextLines, after).join('\n');
      const tmp = abs + '.hx-tmp-' + process.pid;
      fs.writeFileSync(tmp, next, 'utf8');
      fs.renameSync(tmp, abs);
      return json(res, 200, { ok: true, replacedLines: [target.start, target.end] });
    } catch (e) { return json(res, 500, { error: String(e.message || e) }); }
  }

  if (p === '/api/file' && req.method === 'POST') {
    try {
      const body = JSON.parse(await readBody(req));
      const abs = safeResolve(body.rel);
      if (!abs) return json(res, 400, { error: 'invalid path' });
      fs.mkdirSync(path.dirname(abs), { recursive: true });
      const tmp = abs + '.hx-tmp-' + process.pid;
      fs.writeFileSync(tmp, body.content ?? '', 'utf8');
      fs.renameSync(tmp, abs);
      return json(res, 200, { ok: true });
    } catch (e) { return json(res, 500, { error: String(e.message || e) }); }
  }

  if (p === '/api/mitemite' && req.method === 'GET') {
    const abs = safeResolve(u.searchParams.get('rel') || '');
    if (!abs) return json(res, 400, { error: 'invalid path' });
    const mit = path.join(path.dirname(abs), '.hx-mitemite.md');
    try {
      const content = fs.readFileSync(mit, 'utf8');
      return json(res, 200, { ok: true, content, blocks: parseMitemiteBlocks(content) });
    } catch { return json(res, 404, { ok: false, error: 'no .hx-mitemite.md' }); }
  }

  if (p === '/api/mitemite' && req.method === 'POST') {
    try {
      const body = JSON.parse(await readBody(req));
      const abs = safeResolve(body.rel);
      if (!abs) return json(res, 400, { error: 'invalid path' });
      const mit = path.join(path.dirname(abs), '.hx-mitemite.md');
      fs.mkdirSync(path.dirname(mit), { recursive: true });
      const tmp = mit + '.hx-tmp-' + process.pid;
      fs.writeFileSync(tmp, body.content ?? '', 'utf8');
      fs.renameSync(tmp, mit);
      return json(res, 200, { ok: true });
    } catch (e) { return json(res, 500, { error: String(e.message || e) }); }
  }


  // 只更新某 block 的答案 (题目不可经此接口修改): body {rel, seq, hash, answer}
  if (p === '/api/mitemite/answer' && req.method === 'POST') {
    try {
      const body = JSON.parse(await readBody(req));
      const abs = safeResolve(body.rel);
      if (!abs) return json(res, 400, { error: 'invalid path' });
      const mit = path.join(path.dirname(abs), '.hx-mitemite.md');
      let content = '';
      try { content = fs.readFileSync(mit, 'utf8'); } catch { return json(res, 404, { error: 'no .hx-mitemite.md' }); }
      const seq = String(body.seq || '');
      const hash = String(body.hash || '');
      const answer = String(body.answer ?? '').replace(/\r\n/g, '\n').trimEnd();
      const headerRe = new RegExp('^##\\s+' + seq + '\\s+' + hash + '\\s+begin\\s+\\{$', 'm');
      const hm = content.match(headerRe);
      if (!hm || hm.index === undefined) return json(res, 404, { error: 'block not found' });
      const blockStart = hm.index;
      const next = content.indexOf('## ', blockStart + hm[0].length);
      const blockEnd = next >= 0 ? next : content.length;
      const blockSrc = content.slice(blockStart, blockEnd);
      const aPos = blockSrc.indexOf('**A**:');
      if (aPos < 0) return json(res, 400, { error: '**A**: not found' });
      const aLineEnd = blockSrc.indexOf('\n', aPos);
      const aBodyStart = aLineEnd >= 0 ? aLineEnd + 1 : blockSrc.length;
      let closePos = -1;
      const blines = blockSrc.split('\n');
      for (let li = blines.length - 1; li >= 0; li--) {
        if (/^\s*\}\s*$/.test(blines[li])) {
          closePos = blines.slice(0, li).join('\n').length + (li > 0 ? 1 : 0);
          break;
        }
      }
      if (closePos < 0) return json(res, 400, { error: 'closing brace not found' });
      const newBlock = blockSrc.slice(0, aPos) + '**A**:' + '\n' + answer + '\n'; // 答案行后补一个换行, tail 从 } 开始
      // 保留 close 行之后的原有空白/换行 (blockEnd 是下一个 block 起点)
      const tail = blockSrc.slice(closePos); // 从 close 行开始, 含 } 与后续空行
      const newContent = content.slice(0, blockStart) + newBlock + tail + content.slice(blockEnd);
      if (newContent === content) return json(res, 200, { ok: true, unchanged: true });
      const tmp = mit + '.hx-tmp-' + process.pid;
      fs.writeFileSync(tmp, newContent, 'utf8');
      fs.renameSync(tmp, mit);
      return json(res, 200, { ok: true, seq, hash });
    } catch (e) { return json(res, 500, { error: String(e.message || e) }); }
  }

  if (p === '/api/open-in-vscode' && req.method === 'GET') {
    const raw = u.searchParams.get('rel') || '';
    const [relPart, lineStr, colStr] = raw.split(':');
    const abs = safeResolve(relPart);
    if (!abs) return json(res, 400, { error: 'invalid path' });
    let line = Number.isFinite(Number(lineStr)) ? Math.max(1, Number(lineStr)) : 0;
    if (line === 0) {
      // 自动定位到文件第一个 # H1 行
      try {
        const raw = fs.readFileSync(abs, 'utf8');
        const lines = raw.split('\n');
        const hi = lines.findIndex((l) => l.startsWith('# ') && !l.startsWith('## '));
        if (hi >= 0) line = hi + 1;
      } catch {} 
    }
    if (line < 1) line = 1;
    const col = Number.isFinite(Number(colStr)) ? Math.max(1, Number(colStr)) : 1;
    const child = spawn('code', ['-g', abs + ':' + line + ':' + col], { stdio: 'ignore', detached: true });
    child.unref();
    focusVSCode(line, col);
    return json(res, 200, { ok: true, line, col });
  }


  // ===== AI 知识库侧边栏重建 =====
  if (p === '/api/sidebar/current' && req.method === 'GET') {
    try {
      const tree = parseCurrentSidebarTree();
      return json(res, 200, { ok: true, tree });
    } catch (e) {
      return json(res, 500, { ok: false, error: String(e.message || e) });
    }
  }

  if (p === '/api/sidebar/rebuild' && req.method === 'POST') {
    try {
      const oldTree = parseCurrentSidebarTree();
      const scan = scanAiDocs(path.join(ROOT, 'ai-docs'));
      const newTree = scan.items;
      const diff = diffSidebarTrees(oldTree, newTree);
      if (!diff.hasChanges) {
        return json(res, 200, { ok: true, diff, unchanged: true, tree: newTree });
      }
      writeSidebarTree(newTree);
      return json(res, 200, { ok: true, diff, unchanged: false, tree: newTree });
    } catch (e) {
      return json(res, 500, { ok: false, error: String(e.message || e) });
    }
  }

  return json(res, 404, { error: 'not found' });
});

server.listen(PORT, () => {
  console.log('[dev-edit-server] listening on http://localhost:' + PORT + ' (root: ' + ROOT + ')');
});
