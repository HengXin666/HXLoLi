/**
 * Docusaurus plugin: kyara 角色配图.
 *
 * 加载原理: static/img/kyara/{name}/NNN-标题@作者.ext, 按序号数值升序 (同序号按文件名), 不合规的文件跳过并 warn;
 * 可选 static/img/kyara/{name}/kyara.json = { displayName?, work?, bangumiCharacterId?, accent?, cover? }.
 *
 * - contentLoaded: 扫描目录, createData 产出每个角色的 KyaraData 与总 KyaraIndex, addRoute:
 *     /anime/kyara          → @site/src/components/anime/kyara/KyaraIndexPage  (modules { index })
 *     /anime/kyara/{name}   → @site/src/components/anime/kyara/KyaraPage       (modules { kyara })
 *   Docusaurus 的 src/pages 不支持动态段, 所以角色页由插件逐个注册.
 * - 运行时清单 (本地模式用): build 时 postBuild 写 kyara/index.json 与 kyara/data/{name}.json; dev 下由中间件现读现算.
 * - getPathsToWatch: static/img/kyara/**, 放图后 dev 自动重扫.
 * - dev-only 写入通道 (防跨站规则照 anime-overlay-plugin: 只认本机 Host + X-HX-Kyara: 1 头, 自定义头会触发 CORS 预检):
 *     POST /__hx/kyara/{name}/import            { url }       → 解析 (pixiv / x / 直链) → 下载 → 按下一个序号落盘
 *     PUT  /__hx/kyara/{name}/file/{filename}   二进制体      → 本地文件提交到仓库 (已存在回 409, ?overwrite=1 覆盖)
 *     POST /__hx/kyara/{name}/reorder           { order }     → 按给定顺序重写序号 (先改临时名再改正式名)
 *
 * 命名规则与解析器和前端共用 src/utils/kyara/filename.ts / resolvers.ts (Node 原生 type stripping 直接 import .ts).
 * CLI scripts/kyara-import.mjs 复用这里导出的 importKyaraUrl / scanKyara.
 */

import fs from 'node:fs';
import path from 'node:path';
import {
  compareKyaraImages,
  formatKyaraFilename,
  isValidKyaraName,
  kyaraExtOf,
  nextKyaraSeq,
  parseKyaraFilename,
} from '../src/utils/kyara/filename.ts';
import { NODE_UA, resolveKyaraUrl } from '../src/utils/kyara/resolvers.ts';

const ROUTE = '/__hx/kyara/';
const MAX_JSON = 64 * 1024;
const MAX_IMAGE = 50 * 1024 * 1024;
const SILENT_FILES = new Set(['kyara.json', 'README.md', '.gitkeep', '.DS_Store', 'Thumbs.db']);

export function kyaraDirOf(siteDir) {
  return path.join(siteDir, 'static', 'img', 'kyara');
}

/** 只读文件头推宽高: png / gif / webp / jpeg; 认不出返回 {} */
export function readImageSize(file) {
  let fd;
  try {
    fd = fs.openSync(file, 'r');
    const buf = Buffer.alloc(512 * 1024);
    const n = fs.readSync(fd, buf, 0, buf.length, 0);
    return imageSizeOf(buf.subarray(0, n));
  } catch {
    return {};
  } finally {
    if (fd !== undefined) fs.closeSync(fd);
  }
}

export function imageSizeOf(b) {
  if (b.length >= 24 && b.readUInt32BE(0) === 0x89504e47) return { w: b.readUInt32BE(16), h: b.readUInt32BE(20) };
  if (b.length >= 10 && b.toString('latin1', 0, 3) === 'GIF') return { w: b.readUInt16LE(6), h: b.readUInt16LE(8) };
  if (b.length >= 30 && b.toString('latin1', 0, 4) === 'RIFF' && b.toString('latin1', 8, 12) === 'WEBP') {
    const kind = b.toString('latin1', 12, 16);
    if (kind === 'VP8X') return { w: 1 + b.readUIntLE(24, 3), h: 1 + b.readUIntLE(27, 3) };
    if (kind === 'VP8L') {
      const bits = b.readUInt32LE(21);
      return { w: 1 + (bits & 0x3fff), h: 1 + ((bits >> 14) & 0x3fff) };
    }
    if (kind === 'VP8 ') return { w: b.readUInt16LE(26) & 0x3fff, h: b.readUInt16LE(28) & 0x3fff };
  }
  if (b.length >= 4 && b[0] === 0xff && b[1] === 0xd8) {
    let i = 2;
    while (i + 9 < b.length) {
      if (b[i] !== 0xff) { i++; continue; }
      const marker = b[i + 1];
      if (marker === 0xd8 || marker === 0x01 || (marker >= 0xd0 && marker <= 0xd7)) { i += 2; continue; }
      const len = b.readUInt16BE(i + 2);
      if (marker >= 0xc0 && marker <= 0xcf && marker !== 0xc4 && marker !== 0xc8 && marker !== 0xcc) {
        return { w: b.readUInt16BE(i + 7), h: b.readUInt16BE(i + 5) };
      }
      i += 2 + len;
    }
  }
  if (b.length >= 12 && b.toString('latin1', 4, 8) === 'ftyp' && /avi[fs]/.test(b.toString('latin1', 8, 12))) return {};
  return {};
}

function looksLikeImage(b) {
  if (b.length < 12) return false;
  return (
    b.readUInt32BE(0) === 0x89504e47 ||
    (b[0] === 0xff && b[1] === 0xd8) ||
    b.toString('latin1', 0, 3) === 'GIF' ||
    (b.toString('latin1', 0, 4) === 'RIFF' && b.toString('latin1', 8, 12) === 'WEBP') ||
    (b.toString('latin1', 4, 8) === 'ftyp' && /avi[fs]/.test(b.toString('latin1', 8, 12)))
  );
}

function srcOf(baseUrl, name, file) {
  return `${baseUrl}img/kyara/${encodeURIComponent(name)}/${encodeURIComponent(file)}`;
}

function readMetaJson(dir, warn) {
  const f = path.join(dir, 'kyara.json');
  if (!fs.existsSync(f)) return {};
  try {
    const j = JSON.parse(fs.readFileSync(f, 'utf8'));
    return j && typeof j === 'object' && !Array.isArray(j) ? j : {};
  } catch (e) {
    warn(`${f} 不是合法 JSON, 忽略: ${e instanceof Error ? e.message : e}`);
    return {};
  }
}

/** 扫描一个角色目录 → KyaraData (目录不存在返回 null) */
export function scanKyaraCharacter(siteDir, baseUrl, name, warn = () => {}) {
  const dir = path.join(kyaraDirOf(siteDir), name);
  if (!fs.existsSync(dir) || !fs.statSync(dir).isDirectory()) return null;
  const images = [];
  for (const ent of fs.readdirSync(dir, { withFileTypes: true })) {
    if (!ent.isFile() || ent.name.startsWith('.kyara-tmp-')) continue;
    const parsed = parseKyaraFilename(ent.name);
    if (!parsed) {
      if (!SILENT_FILES.has(ent.name)) warn(`跳过不合规文件 static/img/kyara/${name}/${ent.name} (应为 NNN-标题@作者.png)`);
      continue;
    }
    const size = readImageSize(path.join(dir, ent.name));
    images.push({ ...parsed, src: srcOf(baseUrl, name, ent.name), source: 'static', ...(size.w && size.h ? size : {}) });
  }
  images.sort(compareKyaraImages);
  const raw = readMetaJson(dir, warn);
  const meta = { name, displayName: typeof raw.displayName === 'string' && raw.displayName ? raw.displayName : name };
  if (typeof raw.work === 'string' && raw.work) meta.work = raw.work;
  if (Number.isInteger(raw.bangumiCharacterId)) meta.bangumiCharacterId = raw.bangumiCharacterId;
  if (typeof raw.accent === 'string' && raw.accent) meta.accent = raw.accent;
  if (typeof raw.cover === 'string' && raw.cover) {
    const hit = images.find((i) => i.file === raw.cover);
    if (hit) meta.cover = hit.src;
    else if (/^(https?:)?\//.test(raw.cover)) meta.cover = raw.cover;
    else warn(`static/img/kyara/${name}/kyara.json 的 cover "${raw.cover}" 不在目录里, 改用第一张`);
  }
  return { meta, images };
}

/** 扫描全部角色 → { characters: KyaraData[], index: KyaraIndex } */
export function scanKyara(siteDir, baseUrl, warn = () => {}) {
  const root = kyaraDirOf(siteDir);
  const characters = [];
  if (fs.existsSync(root)) {
    for (const ent of fs.readdirSync(root, { withFileTypes: true })) {
      if (!ent.isDirectory() || ent.name.startsWith('.')) continue;
      if (!isValidKyaraName(ent.name)) { warn(`跳过不合法的角色目录名: ${ent.name}`); continue; }
      const data = scanKyaraCharacter(siteDir, baseUrl, ent.name, warn);
      if (data) characters.push(data);
    }
  }
  characters.sort((a, b) => a.meta.displayName.localeCompare(b.meta.displayName, 'zh'));
  const index = {
    characters: characters.map((c) => ({
      meta: c.meta,
      count: c.images.length,
      cover: c.meta.cover ?? c.images[0]?.src ?? null,
    })),
  };
  return { characters, index };
}

function routeOf(baseUrl, name) {
  return name === undefined ? `${baseUrl}anime/kyara` : `${baseUrl}anime/kyara/${encodeURIComponent(name)}`;
}

// ── 写盘 ──

const locks = new Map();
/** 同一个角色目录的写操作串行, 防止两个导入抢同一个序号 */
function withLock(name, fn) {
  const prev = locks.get(name) ?? Promise.resolve();
  const next = prev.then(fn, fn);
  locks.set(name, next.catch(() => {}));
  return next;
}

function writeAtomic(file, buf) {
  const tmp = path.join(path.dirname(file), `.kyara-tmp-${process.pid}-${Date.now()}`);
  fs.writeFileSync(tmp, buf);
  fs.renameSync(tmp, file);
}

function assertName(name) {
  if (!isValidKyaraName(name)) throw Object.assign(new Error(`角色名不合法: ${name}`), { status: 400 });
}

async function download(item) {
  const headers = { 'User-Agent': NODE_UA, Accept: 'image/avif,image/webp,image/png,image/*;q=0.8' };
  if (item.referer) headers.Referer = item.referer;
  const res = await fetch(item.imageUrl, { headers, redirect: 'follow' });
  if (!res.ok) throw new Error(`下载失败 HTTP ${res.status}: ${item.imageUrl}`);
  const len = Number(res.headers.get('content-length') ?? 0);
  if (len > MAX_IMAGE) throw new Error(`图片过大 (${len} 字节): ${item.imageUrl}`);
  const buf = Buffer.from(await res.arrayBuffer());
  if (buf.length > MAX_IMAGE) throw new Error(`图片过大: ${item.imageUrl}`);
  if (!looksLikeImage(buf)) throw new Error(`下载到的不是图片: ${item.imageUrl}`);
  const ext = kyaraExtOf(res.headers.get('content-type'), item.imageUrl) ?? 'png';
  return { buf, ext };
}

/**
 * 解析一条链接 (pixiv / x / 直链) → 下载 → 按下一个序号落盘 NNN-标题@作者.ext. 返回新写入的 KyaraImage[].
 * 插件 POST import 与 CLI 共用.
 */
export async function importKyaraUrl(siteDir, baseUrl, name, url) {
  assertName(name);
  const items = await resolveKyaraUrl(String(url));
  const fetched = [];
  for (const it of items) fetched.push({ it, ...(await download(it)) });
  return withLock(name, () => {
    const dir = path.join(kyaraDirOf(siteDir), name);
    fs.mkdirSync(dir, { recursive: true });
    const cur = scanKyaraCharacter(siteDir, baseUrl, name)?.images ?? [];
    let seq = nextKyaraSeq(cur);
    const out = [];
    for (const { it, buf, ext } of fetched) {
      const file = formatKyaraFilename(seq++, it.title, it.author || null, ext);
      writeAtomic(path.join(dir, file), buf);
      const size = imageSizeOf(buf);
      out.push({ ...parseKyaraFilename(file), src: srcOf(baseUrl, name, file), source: 'static', sourceUrl: it.sourceUrl, ...(size.w && size.h ? size : {}) });
    }
    return out;
  });
}

/** 写入一个已按规则命名的文件 (本地模式 "写进仓库") */
export async function putKyaraFile(siteDir, baseUrl, name, filename, buf, overwrite = false) {
  assertName(name);
  const parsed = parseKyaraFilename(filename);
  if (!parsed || filename !== path.basename(filename)) throw Object.assign(new Error(`文件名不合规: ${filename} (应为 NNN-标题@作者.png)`), { status: 400 });
  if (!looksLikeImage(buf)) throw Object.assign(new Error('请求体不是图片'), { status: 415 });
  return withLock(name, () => {
    const dir = path.join(kyaraDirOf(siteDir), name);
    fs.mkdirSync(dir, { recursive: true });
    const file = path.join(dir, filename);
    if (fs.existsSync(file) && !overwrite) throw Object.assign(new Error(`已存在: ${filename}`), { status: 409 });
    writeAtomic(file, buf);
    const size = imageSizeOf(buf);
    return { ...parsed, src: srcOf(baseUrl, name, filename), source: 'static', ...(size.w && size.h ? size : {}) };
  });
}

/** 按 order 重写序号 001..N, 标题@作者.ext 原样保留; order 必须恰好是目录里全部合规文件 */
export async function reorderKyara(siteDir, baseUrl, name, order) {
  assertName(name);
  if (!Array.isArray(order) || order.some((f) => typeof f !== 'string')) throw Object.assign(new Error('order 必须是文件名数组'), { status: 400 });
  return withLock(name, () => {
    const data = scanKyaraCharacter(siteDir, baseUrl, name);
    if (!data) throw Object.assign(new Error(`没有这个角色: ${name}`), { status: 404 });
    const have = new Set(data.images.map((i) => i.file));
    if (order.length !== have.size || new Set(order).size !== order.length || order.some((f) => !have.has(f))) {
      throw Object.assign(new Error('order 必须恰好包含该角色目录下的全部图片文件名'), { status: 400 });
    }
    const dir = path.join(kyaraDirOf(siteDir), name);
    const width = Math.max(3, String(order.length).length);
    const plan = order.map((f, i) => ({ from: f, to: `${String(i + 1).padStart(width, '0')}${f.slice(f.indexOf('-'))}` }));
    const moving = plan.filter((p) => p.from !== p.to);
    const tmps = moving.map((p, i) => {
      const tmp = `.kyara-tmp-${process.pid}-${i}-${p.from}`;
      fs.renameSync(path.join(dir, p.from), path.join(dir, tmp));
      return tmp;
    });
    moving.forEach((p, i) => fs.renameSync(path.join(dir, tmps[i]), path.join(dir, p.to)));
    return scanKyaraCharacter(siteDir, baseUrl, name).images;
  });
}

// ── dev 中间件 ──

function isLocalHost(host) {
  const h = String(host ?? '').replace(/:\d+$/, '');
  return h === 'localhost' || h === '127.0.0.1' || h === '[::1]';
}

function send(res, code, obj) {
  res.statusCode = code;
  res.setHeader('Content-Type', 'application/json; charset=utf-8');
  res.setHeader('Cache-Control', 'no-store');
  res.end(JSON.stringify(obj));
}

function readBody(req, max) {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks = [];
    req.on('data', (c) => {
      size += c.length;
      if (size > max) { reject(Object.assign(new Error('body 过大'), { status: 413 })); req.destroy(); return; }
      chunks.push(c);
    });
    req.on('end', () => resolve(Buffer.concat(chunks)));
    req.on('error', reject);
  });
}

function safeDecode(s) {
  try { return decodeURIComponent(s); } catch { return null; }
}

/**
 * 插件入口: 扫描 static/img/kyara 并注册 /anime/kyara 与 /anime/kyara/{name}, 外加 dev-only 写入通道.
 * .agents/notes/implemented/feature/2026-11-06-anime-kyara-gallery.md
 */
export default function kyaraGalleryPlugin(context) {
  const { siteDir, baseUrl } = context;
  const root = kyaraDirOf(siteDir);
  const warn = (m) => console.warn(`[kyara-gallery] ${m}`);
  const manifestPrefix = `${baseUrl}kyara/`;

  async function handleWrite(req, res, rest) {
    if (!isLocalHost(req.headers.host)) return send(res, 403, { ok: false, error: '只接受本机请求' });
    if (req.headers['x-hx-kyara'] !== '1') return send(res, 403, { ok: false, error: '缺少 X-HX-Kyara 头' });
    const [pathPart, query = ''] = rest.split('?');
    const seg = pathPart.split('/');
    const name = safeDecode(seg[0] ?? '');
    if (!name || !isValidKyaraName(name)) return send(res, 400, { ok: false, error: '角色名不合法' });
    const action = seg[1];
    if (req.method === 'POST' && action === 'import' && seg.length === 2) {
      if (!String(req.headers['content-type'] ?? '').startsWith('application/json')) return send(res, 415, { ok: false, error: '需要 application/json' });
      const body = JSON.parse((await readBody(req, MAX_JSON)).toString('utf8'));
      if (!body || typeof body.url !== 'string') return send(res, 400, { ok: false, error: '需要 { url }' });
      const images = await importKyaraUrl(siteDir, baseUrl, name, body.url);
      return send(res, 200, { ok: true, images });
    }
    if (req.method === 'PUT' && action === 'file' && seg.length === 3) {
      const filename = safeDecode(seg[2]);
      if (!filename) return send(res, 400, { ok: false, error: '文件名不合法' });
      const buf = await readBody(req, MAX_IMAGE);
      const image = await putKyaraFile(siteDir, baseUrl, name, filename, buf, /(^|&)overwrite=1(&|$)/.test(query));
      return send(res, 200, { ok: true, image });
    }
    if (req.method === 'POST' && action === 'reorder' && seg.length === 2) {
      if (!String(req.headers['content-type'] ?? '').startsWith('application/json')) return send(res, 415, { ok: false, error: '需要 application/json' });
      const body = JSON.parse((await readBody(req, MAX_JSON)).toString('utf8'));
      const images = await reorderKyara(siteDir, baseUrl, name, body?.order);
      return send(res, 200, { ok: true, images });
    }
    return send(res, 404, { ok: false, error: '未知的 kyara 写入路由' });
  }

  async function handle(req, res, next) {
    const url = req.url ?? '';
    if (url.startsWith(ROUTE)) {
      try {
        return await handleWrite(req, res, url.slice(ROUTE.length));
      } catch (e) {
        const status = e && typeof e === 'object' && 'status' in e ? e.status : e instanceof SyntaxError ? 400 : 502;
        return send(res, status, { ok: false, error: e instanceof Error ? e.message : String(e) });
      }
    }
    // dev 下的运行时清单: 现读现算, 跟 postBuild 写出的文件同形
    if ((req.method === 'GET' || req.method === 'HEAD') && url.startsWith(manifestPrefix)) {
      const rest = url.slice(manifestPrefix.length).split('?')[0];
      if (rest === 'index.json') return send(res, 200, scanKyara(siteDir, baseUrl).index);
      const m = /^data\/([^/]+)\.json$/.exec(rest);
      const name = m ? safeDecode(m[1]) : null;
      if (name && isValidKyaraName(name)) {
        const data = scanKyaraCharacter(siteDir, baseUrl, name);
        return data ? send(res, 200, data) : send(res, 404, { ok: false, error: '没有这个角色' });
      }
    }
    return next();
  }

  return {
    name: 'kyara-gallery',
    getPathsToWatch() {
      return [path.join(root, '**')];
    },
    async loadContent() {
      return scanKyara(siteDir, baseUrl, warn);
    },
    async contentLoaded({ content, actions }) {
      const { createData, addRoute } = actions;
      // 页面组件属于路 4 (src/components/anime/kyara/), 还没落盘时不注册路由, 免得整站编译失败
      const hasPage = (base) => ['.tsx', '.jsx', '.ts', '.js'].some((ext) => fs.existsSync(path.join(siteDir, 'src', 'components', 'anime', 'kyara', base + ext)));
      if (!hasPage('KyaraIndexPage') || !hasPage('KyaraPage')) {
        warn('src/components/anime/kyara/KyaraIndexPage|KyaraPage 不存在, 暂不注册 /anime/kyara 路由 (数据与写入通道照常)');
        return;
      }
      const indexPath = await createData('kyara-index.json', JSON.stringify(content.index));
      addRoute({
        path: routeOf(baseUrl),
        component: '@site/src/components/anime/kyara/KyaraIndexPage',
        modules: { index: indexPath },
        exact: true,
      });
      for (const data of content.characters) {
        const dataPath = await createData(`kyara-${encodeURIComponent(data.meta.name).replace(/%/g, '_')}.json`, JSON.stringify(data));
        addRoute({
          path: routeOf(baseUrl, data.meta.name),
          component: '@site/src/components/anime/kyara/KyaraPage',
          modules: { kyara: dataPath },
          exact: true,
        });
      }
    },
    async postBuild({ outDir, content }) {
      const dir = path.join(outDir, 'kyara');
      fs.mkdirSync(path.join(dir, 'data'), { recursive: true });
      fs.writeFileSync(path.join(dir, 'index.json'), JSON.stringify(content.index));
      for (const data of content.characters) fs.writeFileSync(path.join(dir, 'data', `${data.meta.name}.json`), JSON.stringify(data));
    },
    configureWebpack(config, isServer) {
      if (isServer || config.mode !== 'development') return {};
      return {
        devServer: {
          // 同 anime-overlay-plugin: 用 onListening 挂在栈末尾, 不用 setupMiddlewares (webpack-merge 会把栈拼两遍)
          onListening(devServer) {
            devServer.app.use((req, res, next) => { handle(req, res, next).catch(next); });
          },
        },
      };
    },
  };
}
