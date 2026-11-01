/**
 * Docusaurus plugin: 番剧本人标注 overlay 的 dev 写入通道.
 *
 * 只在 `docusaurus start` (webpack mode = development, 客户端那份配置) 下往 dev server 挂两个路由:
 *   PUT    /__hx/anime-overlay/<subjectId>   body = AnimeOverlayEntry (JSON), 整条替换
 *   DELETE /__hx/anime-overlay/<subjectId>   删掉这部番的 overlay
 * 落盘到 data/animeOverlay.json: 键按数字升序, 字段顺序固定, 2 空格缩进 + 结尾换行, 一次改动只动一块, git diff 可读.
 * 先写临时文件再 rename, 写一半崩掉不会留下半个 JSON.
 *
 * 生产构建 (`docusaurus build`) 里 configureWebpack 直接返回 {}, 这里的代码是 Node 侧插件, 本来就不进前端包;
 * 前端那半 (编辑器与 fetch) 由 src/components/home/lazyChunks.js 按 NODE_ENV 常量裁掉, 见 note.
 *
 * 防跨站写: 只接受 Content-Type: application/json + 自定义头 X-HX-Overlay (浏览器会先发 CORS 预检, dev server 不放行),
 * 并要求 Host 是 localhost / 127.0.0.1 / [::1].
 */

import fs from 'node:fs';
import path from 'node:path';

const ROUTE = '/__hx/anime-overlay/';
const FIELD_ORDER = ['watch_status', 'watched_eps', 'rate', 'tags', 'comment', 'updated_at'];
const MAX_BODY = 64 * 1024;

/**
 * 写入通道的形状与落盘格式 (键升序、字段固定、临时文件 + rename) 见文件头注释.
 * .agents/notes/implemented/feature/2026-11-01-anime-local-overlay.md
 */
function sanitizeEntry(raw) {
  if (!raw || typeof raw !== 'object' || Array.isArray(raw)) throw new Error('body 必须是对象');
  const out = {};
  if (raw.tags !== undefined) {
    if (!Array.isArray(raw.tags) || raw.tags.some((t) => typeof t !== 'string')) throw new Error('tags 必须是字符串数组');
    const tags = [...new Set(raw.tags.map((t) => t.trim()).filter(Boolean))];
    if (tags.length > 40 || tags.some((t) => t.length > 30)) throw new Error('tags 过多或过长');
    out.tags = tags;
  }
  if (raw.watch_status !== undefined) {
    if (![1, 2, 3, 4, 5].includes(raw.watch_status)) throw new Error('watch_status 只能是 1-5');
    out.watch_status = raw.watch_status;
  }
  if (raw.watched_eps !== undefined) {
    if (!Number.isInteger(raw.watched_eps) || raw.watched_eps < 0 || raw.watched_eps > 10000) throw new Error('watched_eps 必须是非负整数');
    out.watched_eps = raw.watched_eps;
  }
  if (raw.rate !== undefined) {
    if (!Number.isInteger(raw.rate) || raw.rate < 1 || raw.rate > 10) throw new Error('rate 只能是 1-10 的整数');
    out.rate = raw.rate;
  }
  if (raw.comment !== undefined) {
    if (typeof raw.comment !== 'string' || raw.comment.length > 4000) throw new Error('comment 必须是 4000 字以内的字符串');
    out.comment = raw.comment;
  }
  if (Object.keys(out).length === 0) throw new Error('没有可写的字段; 要清空请用 DELETE');
  out.updated_at = new Date().toISOString();
  return out;
}

function readOverlay(file) {
  try {
    const parsed = JSON.parse(fs.readFileSync(file, 'utf8'));
    return parsed && typeof parsed === 'object' && !Array.isArray(parsed) ? parsed : {};
  } catch (e) {
    if (e && e.code === 'ENOENT') return {};
    throw e;
  }
}

function writeOverlay(file, data) {
  const sorted = {};
  for (const id of Object.keys(data).sort((a, b) => Number(a) - Number(b))) {
    const entry = {};
    for (const k of FIELD_ORDER) if (data[id][k] !== undefined) entry[k] = data[id][k];
    sorted[id] = entry;
  }
  const tmp = `${file}.tmp-${process.pid}`;
  fs.writeFileSync(tmp, `${JSON.stringify(sorted, null, 2)}\n`);
  fs.renameSync(tmp, file);
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks = [];
    req.on('data', (c) => {
      size += c.length;
      if (size > MAX_BODY) { reject(new Error('body 过大')); req.destroy(); return; }
      chunks.push(c);
    });
    req.on('end', () => resolve(Buffer.concat(chunks).toString('utf8')));
    req.on('error', reject);
  });
}

function isLocalHost(host) {
  const h = String(host ?? '').replace(/:\d+$/, '');
  return h === 'localhost' || h === '127.0.0.1' || h === '[::1]';
}

function send(res, code, obj) {
  res.statusCode = code;
  res.setHeader('Content-Type', 'application/json; charset=utf-8');
  res.end(JSON.stringify(obj));
}

export default function animeOverlayPlugin(context) {
  const file = path.join(context.siteDir, 'data', 'animeOverlay.json');

  async function handle(req, res, next) {
    if (!req.url || !req.url.startsWith(ROUTE)) return next();
    if (req.method !== 'PUT' && req.method !== 'DELETE') return next();
    if (!isLocalHost(req.headers.host)) return send(res, 403, { ok: false, error: '只接受本机请求' });
    if (req.headers['x-hx-overlay'] !== '1') return send(res, 403, { ok: false, error: '缺少 X-HX-Overlay 头' });
    const id = req.url.slice(ROUTE.length).split('?')[0];
    if (!/^[1-9]\d{0,9}$/.test(id)) return send(res, 400, { ok: false, error: 'subject id 不合法' });
    try {
      const data = readOverlay(file);
      if (req.method === 'DELETE') {
        delete data[id];
        writeOverlay(file, data);
        return send(res, 200, { ok: true, id: Number(id), entry: null });
      }
      if (!String(req.headers['content-type'] ?? '').startsWith('application/json')) {
        return send(res, 415, { ok: false, error: '需要 application/json' });
      }
      const entry = sanitizeEntry(JSON.parse(await readBody(req)));
      data[id] = entry;
      writeOverlay(file, data);
      return send(res, 200, { ok: true, id: Number(id), entry });
    } catch (e) {
      return send(res, 400, { ok: false, error: e instanceof Error ? e.message : String(e) });
    }
  }

  return {
    name: 'anime-overlay-dev-writer',
    configureWebpack(config, isServer) {
      if (isServer || config.mode !== 'development') return {};
      return {
        devServer: {
          // 用 onListening 而不是 setupMiddlewares: Docusaurus 自己定义了 setupMiddlewares,
          // webpack-merge 6 会把两个函数的返回数组拼起来, 中间件栈会重复一遍. onListening 它没定义, 合并时只有这一份.
          // 挂在栈末尾也够用: 前面的 dev-middleware / static / historyApiFallback 都只处理 GET/HEAD.
          onListening(devServer) {
            devServer.app.use((req, res, next) => { handle(req, res, next).catch(next); });
          },
        },
      };
    },
  };
}
