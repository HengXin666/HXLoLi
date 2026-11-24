/*
 * HXLoLi 图片缓存 Service Worker (只管图片, 其它请求一律不碰).
 *
 * 1. 番剧海报 / 角色图: HXLoLi-ANiMe@<tag>/data/(anime|kyara)/... 的图片.
 *    URL 里带 CDN tag 且 CDN 节点会自动切换, 同一张图换 tag / 换节点就是新 URL, HTTP 缓存全部失效.
 *    这里按 "data/ 之后的路径" 做缓存键, 与节点和 tag 无关; 缓存优先, 30 天后重新拉.
 * 2. 立绘: 同源 /img/catgirl/ 下的图片. 先给缓存, 后台再拉一次更新 (stale-while-revalidate).
 * 3. 音乐库 HXLoLi-Music@<tag>/static/... (不带 Range 的整文件请求):
 *    字体和封面按路径缓存 (与 tag 无关, 30 天); 歌词 / 详细配置 / 切片按 "tag + 路径" 缓存 (同 tag 内容不变).
 *    带 Range 的请求 (音频流, 分段下载) 不碰, 交给浏览器.
 *
 * 改这个文件后把 VERSION 加一, 旧缓存会在 activate 时清掉.
 */
const VERSION = 1;
const ANIME_CACHE = `hx-img-anime-v${VERSION}`;
const NEKO_CACHE = `hx-img-neko-v${VERSION}`;
const MUSIC_CACHE = `hx-music-v${VERSION}`;
const KEEP = new Set([ANIME_CACHE, NEKO_CACHE, MUSIC_CACHE]);
const MUSIC_MAX_ENTRIES = 800;
const MUSIC_RE = /\/HXLoLi-Music@([^/]+)\/(static\/[^?#]+)$/;
const MUSIC_STABLE_RE = /\.(?:ttf|otf|ttc|woff2?|jpe?g|png|webp|avif)$/i;
const MUSIC_SKIP_RE = /\.(?:mp3|flac|m4a|aac|ogg|opus|wav|webm)$/i;
const ANIME_MAX_ENTRIES = 2000;
const ANIME_MAX_AGE_MS = 30 * 24 * 3600 * 1000;
const ANIME_RE = /\/HXLoLi-ANiMe@[^/]+\/(data\/(?:anime|kyara)\/[^?#]+\.(?:jpe?g|png|webp|avif|gif))$/i;
const NEKO_RE = /\/img\/catgirl\/[^?#]+\.(?:webp|png)$/i;

self.addEventListener('install', () => self.skipWaiting());

self.addEventListener('activate', (event) => {
    event.waitUntil((async () => {
        for (const name of await caches.keys()) {
            if ((name.startsWith('hx-img-') || name.startsWith('hx-music-')) && !KEEP.has(name)) await caches.delete(name);
        }
        await self.clients.claim();
    })());
});

self.addEventListener('fetch', (event) => {
    const req = event.request;
    if (req.method !== 'GET') return;
    const url = new URL(req.url);
    const anime = url.pathname.match(ANIME_RE);
    if (anime) {
        event.respondWith(animeImage(req, anime[1]));
        return;
    }
    const music = url.pathname.match(MUSIC_RE);
    // 带查询串的 (测速, 防缓存探测) 不碰
    if (music && !url.search && !req.headers.has('range')) {
        const path = decodeURIComponent(music[2]);
        if (MUSIC_SKIP_RE.test(path)) return;
        const stable = MUSIC_STABLE_RE.test(path);
        const key = stable ? `music/${path}` : `music@${music[1]}/${path}`;
        event.respondWith(cacheFirst(req, MUSIC_CACHE, key, stable ? ANIME_MAX_AGE_MS : Infinity, MUSIC_MAX_ENTRIES));
        return;
    }
    if (url.origin === self.location.origin && NEKO_RE.test(url.pathname)) {
        event.respondWith(nekoImage(event, req));
    }
});

/** 用 cors 模式拉, 拿到可读响应才进缓存; 节点不给 CORS 就退回原请求, 只是不缓存 */
async function fetchCors (url) {
    try {
        const res = await fetch(url, { mode: 'cors', credentials: 'omit' });
        return res;
    } catch (_) {
        return null;
    }
}

function animeImage (req, path) {
    return cacheFirst(req, ANIME_CACHE, path, ANIME_MAX_AGE_MS, ANIME_MAX_ENTRIES);
}

/**
 * 缓存优先: 键与 CDN 节点无关 (/__hx-img/<key>). 命中且未过期直接给; 否则 cors 拉一次,
 * 成功才存. 拉失败时给过期的旧缓存, 再不行就退回原请求.
 */
async function cacheFirst (req, cacheName, keyPath, maxAgeMs, maxEntries) {
    const cache = await caches.open(cacheName);
    const key = new Request(`${self.location.origin}/__hx-img/${encodeURI(keyPath)}`);
    const hit = await cache.match(key);
    const at = hit ? Number(hit.headers.get('x-hx-cached-at') || 0) : 0;
    if (hit && Date.now() - at < maxAgeMs) return hit;
    const res = await fetchCors(req.url);
    if (res && res.ok) {
        const body = await res.blob();
        const headers = new Headers({
            'content-type': res.headers.get('content-type') || body.type || 'application/octet-stream',
            'content-length': String(body.size),
            'x-hx-cached-at': String(Date.now()),
        });
        const stored = new Response(body, { status: 200, headers });
        await cache.put(key, stored.clone());
        trimLater(cache, maxEntries);
        return stored;
    }
    if (hit) return hit;
    return fetch(req);
}

async function nekoImage (event, req) {
    const cache = await caches.open(NEKO_CACHE);
    const hit = await cache.match(req);
    const update = fetch(req).then(async (res) => {
        if (res && res.ok) await cache.put(req, res.clone());
        return res;
    }).catch(() => null);
    if (hit) {
        event.waitUntil(update);
        return hit;
    }
    const res = await update;
    return res || fetch(req);
}

let puts = 0;
function trimLater (cache, maxEntries) {
    puts += 1;
    if (puts % 50 !== 0) return;
    cache.keys().then(async (keys) => {
        const extra = keys.length - maxEntries;
        for (let i = 0; i < extra; i += 1) await cache.delete(keys[i]);
    });
}
