/*
 * HXLoLi 图片缓存 Service Worker (只管图片, 其它请求一律不碰).
 *
 * 1. 番剧海报 / 角色图: HXLoLi-ANiMe@<tag>/data/(anime|kyara)/... 的图片.
 *    URL 里带 CDN tag 且 CDN 节点会自动切换, 同一张图换 tag / 换节点就是新 URL, HTTP 缓存全部失效.
 *    这里按 "data/ 之后的路径" 做缓存键, 与节点和 tag 无关; 缓存优先, 30 天后重新拉.
 * 2. 立绘: 同源 /img/catgirl/ 下的图片. 先给缓存, 后台再拉一次更新 (stale-while-revalidate).
 *
 * 改这个文件后把 VERSION 加一, 旧缓存会在 activate 时清掉.
 */
const VERSION = 1;
const ANIME_CACHE = `hx-img-anime-v${VERSION}`;
const NEKO_CACHE = `hx-img-neko-v${VERSION}`;
const KEEP = new Set([ANIME_CACHE, NEKO_CACHE]);
const ANIME_MAX_ENTRIES = 2000;
const ANIME_MAX_AGE_MS = 30 * 24 * 3600 * 1000;
const ANIME_RE = /\/HXLoLi-ANiMe@[^/]+\/(data\/(?:anime|kyara)\/[^?#]+\.(?:jpe?g|png|webp|avif|gif))$/i;
const NEKO_RE = /\/img\/catgirl\/[^?#]+\.(?:webp|png)$/i;

self.addEventListener('install', () => self.skipWaiting());

self.addEventListener('activate', (event) => {
    event.waitUntil((async () => {
        for (const name of await caches.keys()) {
            if (name.startsWith('hx-img-') && !KEEP.has(name)) await caches.delete(name);
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

async function animeImage (req, path) {
    const cache = await caches.open(ANIME_CACHE);
    const key = new Request(`${self.location.origin}/__hx-img/${path}`);
    const hit = await cache.match(key);
    const at = hit ? Number(hit.headers.get('x-hx-cached-at') || 0) : 0;
    if (hit && Date.now() - at < ANIME_MAX_AGE_MS) return hit;
    const res = await fetchCors(req.url);
    if (res && res.ok) {
        const body = await res.blob();
        const headers = new Headers({
            'content-type': res.headers.get('content-type') || 'image/jpeg',
            'x-hx-cached-at': String(Date.now()),
        });
        const stored = new Response(body, { status: 200, headers });
        await cache.put(key, stored.clone());
        trimLater(cache);
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
function trimLater (cache) {
    puts += 1;
    if (puts % 50 !== 0) return;
    cache.keys().then(async (keys) => {
        const extra = keys.length - ANIME_MAX_ENTRIES;
        for (let i = 0; i < extra; i += 1) await cache.delete(keys[i]);
    });
}
