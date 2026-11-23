/**
 * 番剧数据文件的持久缓存 (Cache Storage, 按 ANIME_CDN_TAG 分桶).
 *
 * 为什么要有: ANiMeRecord.json 解压后约 15 MB (gzip 约 2 MB), 详情页每次冷加载都要整份拉下来.
 * HTTP 缓存兜不住, 因为 CDN 节点会被自动优选切换 (cdn.jsdelivr / fastly / ...), URL 一变就是新请求.
 * 数据文件挂在 tag 上, 同一个 tag 下内容不会变, 所以按 tag 存一份就能一直用, 换 tag 时清掉旧的.
 *
 * 拿不到 Cache Storage (SSR, 非安全上下文, 隐私模式) 时直接走网络, 行为和以前一样.
 */
import { ANIME_CDN_TAG } from "@site/data/animeCdnVersion";
import { reqAnimeByCDN } from "@site/src/utils/cdn/linkJsDelivr";

const CACHE_PREFIX = "hxloli-anime-data:";
const CACHE_NAME = `${CACHE_PREFIX}${ANIME_CDN_TAG}`;
/** Cache Storage 的 key 必须是 URL, 这里只当命名空间用, 不会真的请求它 */
const keyOf = (path: string) => `https://hxloli-anime-data.local/${ANIME_CDN_TAG}${path.startsWith("/") ? path : `/${path}`}`;

const inflight = new Map<string, Promise<string>>();
let cleaned = false;

function hasCacheStorage (): boolean {
    return typeof window !== "undefined" && typeof caches !== "undefined";
}

/** 删掉旧 tag 的桶, 每次页面加载只做一次, 失败不影响读数据 */
function cleanOldBuckets (): void {
    if (cleaned || !hasCacheStorage()) return;
    cleaned = true;
    caches.keys().then((names) => Promise.all(
        names.filter((n) => n.startsWith(CACHE_PREFIX) && n !== CACHE_NAME).map((n) => caches.delete(n)),
    )).catch(() => { /* 忽略 */ });
}

async function fromNetwork (path: string): Promise<string> {
    const res = await reqAnimeByCDN(path);
    return res.blob.text();
}

async function load (path: string): Promise<string> {
    if (!hasCacheStorage()) return fromNetwork(path);
    cleanOldBuckets();
    let cache: Cache | null = null;
    try {
        cache = await caches.open(CACHE_NAME);
        const hit = await cache.match(keyOf(path));
        if (hit) return await hit.text();
    } catch { /* 缓存读不了就当没命中 */ }
    const text = await fromNetwork(path);
    if (cache) {
        cache.put(keyOf(path), new Response(text, { headers: { "Content-Type": "application/json" } }))
            .catch(() => { /* 配额不够等: 这次不存, 下次再拉 */ });
    }
    return text;
}

/** 读番剧数据仓库里的文本文件 (路径同 reqAnimeByCDN), 同一 tag 下命中本地缓存就不走网络 */
export function fetchAnimeDataText (path: string): Promise<string> {
    const hit = inflight.get(path);
    if (hit) return hit;
    const p = load(path);
    inflight.set(path, p);
    p.catch(() => { inflight.delete(path); });
    return p;
}
