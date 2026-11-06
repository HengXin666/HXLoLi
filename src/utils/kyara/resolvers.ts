import type { KyaraImportItem, KyaraSourceResolver } from "./types";

// 本文件也被 Node 侧 (插件 / CLI) 直接 import, 只许 `import type` (见 filename.ts 顶部说明).
// 浏览器和 Node 共用: 只用全局 fetch / URL, 不碰 node:*.

export const PIXIV_REFERER = "https://www.pixiv.net/";
/** Node 侧请求 pixiv / 图床时带的浏览器 UA (默认的 node UA 会被部分站点拒绝) */
export const NODE_UA = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/130.0 Safari/537.36";

const PIXIV_RE = /^https?:\/\/(?:www\.)?pixiv\.net\/(?:[a-z]{2}\/)?(?:artworks|i)\/(\d+)(?:[/?#]|$)/i;
const PIXIV_LEGACY_RE = /^https?:\/\/(?:www\.)?pixiv\.net\/member_illust\.php\?(?:[^#]*&)?illust_id=(\d+)/i;
const X_RE = /^https?:\/\/(?:www\.|mobile\.)?(?:x|twitter)\.com\/([A-Za-z0-9_]{1,15})\/status(?:es)?\/(\d+)(?:[/?#]|$)/i;
const IMAGE_PATH_RE = /\.(png|jpe?g|webp|gif|avif)$/i;

export class KyaraResolveError extends Error {
    code: "node-only" | "http" | "no-image" | "unsupported";
    constructor(code: "node-only" | "http" | "no-image" | "unsupported", message: string) {
        super(message);
        this.name = "KyaraResolveError";
        this.code = code;
    }
}

function isBrowser(): boolean {
    return typeof window !== "undefined" && typeof document !== "undefined";
}

function pickFetch(fetchImpl?: typeof fetch): typeof fetch {
    const f = fetchImpl ?? (typeof fetch === "function" ? fetch : undefined);
    if (!f) throw new KyaraResolveError("unsupported", "当前环境没有 fetch");
    return f;
}

async function getJson(f: typeof fetch, url: string, headers: Record<string, string>): Promise<any> {
    const res = await f(url, { headers });
    if (!res.ok) throw new KyaraResolveError("http", `${url} 返回 HTTP ${res.status}`);
    return res.json();
}

function shortText(s: string, max: number): string {
    const t = s.replace(/https?:\/\/\S+/g, "").replace(/\s+/g, " ").trim();
    return Array.from(t).slice(0, max).join("");
}

function pixivId(url: string): string | null {
    const m = PIXIV_RE.exec(url) ?? PIXIV_LEGACY_RE.exec(url);
    return m ? m[1] : null;
}

/** pixiv: ajax 接口拿标题 / 作者 / 原图; 原图下载必须带 Referer, 浏览器里会被 CORS 挡住, 只在 Node 侧可用 */
const pixivResolver: KyaraSourceResolver = {
    id: "pixiv",
    match: (url) => pixivId(url) !== null,
    async resolve(url, fetchImpl) {
        const id = pixivId(url);
        if (!id) throw new KyaraResolveError("unsupported", `不是 pixiv 作品链接: ${url}`);
        if (isBrowser() && !fetchImpl) throw new KyaraResolveError("node-only", "pixiv 需要 Referer, 浏览器里拿不到, 请在本机 dev 下导入");
        const f = pickFetch(fetchImpl);
        const headers: Record<string, string> = { Referer: PIXIV_REFERER, Accept: "application/json" };
        if (!isBrowser()) headers["User-Agent"] = NODE_UA;
        const info = await getJson(f, `https://www.pixiv.net/ajax/illust/${id}`, headers);
        if (info?.error) throw new KyaraResolveError("http", `pixiv: ${info.message || "作品不可访问"}`);
        const pages = await getJson(f, `https://www.pixiv.net/ajax/illust/${id}/pages`, headers);
        const list: any[] = Array.isArray(pages?.body) ? pages.body : [];
        const urls = list.map((p) => p?.urls?.original).filter((u): u is string => typeof u === "string");
        if (urls.length === 0) throw new KyaraResolveError("no-image", `pixiv ${id} 没有可下载的原图 (可能需要登录)`);
        const title: string = String(info?.body?.title ?? `pixiv-${id}`);
        const author: string = String(info?.body?.userName ?? "");
        const sourceUrl = `https://www.pixiv.net/artworks/${id}`;
        return urls.map((imageUrl, i) => ({
            imageUrl,
            title: urls.length > 1 ? `${title} p${i + 1}` : title,
            author,
            sourceUrl,
            referer: PIXIV_REFERER,
        }));
    },
};

function origTwimg(u: string): string {
    try {
        const url = new URL(u);
        if (url.hostname === "pbs.twimg.com") url.searchParams.set("name", "orig");
        return url.toString();
    } catch {
        return u;
    }
}

/** x.com / twitter.com: 走 fxtwitter 的公开 JSON 拿图, 原图加 name=orig */
const xResolver: KyaraSourceResolver = {
    id: "x",
    match: (url) => X_RE.test(url),
    async resolve(url, fetchImpl) {
        const m = X_RE.exec(url);
        if (!m) throw new KyaraResolveError("unsupported", `不是推文链接: ${url}`);
        const [, user, id] = m;
        const f = pickFetch(fetchImpl);
        const data = await getJson(f, `https://api.fxtwitter.com/status/${id}`, { Accept: "application/json" });
        const tweet = data?.tweet;
        if (!tweet) throw new KyaraResolveError("http", `fxtwitter: ${data?.message || "推文不可访问"}`);
        const photos: any[] = Array.isArray(tweet?.media?.photos) ? tweet.media.photos : [];
        const urls = photos.map((p) => p?.url).filter((u): u is string => typeof u === "string");
        if (urls.length === 0) throw new KyaraResolveError("no-image", `推文 ${id} 里没有图片`);
        const screen = String(tweet?.author?.screen_name ?? user);
        const title = shortText(String(tweet?.text ?? ""), 30) || `status-${id}`;
        const sourceUrl = `https://x.com/${screen}/status/${id}`;
        return urls.map((u, i) => ({
            imageUrl: origTwimg(u),
            title: urls.length > 1 ? `${title} ${i + 1}` : title,
            author: `@${screen}`,
            sourceUrl,
        }));
    },
};

function isDirectImage(url: string): boolean {
    try {
        const u = new URL(url);
        if (u.protocol !== "https:" && u.protocol !== "http:") return false;
        if (u.hostname === "pbs.twimg.com" && u.pathname.startsWith("/media/")) return true;
        return IMAGE_PATH_RE.test(u.pathname);
    } catch {
        return false;
    }
}

/** 图片直链: 标题取文件名主干; 文件名本身符合 NNN-标题@作者 时按最后一个 @ 取作者 */
const directResolver: KyaraSourceResolver = {
    id: "direct",
    match: isDirectImage,
    async resolve(url) {
        if (!isDirectImage(url)) throw new KyaraResolveError("unsupported", `不是图片直链: ${url}`);
        const u = new URL(url);
        let stem = decodeURIComponent(u.pathname.split("/").pop() ?? "").replace(IMAGE_PATH_RE, "");
        stem = stem.replace(/^\d{3,}-/, "");
        const at = stem.lastIndexOf("@");
        const title = (at > 0 ? stem.slice(0, at) : stem) || "untitled";
        const author = at > 0 ? stem.slice(at + 1) : "";
        const referer = /(^|\.)pximg\.net$/i.test(u.hostname) ? PIXIV_REFERER : undefined;
        return [{ imageUrl: u.hostname === "pbs.twimg.com" ? origTwimg(url) : url, title, author, sourceUrl: url, ...(referer ? { referer } : {}) }];
    },
};

/** 按顺序匹配: pixiv → x → 直链 */
export const kyaraResolvers: KyaraSourceResolver[] = [pixivResolver, xResolver, directResolver];

export function findKyaraResolver(url: string): KyaraSourceResolver | null {
    const u = url.trim();
    return kyaraResolvers.find((r) => r.match(u)) ?? null;
}

/** 一条链接 → 导入条目; 没有解析器能处理时抛 unsupported */
export async function resolveKyaraUrl(url: string, fetchImpl?: typeof fetch): Promise<KyaraImportItem[]> {
    const u = url.trim();
    const r = findKyaraResolver(u);
    if (!r) throw new KyaraResolveError("unsupported", `不支持的链接: ${u} (支持 pixiv 作品 / x.com 推文 / 图片直链)`);
    return r.resolve(u, fetchImpl);
}
