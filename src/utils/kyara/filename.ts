import type { KyaraImage } from "./types";

// 本文件也被 Node 侧 (plugins/kyara-gallery-plugin.mjs, scripts/kyara-import.mjs) 直接 import:
// Node 原生 type stripping 只擦除类型, 所以这里只许有 `import type`, 不许有运行时 import / enum / 参数属性.

/** NNN-标题@作者.ext; 标题惰性匹配 + 作者不含 @, 等价于按最后一个 @ 拆 */
export const KYARA_FILE_RE = /^(\d{3,})-(.+?)(?:@([^@]+))?\.(png|jpe?g|webp|gif|avif)$/i;

export const KYARA_IMAGE_EXTS = ["png", "jpg", "jpeg", "webp", "gif", "avif"] as const;

const FULLWIDTH: Record<string, string> = {
    "/": "／", "\\": "＼", ":": "：", "*": "＊", "?": "？", "\"": "＂", "<": "＜", ">": "＞", "|": "｜", "@": "＠",
};

export function parseKyaraFilename(file: string): Omit<KyaraImage, "src"> | null {
    const m = KYARA_FILE_RE.exec(file);
    if (!m) return null;
    const author = m[3] !== undefined ? m[3].trim() : "";
    return { seq: Number.parseInt(m[1], 10), title: m[2], author: author ? author : null, file };
}

function cleanPart(s: string): string {
    return s
        .replace(/[\u0000-\u001f\u007f]/g, " ")
        .replace(/[/\\:*?"<>|@]/g, (c) => FULLWIDTH[c] ?? c)
        .replace(/\s+/g, " ")
        .trim()
        .slice(0, 80)
        .replace(/\.+$/, "");
}

/** 非法字符 (/\:*?"<>| 与 @) 替换为全角; 序号至少 3 位 */
export function formatKyaraFilename(seq: number, title: string, author: string | null, ext: string): string {
    const n = Math.max(0, Math.floor(seq));
    const t = cleanPart(title) || "untitled";
    const a = author == null ? "" : cleanPart(author.replace(/^@+/, ""));
    let e = ext.replace(/^\./, "").toLowerCase();
    if (!(KYARA_IMAGE_EXTS as readonly string[]).includes(e)) e = "png";
    return `${String(n).padStart(3, "0")}-${t}${a ? `@${a}` : ""}.${e}`;
}

/** 序号数值升序, 序号相同按文件名 */
export function compareKyaraImages(a: { seq: number; file: string }, b: { seq: number; file: string }): number {
    return a.seq - b.seq || (a.file < b.file ? -1 : a.file > b.file ? 1 : 0);
}

export function nextKyaraSeq(images: { seq: number }[]): number {
    return images.reduce((m, i) => Math.max(m, i.seq), 0) + 1;
}

/** 保留段: /anime/kyara/local 是纯本地角色页的路由 */
export const KYARA_RESERVED_NAMES: readonly string[] = ["local"];

/** 角色目录名 = URL 段: 允许中日文, 不许路径分隔符 / 控制字符 / 以点开头 / 保留段, 最长 64 */
export function isValidKyaraName(name: string): boolean {
    return name.length > 0 && name.length <= 64 && !/[/\\\u0000-\u001f\u007f]/.test(name) && !name.startsWith(".") && !KYARA_RESERVED_NAMES.includes(name.toLowerCase());
}

/** 由显示名推目录名: 去掉 / \\ ? # % 与控制字符, 去开头的点, 空白压成一个, 截到 64; 推不出返回空串 */
export function kyaraNameFrom(displayName: string): string {
    const s = displayName
        .replace(/[/\\?#%\u0000-\u001f\u007f]/g, "")
        .replace(/\s+/g, " ")
        .trim()
        .replace(/^\.+/, "")
        .slice(0, 64)
        .trim();
    return isValidKyaraName(s) ? s : "";
}

/** 从 Content-Type 或 URL 推扩展名, 推不出返回 null */
export function kyaraExtOf(contentType: string | null | undefined, url?: string): string | null {
    const ct = (contentType ?? "").toLowerCase();
    const byCt: Record<string, string> = {
        "image/png": "png", "image/jpeg": "jpg", "image/jpg": "jpg", "image/webp": "webp", "image/gif": "gif", "image/avif": "avif",
    };
    for (const k of Object.keys(byCt)) if (ct.startsWith(k)) return byCt[k];
    if (url) {
        try {
            const u = new URL(url);
            const fmt = u.searchParams.get("format");
            if (fmt && (KYARA_IMAGE_EXTS as readonly string[]).includes(fmt.toLowerCase())) return fmt.toLowerCase();
            const m = /\.([a-z0-9]{3,4})$/i.exec(u.pathname);
            if (m && (KYARA_IMAGE_EXTS as readonly string[]).includes(m[1].toLowerCase())) return m[1].toLowerCase();
        } catch {
            /* 不是 URL */
        }
    }
    return null;
}
