import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import type { KyaraData, KyaraImage, KyaraImportItem, KyaraLocalStore } from "./types";
import { compareKyaraImages, formatKyaraFilename, isValidKyaraName, kyaraExtOf, nextKyaraSeq, parseKyaraFilename } from "./filename";
import { resolveKyaraUrl } from "./resolvers";

// 本地模式: 浏览器 IndexedDB 库 `hx-kyara`, 线上也能用.
//   - images 仓 (keyPath "key" = `${name}/${id}`) 存 Blob 与元数据, order 仓 (keyPath "name") 存每个角色的本地顺序.
//   - 本地条目的序号不落库, 由 "仓库最大序号 + 本地位次" 推算, 所以永远排在仓库图片之后, exportZip / 写进仓库时用的就是这个序号.
//   - dev (NODE_ENV=development 且本机 host) 下 serverWritable = true, 可经 plugins/kyara-gallery-plugin.mjs 的写入通道直接落盘.

const DB_NAME = "hx-kyara";
const DB_VERSION = 1;
const IMAGES = "images";
const ORDER = "order";
const ENABLED_KEY = "hx-kyara-local-enabled";
const KYARA_HEADER = "X-HX-Kyara";

interface LocalRow {
    key: string;
    name: string;
    id: string;
    title: string;
    author: string | null;
    ext: string;
    blob: Blob;
    sourceUrl?: string;
    w?: number;
    h?: number;
    addedAt: string;
}

function hasIdb(): boolean {
    return typeof indexedDB !== "undefined";
}

let dbPromise: Promise<IDBDatabase> | null = null;
function openDb(): Promise<IDBDatabase> {
    if (!hasIdb()) return Promise.reject(new Error("当前环境没有 IndexedDB"));
    if (!dbPromise) {
        dbPromise = new Promise<IDBDatabase>((resolve, reject) => {
            const req = indexedDB.open(DB_NAME, DB_VERSION);
            req.onupgradeneeded = () => {
                const db = req.result;
                if (!db.objectStoreNames.contains(IMAGES)) db.createObjectStore(IMAGES, { keyPath: "key" }).createIndex("byName", "name");
                if (!db.objectStoreNames.contains(ORDER)) db.createObjectStore(ORDER, { keyPath: "name" });
            };
            req.onsuccess = () => resolve(req.result);
            req.onerror = () => { dbPromise = null; reject(req.error); };
        });
    }
    return dbPromise;
}

function reqP<T>(r: IDBRequest<T>): Promise<T> {
    return new Promise((resolve, reject) => { r.onsuccess = () => resolve(r.result); r.onerror = () => reject(r.error); });
}

function txDone(tx: IDBTransaction): Promise<void> {
    return new Promise((resolve, reject) => { tx.oncomplete = () => resolve(); tx.onerror = () => reject(tx.error); tx.onabort = () => reject(tx.error); });
}

function newId(): string {
    const c = (globalThis as any).crypto;
    if (c && typeof c.randomUUID === "function") return c.randomUUID().replace(/-/g, "").slice(0, 12);
    return `${Date.now().toString(36)}${Math.random().toString(36).slice(2, 8)}`;
}

async function readRows(name: string): Promise<LocalRow[]> {
    const db = await openDb();
    const tx = db.transaction([IMAGES, ORDER], "readonly");
    const rows = await reqP<LocalRow[]>(tx.objectStore(IMAGES).index("byName").getAll(name));
    const ord = await reqP<{ name: string; ids: string[] } | undefined>(tx.objectStore(ORDER).get(name));
    const pos = new Map((ord?.ids ?? []).map((id, i) => [id, i] as const));
    return rows.sort((a, b) => (pos.get(a.id) ?? 1e9) - (pos.get(b.id) ?? 1e9) || a.addedAt.localeCompare(b.addedAt));
}

async function writeOrder(name: string, ids: string[]): Promise<void> {
    const db = await openDb();
    const tx = db.transaction(ORDER, "readwrite");
    tx.objectStore(ORDER).put({ name, ids });
    await txDone(tx);
}

function localFile(row: LocalRow): string {
    return `local-${row.id}.${row.ext}`;
}

async function measure(blob: Blob): Promise<{ w?: number; h?: number }> {
    try {
        if (typeof createImageBitmap !== "function") return {};
        const bmp = await createImageBitmap(blob);
        const out = { w: bmp.width, h: bmp.height };
        bmp.close();
        return out;
    } catch {
        return {};
    }
}

async function rowFromFile(name: string, f: File): Promise<LocalRow> {
    const ext = kyaraExtOf(f.type, `https://x/${encodeURIComponent(f.name)}`) ?? "png";
    const parsed = parseKyaraFilename(f.name);
    const stem = f.name.replace(/\.[^.]+$/, "");
    const id = newId();
    return {
        key: `${name}/${id}`, name, id, ext, blob: f, addedAt: new Date().toISOString(),
        title: parsed ? parsed.title : stem || "untitled",
        author: parsed ? parsed.author : null,
        ...(await measure(f)),
    };
}

async function rowFromItem(name: string, item: KyaraImportItem): Promise<LocalRow> {
    const res = await fetch(item.imageUrl, { referrerPolicy: "no-referrer" });
    if (!res.ok) throw new Error(`下载失败 HTTP ${res.status}: ${item.imageUrl}`);
    const blob = await res.blob();
    const ext = kyaraExtOf(blob.type, item.imageUrl);
    if (!ext) throw new Error(`不是图片: ${item.imageUrl}`);
    const id = newId();
    return {
        key: `${name}/${id}`, name, id, ext, blob, addedAt: new Date().toISOString(),
        title: item.title || "untitled",
        author: item.author ? item.author.replace(/^@+/, "") : null,
        sourceUrl: item.sourceUrl,
        ...(await measure(blob)),
    };
}

function isFileList(x: File[] | KyaraImportItem[]): x is File[] {
    return x.length > 0 && typeof File !== "undefined" && x[0] instanceof File;
}

/** 本地条目 → KyaraImage; seq 从 base 之后依次排; src 由调用方提供 (blob: URL) */
function toImage(row: LocalRow, seq: number, src: string): KyaraImage {
    return {
        seq, title: row.title, author: row.author, file: localFile(row), src, source: "local",
        ...(row.sourceUrl ? { sourceUrl: row.sourceUrl } : {}),
        ...(row.w ? { w: row.w } : {}), ...(row.h ? { h: row.h } : {}),
    };
}

/** 合规文件名 (仓库命名规则), 供 exportZip 与写进仓库 */
export function kyaraLocalFilename(img: KyaraImage): string {
    const ext = /\.([a-z0-9]+)$/i.exec(img.file)?.[1] ?? "png";
    return formatKyaraFilename(img.seq, img.title, img.author, ext);
}

// ── 最小 zip (STORE, 不压缩; 图片本身已压缩) ──
const CRC_TABLE = (() => {
    const t = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
        let c = n;
        for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
        t[n] = c >>> 0;
    }
    return t;
})();
function crc32(buf: Uint8Array): number {
    let c = 0xffffffff;
    for (let i = 0; i < buf.length; i++) c = CRC_TABLE[(c ^ buf[i]) & 0xff] ^ (c >>> 8);
    return (c ^ 0xffffffff) >>> 0;
}
async function buildZip(entries: { name: string; blob: Blob }[]): Promise<Blob> {
    const enc = new TextEncoder();
    const parts: BlobPart[] = [];
    const central: Uint8Array[] = [];
    let offset = 0;
    const now = new Date();
    const dosTime = (now.getHours() << 11) | (now.getMinutes() << 5) | (now.getSeconds() >> 1);
    const dosDate = ((now.getFullYear() - 1980) << 9) | ((now.getMonth() + 1) << 5) | now.getDate();
    for (const e of entries) {
        const data = new Uint8Array(await e.blob.arrayBuffer());
        const nameBytes = enc.encode(e.name);
        const crc = crc32(data);
        const local = new DataView(new ArrayBuffer(30));
        local.setUint32(0, 0x04034b50, true); local.setUint16(4, 20, true); local.setUint16(6, 0x0800, true);
        local.setUint16(8, 0, true); local.setUint16(10, dosTime, true); local.setUint16(12, dosDate, true);
        local.setUint32(14, crc, true); local.setUint32(18, data.length, true); local.setUint32(22, data.length, true);
        local.setUint16(26, nameBytes.length, true); local.setUint16(28, 0, true);
        parts.push(local.buffer, nameBytes, data);
        const cen = new DataView(new ArrayBuffer(46));
        cen.setUint32(0, 0x02014b50, true); cen.setUint16(4, 20, true); cen.setUint16(6, 20, true); cen.setUint16(8, 0x0800, true);
        cen.setUint16(10, 0, true); cen.setUint16(12, dosTime, true); cen.setUint16(14, dosDate, true);
        cen.setUint32(16, crc, true); cen.setUint32(20, data.length, true); cen.setUint32(24, data.length, true);
        cen.setUint16(28, nameBytes.length, true); cen.setUint32(42, offset, true);
        const c = new Uint8Array(46 + nameBytes.length);
        c.set(new Uint8Array(cen.buffer), 0); c.set(nameBytes, 46);
        central.push(c);
        offset += 30 + nameBytes.length + data.length;
    }
    const cenSize = central.reduce((s, c) => s + c.length, 0);
    const end = new DataView(new ArrayBuffer(22));
    end.setUint32(0, 0x06054b50, true); end.setUint16(8, entries.length, true); end.setUint16(10, entries.length, true);
    end.setUint32(12, cenSize, true); end.setUint32(16, offset, true);
    return new Blob([...parts, ...central, end.buffer], { type: "application/zip" });
}

/**
 * IndexedDB 版本地库. staticBase = 仓库里该角色的最大序号, 本地条目从 staticBase+1 往后排.
 * src 用 blob: URL; urls 由调用方 (useKyaraLocal) 统一回收.
 */
export function createKyaraLocalStore(getStaticBase: (name: string) => number = () => 0, urls?: Map<string, string>): KyaraLocalStore {
    const urlOf = (row: LocalRow): string => {
        const cached = urls?.get(row.key);
        if (cached) return cached;
        const u = URL.createObjectURL(row.blob);
        urls?.set(row.key, u);
        return u;
    };
    const list = async (name: string): Promise<KyaraImage[]> => {
        if (!hasIdb()) return [];
        const base = getStaticBase(name);
        return (await readRows(name)).map((r, i) => toImage(r, base + i + 1, urlOf(r)));
    };
    return {
        list,
        async add(name, files) {
            if (!isValidKyaraName(name)) throw new Error(`角色名不合法: ${name}`);
            if (files.length === 0) return list(name);
            const rows: LocalRow[] = [];
            if (isFileList(files)) {
                for (const f of files) if (kyaraExtOf(f.type, `https://x/${encodeURIComponent(f.name)}`)) rows.push(await rowFromFile(name, f));
            } else {
                for (const it of files as KyaraImportItem[]) rows.push(await rowFromItem(name, it));
            }
            const before = await readRows(name);
            const db = await openDb();
            const tx = db.transaction(IMAGES, "readwrite");
            for (const r of rows) tx.objectStore(IMAGES).put(r);
            await txDone(tx);
            await writeOrder(name, [...before.map((r) => r.id), ...rows.map((r) => r.id)]);
            return list(name);
        },
        async remove(name, file) {
            const rows = await readRows(name);
            const row = rows.find((r) => localFile(r) === file);
            if (!row) return;
            const db = await openDb();
            const tx = db.transaction(IMAGES, "readwrite");
            tx.objectStore(IMAGES).delete(row.key);
            await txDone(tx);
            const u = urls?.get(row.key);
            if (u) { URL.revokeObjectURL(u); urls?.delete(row.key); }
            await writeOrder(name, rows.filter((r) => r !== row).map((r) => r.id));
        },
        async reorder(name, files) {
            const rows = await readRows(name);
            const byFile = new Map(rows.map((r) => [localFile(r), r] as const));
            const picked = files.map((f) => byFile.get(f)).filter((r): r is LocalRow => !!r);
            const rest = rows.filter((r) => !picked.includes(r));
            await writeOrder(name, [...picked, ...rest].map((r) => r.id));
        },
        async exportZip(name) {
            const base = getStaticBase(name);
            const rows = await readRows(name);
            return buildZip(rows.map((r, i) => ({ name: kyaraLocalFilename(toImage(r, base + i + 1, "")), blob: r.blob })));
        },
    };
}

// ── dev 写入通道 (plugins/kyara-gallery-plugin.mjs), 只在本机 dev 下可用 ──

function devServerAvailable(): boolean {
    if (process.env.NODE_ENV !== "development" || typeof window === "undefined") return false;
    const h = window.location.hostname;
    return h === "localhost" || h === "127.0.0.1" || h === "[::1]" || h === "::1";
}

async function devJson<T>(url: string, init: RequestInit): Promise<T> {
    const res = await fetch(url, init);
    const data = await res.json().catch(() => ({ ok: false, error: `HTTP ${res.status}` }));
    if (!res.ok || !data?.ok) throw new Error(data?.error || `HTTP ${res.status}`);
    return data as T;
}

/** dev 下的仓库写入: import (服务端解析 + 下载, pixiv 走这里), putFile (本地文件提交), reorder (重排序号) */
export const kyaraServer = {
    importUrl(name: string, url: string): Promise<{ ok: true; images: KyaraImage[] }> {
        return devJson(`/__hx/kyara/${encodeURIComponent(name)}/import`, {
            method: "POST", headers: { "Content-Type": "application/json", [KYARA_HEADER]: "1" }, body: JSON.stringify({ url }),
        });
    },
    putFile(name: string, filename: string, blob: Blob): Promise<{ ok: true; image: KyaraImage }> {
        return devJson(`/__hx/kyara/${encodeURIComponent(name)}/file/${encodeURIComponent(filename)}`, {
            method: "PUT", headers: { "Content-Type": blob.type || "application/octet-stream", [KYARA_HEADER]: "1" }, body: blob,
        });
    },
    reorder(name: string, order: string[]): Promise<{ ok: true; images: KyaraImage[] }> {
        return devJson(`/__hx/kyara/${encodeURIComponent(name)}/reorder`, {
            method: "POST", headers: { "Content-Type": "application/json", [KYARA_HEADER]: "1" }, body: JSON.stringify({ order }),
        });
    },
};

/** 把本地条目逐个写进仓库 (dev), 成功的从本地库删掉; 返回写入后的仓库条目 */
export async function commitKyaraLocal(name: string, store: KyaraLocalStore): Promise<KyaraImage[]> {
    const done: KyaraImage[] = [];
    for (const img of await store.list(name)) {
        const blob = await (await fetch(img.src)).blob();
        const r = await kyaraServer.putFile(name, kyaraLocalFilename(img), blob);
        done.push(r.image);
        await store.remove(name, img.file);
    }
    return done;
}

/** 解析一条链接并加入本地库; pixiv 在浏览器里拿不到会抛 code=node-only 的错误, 由 UI 提示去 dev 下导入 */
export async function importKyaraLinkLocal(name: string, url: string, store: KyaraLocalStore): Promise<KyaraImage[]> {
    return store.add(name, await resolveKyaraUrl(url));
}

/**
 * 角色页的本地模式状态. enabled 存 localStorage (全站一个开关); images 只含本地条目, 序号接在仓库条目之后.
 * 仓库最大序号从插件输出的运行时清单 `${baseUrl}kyara/data/{name}.json` 读; 读不到按 0.
 */
export function useKyaraLocal(name: string): { enabled: boolean; setEnabled(v: boolean): void; images: KyaraImage[]; store: KyaraLocalStore; serverWritable: boolean } {
    const { siteConfig } = useDocusaurusContext();
    const baseUrl = siteConfig.baseUrl;
    const [enabled, setEnabledState] = useState(false);
    const [images, setImages] = useState<KyaraImage[]>([]);
    const [serverWritable, setServerWritable] = useState(false);
    const baseRef = useRef<Map<string, number>>(new Map());
    const urlsRef = useRef<Map<string, string>>(new Map());

    useEffect(() => {
        try { setEnabledState(window.localStorage.getItem(ENABLED_KEY) === "1"); } catch { /* 隐私模式 */ }
        setServerWritable(devServerAvailable());
    }, []);

    const setEnabled = useCallback((v: boolean) => {
        setEnabledState(v);
        try { window.localStorage.setItem(ENABLED_KEY, v ? "1" : "0"); } catch { /* 隐私模式 */ }
    }, []);

    const raw = useMemo(() => createKyaraLocalStore((n) => baseRef.current.get(n) ?? 0, urlsRef.current), []);
    const refresh = useCallback(async () => {
        try { setImages(await raw.list(name)); } catch { setImages([]); }
    }, [raw, name]);

    // 包一层: 任何写操作后刷新 images
    const store = useMemo<KyaraLocalStore>(() => ({
        list: raw.list,
        exportZip: raw.exportZip,
        async add(n, files) { const r = await raw.add(n, files); if (n === name) await refresh(); return r; },
        async remove(n, file) { await raw.remove(n, file); if (n === name) await refresh(); },
        async reorder(n, files) { await raw.reorder(n, files); if (n === name) await refresh(); },
    }), [raw, name, refresh]);

    useEffect(() => {
        if (!enabled) { setImages([]); return; }
        let alive = true;
        (async () => {
            if (!baseRef.current.has(name)) {
                try {
                    const res = await fetch(`${baseUrl}kyara/data/${encodeURIComponent(name)}.json`);
                    const data: KyaraData | null = res.ok ? await res.json() : null;
                    baseRef.current.set(name, data ? nextKyaraSeq([...data.images].sort(compareKyaraImages)) - 1 : 0);
                } catch {
                    baseRef.current.set(name, 0);
                }
            }
            if (alive) await refresh();
        })();
        return () => { alive = false; };
    }, [enabled, name, baseUrl, refresh]);

    useEffect(() => () => {
        for (const u of urlsRef.current.values()) URL.revokeObjectURL(u);
        urlsRef.current.clear();
    }, []);

    return { enabled, setEnabled, images, store, serverWritable };
}
