import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { KyaraImage, KyaraImportItem, KyaraIndexEntry, KyaraLocalCharacter, KyaraLocalStore, KyaraMeta } from "./types";
import { formatKyaraFilename, isValidKyaraName, kyaraExtOf, parseKyaraFilename } from "./filename";
import { resolveKyaraUrl } from "./resolvers";

// 本地模式: 浏览器 IndexedDB 库 `hx-kyara`, 线上与 dev 都可用.
//   - images 仓 (keyPath "key" = `${name}/${id}`) 存 Blob 与元数据, order 仓 (keyPath "name") 存每个角色的本地顺序,
//     characters 仓 (keyPath "name") 登记纯本地角色 (仓库里还没有目录的角色), 索引页与右键菜单据此列出它们.
//   - 本地条目的序号不落库, 由 "仓库最大序号 + 本地位次" 推算, 所以永远排在仓库图片之后, exportZip / 写进仓库时用的就是这个序号.
//   - 开关只有一个 localStorage 键 MODE_KEY, 全站共用; 同标签页经自定义事件、跨标签页经 storage 事件同步.
//   - 写入仓库的 dev 通道不在这里, 见 src/components/anime/kyara/devChunks.ts (生产构建里整段被裁掉).

const DB_NAME = "hx-kyara";
const DB_VERSION = 2;
const IMAGES = "images";
const ORDER = "order";
const CHARACTERS = "characters";
const MODE_KEY = "hx-kyara-local";
const LEGACY_MODE_KEYS = ["hx-kyara-local-enabled", "hx-kyara-local-index"];
const MODE_EVENT = "hx-kyara-local-mode";
const DATA_EVENT = "hx-kyara-local-data";

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
    if (!hasIdb()) return Promise.reject(new Error("当前浏览器不支持 IndexedDB"));
    if (!dbPromise) {
        dbPromise = new Promise<IDBDatabase>((resolve, reject) => {
            const req = indexedDB.open(DB_NAME, DB_VERSION);
            req.onupgradeneeded = () => {
                const db = req.result;
                if (!db.objectStoreNames.contains(IMAGES)) db.createObjectStore(IMAGES, { keyPath: "key" }).createIndex("byName", "name");
                if (!db.objectStoreNames.contains(ORDER)) db.createObjectStore(ORDER, { keyPath: "name" });
                if (!db.objectStoreNames.contains(CHARACTERS)) db.createObjectStore(CHARACTERS, { keyPath: "name" });
            };
            req.onsuccess = () => {
                const db = req.result;
                db.onversionchange = () => { db.close(); dbPromise = null; };
                resolve(db);
            };
            req.onerror = () => { dbPromise = null; reject(req.error); };
            req.onblocked = () => { dbPromise = null; reject(new Error("本地库被其他标签页占用, 关掉旧标签页后重试")); };
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

/** 通知同页其他订阅者 (索引页列表、右键菜单、角色页) 本地数据变了 */
function emitData(name: string): void {
    if (typeof window !== "undefined") window.dispatchEvent(new CustomEvent(DATA_EVENT, { detail: { name } }));
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

/** kyara.json 的内容: 只留有值的白名单字段 */
export function kyaraMetaJson(meta: Partial<KyaraMeta>): Record<string, unknown> {
    const out: Record<string, unknown> = {};
    if (meta.displayName) out.displayName = meta.displayName;
    if (meta.work) out.work = meta.work;
    if (Number.isInteger(meta.bangumiCharacterId)) out.bangumiCharacterId = meta.bangumiCharacterId;
    if (meta.bangumiSubjectIds && meta.bangumiSubjectIds.length > 0) out.bangumiSubjectIds = meta.bangumiSubjectIds;
    if (meta.accent) out.accent = meta.accent;
    if (meta.cover) out.cover = meta.cover;
    return out;
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
            emitData(name);
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
            emitData(name);
        },
        async reorder(name, files) {
            const rows = await readRows(name);
            const byFile = new Map(rows.map((r) => [localFile(r), r] as const));
            const picked = files.map((f) => byFile.get(f)).filter((r): r is LocalRow => !!r);
            const rest = rows.filter((r) => !picked.includes(r));
            await writeOrder(name, [...picked, ...rest].map((r) => r.id));
            emitData(name);
        },
        async exportZip(name, meta) {
            const base = getStaticBase(name);
            const rows = await readRows(name);
            const entries = rows.map((r, i) => ({ key: localFile(r), name: kyaraLocalFilename(toImage(r, base + i + 1, "")), blob: r.blob }));
            const out: { name: string; blob: Blob }[] = entries.map(({ name: n, blob }) => ({ name: n, blob }));
            if (meta) {
                const cover = meta.cover ? entries.find((e) => e.key === meta.cover)?.name ?? meta.cover : undefined;
                const json = kyaraMetaJson({ ...meta, cover });
                if (Object.keys(json).length > 0) {
                    out.push({ name: "kyara.json", blob: new Blob([`${JSON.stringify(json, null, 2)}\n`], { type: "application/json" }) });
                }
            }
            return buildZip(out);
        },
    };
}

// ── 纯本地角色 (characters 仓) ──

export async function listLocalCharacters(): Promise<KyaraLocalCharacter[]> {
    if (!hasIdb()) return [];
    const db = await openDb();
    const tx = db.transaction(CHARACTERS, "readonly");
    const all = await reqP<KyaraLocalCharacter[]>(tx.objectStore(CHARACTERS).getAll());
    return all.sort((a, b) => a.displayName.localeCompare(b.displayName, "zh"));
}

export async function getLocalCharacter(name: string): Promise<KyaraLocalCharacter | null> {
    if (!hasIdb()) return null;
    const db = await openDb();
    const tx = db.transaction(CHARACTERS, "readonly");
    return (await reqP<KyaraLocalCharacter | undefined>(tx.objectStore(CHARACTERS).get(name))) ?? null;
}

/** 新建或更新纯本地角色; 新建时 createdAt 取当前时间 */
export async function putLocalCharacter(c: Omit<KyaraLocalCharacter, "createdAt"> & { createdAt?: string }): Promise<KyaraLocalCharacter> {
    if (!isValidKyaraName(c.name)) throw new Error(`角色名不合法: ${c.name}`);
    const prev = await getLocalCharacter(c.name);
    const row: KyaraLocalCharacter = { ...prev, ...c, displayName: c.displayName || c.name, createdAt: prev?.createdAt ?? c.createdAt ?? new Date().toISOString() };
    const db = await openDb();
    const tx = db.transaction(CHARACTERS, "readwrite");
    tx.objectStore(CHARACTERS).put(row);
    await txDone(tx);
    emitData(c.name);
    return row;
}

/** 删除纯本地角色及其全部本地图 */
export async function removeLocalCharacter(name: string): Promise<void> {
    const rows = hasIdb() ? await readRows(name) : [];
    const db = await openDb();
    const tx = db.transaction([CHARACTERS, IMAGES, ORDER], "readwrite");
    tx.objectStore(CHARACTERS).delete(name);
    for (const r of rows) tx.objectStore(IMAGES).delete(r.key);
    tx.objectStore(ORDER).delete(name);
    await txDone(tx);
    emitData(name);
}

/** 给本地角色补一个关联条目 (从另一季的详情页打开时), 已有则不动 */
export async function linkLocalCharacterSubject(name: string, subjectId: number): Promise<void> {
    const c = await getLocalCharacter(name);
    if (!c || (c.bangumiSubjectIds ?? []).includes(subjectId)) return;
    await putLocalCharacter({ ...c, bangumiSubjectIds: [...(c.bangumiSubjectIds ?? []), subjectId] });
}

/** 解析一条链接并加入本地库; pixiv 在浏览器里拿不到会抛 code=node-only 的错误 */
export async function importKyaraLinkLocal(name: string, url: string, store: KyaraLocalStore): Promise<KyaraImage[]> {
    return store.add(name, await resolveKyaraUrl(url));
}

// ── hooks ──

function readMode(): boolean {
    try {
        const ls = window.localStorage;
        const v = ls.getItem(MODE_KEY);
        if (v !== null) return v === "1";
        // 兼容早先分开存的两个键: 任一为 1 视为开启, 迁移到 MODE_KEY 后清掉
        const on = LEGACY_MODE_KEYS.some((k) => ls.getItem(k) === "1");
        ls.setItem(MODE_KEY, on ? "1" : "0");
        for (const k of LEGACY_MODE_KEYS) ls.removeItem(k);
        return on;
    } catch {
        return false;
    }
}

/** 本地模式开关: 全站一个键, SSR / 首帧为 false, 挂载后读 localStorage; 各处实例同步 */
export function useKyaraLocalMode(): [boolean, (v: boolean) => void] {
    const [on, setOn] = useState(false);
    useEffect(() => {
        setOn(readMode());
        const sync = (): void => setOn(readMode());
        const onStorage = (e: StorageEvent): void => { if (e.key === MODE_KEY) sync(); };
        window.addEventListener(MODE_EVENT, sync);
        window.addEventListener("storage", onStorage);
        return () => {
            window.removeEventListener(MODE_EVENT, sync);
            window.removeEventListener("storage", onStorage);
        };
    }, []);
    const set = useCallback((v: boolean) => {
        setOn(v);
        try { window.localStorage.setItem(MODE_KEY, v ? "1" : "0"); } catch { /* 隐私模式 */ }
        window.dispatchEvent(new Event(MODE_EVENT));
    }, []);
    return [on, set];
}

/** 订阅本地数据变化 (name 省略 = 任意角色) */
function useLocalDataTick(name?: string): number {
    const [tick, setTick] = useState(0);
    useEffect(() => {
        const on = (e: Event): void => {
            const n = (e as CustomEvent<{ name: string }>).detail?.name;
            if (!name || n === name) setTick((t) => t + 1);
        };
        window.addEventListener(DATA_EVENT, on);
        return () => window.removeEventListener(DATA_EVENT, on);
    }, [name]);
    return tick;
}

export interface KyaraLocalApi {
    /** 是否在读写本地库 (本地模式开启, 或这是纯本地角色) */
    active: boolean;
    images: KyaraImage[];
    store: KyaraLocalStore;
    /** 纯本地角色的登记信息; 仓库角色为 null */
    character: KyaraLocalCharacter | null;
    loaded: boolean;
}

/**
 * 角色页的本地数据. active 为 false 时不读库, images 为空.
 * staticMax = 仓库里该角色的最大序号, 本地条目接着往后排.
 */
export function useKyaraLocal(name: string, active: boolean, staticMax: number): KyaraLocalApi {
    const [images, setImages] = useState<KyaraImage[]>([]);
    const [character, setCharacter] = useState<KyaraLocalCharacter | null>(null);
    const [loaded, setLoaded] = useState(false);
    const baseRef = useRef(staticMax);
    baseRef.current = staticMax;
    const urlsRef = useRef<Map<string, string>>(new Map());
    const tick = useLocalDataTick(name);

    const store = useMemo(() => createKyaraLocalStore(() => baseRef.current, urlsRef.current), []);

    useEffect(() => {
        if (!active || !hasIdb()) {
            setImages([]);
            setCharacter(null);
            setLoaded(true);
            return undefined;
        }
        let alive = true;
        (async () => {
            try {
                const [imgs, c] = await Promise.all([store.list(name), getLocalCharacter(name)]);
                if (alive) { setImages(imgs); setCharacter(c); }
            } catch {
                if (alive) { setImages([]); setCharacter(null); }
            } finally {
                if (alive) setLoaded(true);
            }
        })();
        return () => { alive = false; };
    }, [active, name, staticMax, store, tick]);

    useEffect(() => () => {
        for (const u of urlsRef.current.values()) URL.revokeObjectURL(u);
        urlsRef.current.clear();
    }, []);

    return { active, images, store, character, loaded };
}

/** 本地角色 → 索引条目 (封面 / 预览是 blob: URL, 由 useKyaraLocalCharacters 回收) */
function entryOf(c: KyaraLocalCharacter, imgs: KyaraImage[]): KyaraIndexEntry {
    const cover = (c.cover && imgs.find((i) => i.file === c.cover)?.src) || imgs[0]?.src || null;
    return {
        meta: {
            name: c.name, displayName: c.displayName,
            ...(c.work ? { work: c.work } : {}),
            ...(c.bangumiCharacterId ? { bangumiCharacterId: c.bangumiCharacterId } : {}),
            ...(c.bangumiSubjectIds?.length ? { bangumiSubjectIds: c.bangumiSubjectIds } : {}),
            ...(c.accent ? { accent: c.accent } : {}),
        },
        count: imgs.length,
        cover,
        authors: new Set(imgs.map((i) => i.author).filter(Boolean)).size,
        previews: imgs.slice(0, 4).map((i) => i.src),
        local: true,
    };
}

/** 本地模式开启时列出全部纯本地角色 (带封面与张数); 关闭时为空 */
export function useKyaraLocalCharacters(active: boolean): { characters: KyaraIndexEntry[]; loaded: boolean } {
    const [characters, setCharacters] = useState<KyaraIndexEntry[]>([]);
    const [loaded, setLoaded] = useState(false);
    const urlsRef = useRef<Map<string, string>>(new Map());
    const tick = useLocalDataTick();
    const store = useMemo(() => createKyaraLocalStore(() => 0, urlsRef.current), []);

    useEffect(() => {
        if (!active || !hasIdb()) {
            setCharacters([]);
            setLoaded(true);
            return undefined;
        }
        let alive = true;
        (async () => {
            try {
                const list = await listLocalCharacters();
                const out: KyaraIndexEntry[] = [];
                for (const c of list) out.push(entryOf(c, await store.list(c.name)));
                if (alive) setCharacters(out);
            } catch {
                if (alive) setCharacters([]);
            } finally {
                if (alive) setLoaded(true);
            }
        })();
        return () => { alive = false; };
    }, [active, store, tick]);

    useEffect(() => () => {
        for (const u of urlsRef.current.values()) URL.revokeObjectURL(u);
        urlsRef.current.clear();
    }, []);

    return { characters, loaded };
}
