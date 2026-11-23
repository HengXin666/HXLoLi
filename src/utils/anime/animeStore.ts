import { parse } from "@site/src/utils/anime/parser";
import { Actor, ANiMeRecord } from "@site/src/utils/anime/types";
import { useState, useEffect, useCallback } from "react";
import { fetchAnimeDataText } from '@site/src/utils/anime/dataCache'
import { ANIME_OVERLAY, AnimeOverlay, AnimeOverlayEntry, applyOverlay, applyOverlayEntry } from "@site/src/utils/anime/overlay";
import { setAnimeLabels } from "@site/src/utils/anime/tags";
import { fetchAnimeLabels } from "@site/src/utils/anime/labels";

/** CDN 原始记录 (未盖 overlay) */
let baseRecords: readonly ANiMeRecord[] | null = null;
/** 盖过本地 overlay 的记录, 页面读的是这份. 本地标注模式的决策见 src/utils/anime/overlay.ts 顶部注释 */
let cachedRecords: readonly ANiMeRecord[] | null = null;
/** 当前生效的 overlay. 初值是打进包里的 data/animeOverlay.json; dev 编辑器保存后经 setOverlayEntry 就地更新 */
let liveOverlay: AnimeOverlay = { ...ANIME_OVERLAY };
/** 记录有变 (overlay 改动 / 标注到达) 时通知在用的 hook, 各 hook 按自己的范围重新读取 */
const listeners = new Set<() => void>();
const notify = () => listeners.forEach((fn) => fn());

/**
 * 轻量列表 data/ANiMeIndex.json: 没有角色, 每集只有类型和时长等少数字段, gzip 约为全量的八分之一.
 * 列表页 / 首页 / 系列页 / 详情页的相似推荐只要它; 声优页和关系图要角色, 仍读全量.
 * 全量已经在内存里时, 轻量范围直接用全量 (字段是超集).
 */
export type AnimeRecordScope = "full" | "lite";
let liteBase: readonly ANiMeRecord[] | null = null;
let liteCached: readonly ANiMeRecord[] | null = null;
let liteLoading: Promise<readonly ANiMeRecord[]> | null = null;

/** 单部番的完整记录 data/record/<id>.json, 详情页用. single 存盖过 overlay 的版本, 保证同一部番多次读取是同一个对象 */
const singleBase = new Map<number, ANiMeRecord>();
const single = new Map<number, ANiMeRecord>();
const singleLoading = new Map<number, Promise<ANiMeRecord | null>>();

function currentRecords (scope: AnimeRecordScope): readonly ANiMeRecord[] | null {
    if (cachedRecords) return cachedRecords;
    return scope === "lite" ? liteCached : null;
}

/** 重算所有已加载范围的 overlay (dev 编辑器保存后) */
function recomputeOverlay (): void {
    if (baseRecords) cachedRecords = Object.freeze(applyOverlay(baseRecords, liveOverlay));
    if (liteBase) liteCached = Object.freeze(applyOverlay(liteBase, liveOverlay));
    for (const [id, r] of singleBase) single.set(id, applyOverlayEntry(r, liveOverlay[String(id)]));
}

/**
 * 改一条 overlay 并立刻重算记录, 通知所有在用的页面. 只改内存; 落盘由调用方 (dev 编辑器) 先经写入通道完成.
 * entry 为 null 表示删除这部番的 overlay. 生产环境没有调用方.
 */
export function setOverlayEntry (id: number, entry: AnimeOverlayEntry | null): void {
    const next = { ...liveOverlay };
    if (entry) next[String(id)] = entry;
    else delete next[String(id)];
    liveOverlay = next;
    recomputeOverlay();
    notify();
}

export function getOverlayEntry (id: number): AnimeOverlayEntry | undefined {
    return liveOverlay[String(id)];
}
let loadingPromise: Promise<readonly ANiMeRecord[]> | null = null;

let actorMap: Map<number, Actor> | null = null;
let loadingActorPromise: Promise<Map<number, Actor>> | null = null;

function loadAnimeRecords (): Promise<readonly ANiMeRecord[]> {
    if (cachedRecords) {
        return Promise.resolve(cachedRecords);
    }

    if (loadingPromise) {
        return loadingPromise;
    }

    loadingPromise = (async () => {
        const text = await fetchAnimeDataText('/data/ANiMeRecord.json');
        const data = parse<ANiMeRecord[]>(text);

        baseRecords = Object.freeze(data);
        cachedRecords = Object.freeze(applyOverlay(baseRecords, liveOverlay));
        return cachedRecords;
    })();
    // 失败后清掉在途 promise, 让"重试"真的重新请求, 而不是拿到同一个 rejected promise
    loadingPromise.catch(() => { loadingPromise = null; });

    return loadingPromise;
}

function loadLiteRecords (): Promise<readonly ANiMeRecord[]> {
    if (cachedRecords) return Promise.resolve(cachedRecords);
    if (liteCached) return Promise.resolve(liteCached);
    if (liteLoading) return liteLoading;
    liteLoading = (async () => {
        let text: string;
        try {
            text = await fetchAnimeDataText('/data/ANiMeIndex.json');
        } catch {
            // 旧 tag 还没有索引文件: 退回全量
            return loadAnimeRecords();
        }
        liteBase = Object.freeze(parse<ANiMeRecord[]>(text));
        liteCached = Object.freeze(applyOverlay(liteBase, liveOverlay));
        return liteCached;
    })();
    liteLoading.catch(() => { liteLoading = null; });
    return liteLoading;
}

function findIn (records: readonly ANiMeRecord[], id: number): ANiMeRecord | null {
    return records.find((r) => r.anime_data.id === id) ?? null;
}

/** 已知结果: 记录对象, null = 库里确定没有, undefined = 还不知道 */
function knownSingle (id: number): ANiMeRecord | null | undefined {
    const hit = single.get(id);
    if (hit) return hit;
    if (cachedRecords) return findIn(cachedRecords, id);
    return undefined;
}

/**
 * 读一部番的完整记录. 先拉 record/<id>.json (十几 KB);
 * 拿不到时用轻量列表判断库里有没有这部: 没有就是 404, 不必下载全量; 有 (旧 tag 没有单部文件) 才退回全量.
 */
function loadSingleRecord (id: number): Promise<ANiMeRecord | null> {
    const known = knownSingle(id);
    if (known !== undefined) return Promise.resolve(known);
    const inflight = singleLoading.get(id);
    if (inflight) return inflight;
    const p = (async () => {
        try {
            const raw = Object.freeze(parse<ANiMeRecord>(await fetchAnimeDataText(`/data/record/${id}.json`)));
            singleBase.set(id, raw);
            single.set(id, applyOverlayEntry(raw, liveOverlay[String(id)]));
            return single.get(id)!;
        } catch {
            const lite = await loadLiteRecords();
            if (!findIn(lite, id)) return null;
            return findIn(await loadAnimeRecords(), id);
        }
    })();
    singleLoading.set(id, p);
    p.finally(() => { singleLoading.delete(id); }).catch(() => { /* 由调用方处理 */ });
    return p;
}

function loadActorMap (): Promise<Map<number, Actor>> {
    if (actorMap) {
        return Promise.resolve(actorMap);
    }
    
    if (loadingActorPromise) {
        return loadingActorPromise;
    }
    
    loadingActorPromise = (async () => {
        const text = await fetchAnimeDataText('/data/Actor.json');
        const data = parse<Actor[]>(text);
        
        const map = new Map<number, Actor>();
        data.forEach(actor => {
            map.set(actor.id, actor);
        });
        
        actorMap = map;
        return actorMap;
    })();
    loadingActorPromise.catch(() => { loadingActorPromise = null; });
    
    return loadingActorPromise;
}

/**
 * 人工标注 HXLoLi-ANiMe/data/labels/<id>.json: 唯一的 tag 来源 (hx-anime-tag-refactor-design.md 第 4 节).
 * 2026-11-04 起不再拉 ANiMeTags.json (原 loadSourceTags 已删除): 自动上墙的分配不再参与渲染.
 * 到了以后给所有在用的页面推一份新数组 (记录对象不变, 只为让依赖 records 的 useMemo 重算标签).
 * 拉不到时标注为空 (页面上不显示 tag), 状态记为 error, 专属页据此提示"标注没拉到"而不是"一部都没有".
 */
export type AnimeLabelsStatus = "loading" | "ready" | "error";
let labelsPromise: Promise<void> | null = null;
let labelsStatus: AnimeLabelsStatus = "loading";
const labelListeners = new Set<(s: AnimeLabelsStatus) => void>();

function loadLabels (): void {
    if (labelsPromise) return;
    labelsStatus = "loading";
    labelsPromise = fetchAnimeLabels().then(
        (map) => {
            setAnimeLabels(map);
            labelsStatus = "ready";
            labelListeners.forEach((fn) => fn(labelsStatus));
            // 记录对象不变, 只换数组引用, 让依赖 records 的 useMemo 重算标签
            if (cachedRecords) cachedRecords = Object.freeze([...cachedRecords]);
            if (liteCached) liteCached = Object.freeze([...liteCached]);
            notify();
        },
        () => {
            labelsStatus = "error";
            labelsPromise = null; // 下次挂载 / 重试时重新拉
            labelListeners.forEach((fn) => fn(labelsStatus));
        },
    );
}

/** 标注的加载状态 (susume 专属页 / 首页入口卡用来区分"还在拉"与"没有") */
export function useAnimeLabelsStatus (): AnimeLabelsStatus {
    const [s, setS] = useState<AnimeLabelsStatus>(labelsStatus);
    useEffect(() => {
        loadLabels();
        setS(labelsStatus);
        labelListeners.add(setS);
        return () => { labelListeners.delete(setS); };
    }, []);
    return s;
}

export type AnimeLoadStatus = "loading" | "ready" | "error";

export interface AnimeRecordsState {
    records: readonly ANiMeRecord[];
    status: AnimeLoadStatus;
    error: string | null;
    retry: () => void;
}

/**
 * 带加载状态的番剧记录. 页面据此区分"还在拉"(骨架)、"拉失败"(错误 + 重试) 与"拉到了但没有这部"(404),
 * 详情页冷启动不再在数据到达前闪 404. 已缓存时首帧即 ready.
 * scope = "lite" 读轻量列表 (没有角色和每集简介), 只有声优页和关系图这类要角色的页面用默认的 "full".
 * 筛选与加载态的决策见 src/utils/anime/filters.ts 顶部注释
 */
export function useAnimeRecordsState (baseUrl: string, scope: AnimeRecordScope = "full"): AnimeRecordsState {
    const [records, setRecords] = useState<readonly ANiMeRecord[]>(() => currentRecords(scope) ?? []);
    const [status, setStatus] = useState<AnimeLoadStatus>(() => (currentRecords(scope) ? "ready" : "loading"));
    const [error, setError] = useState<string | null>(null);
    const [attempt, setAttempt] = useState(0);

    useEffect(() => {
        let alive = true;
        loadLabels();
        if (!currentRecords(scope)) setStatus("loading");
        (scope === "lite" ? loadLiteRecords() : loadAnimeRecords()).then(
            (data) => { if (alive) { setRecords(data); setStatus("ready"); setError(null); } },
            (e: unknown) => { if (alive) { setStatus("error"); setError(e instanceof Error ? e.message : String(e)); } },
        );
        return () => { alive = false; };
    }, [baseUrl, attempt, scope]);

    // overlay 改动 (dev 编辑器保存) 或标注到达后拿新的记录
    useEffect(() => {
        const fn = () => { const next = currentRecords(scope); if (next) setRecords(next); };
        listeners.add(fn);
        return () => { listeners.delete(fn); };
    }, [scope]);

    const retry = useCallback(() => setAttempt((n) => n + 1), []);
    return { records, status, error, retry };
}

/** 只要记录数组的旧接口 (首页番剧块等在用); 加载中与失败时都是空数组 */
export function useAnimeRecords (baseUrl: string, scope: AnimeRecordScope = "full") {
    return useAnimeRecordsState(baseUrl, scope).records;
}

export type AnimeRecordStatus = "loading" | "ready" | "notfound" | "error";

export interface AnimeRecordState {
    record: ANiMeRecord | null;
    status: AnimeRecordStatus;
    error: string | null;
    retry: () => void;
}

/**
 * 详情页用: 只拉这一部的完整记录, 不再为一部番下载全部记录.
 * id 为 null (参数非法) 时直接 notfound.
 * .agents/notes/implemented/feature/2026-11-23-anime-detail-on-demand.md
 */
export function useAnimeRecord (id: number | null, baseUrl: string): AnimeRecordState {
    const initial = id == null ? null : knownSingle(id);
    const [record, setRecord] = useState<ANiMeRecord | null>(initial ?? null);
    const [status, setStatus] = useState<AnimeRecordStatus>(
        id == null ? "notfound" : initial === undefined ? "loading" : initial ? "ready" : "notfound",
    );
    const [error, setError] = useState<string | null>(null);
    const [attempt, setAttempt] = useState(0);
    const [, setTick] = useState(0);

    useEffect(() => {
        if (id == null) { setRecord(null); setStatus("notfound"); return; }
        let alive = true;
        loadLabels();
        const known = knownSingle(id);
        if (known === undefined) { setRecord(null); setStatus("loading"); }
        loadSingleRecord(id).then(
            (r) => { if (alive) { setRecord(r); setStatus(r ? "ready" : "notfound"); setError(null); } },
            (e: unknown) => { if (alive) { setStatus("error"); setError(e instanceof Error ? e.message : String(e)); } },
        );
        return () => { alive = false; };
    }, [id, baseUrl, attempt]);

    useEffect(() => {
        if (id == null) return;
        // 标注到达时记录对象不变, 用计数强制重渲染, 让标签跟上
        const fn = () => { const next = knownSingle(id); if (next) setRecord(next); setTick((n) => n + 1); };
        listeners.add(fn);
        return () => { listeners.delete(fn); };
    }, [id]);

    const retry = useCallback(() => setAttempt((n) => n + 1), []);
    return { record, status, error, retry };
}

export function useActorMap(baseUrl: string) {
    const [data, setData] = useState<Map<number, Actor>>(actorMap ?? new Map());
    
    useEffect(() => {
        loadActorMap().then(setData, () => { /* 声优表拉不到只影响角色卡上的声优名, 不阻塞页面 */ });
    }, [baseUrl]);
    
    return data;
}

export function useAnimeRecordMap(baseUrl: string) {
    const records = useAnimeRecords(baseUrl);
    const [data, setData] = useState<Map<number, ANiMeRecord>>(new Map());

    useEffect(() => {
        const map = new Map<number, ANiMeRecord>();
        records.forEach(record => {
            map.set(record.anime_data.id, record);
        });
        setData(map);
    }, [records]);

    return data;
}
