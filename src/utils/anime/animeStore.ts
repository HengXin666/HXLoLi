import { parse } from "@site/src/utils/anime/parser";
import { Actor, ANiMeRecord } from "@site/src/utils/anime/types";
import { useState, useEffect, useCallback } from "react";
import { reqAnimeByCDN } from '@site/src/utils/cdn/linkJsDelivr'
import { ANIME_OVERLAY, AnimeOverlay, AnimeOverlayEntry, applyOverlay } from "@site/src/utils/anime/overlay";

/** CDN 原始记录 (未盖 overlay) */
let baseRecords: readonly ANiMeRecord[] | null = null;
/** 盖过本地 overlay 的记录, 页面读的是这份. 本地标注模式的决策见 src/utils/anime/overlay.ts 顶部注释 */
let cachedRecords: readonly ANiMeRecord[] | null = null;
/** 当前生效的 overlay. 初值是打进包里的 data/animeOverlay.json; dev 编辑器保存后经 setOverlayEntry 就地更新 */
let liveOverlay: AnimeOverlay = { ...ANIME_OVERLAY };
const listeners = new Set<(records: readonly ANiMeRecord[]) => void>();

/**
 * 改一条 overlay 并立刻重算记录, 通知所有在用的页面. 只改内存; 落盘由调用方 (dev 编辑器) 先经写入通道完成.
 * entry 为 null 表示删除这部番的 overlay. 生产环境没有调用方.
 */
export function setOverlayEntry (id: number, entry: AnimeOverlayEntry | null): void {
    const next = { ...liveOverlay };
    if (entry) next[String(id)] = entry;
    else delete next[String(id)];
    liveOverlay = next;
    if (!baseRecords) return;
    cachedRecords = Object.freeze(applyOverlay(baseRecords, liveOverlay));
    listeners.forEach((fn) => fn(cachedRecords!));
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
        const result = await reqAnimeByCDN('/data/ANiMeRecord.json');
        const text = await result.blob.text();
        const data = parse<ANiMeRecord[]>(text);

        baseRecords = Object.freeze(data);
        cachedRecords = Object.freeze(applyOverlay(baseRecords, liveOverlay));
        return cachedRecords;
    })();
    // 失败后清掉在途 promise, 让"重试"真的重新请求, 而不是拿到同一个 rejected promise
    loadingPromise.catch(() => { loadingPromise = null; });

    return loadingPromise;
}

function loadActorMap (): Promise<Map<number, Actor>> {
    if (actorMap) {
        return Promise.resolve(actorMap);
    }
    
    if (loadingActorPromise) {
        return loadingActorPromise;
    }
    
    loadingActorPromise = (async () => {
        const result = await reqAnimeByCDN('/data/Actor.json');
        const text = await result.blob.text();
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

export type AnimeLoadStatus = "loading" | "ready" | "error";

export interface AnimeRecordsState {
    records: readonly ANiMeRecord[];
    status: AnimeLoadStatus;
    error: string | null;
    retry: () => void;
}

/**
 * 带加载状态的番剧记录. 页面据此区分"还在拉 12 MB 的 JSON"(骨架)、"拉失败"(错误 + 重试) 与"拉到了但没有这部"(404),
 * 详情页冷启动不再在数据到达前闪 404. 已缓存时首帧即 ready.
 * 筛选与加载态的决策见 src/utils/anime/filters.ts 顶部注释
 */
export function useAnimeRecordsState (baseUrl: string): AnimeRecordsState {
    const [records, setRecords] = useState<readonly ANiMeRecord[]>(cachedRecords ?? []);
    const [status, setStatus] = useState<AnimeLoadStatus>(cachedRecords ? "ready" : "loading");
    const [error, setError] = useState<string | null>(null);
    const [attempt, setAttempt] = useState(0);

    useEffect(() => {
        let alive = true;
        if (!cachedRecords) setStatus("loading");
        loadAnimeRecords().then(
            (data) => { if (alive) { setRecords(data); setStatus("ready"); setError(null); } },
            (e: unknown) => { if (alive) { setStatus("error"); setError(e instanceof Error ? e.message : String(e)); } },
        );
        return () => { alive = false; };
    }, [baseUrl, attempt]);

    // overlay 改动 (dev 编辑器保存) 后拿新的记录
    useEffect(() => {
        const fn = (next: readonly ANiMeRecord[]) => setRecords(next);
        listeners.add(fn);
        return () => { listeners.delete(fn); };
    }, []);

    const retry = useCallback(() => setAttempt((n) => n + 1), []);
    return { records, status, error, retry };
}

/** 只要记录数组的旧接口 (首页番剧块等在用); 加载中与失败时都是空数组 */
export function useAnimeRecords (baseUrl: string) {
    return useAnimeRecordsState(baseUrl).records;
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
