import { useMemo } from "react";
import { usePluginData } from "@docusaurus/useGlobalData";
import type { KyaraGlobalData, KyaraIndexEntry } from "./types";
import { useKyaraLocalCharacters, useKyaraLocalMode } from "./localStore";

// 番剧 ↔ kyara 的关联: 仓库角色来自插件 setGlobalData (SSR 可用), 纯本地角色在本地模式开启时由客户端并入.
// 匹配键是 Bangumi 角色 id (跨季共用); 番剧条目 id 只用来分组和回链.

const PLUGIN = "kyara-gallery";

/** 仓库里的全部角色 (插件注入, 无网络请求) */
export function useKyaraStatic(): KyaraIndexEntry[] {
    const data = usePluginData(PLUGIN) as KyaraGlobalData | undefined;
    return data?.characters ?? [];
}

/** 合并仓库角色与本地角色; 同名时以仓库为准 (该角色的本地图在它自己的页面里照样显示) */
export function mergeKyaraEntries(statics: KyaraIndexEntry[], locals: KyaraIndexEntry[]): KyaraIndexEntry[] {
    const taken = new Set(statics.map((c) => c.meta.name));
    return [...statics, ...locals.filter((c) => !taken.has(c.meta.name))];
}

export interface KyaraCatalog {
    entries: KyaraIndexEntry[];
    localOn: boolean;
    setLocalOn: (v: boolean) => void;
}

/** 当前可见的全部画集: 仓库 + (本地模式开启时) 本地 */
export function useKyaraCatalog(): KyaraCatalog {
    const statics = useKyaraStatic();
    const [localOn, setLocalOn] = useKyaraLocalMode();
    const { characters: locals } = useKyaraLocalCharacters(localOn);
    const entries = useMemo(() => mergeKyaraEntries(statics, locals), [statics, locals]);
    return { entries, localOn, setLocalOn };
}

export function findKyaraForCharacter(entries: KyaraIndexEntry[], bangumiCharacterId: number): KyaraIndexEntry | null {
    return entries.find((c) => c.meta.bangumiCharacterId === bangumiCharacterId) ?? null;
}

export function kyaraForSubject(entries: KyaraIndexEntry[], subjectId: number): KyaraIndexEntry[] {
    return entries.filter((c) => (c.meta.bangumiSubjectIds ?? []).includes(subjectId));
}

/** 画集页地址 (不含 baseUrl, 交给 Link / history) */
export function kyaraHref(entry: { meta: { name: string }; local?: boolean }): string {
    const n = encodeURIComponent(entry.meta.name);
    return entry.local ? `/anime/kyara/local?name=${n}` : `/anime/kyara/${n}`;
}

/** 作品分组键: 有关联条目用第一个条目 id, 否则用作品名, 都没有归入 "其他" */
export function kyaraWorkKey(entry: KyaraIndexEntry): string {
    const sid = entry.meta.bangumiSubjectIds?.[0];
    if (sid) return `s${sid}`;
    return entry.meta.work ? `w${entry.meta.work}` : "other";
}
