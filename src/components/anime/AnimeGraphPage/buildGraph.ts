import type { Actor, ANiMeRecord, Character } from "@site/src/utils/anime/types";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";

// 关系图的纯数据构建 (无 React, 无 DOM)

/** relations 里视为 "同一系列" 的关系类型 (bangumi 关系文本) */
export const SERIES_RELATIONS: ReadonlySet<string> = new Set([
    "前传",
    "续集",
    "番外篇",
    "总集篇",
    "主线故事",
]);

/** 名字前缀兜底时要求的最短名字长度, 太短 (如 "Fate") 会把整个品牌连成一团 */
const FALLBACK_MIN_NAME_LENGTH = 4;

export interface SeriesLink {
    source: number;
    target: number;
    /** relations: 来自 anime_data.relations; name-fallback: 名字前缀兜底 */
    via: "relations" | "name-fallback";
    /** 来自 relations 时的原始关系文本 (前传/续集...) */
    relation?: string;
}

export interface SeriesIndex {
    links: SeriesLink[];
    /** anime id -> 同系列直接相邻的 anime id */
    adjacency: Map<number, Set<number>>;
}

function pairKey (a: number, b: number): string {
    return a < b ? `${a}-${b}` : `${b}-${a}`;
}

function isNamePrefix (a: string | undefined, b: string | undefined): boolean {
    const p = (a || "").trim();
    const q = (b || "").trim();
    if (p.length < FALLBACK_MIN_NAME_LENGTH || q.length < FALLBACK_MIN_NAME_LENGTH) return false;
    return q.startsWith(p) || p.startsWith(q);
}

/**
 * 续作 / 同系列边.
 *
 * 1. 主来源: anime_data.relations 中关系类型属于 SERIES_RELATIONS, 且对方 id 也在库里. 双向去重.
 * 2. 兜底: relations 的系列链在库内断开时 (典型: 中间一季没看, 第一季与第三季之间没有直接关系),
 *    按首播日期排序, 对 "一方的原名或中文名整体是另一方的前缀 (两边都至少 4 个字符)" 的两部番补一条边,
 *    且只在两者此前不属于同一系列连通分量时才补 (并查集), 所以兜底不会把一个系列连成完全图,
 *    也不会把只共享 "Fate/" 这种品牌前缀的作品误连.
 */
export function buildSeriesIndex (records: readonly ANiMeRecord[]): SeriesIndex {
    const ids = new Set<number>(records.map(r => r.anime_data.id));
    const parent = new Map<number, number>();
    ids.forEach(id => parent.set(id, id));
    const find = (x: number): number => {
        let root = x;
        while (parent.get(root) !== root) root = parent.get(root)!;
        let cur = x;
        while (parent.get(cur) !== root) {
            const next = parent.get(cur)!;
            parent.set(cur, root);
            cur = next;
        }
        return root;
    };
    const union = (a: number, b: number) => parent.set(find(a), find(b));

    const links: SeriesLink[] = [];
    const seen = new Set<string>();
    const adjacency = new Map<number, Set<number>>();
    const addLink = (link: SeriesLink) => {
        const key = pairKey(link.source, link.target);
        if (seen.has(key)) return;
        seen.add(key);
        links.push(link);
        union(link.source, link.target);
        if (!adjacency.has(link.source)) adjacency.set(link.source, new Set());
        if (!adjacency.has(link.target)) adjacency.set(link.target, new Set());
        adjacency.get(link.source)!.add(link.target);
        adjacency.get(link.target)!.add(link.source);
    };

    records.forEach(r => {
        const a = r.anime_data;
        (a.relations || []).forEach(rel => {
            if (rel.id === a.id || !ids.has(rel.id) || !SERIES_RELATIONS.has(rel.relation)) return;
            addLink({ source: a.id, target: rel.id, via: "relations", relation: rel.relation });
        });
    });

    const byDate = [...records].sort((x, y) =>
        (x.anime_data.date || "").localeCompare(y.anime_data.date || ""));
    for (let i = 0; i < byDate.length; i++) {
        const a = byDate[i].anime_data;
        for (let j = i + 1; j < byDate.length; j++) {
            const b = byDate[j].anime_data;
            if (find(a.id) === find(b.id)) continue;
            if (isNamePrefix(a.name, b.name) || isNamePrefix(a.name_cn, b.name_cn)) {
                addLink({ source: a.id, target: b.id, via: "name-fallback" });
            }
        }
    }

    return { links, adjacency };
}

/** 从一部番出发沿系列边 BFS, 返回整个系列的 anime id (含自身) */
export function collectSeries (startId: number, adjacency: Map<number, Set<number>>): Set<number> {
    const visited = new Set<number>([startId]);
    const queue = [startId];
    while (queue.length > 0) {
        const cur = queue.shift()!;
        adjacency.get(cur)?.forEach(next => {
            if (visited.has(next)) return;
            visited.add(next);
            queue.push(next);
        });
    }
    return visited;
}

// ================= 两层视图的图数据 =================

export type GraphNodeType = "Anime" | "Character" | "Actor";
export type GraphLinkType = "Include" | "VoicedBy" | "Sequel" | "Related" | "SharedCV" | "Appears";

/** 画布节点的纯数据部分 (力导向坐标由图库在运行时挂上) */
export interface GraphNodeData {
    id: string;
    type: GraphNodeType;
    name: string;
    name_cn?: string;
    image_url?: string;
    val: number;
    rawCharacter?: Character;
    rawAnimeId?: number;
    rawActorId?: number;
    /** 聚焦层里 "声优的其它作品": 淡色占位, 双击切换为新的中心 */
    ghost?: boolean;
}

export interface GraphLinkData {
    source: string;
    target: string;
    type: GraphLinkType;
    label?: string;
    /** 共享声优边: 共享的声优人数 */
    weight?: number;
}

export interface GraphDataPlain {
    nodes: GraphNodeData[];
    links: GraphLinkData[];
}

export const animeNodeId = (id: number) => `anime_${id}`;
const charNodeId = (id: number) => `char_${id}`;
const actorNodeId = (id: number) => `actor_${id}`;

function animeNode (anime: ANiMeRecord["anime_data"], val: number, ghost = false): GraphNodeData {
    return {
        id: animeNodeId(anime.id),
        type: "Anime",
        name: anime.name,
        name_cn: anime.name_cn,
        image_url: toAnimeCdnUrl(`/data/anime/${anime.id}.jpg`),
        val,
        rawAnimeId: anime.id,
        ghost,
    };
}

function visibleActorIds (anime: ANiMeRecord["anime_data"], hiddenRelations: readonly string[]): Set<number> {
    const set = new Set<number>();
    anime.characters.forEach(ch => {
        if (hiddenRelations.includes(ch.relation)) return;
        ch.actor_ids.forEach(a => set.add(a));
    });
    return set;
}

/**
 * 概览层: 只有番剧节点. 边 = 续作 / 同系列 + 共享声优 (按共享人数加权, 只统计未被角色定位过滤掉的角色,
 * 共享人数 >= minShared 才连). 160 部、默认只看主角、阈值 2 时约 150 条共享边.
 */
export function buildOverviewGraph (
    records: readonly ANiMeRecord[],
    series: SeriesIndex,
    hiddenRelations: readonly string[],
    minShared: number,
): GraphDataPlain {
    const nodes = records.map(r => animeNode(r.anime_data, 60));
    const links: GraphLinkData[] = series.links.map(l => ({
        source: animeNodeId(l.source),
        target: animeNodeId(l.target),
        type: "Sequel" as const,
        label: l.relation,
    }));

    // 声优 -> 出演的番剧 (倒排), 再两两累计共享人数
    const actorToAnime = new Map<number, number[]>();
    records.forEach(r => {
        visibleActorIds(r.anime_data, hiddenRelations).forEach(actorId => {
            const list = actorToAnime.get(actorId);
            if (list) list.push(r.anime_data.id);
            else actorToAnime.set(actorId, [r.anime_data.id]);
        });
    });
    const shared = new Map<string, { a: number; b: number; count: number }>();
    actorToAnime.forEach(list => {
        for (let i = 0; i < list.length; i++) {
            for (let j = i + 1; j < list.length; j++) {
                const a = Math.min(list[i], list[j]);
                const b = Math.max(list[i], list[j]);
                const key = `${a}-${b}`;
                const cur = shared.get(key);
                if (cur) cur.count++;
                else shared.set(key, { a, b, count: 1 });
            }
        }
    });
    const threshold = Math.max(1, minShared);
    shared.forEach(({ a, b, count }) => {
        if (count < threshold) return;
        links.push({
            source: animeNodeId(a),
            target: animeNodeId(b),
            type: "SharedCV",
            weight: count,
            label: `共享 ${count} 位声优`,
        });
    });
    return { nodes, links };
}

/**
 * 聚焦层: 以一部番所在的系列为中心, 展开角色与声优, 再挂上这些声优在库里的其它作品 (ghost 节点).
 */
export function buildFocusGraph (
    records: readonly ANiMeRecord[],
    actorMap: ReadonlyMap<number, Actor>,
    series: SeriesIndex,
    focusAnimeId: number,
    hiddenRelations: readonly string[],
): GraphDataPlain {
    const nodes = new Map<string, GraphNodeData>();
    const links: GraphLinkData[] = [];
    const linkKeys = new Set<string>();
    const addNode = (node: GraphNodeData) => { if (!nodes.has(node.id)) nodes.set(node.id, node); };
    const addLink = (link: GraphLinkData) => {
        const key = `${link.source}|${link.target}|${link.type}`;
        if (linkKeys.has(key)) return;
        linkKeys.add(key);
        links.push(link);
    };

    const seriesIds = collectSeries(focusAnimeId, series.adjacency);
    const seriesRecords = records.filter(r => seriesIds.has(r.anime_data.id));
    const shownActors = new Set<number>();

    seriesRecords.forEach(record => {
        const anime = record.anime_data;
        addNode(animeNode(anime, 80));
        anime.characters.forEach(char => {
            if (hiddenRelations.includes(char.relation)) return;
            addNode({
                id: charNodeId(char.id),
                type: "Character",
                name: char.name,
                name_cn: char.name_cn,
                image_url: toAnimeCdnUrl(`/data/kyara/${char.id}.jpg`),
                val: 20,
                rawCharacter: char,
                rawAnimeId: anime.id,
            });
            addLink({ source: animeNodeId(anime.id), target: charNodeId(char.id), type: "Include" });
            char.actor_ids.forEach(actorId => {
                const actor = actorMap.get(actorId);
                if (!actor) return;
                shownActors.add(actorId);
                addNode({
                    id: actorNodeId(actorId),
                    type: "Actor",
                    name: actor.name,
                    image_url: toAnimeCdnUrl(`/data/cv/${actorId}.jpg`),
                    val: 12,
                    rawActorId: actorId,
                });
                addLink({ source: charNodeId(char.id), target: actorNodeId(actorId), type: "VoicedBy" });
            });
        });
    });

    series.links.forEach(l => {
        if (!seriesIds.has(l.source) || !seriesIds.has(l.target)) return;
        addLink({ source: animeNodeId(l.source), target: animeNodeId(l.target), type: "Sequel", label: l.relation });
    });

    // 声优的其它在库作品 (同样只算未被过滤的角色定位)
    records.forEach(record => {
        const anime = record.anime_data;
        if (seriesIds.has(anime.id)) return;
        const actors = visibleActorIds(anime, hiddenRelations);
        actors.forEach(actorId => {
            if (!shownActors.has(actorId)) return;
            addNode(animeNode(anime, 36, true));
            addLink({ source: actorNodeId(actorId), target: animeNodeId(anime.id), type: "Appears" });
        });
    });

    return { nodes: Array.from(nodes.values()), links };
}

export interface SearchHit {
    key: string;
    animeId: number;
    title: string;
    subtitle: string;
}

/** 搜索番剧 (原名 / 中文名) 与角色 (命中角色时定位到其所在番剧), 大小写不敏感 */
export function searchGraph (records: readonly ANiMeRecord[], query: string, preferCn: boolean, limit = 10): SearchHit[] {
    const q = query.trim().toLowerCase();
    if (!q) return [];
    const hits: SearchHit[] = [];
    const match = (s: string | undefined) => !!s && s.toLowerCase().includes(q);
    const title = (name: string, nameCn?: string) => (preferCn ? (nameCn || name) : name);

    for (const r of records) {
        const a = r.anime_data;
        if (match(a.name) || match(a.name_cn)) {
            hits.push({ key: `a${a.id}`, animeId: a.id, title: title(a.name, a.name_cn), subtitle: "番剧" });
            if (hits.length >= limit) return hits;
        }
    }
    for (const r of records) {
        const a = r.anime_data;
        for (const ch of a.characters) {
            if (match(ch.name) || match(ch.name_cn)) {
                hits.push({
                    key: `c${a.id}-${ch.id}`,
                    animeId: a.id,
                    title: title(ch.name, ch.name_cn),
                    subtitle: `角色 · ${title(a.name, a.name_cn)}`,
                });
                if (hits.length >= limit) return hits;
            }
        }
    }
    return hits;
}
