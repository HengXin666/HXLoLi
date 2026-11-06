import { useMemo } from "react";
import { buildSeriesIndex, collectSeries } from "@site/src/components/anime/AnimeGraphPage/buildGraph";
import { normalEpisodeCount, animeSeasonOf, SortKey } from "@site/src/utils/anime/filters";
import { getRecordTags } from "@site/src/utils/anime/tags";
import { ANiMeRecord, Relation, SubjectType, WatchStatus } from "@site/src/utils/anime/types";

/**
 * 番剧系列 (「系列」视图): 把 bgm 上用 前传 / 续集 (以及 番外篇 / 总集篇 / 主线故事) 链起来的条目聚成一个系列,
 * 番剧库 `?fold=series` 一个系列一张卡, `/anime/series?id=` 过渡页按观看顺序列出主线 (含链上的剧场版) 与特别篇.
 * 衍生作品 (某科学的超电磁炮 主线故事 -> 魔法禁书目录, 且自己是 >= 6 话的长篇) 自成系列, 经 related 互相链出.
 * 系列边直接复用关系图的 buildSeriesIndex (relations 主来源 + 名字前缀兜底, 并查集防误连), 不另写一套判定.
 * 设计: HXLoLis/hx-anime-series-fold-design.md
 * .agents/notes/implemented/feature/2026-11-05-anime-series-fold.md
 */
export interface AnimeSeason {
    record: ANiMeRecord;
    /**
     * main: 主线 (经 前传 / 续集 串起来的正片剧情, 含链上的剧场版 / OVA / 完结篇);
     * special: 不在主线上的番外篇 / 总集篇 / 剧场版 / OVA
     */
    kind: "main" | "special";
    /**
     * 正篇第几季 (从 1 起); 特别篇与主线上的短篇 (剧场版 / OVA / 完结篇) 为 null. 不是条目序号: 优先取标题里写明的季号 (第二季 / Season 3 / 2nd season / Ⅱ),
     * 分割放送的上下半 (Part.2 / 第2クール / 后半 / 同季号的 襲擊編·反擊編) 共用一个 no.
     */
    no: number | null;
    /** 同一季的第几部分 (分割放送), 该季只有一部分时为 null */
    part: number | null;
    /** 最终季 (The Final Season / 最终季), 显示为 "最终季" */
    final: boolean;
    /** 不编季号的作品种类: 剧场版 / 完结篇 前篇 / OVA / 总集篇 / 番外篇 / 特别篇 / 待播; 编了季号的 TV 季为 null */
    special: string | null;
}

export interface AnimeSeries {
    /** 系列 id = 首播最早的正篇条目的 bgm id; 也是过渡页 URL 的 id */
    key: number;
    /** 系列名 (第 1 季中文名去掉季度后缀) */
    title: string;
    /** 系列原名 (第 1 季原名去掉季度后缀) */
    originalTitle: string;
    /** 主线在前 (按 前传 / 续集 的观看顺序), 特别篇在后 (按首播) */
    seasons: AnimeSeason[];
    /** relations 里指向的 前传 / 续集, 但不在库里的动画条目 (过渡页灰显) */
    missing: Relation[];
    /** 同上, 带插入位置: 挂在 anchor 那一部的前 (前传) / 后 (续集) */
    missingAt: MissingWork[];
    /** 库内同一世界观的其它系列 (魔法禁书目录 <-> 某科学的超电磁炮 / 某科学的一方通行): 衍生作品自成系列, 不再并进来 */
    related: RelatedSeries[];
}

export interface MissingWork {
    rel: Relation;
    /** 引用它的库内条目 id */
    anchor: number;
    /** true: 前传, 排在 anchor 前; false: 续集, 排在 anchor 后 */
    before: boolean;
}

export interface RelatedSeries {
    /** 对方系列 key */
    key: number;
    /** 主线 / 衍生 / 外传 / 同一世界观 (从本系列看对方) */
    relation: string;
}

// ---------------------------------------------------------------- 季号解析

/** 标题里写明的季度标记 (只读标题, 不看条目顺序) */
export interface SeasonMarker {
    /** 写明的季号 */
    no: number | null;
    /** 写明的分割部分 (Part.2 / 第2クール / 后半 -> 2, 前半 -> 1) */
    part: number | null;
    /** 最终季 */
    final: boolean;
    /** 剧场版 / 映画 */
    movie: boolean;
}

const CN_DIGIT: Record<string, number> = { 一: 1, 二: 2, 三: 3, 四: 4, 五: 5, 六: 6, 七: 7, 八: 8, 九: 9, 十: 10 };
const ROMAN: Record<string, number> = { II: 2, III: 3, IV: 4, VI: 6, VII: 7, VIII: 8 };

function cnToInt (x: string): number | null {
    if (/^\d+$/.test(x)) return Number(x);
    if (x.length === 1) return CN_DIGIT[x] ?? null;
    if (x.length === 2 && x[0] === "十") return 10 + (CN_DIGIT[x[1]] ?? 0);
    if (x.length === 2 && x[1] === "十") return (CN_DIGIT[x[0]] ?? 0) * 10 || null;
    return null;
}

/** NFKC: 全角数字 -> 半角, Ⅱ -> II, ～ 等统一 */
const norm = (t: string): string => (t || "").normalize("NFKC");

function seasonNoOf (t: string): number | null {
    let m = t.match(/第\s*([一二三四五六七八九十\d]+)\s*[季期]/);
    if (m) return cnToInt(m[1]);
    m = t.match(/(\d+)\s*(?:st|nd|rd|th)\s*(?:season|シーズン)/i);
    if (m) return Number(m[1]);
    m = t.match(/(?:season|シーズン)\s*(\d+)/i);
    if (m) return Number(m[1]);
    // 独立的罗马数字 (オーバーロードⅡ / 無職転生Ⅱ ～…～); 单个 V / I 太容易误伤, 不认
    m = t.match(/(?:^|[^A-Za-z])(VIII|VII|VI|IV|III|II)(?![A-Za-z])/);
    if (m) return ROMAN[m[1]];
    // 紧跟在非拉丁字符后的结尾数字 (異世界かるてっと2); "86" 这类整串数字不算
    m = t.match(/[^\x00-\x7F]\s?([2-9])\s*$/);
    if (m) return Number(m[1]);
    return null;
}

function partOf (t: string): number | null {
    let m = t.match(/part\s*\.?\s*(\d+)/i);
    if (m) return Number(m[1]);
    m = t.match(/第\s*([一二三四五六七八九十\d]+)\s*(?:部分|クール|cour)/i);
    if (m) return cnToInt(m[1]);
    m = t.match(/(\d+)\s*(?:st|nd|rd|th)?\s*cour/i);
    if (m) return Number(m[1]);
    if (/後半|后半|下半/.test(t)) return 2;
    if (/前半|上半/.test(t)) return 1;
    return null;
}

/** 解析标题里的季度标记; 中文名与原名都看, 先命中的为准 */
export function parseSeasonMarker (...titles: string[]): SeasonMarker {
    const ts = titles.filter(Boolean).map(norm);
    const pick = <T,>(f: (t: string) => T | null): T | null => {
        for (const t of ts) { const v = f(t); if (v !== null) return v; }
        return null;
    };
    return {
        no: pick(seasonNoOf),
        part: pick(partOf),
        final: ts.some((t) => /final\s*season|the\s*last\s*season|最终季|最終季|ファイナルシーズン/i.test(t)),
        movie: ts.some((t) => /剧场版|劇場版|映画|电影|the\s*movie/i.test(t)),
    };
}

const markerOf = (r: ANiMeRecord): SeasonMarker => parseSeasonMarker(r.anime_data.name_cn, r.anime_data.name);

/** 剧情主线上的关系: 只有 前传 / 续集 决定观看顺序 */
const MAIN_RELATIONS: ReadonlySet<string> = new Set(["前传", "续集"]);
/** 只靠名字兜底挂进系列时, 正片集数达到这个数才算主线 (剧场版 / OVA 多为 1-3 集) */
const MAIN_MIN_EPS = 6;
/** 正片集数在 (0, SHORT_MAX_EPS] 的条目不编季号 (剧场版 / OVA / 完结篇 前后篇), 但在 前传 / 续集 链上时仍属主线 */
const SHORT_MAX_EPS = 3;
/** 特别篇种类优先取 bgm 关系名 */
const SPECIAL_RELATIONS: ReadonlySet<string> = new Set(["番外篇", "总集篇", "衍生", "外传", "不同演绎", "剧场版", "主线故事"]);
/** 库内两个不同系列之间, 在过渡页「同一世界观」里互相链出的关系 */
const WORLD_RELATIONS: Readonly<Record<string, string>> = { 主线故事: "主线", 衍生: "衍生", 外传: "外传", 相同世界观: "同一世界观" };

const byAirDate = (a: ANiMeRecord, b: ANiMeRecord): number =>
    (a.anime_data.date || "9999").localeCompare(b.anime_data.date || "9999") || a.anime_data.id - b.anime_data.id;

/** 去掉标题末尾的季度后缀: 第二季 / 第3期 / Season 2 / 2nd Season / Ⅱ / 续篇 ... */
const SEASON_SUFFIX = new RegExp(
    "(?:" + [
        "第[一二三四五六七八九十百\\d]+[季期部章]",
        "(?:the\\s+)?\\d+(?:st|nd|rd|th)\\s+season",
        "season\\s*\\d+",
        "\\bs\\d+",
        "part\\s*\\d+",
        "[ⅡⅢⅣⅤⅥⅦⅧⅨⅩ]",
        "(?<=[^A-Za-z])(?:II|III|IV|V|VI)",
        "(?<=\\D)\\s+\\d{1,2}",
        "续篇|續篇|续集|新章",
    ].map((p) => `(?:${p})`).join("|") + ")\\s*$",
    "i",
);

export function stripSeasonSuffix (title: string): string {
    let t = (title || "").trim();
    for (let i = 0; i < 3; i++) {
        const cut = t.replace(SEASON_SUFFIX, "");
        if (cut === t) break;
        const next = cut.replace(/[\s:：\-—~～]+$/, "").trim();
        if (next === t) break;
        t = next;
    }
    return t || (title || "").trim();
}

/** 剧场版 / OVA / 完结篇 这类不编季号的短篇 (集数 1-3, 或标题写明剧场版, 或未放送且标题没写季号) */
function isShortWork (r: ANiMeRecord): boolean {
    const eps = normalEpisodeCount(r);
    const mk = markerOf(r);
    if (eps > 0 && eps <= SHORT_MAX_EPS) return true;
    if (mk.movie && mk.no === null) return true;
    return eps === 0 && mk.no === null && !mk.final;
}

/** 自成一部的长篇 (>= 6 话的 TV / WEB, 不是剧场版): 它的 主线故事 指向的是"另一部番", 不是自己的正片 */
const isFeatureLength = (r: ANiMeRecord): boolean => normalEpisodeCount(r) >= MAIN_MIN_EPS && !markerOf(r).movie;

const pairKey = (a: number, b: number): string => (a < b ? `${a}-${b}` : `${b}-${a}`);

/**
 * 衍生作品的边: 库内 X 写 主线故事 -> Y (或 Y 写 番外篇 / 总集篇 -> X), 且 X 本身是 >= 6 话的长篇.
 * 例: 某科学的超电磁炮 (24 话) 主线故事 -> 魔法禁书目录, 某科学的一方通行 / 某暗部的少女共栖 / 转生史莱姆日记 同理.
 * 这些是同一世界观下的另一部番, 不并进同一个系列; 剧场版 / OVA 的 主线故事 (恩底弥翁的奇迹 -> 禁书目录 第二季) 照常并入.
 */
function spinoffPairs (records: readonly ANiMeRecord[]): Set<string> {
    const byId = new Map(records.map((r) => [r.anime_data.id, r]));
    const out = new Set<string>();
    for (const r of records) {
        const a = r.anime_data;
        for (const rel of a.relations || []) {
            const other = byId.get(rel.id);
            if (!other || rel.id === a.id) continue;
            // X 主线故事 -> Y: X 是衍生
            if (rel.relation === "主线故事" && isFeatureLength(r)) out.add(pairKey(a.id, rel.id));
            // Y 番外篇 / 总集篇 -> X: X 是衍生 (长篇的 "番外" 也是另一部番)
            if ((rel.relation === "番外篇" || rel.relation === "总集篇") && isFeatureLength(other)) out.add(pairKey(a.id, rel.id));
        }
    }
    // 两部之间同时有 前传 / 续集 时以 前传 / 续集 为准 (bgm 偶尔两种都写)
    for (const r of records) {
        for (const rel of r.anime_data.relations || []) {
            if (MAIN_RELATIONS.has(rel.relation)) out.delete(pairKey(r.anime_data.id, rel.id));
        }
    }
    return out;
}

/** 剥掉衍生边后的记录视图 (浅拷贝, 只给 buildSeriesIndex 判系列连通用; 系列成员仍是原记录) */
function withoutSpinoffEdges (records: readonly ANiMeRecord[], drop: ReadonlySet<string>): ANiMeRecord[] {
    if (!drop.size) return [...records];
    return records.map((r) => {
        const rels = r.anime_data.relations || [];
        const kept = rels.filter((rel) => !drop.has(pairKey(r.anime_data.id, rel.id)));
        return kept.length === rels.length ? r : { ...r, anime_data: { ...r.anime_data, relations: kept } };
    });
}

/** 两部之间 bgm 明写了 "是另一部番" 的关系 (相同世界观 / 衍生 / 主线故事 / 外传): 名字前缀兜底不能把它们连起来 (工作细胞 / 工作细胞BLACK) */
function worldPairs (records: readonly ANiMeRecord[]): Set<string> {
    const ids = new Set(records.map((r) => r.anime_data.id));
    const out = new Set<string>();
    for (const r of records) {
        for (const rel of r.anime_data.relations || []) {
            if (ids.has(rel.id) && rel.relation in WORLD_RELATIONS) out.add(pairKey(r.anime_data.id, rel.id));
        }
    }
    return out;
}

/**
 * 系列连通边: 复用关系图的 buildSeriesIndex (relations 主来源 + 名字前缀兜底), 但
 * 1. 先剥掉衍生作品的 主线故事 / 番外篇 边 (spinoffPairs);
 * 2. 再丢掉连接 "bgm 明写是另一部番" 两端的名字兜底边 (worldPairs).
 */
function seriesAdjacency (records: readonly ANiMeRecord[]): Map<number, Set<number>> {
    const { links } = buildSeriesIndex(withoutSpinoffEdges(records, spinoffPairs(records)));
    const world = worldPairs(records);
    const adjacency = new Map<number, Set<number>>();
    for (const l of links) {
        if (l.via === "name-fallback" && world.has(pairKey(l.source, l.target))) continue;
        if (!adjacency.has(l.source)) adjacency.set(l.source, new Set());
        if (!adjacency.has(l.target)) adjacency.set(l.target, new Set());
        adjacency.get(l.source)!.add(l.target);
        adjacency.get(l.target)!.add(l.source);
    }
    return adjacency;
}

/**
 * 主线排序: 按 前传 / 续集 的方向做拓扑排序, 同时可排的按首播先后. 环 (bgm 偶有互指) 用首播打断.
 * 番剧A -> 剧场版 -> 番剧B 这种, 剧场版就排在两季之间.
 */
function storyOrder (main: readonly ANiMeRecord[]): ANiMeRecord[] {
    const ids = new Set(main.map((r) => r.anime_data.id));
    const after = new Map<number, Set<number>>(); // a -> 排在 a 之后的
    const indeg = new Map<number, number>(main.map((r) => [r.anime_data.id, 0]));
    const edge = (a: number, b: number) => {
        if (a === b || !ids.has(a) || !ids.has(b)) return;
        if (!after.has(a)) after.set(a, new Set());
        if (after.get(a)!.has(b)) return;
        after.get(a)!.add(b);
        indeg.set(b, (indeg.get(b) ?? 0) + 1);
    };
    for (const r of main) {
        for (const rel of r.anime_data.relations || []) {
            if (rel.relation === "续集") edge(r.anime_data.id, rel.id);
            else if (rel.relation === "前传") edge(rel.id, r.anime_data.id);
        }
    }
    const byId = new Map(main.map((r) => [r.anime_data.id, r]));
    const pending = [...main].sort(byAirDate);
    const out: ANiMeRecord[] = [];
    const done = new Set<number>();
    while (out.length < main.length) {
        // 入度为 0 中首播最早的; 都不为 0 (环) 就取首播最早的未排条目
        const next = pending.find((r) => !done.has(r.anime_data.id) && (indeg.get(r.anime_data.id) ?? 0) <= 0)
            ?? pending.find((r) => !done.has(r.anime_data.id))!;
        const id = next.anime_data.id;
        done.add(id);
        out.push(byId.get(id)!);
        after.get(id)?.forEach((b) => indeg.set(b, (indeg.get(b) ?? 0) - 1));
    }
    return out;
}

/**
 * 把一个系列里的条目分成 主线 / 特别篇.
 * 主线: 与起点经 前传 / 续集 边 (只走库内) 连通的全部条目, 含链上的剧场版 / OVA / 完结篇 (它们属于正片剧情, 只是不编季号);
 *   起点 = 有 前传 / 续集 边、集数多、首播早的那部.
 *   另外: 前一季没收录导致链断开时, 标题写明季号且与起点同名前缀的也算主线; 只靠名字兜底挂进来的 >= 6 话长篇也算主线.
 * 特别篇: 其余 (番外篇 / 总集篇 / 不在链上的剧场版 / OVA).
 */
function splitSeasons (members: ANiMeRecord[], ids: ReadonlySet<number>): AnimeSeason[] {
    const mainAdj = new Map<number, Set<number>>();
    const touched = new Set<number>();
    const link = (a: number, b: number) => {
        if (!mainAdj.has(a)) mainAdj.set(a, new Set());
        if (!mainAdj.has(b)) mainAdj.set(b, new Set());
        mainAdj.get(a)!.add(b);
        mainAdj.get(b)!.add(a);
    };
    const memberIds = new Set(members.map((r) => r.anime_data.id));
    for (const r of members) {
        for (const rel of r.anime_data.relations || []) {
            if (!ids.has(rel.id) || !memberIds.has(rel.id) || rel.id === r.anime_data.id) continue;
            touched.add(r.anime_data.id);
            touched.add(rel.id);
            if (MAIN_RELATIONS.has(rel.relation)) link(r.anime_data.id, rel.id);
        }
    }

    const sorted = [...members].sort(byAirDate);
    const start = [...sorted].sort((a, b) => {
        const ea = mainAdj.has(a.anime_data.id) ? 1 : 0;
        const eb = mainAdj.has(b.anime_data.id) ? 1 : 0;
        if (ea !== eb) return eb - ea;
        const na = normalEpisodeCount(a) >= MAIN_MIN_EPS ? 1 : 0;
        const nb = normalEpisodeCount(b) >= MAIN_MIN_EPS ? 1 : 0;
        if (na !== nb) return nb - na;
        return byAirDate(a, b);
    })[0];

    const linked = collectSeries(start.anime_data.id, mainAdj);
    const base = norm(stripSeasonSuffix(start.anime_data.name)).toLowerCase();
    const baseCn = norm(stripSeasonSuffix(start.anime_data.name_cn)).toLowerCase();
    const sameBase = (r: ANiMeRecord): boolean =>
        (!!base && norm(r.anime_data.name).toLowerCase().startsWith(base)) ||
        (!!baseCn && norm(r.anime_data.name_cn).toLowerCase().startsWith(baseCn));
    const isMain = (r: ANiMeRecord): boolean => {
        if (linked.has(r.anime_data.id)) return true; // 链上的剧场版 / OVA 也是正片
        if (isShortWork(r)) return false;
        const mk = markerOf(r);
        // 前一季没收录导致 续集 链断开 (史莱姆 第2期 第2部分 不在库 -> 第3期 断链): 标题写明季号且与起点同名前缀, 也算主线
        if ((mk.no !== null || mk.final) && sameBase(r)) return true;
        // 没有任何库内 relations 的条目 (只靠名字兜底连进来): 按集数判
        return !touched.has(r.anime_data.id) && normalEpisodeCount(r) >= MAIN_MIN_EPS;
    };

    const main = storyOrder(sorted.filter(isMain));
    const specials = sorted.filter((r) => !isMain(r)).map((record) => specialSeason(record, members));
    return [...numberMainSeasons(main, members), ...specials];
}

/** 不编季号的作品种类: 剧场版 / 完结篇 前篇 / OVA / 待播 / 关系名 / 特别篇 */
function workKind (record: ANiMeRecord, members: readonly ANiMeRecord[]): string {
    const id = record.anime_data.id;
    const mk = markerOf(record);
    const eps = normalEpisodeCount(record);
    const title = norm(`${record.anime_data.name_cn} ${record.anime_data.name}`);
    if (mk.movie) return "剧场版";
    const fin = title.match(/(完结篇|完結編)\s*(前篇|前編|后篇|後編|中篇|中編)?/);
    if (fin) return fin[2] ? `完结篇 ${fin[2].replace("編", "篇").replace("後", "后")}` : "完结篇";
    let rel: string | null = null;
    for (const m of members) {
        for (const r of m.anime_data.relations || []) if (r.id === id && SPECIAL_RELATIONS.has(r.relation)) { rel = r.relation; break; }
        if (rel) break;
    }
    if (rel === "剧场版") return "剧场版";
    if (eps === 0) return "待播";
    if (/\b(?:OVA|OAD)\b/i.test(title)) return "OVA";
    if (rel && rel !== "主线故事") return rel;
    return "特别篇";
}

/**
 * 给主线编季号 (按观看顺序): 短篇 (剧场版 / OVA / 完结篇) 不编号, 也不打断前后季的编号;
 * 写明季号的直接用; 写明 Part.N (N > 1) / 后半 的并入上一季; 最终季接在上一季后; 其余顺延 +1.
 * 同一季出现多部分时补上 part (第一部分回填为 1).
 */
function numberMainSeasons (main: readonly ANiMeRecord[], members: readonly ANiMeRecord[]): AnimeSeason[] {
    const out: AnimeSeason[] = [];
    let prev: AnimeSeason | null = null;
    for (const record of main) {
        if (isShortWork(record)) {
            out.push({ record, kind: "main", no: null, part: null, final: false, special: workKind(record, members) });
            continue;
        }
        const mk = markerOf(record);
        const prevNo = prev?.no ?? 0;
        let no: number;
        if (mk.no !== null) no = mk.no;
        else if (mk.final) no = prev?.final ? prevNo : prevNo + 1;
        else if (mk.part !== null && mk.part > 1 && prev) no = prevNo;
        else no = prevNo + 1;
        const final = mk.final || (!!prev?.final && no === prevNo);
        let part: number | null = mk.part;
        if (prev && no === prevNo) {
            if (prev.part === null) prev.part = 1;
            if (part === null || part <= prev.part) part = prev.part + 1;
        } else if (part === 1) {
            part = null; // "前半" 但后半没收录: 不必标部分
        }
        const season: AnimeSeason = { record, kind: "main", no, part, final, special: null };
        out.push(season);
        prev = season;
    }
    return out;
}

function specialSeason (record: ANiMeRecord, members: readonly ANiMeRecord[]): AnimeSeason {
    return { record, kind: "special", no: null, part: null, final: false, special: workKind(record, members) };
}

/** 未收录的前传 / 续集, 带上它挂在哪一部的前 / 后 (过渡页按这个插进主线) */
function missingOf (members: readonly ANiMeRecord[], ids: ReadonlySet<number>): MissingWork[] {
    const out = new Map<number, MissingWork>();
    for (const r of members) {
        for (const rel of r.anime_data.relations || []) {
            if (ids.has(rel.id) || !MAIN_RELATIONS.has(rel.relation)) continue;
            if (rel.type !== undefined && rel.type !== SubjectType.ANIME) continue;
            if (!out.has(rel.id)) out.set(rel.id, { rel, anchor: r.anime_data.id, before: rel.relation === "前传" });
        }
    }
    return [...out.values()];
}

/** 全库 -> 系列列表 (含只有一部的系列). 顺序: 按系列 key 的首播日期 */
export function buildSeriesList (records: readonly ANiMeRecord[]): AnimeSeries[] {
    const ids = new Set(records.map((r) => r.anime_data.id));
    const byId = new Map(records.map((r) => [r.anime_data.id, r]));
    const adjacency = seriesAdjacency(records);
    const seen = new Set<number>();
    const list: AnimeSeries[] = [];
    for (const r of [...records].sort(byAirDate)) {
        const id = r.anime_data.id;
        if (seen.has(id)) continue;
        const comp = collectSeries(id, adjacency);
        comp.forEach((x) => seen.add(x));
        const members = [...comp].map((x) => byId.get(x)).filter((x): x is ANiMeRecord => !!x);
        const seasons = splitSeasons(members, ids);
        const first = seasons[0].record.anime_data;
        const missingAt = missingOf(members, ids);
        list.push({
            key: first.id,
            title: stripSeasonSuffix(first.name_cn || first.name),
            originalTitle: stripSeasonSuffix(first.name),
            seasons,
            missing: missingAt.map((m) => m.rel),
            missingAt,
            related: [],
        });
    }
    // 同一世界观的其它系列 (衍生 / 主线 / 相同世界观), 只连库内
    const keyOf = new Map<number, AnimeSeries>();
    for (const s of list) for (const x of s.seasons) keyOf.set(x.record.anime_data.id, s);
    for (const s of list) {
        const seen2 = new Map<number, RelatedSeries>();
        for (const x of s.seasons) {
            for (const rel of x.record.anime_data.relations || []) {
                const label = WORLD_RELATIONS[rel.relation];
                const other = keyOf.get(rel.id);
                if (!label || !other || other === s) continue;
                const cur = seen2.get(other.key);
                // 同一对系列写了多种关系时, 主线 > 衍生 > 外传 > 同一世界观
                const rank = (l: string) => ["主线", "衍生", "外传", "同一世界观"].indexOf(l);
                if (!cur || rank(label) < rank(cur.relation)) seen2.set(other.key, { key: other.key, relation: label });
            }
        }
        s.related = [...seen2.values()].sort((a, b) =>
            ["主线", "衍生", "外传", "同一世界观"].indexOf(a.relation) - ["主线", "衍生", "外传", "同一世界观"].indexOf(b.relation)
            || (keyOf.get(a.key)!.seasons[0].record.anime_data.date || "9999").localeCompare(keyOf.get(b.key)!.seasons[0].record.anime_data.date || "9999"));
    }
    return list;
}

let memoInput: readonly ANiMeRecord[] | null = null;
let memoList: AnimeSeries[] = [];
let memoById = new Map<number, AnimeSeries>();

/** 记忆化: 记录数组引用不变就不重算 (番剧库 / 过渡页 / 详情页共用一份) */
export function getSeriesList (records: readonly ANiMeRecord[]): { list: AnimeSeries[]; byId: Map<number, AnimeSeries> } {
    if (records !== memoInput) {
        memoInput = records;
        memoList = buildSeriesList(records);
        memoById = new Map();
        for (const s of memoList) for (const season of s.seasons) memoById.set(season.record.anime_data.id, s);
    }
    return { list: memoList, byId: memoById };
}

export function useAnimeSeries (records: readonly ANiMeRecord[]): { list: AnimeSeries[]; byId: Map<number, AnimeSeries> } {
    return useMemo(() => getSeriesList(records), [records]);
}

// ---------------------------------------------------------------- 聚合取值

/** 主线 (观看顺序, 含链上的剧场版 / OVA) */
export const mainSeasons = (s: AnimeSeries): AnimeSeason[] => s.seasons.filter((x) => x.kind === "main");
/** 主线上编了季号的 TV 季 */
export const tvSeasons = (s: AnimeSeries): AnimeSeason[] => s.seasons.filter((x) => x.kind === "main" && x.no !== null);
export const specialSeasons = (s: AnimeSeries): AnimeSeason[] => s.seasons.filter((x) => x.kind === "special");
export const isMultiSeason = (s: AnimeSeries): boolean => s.seasons.length > 1;
/** 正篇季数: 按季号去重 (分割放送的上下半算一季), 不是正篇条目数 */
export const seasonCount = (s: AnimeSeries): number => new Set(tvSeasons(s).map((x) => x.no)).size;

/** "3 季 + 1 剧场版 · 2 特别篇": 季数 / 主线上的短篇 / 主线外的特别篇 */
export function seriesCountText (s: AnimeSeries): string {
    const n = seasonCount(s);
    const shorts = mainSeasons(s).filter((x) => x.no === null);
    const movies = shorts.filter((x) => x.special === "剧场版").length;
    const others = shorts.length - movies;
    const sp = specialSeasons(s).length;
    const parts = [n ? `${n} 季` : ""];
    if (movies) parts.push(`${movies} 剧场版`);
    if (others) parts.push(`${others} 篇`);
    return parts.filter(Boolean).join(" + ") + (sp ? ` · ${sp} 特别篇` : "");
}

/** 系列聚合状态: 任一季在看 -> 在看; 正篇全看过 -> 看过; 看过 + 想看 -> 追到第 N 季; 全想看 -> 想看; 其余取最后一季 */
export function seriesStatus (s: AnimeSeries): { status: WatchStatus; label: string } {
    const main = mainSeasons(s);
    const sts = main.map((x) => x.record.user_status.watch_status);
    if (s.seasons.some((x) => x.record.user_status.watch_status === WatchStatus.WATCHING)) return { status: WatchStatus.WATCHING, label: "在看" };
    if (sts.length && sts.every((x) => x === WatchStatus.WATCHED)) return { status: WatchStatus.WATCHED, label: "看过" };
    if (sts.length && sts.every((x) => x === WatchStatus.WANT_TO_WATCH)) return { status: WatchStatus.WANT_TO_WATCH, label: "想看" };
    const lastWatched = [...main].reverse().find((x) => x.record.user_status.watch_status === WatchStatus.WATCHED);
    if (lastWatched && sts.includes(WatchStatus.WANT_TO_WATCH)) return { status: WatchStatus.WATCHING, label: `追到${seasonLabel(lastWatched).replace(/\s/g, "")}` };
    const last = (main[main.length - 1] ?? s.seasons[s.seasons.length - 1]).record.user_status.watch_status;
    const LABEL: Record<number, string> = { 1: "想看", 2: "看过", 3: "在看", 4: "搁置", 5: "抛弃" };
    return { status: last, label: LABEL[last] ?? "未知" };
}

/** 主线合计进度 (含链上的剧场版) */
export function seriesProgress (s: AnimeSeries): { watched: number; total: number; ratio: number } {
    let watched = 0;
    let total = 0;
    for (const { record } of mainSeasons(s)) {
        const n = normalEpisodeCount(record);
        total += n;
        watched += record.user_status.watch_status === WatchStatus.WATCHED ? n : Math.min(n || Infinity, Math.max(0, record.user_status.watched_eps || 0));
    }
    return { watched, total, ratio: total > 0 ? Math.min(1, watched / total) : 0 };
}

export function seriesScoreRange (s: AnimeSeries): { min: number; max: number } | null {
    const scores = s.seasons.map((x) => x.record.anime_data.score).filter((x) => x > 0);
    if (!scores.length) return null;
    return { min: Math.min(...scores), max: Math.max(...scores) };
}

export function seriesYearRange (s: AnimeSeries): string {
    const years = s.seasons.map((x) => Number((x.record.anime_data.date || "").slice(0, 4))).filter((y) => y > 1900);
    if (!years.length) return "";
    const lo = Math.min(...years);
    const hi = Math.max(...years);
    return lo === hi ? String(lo) : `${lo}–${hi}`;
}

export const seriesLastUpdate = (s: AnimeSeries): string =>
    s.seasons.reduce((m, x) => ((x.record.user_status.last_update ?? "") > m ? x.record.user_status.last_update ?? "" : m), "");

export const seriesFirstAir = (s: AnimeSeries): string => s.seasons[0].record.anime_data.date || "";

export const seriesMaxScore = (s: AnimeSeries): number => seriesScoreRange(s)?.max ?? 0;

/** 封面: 有在看的季用在看那季, 否则第 1 季 */
export function seriesCoverId (s: AnimeSeries): number {
    const watching = s.seasons.find((x) => x.record.user_status.watch_status === WatchStatus.WATCHING);
    return (watching ?? s.seasons[0]).record.anime_data.id;
}

/** 各季人工标注的并集, 保持首次出现的顺序 */
export function seriesTags (s: AnimeSeries): string[] {
    const out: string[] = [];
    for (const { record } of s.seasons) for (const t of getRecordTags(record)) if (!out.includes(t)) out.push(t);
    return out;
}

/** 继续观看: 按顺序第一个在看的正篇季, 否则第一个没看完的正篇季; 全看完返回 null */
export function nextUpSeason (s: AnimeSeries): AnimeSeason | null {
    const main = mainSeasons(s);
    return main.find((x) => x.record.user_status.watch_status === WatchStatus.WATCHING)
        ?? main.find((x) => x.record.user_status.watch_status !== WatchStatus.WATCHED)
        ?? null;
}

/** 完整标签: "第 2 季" / "第 2 季 · 第 2 部分" / "最终季 · 第 2 部分" / "剧场版" / "OVA" ... */
export function seasonLabel (x: AnimeSeason): string {
    if (x.no === null) return x.special ?? "特别篇";
    const head = x.final ? (markerOf(x.record).no !== null ? `第 ${x.no} 季 · 最终季` : "最终季") : `第 ${x.no} 季`;
    return x.part ? `${head} · 第 ${x.part} 部分` : head;
}

/** 短标签 (卡片 chip / 时间线): "S2" / "S2-2" / "FS" / "FS-2" / "剧场版" / "SP" */
export function seasonShort (x: AnimeSeason): string {
    if (x.no === null) {
        const sp = x.special ?? "";
        return sp === "剧场版" || sp === "OVA" || sp === "待播" ? sp : sp.startsWith("完结篇") ? "完结篇" : "SP";
    }
    const head = x.final ? "FS" : `S${x.no}`;
    return x.part ? `${head}-${x.part}` : head;
}

/** 未收录的前传 / 续集 (只有关系里的名字): 标题写了季号就显示季号, 否则显示关系名 */
export function missingLabel (rel: Relation): string {
    const mk = parseSeasonMarker(rel.name_cn, rel.name);
    if (mk.no === null && !mk.final) return mk.part ? `第 ${mk.part} 部分` : rel.relation;
    const head = mk.final ? "最终季" : `第 ${mk.no} 季`;
    return mk.part ? `${head} · 第 ${mk.part} 部分` : head;
}

// ---------------------------------------------------------------- 番剧库「系列」视图: 命中 / 排序 / 分组

/** 一个命中的系列: hits = 系列里满足当前筛选的季 (id) */
export interface SeriesHit {
    series: AnimeSeries;
    hits: Set<number>;
}

/** 已筛选的单季记录 -> 命中的系列 (系列只要有一季命中就出现) */
export function seriesHits (filtered: readonly ANiMeRecord[], byId: ReadonlyMap<number, AnimeSeries>): SeriesHit[] {
    const out = new Map<number, SeriesHit>();
    for (const r of filtered) {
        const s = byId.get(r.anime_data.id);
        if (!s) continue;
        let h = out.get(s.key);
        if (!h) { h = { series: s, hits: new Set() }; out.set(s.key, h); }
        h.hits.add(r.anime_data.id);
    }
    return [...out.values()];
}

function compareSeries (sort: SortKey): (a: SeriesHit, b: SeriesHit) => number {
    switch (sort) {
        case "air":
            return (a, b) => (seriesFirstAir(b.series) || "0000").localeCompare(seriesFirstAir(a.series) || "0000");
        case "score":
            return (a, b) => seriesMaxScore(b.series) - seriesMaxScore(a.series);
        case "name":
            return (a, b) => a.series.title.localeCompare(b.series.title, "zh-Hans-CN");
        case "update":
        default:
            return (a, b) => seriesLastUpdate(b.series).localeCompare(seriesLastUpdate(a.series));
    }
}

export interface SeriesGroup {
    key: string;
    label: string;
    items: SeriesHit[];
}

/**
 * 与 filters.ts 的 groupRecords 同一套分组规则, 取值换成系列:
 * 首播 -> 第 1 季所在番剧季度; 最近更新 -> 系列内最新更新的年月; 评分 / 名称 -> 不分组.
 */
export function groupSeries (hits: readonly SeriesHit[], sort: SortKey): SeriesGroup[] {
    const sorted = [...hits].sort(compareSeries(sort));
    if (sort === "score" || sort === "name") return sorted.length ? [{ key: "all", label: "", items: sorted }] : [];
    const groups: SeriesGroup[] = [];
    const byKey = new Map<string, SeriesGroup>();
    for (const h of sorted) {
        let key: string;
        let label: string;
        if (sort === "air") {
            const s = animeSeasonOf(seriesFirstAir(h.series));
            key = s ? `${s.year}-${s.index}` : "unknown";
            label = s ? `${s.year} ${s.season} · ${s.index * 3 + 1} 月番` : "未知日期";
        } else {
            const d = seriesLastUpdate(h.series);
            const m = d.match(/(\d{4})-(\d{2})/);
            key = m ? `${m[1]}-${m[2]}` : "unknown";
            label = m ? `${m[1]} 年 ${Number(m[2])} 月更新` : "未知日期";
        }
        let g = byKey.get(key);
        if (!g) { g = { key, label, items: [] }; byKey.set(key, g); groups.push(g); }
        g.items.push(h);
    }
    return groups;
}
