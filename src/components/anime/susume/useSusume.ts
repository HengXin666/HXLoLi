import React from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { ANIME_SUSUME_DEFS, type AnimeSusumeDef, type AnimeSusumeKey } from "@site/data/animeTags";
import { useAnimeLabelsStatus, useAnimeRecordsState } from "@site/src/utils/anime/animeStore";
import { recordsOfSusume, susumeDefOfTag, susumePath } from "@site/src/utils/anime/tags";
import { DEFAULT_FILTERS, parseFilters, serializeFilters, type AnimeFilters } from "@site/src/utils/anime/filters";
import type { ANiMeRecord } from "@site/src/utils/anime/types";

export interface SusumeList {
    def: AnimeSusumeDef;
    path: string;
    /** 这一类的番, 按番名 A-Z (中文按拼音) */
    records: ANiMeRecord[];
}

let collator: Intl.Collator | null = null;
const titleOf = (r: ANiMeRecord): string => r.anime_data.name_cn || r.anime_data.name;
/** 番名排序: 中文按拼音, 数字按数值 */
export function byTitle (a: ANiMeRecord, b: ANiMeRecord): number {
    collator ??= new Intl.Collator("zh-Hans-u-co-pinyin", { sensitivity: "base", numeric: true });
    return collator.compare(titleOf(a), titleOf(b));
}

export interface SusumeData {
    /** 全部追番记录 (子导航计数要用) */
    all: readonly ANiMeRecord[];
    lists: SusumeList[];
    loading: boolean;
    error: string | null;
    retry: () => void;
}

/**
 * 四个推荐分类的数据 (只读人工标注), 每类按番名 A-Z, 加上三态.
 * 排序的取舍见 SusumeCategoryPage 上方引用的 note.
 */
export function useSusumeData (): SusumeData {
    const { siteConfig } = useDocusaurusContext();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl, "lite");
    const labelsStatus = useAnimeLabelsStatus();

    const lists = React.useMemo(
        () => ANIME_SUSUME_DEFS.map((def) => ({
            def,
            path: susumePath(def),
            records: [...recordsOfSusume(records, def)].sort(byTitle),
        })),
        // 标注比记录晚到, 到了之后 recordsOfSusume 的结果会变
        // eslint-disable-next-line react-hooks/exhaustive-deps
        [records, labelsStatus],
    );

    const loading = (status === "loading" && records.length === 0) || labelsStatus === "loading";
    const err = status === "error" && records.length === 0
        ? `追番记录没能加载出来${error ? `: ${error}` : ""}`
        : labelsStatus === "error" ? "标注没能加载出来 (CDN 或 GitHub 接口暂时不可用)" : null;

    return { all: records, lists, loading, error: err, retry: () => (labelsStatus === "error" ? window.location.reload() : retry()) };
}

export function listOf (data: SusumeData, key: AnimeSusumeKey): SusumeList {
    return data.lists.find((l) => l.def.key === key)!;
}

/**
 * 分类页的筛选也写在 URL 上, 与番剧库同一套 (utils/anime/filters.ts), 只是默认排序是「名称」:
 * 地址栏没有 sort 时按名称, 选回名称时去掉 sort, 选「最近更新」时显式写 sort=update.
 */
export function parseSusumeFilters (search: string): AnimeFilters {
    const f = parseFilters(search);
    return new URLSearchParams(search).has("sort") ? f : { ...f, sort: "name" };
}

export function serializeSusumeFilters (f: AnimeFilters): string {
    const p = new URLSearchParams(serializeFilters({ ...f, sort: f.sort === "name" ? DEFAULT_FILTERS.sort : f.sort }));
    if (f.sort === DEFAULT_FILTERS.sort) p.set("sort", f.sort);
    return p.toString();
}

/** 子导航两端: 番剧库 (/anime, 默认排序「最近更新」) 与推荐分类页 (/anime/susume/<slug>, 默认排序「名称」) */
export type AnimeListKind = "index" | "susume";

/**
 * 在「全部」与各推荐分类之间切换时带上当前筛选 (搜索 / 状态 / 标签 / 年份 / 评分 / 排序 / 单部·系列),
 * 不然一切 tab 全部回到默认. 两边默认排序不同, 所以不能直接拷查询串: 先按来源页解析成完整状态,
 * 再按目标页序列化, 画面上是什么排序过去还是什么排序.
 * 去番剧库时丢掉推荐分类标签 (ロリ 等), 否则会触发首页的旧链接跳转又弹回分类页.
 * 返回带 "?" 的查询串, 没有条件时返回空串.
 */
export function carryFilterSearch (search: string, from: AnimeListKind, to: AnimeListKind): string {
    const f = from === "susume" ? parseSusumeFilters(search) : parseFilters(search);
    const next: AnimeFilters = to === "index" ? { ...f, tags: f.tags.filter((t) => !susumeDefOfTag(t)) } : f;
    const out = to === "susume" ? serializeSusumeFilters(next) : serializeFilters(next);
    return out ? `?${out.replace(/%2C/g, ",")}` : "";
}
