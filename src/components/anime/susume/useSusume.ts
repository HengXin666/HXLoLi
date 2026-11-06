import React from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { ANIME_SUSUME_DEFS, type AnimeSusumeDef, type AnimeSusumeKey } from "@site/data/animeTags";
import { useAnimeLabelsStatus, useAnimeRecordsState } from "@site/src/utils/anime/animeStore";
import { getRecordRate, getRecordReview, recordsOfSusume, susumePath } from "@site/src/utils/anime/tags";
import { statusLabel } from "@site/src/utils/anime/filters";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import { WatchStatus, type ANiMeRecord } from "@site/src/utils/anime/types";

/** 每个推荐分类的展示文案: 罗马音副标 + 一句说明 (2026-11-06 第二版去掉了舞台猫娘的台词) */
export const SUSUME_COPY: Record<AnimeSusumeKey, { romaji: string; pitch: string }> = {
    monosugoi: { romaji: "monosugoi", pitch: "看完会想马上推荐给别人的那种." },
    loli: { romaji: "loli", pitch: "小个子角色是主角或核心看点的番." },
    loligaaru: { romaji: "loli ga aru", pitch: "里面有小个子角色就算, 不一定是卖点." },
    imouto: { romaji: "shisukon", pitch: "兄弟姐妹关系是主要看点的番." },
};

export const susumeAccent = (key: AnimeSusumeKey): string => `var(--hx-susume-${key})`;

export interface SusumePick {
    id: number;
    rank: number;
    record: ANiMeRecord;
    title: string;
    original: string;
    cover: string;
    href: string;
    year: string;
    rate?: number;
    score: number;
    review: string;
    status: string;
    eps: number;
}

export interface SusumeList {
    def: AnimeSusumeDef;
    path: string;
    picks: SusumePick[];
    avgRate: number | null;
    years: [number, number] | null;
}

const cover = (id: number): string => toAnimeCdnUrl(`/data/anime/${id}.jpg`);

function toPick (r: ANiMeRecord, i: number): SusumePick {
    const a = r.anime_data;
    return {
        id: a.id,
        rank: i + 1,
        record: r,
        title: a.name_cn || a.name,
        original: a.name_cn && a.name !== a.name_cn ? a.name : "",
        cover: cover(a.id),
        href: `/anime/details?id=${a.id}`,
        year: (a.date ?? "").slice(0, 4),
        rate: getRecordRate(r),
        score: a.score || 0,
        review: getRecordReview(r),
        status: statusLabel(r.user_status.watch_status),
        eps: a.eps || 0,
    };
}

function summarize (def: AnimeSusumeDef, picks: SusumePick[]): SusumeList {
    const rated = picks.map((p) => p.rate).filter((x): x is number => typeof x === "number");
    const years = picks.map((p) => Number(p.year)).filter((y) => y > 1900);
    return {
        def,
        path: susumePath(def),
        picks,
        avgRate: rated.length ? rated.reduce((s, x) => s + x, 0) / rated.length : null,
        years: years.length ? [Math.min(...years), Math.max(...years)] : null,
    };
}

export interface SusumeData {
    lists: SusumeList[];
    /** Hero 封面墙: 有推荐用推荐, 一部都没有时用看过的高分番做背景 (纯装饰) */
    backdrop: string[];
    loading: boolean;
    error: string | null;
    retry: () => void;
}

/** 三个推荐分类的数据 (只读人工标注), 加上 Hero 背景和三态 */
export function useSusumeData (): SusumeData {
    const { siteConfig } = useDocusaurusContext();
    const { records, status, error, retry } = useAnimeRecordsState(siteConfig.baseUrl);
    const labelsStatus = useAnimeLabelsStatus();

    const lists = React.useMemo(
        () => ANIME_SUSUME_DEFS.map((def) => summarize(def, recordsOfSusume(records, def).map(toPick))),
        // 标注比记录晚到, 到了之后 recordsOfSusume 的结果会变
        // eslint-disable-next-line react-hooks/exhaustive-deps
        [records, labelsStatus],
    );

    const backdrop = React.useMemo(() => {
        const picked = [...new Set(lists.flatMap((l) => l.picks.map((p) => p.cover)))];
        if (picked.length >= 6) return picked;
        const top = [...records]
            .filter((r) => r.user_status.watch_status === WatchStatus.WATCHED || r.user_status.watch_status === WatchStatus.WATCHING)
            .sort((a, b) => (b.anime_data.score || 0) - (a.anime_data.score || 0))
            .slice(0, 30)
            .map((r) => cover(r.anime_data.id));
        return [...new Set([...picked, ...top])];
    }, [lists, records]);

    const loading = (status === "loading" && records.length === 0) || labelsStatus === "loading";
    const err = status === "error" && records.length === 0
        ? `追番记录没能加载出来${error ? `: ${error}` : ""}`
        : labelsStatus === "error" ? "标注没能加载出来 (CDN 或 GitHub 接口暂时不可用)" : null;

    return { lists, backdrop, loading, error: err, retry: () => (labelsStatus === "error" ? window.location.reload() : retry()) };
}

export function listOf (data: SusumeData, key: AnimeSusumeKey): SusumeList {
    return data.lists.find((l) => l.def.key === key)!;
}
