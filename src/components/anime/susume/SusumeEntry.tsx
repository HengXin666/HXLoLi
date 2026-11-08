import React from "react";
import Link from "@docusaurus/Link";
import { ANIME_SUSUME_DEFS, type AnimeSusumeKey } from "@site/data/animeTags";
import type { ANiMeRecord } from "@site/src/utils/anime/types";
import type { AnimeLabelsStatus } from "@site/src/utils/anime/animeStore";
import { recordsOfSusume, susumePath } from "@site/src/utils/anime/tags";

/**
 * 番剧首页 /anime 子导航里的推荐分类: 四个分类 (名字 + 数量), 放在 "全部" 与 "キャラ" 之间.
 * 只输出链接 (名字 + 数量, 不带悬停提示), 样式由页面的子导航决定. susume 分类页也用同一条子导航, current 标出当前分类.
 */
export default function SusumeEntry ({ records, labelsStatus, countClass, current }: {
    records: readonly ANiMeRecord[];
    labelsStatus: AnimeLabelsStatus;
    countClass?: string;
    current?: AnimeSusumeKey;
}): React.ReactElement {
    const counts = React.useMemo(
        () => ANIME_SUSUME_DEFS.map((def) => ({ def, count: recordsOfSusume(records, def).length })),
        // eslint-disable-next-line react-hooks/exhaustive-deps
        [records, labelsStatus],
    );
    const loading = labelsStatus === "loading" || records.length === 0;

    return (
        <>
            {counts.map(({ def, count }) => (
                <Link key={def.key} to={susumePath(def)} aria-current={def.key === current ? "page" : undefined}>
                    {def.display}
                    <span className={countClass}>{loading ? "…" : count}</span>
                </Link>
            ))}
        </>
    );
}
