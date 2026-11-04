import React from "react";
import SusumePage from "@site/src/components/anime/SusumePage";

/**
 * /anime/susume/monosugoi —— 推荐分类「物凄い」(我最喜欢的番) 专属页. 独立路由, 不是 /anime?tag=物凄い 的筛选结果;
 * 旧式 /anime?tag=物凄い 由番剧首页客户端重定向到这里 (src/pages/anime/index.tsx).
 * 只读人工标注 HXLoLi-ANiMe/data/labels/<id>.json, 按本人评分降序, 附一句话观后感. 页面本体见 components/anime/SusumePage.tsx.
 * hx-anime-tag-refactor-design.md 第 3.1 节
 * .agents/notes/proposed/feature/2026-11-04-hx-anime-site-manual-labels.md
 */
export default function Susume_monosugoi (): React.ReactElement {
    return <SusumePage susume="monosugoi" />;
}
