import React from "react";
import SusumeCategoryPage from "@site/src/components/anime/susume/SusumeCategoryPage";

/**
 * /anime/susume/monosugoi —— 推荐分类「物凄い」(我最喜欢的番) 专属页. 旧式 /anime?tag=物凄い 由番剧首页客户端重定向到这里.
 * 页面本体见 components/anime/susume/SusumeCategoryPage.tsx.
 * .agents/notes/proposed/feature/2026-11-04-hx-anime-site-manual-labels.md
 */
export default function Susume_monosugoi (): React.ReactElement {
    return <SusumeCategoryPage susume="monosugoi" />;
}
