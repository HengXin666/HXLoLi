import React from "react";
import SusumeHubPage from "@site/src/components/anime/susume/SusumeHubPage";

/**
 * /anime/susume —— 推荐总览: 三个推荐分类的大横幅 + 评分最高的几部. 页面本体见 components/anime/susume/SusumeHubPage.tsx.
 * .agents/notes/implemented/feature/2026-11-06-anime-susume-neko-redesign.md
 */
export default function SusumeIndex (): React.ReactElement {
    return <SusumeHubPage />;
}
