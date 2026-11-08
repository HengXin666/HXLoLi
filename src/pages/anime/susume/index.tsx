import React from "react";
import SusumeHubPage from "@site/src/components/anime/susume/SusumeHubPage";

/**
 * /anime/susume —— 推荐总览: 番剧库同款页头与子导航, 每个推荐分类一节, 按番名 A-Z 露出前 12 张卡. 页面本体见 components/anime/susume/SusumeHubPage.tsx.
 * .agents/notes/implemented/feature/2026-11-08-anime-susume-wall.md
 */
export default function SusumeIndex (): React.ReactElement {
    return <SusumeHubPage />;
}
