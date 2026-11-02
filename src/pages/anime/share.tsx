import React from "react";
import Layout from "@theme/Layout";
import AnimeShareBoard from "@site/src/components/anime/AnimeShareBoard";
import { SideCats } from "@site/src/components/BlogWithCats";

/**
 * /anime/share?<番剧库同款筛选>&pick=<压缩的 id 集合>: 可交互番单, 点番标记"我看过", 生成自己的链接再分享.
 * 页面只做外壳, 逻辑在 src/components/anime/AnimeShareBoard.tsx.
 * 两侧猫娘 SideCats 按分享页正文宽 1200 算留白, 放不下就不渲染.
 * .agents/notes/implemented/feature/2026-11-01-anime-share-link.md
 * .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md
 */
export default function AnimeSharePage (): React.ReactElement {
    return (
        <Layout title="分享番单" description="点击选择你看过的番, 生成自己的链接分享给别人">
            <SideCats contentWidth={1200} />
            <AnimeShareBoard />
        </Layout>
    );
}
