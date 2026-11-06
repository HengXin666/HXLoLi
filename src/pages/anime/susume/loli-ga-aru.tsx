import React from "react";
import SusumeCategoryPage from "@site/src/components/anime/susume/SusumeCategoryPage";

/**
 * /anime/susume/loli-ga-aru —— 推荐分类「ロリがある」(有萝莉角色) 专属页. 旧式 /anime?tag=ロリがある 由番剧首页客户端重定向到这里.
 * 页面本体见 components/anime/susume/SusumeCategoryPage.tsx.
 */
export default function Susume_loligaaru (): React.ReactElement {
    return <SusumeCategoryPage susume="loligaaru" />;
}
