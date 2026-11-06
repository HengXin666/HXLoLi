import React from "react";
import SusumeCategoryPage from "@site/src/components/anime/susume/SusumeCategoryPage";

/**
 * /anime/susume/loli —— 推荐分类「ロリ」(萝莉为卖点) 专属页. 旧式 /anime?tag=ロリ 由番剧首页客户端重定向到这里.
 * 页面本体见 components/anime/susume/SusumeCategoryPage.tsx.
 */
export default function Susume_loli (): React.ReactElement {
    return <SusumeCategoryPage susume="loli" />;
}
