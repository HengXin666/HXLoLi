import React from "react";
import SusumeCategoryPage from "@site/src/components/anime/susume/SusumeCategoryPage";

/**
 * /anime/susume/imouto —— 推荐分类「シスコン」(妹控) 专属页. 旧式 /anime?tag=シスコン 由番剧首页客户端重定向到这里.
 * 页面本体见 components/anime/susume/SusumeCategoryPage.tsx.
 */
export default function Susume_imouto (): React.ReactElement {
    return <SusumeCategoryPage susume="imouto" />;
}
