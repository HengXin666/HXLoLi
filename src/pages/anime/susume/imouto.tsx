import React from "react";
import SusumePage from "@site/src/components/anime/SusumePage";

/**
 * /anime/susume/imouto —— 推荐分类「シスコン」(妹控) 专属页. 独立路由, 不是 /anime?tag=シスコン 的筛选结果;
 * 旧式 /anime?tag=シスコン 由番剧首页客户端重定向到这里 (src/pages/anime/index.tsx).
 * 只读人工标注 HXLoLi-ANiMe/data/labels/<id>.json, 按本人评分降序, 附一句话观后感. 页面本体见 components/anime/SusumePage.tsx.
 * hx-anime-tag-refactor-design.md 第 3.1 节
 */
export default function Susume_imouto (): React.ReactElement {
    return <SusumePage susume="imouto" />;
}
