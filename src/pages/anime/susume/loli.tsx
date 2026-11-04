import React from "react";
import SusumePage from "@site/src/components/anime/SusumePage";

/**
 * /anime/susume/loli —— 推荐分类「ロリ」(萝莉为卖点) 专属页. 独立路由, 不是 /anime?tag=ロリ 的筛选结果;
 * 旧式 /anime?tag=ロリ 由番剧首页客户端重定向到这里 (src/pages/anime/index.tsx).
 * 只读人工标注 HXLoLi-ANiMe/data/labels/<id>.json, 按本人评分降序, 附一句话观后感. 页面本体见 components/anime/SusumePage.tsx.
 * hx-anime-tag-refactor-design.md 第 3.1 节
 */
export default function Susume_loli (): React.ReactElement {
    return <SusumePage susume="loli" />;
}
