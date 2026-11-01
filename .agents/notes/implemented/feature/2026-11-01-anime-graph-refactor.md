# Agent Note: 番剧关系图重构 (续作改用 relations、物理参数首屏生效、三态与站点 Layout、视觉统一、概览层 / 聚焦层)

Status: implemented

Decision-ID: anime-graph-refactor

## Code

- `src/components/anime/AnimeGraphPage/index.tsx`
- `src/pages/anime/graph.tsx`

## Problem

关系图页 (`/anime/graph`) 的两个功能性 bug, 本轮先修 (调研见 `.agents/research/anime-graph-redesign.md`):

Bug A, 物理参数首屏不生效: 设置斥力与连线距离的 `useEffect` 依赖 `fgRef.current`. ref 赋值不触发渲染, 图在 `BrowserOnly` 里晚于首次 effect 挂载, effect 在 ref 为空时跑完一次后就不再跑, 所以 URL 里的 `charge` / `dist` (以及路由默认的 -180 / 100) 首屏不生效, 要拖一下滑杆才起作用, 期间用的是库默认力.

Bug B, 续作判定靠名字前缀猜: 聚焦时 BFS 用 `name.length > 4` 且前 4 字符前缀, 连边时用 `> 3`, 两处不一致; 只做单向 `startsWith`; 连边是全部番两两比较, 一个系列被连成完全图, 只共享 "Fate/" 这类品牌前缀的作品也被误连 (如 Fate/kaleid liner 与 Fate/Grand Order). 数据里每部番都有 `anime_data.relations` (bangumi 关联主题 id 与关系类型), 却没用. 对 160 部的实测: 前缀规则产生 157 对, 其中 94 对 relations 里不存在 (多为同系列非相邻季被两两连上, 以及品牌前缀误连), 另有 12 对 relations 里存在而前缀规则漏掉.

## Decision

### 第 1 步: 两个 bug

Bug A: 用稳定的回调 ref (`attachGraph`) 接住图实例, 同时写入 `fgRef` 与 state `fgInstance`; 物理参数 effect 依赖 `fgInstance`, 图挂载即执行一次, 之后随滑杆变化执行.

Bug B: 新增纯函数 `buildSeriesIndex(records)` / `collectSeries(id, adjacency)` (同目录 `buildGraph.ts`), 聚焦 BFS 与续作连边共用同一份系列边:

1. 主来源: `relations` 中关系类型属于 前传 / 续集 / 番外篇 / 总集篇 / 主线故事, 且对方 id 也在库里; 按无序 id 对去重. 实测得 59 条系列边.
2. 兜底条件 (三条同时满足才补边): 两部番的原名或中文名中, 一方整体是另一方的前缀, 且两边都至少 4 个字符; 两部番此前不在同一个系列连通分量里 (并查集); 按首播日期升序遍历, 只补一条. 用途是中间一季不在库里导致 relations 链断开的情况. 实测补 5 条: 请问您今天要来点兔子吗 与 BLOOM、关于我转生变成史莱姆这档事 与 第三季、盾之勇者成名录 与 第四季、间谍过家家 与 第二季、工作细胞 与 工作细胞BLACK (最后一条是衍生而非续作, 但同属一个系列, 可接受).
3. 不同世界观 / 相同世界观 / 衍生 / 角色出演 等关系不算续作, 不进系列边 (留给后续概览层作为 "相关" 边).

续作边带上 relations 的原始关系文本 (`label`), 兜底边没有.

### 第 2 步: 加载 / 错误 / 空态 + 套站点 Layout

路由 `src/pages/anime/graph.tsx` 套 `@theme/Layout` (noFooter), 不再藏导航栏; 画布区高 `100vh - --ifm-navbar-height`. 数据改用 `useAnimeRecordsState`, 按状态切换覆盖层 (同目录 `GraphStatus.tsx` + `graph.module.css`):

- 加载: 阶段文案 "正在下载番剧数据" + 数据量说明 + 不定进度条 + 每 0.5s 刷新的已用时, 超过 15s 加一句慢网提示. `animeStore` 不暴露字节进度 (`reqAnimeByCDN` 支持 onProgress 但 store 没传), 本轮不改 `src/utils/anime/`, 所以不显示百分比.
- 错误: 错误信息 + 重试 (调用 store 的 `retry`, store 失败后会清掉在途 promise, 重试真的重新请求).
- 空: 库为空; 以及链接里的 `id` 不在库中, 给 "返回全部番剧".

### 第 3 步: 视觉统一

`index.tsx` 的 `THEME` 改为站点暗色底 `#121212` + 低饱和三色 (番剧米白 `#d9d4c7`、角色灰青 `#7fa6a1`、声优灰金 `#b9a37a`), 强调色只用站点主色 `#ff88ff` 做悬停 / 选中描边. 番剧画圆角矩形 (按封面比例), 角色与声优画圆形, 形状本身区分层级. 头像只在缩放 >= 1.2 时绘制. 标签: 番剧常显, 角色 / 声优在缩放 >= 1.6 或被高亮时显示; 标签背景用独立的半透明深底, 文字 `#ece9e2`, 修掉原来文字背景沿用节点底色、字是 `#990099` 导致看不清的问题. 变暗统一用 `globalAlpha` 0.18. 边按类型区分颜色 / 线宽 / 线型 (`EDGE_STYLE`, 通过 `linkLineDash`): 续作实线 2px, 共享声优虚线 (按人数加粗), 番剧—角色细实线, 角色—声优虚线, 声优的其它作品点线. 左下图例 (`GraphLegend`) 与画布共用同一份常量, 按当前层只列出该层出现的节点与边. 面板、菜单、顶栏样式都在 `graph.module.css`, 不再用内联样式.

### 第 4 步: 两层视图 (概览层 / 聚焦层)

- 概览层 (默认, URL 无 `id`): `buildOverviewGraph` 只画番剧节点; 边 = 续作 / 同系列 + 共享声优. 共享声优按 "声优 -> 出演番剧" 倒排表两两累计人数, 只统计未被角色定位过滤掉的角色, 共享人数 >= 阈值 (`cv`, 默认 2, 面板可调 1 到 6) 才连, 线宽随人数加粗, 悬停边显示 "共享 N 位声优". 实测 160 部、默认只看主角: 160 个节点, 64 条续作边 + 155 条共享声优边.
- 聚焦层 (URL `id=<番剧>`): `buildFocusGraph` 以该番剧所在系列 (`collectSeries`) 为中心, 展开角色、声优, 再挂上这些声优在库里的其它作品 (淡色 ghost 节点, 点线). 例: 间谍过家家 默认只看主角时 3 部 + 4 角色 + 13 声优 + 13 部其它作品.
- 进入: 概览层双击番剧、右键 "展开角色与声优"、或顶栏搜索选中 (搜番剧原名 / 中文名与角色名, 命中角色时进入其所在番剧; "/" 聚焦, 上下键 + 回车). 聚焦层双击 ghost 作品切换中心. 退出: 面包屑 "全部番剧"、Esc、浏览器后退.
- 状态全部放 URL (`useGraphUrlState.ts`), 只写与默认值不同的参数; 进出聚焦层用 push, 其余用 replace. 旧链接 (`?id=&hide=&cn=&img=&charge=&dist=`) 全部兼容, 番剧详情页的 "关系图" 链接 `graph?id=` 直接进入聚焦层.
- 交互调整: 单击 = 选中并高亮一跳邻居 (原来两跳), 不再强制放大; 去掉中键锁定; 换层后布局稳定时自动 `zoomToFit`; 左下加 + / − / 适应全部; 角色定位过滤改为 "勾选 = 显示" 的 chip; 物理参数收进折叠区.
- 性能: `onRenderFramePre` 由画布变换反推可见范围, `nodeCanvasObject` 跳过屏外节点; 重建图数据时沿用旧节点坐标, 切换过滤不整体重排; `d3VelocityDecay` 0.1 -> 0.35, 收敛更快不再长时间抖动; 续作连边从 O(n^2) 名字比对改为 id 索引.
- 文件: `buildGraph.ts` (纯数据), `useGraphUrlState.ts`, `SearchBox.tsx`, `GraphStatus.tsx`, `graph.module.css`; `react-force-graph-2d` 的 type import 只在 `index.tsx` 保留一行, 不在其它文件引入, 避免新增 TS1479.

## Alternatives considered

- 什么都不做: 最强理由是现在图能画出来, 改动有风险. 否决: Bug A 让 URL 分享的物理参数失效, Bug B 让聚焦结果和续作边都不准, 是关系图的核心信息错误.
- 复用现有名字前缀规则, 只把两处阈值统一并改成双向: 改动最小. 否决: 统一阈值后仍然会把整个系列连成完全图, 仍会因品牌前缀误连, 且有现成的 relations 不用.
- 只用 relations, 不做兜底: 最干净. 否决: 库里漏看中间一季时系列会断成两截, 实测 5 处.
- Bug A 改用 `onEngineTick` 首帧设置力: 不新增 state. 否决: 每帧回调里加判断, 且滑杆变化仍需要另一条路径; 回调 ref 一处解决.

- 加载进度显示真实百分比: 需要给 `animeStore` 的 `loadAnimeRecords` 透传 `onProgress`. 否决 (本轮): 超出约定范围 (只动 AnimeGraphPage 与路由), 且 store 被番剧库与详情页共用; 若要百分比, 单独一轮改 store.

- 概览层仍画全部角色与声优 (旧行为), 只靠 LOD 与裁剪提速: 不改信息架构. 否决: 几千节点即便画得动也读不出关系, 两层视图是已确认的方案.
- 搜索支持拼音首字母 (`pinyin-pro` 已在依赖里): 本轮未做, 中文名与原名包含匹配已覆盖主要用法, 拼音需要为 5000 个角色名预计算, 留作后续.

## Consequences

聚焦一部番时载入的是 relations 定义的系列 (加兜底), 不再顺带载入同品牌前缀的无关作品, 续作边数量从两两相连降为相邻相连. relations 依赖 bangumi 数据, 新番入库后若 bangumi 尚未建立前传/续集关系, 由名字兜底覆盖, 兜底不覆盖改名较大的续作 (如副标题完全不同). `buildGraph.ts` 是 `AnimeGraphPage` 目录内的兄弟文件, 由本 note 经 `index.tsx` 代表. `AnimeGraphController` 的 props 变了 (去掉 `targetAnimeId` / `allRecords`, 加 `layer` / `minShared`), 仓库内只有 `src/pages/anime/graph.tsx` 使用它; `src/pages/anime/index.tsx` 与 `details.tsx` 只是链接到 `/anime/graph`, 未改动. 加载进度只有不定进度条与已用时, 没有百分比. `index.tsx` 的 `react-force-graph-2d` type import 保持不动, 基线 `src/components/anime/AnimeGraphPage/index.tsx|TS1479` 不变.
