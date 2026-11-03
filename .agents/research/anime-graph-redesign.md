# 番剧关系图重构调研

> 范围: `src/components/anime/AnimeGraphPage/index.tsx` (717 行, 唯一文件), 路由 `src/pages/anime/graph.tsx`, 数据 `src/utils/anime/animeStore.ts` / `types.ts`。
> 参考但不改: `src/components/home/EcosystemGraph.tsx` + `data/ecosystem.ts`。本文件只是调研, 未改任何代码。

## 1. 现有功能 (从代码读出)

### 路由 `src/pages/anime/graph.tsx`
1. 全屏 `100vh` 黑底容器 (`THEME.background`), 未套 `@theme/Layout`, 页面没有站点导航栏。
2. URL 与状态双向同步: `id` / `hide` / `cn` / `img` / `charge` / `dist`, 每次变化用 `history.replace` 写入, 且所有参数总是写入 (含默认值)。
3. 默认隐藏角色定位: `["配角", "客串", "闲角", "旁白"]`。
4. 默认 `preferCn=true`, `showImages=true`, `chargeStrength=-180`, `linkDistance=100`。
5. 数据用 `useAnimeRecords` (只有数组), 不是 `useAnimeRecordsState`, 所以没有加载态和错误态。

### 控制面板 `AnimeGraphController` (右下浮层, 320px)
6. 聚焦番剧: 原生 `<select>` 列出全部番剧 + "全部番剧", 不可搜索; 选中后提示 "已自动加载前作/续作关系网"。
7. 角色定位过滤: 主角/配角/客串/闲角/旁白 五个 checkbox, 勾选 = 隐藏 (语义反向)。
8. 中文优先开关。
9. 显示头像开关。
10. 斥力滑杆 (-500 ~ -10, 步长 10)。
11. 连线距离滑杆 (10 ~ 200, 步长 5)。
12. 面板可折叠, 左侧 ⚙️ / » 按钮, `translateX` 动画。

### 图 `AnimeForceGraph` (react-force-graph-2d, `BrowserOnly` + `require` 客户端加载)
13. 三类节点: Anime (val 80) / Character (val 20) / Actor (val 12); 节点是方块, 有图时按图片宽高比拉高。
14. 三类边: Anime→Character (`Include`), Character→Actor (`VoicedBy`), Anime↔Anime (`Sequel`); 类型里声明的 `Related` 未使用。
15. 续作判定: 名字 (`anime_data.name`) 前 4 字符前缀匹配。聚焦时先用该规则 BFS 扩展出整个系列 (阈值 `length > 4`), 连边时阈值是 `length > 3`, 两处不一致; 只做单向 `startsWith`; 连边是 O(n²) 双重循环。
16. 角色与声优节点按 id 去重, 因此跨作品共享的声优会把多部番连在一起。
17. 头像 URL 走 `toAnimeCdnUrl` (`/data/anime/{id}.jpg`, `/data/kyara/{id}.jpg`, `/data/cv/{id}.jpg`), 全局 `imgCache` 缓存 `Image`。
18. 悬停高亮: 以悬停或锁定节点为中心, 高亮两跳邻居 (`neighborMap` 邻接表), 其余节点变暗 (`#222`, alpha 0.2), 边变为 `rgba(255,255,255,0.02)`。
19. 锁定: 中键点节点切换锁定, 中键点空白取消; 左键点空白也取消锁定; 拦截中键默认的自动滚动。
20. 左键点节点: `centerAt` 居中 + 强制 `zoom(3)`。
21. 右键菜单: 显示类型和名称; "聚焦" (居中 + zoom 4); 仅 Anime 节点有 "跳转番剧详情" (新标签打开 `anime/details?id=`); "关闭"。
22. 自定义 canvas 绘制: 背景方块 → 头像 (clip) → 高亮紫色描边 `#990099` → 底部文字标签 (字号 `12/globalScale`, 下限 4)。
23. ResizeObserver 跟随容器尺寸。
24. 物理参数: `d3VelocityDecay=0.1`, `cooldownTicks=100`; 斥力/距离通过 `useEffect` 设置 `d3Force` 并 reheat。

## 2. 最该改的三个问题

### 问题一: 默认全量铺图, 信息过载且卡
- 现状: 不聚焦时把所有番剧、所有未被隐藏的角色、以及他们的声优一次性放进同一张力导向图; 每帧对每个节点画方块 + 图片 + 文字背景 + 文字, 无视口裁剪、无按缩放级别降级; `hoverNode` / `highlightNodes` 是 `nodeCanvasObject` 的依赖, 每次悬停都重建回调并触发 React 重渲染; `d3VelocityDecay=0.1` 让模拟长时间抖动; 两跳高亮在全量图上常点亮大半张图。
- 为什么是问题: 记录数据有约 12 MB, 节点会到数千, 拖拽和悬停掉帧; 打开页面看到的是一团毛线球, 读不出任何关系, 两跳高亮也失去 "谁和谁相关" 的意义。

### 问题二: 交互难发现、难用, 且有两个 bug
- 现状: 锁定靠中键 (无任何提示); 左键点节点强制放大到 3 倍; 过滤项勾选 = 隐藏; 几百部番塞在不可搜索的 `<select>`; 选中节点后除了右键菜单没有任何信息展示, Character / Actor 节点无法跳转 (仓库已有 `src/pages/anime/cv.tsx`)。
- Bug A: 物理参数 `useEffect` 依赖 `fgRef.current`, 图挂载时不会触发, URL 里的 `charge` / `dist` 首屏不生效, 要拖一下滑杆才起作用。
- Bug B: 续作判定阈值两处不一致 (`>4` vs `>3`), 只单向前缀匹配, 而数据里已有 `anime_data.relations` (含关联主题 id 和关系类型) 却完全没用, 容易误连/漏连。
- 为什么是问题: 用户找不到想看的番、看不到节点详情, 点一下就丢失视野; 关系本身还不准, 图的核心价值被削弱。

### 问题三: 视觉与站点基调不统一, 层次不清, 缺状态
- 现状: 红 `#ff6b6b` / 青 `#4ecdc4` / 黄 `#ffe66d` 三种高饱和主色, 标签文字是紫色 `#990099`, 且文字背景沿用节点底色 (fillStyle 未重置), 多数情况下文字看不清; 边一律白色细线不分类型; 没有图例; 没有加载态、错误态、空态; 页面不在站点 Layout 内。
- 为什么是问题: 与站点深色克制的风格冲突, 用户无法从颜色和线型分辨节点/关系类型; 冷启动拉 12 MB 数据时是一片空黑屏, 失败时也没有重试入口。

## 3. 重构方案

### 布局与信息架构
- 页面套站点 `Layout` (noFooter), 画布填满导航栏以下区域。
- 两层视图:
  - 概览层 (默认): 只画番剧节点; 边为续作关系 + "共享声优" (按共享人数加权, 可设阈值)。节点数 = 番剧数, 可读且流畅。
  - 聚焦层 (双击或搜索选中进入): 以一部番 / 一个系列为中心, 展开角色、声优, 以及这些声优在库里的其它作品 (淡色占位, 点击切换中心)。顶部面包屑可退回概览。
- 左上: 搜索框 + 可折叠筛选区 (角色定位改为 "显示哪些" chip, 声优边阈值, 物理参数收进 "高级")。
- 右侧: 详情抽屉, 与选中节点联动: 番剧 = 封面/评分/主要角色; 角色 = 简介/CV; 声优 = 在库作品。抽屉内条目点击在图上定位, 并带跳 `details` / `cv` 页链接。
- 左下: 图例 + 缩放控件 (+ / - / 适应全部)。
- 状态: 加载中骨架 + 提示 (改用 `useAnimeRecordsState`), 失败显示错误 + 重试, 搜索无结果 / 筛选后为空给空态。
- 续作改用 `relations` 按 id 匹配, 名字前缀仅在无 relations 时兜底 (动手前先抽查数据中实际的关系文本)。

### 交互
- 单击: 选中, 高亮一跳邻居, 打开抽屉, 不强制缩放。
- 双击: 进入聚焦层。
- 悬停: 轻量高亮 + 小 tooltip。
- 拖拽平移、滚轮缩放保留; 拖动节点后固定在原位, 双击解除。
- 搜索: 同时搜番剧 / 角色 / 声优, 支持中文、原名、拼音首字母 (复用已有依赖 `pinyin-pro`), 键盘上下选择、回车定位。
- 键盘: `/` 聚焦搜索, `Esc` 取消选中或返回上一层, `F` 适应视图。
- 去掉中键锁定; 右键菜单精简或并入抽屉。
- URL 只写非默认值; 层级与选中写入 URL, 浏览器前进后退可在层级间切换。修复物理参数首屏不生效 (在图 ref 就绪回调里应用 force)。

### 视觉
- 低饱和配色: 番剧为中性浅色 + 封面缩略图, 角色 / 声优用两种暗调强调色区分; 节点改圆角/圆形, 大小体现度数或重要性。
- 边按类型区分: 续作实线、包含细线、配音虚线、共享声优按权重调透明度与粗细。
- 变暗统一用透明度, 不换色; 高亮描边用站点强调色而非 `#990099`。
- 标签仅在缩放足够大或节点被高亮时出现, 统一半透明深底 + 浅色字, 番剧标签优先级最高。
- 样式抽到组件目录的 CSS Module, 与站点深色变量对齐 (参考 `EcosystemGraph` 的克制风格, 不改它)。

### 性能
- 不新增依赖, 沿用 `react-force-graph-2d`。
- 分层本身就是最大优化: 默认只有番剧级节点。
- LOD: 缩放很小时只画圆点; 中等显示标签; 足够大才画头像。
- 视口裁剪: 在 `nodeCanvasObject` 里用当前变换判断节点是否在屏幕内, 屏外直接跳过绘制。
- 悬停/高亮状态放 ref, 配合 `autoPauseRedraw`, 避免每次悬停 React 重渲染; 颜色/宽度回调稳定化。
- `d3VelocityDecay` 调到约 0.4, `cooldownTicks` 按节点数自适应, 稳定后自动 `zoomToFit`。
- 构图改为按 id 建索引, 去掉 O(n²) 名字比对; 共享声优边用 actor → animes 倒排表计算。

### 文件拆分 (仅 AnimeGraphPage 目录 + graph.tsx 路由)
- `index.tsx`: 对外导出; 保留 `react-force-graph-2d` 的 type import, 使基线 `src/components/anime/AnimeGraphPage/index.tsx|TS1479` 不变成新增。
- `buildGraph.ts` (纯函数, 概览/聚焦两层), `useGraphUrlState.ts`, `GraphCanvas.tsx`, `SearchBox.tsx`, `DetailPanel.tsx`, `graph.module.css`。
- `src/pages/anime/graph.tsx` 改为薄壳。
- 配 note: `.agents/notes/implemented/feature/YYYY-MM-DD-anime-graph-refactor.md`, Code 段两行: `src/components/anime/AnimeGraphPage/index.tsx`、`src/pages/anime/graph.tsx`。
- 验证: `npx tsc --noEmit` 对比基线零新增, `uv run scripts/redlines/agent_notes.py --diff`; 不运行 `npm start` / `npm run build`。

### 待确认
- 默认进入概览层 (只有番剧) 是否接受, 还是保留现在的全量图为默认。
- 是否做 "共享声优" 边 (这是新增信息, 不是单纯重排)。
