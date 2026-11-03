# 首页重构: 取舍说明与改动清单

日期: 2026-11-01. 接着 2026-10-31 那一轮未提交的重构现场继续, 没有推倒重来, 也没有 `git checkout` 整片文件.

## 0. "那个图"回档: 还原哪一份, 为什么

读盘确认的事实:

- 提交 `926b31ba76` ([feat] 首页改成可编辑的项目生态图, #150), 父提交 `e126dc22`. 它改了 7 个代码/数据文件加 59 张封面图: 新增 `data/ecosystem.ts`、`src/components/home/{EcosystemGraph,HomePage,ProjectDetail}.tsx`、`home.module.css` 和 note `2026-10-31-homepage-ecosystem-graph.md`, 并把 `src/pages/index.tsx` 从 305 行的徽章墙首页改成只渲染整页生态图.
- 这次提交**没有动** `WordCountChart`、`ProjectCarousel`、`data/projectData.ts`; 那三处的改动全部来自上一轮未提交的现场.

还原的是"首页布局", 不是整个提交:

- 撤掉: 926b31ba76 让生态图占满首页的布局, 以及它的 note `homepage-ecosystem-graph.md` (已删除, 全仓库无残留引用; 决策并入 `2026-10-31-homepage-profile-and-project-index.md` 的"回档"一段).
- 保留: `EcosystemGraph.tsx`、`ProjectDetail.tsx`、`data/ecosystem.ts`、`static/img/ecosystem/` 与 926b31ba76 逐字节一致. 需求 3 要求关系图留着, 只是挪进"更多".
- 不回到 `e126dc22` 的徽章墙首页: 那版依赖 vercel 上的第三方统计卡, 没有关系图, 需求 4 又要求重做自我介绍主页, 回过去等于再拆一次.

## 1. 历史字数图: ai 曲线 + 纵轴

问题根因: 两根纵轴的 0 不在同一高度. 左轴 (增量) 被删字提交拉到负数, 右轴 (累计) 从 0 起, 于是累计曲线看起来是从左轴的负数出发的. 上一轮把两根轴都钉在 `[0, 上界]`, 0 是对齐了, 但删字提交被整个压没, 看着和零增量一样.

本轮做法: 两根轴共用同一套分段 (正向 4 段, 负向段数相同), 0 落在同一条水平线, 网格线一一对齐, 加一条共同的 0 虚线. 负区最多 2 段 (正向高度的一半), 更深的删字由 `allowDataOverflow` 压底, tooltip 给真值. 右轴负区只占位不标刻度. ai-docs 曲线与 docs/blog、总量共轴, 相对高度即真实比例.

- `src/components/chart/WordCountChart/index.tsx` 第 53-83 行: `alignedAxes` (替换掉 `niceAxis`)
- 同文件第 107-120 行: 按刷选区间同时算两根轴
- 同文件第 206-226 行: 两根 `YAxis` 用对齐后的 domain/ticks
- 同文件第 255-256 行: 共同 0 线 `ReferenceLine`
- note: `.agents/notes/implemented/bug-fix/2026-10-31-word-count-chart-zero-baseline.md` 已同步改写 Decision / Alternatives / Consequences

## 2. 项目滚动条: 留着, 换成近一年在做的

- `data/projectData.ts` 第 10 行 `PROJECT_SINCE = "2025-10-31"`; 第 50 行起是项目列表 (上一轮按 gh 推送日期重写).
- 本轮删掉 HX-Music: 最后推送 2025-10-02, 早于近一年起点 (与 `data/ecosystem.ts` 的 `pushedAt` 一致). 它仍在关系图里.
- `src/components/ProjectCarousel/index.tsx` 第 25-31 行: 渲染时按 `PROJECT_SINCE` 过滤并按 `pushedAt` 倒序. 之前注释说"按 pushedAt 排序"但代码其实按数据文件顺序走.
- 同文件第 16-18 行: 顶部注释改成和实际一致 (入口是条带下方横条, 不是条带里的卡).

## 3. 项目关系图: 收进滚动条的"更多"

上一轮已实现, 本轮核对无误:

- `src/components/ProjectCarousel/index.tsx` 第 205 行: 条带下方常驻"更多 · 项目关系图"横条.
- `src/components/home/HomePage.tsx` 第 121 行把开关交给轮播, 第 124-134 行点开才渲染 `EcosystemGraph`, 默认不出现在首页.
- 取舍: 入口不做成条带里的卡片, 条带是无限循环滚动的, 卡片会一直转走.

## 4. 自我介绍主页

上一轮已按新结构重写, 不沿用旧首页的写法. 顺序: 自我介绍 + 字数图 → 最近笔记 → WakaTime → 近一年项目 (+ 更多: 关系图) → 关于 (技术栈 + 联系方式).

- `src/components/home/HomePage.tsx` 第 41-82 行: 首屏, 头像/名字/tagline, 第 52 行三段自我介绍, 第 56 行四个数字卡, 第 77 行字数图.
- 同文件第 111 行起: 近一年项目段; 第 147 行起: 技术栈分组与联系方式.
- `data/profile.ts`: 第 26 行 `PROFILE_AS_OF`, 第 28 行数字卡, 第 35 行技术栈, 第 54 行链接, 第 62 行自我介绍文案. 改文案和数字只改这一个文件.
- `src/pages/index.tsx` 只做入口.

## 未做 / 待你核对

- 未本地构建 (仓库红线: 不主动启动博客程序). 请自行 `npm start` 看一眼首屏和字数图的负区.
- 未跑 `uv run scripts/redlines/agent_notes.py --diff`, 需要在本机跑.
- `home.module.css`、`ProjectCarousel/css.css` 是非 AST 资源, 按红线需人工核对.
- 字数图首条记录 (2025-04-28 迁移, +341 万) 仍在数据里, 默认刷选从 20% 处开始所以看不到; 拖到最左时左轴会被它撑大, 属预期.
