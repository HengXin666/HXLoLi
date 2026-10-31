# Agent Note: 首页用一张可编辑的项目生态图, 不再堆徽章

Status: implemented

Decision-ID: homepage-ecosystem-graph

## Code

- `src/pages/index.tsx`
- `src/components/home/HomePage.tsx`

## Problem

首页把 GitHub 徽章、统计图和项目轮播叠在一张深色底上. 看不出仓库之间谁依赖谁, 换一个项目也没有单独的数据入口. 主题的浅色模式在这张首页上基本用不上.

## Decision

首页改成产品发布页的结构: 标题、生态图、痛点如何变成轮子、代表项目、最近文章、笔记、字数和关于. 生态图自下而上分五层, 边按依赖、子模块、派生、功能相关区分, 推断边用更淡的虚线. 点节点打开详情, 里面是封面、中文简介、技术栈和仓库链接.

项目、图层和连线只写在 `data/ecosystem.ts`. 封面路径写在每条记录的 `image` 字段, 文件放在 `static/img/`. 连线必须能指出仓库里的文件. 推出来的关系把 `inferred` 设为 true. 私有子模块不进入这份数据.

同目录的演示页滚轮修复不因这次首页改动而改变.

## Alternatives considered

- 什么都不做 / 复用现有轮播和 `data/projectData.ts`: 项目卡片已经能点进 GitHub, 改动最小. 否决是因为那份数据没有图层和连线, 首页也仍然是徽章墙, 回答不了仓库之间的关系.
- 复用站内的 `react-force-graph`: 番剧页已经用它画过力导向图, 交互是现成的. 否决是因为力导向停不稳五层结构, 刷新后位置会变, 首页还要多付一份模拟的开销.
- 把每个项目写成一篇 Markdown: 和笔记的写法一致, 图可以事后生成. 否决是因为加一个仓库要新建一篇文档, 不符合单独一个数据文件就能改的要求.

## Consequences

加项目或改连线只改 `data/ecosystem.ts` 和对应图片. 推断边在图上和详情里都会标出来. 首页样式表是非 AST 资源, `src/pages` 里原有的演示页决策也不会被这次改动更新, 这两处需要人工看过诊断.
