# Agent Note: kyara 角色配图页 UI (/anime/kyara, /anime/kyara/{name}), 与 susume 同款

Status: implemented

Decision-ID: anime-kyara-pages

## Code

- `src/components/anime/kyara/KyaraPage.tsx`

## Problem

1. 番剧区需要一套按角色给图片配图的页面: 用户把图放进 `static/img/kyara/{name}/001-标题@作者.png` 就按序号顺序渲染, 也能在浏览器本地攒图, 并从 pixiv / x.com 链接导入.
2. 新页面必须和重构后的 susume 同一套视觉与动效语言 (HX おすすめ Glass: 粉色主调、猫娘、日系圆体、bento、framer-motion 入场), 不能另起一套.
3. 数据层 (扫描、路由、解析器、IndexedDB、dev 写入通道) 由另一路 (`src/utils/kyara/`, `plugins/kyara-gallery-plugin.mjs`) 负责, 原子组件由 `src/components/anime/ui/` 负责; 本决策只做页面, 两边都只按冻结契约引用.

## Decision

1. 目录 `src/components/anime/kyara/` 六个文件, 同目录兄弟均是本决策的代码端, 由 `KyaraPage.tsx` 的默认导出代表:
   - `KyaraPage.tsx`: 路由组件, 收插件注入的 `kyara: KyaraData`; 主体抽成 `KyaraView` 供索引页的「新角色」视图复用. 结构 = 面包屑 → `HxHero` (kicker `キャラ · {work}`, 计数「N 张」, 光晕取 `--kyara-accent`, 背后一层首图大幅模糊铺底, 右上本地模式开关) → `HxPills` 全部 / 仓库 / 本地 → CSS columns 瀑布流 (按 `w/h` 预留 `aspect-ratio` 防跳动, 左上序号章 `001`, 悬浮浮出 `title · @作者` 与来源图标) → `Lightbox`. 本地条目排在仓库条目之后.
   - `Lightbox.tsx`: `role="dialog"` + `aria-modal`, 与缩略图共享 `layoutId` (`kyara-{source}:{file}`) 放大; ←/→ 翻页, Esc 关闭, 触屏左右拖动翻页; 底部胶片条; 来源链接与原图链接; 打开时锁滚动、焦点困在框内、关闭还焦点 (`useDialogFocus`, 导入框复用). 同文件导出 `SourceIcon` / `seqLabel` / `ratioOf` / `keyOf` / `cssVars` 小工具, 避免页面与灯箱循环引用.
   - `LocalPanel.tsx`: 本地模式面板 (`HxCard`): 拖拽区 / 选择文件 / 全局粘贴 (图片直接入库, 粘贴链接则打开导入框预填), framer `Reorder` 拖拽排序 (松手才 `store.reorder` 落库), 导出 zip, dev 下「写进仓库」= 逐张 `PUT /__hx/kyara/{name}/file/{NNN-标题@作者.ext}` (序号接在仓库最大序号之后, 文件名用 `formatKyaraFilename`), 成功后从本地移除.
   - `ImportDialog.tsx`: `role="dialog"`, 多行链接 → 用 `kyaraResolvers` 逐条匹配 (pixiv / x / direct) → 预览 (标题可改) → 确认. pixiv 在浏览器里会被 Referer / CORS 挡住, dev 下交给 `POST /__hx/kyara/{name}/import`, 线上提示去本机 dev; x / 直链浏览器拿得到就进 IndexedDB, 拿不到且在 dev 下回落服务端. dev 下可选「写进仓库」. 同文件导出 `kyaraDev` (带 `X-HX-Kyara` 头的 dev 请求).
   - `KyaraIndexPage.tsx`: `/anime/kyara` 角色 bento 网格 (封面 + 名字 + 作品 + 张数, 首个角色 2×2), 数据优先用插件注入的 `index`, 否则运行时拉 `/kyara/index.json`; 骨架 / 错误重试 / 空态 (猫娘 + 命名说明) 三态. 本地模式开关 (localStorage `hx-kyara-local-index`) 打开后末尾出现「＋ 新角色」卡, 提交到 `?new={name}` 渲染纯本地 `KyaraView` (仓库里还没有目录, 没有静态路由).
   - `Kyara.module.css`: 颜色只用 `--hx-*` 令牌与 `--kyara-accent`, 派生一律 `color-mix`, 不写 hex / rgba 字面量; 断点 ≤996 / ≤640 (触屏标题常驻); `prefers-reduced-motion: reduce` 全部瞬时.
2. 类型只从 `@site/src/utils/kyara/types` 引入, 函数从 `filename` / `resolvers` / `localStore` 引入, 不在本目录重定义契约.

## Alternatives considered

- 什么都不做: 最强理由是零风险, 不增加页面. 否决: 用户明确要求 kyara 配图页, 且要与 susume 同款.
- 复用现有 `AnimeShareBoard` 或番剧库海报网格渲染配图: 现成、风格接近. 否决: 海报网格是固定 2:3 封面比例, 插画横竖混排会被裁掉; 也没有序号、作者、来源与灯箱.
- 用 JS 计算的 masonry (按列高分配): 顺序更接近"按行". 否决: CSS columns 零脚本、SSR 稳定, 且按 w/h 预留比例后不跳动; 顺序按列向下读, 序号章明确标出顺序.
- 灯箱用第三方库 (yet-another-react-lightbox 等): 功能齐. 否决: 新增依赖, 且无法与缩略图共享 framer `layoutId`, 视觉也要再覆盖一遍.

## Consequences

依赖 `src/components/anime/ui/` (路 0) 与 `src/utils/kyara/` (路 3) 按契约落地; 在它们落盘之前, 本目录的 tsc 会报找不到模块. 「写进仓库」要求插件的 PUT 通道在角色目录不存在时自行创建目录. 页面入口 (`/anime` 到 `/anime/kyara` 的链接) 由收口路添加. 页面效果由用户在 `localhost:3000/HXLoLi/anime/kyara` 与 `/anime/kyara/<示例>` 上以 1440 / 1024 / 390 三档宽度、亮暗各看一次.
