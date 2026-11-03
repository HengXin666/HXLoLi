# 首页重设计: 设计说明

日期: 2026-11-01. 接 `.agents/research/home-refactor.md` (原 `docs/home-refactor.md`) 的两轮修改. 本文件回答: 参考了谁、首屏加载了什么、颜色字号间距的数值、每块可视化的图表选择、每处重叠的定位与改法, 以及我验证了什么、没验证什么.

> 位置: 本文件与 `home-refactor.md` 原先放在 `docs/`, 而 `docs/` 是 docs 插件的 contentPath (`include: ['**/*.{md,mdx}']`), 两份内部说明因此被发布成 `/docs/home-refactor`、`/docs/home-redesign` 页面并进了首页"最近笔记". 2026-11-01 第二轮把两份挪到 `.agents/research/` (docs 插件、docs-rss-plugin、侧边栏生成器都只扫 `docs/`), 并从 `sidebars.ts`、`data/latestDocs.ts`、`data/noteReferences.json` 里删掉对应条目. 远程写入通道只能读写、不能删文件, 所以 `docs/home-refactor.md`、`docs/home-redesign.md` 暂时留成只有 `draft: true` front matter 加一行迁移说明的占位文件: 生产构建不收 draft, docs-rss-plugin 跳过 draft (`fm.draft` 判断), 不会进"最近笔记". 这两个占位需要手动 `rm`, 删掉后 `docs/` 下就没有它们了. 没用 `_` 前缀或 `draft: true`: 前者仍留在内容目录里, 靠约定排除; 后者 dev 下照样渲染.

## 0. 怎么看到渲染的

仓库红线禁止启动博客, 所以我没有在你机器上 `npm start` / `npm run build`. 我在自己沙箱里搭了最小渲染环境: 把首页相关源码 (HomePage、ProjectCarousel、两个图表、关系图、`data/*.ts`、`src/css/custom.css`) 拉下来, 用 esbuild 打包, Infima 0.2.0-alpha.45 作为主题底样式, `data-theme="dark"`, `@docusaurus/*` 和 `@theme/Layout` 用 5 行的桩替代, 再用 Playwright Chromium 在 1440x900、1280x720、390x844 三个视口截图. WakaTime 的数据源 raw.githubusercontent.com 在沙箱里不通, 截图时用 400 天的合成数据替代 (只用来看布局, 数值不真).

## 1. 参考站点与参考了什么

四家都是用沙箱 Chromium 实际打开测的 (1440 宽, 计页面 load 后 2.5 秒内完成的请求体字节).

| 站点 | 实测 | 参考了什么 |
|---|---|---|
| brittanychiang.com | 40 请求 / 2,909 KB (其中字体 1,159 KB); 背景 #0f172a, 正文 #94a3b8, h1 48px/700, 右栏正文宽 607px | 桌面端左栏粘性个人卡 + 右栏内容流; 左栏分节导航, 当前节的短线拉长点亮. 反面教材: 1.1 MB 字体, 我们不加网络字体. |
| antfu.me | 55 请求 / 903 KB (字体 47 KB); h1 36px/800, 正文 16px/24px | 自我介绍用一行项目胶囊 ("Creator of [Vitest] [Slidev] ...") 代替长段落 -> 个人卡里的"在做"胶囊行; 页面底的点阵纹理 -> 用 CSS 径向渐变做, 0 字节. |
| leerob.com | 92 请求 / 554 KB, 首页只有 1 张图; h1 42.4px/600, 正文 17px/27.2px | 插画固定尺寸放侧边, 不铺满; 文章列表一行一条、日期右对齐、细分隔线 -> "最近笔记". |
| overreacted.io | 25 请求, 文档本身 11 KB (另有 1,229 KB 预取) | 首页只放标题 + 日期, 不放动画和装饰; 首屏文字先出, 重资源延后. |

## 2. 首屏资源: 改造前后

### 2.1 首页自身的代码 (同一沙箱用 esbuild minify 打包, React 外置, 与 Docusaurus 实际 chunk 不完全相同, 但量级可比)

| | 改造前 | 改造后 |
|---|---|---|
| 首页入口 JS | 736,637 B (gzip 220,049 B), 单个 chunk | 24,103 B (gzip 8,023 B); 第二轮把 import() 挪进 `lazyChunks.js` 后重测 |
| 首页入口 CSS | 含 swiper 样式 | 16,481 B (gzip 3,962 B) |
| 入口里的大件 | recharts 236 KB、motion-dom 113 KB、swiper 87 KB、wordStats 35 KB、lodash 33 KB、ecosystem 30 KB、framer-motion 15 KB | 无; 只剩 projectData 9 KB、HomePage 5 KB、latestDocs 3 KB |

改造后按需加载的块:

- recharts 共享块 410,461 B (gzip 112,402 B): 字数图进入视口时加载 (首屏在右栏顶部, 所以水合后立刻开始, 但不挡首屏文字和头图).
- 字数数据 wordStats 35 KB (gzip 10 KB): 与字数图同一时机.
- WakaTime 热力格 `Pulse` 5 KB: 滚到"时间"一节附近才加载, 不依赖 recharts.
- 每日曲线 `WakaTimeChart` 13 KB: 点"每日曲线"才加载.
- 关系图 `EcosystemPanel` 38 KB (gzip 11 KB, 含 ecosystem.ts 30 KB): 点"更多 · 项目关系图"才加载.
- swiper (87 KB) 与 framer-motion + motion-dom (约 128 KB) 首页不再引用.

### 2.2 头图

`static/img/main_menu_misaka.png`: 1441x755, 1,752,603 B, 原来被 `<link rel=preload as=image>` 抢先下载, 显示宽约 1310px, 高约 690px (1440 视口).

改成个人卡顶部横幅, 宽 320px, 高 168px (= 320 x 755 / 1441, 不裁切); 窄屏 (<=996px) 宽 100%, 固定高 160px, `object-fit: cover`. 文件换成新增的两张 WebP (质量 72, method 6, 沙箱 Pillow 生成):

- `static/img/main_menu_misaka-640.webp`: 640x335, 15,486 B (2x 屏的 320px)
- `static/img/main_menu_misaka-960.webp`: 960x503, 26,168 B (3x 屏或窄屏)

1,752,603 B -> 15,486 B, 减少 99.1%. 原 PNG 保留, 首页不再引用. `<img>` 带 `width/height` 与 `aspect-ratio`, 不产生布局跳动.

### 2.3 全站共享、本轮没动、建议下一步砍的

数字来自仓库里现有的 `build/` (旧版首页时的构建产物, 只能说明量级):

- `assets/js/main.*.js` 1,847,160 B (gzip 362,489 B), `assets/css/styles.*.css` 368,632 B (gzip 73,811 B): 全站主包.
- `img/logo.png` 1,086,560 B: 导航栏 logo, 每页都 preload.
- `default-img/neko_left.png` 440,970 B + `neko_right.png` 435,685 B: 每页都 preload.

这三张图合计约 1.96 MB, 换成 WebP 并按显示尺寸缩放是下一个最大收益, 但它们属于主题层, 不在首页决策范围内, 我没改.

## 3. 配色、字号、间距、动效 (全部取现有令牌)

- 主色 `--ifm-color-primary` #ff88ff (悬停 `--ifm-color-primary-light` #ffafff); 背景 `--ifm-background-color` #121212 (暗色); 线框 `--ifm-color-emphasis-200`; 次级文字 `--ifm-color-emphasis-700`, 三级 `--ifm-color-emphasis-600`; 面板底 rgba(255,255,255,0.025). 没有新增颜色变量.
- 数据色沿用字数图原有四色: 增量 #FF61FF、docs/blog #F7CE45、ai-docs #8CFF00、删字 rgb(97,255,97); WakaTime 语言色沿用 `WakaTimeChart/colors.ts`.
- 字号层级: 28 (站名, 700, letter-spacing -0.02em) / 20 (节标题, 600) / 15 (正文, 行高 1.6) / 13 (说明、按钮) / 12 (标签、刻度) / 11 (数字卡标注). 数字统一 `--ifm-font-family-monospace` + `tabular-nums`.
- 间距: 4px 基数, 实际用 4 / 8 / 12 / 16 / 24 / 32 / 48 / 56. 左右栏间距 48px, 节与节 56px (窄屏 40px), 节标题下 16px, 面板内边距 16px. 圆角: 个人卡 16px, 面板与卡片 12px, 胶囊 999px.
- 动效: 悬停 150ms, 展开与导航短线 240ms, 缓动统一 cubic-bezier(0.2, 0.8, 0.2, 1); 项目卡悬停上移 4px; 跑马灯线性, 每张卡 4 秒. `prefers-reduced-motion: reduce` 下全部关闭, 跑马灯改为横滑列表.
- 字体: 不引入网络字体, 沿用 Infima 系统字体栈.
- 背景: 24px 间距的 1px 点阵, `radial-gradient(rgba(255,255,255,0.07) 1px, transparent 1px)`, 0 字节.
- 新增依赖: 无.

## 4. 每块可视化的图表选择

| 数据 | 位置 | 之前 | 现在 | 为什么更好 |
|---|---|---|---|---|
| 笔记字数 | 右栏第一节 | 一张图四条线、两根纵轴 | 上: docs/blog + ai-docs 堆叠面积 (顶边 = 总量), 200px; 下: 每次提交增量柱, 正负分色, 0 线常显, 96px; 图头给总字数和区间净增 | 两类累计不再重合, ai-docs 的厚度即占比; 增量单独一根轴, 没有两根轴 0 对不齐的问题 |
| WakaTime | 右栏"时间" | 两张每日多折线 (系统、语言), 7 条语言线每天抖动, 几乎全部重叠 | 默认: 53 周每日热力格 (分位数分五档) + 30/90/365 天语言占比条与列表; 每日曲线收进按钮 | 热力格一眼看出节奏与断档, 占比条读得出排名, 两者都不重叠; 不用 recharts, 5 KB |
| 项目关系 | 项目条带的"更多"里 | 默认渲染, 连线穿过节点名 | 点开才加载; 连线压在节点下; 悬停或聚焦节点只点亮它的连线, 其余降到 8% | 关系图只在需要时付费; 60+ 节点时也能看清一条依赖链 |
| 近一年项目 | 右栏"近一年在做" | swiper 轮播 | CSS 跑马灯, 卡片 240x252 | 功能不变, 少 87 KB, 悬停/聚焦暂停, 减弱动效下可横滑 |

## 5. 每处重叠的定位与改法

| # | 叠在一起的元素 | 原因 (哪条规则) | 改法 | 文件:行 |
|---|---|---|---|---|
| 1 | 关系图连线 `svg.edges` 压在节点按钮 `.node` 的名字上 (HXLoLi-NaGaMe、HX-AI-Review 等, 1440 宽截图可见) | `.edges { position: absolute }` 没有 z-index, 节点是普通文档流, 绝对定位元素按绘制顺序画在其上 | `.edges` 加 `z-index: 0`; `.laneHead, .band` 加 `position: relative; z-index: 1`, 节点底色不透明, 连线从节点后面穿过 | `src/components/home/home.module.css` 111, 442-446 |
| 2 | 关系图节点名被拆成两行且断在词中间 ("ANiMeRenameTool / s"、"BiLiBiLi_DanMu_Cr / awling") | `.nodeName { overflow-wrap: anywhere }` 在 8.2rem 宽格子里任意位置断行 | 单行 `white-space: nowrap; text-overflow: ellipsis`, 全名放 `title` | `home.module.css` 212-220; `EcosystemGraph.tsx` 229 |
| 3 | 项目卡标签行压住描述第 4、5 行 | 标签 `position: absolute; bottom: 22px`, 描述 `-webkit-line-clamp: 5`, 卡片固定 300px 高 | 卡片改纵向 flex: 封面 120px / 正文 flex:1 (描述 3 行) / 底栏 (标签 + 日期) 在文档流里 | `src/components/ProjectCarousel/css.css` 32-49, 73-83 |
| 4 | 项目卡封面与卡片标题重复, 两侧渐变遮罩盖住可点区域 | 遮罩是一层 `position:absolute; z-index:10` 的 div | 遮罩改为容器 `mask-image`, 不再有覆盖层 | `ProjectCarousel/css.css` 4-9 |
| 5 | 字数图 docs/blog 线与总量线完全重合; ai-docs 线与增量线的 0 段重合 | 三条累计线共用右轴, ai-docs 出现前 docs/blog = 总量; 增量与累计共用画布 | 改为堆叠面积 + 独立增量柱两块图 | `src/components/chart/WordCountChart/index.tsx` 121-146, 150-182 |
| 6 | 个人卡站名贴着头图下沿 | 头像 `margin-top: -28px` 上移, 站名块只有 26px 顶距 | 站名块顶距 32px | `src/components/home/page.module.css` 77 |

## 6. 首屏压缩

- 自我介绍三段 (约 190 字) -> 一句 (`data/profile.ts` 的 `profileLede`): 第二段的"笔记 -> 库 -> 应用"并进这一句; 第三段讲站点迁移, 与首屏无关, 删除. 详细的技术栈仍在页尾"关于".
- 头图 690px -> 168px (见 2.2).
- 1440x900 首屏现在同时看得到: 个人卡全部内容 + 分节导航 + 完整的字数图 + 最近笔记的前五条; 改造前首屏只有自我介绍、四个数字卡和字数图, 头图要再滚一屏.

## 7. 改了哪些文件

- 改写: `src/components/home/HomePage.tsx`、`src/components/home/home.module.css` (只留关系图与详情卡样式)、`src/components/home/EcosystemGraph.tsx` (悬停高亮、节点 title)、`src/components/ProjectCarousel/index.tsx`、`src/components/ProjectCarousel/css.css`、`src/components/chart/WordCountChart/index.tsx`、`data/profile.ts`
- 新增: `src/components/home/page.module.css`、`src/components/home/LazyMount.tsx`、`src/components/home/EcosystemPanel.tsx`、`src/components/home/lazyChunks.js`、`src/components/home/lazyChunks.d.ts` (第二轮)、`src/components/chart/WakaTimeChart/Pulse.tsx`、`src/components/chart/WakaTimeChart/pulse.module.css`、`static/img/main_menu_misaka-640.webp`、`static/img/main_menu_misaka-960.webp`
- note: `.agents/notes/implemented/feature/2026-10-31-homepage-profile-and-project-index.md`、`.agents/notes/implemented/bug-fix/2026-10-31-word-count-chart-zero-baseline.md` 就地改写
- 第二轮 (挪走内部说明): `docs/home-refactor.md` -> `.agents/research/home-refactor.md`、`docs/home-redesign.md` -> `.agents/research/home-redesign.md` (原路径留 draft 占位, 待手动删除); `sidebars.ts` 删 `home-refactor` 条目 (不删的话文件挪走后侧边栏引用不存在的 doc id, Docusaurus 启动即报错); `data/latestDocs.ts` 删 `/docs/home-refactor` 条目 (现剩 4 条, 下次 docs-rss-plugin 生成会补第 5 条); `data/noteReferences.json` 删两条 `docs/home-re*.md` 键
- 没动: `src/pages/index.tsx`、`ProjectDetail.tsx`、`data/ecosystem.ts`、`data/projectData.ts`、`WakaTimeChart/index.tsx`、`LatestDocsSection`、`package.json`

## 8. 验证了什么, 没验证什么

### 第二轮 (2026-11-01 下午): 类型检查改为对得上门禁的做法

上一轮这里写的是"沙箱里 strict 关、桩替代 Docusaurus 模块的检查无报错". 那个检查和门禁不是一回事: 它没用仓库的 `module/moduleResolution: nodenext`, 所以没报 `HomePage.tsx` 里 5 处 `import()` 的错误. 那句话作废.

这一轮的做法:

- 环境: 沙箱里 `typescript@5.6.3` (仓库锁 `~5.6.2`), `@types/react@19`、`@types/node@22`、`recharts@2.15.3`、`@docusaurus/module-type-aliases@3.7.0`、`@docusaurus/types@3.7.0`、`react-icons@5`, 外加 `@docusaurus/theme-classic@3.7.0` 的 `src/theme-classic.d.ts` (给 `@theme/Layout` 的 `title` 等 props). 目录里放无 `"type"` 字段的 `package.json`, 与仓库一样让 `.ts/.tsx` 按 CJS 格式判定.
- 配置: 仓库原样的 `tsconfig.json` (nodenext + `strict: true` + `paths: {"@site/*": ["./*"]}`), 用 `tsconfig.check.json` 继承它, `include` 改为切片目录下的 `src/**/*` 加上面两份类型引用.
- 文件: 从你盘上拉下来的真实文件, 不是桩: `src/components/home/` 全部、`chart/WordCountChart/index.tsx`、`chart/WakaTimeChart/{index,Pulse,colors}`、`ProjectCarousel/index.tsx`、`src/utils/wordCountUtils.ts`、`src/types/*.d.ts`、`data/{profile,wordStats,ecosystem,projectData,latestDocs}.ts`; `*.module.css` 走 module-type-aliases 的声明. 共 497 个文件进入编译 (含 node_modules 类型).
- 命令: `npx tsc --noEmit -p tsconfig.check.json`

改之前的输出 (与你盘上看到的 5 条逐字一致):

```
src/components/home/HomePage.tsx(23,25): error TS2834: Relative import paths need explicit file extensions in ECMAScript imports when '--moduleResolution' is 'node16' or 'nodenext'. Consider adding an extension to the import path.
src/components/home/HomePage.tsx(23,60): error TS2307: Cannot find module '@site/data/wordStats' or its corresponding type declarations.
src/components/home/HomePage.tsx(27,37): error TS2835: Relative import paths need explicit file extensions in ECMAScript imports when '--moduleResolution' is 'node16' or 'nodenext'. Did you mean '../chart/WakaTimeChart/Pulse.js'?
src/components/home/HomePage.tsx(28,41): error TS2834: ... Consider adding an extension to the import path.
src/components/home/HomePage.tsx(29,42): error TS2835: ... Did you mean './EcosystemPanel.js'?
```

(未加 theme-classic 类型时还有一条 `HomePage.tsx(79,13) TS2322` Layout 没有 `title`, 那是切片缺类型造成的, 加上后消失; 你盘上的 tsc 没有这条.)

改法: 5 处 `import()` 统一挪进 `src/components/home/lazyChunks.js`, 类型由同名 `lazyChunks.d.ts` 给 (`typeof import('...')`, 按文件自身 CJS 格式解析, 不要求扩展名, `@site/*` 映射有效). tsconfig 没开 `allowJs`, tsc 不检查 `.js`; webpack 照常把每个 `import()` 拆成独立 chunk. 没用 `.js` 扩展名方案: tsc 能过, 但 Docusaurus 3.7 的 webpack (`lib/webpack/base.js`) 只配了 `resolve.extensions`, 没配 `extensionAlias`, 构建时 `index.js` 解析不到.

改之后的输出: 无任何输出, `exit=0` (`--listFilesOnly` 确认 `HomePage.tsx`、`Pulse.tsx`、`EcosystemPanel.tsx`、`WordCountChart/index.tsx`、`lazyChunks.d.ts` 都在编译范围内).

附带: 首页去掉 swiper 后, 基线里 `ProjectCarousel/index.tsx|TS1479` 两条在切片里已不再出现; 门禁的规则是少了记 warn ("可运行 --update-baseline 收紧"), 不会 fail. 基线文件我没改.

拆块复测: esbuild (`--bundle --splitting`, React 与 `@docusaurus/*` 外置) 入口 `HomePage.js` 24,103 B (gzip 8,023 B), WordCountChart / wordStats / Pulse / WakaTimeChart / EcosystemPanel 各自成块, recharts 共享块 410,455 B.

这一轮仍没做的: 全仓 `npx tsc --noEmit` (沙箱没有全仓依赖, 只跑了上述切片, 切片外的文件结果以你本机为准); `npm run gate`; `npm start` / `npm run build`; `uv run scripts/redlines/agent_notes.py --diff` 仍然跑不了, 没有通过与否的结论. `sidebars.ts` 在 notes.config.json 的 guarded 列表里, 这次删条目需人工核对.

### 第一轮

验证了:

- 沙箱渲染 1440x900 / 1280x720 / 390x844 三个视口, 逐屏截图看过: 上面 6 处重叠在新截图里都不再出现; 关系图展开后连线在节点下方.
- (作废, 见第二轮) 第一轮的类型检查用的是 strict 关、桩替代 Docusaurus 模块、非 nodenext 的配置, 与门禁不等价, 漏掉了 5 条 TS 错误.
- 懒加载拆块: esbuild 打包确认 recharts、wordStats、关系图、热力格、每日曲线各自成块, 入口不含它们.
- 某个懒加载块下载失败时, 只有那一块显示提示, 不会整页白屏 (沙箱里实际遇到过一次本地服务掉线, 加了错误边界后复测).

没验证:

- 没在真实 Docusaurus 里构建或运行 (红线). Docusaurus 的 webpack 拆块、SSR 输出与我的 esbuild 结果会有差异; recharts、framer-motion 如果已被别的页面打进全站主包, 首页的实际节省会比上表小.
- `React.lazy` 在 Docusaurus 3.7 的 SSR 下: `LazyMount` 首次渲染只出占位, 服务端不会碰懒加载组件, 按理不影响 SSR, 但未实测.
- WakaTime 用的是合成数据; 真实数据的分布、`system` 字段是否每天都有, 未核对. 热力格的"天数 / 连续天数"口径依赖真实数据.
- 浅色主题: 站点配置禁用了切换 (`disableSwitch: true`, 默认暗色), 只看了暗色.
- `uv run scripts/redlines/agent_notes.py --diff` 我跑不了 (脚本与仓库在你机器上), 没有声称通过; 请在本机运行. 新增的 `page.module.css`、`pulse.module.css`、两张 WebP 属于非 AST 资源, 需人工核对.
- Lighthouse / 真实网络下的 LCP、CLS 没测.

## 9. 第三轮 (2026-11-01 晚)

### 9.1 最近更新拆成三栏

- `plugins/docs-rss-plugin.mjs` 在 `allContentLoaded` 里直接读 Docusaurus 已加载的 blog / docs / ai-docs 内容, 用它们自带的 permalink, 不再手拼路由; 过滤 `draft`、`unlisted`、`hx_protected`; ai-docs 的日期优先取 `frontMatter.created_at`.
- `data/latestDocs.ts` 导出 `latestBlogs`、`latestAiDocs`, 保留 `latestDocs` (笔记); 首页用三个 tab (博客 / AI 文章 / 笔记), 各最多 6 条. 博客链到 `/blog/...`, AI 文章链到 `/knowledge-base/...`.
- 用 mock 的 `loadContent` / `allContentLoaded` 连续生成两次, 输出逐字节一致.

### 9.2 WakaTime: 两个维度分开

- 编码时长 = 当天 `languages` 各项之和; 使用设备 = 当天 `system` 各项之和, 按 Linux / Windows / Mac 拆. 实测一天的设备时长常是编码时长的 5 到 20 倍 (含挂机), 上一轮把 system 之和当编码时长是错的.
- 解析放在 `WakaTimeChart/wakaData.ts`, 摘要和趋势图共用一次下载.
- 热力格三种模式: 编码 / 设备 / 叠加 (设备做底色, 编码是格子中间的小方块). 每格悬浮卡片给当天两项时长、各系统和前三语言, 方向键可逐格移动.
- 展开后是一张满宽约 340px 高的图: 维度是两个独立开关 (单看或叠加, 叠加时设备按系统堆叠压暗做底、编码是粉线), 粒度 日 / 周 / 月 (周、月取日均), 单位 小时 / 分钟. "更多 · 展开趋势图"用 `grid-template-rows` + `clip-path` 平滑展开, `prefers-reduced-motion` 下直接显示.
- 数据地址: 原 `raw.githubusercontent.com/HengXin666/HengXin666/main/data/history.json` 2026-11-01 实测 404, 换成 jsDelivr 同文件 (200, CORS `*`, 当天 731 条, 2024-11-01 到 2026-11-01). jsDelivr 回 `max-age=604800`, 所以请求带 `?d=当天日期`.

### 9.3 项目数据没有流水线

- `.github/workflows/` 里没有 `schedule` / `cron`; `scripts/` 里没有写 `data/projectData.ts` 的脚本. 所以 `pushedAt`、`stats` 是手工维护的, 首页文案也改成"手工维护".
- `HX-Jungle` 匿名访问 GitHub 页面 404 (其余仓库 200), 按私有 / 不可见移除. GitHub API 当时对我这边的账号返回 403, 没能用 API 复核.

### 9.4 关系图为什么之前又窄又高, 现在怎么放

- 旧版把 7 列塞进约 760px 的右栏, 每格只放得下一个节点, 只能竖排, 所以细长.
- 现在是浮层: 宽 `min(1320px, 100vw - 48px)`, 高不超过 92vh. 行 = 7 条产品线, 列 = 5 个层级 (源头 -> 门户), 列宽按每层单格最多节点数加权 (源头 1.6 : 应用 1.45 : 轮子 0.95 : 平台 0.9 : 门户 0.6). 节点是 24px 高、最宽 168px 的胶囊, 同一格横向换行. 1440x900 下 7 行全部放进浮层, 不需要滚动.
- 连线默认淡色, 推断关系是虚线; 悬停 / 聚焦节点时相关线点亮并流动, 其余节点和线变暗. 图例四种连线可开关. 点节点叠详情卡 (层级高于浮层). 窄屏浮层贴底, 图横向滚动.

### 9.5 悬浮提示统一用 `src/components/common/Tooltip`

在仓库已有组件上扩展, 没另起一套: 支持鼠标、键盘聚焦、触屏轻点、Esc 关闭, `role="tooltip"` + `aria-describedby`, 固定定位并夹在视口内, 上方不够时翻到下方; 新增 `useHoverCard` 给热力格共享一张卡片; 旧番剧页的 `selectable` / `style` 用法保持兼容.

原生 `title` 悬浮共处理 9 处: 换成统一 Tooltip 6 处 (侧栏统计数字、项目卡、项目卡标签 (并进项目卡的卡片)、关系图图例的连线类型、关系图节点、热力格), 删除或挪进正文 3 处 ("找到我"链接的说明改成链接下方可见文字; 图例里"推断"的说明改成图例旁常驻文字; 关系图连线上的 SVG `<title>` 关系依据挪进节点卡片). `WakaTimeChart/index.tsx` 里旧的 `title="操作系统使用"` 等是图表组件的标题 prop, 不是原生悬浮, 不计入. `Layout` 的 `title` 是页面标题 prop, 不算; `img` 的 `alt` 未动.

"找到我"四个链接用已有的 `react-icons@5.5.0` (`FaGithub`、`SiBilibili`、`SiLeetcode`、`FaEnvelope`), 没有新增依赖.

### 9.6 验证了什么, 没验证什么

验证了:

- 切片 tsc (同第二轮的 harness): `npx tsc --noEmit -p tsconfig.check.json` 无输出, `exit=0`.
- `node --check plugins/docs-rss-plugin.mjs` 通过; mock 生成测试通过, 博客与 AI 文章链接正确.
- 首页 SSR smoke (esbuild 打包后 `renderToString`): 3 个 feed panel、4 个 SVG 链接图标、首页无 `title` 属性, 关系图无 `<title>`.
- esbuild 拆块: 首页入口 JS 约 35.6 KB (gzip 约 12.1 KB), 首页 CSS 约 28.5 KB (gzip 约 6.2 KB); Pulse 约 12 KB、趋势图约 8 KB、关系图约 40 KB 各自成块, recharts 共享块约 415 KB.
- 沙箱 Chromium (Playwright, 用 jsDelivr 拉来的真实 history.json) 1440x900 与 390x844 截图看过: 热力格三种模式与悬浮卡片、趋势图叠加与提示、项目卡 Tooltip、"找到我"图标、关系图整张放进浮层; 页面无 console error; 触屏轻点统计数字能弹出提示.
- 写盘: 所有改动文件用 PUT 写到你的盘上, 逐个读回比对 md5. (`.css` 的 GET 会命中 CDN 缓存读回旧内容, 加防缓存参数后一致.)

没验证:

- 真实 Docusaurus `build` / `start` (红线, 没跑); webpack 拆块与 SSR 输出可能与 esbuild 有差异.
- 全仓 tsc、`npm run gate`、`uv run scripts/redlines/agent_notes.py --diff`.
- Lighthouse、真实网络下的 LCP / CLS.
- 浏览器截图用的是沙箱里的 esbuild 拼装页, 不是 Docusaurus 主题外壳; 导航栏、全站样式叠加后的效果以本机为准.

## 第三轮 (2026-11-01 晚): 语言占比、首页追番、番剧标签

详细决策见 `.agents/notes/implemented/feature/2026-11-01-anime-tags-and-home-anime.md`; WakaTime 部分并入 `2026-10-31-homepage-profile-and-project-index.md`.

- 语言占比: `WakaTimeChart` 加「数值: 绝对时长 / 语言占比」切换, 同一张趋势图改百分比堆叠面积. 前 5 语言 + 其他, 最大余数法按 0.1% 分配, 每期合计恰好 100.0 (真实 history.json 730 个编码日全部验证). Y 轴固定 0–100%. tooltip 给各语言百分比 (降序) 与该点编码总时长. 不用 recharts `stackOffset="expand"`: 它在渲染时归一化, tooltip 拿不到百分比, 零值期会出 NaN.
- 首页追番: `HomeAnime.tsx`, 状态计数 + 最近更新的 6 张在看封面 + 标签入口 (萝莉高亮, 链到 `/anime?tag=`). 数据走 `useAnimeRecords` (HXLoLi-ANiMe CDN), 无硬编码数字. ANiMeRecord.json 约 12.5 MB (gzip 2.4 MB), 经 `lazyChunks.js` 的 `loadHomeAnime` + LazyMount 懒加载. 不画分布图: 160 部里看过占 134, 分布图信息量低, 还要再拉一份 recharts.
- 番剧标签: `data/animeTags.ts` 手工表 (bgm subject id → 标签), 8 类: 萝莉 34、异世界 35、日常 30、恋爱 17、搞笑 29、战斗 41、催泪 6、悬疑 11; 157 部有标签. `getRecordTags` 合并表内标签与原 `user_status.tags`. 番剧页 `TagFilter` 按 `?tag=` 筛选, 带 tag 进入自动切「全部」. 角色 tags 是资料框键值, 不用来推断萝莉.

没验证: 全仓 tsc、agent_notes `--diff`、真实 Docusaurus 渲染; CDN 失败时首页番剧块会一直显示骨架 (animeStore 无错误态).
