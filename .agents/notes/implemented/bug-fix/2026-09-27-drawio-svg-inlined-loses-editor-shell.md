# Agent Note: 小于 10KB 的 .drawio.svg 被内联后丢掉 draw.io 编辑外壳

Status: implemented

- **引入于**: `cf8d6ef972`

- 影响: `scripts/run-docusaurus.mjs`(新增) / `package.json` 的 `start` `build` `serve` `deploy` `dev:private` / `docusaurus.config.ts` 的构建护栏 / `src/theme/MDXComponents/Img/index.tsx` 的 draw.io 分支

## Problem

ai-docs 的一篇笔记里两张 `.drawio.svg` 退化成裸图片: 没有 `draw.io` 标题条, 也没有"编辑"按钮,
和同一站点其它笔记里的同款配图长得完全不一样。

伪装的点在于**同一页里两种形态并存**: 我先看 `C++20协程原理与应用`, 发现一张图有外壳、一张没有,
很容易误判成"笔记写法不一致"。实际拿两条 `<img>` 对比才看清:

```text
co-await-flow.drawio.svg   src="/assets/images/co-await-flow.drawio-<hash>.svg"   <- 有外壳
two-threads.drawio.svg     src="data:image/svg+xml;base64,PD94bWwg..."            <- 裸图
```

决定性线索是文件体积: `co-await-flow.drawio.svg` 是 10261 字节, `two-threads.drawio.svg`
是 6488 字节, 阈值正好卡在两者之间。

## Root cause

三个事实叠在一起, 每个单独看都正常:

1. `src/theme/MDXComponents/Img/index.tsx` 用 `src.endsWith('.svg')` 决定走不走 draw.io 外壳。
   对文件 URL 成立, 对 data URI 不成立。
2. Docusaurus 给 markdown 图片挂的是**内联 loader 前缀**
   (`!!url-loader?limit=<WEBPACK_URL_LOADER_LIMIT>&fallback=file-loader!./x.svg`), 它绕过全部
   `module.rules`, 所以想靠加一条 webpack 规则放行 `.drawio.svg` 是走不通的 ——
   唯一的开关就是那个 limit。
3. `WEBPACK_URL_LOADER_LIMIT` 默认 10000 (`@docusaurus/utils/lib/constants.js:79`),
   于是小于 10KB 的图被 base64 内联, `src` 不再以 `.svg` 结尾, 判定静默失败。

"静默"是这条 bug 的要害: 构建不报错, 页面正常显示那张图, 只是丢掉了画它的人后续还要用的编辑入口。
配图越简洁 (节点少、文字短) 体积越小, 越容易踩中 —— 也就是说**图越画得好越可能出这个问题**。

## Decision

**把内联阈值关成 0, 并保证它一定被注入。**

- 新增 `scripts/run-docusaurus.mjs`: 在 `import` 任何 `@docusaurus/*` **之前**设置
  `process.env.WEBPACK_URL_LOADER_LIMIT = '0'`, 再 spawn 真正的 docusaurus CLI, 原样透传参数与
  终止信号。
- `package.json` 的 `start` / `build` / `serve` / `deploy` / `dev:private` 全部改走这个包装器。
- `docusaurus.config.ts` 加一道护栏: 构建类命令下若该变量不为 `0`, 直接抛错并给出正确命令。
  这是为了拦住 `npx docusaurus build` 这类绕过包装器的调用 —— 否则同一份源码又会静默产出坏页面。

关键约束是**注入时机, 不是注入位置**: `constants.js` 在模块加载那一刻就把该值求值成常量, 而
`@docusaurus/core/bin/docusaurus.mjs` 顶部第一件事就是 `import {DOCUSAURUS_VERSION} from '@docusaurus/utils'`。
所以在 `docusaurus.config.ts` 里写 `process.env` 是无效的 —— 实测仍然拿到 `limit=10000`。
`scripts/` 下的既有 `patch-image-size-svg.cjs` 走的是"改 node_modules 源码", 那条路可以留在
postinstall, 但阈值这件事有官方环境变量可用, 不该再去改依赖代码。

## Alternatives considered

- **什么都不做, 让作者把图做大一点 (超过 10KB) 绕过**: 零成本, 而且现有图确实大多是这么"碰巧"正常的。
  否决理由: 它把一条渲染契约变成了对配图体积的隐性要求, 而体积跟"这张图讲得清不清楚"毫无关系。
  同一份 `hx_drawio.py` 生成的图, 节点少一张就踩雷, 等于给写作埋了一个不可见的陷阱。
- **在 `src/theme/MDXComponents/Img/index.tsx` 里改成"文件名匹配 `.drawio`"**: 改动最小,
  就地修那一处判定。否决理由: data URI 里根本没有文件名可匹配 —— 内联把文件身份整个抹掉了,
  在组件这一层已经无从判断。要修必须修在"别让它被内联"。
- **写一个 remark 插件, 给 `.drawio.svg` 的图片改成 `pathname://` 前缀绕过 webpack**:
  精准, 只影响 drawio 图。否决理由: `pathname://` 是 Docusaurus 的逃生舱, 用在一个高频路径上等于
  把这类资源的哈希指纹、存在性校验、构建期断链检查全部放弃; 而且还要自己处理 baseUrl 拼接与
  保护页面路径。为一张图放弃整套资源管线不划算。
- **在 `docusaurus.config.ts` 顶部直接设置 `process.env`**: 最省事, 不用新增脚本, 不用改 scripts。
  否决理由: **实测无效**。`@docusaurus/utils` 在 config 之前就已加载, 常量早固化了
  (验证方式是单独跑一段脚本: 先 `require` utils 再设 env, 拿到的 loader 串仍是 `limit=10000`)。
  这个方案看起来最自然, 因此也最危险 —— 它会让下一个人以为"已经修好了"。
- **只改 `package.json` 的 scripts, 不加 config 护栏**: 少一处代码。否决理由: 站点里
  `.agents/skills/hx-note` 与若干 Agent Note 都写着 `npx docusaurus build` 作为验证命令,
  这种调用会绕过 npm scripts; 没有护栏时它会静默产出退化页面, 与修之前完全一样。

## Consequences

- 所有 markdown 图片一律以文件形态产出, 不再有 base64 内联。代价是多几个 HTTP 请求与若干
  `assets/images/` 文件 —— 实测全站受影响的小图只有 14 个 (10 张 png / 4 张 svg), 其中 4 张就是
  本 bug 里的 `.drawio.svg`。
- 构建多了一层 node 包装进程。信号 (Ctrl-C / CI 取消) 已显式转发, 不影响 dev server 的退出。
- 判断"这张图会不会丢编辑入口"不再依赖体积, 而是"它是不是 `.drawio.svg`" —— 作者可以按内容
  需要自由决定图的繁简。
- 这条约束的位置写在两处: 包装器 (实现) 与 config (护栏)。后来者若删掉护栏, 单靠 npm scripts
  仍能正常工作, 只有绕过 scripts 的调用才会失败, 且失败信息里直接给了正确命令。
