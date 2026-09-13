# Agent Note: 站点质量红线: 一条命令锁住所有既有功能

Status: implemented

- 影响: `scripts/quality-gate.mjs` (新增) / `scripts/regenerate-tag-index.mjs` (新增) / `scripts/quality-baseline.json` (新增) / `package.json` (新增 `gate` / `gate:fast`)
- 现象: 改 A 弄坏 B 却没人发现; 侧车 `.html` 在 dev 上"加载不出来"

## Problem

用户提出两件事, 它们其实是同一个病:

1. "这个 HTML 怎么总是加载不出来啊?"
2. "感觉我们项目需要有一个质量检查红线, 用于检查我们和测试之前所有的功能, 不然的话到时候不小心改了什么东西, 就会影响到别的东西。而我们却没发现, 这就不好了。"

第 2 点是根因陈述: 项目当时**没有任何回归门禁**。已有的检查(hxid / tag / 标点 / tsc / decks)全是
散落的独立命令, 依赖人记得去跑; CI 里只有部署流水线, 没有测试套件。

## Root cause

### A. 侧车 `.html` 在 dev server 上 404, 且**看起来不像报错**

`ppt-html-assets` 插件(`docusaurus.config.ts` 内的内联插件)通过 `configureWebpack` + `copy-webpack-plugin`
把与笔记同目录的 `.html` 复制到对应路由, **只在客户端构建时枚举一次**:

```ts
configureWebpack(_config, isServer) {
  if (isServer) return {};
  const patterns = getPptHtmlCopyPatterns(siteDir);  // ← 启动时扫盘
  ...
}
```

因此:

| 场景 | 结果 |
|---|---|
| `docusaurus build` | 每次构建重新枚举, **总是正确** |
| `docusaurus start` 启动**前**已存在的侧车 | 正确 (webpack copy 覆盖) |
| `docusaurus start` 启动**后**新增的侧车 | **404** —— 没有对应 asset |

dev server 上这个 404 被 `historyApiFallback` 兜成**应用外壳**: HTTP 200 + `content-type: text/html` +
约 2092 字节。iframe 里显示站点的"找不到页面", 但网络面板看不到 404, 所以表现成"加载不出来"而不是
"链接坏了" —— 这正是它难以定位的原因。

决定性证据(同一个 `.html`, 两种 dev server):

```
STALE  dev (:3000, 18:07 启动, 早于侧车)  -> 200 but 2092 bytes (应用外壳)
FRESH  dev (:3555, 侧车已存在后启动)      -> 200 and 712699 bytes (真实侧车)
prod   serve (build 产物)                 -> 200 and 712699 bytes (真实侧车)
```

同批 12 个侧车里, 恰好**只有新增的 2 个**失败, 旧的 10 个全部正常 —— 与"枚举发生在启动时"完全吻合。

### B. 断链不会让构建失败

`docusaurus.config.ts` 里 `onBrokenLinks: "warn"` / `onBrokenMarkdownLinks: "warn"`。
构建"成功"不代表链接是通的, 所以门禁必须自己抓构建输出里的 `Broken link`。

### C. 生成物可以静默脱钩

`sidebarsAiDocs.ts` / `src/hxdeck/decks.generated.ts` / `data/aiDocTags.ts` / `ai-docs/.hx-tags.toml`
都是**入库的生成物**。改了源不重跑生成器, 站点照样构建成功, 只是内容悄悄不对。

## Decision

新增 `scripts/quality-gate.mjs`, 收敛为 **一条命令 `npm run gate`**, 覆盖 8 类静默失效:

1. `hxid check` —— 唯一 / 合法 / 链接路径最新
2. `hxid links` —— 跨文章本地引用可达
3. tag 词表合规 + tag 注册表 generated 层新鲜
4. 中文标点归一化 (全 `ai-docs` markdown)
5. **生成物新鲜度**: 跑生成器 → 比对 → 回滚写入, 检查本身保持只读
6. 演示页引用完整性: `.html` 侧车文件存在; `.tsx` 演示页已注册进 `decks.generated.ts`
7. TS 类型检查, **带存量基线**: `scripts/quality-baseline.json` 登记 9 条历史错误, 只对**新增**报错失败。
   基线按 `文件|TS码` 计数(而非仅文件名), 因此 "同一文件新增另一类错误" 也能识别
8. 构建 + 断链断言 + **侧车产物断言**: `.html` 必须发布成真实静态文件, 而非 `plugin-pages` 应用外壳

外加可选的 `--dev[=URL]` 探测, 直接复现上面 A 类故障(专门抓"200 但只有几 KB"的软 404)。

三处**时间戳/枚举**细节必须排除, 否则门禁会自己误报:

- `data/aiDocTags.ts` 的 `generatedAt`、`.hx-tags.toml` 的 `generated_at` 是时间戳, 比对前归一。
- 路由必须复刻 `stripNumberPrefix`(去掉 `NNN-` 前缀), 否则用原始目录名拼 URL 会**全部** 404。
  第一版就踩了这个坑: 探测把 12 个侧车全报成失败, 修正后才精确命中 2 个。

## Evidence

故障注入验证(故意制造 4 类破坏, 门禁全部拦下, 退出码 1):

| 注入的破坏 | 被哪条拦下 |
|---|---|
| 新增笔记但不重跑生成器 | 侧边栏过期 / tag 索引过期 / tag 注册表过期 |
| `hxid` 格式非法 | hxid check |
| 指向不存在 hxid 的链接 | hxid links (`-> hx-deadbeef`) |
| 自造 tag + 全角标点 | tag 词表合规 / 标点归一化 |

同时确认门禁**能区分真故障与噪声**: 全绿状态下 13 项通过、0 失败; 对 `:3000` 的 dev 探测
只报 2 个新增侧车, 不误伤 10 个旧侧车。

基线本身也验证过一轮: 第一版解析 TS 报错时取错了正则捕获组(`m[3]` 是列号, 实际的码是 `m[2]`),
于是基线里 9 条全被记成 `undefined` —— 那样的基线**任何**新错误都会被当成"已知", 红线形同虚设。
修正后重放同一个 TS2322, 门禁准确报出
`新增 1 条: .../obsidian-second-brain-deck.tsx|TS2322`。

顺带修掉一个真实缺陷: 新演示页 `obsidian-second-brain-deck.tsx` 给 `<CodeBlock>` 传了 `i={1}`,
但 `CodeBlockProps` 没有 `i` 字段(`i` 是 `PageHeader`/`Callout` 的错峰序号)。`tsc` 报 TS2322,
已删除。这正是基线机制的价值: 该错误先前混在 9 条历史错误里被忽略。

## Consequences

- 站点第一次有了"一条命令锁住既有功能"的红线 (`scripts/quality-gate.mjs` + `quality-baseline.json`);
  代价是新增一条基线文件, 历史错误要通过基线豁免而不是一次修完。
- 9 条历史 `tsc` 错误被基线豁免而非修复 (涉及 CommonJS/ESM 互操作与历史组件), 属于独立议题 ——
  红线**从现在开始**有效, 但不代表存量是干净的。
- `onBrokenLinks` 没有改成 `"throw"`: 那会让存量断链一次性阻断所有人的构建, 覆盖面对比下不如在门禁里抓构建输出。
- dev 侧车 404 的根因未修, 只让它可见并提示重启 dev server; 绕过办法是 `npm run build && npm run serve`。
- 门禁尚未接进 `.github/workflows/` —— CI 接线涉及构建时长与失败策略, 需要单独拍板。

## Alternatives considered

- **只加 CI, 不做本地脚本**: CI 只覆盖 push/PR, 本地改完到推送之间仍有盲区; 且 CI 反馈慢,
  不适合"改完立刻自检"。做法是脚本优先, CI 后续直接复用同一条命令即可。
- **把 `onBrokenLinks` 改成 `"throw"`**: 更直接, 但那会让**存量**断链一下子阻断所有人的构建;
  且它只覆盖链接一类。改为在门禁里抓构建输出, 覆盖面更广且可以逐类放宽。
- **用 vitest/jest 搭真测试套件**: 站点绝大多数风险是"内容与生成物脱钩", 不是函数逻辑;
  端到端构建 + 产物断言比单元测试更贴近真实失效面, 且无需引入测试框架依赖。
- **让 tsc 直接全绿**: 9 条历史错误涉及 CommonJS/ESM 互操作与若干历史组件, 属于独立议题;
  基线机制让红线**从现在开始**有效, 不阻塞在当前任务上。
- **顺手修 dev 侧车 404 的根因**(把 webpack copy 换成 dev 期中间件): Docusaurus 3.7 没有暴露
  `configureDevServer` 钩子, 唯一注入点是 `configureWebpack` 返回的 `devServer` 字段 —— 可行但侵入较大。
  本次先让它**可见**(门禁 `--dev` 探测 + 提示重启), 修根因留待单独决策。

## 未做

- 没有把门禁接进 `.github/workflows/`。CI 接线涉及构建时长与失败策略, 需要单独拍板。
- 没有修 dev server 的侧车 404 根因(见上)。绕过办法: 新增侧车后重启 dev server, 或用 `npm run build && npm run serve`。
