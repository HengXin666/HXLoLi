# Agent Note: 笔记引用关系改为页面底部自动渲染, 取消手写参考来源章节

Status: implemented

- **引入于**: `f28d9cfd13`
- 影响: 新增 `plugins/note-references-plugin.mjs` 与 `src/components/NoteReferences/`; `DocItem/Layout` 挂载; `docusaurus.config.ts` 注册; 两个模板与 `makeDoc.py` 去掉「参考来源」槽位; `steps/5-place/impl/naming-and-identity.md` 改章节编号规则

## Problem

每篇笔记末尾手写一节 `## 0x0A 参考来源`, 有四个问题:

1. **读者读完就忘, 而且看不出方向.** 它是线性文字, 分不清「本文引了谁」与「谁引了本文」——
   前者是延伸阅读, 后者是「谁在依赖这套结论」, 对读者是两件事。
2. **必然与正文漂移.** 正文里的链接改了, 末章不会跟着改。
3. **分不出站内与站外.**
4. 因为「参考来源」被固定写成 `0x0A`, 正文最后一个 `0x0N` 与它之间永远缺几个号, 读者会以为漏了内容。

## Decision

引用关系改为**构建期从正文抽出 + 页面底部渲染**, 取消手写章节。

- `plugins/note-references-plugin.mjs` 在 `contentLoaded` 扫 `ai-docs/`, 解析链接建三张边表:
  本文引用 (站内出边) / 本文被引用 (站内入边) / 站外来源; 落盘 `data/noteReferences.ts`,
  页面 import (同 `tag-index-plugin` 的模式)。
- `src/components/NoteReferences/` 渲染成可切换方框, 由 `DocItem/Layout` 挂在正文与 `DocItemFooter` 之间,
  与 `AIDocHeader`、`MitemitePanel` 同一处。
- 两个模板与 `makeDoc.py` 去掉「参考来源」槽位, 写明不要写这个章节。

识别两种写法: md 链接 title 里的 `hxid:hx-xxx` (站内主路径), 与相对路径 (用 hxid 反查, 兼容漏 title);
`http(s)://` 归为站外来源。

## Alternatives considered

- **保留手写章节只改格式** — 最强理由: 不动构建链路, 零风险, 而且作者习惯在末章写一句「这篇提供了什么」,
  那句话有解释价值。否决: 那句话放正文引入里更自然; 而结构信息 (谁引谁) 由人手抄必然漂移, 也分不出方向。
- **写成组件让作者在正文手写 `<NoteReferences />`** — 理由: 显式可控。否决: 每篇都要记得加, 忘一篇少一篇;
  这是每篇都该有的东西, 不该靠人记。
- **复用既有的 docs-graph 全站图** — 理由: 已有实现, 最省。否决: 它扫 `docs/` 不含 `ai-docs/`,
  且是独立图页面, 不是嵌在每篇底部的小方块。

## Consequences

- 31 篇笔记产出 33 条站内引用、150 条站外来源, 零人工维护。
- 没有引用的页面整个不渲染, 不留空框。
- 代价: 引用图在构建期才算, 新写的链接要重新构建才出现在方框里。
- 踩过的坑: 链接前缀不能写死 `/knowledge-base/...` —— 本站双平台部署, baseUrl 在 GitHub Pages 是
  `/HXLoLi`、Cloudflare 是 `/`, 写死会在其中一边 404, 必须用 `useBaseUrl`; 而且它是 Hook,
  只能在组件顶层调用, 不能放进渲染回调。

## Testing

```
npx docusaurus build
# 期望: [note-references] 31 篇笔记, 33 条站内引用, 150 条外部来源
grep -o 本文引用 build/knowledge-base/程序语言/Python/Python爬虫库选型调研.html
```
