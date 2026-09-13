# Agent Note: 空壳笔记「AI驱动的视频调研与知识沉淀工作流」被当成正常页面发布

Status: implemented

- 影响: `ai-docs/002-AI/004-记忆/004-AI驱动的视频调研与知识沉淀工作流/`(删除) /

## Problem

用户访问 <https://km.woa.qzz.io/knowledge-base/AI/记忆/AI驱动的视频调研与知识沉淀工作流>
报告「内容缺失、有些链接不是链接」, 怀疑 Markdown 语法写错。

实际情况不是渲染错误, 而是**一篇 e2e 测试跑出来的模板占位稿被当成正式笔记发布**:

| 现象 | 真实原因 |
| --- | --- |
| 正文大面积"缺失" | 源码只有 28 行, `0x00` 是「待补充: 请基于采集材料凝练核心结论.」`0x02` 是「无结构化 key_points」, 全是模板话术 |
| `> [!NOTE]` 下面是空的 | callout 语法正确, 是引用块内容被删了 —— `244e9df2aa` 提交信息写明"清 17 篇正文过程痕迹"时连正文一起删掉, 只剩空壳 |
| 有些链接不是链接 | `- [NOTE] 本正文由 AI 基于转写材料生成...` 是模板提示语残留文本; `/tmp/brtest/...` 与 `/home/hx/.dsh/dsh-blog-research/...` 是裸本地路径, 本就不是链接, 且 `/tmp/brtest` 已不存在 |
| 素材来源可疑 | 唯一"视频"是 `/tmp/brtest/e2e-video-sample.srt`(e2e 测试样本), 转写只有 6 段 40 秒演示字幕 |

## Root cause

`hx-to-ai-docs` 沉淀流水线的 e2e 用例跑完把 staging 模板 (`staging/draft.md`:
带占位符 + 本地路径 + `[NOTE]` 提示语) 直接落盘进了 `ai-docs/`, 之后**没有任何收尾门禁**
拦「未完成稿」, 于是目录重分类(`244e9df2aa`)与 tag 治理(`dfecd341b0`)两轮全库整理都把它
当成正常笔记搬运、补 hxid、换 tag, 每次都让它更像"正式文档"。

## Evidence

- 全库扫描: `ai-docs/**/index.md` 里占位符「待补充/TODO:」只此 1 处, 裸本地路径 4 处全在这一篇;
  20 处 `> [!NOTE]` 里只有它是空壳(其余都跟着真实提示文案)。
- 渲染核对: 无头 Chromium 打开 dev 站点, DOM 正文与 md 源文件**逐字一致**
  —— 页面没丢任何东西, 是源头就空。`build/` 里同日 19:40 的构建产物同样只有这四小节。
- 素材核对: `~/.dsh/dsh-blog-research/artifacts/rs_1788076239468/` 保留着 e2e 现场,
  `metadata.json` 的 `input_type` 为 `local-subtitle`, 源文件指向已不存在的 `/tmp/brtest`。
- hxid 反查: 全仓 `grep hx-84191bf0` 与标题引用, 除自身外**零引用**, 删除不会造成断链。

## Decision

整个目录删除 (含 `.hx-mitemite.md` 答题卡空壳), 并按既有治理流程收尾:

1. `ai-docs/.hx-id-snapshot.json` 摘掉 `hx-84191bf0`(27 篇在册), 不与剩余 hxid 冲突。
2. `node scripts/generateAiDocsSidebar.js` 重建侧边栏。
3. 直接跑 `tag-index-plugin` 的 `loadContent()` 重建 `data/aiDocTags.ts`(等价于构建期行为)。
4. `hxloli_tags.py generate` 重建 tag 注册表 generated 层。

校验: `hx_docs_id.py check` -> 「28 篇笔记的 hxid 唯一且合法, hxid 链接全部可达且路径最新」;
dev 站点原 URL 落到"找不到页面", 侧边栏「记忆」分类只剩 4 篇真实笔记。

## Consequences

- 空壳目录连同 `.hx-mitemite.md` 答题卡整体删除, 其 `hx-84191bf0` 从 `.hx-id-snapshot.json` 摘除, 不再占用 hxid。
- 侧边栏、`data/aiDocTags.ts`、tag 注册表三层生成物按既有治理流程重建, 原 URL 变为"找不到页面"。
- 该 hxid 从此不在册 —— 若将来要复用这个身份, 需要重新分配而不是恢复快照。
- 同类"低价值素材沉淀出的空壳"仍可能在下次沉淀时出现, 本次没有加自动护栏。

## Alternatives considered

- **补完正文**: transcript 只有 40 秒演示字幕, 补出来仍是低价值内容, 不如等真实素材重跑一次沉淀。
- **只做低风险修补**(删空 callout / 裸路径转行内代码): 页面不难看了, 但一篇只有模板话术的空壳
  继续占着 hxid、侧边栏与 tag 索引, 后续整理还会反复消耗人力。
- **保留但标记 `unlisted`**: 与"清理测试残留"的意图不符, 且它本来就没有读者价值。
