# Agent Note: 语气档案补手法层, 与措辞语料一起注入 status

Status: implemented

Decision-ID: voice-profile-dimension-layer

- **引入于**: 本次改动 (待提交后回填该次提交的 sha)
- **引用落点**: `flow_read.py` 的 `voice_profile_lines` 与 `print_status`

## Code

- `.agents/skills/hx-note/scripts/cli/flow/flow_read.py`

## Problem

voice-corpus-forced-injection 自己写了局限: 语料只覆盖**措辞层**, 手法层 (情绪是输入还是装饰、读者在不在场) 测不出来也注入不了. 2026-11-08 对 45 篇 ai-docs 与作者手写语料的统计印证了这一点, 措辞黑名单在, 手法上的差距仍然很大:

| 指标 | ai-docs | 作者手写 |
|---|---|---|
| 第一人称每千字 | 0.27 | docs 0.9, blog 3.07 |
| 开头 `> [!NOTE]` 反问钩子 | 30/45 篇 | blog 0 篇 |
| 「不是 X, 而是 Y」 | 30/45 篇 | blog 2/43 篇 |
| `## 0xNN` 标题 | 44/45 篇 | docs 与 blog 都是 0 |

另一个缺口: 现有的正向参照 (`exemplars.md` 与画像里的「口吻样本」) 全是 blog 的单句, 带 qwq、=-= 和吐槽. 知识稿照着学, 要么学不像, 要么把日记口吻搬进教程. docs/ 里 439 篇作者自己写的技术笔记 (C++/刷题/游戏引擎/开源项目) 才是知识稿该对标的语料, 之前没被用上

## Decision

1. 新增 `ai-docs/.hx-voice-profile.md`, 结构参考 guizang-voice 的分维度档案: docs 体 D1 到 D6 (知识稿默认), 情绪手法 E1 到 E3 (只在素材里真有那件事时用), blog 体 B1 到 B2 (只在有遭遇时借), 外加 ai-docs 指纹反例表. 每条带频率与两处原文出处
2. `flow_read.py` 新增 `voice_profile_lines`, 在 status 的语料框之后打印每条手法的编号、名字与频率. 原文证据不注入, 留在档案里
3. `shared/voice/index.md` 写明扩展规则: 加一条手法前先找到两处原文, 找不到就不加. 档案不进 lint

## Alternatives considered

- **什么都不做 / 复用现有** — 最强理由: 已有 `.hx-voice.toml` 的 good 样本和画像的口吻样本, 再加一份档案是第三处语气来源. 否决: 现有两处都是单句, 只能告诉模型"作者说过这句话", 不能告诉它"作者在什么情况下用什么手法". 统计里第一人称 0.27 对 0.9 的差距在单句语料下一直没缩小
- **把手法写进 lint 规则** — 最强理由: 机械闸门是这套流程的一贯做法. 否决: 手法是启发, 不是判据; 例如要求每千字至少 0.9 个「我」, 模型会往每段塞一个「我」, 变成新的指纹. 与 voice-corpus 那篇否决"查有没有用上语料"同一个理由
- **全量注入档案原文** — 最强理由: 证据就在眼前, 模型不用再开文件. 否决: 档案约 5000 字, 每一步都注入会淹没阶段指引; 只注入摘要, 要写时再读原文
- **只用 blog 语料** — 最强理由: blog 口吻最鲜明, 也是画像现成的来源. 否决: blog 是日记, 使用者明确说 ai-docs 不该照抄那种口吻

## Consequences

- status 每次多约 12 行
- 档案是人工整理的统计结果, 语料增长后数字会过时; 目前没有脚本自动重算, 改条目时要重跑统计
- docs/ 里混有课程笔记的转抄 (如 Docker、Redis 的教程段落), 统计时只取 C++/刷题/游戏引擎/开源项目四个目录, 仍可能混入少量非作者原文

## Testing

```
uv run .agents/skills/hx-note/scripts/cli/flow/hx_flow.py status --slug <任一 slug>
# -> 语料框之后出现「语气档案」框, 列 D1..D6 / E1..E3 / B1..B2 共 11 行

mv ai-docs/.hx-voice-profile.md /tmp/ && uv run ... status --slug <slug>
# -> 没有档案框, 其余输出不变; 再移回
```
