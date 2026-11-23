# voice — 去 AI 味 (跨步骤能力)

**被步骤 3 / 6 / 7 / 8 共用.** 它不属于任何单个步骤, 所以放在 `shared/`

## 三个动作

| 何时 | 做什么 | 命令 |
|---|---|---|
| 步骤 6 动笔前 | 读作者真文样本, 照着语气先写引入 | `impl/exemplars.md` |
| 步骤 6 动笔前 | 读一遍已积累的好/坏表达 | `hx_voice.py samples` |
| 步骤 6 动笔前 | 从语气档案挑 2 到 3 条手法照着写 (status 每次会摘要列出) | `ai-docs/.hx-voice-profile.md` |
| 步骤 3 / 6 写完后 | 静态检测 | `hx_voice.py lint --profile atom\|article` |
| 步骤 7 盲审后 | 把新发现的表达写回语料库 | `hx_voice.py learn` |

## 产物

两个 profile, 分别对应两类产物: `--profile atom` 查 `.hx-info.md`, `--profile article` 查 `index.md`

## 实现

正向样本: `impl/exemplars.md` (作者整段真文, 仿写用). 黑名单只能堵已知指纹, 模型会换一种同样机器的说法; 样本给的是该往哪写

模板判据的来由与反例: `impl/outlook-rationale.md` (盲审判 FAIL 时读)

盲审协议 (污染控制三铁律、两类审的顺序、异源与顺序对调两条硬约束、什么时候可以跳过、**不要指望模型自我纠错**): `impl/blind-audit.md`. prompt 在 `assets/voice-audit-prompt.md` 与 `assets/fidelity-audit-prompt.md`

## 语料库

`ai-docs/.hx-voice.toml`. **扩展方式**: 新 AI 味规则 = 用 `hx_voice.py learn --bad` 追加一条(带 `--pattern` 会升级成 lint 的 E 级规则, 这才是让一次盲审变成永久拦截的开关)

## 语气档案

`ai-docs/.hx-voice-profile.md`. 语料库管**措辞** (哪个词能用), 档案管**手法** (作者怎么推理、怎么停顿、情绪从哪来). 按维度分 docs 体 / 情绪手法 / blog 体, 每条带频率和两处原文出处, 外加 ai-docs 现存指纹的反例表. 知识稿默认 docs 体

**扩展方式**: 加一条手法之前先在 `docs/` 或 `blog/` 里找到两处原文; 找不到两处就不加. 只出现一次的是个例, 写进档案就会变成新的模仿指纹. 档案不进 lint: 它是启发, 不是判据 (理由同语料库, 见 voice-corpus-forced-injection 那篇 note)
