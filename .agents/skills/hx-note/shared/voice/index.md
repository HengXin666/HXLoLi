# voice — 去 AI 味 (跨步骤能力)

**被步骤 3 / 6 / 7 / 8 共用. ** 它不属于任何单个步骤, 所以放在 `shared/`

## 三个动作

| 何时 | 做什么 | 命令 |
|---|---|---|
| 步骤 6 动笔前 | 读一遍已积累的好/坏表达 | `hx_voice.py samples` |
| 步骤 3 / 6 写完后 | 静态检测 | `hx_voice.py lint --profile atom\|article` |
| 步骤 7 盲审后 | 把新发现的表达写回语料库 | `hx_voice.py learn` |

## 产物

两个 profile, 分别对应两类产物: `--profile atom` 查 `.hx-info.md`, `--profile article` 查 `index.md`

## 实现

盲审协议 (污染控制三铁律、两类审的顺序、异源与顺序对调两条硬约束、什么时候可以跳过、**不要指望模型自我纠错**): `impl/blind-audit.md`. prompt 在 `assets/voice-audit-prompt.md` 与 `assets/fidelity-audit-prompt.md`

## 语料库

`ai-docs/.hx-voice.toml`. **扩展方式**: 新 AI 味规则 = 用 `hx_voice.py learn --bad` 追加一条(带 `--pattern` 会升级成 lint 的 E 级规则, 这才是让一次盲审变成永久拦截的开关)
