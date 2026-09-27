# 2 collect — 素材里有什么

## 只做这一件事

把素材变成**可引用的原始材料**。只采集, 不写文章、不建目录。

## 产物

写进暂存区 `ai-docs/.hx-staging/<slug>/source/`:

- `material.md` —— 整理后的要点, **每条带定位** (时间戳 / 小标题 / 函数名)
- `provenance.md` —— 来源类型、获取方式、获取时间、可信度、**已知缺口**

## 选哪条路径

`--kind` 决定。每条路径的完整做法在 `impl/`:

| `--kind` | 素材 | 实现 |
|---|---|---|
| `article` | 公开文章 / 网页 | `impl/article.md` |
| `video` | 视频 / 音频 / 字幕 | `impl/video.md` (先过 `shared/transcribe` 拿 transcript) |
| `local` | 本地代码变更 / git 历史 | `impl/local.md` |
| `research` | 多源调研 / 对比 | `impl/research.md` |
| `custom` | 登录墙后 / 需特殊解析 | `impl/custom.md` |

画像的数据源清单与扩展方式见 `impl/providers.md`。

## 这一阶段也在"学表述"

素材原文是人写的。看它怎么开场、怎么过渡、怎么下判断, 发现好的表达**当场记进语料库**,
否则读过就忘:

```bash
uv run .agents/skills/hx-note/scripts/hx_voice.py learn \
  --good "<原句>" --why "<好在哪>" --source "<素材标识>"
```

## 过关条件

`material.md` 里每个要点都能指回 `provenance.md` 里的来源。

## 扩展方式

新素材类型 = 在 `impl/` 加一个文档 + 在 `hx_flow.py` 的 `--kind` 加一个取值。
**步骤 3-9 完全不用改** —— 新类型只换采集方法。
