# Agent Note: model 字段改为登记表枚举, 不再从配置文件推断

Status: implemented

Decision-ID: model-field-enum

- **引入于**: 本次改动 (待提交后回填该次提交的 sha)
- **引用落点**: `makedoc_meta.py` 的 `load_model_registry`; `flow_doctor.py` 的 `_model_value`

## Code

- `.agents/skills/hx-note/scripts/cli/authoring/makedoc_meta.py`
- `.agents/skills/hx-note/scripts/cli/flow/flow_doctor.py`

## Problem

使用者的抱怨是"不同模型写出来质量参差", 但 2026-11-08 统计 45 篇 ai-docs 的 `model` 字段, 有 12 种写法, 其中 14 篇是 `Unknown`, 同一个模型还有 `DeepSeek-V4-Flash` 与 `deepseek-v4-flash` 两种大小写. 按模型分组比较质量做不到

另外 `get_project_model()` 在环境变量缺失时会去读一个 `config.toml`, 而 `model-selection.md` 明文禁止从配置文件推断模型. 那个路径还被一次批量改名改坏成 `hx-note 的步骤 2/config.toml`, 实际永远读不到, 结果总是 `Unknown`

## Decision

1. 新增 `shared/models.toml`: 每个模型一个规范 id 加别名; `codex` 与 `unknown` 标 `legacy = true`, 只给存量用. 推理强度后缀不用登记
2. `makedoc_meta.normalize_model()` 按登记表归一; `makeDoc.py` 写入前归一, 表外取值打警告并原样写
3. 删掉 `config.toml` 回退; 环境变量读不到就写 `unknown`, 交给 doctor 拦
4. `doctor` 新增一项: `model` 必须是登记表里的规范 id, 且不是 legacy
5. 存量 45 篇按别名归一 (只改写法, 不改含义; `Unknown` 归成 `unknown`, `Codex` 归成 `codex`)

## Alternatives considered

- **什么都不做 / 复用现有** — 最强理由: `model` 只是元数据, 不影响生成质量, `model-selection.md` 也已经写了"如实填写". 否决: 写了如实填写, 结果仍有 14 篇 Unknown 和 12 种写法; 要回答"换模型就变"这个问题, 必须先能按模型分组
- **自由文本 + 大小写归一** — 最强理由: 不用维护登记表, 新模型直接能写. 否决: 归一只解决大小写, 解决不了 `GPT-5 Codex` 与 `gpt-5-codex`、`Codex` 这类产品名与型号混写
- **把存量 Unknown 按 created_at 与当时常用模型反推** — 最强理由: 能救回 14 篇的分组数据. 否决: 这正是 model-selection.md 禁止的"编一个看起来对的模型名", 会被当真

## Consequences

- 新模型要先登记一行才能写; doctor 会拦住忘了登记的情况
- 存量 14 篇 `unknown` 与 2 篇 `codex` 仍无法分组, 只能从对照实验开始积累可比数据
- 走 Hark 这类路由平台时只能记到 `hark`, 拿不到底层型号

## Testing

```
python3 -c "import sys; sys.path[:0]=['.agents/skills/hx-note/scripts/cli']; \
from authoring.makedoc_meta import normalize_model as n; \
print(n('DeepSeek-V4-Flash'), n('gpt-5.5-xhigh'), n('Unknown'), n('foo'))"
# -> ('deepseek-v4-flash', False) ('gpt-5.5-xhigh', False) ('unknown', True) None

grep -rh '^model:' ai-docs --include=index.md | sort | uniq -c
# -> 只剩登记表里的 id
```
