# Agent Note: 标点门禁按文件与规则聚合输出, 不再一行一条

Status: implemented

Decision-ID: prose-gate-groups-violations-by-file-and-rule

- **引入于**: 本次改动 (待提交后回填该次提交的 sha)
- **引用落点**: `.agents/skills/hx-make-skill/scripts/prose_rules.py` 的报告循环, 与 `.agents/skills/hx-make-skill/scripts/prose_core.py` 的 `group_violations()`

## Code

- `.agents/skills/hx-make-skill/scripts/prose_core.py`

## Problem

门禁的读者是模型, 而模型的上下文按行计费. 旧输出对每条违规印一行:

```
x.md:12: P1 fullwidth ， -> ,
x.md:19: P1 fullwidth ， -> ,
x.md:31: P1 fullwidth ， -> ,
```

一个文件里同一规则命中 40 次就要付 40 行, 而它携带的事实只有一个: 这个文件有 40 处该规则违规, 在这些行上. 行号本身有用, 重复的规则名与修复提示没有

实测本仓存量 skill 的违规量是千级 (`hx-code-quality` 1111 处、`hx-archify` 1128 处, `hx-note` 收敛前 1706 处), 所以这不是理论问题: 一次 `--check` 的输出就能吃掉调用方相当一部分预算, 而调用方多半只需要知道"该去改哪几处"

## Decision

在 `prose_core.py` 加 `group_violations(names, rows)`, 把 `(文件, 行, 规则, 消息)` 折叠成每个文件一段、每个规则一行:

```
x.md
  P1 x5 @ 1, 2, 3, 4, 5
  P3 x1 @ 6
```

- 行号超过 12 个时截断并附 `(+n)`, 因为读的人要去的是文件而不是背下行号表
- 规则名与计数保留: 它决定**改成什么**, 是这次检查唯一不可省的字段
- 逐条消息 (`fullwidth ， -> ,` 这类) 被丢弃, 因为同一个规则名已经蕴含了它
- `group_violations` 放在 `prose_core.py` 而不是 `prose_rules.py`: 后者 272 行, 逼近 300 行上限; 而 `prose_core.py` 的既定职责就是"被门禁与修复器共用"

## Alternatives considered

- **什么都不做, 保持一行一条** — 最强理由: 逐条输出是唯一能按行号直接跳转的形态, 且现有调用方 (pre-commit、CI 日志) 可能按行解析. 否决: 本仓没有任何消费方解析这些行  `references/prose-rules.md` 与各 skill 的 SKILL.md 都只要求"退出码 0 为过", 说明输出是给人或模型看的, 不是给程序解析的; 而那些千级违规量恰恰证明逐条形态没人真读得完
- **加 `--verbose` 开关保留逐条, 默认聚合** — 最强理由: 两种需求都真实存在, 逐条对"修一处看一眼"的场景更方便. 否决: 多一个开关就多一条要在五个引用方 (各仓 SKILL.md) 同步的行为分支, 而聚合输出已经保留了全部行号, 需要逐条时按行号打开文件即可; 这个开关的收益撑不起它的同步成本
- **输出 JSON, 让调用方自己格式化** — 最强理由: 结构与展示分离, 最灵活. 否决: 观众是模型, 而模型的默认渲染是文本; 加一层 JSON 只让"人眼扫一遍"变难, 却没换到任何本仓在用的程序化消费
- **截断到只报前 N 个文件** — 最强理由: 违规面很大时, 只报规模最大的几个文件能更快收敛预算. 否决: 那会静默隐藏文件, 而"哪些文件该改"正是这次检查的输出本身; 按文件分组已经让每个文件只占一行开销, 不需要再丢文件

## Consequences

- 上下文开销随**文件数与规则种类**增长, 不再随违规条数增长. 实测夹具: 2 文件 / 9 处违规, 由 9 行降到 4 行; 单文件 40 处同规则则由 40 行降到 2 行
- 代价: 看不到逐条的原始消息. 判断"这一处的具体上下文"需要按行号打开文件, 但这本来也是修的正确姿势
- 落点选择把体量压力推给了 `prose_core.py` (176 -> 199 行), `prose_rules.py` 264 -> 272 行; 两者都仍在 300 行上限内

## Testing

- `uv run .agents/skills/hx-make-skill/scripts/check_layout.py .` 与 `validate_skill.py .` 均退出 0
- 回归: `prose_rules.py --check .agents/skills/hx-note` 仍退出 0 (干净目录行为不变)
- 正向夹具: 2 文件 9 处违规 -> 输出 4 行, 退出 1; 断言 P1 与 P3 各自聚合且行号升序
- 修复路径不变: `--fix` 对全角逗号夹具就地改写成功, 退出码仍为 0
- 门禁自检: `prose_rules.py --check .` 对 hx-make-skill 自身退出 0
