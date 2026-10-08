# Agent Note: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织

Status: implemented

Decision-ID: skill-steps-as-template-method

- **引入于**: `f28d9cfd13`
- **引用落点**: 无源码引用 (约束的是 .agents/skills/ 的**目录组织方式**, 落在各 skill 的 markdown 结构里; 无对应代码声明)

## Code

- `.agents/skills/hx-note/scripts/cli/authoring/makeDoc.py`
- `.agents/skills/hx-note/scripts/cli/flow/hx_flow.py`
- `.agents/skills/hx-note/scripts/cli/identity/hxid_core.py`
- `.agents/skills/hx-note/scripts/cli/illustration/hx_drawio.py`
- `.agents/skills/hx-note/scripts/cli/mitemite/mitemite_add.py`
- `.agents/skills/hx-note/scripts/cli/persona/hx_persona.py`
- `.agents/skills/hx-note/scripts/cli/taxonomy/tag_apply.py`
- `.agents/skills/hx-note/scripts/cli/textfmt/punct_core.py`
- `.agents/skills/hx-note/scripts/cli/transcribe/transcribe_cli.py`
- `.agents/skills/hx-note/scripts/cli/voice/hx_voice.py`
- `.agents/skills/hx-note/scripts/lib/textpaths.py`

## Problem

hx-note 合并九个 skill 之后, 资源全部平铺, 出现两个问题:

1. **索引写法浪费.** SKILL.md 里 32 个引用全写成 `[references/x.md](references/x.md)`  给 AI 读的索引用不着可点击,
   括号里的路径把同一串字符重复一遍, 净多付 870 字符 (约 290 token), 零信息量。
2. **没有结构性扩展点.** collect / derive / organize 有子目录, 但 stages.md / gates.md / pipeline.md
   等散在顶层。**没有统一规则, 每加一样东西都要重新判断放哪**  这才是扩展性差的来源, 不是文件多。

## Decision

改用**模板方法模式**组织: 一个步骤 = 一个文件夹, 里面固定两样。

```
steps/<N>-<key>/
├── index.md     契约: 只做这一件事 / 产物 / 过关条件
└── impl/        可插拔实现: 这一步的多种做法各一个文件

entries/<名>/    独立入口 (不重叠于主流程的)
shared/          跨步骤能力 (不属于任何单步的)
```

- `index.md` 只写**契约** (步骤边界与过关条件), 不写具体做法  换实现不用动它。
- `impl/` 按场景分文件。加一种新做法 = 加一个文件, **契约与 SKILL.md 都不用改**。
- SKILL.md 退回**步骤表**: 每步一行, 说清「关心什么 / 契约在哪」。120 行、0 个 markdown 链接。

两条规则同时写进 hx-make-skill 的规范与校验器, 否则下一个 skill 会重复同样的错:

| 规则 | 校验等级 |
|---|---|
| 索引写裸路径, 不写「方括号路径 + 圆括号路径」那种写法 | WARN |
| 每个路径后必须跟一句描述, 不许平铺 | WARN |

校验时**跳过代码块与行内 code**  规范文档需要用它们示范错误写法, 那些不算违规。

## Alternatives considered

- **什么都不做, 保持平铺** — 最强理由: 文件已经能跑, 重组不改变行为, 而移动上百份文件有破坏引用的风险。
  否决: 扩展性差是**持续成本**, 每加一个素材类型都要重新判断归属; 风险是一次性的, 且校验器能兜住
  (实测重组后引用 0 处失效)。
- **只修链接写法, 不动目录结构** — 理由: 那 290 token 是唯一可量化的收益, 结构是主观的。
  否决: 链接写法只是症状。真正的问题是「没有统一规则 -> 每次临时判断」, 只修写法等于擦掉症状留着病因。
- **按类型分 (知识型/流程型/能力型各一套)** — 理由: 不同 skill 形态确实不同, 一套结构未必通用。
  否决: 实测只有流程型需要这种组织; 知识型本来就该是单文件, 强行统一会造出空目录。
  规范里写成「流程型 skill 用」而不是「所有 skill 用」。

## Consequences

- SKILL.md 165 -> 120 行; markdown 链接 32 -> 0; 引用字符 1612 -> 约 742。
- 扩展面从「改索引 + 改正文 + 改引用」收敛成「加一个文件」。
- 代价: 目录变深, 从 skill 根看一个 reference 要经过 steps/2-collect/impl/ 三段。这是刻意的 
  三段正好回答「哪一步 / 什么类型」, 比平铺时的文件名前缀更稳定。
- 校验器新增两条 WARN 后, 现有的 hx-agent-notes 与 hx-note 都被检出问题并已修;
  说明这两条规则此前**从未被遵守过**, 不是新增负担。

## Testing

```
uv run .agents/skills/hx-make-skill/scripts/validate_skill.py .agents/skills/hx-note        # PASS
uv run .agents/skills/hx-make-skill/scripts/validate_skill.py .agents/skills/hx-make-skill  # PASS
uv run .agents/skills/hx-make-skill/scripts/validate_skill.py .agents/skills/hx-agent-notes # PASS
```
