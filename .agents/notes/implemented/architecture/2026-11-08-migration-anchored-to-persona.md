# Agent Note: 迁移算子以作者画像为输入, done atom 查锚点

Status: implemented

Decision-ID: migration-anchored-to-persona

- **引入于**: 本次改动 (待提交后回填该次提交的 sha)
- **引用落点**: `flow_steps.py` 的 `persona_anchors` 与 `migration_gate`; `persona_providers.py` 的 `collect_git`

## Code

- `.agents/skills/hx-note/scripts/cli/flow/flow_steps.py`
- `.agents/skills/hx-note/scripts/cli/persona/persona_providers.py`

## Problem

发散算子 (见 divergence-operators-gate) 解决了"只会复述", 但迁移算子只说"搬到我自己的项目上", 没给它作者的项目是什么. 模型手里只有素材, 迁移就只能对着素材自己推, 产出「这套思路也能用在后端」这类空话, 或者迁到一个和作者无关的假想项目上. 读者读完拿不到和自己有关的东西

使用者 2026-11-02 的原话是「结合我当前最近做的一些业务的事情去写」. 画像 (`hx_persona.py`) 早就有「最近在做什么项目」「最近改了什么」, 但流程里没有任何一步要求迁移用它

顺带暴露两个画像的问题:

1. `collect_git` 取最近 40 条提交, 实测 20 条展示里 11 条是 `[LoLi-Bot] 同步 ANiMe` 与 merge, 人做的事被挤掉
2. `hx_persona.py build --out <新文件>` 被"映射未建立"守卫拦下: 守卫只该管默认路径, 显式 `--out` 到一个还不存在的文件也被拒

## Decision

1. `divergence.md` 算子三改为: 输入是 `hx_persona.py show`, 迁移目标写成画像里的条目原名, 并给检查动作. 画像里确实无关就写一行 `画像无关联: <原因>`, 不许硬凑. 模板 `assets/divergence-template.md` 同步加这一行
2. `done atom` 在原有"至少 4 条带判据产出"之后再查 `migration_gate`: 算子三一节里要么出现画像的某个条目名 (项目粗体名 / blog 与 ai-docs 标题 / 高频 tag), 要么有 `画像无关联:` 加至少 7 个字的原因. 缺画像文件时报错并指向 `hx_persona.py build`
3. `collect_git` 过滤 `[LoLi-Bot]` 与 merge 提交, 窗口从 40 条放到 120 条再过滤
4. `hx_persona.py` 的映射守卫只在没传 `--out` 时生效

## Alternatives considered

- **什么都不做 / 复用现有** — 最强理由: 画像已经在步骤 6 被引入段使用, 算子三的文字里也写了"我自己的项目", 模型理论上会去读. 否决: voice-corpus 与 human-review 两篇 note 都实测过"文档里写了, 没人照做"; 这里也一样, 现有发散记录里迁移目标没有一条是画像里的项目
- **把画像直接贴进 status 输出** — 最强理由: 和语料注入同一套"摆在眼前"的做法, 不依赖模型去跑命令. 否决: 画像含私有项目信息, 而且有 120 行, 每一步都贴会淹没当前阶段的指引; 只有步骤 3 需要它
- **要求每条迁移都落到画像** — 最强理由: 更强的约束, 更多作者语境. 否决: 很多素材 (例如选电脑包) 和作者的软件项目本来就无关, 强制全落会逼出硬凑的共鸣, 那比没有共鸣更伤 (SKILL.md 红线「不臆造」)
- **用 LLM 判断迁移是否相关** — 最强理由: 字符串匹配会被一个条目名糊弄过去. 否决: 需要额外调用与采样, 结果不稳定; 名字匹配已经能拦住"根本没看画像"这个主要失败, 糊弄留给步骤 8 保真盲审

## Consequences

- `done atom` 多一个拒绝原因; 纯资讯素材仍可 `--force` 并写原因
- 画像过期 (TTL 14 天) 时要先 build, 步骤 3 多一次约 20 秒的调用
- 名字匹配是弱判据: 只证明"看过画像", 不证明迁移有质量
- 画像的 git 节更能反映人做了什么, 但 120 条窗口在机器人提交极多的时期仍可能不够

## Testing

```
# 命中画像条目 -> 放行; 无命中 -> 拒绝; 写明无关联 -> 放行
printf '## 算子三: 迁移\n| 把规则搬到 HX-Git-DB | 查 x | y | z |\n' > /tmp/d1.md
printf '## 算子三: 迁移\n| 把规则搬到别处 | 查 x | y | z |\n' > /tmp/d2.md
printf '## 算子三: 迁移\n画像无关联: 画像全是软件项目, 电脑包与之无关\n' > /tmp/d3.md
# 对三份分别调用 flow_steps.migration_gate(Path(d), Path("ai-docs/.hx-persona.md"))
# -> d1 "" ; d2 报错 ; d3 ""

# 显式 --out 到新文件可写
touch /tmp/p.md && uv run .agents/skills/hx-note/scripts/cli/persona/hx_persona.py --out /tmp/p.md build --refresh
grep -c LoLi-Bot /tmp/p.md   # -> 0
```
