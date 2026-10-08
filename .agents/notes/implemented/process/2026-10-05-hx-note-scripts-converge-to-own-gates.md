# Agent Note: hx-note 的 scripts 收敛到自己的体量与文本门禁

Status: implemented

Decision-ID: hx-note-scripts-converge-to-own-gates

- **引入于**: 本次改动 (待提交后回填该次提交的 sha)
- **引用落点**: `.agents/skills/hx-note/scripts/cli/flow/flow_core.py` 的 `SCRIPTS_DIR` / `CLI_DIR` 常量块 (所有跨脚本路径的唯一真源), 与 `.agents/skills/hx-note/SKILL.md` 的 `## 脚本` 一节

## Code

- `.agents/skills/hx-note/scripts/cli/flow/flow_core.py`
- `scripts/quality-gate.mjs`

## Problem

`.agents/notes/implemented/process/2026-10-05-skill-artifacts-obey-prose-and-size-gates.md` 立了两条 skill 产物标准 (体量 `check_layout.py`, 文本 `prose_rules.py`), 并存量为接受状态. 该 note 明确记下 `hx-note` 的收敛是**一次独立工程**, 理由是它的 `scripts/` 被 `HXLoLi/package.json`、CI、pre-commit hook 与三个子仓同时引用, 一次重构要把调用方路径全改掉

本次就是那次工程. 实测的违规面是 19 处: `scripts/` 13 个文件 (限 6); 8 份超 300 行 (`hx_flow.py` 1081、`hx_voice.py` 856、`hxloli_tags.py` 752、`hx_look_video_prepare.py` 743、`hx_persona.py` 440、`hx_docs_id.py` 426、`makeDoc.py` 362、`hx_drawio.py` 301); `assets/` 与 `steps/2-collect/impl/` 各 7 个文件; 以及 8 处超 10 行的连续注释块

## Decision

把 `hx-note` 的脚本按**域**切成包, 每个域一个目录, 入口与兄弟模块分居: `scripts/lib/` 放跨域共享库 (`textpaths.py`), `scripts/cli/<域>/` 放入口与它私有的兄弟模块. 十个域是 `flow / voice / textfmt / taxonomy / identity / authoring / illustration / persona / mitemite / transcribe`

三条具体约定

1. **拆模块, 不压注释.** 超 300 行一律按职责切 (如 `hx_flow.py` 的 1081 行切成 `flow_core` 状态机 / `flow_read` 读数 / `flow_steps` 推进 / `flow_place` 落地 / `flow_doctor` 闸门 / `hx_flow` 入口). 同时把超 10 行的注释块压到 ≤10 行, 来龙去脉移进 Agent Note
2. **跨脚本路径收进唯一常量块.** `flow_core.py` 定义 `SCRIPTS_DIR` / `CLI_DIR` 与被调脚本的绝对路径 (`VOICE_CLI` / `PUNCT_CLI` / `TAGS_CLI` / `MAKEDOC_CLI` / `MITEMITE_RES_CLI`). 原先 `flow_doctor.py` 用 `Path(__file__).parent` 拼兄弟脚本名, 搬目录后四处检查会**静默全绿失败**  这个坑以前踩过一次, 常量集中后不可能再踩
3. **目录预算用 `layout.json` 声明而不是拆散.** `assets/` (7 个产出模板) 与 `steps/2-collect/impl/` (7 种素材来源各一份) 是结构性的, 拆开只会让作者拷贝时手工裁剪. 两处各放一份 `layout.json` 写明 `maxFiles` 与理由, 豁免随目录走

## Alternatives considered

- **什么都不做, 让 hx-note 继续作为"已接受的存量不合规"** — 最强理由: 它当前能跑, 而重构要把所有调用方路径一次性改掉, 风险与收益不成比例. 否决: 门禁存在的意义是让规则可执行, 留一个 13 文件 / 1081 行的反例在那里, 后来者只会读到"规则可以例外". 而且这次同时暴露了一个真缺陷 (见 Decision 第 2 条)  不重构就发现不了它
- **只拆目录, 不拆超长文件** — 最强理由: 目录问题是门禁真正的判据, 行数只是可读性. 否决: 1081 行的 `hx_flow.py` 混了状态机、读数、闸门三件事, 模型读一次拿不全, 人改一处要翻半天  这正是 C2 想防的. 而且压注释凑行数是门禁明令禁止的替代路径
- **把 13 个脚本合并成更少的文件以绕过 C1** — 最强理由: 文件数少更好一次看完. 否决: 那会造出更多超 300 行的文件, 直接把 C2 顶破, 是拿一个违规换另一个
- **给 `scripts/` 整体放一份 `layout.json` 声明豁免** — 最强理由: 改动量最小, 一行 JSON 解决 13 个文件的问题. 否决: 门禁的语义是"目录确需这么多文件时声明理由", 而 `scripts/` 并不需要  它只是没被分组. 用豁免掩盖结构问题会让 `layout.json` 这个机制本身贬值

## Consequences

- 门禁两条全绿: `check_layout.py` PASS, `prose_rules.py --check` exit 0; `validate_skill.py` PASS (0 errors, 0 warnings)
- 47 份模块, 单文件最长 286 行 (`flow_read.py`)  留了余量, 因为 300 是硬上限, 贴着写下一次小改就超
- 代价一: 11 个入口脚本路径全变. 已同步 `HXLoLi/scripts/quality-gate.mjs` (3 处调用)、`assets/pre-commit-note.sh` (1 个 `PY` 变量)、`SKILL.md` 的脚本索引表与 skill 内全部 md. 实测 `hx_flow.py doctor` 的「技能文档里的路径全部可达」一项从 6 条断裂降到 1 条 (**余下那条属于 `hx-make-skill/references/legacy-audit.md` 对 `hx-archify` 的引用, 是本次改动之前就存在的**)
- 代价二: `ai-docs/.hx-tags.toml` 的 generated 区块里嵌了一句带脚本路径的注释, 所以生成脚本改路径后该区块会"过期". 已就地重建该区块
- 代价三: 因为脚本是纯搬迁 (逐字节复制源码片段), 功能不变的验证用"同一命令对比新旧输出"而不是单元测试. 唯一有意的行为差异是报错信息里的自身路径 (从 `scripts/x.py` 变成 `scripts/cli/<域>/x.py`)
- 未做: 没有为这些脚本补测试. 它们当前的验证方式是"跑真命令对比输出", 机制上够用但不可回归

## Testing

- 体量门禁: `uv run .agents/skills/hx-make-skill/scripts/check_layout.py HXLoLi/.agents/skills/hx-note` → PASS
- 文本门禁: `uv run .agents/skills/hx-make-skill/scripts/prose_rules.py --check HXLoLi/.agents/skills/hx-note` → exit 0
- 结构门禁: `uv run .agents/skills/hx-make-skill/scripts/validate_skill.py HXLoLi/.agents/skills/hx-note` → PASS (0 errors, 0 warnings)
- 拆分未改语义: 47 份模块全部 `ast.parse` 通过; `pyflakes` 只剩 3 条**改动前就存在**于原文件的警告 (两处 f-string 缺占位符、一处未用局部变量)
- 功能不变 (18 条命令对比新旧输出): `hx_flow.py list / status / cover / brief`、`hx_voice.py lint (含触发 E 级的文件) / samples`、`format_cn_punct.py --check (单文件与目录)`、`hxloli_tags.py check / scan / suggest / generate`、`hx_docs_id.py check / links / index / resolve`、`review-paragraphs.py` 全部逐字节一致; `hx_flow.py doctor` 除已说明的既有断裂外一致
- 11 个入口脚本全部 `--help` 退出 0; `makeDoc.py` 固定 hxid 后产出与原版逐字节相同; `hx_drawio.py --demo` 产出 SVG 逐字节相同; `hx_look_video_prepare.py` 对本地 txt 与 srt 两种输入产出 transcript 逐字节相同
- 路径自查: 全仓扫描 `hx-note/scripts/...` 引用 76 处, 断裂 9 处**全部**位于 `ai-docs/.hx-staging/` 的过程产物与一篇 archived note (按仓库规则归档件不编辑)
