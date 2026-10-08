# Agent Note: skill 产物受自己那套文本与体量门禁约束, 站点标点规范仍不覆盖 skill

Status: implemented

Decision-ID: skill-artifacts-obey-prose-and-size-gates

- **引入于**: 本次改动 (待提交后回填该次提交的 sha)
- **引用落点**: `.agents/skills/hx-make-skill/scripts/prose_rules.py` 的 `scan()`, 与 `.agents/skills/hx-make-skill/scripts/check_layout.py` 的 `run()`

## Code

- `.agents/skills/hx-make-skill/scripts/prose_rules.py`
- `scripts/quality-gate.mjs`

## Problem

原先只有一条边界: **标点规范只约束站点内容 (ai-docs / blog / docs), 不施加于 skill 目录** (这条写在 `.agents/notes/archived/architecture/2026-10-03-punct-norm-scope-excludes-skills.md`, 已被本次取代). 它解决的是"别拿给人看的规范去批量改 vendored skill 正文", 但留下了第二个洞

**skill 没有任何自己的产物标准.** 结果是三类问题同时存在, 而都不是"某次疏忽"

1. **同一份 skill 的正文混用两套标点**, 或句末全角与半角并存. skill 的读者是模型, 全程零 markdown 渲染, 混用只会平白制造一个需要判断的歧义点
2. **中文正文被手工折行**. Markdown 的软换行在渲染时折叠成空格, 整段折行会让一段话读起来像被截断
3. **产物代码体量失控**. `hx-agent-notes/scripts/` 有 12 个文件, `hx-note/scripts/` 有 13 个且多个超过 300 行; `hx-code-quality` 与 `hx-ui-system` 的脚本还是 `.mjs`, 类型信息只覆盖一半

## Decision

给"写 skill 的 skill"配一套**自己的**产物标准, 并让新建的 skill 继承它

- **文本规范** (`hx-make-skill/references/prose-rules.md`): 只用英文标点; 标点后接内容时补一个空格; 正文行末只允许 `?`; 禁止把一句话拆到两行. 门禁 `scripts/prose_rules.py --check` 退出码 0 为过, `--fix` 就地改写
- **体量上限** (`hx-make-skill/references/code-quality.md`): 单个目录不超过 6 个文件 (C1); 单文件不超过 300 行 (C2); 源码只用 `.ts`, 禁止 `.js` / `.mjs` 且转译产出单独放 `dist/` (C3); 一段连续注释不超过 10 行 (C4). 门禁 `scripts/check_layout.py`. 目录确有需要时, 在目录内放 `layout.json` 声明 `maxFiles` / `maxLines` 与理由  豁免随目录走, 而不是留在评审者的记忆里
- **继承方式**: `hx-make-skill` 的 SKILL.md 把这两条写成硬约定, 并要求**新建的 skill 把同样的要求写进它自己的 SKILL.md**, 引用同一份规则与同一条命令, 不复制正文

边界因此变成两条并列的规范, 而不是一条

| 文件面 | 规范 | 门禁 |
|---|---|---|
| 站点内容 (`ai-docs` / `blog` / `docs`) | `hx-note/shared/hxloli-md.md` | `scripts/quality-gate.mjs` 的标点段 |
| skill 产物 (SKILL.md / references / 模板 / 生成的文档) | `hx-make-skill/references/prose-rules.md` | `prose_rules.py --check` |

两套规范**判据不同**: 站点面允许句末 `.`, skill 面在句末只允许 `?`. 所以站点门禁**仍然**不扫`.agents/skills/`  不是因为没有规范, 而是因为它拿的是另一套规范

## Alternatives considered

- **什么都不做, 保持"skill 无需任何一致性"** — 最强理由: skill 正文不渲染给人看, 标点差异不影响任何读者, 加门禁只是给未来的作者增加一次失败. 否决: 三个问题都不是审美问题  折行会让模型读到被截断的句子, 目录与文件体量决定模型能否在一次加载里看清一个模块, 而 `.mjs` 让类型检查对一半源码失明. 这些都是可判定的功能性缺陷, 且已在本仓的四个 skill 上各自发生过
- **把 skill 并入站点标点规范, 一套统一** — 最强理由: 一条规范比两条好维护, 实现成本是一行 glob. 否决: 判据本身不同  站点面按"给人读"标定, 允许句末 `.`; 若并入, 要么放弃 skill 面的句末约束, 要么让 3000+ 处 ai-docs 正文一起变红. 这个归档件的原 note 已经给出过第三条理由: 批量改写会让 vendored 的 `hx-archify` 变成带人工漂移的三方合并
- **只写规则文档, 不写可执行门禁** — 最强理由: 文本规范里"什么算够简洁"这类判断确实不该由脚本裁定, 而一个会误报的校验器只会训练所有人无视它. 否决: 本次四条文本规则与四条体量上限**全部可机械判定** (全角字符、标点后无空格、行末标点、段落续行、文件计数、行数、扩展名), 没有一条依赖语义判断; 语义部分仍然留在人手里, 由 `hx-agent-notes` 的语义自检清单覆盖
- **把存量 skill 一次性全部改造到合规** — 最强理由: 规则统一才有意义, 留一批不合规的等于规则无效. 否决: `hx-note` 的 `scripts/` 被 `HXLoLi/package.json`、CI、pre-commit hook 与三个子仓同时引用, 一次重构要把调用方路径全改掉; 而它当前的违规量 (1706 处) 说明那是一次独立工程, 不该混在定义规则的改动里. 本次只让 `hx-make-skill` 与 `hx-agent-notes` 自身达标, 存量 skill 的清单另行交付

## Consequences

- 两个 skill 达标: `hx-make-skill` 的 `scripts/` 由 1 份 343 行拆成 6 份 (最长 201 行); `hx-agent-notes` 的 `scripts/` 由 12 份平铺拆成 `cli/ 6 + lib/ 6 + authoring/ 4 + triage/ 1`, `references/` 由 9 份并为 4 份
- 代价一: 脚本路径变了. `HXLoLi/package.json`、`.github/workflows/agent-notes.yml`、三份 notes 契约文件与 `hx-note` 的 pre-commit 片段都已同步; **其余仓库内 vendored 的那份副本仍是旧布局**, 它们各自的 `package.json` 需要单独更新或重新跑一次安装器
- 代价二: `prose_rules.py` 的 `--fix` 会重排段落 (把折行合并回一行). 它是重写而非等价变换, 所以合并后必须逐块比对语义; 本次以"去掉空白与标点后逐字节相同"验证了两个 skill 的全部 markdown
- 代价三: 存量 skill 中 `hx-note` 已按本条规则收敛完毕, 见 `.agents/notes/implemented/process/2026-10-05-hx-note-scripts-converge-to-own-gates.md`; `hx-code-quality` 1111 处、`hx-ui-system` 250 处、`hx-archify` 1128 处仍是**接受**的状态, 按各自任务逐步收敛
- 未做: 没把 `prose_rules.py` / `check_layout.py` 接成仓库级 `package.json` 脚本, 也没接进 CI. 当前它们只按 skill 目录调用, 接线方式等存量收敛后再定

## Testing

- 文本门禁: `uv run .agents/skills/hx-make-skill/scripts/prose_rules.py --check <skill-dir>` 对两个 skill 均退出 0
- 反向探针 (判据真的会红): 全角句号、标点后无空格、行末 `.`、跨行折行四种输入各返回违规并退出 1; `--fix` 后重跑 `--check` 退出 0
- 误报对照 (不查的东西): 小数 `1.5`、文件名 `a.mjs`、范围 `01..03`、相对路径 `../assets/x.md`、行内代码与链接目标内的标点, 均不报 P2; 表格行与标题不参与行末判定
- 体量门禁: `uv run .agents/skills/hx-make-skill/scripts/check_layout.py hx-make-skill hx-agent-notes` 退出 0
- 注释块 (C4) 边界: 12 行连续注释判 FAIL, 10 行 PASS; 拆分过程中 `verify-backlinks.ts` 的 12 行文件头被压到 6 行
- 拆分后功能不变: `node .agents/skills/hx-agent-notes/scripts/cli/verify-all.ts` 的 tree / format / archive / coverage 四项与拆分前逐字同结论; 在临时仓库里跑 `authoring/init-agent-notes.ts` → `authoring/new-note.ts` → `cli/verify-all.ts` → `authoring/build-board.ts` 全链路成功, 板页产出 14537 字节
- `uv run scripts/validate_skill.py` 对两个 skill 均 PASS (0 errors, 0 warnings)
