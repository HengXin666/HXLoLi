# Agent Note: 标点规范只约束站点内容, 不批量施加于 skill 目录

Status: implemented

Archived: 2026-10-05

- **引入于**: `<short-sha>`
- **引用落点**: `HXLoLi/scripts/quality-gate.mjs` 的 `MD_FILES` 声明处 (标点检查的扫描根)
- 影响: 无代码改动。本 note 固定 `format_cn_punct.py` 与 pre-commit 标点段的**适用边界**;
  `hx-make-skill` 自身的全角残留与折行在本次一并清理 (属内容自洽, 不是边界变化)

## Problem

`hx-make-skill` (一个"写 skill 的 skill") 自己的 SKILL.md 里**混用两套标点**:
行 24/25/34/35/55/56/57/59 用全角句号 `。`, 同一文件的其余 173 处句末用半角 `.`。
另外有 3 处中文正文被手工折行 (SKILL.md 2 处、spec-checklist.md 1 处)。

真正的问题不是"它不合规", 而是**没人能说清这类文件的合规标准是什么**:

- 站内标点规范写在 `hx-note` 的 `shared/hxloli-md.md`, 抬头是
  「HXLoLi 用户约定, **依据现有手写 ai-docs 观察**」 它的经验基础是**给人看的站点正文**。
- 而 skill 的读者是模型: DSH 的 loader 是 `readFile(utf8)` 后 `parsed.body.trim()`,
  全程零 markdown 渲染 (实测 `grep -c markdown dsh-skill-filesystem/lib/index.js` = 0)。
  Docusaurus 的 contentDir 只有 `ai-docs` / `blog` (见 `docusaurus.config.ts`), 不收 skill。

于是同一个"标点统一"的直觉, 在两类文件上的后果完全不同。混淆它们会出现两种错:

1. **把 ai-docs 的规范套到全部 skill 上**: 一次 `format_cn_punct.py .agents/skills` 会改掉
   `hx-kfx` 108 处、`hx-code-quality` 972 处、`hx-ui-system` 151 处全角标点 
   其中 `hx-archify` 是 vendored 上游 (`skill-release.json` 指向 tt-a1i/archify,
   带更新清单与 `.mjs` 校验脚本), 手改它的正文会让本地漂移无标记。
2. **反过来认为 skill 无需任何一致性**: 那么单个文件内混用两套标点就可以长期存在,
   而混用与"这个规范管不管我"无关  它是文档自身的缺陷。

## Decision

**两者分开处理, 边界写死:**

- **站内标点规范 (全角->半角 + 中文标点后补空格) 的适用范围 = 站点内容**,
  即 `docusaurus.config.ts` 收的三个 contentDir (`ai-docs` / `blog` / `docs`)。
  三道门禁现状已经如此, 本 note 把"现状"确认为"约定":
  `assets/pre-commit-note.sh` 第 1 段 `case "$f" in HXLoLi/ai-docs/*|ai-docs/*`,
  第 2a 段 `ai-docs/*.md|ai-docs/*/*.md|...`, `scripts/quality-gate.mjs` 的
  `MD_FILES = collectMarkdown(path.join(SITE, 'ai-docs'))`。
- **skill 只要求"单文件自洽"**, 不要求跨 skill 统一到某一种标点。
  `hx-make-skill` 的处置是就事论事: 它已 95% 半角, 就统一到半角
  (165 -> 161 行, 24k token 量级上 +1 token, 实测 token 中立)。
  这**不是**给全部 skill 定一个"必须半角"的规矩。

判据速记: **"这个文件会被渲染给人看吗"** 决定它是否受站内标点规范约束。
skill 的正文不渲染, 所以不受约束; 但它自己被同一个模型整篇读入, 混用两套标点会平白制造
一个需要判断的歧义点, 所以单文件自洽仍要保证。

## Alternatives considered

- **什么都不做, 让现状 (门禁只覆盖 ai-docs) 保持沉默** — 最强理由: 现状已经正确,
  写 note 只是把一件没人打算改的事变成文档负担, 而 AGENTS.md 也说笔记只在非平凡改动时写。
  否决: 现状正确是**巧合**, 不是决定。`hx-make-skill` 里那 10 处全角恰好在本次被发现并修正,
  下一个 agent 看到 `format_cn_punct.py --check <dir>` 现在支持递归了 (见
  `2026-10-03-doc-path-expansion-single-source.md`), 第一反应就是拿它扫全仓 skill 
  这个新能力把"误伤 vendored skill"从不可能变成一条命令。边界不写下来,
  新能力会自己把边界吃掉。
- **反过来: 把标点规范扩到全部 skill, 一次统一** — 理由: 一致比不一致好, 且实现成本是一行
  glob; `hx-make-skill` 既然修了, 别的 skill 没有理由不修。否决: 三处硬伤。
  (a) 经验基础错位  规范的阈值与规则来自"人读 ai-docs"的观察, 对不渲染的文本无限外推没有依据;
  (b) `hx-archify` 是 vendored, 正文改动会让上游同步变成带人工漂移的三方合并;
  (c) 收益不可测  skill 是喂给模型的, 半角/全角在 token 上实测中立 (+180 token 若全改全角,
  -1 若全改半角, 相对 24k 量级可忽略), 而风险 (误改 vendored / 误伤刻意排版) 是实的。
- **只修 `hx-make-skill`, 不写 note** — 理由: 改动落在 `**/*.md`, 按 coverage 的 exempt
  清单本就不需要 note, 写 note 是过度治理。否决: exempt 管的是"要不要为这次改动写 note",
  不管"这个边界以后还算不算数"。本次要固化的不是那 10 个字符, 是那条边界判断。
- **给 skill 目录也加一道"不得混用标点"的机械门禁** — 理由: 判据可以写得很硬
  (同一文件内 `。！？` 与 `.?!` 同时出现即 FAIL), 且能真正拦住下一个 `hx-make-skill`。
  否决(本次): 这是新行为, 需要先决定"哪些 skill 有权保持全角"(如可能存在的日文引用场景),
  而该判断目前没有实测数据支撑。先把边界写清楚, 等真的再出现一次混用再加判据。

## Consequences

- 站内标点规范的范围从"事实如此"变成"写下来的约定", 且写在了会改动它的那个文件旁边
  (`quality-gate.mjs` 的 `MD_FILES`), 而不是散在某个 skill 的参考文档里。
- `hx-make-skill` 现在是纯半角 (全角 0 / 半角 183), 折行 3 -> 0, body 157 -> 153 行。
- 代价一: 边界是**约定不是门禁**  没有任何脚本会阻止下一个人跑
  `format_cn_punct.py .agents/skills`。拦住他的只有这条 note 与被引用的那一行注释。
- 代价二: skill 之间标点继续不统一 (`hx-kfx` / `hx-note` / `hx-code-quality` / `hx-ui-system`
  仍混用, `hx-agent-notes` / `hx-archify` 纯半角)。这是**接受**的状态, 不是待办。
- 未做: 没给 skill 定义"该用半角还是全角"。本次统一 `hx-make-skill` 到半角只用了一条理由 
  它自己已经 95% 是半角, 取最小改动。别的 skill 要修时得独立论证, 不能引用本 note 当依据。

## Testing

边界的三处强制点 (逐条实读原文确认, 非推断):

```
.agents/skills/hx-note/assets/pre-commit-note.sh:76   HXLoLi/ai-docs/*|ai-docs/*) ;;
.agents/skills/hx-note/assets/pre-commit-note.sh:100  ai-docs/*.md|ai-docs/*/*.md|...) ;;
scripts/quality-gate.mjs:106                          const MD_FILES = collectMarkdown(path.join(SITE, 'ai-docs'));
```

`hx-make-skill` 修复后:

```
uv run .agents/skills/hx-make-skill/scripts/validate_skill.py .agents/skills/hx-make-skill  -> PASS (0 errors, 0 warnings)
uv run .agents/skills/hx-note/scripts/format_cn_punct.py --check .agents/skills/hx-make-skill -> exit 0
uv run .agents/skills/hx-note/scripts/hx_voice.py lint .agents/skills/hx-make-skill --profile article -> hard-wrap 3 -> 0
```

副作用为零的验证 (折行合并 + 标点归一化是对正文的重排, 必须证明不动语义):

- frontmatter 逐字节全等; 围栏代码块逐块全等 (SKILL.md 5 块 / spec-checklist.md 3 块 / patterns.md 6 块);
- 三份文件"归一化后渲染文本"与修复前逐字相同 (MarkdownIt 渲染后去标签 + 空白折叠);
- 唯一可观测变化是 `long-sentence` W 级提示 13 -> 15: `check_long_sentence` 是**逐行**切句,
  折行会把长句拆成两行从而遮蔽它, 合并后暴露出来。不是回归, 是判据恢复正常。
