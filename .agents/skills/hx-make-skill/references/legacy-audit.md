# hx-* skill 的门禁状态

这份记录全部 `hx-*` skill 在两条门禁下的**当前**状态. 规则与判据: `prose-rules.md` (P1..P5) 与 `code-quality.md` (C1 目录文件数 / C2 单文件行数 / C3 `.js` `.mjs` 禁令 / C4 注释块长度). 判据与取舍的长篇理由见 `.agents/notes/implemented/process/2026-10-05-skill-artifacts-obey-prose-and-size-gates.md`

## 怎么复现

```bash
S=/home/hx/Loli/code/HXLoLis/.agents/skills/hx-make-skill
uv run $S/scripts/prose_rules.py --check --list <skill-dir>
uv run $S/scripts/check_layout.py <skill-dir>
```

## 当前状态

| skill | 文本规范 | 体量上限 | 备注 |
|---|---|---|---|
| `hx-make-skill` | PASS | PASS | 规则与两条门禁的真身 |
| `hx-agent-notes` | PASS | PASS | 六个子仓里的副本已同步到同一版本 |
| `hx-code-quality` | PASS | PASS | 两个 `.mjs` 已改写为 `.ts`; 14 份骨架用 `layout.json` 声明 |
| `hx-ui-system` | PASS | PASS | 两个 `.mjs` 已改写为 `.ts`; 组件目录用 `layout.json` 声明 |
| `hx-kfx` | PASS | PASS | `scripts/` 归入 `gates/` (4) 与 `tools/` (6) |
| `hx-note` | PASS | PASS | `scripts/` 47 个模块归入 `lib/` 与 `cli/<10 个域>/`, 最长 259 行; 8 处超长注释块已压 |
| `hx-anime-karaoke-ass` | PASS | PASS | `scripts/` 37 个文件归入 11 个职责组, `assfx/` 20 个模块归入 6 个子包 |
| `hx-archify` | VENDORED | VENDORED | 见下 |
| `hx-doc-sync` / `hx-memory-trigger` / `hx-record-session-replay` | PASS | PASS | 单文件 skill, 无需改动 |
| `hx-init` / `hx-test-pipeline` / `hx-skill-orchestrator` / `hx-libs-sentaku` | PASS | PASS | 同上 |

## vendored: hx-archify 不动

`hx-archify` 是上游 **tt-a1i/archify** 的 vendored 副本 (见它的 `skill-release.json` 与 `scripts/check-update.mjs` 的自更新契约). 它的 12 份正文与 39 个 `.mjs` 都按上游发布形态存在

**本仓不改它的文本与模块形态**: 改了会让每次上游同步变成带人工漂移的三方合并, 且 `check-update` 的
哈希校验会失败. 这条豁免不是静默放过  它在 skill 根目录的 `layout.json` 里声明为`{"vendored": true, "reason": "..."}`, 两条门禁都会打印一行 `VENDORED ...` 说明该 skill 被考虑过并刻意豁免. 要改就改上游发布物

## 目录确需超 6 个文件时

在**那个目录里**放 `layout.json` 声明 `maxFiles` / `maxLines` / `maxCommentBlock` 与 `reason`; 豁免随目录走, 别人 clone 到也能复现 (实测: 无声明时 8 文件判 FAIL, 加声明后 PASS). 写在 skill 根目录的声明会被其下所有目录继承

## 三条不可以顺手做的改动

- **`hx-archify`**: 见上, vendored
- **`hx-note/scripts` 被跨仓引用**: `HXLoLi/scripts/quality-gate.mjs`、`assets/pre-commit-note.sh`、以及 skill 内数十处 md 都指向具体脚本路径. 重构必须同一次改动里改全部调用方
- **`hx-anime-karaoke-ass` 的工作区很脏** (`HXLoLi-Music` 有 290 项未提交改动): 改动前先确认不与他人的在途改动冲突

## 三种声明 (不是静默放过)

- **目录确需超 6 个文件**: 在**那个目录里**放 `layout.json` 声明 `maxFiles` / `maxLines` / `maxCommentBlock` 与 `reason`. 豁免随目录走, 别人 clone 到也能复现. 写在 skill 根目录的会被其下所有目录继承
- **上游拥有布局的 vendored skill**: 在根目录写 `{"vendored": true, "reason": "..."}`. 两条门禁都会打印一行 `VENDORED ...` 并跳过它
- **哈希冻结的原创快照**: 在目录里写 `{"frozen": true, "reason": "..."}`. `hx-code-quality/assets/source/` 就是这种  四份原件与 14 份骨架的 sha256 由 `scripts/verify-templates.ts` 互相核对, 改一个字会让骨架校验失败

判据: **这份文本/布局归谁改?** 归上游或归哈希就声明, 归本仓就改
