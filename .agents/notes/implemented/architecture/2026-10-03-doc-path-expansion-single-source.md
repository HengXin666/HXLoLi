# Agent Note: 文风脚本的目录递归只留一份路径展开实现

Status: implemented

Decision-ID: doc-path-expansion-single-source

- **引入于**: <short-sha>
- **引用落点**: `.agents/skills/hx-note/scripts/lib/textpaths.py` 的 `expand_markdown()`  全仓唯一的路径展开实现

## Code

- `.agents/skills/hx-note/scripts/cli/textfmt/format_cn_punct.py`
- `.agents/skills/hx-note/scripts/cli/voice/hx_voice.py`
- `.agents/skills/hx-note/scripts/lib/textpaths.py`

## Problem

两个文风脚本 (`hx_voice.py` 查 AI 味与折行, `format_cn_punct.py` 归一标点) 只接受**单个文件**。
要给一个 skill 目录、一个 `ai-docs/` 子树做一次体检, 只能在外层套 `find | xargs`。
实测三种形态, 其中第二种最坏:

```
uv run hx_voice.py lint <dir>            -> exit 2, "error: 文件不存在: <dir>"
uv run hx_voice.py fix  <dir>            -> exit 0, "没有发现被折行的段落"   # 假阴性
uv run format_cn_punct.py --check <dir>  -> IsADirectoryError traceback, exit 1
```

`fix` 那一条比报错更危险: 它把「没扫任何文件」和「扫了但都干净」印成同一句话, 退出码还是 0。
拿它当门禁用, 会得到一个永远绿的检查。

同时发现一个**与目录无关的既有缺陷**: `guess_profile()` 用 `"/blog/" in path.as_posix()` 判 blog。
前导斜杠要求路径是绝对路径, 于是相对路径 `blog/2026/05/07/01_x.md` 被猜成 article, 换上更严的一套规则。
实测 42 篇 `blog/` 里 2 篇因此多报 E 级 `table` (手写随笔里有一张 2 行与一张 4 行的表格)。

## Decision

**路径展开只留一份实现**: `scripts/lib/textpaths.py` 的 `expand_markdown()`。
两个脚本都必须走它  理由不是省代码, 是这两个脚本默认扫的是同一批文件, 排除规则一旦各写一份,
「指一个目录递归跑」会给出互相矛盾的结论, 那比不支持目录更坏, 因为它看起来是对的。

一次调用接受三种形态, 可混用, 可重复传:

| 参数 | 行为 |
|---|---|
| 文件 | 原样收下 (存在性由调用方判断, 不吞掉 missing) |
| 目录 | 递归收 `*.md`, 路径排序, 跳过 `SKIP_DIR_NAMES` 与隐藏目录 |
| 含 `*?[` 的串 | 相对 cwd 用 `rglob` 展开 (兼容 `fix` 早就写着的通配符形态) |

三条附带的取舍, 都是实测逼出来的:

1. **隐藏目录默认不进。** `ai-docs/` 下 143 个 md 里有 73 个 (51%) 在 `.hx-staging/`,
   是过程产物。混进来会把递归变成噪音源。真要扫就把该目录自己当参数传 
   相对路径从它算起, 这条规则自然不生效。
2. **按实体去重。** 本仓 `.agents/skills/` 下大量是软链 (指向 `HXLoLi/.agents/skills/`),
   同时传目录与软链、或传父目录加子目录时, 同一份文件会被检查两遍, 报告里出现两条互相印证的命中。
   实测: `.agents/skills/hx-make-skill` 与它的绝对路径同传, 报告数 3 (不是 6)。
3. **`guess_profile` 改按路径段判。** `"blog" in path.parts`。

`lint` / `fix` / `--check` 的**单文件行为逐字节未变** (回归见 Testing)。

## Alternatives considered

- **什么都不做, 在外层套 `find . -name '*.md' | xargs`** — 最强理由: 零改动, 且这是 Unix 的正统做法,
  脚本保持「一个参数一个文件」的单纯语义。否决: `fix` 的假阴性正是这套用法的产物 
  管道在参数为空时不报错, 而 `format_cn_punct.py --check` 逐个文件退出码会被 `xargs` 压成最后一个;
  门禁需要的是「这次到底扫了几个文件、哪几个不合规」, 这只有在脚本内部知道。
- **两个脚本各写一份展开** — 理由: 免掉一个共享模块, 也免掉 `uv run <脚本路径>` 之外
  import 不进来的担心。否决: 排除规则分叉的代价是**静默的**  两个脚本对同一目录给出不同文件集,
  谁都不会报错, 只会让「标点全绿但 lint 报了一堆」这种无法解释的状态反复出现。
  实测 `uv run` 下 `sys.path[0]` 就是脚本所在目录, 兄弟 import 直接可用 (另加显式 `sys.path.insert`
  兜住 `python3 -c` 这类非标准入口)。
- **目录参数默认连隐藏目录一起递归, 用 `--no-hidden` 关掉** — 理由: 默认多扫不会漏东西,
  保守方向应该偏「多」而不是「少」。否决: `.hx-staging/` 的体量是 51%, 默认开启等于每次递归都先付一份
  噪声; 而「漏扫」的出口是显式传那个目录, 成本是零。
- **把 `find | xargs` 直接写进 skill 的文档, 不改脚本** — 理由: 文档里能写清楚, 也是最小侵入。
  否决: 上一条已经说明管道会吃掉退出码; 而且 `hx-note` 的调用点分散在
  `steps/3-atom`、`steps/6-derive`、`entries/organize` 与 pre-commit 钩子里,
  每处各写一条管道等于把同一份排除规则抄四遍。

## Consequences

- 「指一个目录递归跑」成为一等用法。实测规模: `ai-docs` 70 个 md 的 `--check` 0.08s、
  `lint` 0.19s; 整个 `.agents/skills` 88 个 md 一条命令出全量报告。
- 代价: 多一个模块, 且它必须被两个脚本同时引用  删掉任一处引用会让那一侧**静默**退回单文件语义。
  所以 `_textpaths.py` 的函数上带了指向本 note 的引用。
- 代价: 隐藏目录的排除是硬编码的目录名清单, 不是配置。清单改动要同时考虑两个脚本的默认扫描面。
- 顺带修掉的 `guess_profile` 会让 **2 篇 blog 少报 E 级 `table`** (篇幅各自一张表, blog profile 不禁表格)。
  这是修复而不是放宽: 那两张表在手写随笔里本来就合法。
- 未做: 没有给 `_textpaths.py` 加「配置文件里读排除名单」。理由: 排除项来自工具产物而非项目策略,
  现在没有第二处需要协商的对象。真出现分歧时再抽。

## Testing

单文件回归 (改动前脚本从 `git show HEAD:` 取出放在 `/tmp/hx-baseline` 对照):

```
lint 单文件 / lint --json / fix 单文件 / --check 单文件 / stdin 管道 / --check stdin / 不存在的路径
```

七项里六项逐字节一致; 唯一差异是 `blog/` 下的一篇  `profile=article` 变成 `profile=blog`,
即上面那条 `guess_profile` 修复的预期效果, 不是回归。

递归与去重:

```
uv run .agents/skills/hx-note/scripts/cli/voice/hx_voice.py lint .agents/skills/hx-make-skill          -> 3 个文件
uv run .agents/skills/hx-note/scripts/cli/voice/hx_voice.py lint '.agents/skills/hx-make-skill/**/*.md' -> 3 个文件
uv run .agents/skills/hx-note/scripts/cli/voice/hx_voice.py lint <目录> <同实体的绝对路径>              -> 3 个文件 (去重生效)
uv run .agents/skills/hx-note/scripts/cli/textfmt/format_cn_punct.py --check ai-docs                     -> 不含 .hx-staging (0 命中)
uv run .agents/skills/hx-note/scripts/cli/textfmt/format_cn_punct.py --check ai-docs/.hx-staging         -> 显式传隐藏目录仍可扫
```

一致性: 一次扫 `.agents/skills` 得 88 个文件, 逐个 skill 扫得 `14 + 12 + 7 + 3 + 52 = 88`, 相等。
