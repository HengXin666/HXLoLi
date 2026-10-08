# Agent Note: 用户画像移入私有仓, 公开仓只留映射

Status: implemented

Decision-ID: persona-moved-to-private-repo

- **引入于**: `f28d9cfd13`
- 取代: 2026-09-24-sediment-two-artifacts-nine-stages.md 里「新增项目级 ai-docs/.hx-persona.md」这一条 (其余仍有效)

## Code

- `.agents/skills/hx-note/scripts/cli/persona/hx_persona.py`
- `scripts/setup-private.mjs`

## Problem

画像生成脚本把它写到 `ai-docs/.hx-persona.md`, 而 `ai-docs/` 属于**公开仓** (GitHub Pages 会发布)。
生成出来的内容里有:

- 「最近在做什么项目」 直接暴露当前工作方向;
- GitHub 活跃仓库列表, 含各仓库描述与**商业数字** (某个注册工具的单价、毛利率、单轮体积);
- 最近 git 提交历史。

这些对「给引入段挑切入点」有用, 但对公开访客是泄露。而画像只在本机写作流程里用,
没有任何理由进公开仓。

## Decision

画像文件移进私人仓 `HXLoLi-imouto/ai-docs/.hx-persona.md`, 公开仓里只留一个符号链接。

三处改动缺一不可:

1. **映射规则要放行隐藏文件.** `setup-private.mjs` 原本跳过所有点开头条目 (为避开 `.git` /
   `.DS_Store`), 而画像本身是点开头。改成白名单: 只有 `privateDotFiles` 里显式列出的才放行。
   **没有放宽全局过滤**  那会把噪声一起带进来。
2. **公开仓要忽略它.** 除了 `setup-private.mjs` 写的 `.git/info/exclude` (那是**本机专属**的,
   换机器 clone 就失效), 还必须在 `.gitignore` 里写一遍。一旦误提交, 私有内容进公开历史就删不干净。
3. **脚本要拒绝写错地方.** 没跑过 `setup-private.mjs` 时那个路径是空的  直接写就会在公开仓里
   **新建一个真文件**。所以落盘前加守卫: 目标是符号链接才写, 既不是链接又不存在就拒绝并提示。

## Alternatives considered

**什么都不做 / 复用现有。** 最强理由是无需新增实现和维护成本. 现有状态仍存在 Problem 中的具体缺口, 因此采用本记录的选择

- **只把敏感小节从画像里删掉, 文件仍留公开仓** — 最强理由: 不改机制, 改动最小, 而且「最近调研方向」
  本来也不敏感。否决: 敏感与不敏感的边界会变, 每加一个 provider 都要重新判断哪些行能公开;
  而漏判一次就是永久泄露。物理隔离比逐行审查可靠。
- **靠 `.git/info/exclude` 一条就够** — 理由: 已生效, 且不动仓库里任何文件。否决: 本机专属,
  换机器或换 clone 就失效  而那正是最容易误提交的时刻。
- **画像不落盘, 每次现算** — 理由: 不落盘就没有泄露面。否决: 构建要跑六个 provider (含 `gh api`
  与 git 历史), 每篇文章都重跑代价过高; 而且 `show` 要能读上一轮结论。

## Consequences

- 私有与公开的边界从「靠人逐行判断」变成「靠符号链接物理隔离」。
- 代价: 新机器必须先跑 `node scripts/setup-private.mjs`, 否则 `hx_persona.py build` 拒绝并报错退出 
  这是刻意的, 报错比静默写进公开仓好。
- 加新的私有数据源时, 若输出文件是点开头, 记得往 `privateDotFiles` 加一行。

## Testing

```
node scripts/setup-private.mjs                    # 应出现 ai-docs/.hx-persona.md 的映射
git check-ignore -v ai-docs/.hx-persona.md        # 应命中 .gitignore
uv run .agents/skills/hx-note/scripts/cli/persona/hx_persona.py build --refresh   # 正常; 删掉链接后应拒绝
uv run .agents/skills/hx-note/scripts/cli/persona/hx_persona.py show             # 仍能读到内容
```
