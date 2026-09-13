# Agent Note: 默认 exempt 不得抵消项目自己声明的 guarded

Status: implemented

- 影响: `.agents/skills/hx-agent-notes/scripts/notes-lib.ts` 的 config 合并, 以及每个仓库的 `.agents/notes.config.json`

## Problem

coverage 门禁判定受保护路径的规则是 `guarded 命中 and exempt 未命中`.
默认 config 的 `exempt` 里带着 `**/*.md` 与 `.agents/**` —— 这是为
"markdown 和 .agents 是文档、不是受保护源码"这个假设准备的.

只要一个项目把 `*.md` 或 `.agents/**` 写进 `guarded`,
它就同时命中默认 exempt, 于是**每一条自己刚声明要守护的路径都被豁免掉**.
外层 HXLoLis 仓库正是这样: `guarded: ["*.md"]` 而改 `README.md` 时门禁放行.
门禁打印的是 "no guarded source changed", 看起来像"这次改动不需要 note",
而真相是**这条门禁从未生效过** —— 比没有门禁更危险, 因为它会让人相信守护存在.

## Decision

合并 config 时做一次抵消检测: 当项目显式提供了 `coverage.guarded`,
就逐条检查默认 `exempt` 里的模式. 若某个 exempt 模式能匹配上某个 guarded 模式
所描述的路径形态, 说明该 exempt 会抵消这条守护, 于是**丢弃那条 exempt**.

判定用 `globToRegExp` 与一个把 `*/**/?` 替换成具体字符的 `samplePath` 来做,
所以 `**/*.md` 会被 `*.md` 判定为抵消, 而 `**/*.test.*` 对 `src/**` 不会.
只删除被抵消的那些, 其余默认豁免 (`**/*.test.*` 等) 原样保留.

## Alternatives considered

- **什么都不做, 让每个项目自己删掉冲突的 exempt** —— 这是原状.
  代价是每个采用者都要先踩一次"门禁看起来在跑其实从不触发", 而它失败时不报错、只沉默,
  正是最难被发现的一类问题. 这条假设由默认值引入, 就该由默认值收场.
- **把默认 exempt 整个清空** —— 会让所有项目都收到 `**/*.md` 这类噪音告警,
  因为绝大多数项目确实把 markdown 当文档. 默认值本身没错, 错的是它压过项目的显式声明.
- **改成 "guarded 优先, exempt 只对其他路径生效"** —— 等价于本方案, 但写成规则容易在
  两条配置各自命中时产生歧义; 在合并期直接剔除被抵消的模式, 读起来没有二义.
- **把冲突当作配置错误直接 exit 1** —— 会让所有沿用默认值的仓库立刻失败, 包括那些
  根本不想守护 md 的仓库. 静默剔除被抵消项既保住了守护, 又不打扰无关项目.

## Consequences

- 项目显式声明的 `guarded` 现在一定生效; 默认 `exempt` 只在不妨碍守护时才继承.
- 外层 HXLoLis 的 `coverage.exempt` 手写项已移除, 交回默认值 + 抵消过滤.
- 判定只看模式形态, 不看实际路径: 一个 guarded 是 `docs/*.md`、exempt 是 `**/*.md`
  时也会判定抵消并剔除 exempt —— 这是刻意的, 因为那条 exempt 确实会让 `docs/*.md` 永不触发.
- 若某个项目真的想让 md 既受守护又整体豁免, 应删掉 guarded 而不是保留一条永远不触发的守护.
