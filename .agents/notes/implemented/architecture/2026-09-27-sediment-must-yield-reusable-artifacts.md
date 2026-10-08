# Agent Note: 沉淀必须产出可复用物

Status: implemented

Decision-ID: sediment-must-yield-reusable-artifacts

- **引入于**: <short-sha>
- **引用落点**: 无源码引用 (约束的是 `.agents/skills/hx-note/` 的规范与 `hx_flow.py doctor` 的闸门)
- 取代: [2026-09-24-sediment-two-artifacts-nine-stages.md](../process/2026-09-24-sediment-two-artifacts-nine-stages.md)
  里"产出只有两个文件"那条**读法** (两个知识产物的原则保留, 但不再被读成"目录里只能有两个文件")

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

按九阶段流程产出的一篇笔记, 跑完 `doctor` 全绿, 读者读完却**拿不到任何能用的东西**。
实测到的四个独立症状:

1. `index.md` 里写满了"为什么好看"的规范, 却没有"用什么搭的"清单  读者知道好在哪, 搭不出来。
2. 四份 `.hx-info.md` 的围栏代码块计数全为 **0**: 事实源里一行代码都没有。
   模板 `assets/info-template.md` 只列 bullet 事实, 没有容纳可运行片段的位置。
3. 唯一做成可复用包的笔记 (`components/HX-VettaUI/`), 靠的是执行者自觉 
   规范与闸门里没有任何位置要求它, 所以那次成功不可复现。
4. 演示页与侧车 (`##PPT` 内联 / `#ppt` 侧车) 的信息只存在于 `impl/` 参考文档里,
   而**决定产物的两层  `templates/` 的槽位表与 `hx_flow.py` 的 `Stage.artifacts`  都没提它**。
   是否产出演示页完全取决于执行者有没有顺手翻那份参考文档。

第 4 点有一个自相矛盾的根源: `SKILL.md` 的红线写"产出只有两个文件, 不许留第三个",
而 `steps/9-land/impl/gates.md` 的产物树里自己列了 `*-deck.tsx` / `*-ppt.html`。
两处互斥时, 绝对的那句会赢  于是侧车被当成违规而不做。

## Decision

### 一、"两个产物"是一条**知识产物**上限, 不是目录里的文件数上限

`.hx-info.md` 是事实源, `index.md` 是阅读稿, 只此两份**知识产物**。
与 `index.md` **同目录**的侧车 (`*-deck.tsx` / `*.html` / `*.drawio.svg` / 复现件 / 契约件)
是笔记的一部分, 照常产出。红线按这个读法重写, 并明确写进 `gates.md` 的产物树。

### 二、可复用物由两件工具强制, 不靠文字要求

`hx_flow.py doctor` 新增两条检查:

| 检查 | 判据 | 拦什么 |
|---|---|---|
| 笔记里有可复用物 | 至少一处**带分组名的代码块** (```lang [组名-标题]`) 或一个**被正文引用的侧车** | 纯散文笔记 |
| 侧车全部被正文引用 | 同目录侧车文件名的出现次数 > 0 | 死侧车 (会被构建照常发布) |

两条都实测过会失败: 造一个含 `unused.html` 的目录 -> `侧车全部被正文引用` 报该文件;
删掉后转绿。`doctor` 从 14 项变成 16 项。

### 三、新增"可复用规格"写法, 结构照 skill 的组织方式

`steps/6-derive/impl/reusable-spec.md` 规定素材本身是"一套可照做的做法"时怎么讲, 分四层:

| 层 | 内容 | 为什么这一层不能省 |
|---|---|---|
| 触发场景 | 这套规格用在什么项目上 | 不知道何时用 = 永远不用 |
| 契约 | 换掉它整套做法就失效的东西, **每条带机械判据** | 没有判据的约束会被部分遵守且无反馈 |
| 约定 | 已替你定死的选择 (页面骨架 / UI 库 / 分层 / 状态放哪) | 不定死就等于每个项目重新选一遍 |
| 偏好 | 写法习惯 (格式化 / 类型 / 注释 / **每页独立 URL** / 路径别名) | 决定别人做出来的东西像不像 |

"每页必须有自己的 URL 路径"这类**结构偏好**也被收进规范, 并给出判据 (索引页面必须渲染一个
路由列表)。第五层是"可直接拷走的件": 契约件 / 复现件 / 演示页, 全部与 `index.md` 同目录。
配套给出填好的前端范例 `impl/reusable-spec-example-frontend.md` (取值全部取自本仓三个前端包的实测现状)。

### 四、默认模板补一个"可复用物"槽位, 事实源补一节可选章

- `templates/interest-outlook.md` 的槽位表加"可复用物", 并把判据从"有一个数字"提高到
  "读者能拿走一件东西"。
- `assets/info-template.md` 加"可复用件"章 (规格件 / 版本 / 依据), 只在素材是可照做做法时保留。
- `steps/3-atom/impl/rules.md` 澄清: 禁止配图的规则**只针对图片**, 代码块鼓励留在事实源里。

### 五、技能文档的路径检查扩到相对式

原来只认 `.agents/skills/...` 绝对式写法, 而实测改名后失效的引用**全部**是
`references/xxx.md` / `../shared/xxx.md` 这种相对式。扩到三种基准解析 (文件所在目录 /
skill 根 / 仓库根), 并豁免"行内 code 里的 markdown 链接" (那是规范文档在示范错误写法)。
上线后一轮就抓出并修掉了 8 处真实失效引用, 其中 3 处是本次自己引入的。

## Alternatives considered

- **什么都不做, 靠 SKILL.md 里加一句"请务必产出演示页"** — 最强理由: 零成本, 且不改任何脚本,
  而现状的产物本身是可用的。否决: 这正是第 4 点的病因  演示页的信息已经写在 `impl/tsx-deck.md` 里
  了, 问题从来不是"没写", 而是"写的位置不在决定产物的那一层"。再加一句同层的话, 结果一样。
  本仓已有实证: `impl/` 里那份演示页规范写得很完整, 而实测 `##PPT` 用量 2 处对 `#ppt` 14 处。
- **把"必须产出演示页"也写成 doctor 闸门** — 理由: 强制力最强, 且能立刻改变产出率。
  否决: 演示页只在"有值得分屏讲的结构"时才成立, 强制会让每篇都硬塞一个。改为闸门只要求
  "有一件可拿走的东西", 把"用哪一种"留给 `reusable-spec.md` 的选择指引。
- **给 HX-UI 补一个 `check-ui-rules.mjs`, 让组件包自己带上门禁** — 理由: 那是个真缺陷 
  `components/HX-UI/package.json` 引用的 `scripts/check-ui-rules.mjs` 不存在,
  `npm run build` 一直是失败的。否决(本次): 那属于组件包自己的修复, 与本条决策 (笔记规范)
  不同类; 本次只把它的**教训**写进 `reusable-spec-example-frontend.md` 当作反例
  ("门禁脚本必须真的在仓库里并被 scripts 引用")。
- **把可复用物要求下沉到 `.hx-info.md` 也强制** — 理由: 事实源才是唯一真相, 强制它能保证
  复用信息不丢。否决: 事实源面向检索, 强制塞可运行件会污染检索块; 且"读者能不能照着用"
  是阅读稿的问题。改为在事实源里**允许**但不强制 (info-template 的可选章)。

## Consequences

- 笔记的"参考价值"从**人品**变成**闸门**: 现在有两条机械检查兜住, 且实测会失败。
- 代价一: 纯讲解型笔记需要显式说明才能跳过可复用物闸门。这是刻意的摩擦 
  默认值必须是"产出东西", 跳过要留痕。
- 代价二: 相对式路径检查会有误报风险 (skill 文档里的路径写法本就混杂三种基准)。
  已通过三重豁免压制: 围栏内不管、行内 markdown 链接不管、`node_modules`/产物目录不管。
  剩余误报的代价是低 (报出来人看一眼就知道), 而漏报的代价是引用静默失效。
- 未做: `SKILL.md` 的 `description` 没有改。新能力 (可复用规格 / 照做型模板) 是否值得占用
  L1 预算**没有实测依据**, 而删掉已有触发词的风险更高。留给下一次有 forward-test 数据时再定。
- 下游一致: `shared/pipeline.md` 的产物表、收尾闸门与 `gates.md` 已同步; 三份文档里
  "13 项闸门"这类写死的计数已改成"以 doctor 实跑为准"。

## Testing

```
uv run .agents/skills/hx-make-skill/scripts/validate_skill.py .agents/skills/hx-note   -> PASS (0 errors, 0 warnings)
uv run .agents/skills/hx-note/scripts/cli/flow/hx_flow.py doctor --slug open-vetta-ui          -> PASS 16/16
uv run .agents/skills/hx-note/scripts/cli/flow/hx_flow.py doctor --slug spoken-style-corpus    -> PASS
```

闸门"真的会失败"的实证 (三条都用 fixture 实跑过):

```
_unorphan_ (含 unused.html 的目录)      -> ['unused.html']; 删除后 -> []
_broken_skill_paths (含 impl/missing.md) -> ['bad-skill/SKILL.md:1 -> impl/missing.md', ...]
                                         且正确豁免了行内 code 里的 markdown 链接反例
doctor 全量路径检查                      -> 上线首轮报出 8 处真实失效引用, 修完转绿
```

已知未通过项 (既有, 与本次改动无关): `verify-all.ts` 的 backlinks 检查 --
`.agents/notes/implemented/bug-fix/2026-09-27-del-inside-link-loses-strikethrough.md`
缺少 `引入于: <short-sha>`, 该文件在本次改动前就已是未追踪状态。
