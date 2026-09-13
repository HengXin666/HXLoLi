---
name: hx-docs-organize
description: "整理 HXLoLi ai-docs 存量笔记的归类、命名与身份: 判定一篇文章该聚合进已有分类还是独立新建分类、识别需要整体重做的分类、补 NNN- 前缀与去非法字符、分配与校验全局唯一 ID (hxid) 并把跨文章链接重算成移动后依然正确的相对路径、把 tag 从标题碎片重做成一棵大类到细分的倒排树 (编程语言 -> C++ -> 模板元编程, 而不是一上来就是偏特化), 以及整理后重建侧边栏、迁移映射表与收尾验证。Use when 需要重新分类/归档/整理已有的 ai-docs 目录, 或用户说 整理一下目录、重新分类、这个分类是不是该拆、笔记放错地方了、tag 太碎/太多近义、移动笔记后链接断了, 以及 hx-to-ai-docs 流水线进入存量巡检阶段时."
metadata:
  author: Heng_Xin
  version: "1.0"
---

# hx-docs-organize

把存量 ai-docs 从"能跑"整理到"能查". 面向**已有笔记**, 与面向新增的 `hx-to-ai-docs` 流水线互补.

## 三条硬约束

1. **先体检, 后动手.** 第一次接触一个库时, 只允许产出清单. 边扫边改会让人类失去复核的机会.
2. **结构变更归人类.** 新建/删除/重命名分类、跨分类搬文章, 一律先申请 (格式见 [decision-matrix](references/decision-matrix.md)). 卫生项可以自己做完再报告.
3. **动过目录就必须重算链接.** 每一批移动之后立刻跑 `hx_docs_id.py resolve --write`, 否则链接停在旧路径.

## 整理的三件事

| 事 | 判据 | 手段 |
|---|---|---|
| **归类** | 这篇文章与所在分类是同一主题吗? 分类名还准确吗? | [decision-matrix](references/decision-matrix.md) 的关联度矩阵 |
| **身份** | 移动/改名后, 别的笔记还能引用到它吗? | [migration](references/migration.md) 的 hxid 机制 |
| **可检索** | 读者想找"某一类知识"时, tag 能带他到吗? | [taxonomy](references/taxonomy.md) 的倒排树 |
| **树形** | 整棵目录树是同一维度切的吗? 深度一致吗? | [taxonomy-directory](references/taxonomy-directory.md) 的三层语义 |

## 自主权三档

| 档 | 内容 | 权限 |
|---|---|---|
| A 卫生项 | 补 `NNN-` 前缀、去空格/非法字符、删空目录、修断链、分配 hxid、同步 tag 注册表 | **直接做**, 做完报告 |
| B 分类内重排 | 文章改名、聚合到已有分类、拆篇/合篇、删无意义中间层 | **列清单待批** |
| C 结构变更 | 新建/删除/重命名分类、跨分类迁移、整类合并 | **必须逐条同意** |

判定依据与申请格式见 [decision-matrix](references/decision-matrix.md).

## 固定顺序

1. **只读体检**: 跑 [batch-audit](references/batch-audit.md) 的 10 项, 出一张"文件 / 问题 / 建议动作 / 档位 / 风险"表. 这一步**不写文件、不跑 --write、不提交**.
2. **提交清单**: A 档说明即动; B/C 档逐条申请. 一次提完, 不要挤牙膏.
3. **建基线**: 人类确认后、动第一刀之前, 再 `git add -A && git commit -m "chore: 整理前基线"`. 工作区已有未提交改动时先问人类这批改动要不要一并进基线.
4. **执行**: 先补身份 (`hx_docs_id.py assign --write`), 再动结构. 这样后续所有移动都被链接机制兜住.
5. **重算链接**: 每批移动后 `hx_docs_id.py resolve --write`.
6. **收尾**: 跑 [batch-audit](references/batch-audit.md) 的闸门 (侧边栏重建 / hxid check / tag check / 标点), 交迁移映射表.

## 脚本

```bash
# hxid: 分配 / 校验 / 映射 / 重算链接 (先 dry-run 再 --write)
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py assign
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py check
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py resolve

# 本地引用体检: 逐条去磁盘上走一遍 (普通相对路径 + 图片 + ppt 侧车)
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py links
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py links --show-orphans

# tag: 别只看 PASS, 要看健康度
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py check --health
```

`check` 在一个还没有 hxid 的库上会报满"缺 hxid", 这是正常的 —— 先 `assign` 看计划, 确认无误再 `assign --write`.

**`check` 与 `links` 管的不是一件事, 两个都要跑.** `check` 校验 hxid 链接的**身份** (ID 唯一 / 没被改写 / 目标笔记还在); `links` 校验所有本地引用**真的存在** (普通相对路径 / 图片 / `#ppt` 侧车). hxid 机制只保护"跨文章引用", 同目录侧车与普通相对路径它完全看不见 —— 搬完目录后 `../005-旧名/index.md` 会静默失效, 站点照样构建、页面照样打开, 只有读者点那一下才发现 404.

口径 (与侧边栏生成脚本一致): `index.md` 与**没有同级 index.md 的孤立 .md** 才是"笔记页面", 需要 hxid; 与 `index.md` 同目录的其它 md 是内容分片, `.hx-mitemite.md` 是答题卡, 都不给 ID. 非笔记页面 (如 `001-关于`) 通过 `ai-docs/.hx-id-ignore` 或环境变量 `HX_DOCS_ID_EXEMPT` 豁免.

## 引用形式怎么选

- **指向另一篇笔记**: 一律 `[标题](hxid:hx-xxxxxxxx)`, 不写相对路径. 相对路径会在那篇笔记改名/搬家时断掉.
- **指向同目录的非笔记资源** (图片、`#ppt` 侧车 `.html`/`.tsx`、`tag.json`): 用普通相对路径. 它们与 `index.md` 同目录、同生共死, 相对关系天然稳定.
- **指向站外**: 原始 URL, 不要缓存成相对路径.

## 参考文件

- [references/decision-matrix.md](references/decision-matrix.md) —— 什么时候读: 决定一篇文章该留、该聚合、还是该独立新建分类; 或需要判断整个分类是否该重做时.
- [references/migration.md](references/migration.md) —— 什么时候读: 要移动/改名目录, 或需要理解 hxid 唯一 ID 机制与链接重算时.
- [references/taxonomy.md](references/taxonomy.md) —— 什么时候读: 要整理 tag、判断某个 tag 该不该存在、或决定它的上级大类时.
- [references/taxonomy-directory.md](references/taxonomy-directory.md) —— 什么时候读: 要决定目录树的大类怎么切、层级该多深、某个分类是不是按错了维度分, 或判断整棵树是否需要重排时.
- [references/batch-audit.md](references/batch-audit.md) —— 什么时候读: 开始一次全库整理、产出体检清单、或收尾验证时.
- [scripts/hx_docs_id.py](scripts/hx_docs_id.py) —— hxid 分配/校验/索引/链接重算脚本 (无第三方依赖).
