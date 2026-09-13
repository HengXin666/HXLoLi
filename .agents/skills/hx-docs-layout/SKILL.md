---
name: hx-docs-layout
description: "HXLoLi ai-docs 的内容布局与文风规范 —— 目录命名与分类、HXLoLiTag 词表汇总、双链与引用、开头反问引入/正文图文并茂/结尾升华展望的成文结构, 以及存量笔记批量体检 (结构/文风/标点/死链/tag/目录). 提供 HXLoLi-MD 语法速查与标点归一化脚本. Use when 需要决定笔记放哪个目录、怎么起标题与 tag、正文该按什么结构写、如何和已有笔记互链、PPT 侧车该放哪, 或要体检整理已有 ai-docs 全库."
---

# hx-docs-layout

规定 **HXLoLi ai-docs 一篇笔记长什么样**. 写正文、定目录、补 tag、做双链、体检存量时读它.

## 三条硬约束

1. **目录与命名**: 分类目录用 `NNN-中文名` 前缀 (如 `002-知识沉淀`); 每篇文章一个目录, 正文固定叫 `index.md`. 新建**分类目录**必须经人类同意; 在已有分类下建文章目录不需要.
2. **正文格式**: 严格遵守 [references/hxloli-md.md](references/hxloli-md.md) 的平台语法与标点约定. 标题编号见下. 定稿前跑标点检查脚本.
3. **注册到前端**: 新增/重命名/移动文章目录后, 必须在 `HXLoLi` 仓库根运行 sidebar 脚本, 否则侧边栏与搜索里不会出现. 只改正文则不必.

```bash
node scripts/generateAiDocsSidebar.js    # 新增/改名/移动目录后必跑
uv run .agents/skills/hx-docs-layout/scripts/format_cn_punct.py --check <note.md>   # 标点检查, 不合规 exit 1
```

## 章节骨架

主章节用十六进制序号:

```markdown
## 0x00 背景
## 0x01 核心结论
## 0x0A 参考来源
```

需要子项时改用中文嵌套 (两套不要混用):

```markdown
## 一、title1
### 1.1
#### 1.1.1
```

格式拿不准时, 参考 `docs/` 与 `ai-docs/` 下 `现代C++` 栏目的既有文章.

## 成文结构 (布局的核心)

1. **开头用引入而非摘要**: 一段设问/反常识切入, 把读者拉进问题; 引用块里放"所以我要回答什么".
2. **正文图文并茂**: 讲机制的段落配图/表/代码; 图优先走 `archify` 出可交互 HTML 侧车, 表格用于对照, 代码块给最小可运行片段. 纯文字长段落是排版事故.
3. **结尾拓展升华展望**: 收束到更大的一层 (这套机制会怎么演进 / 换场景意味着什么), 并明确区分**事实**与**个人判断**.
4. **引用来源固定放末章**: 只列本文真正引用过的文章/视频/网页 (标题 + 链接 + 一句出处价值), 不列本地路径与工具链.

细节与反例见 [references/authoring-style.md](references/authoring-style.md).

## HXLoLiTag (外挂注册表, 不要现编)

**tag 的唯一事实源是 `ai-docs/.hx-tags.toml`, 不在本文件里.** 该文件分两层:

- **curated 层** (人类维护): `[tags."<规范名>"]` 的 `desc` (这个 tag 管什么) 与 `aliases` (要合并进来的近义词).
- **generated 层** (脚本重建): 用法频次、内容指纹、待合并候选. 全部可全量重建, 不要手改.

### 沉淀时的动作顺序

```bash
# 1. 需要 tag 时, 先问注册表: 这个词该归到哪个已有 tag?
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py suggest "记忆架构"

# 2. 全库体检: 频次统计 + 待合并候选簇
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py scan

# 3. 内容变了 (新增/删除笔记或 tag) 后重建 generated 层
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py generate

# 4. 校验某篇/全库的 tag 是否规范 (非规范即 exit 1)
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py check <note.md>

# 5. 把别名统一成规范名 (默认 dry-run, 加 --write 落盘)
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py apply --write
```

rules:

- **优先复用, 不造近义词.** `suggest` 命中已有概念就用它; 都不合适才新增, 一次新增不超过 2 个.
- **合并近义词用 `merge`, 不要靠删改正文**: `hxloli_tags.py merge "记忆架构" --into "记忆系统"`, 然后 `apply --write` 批量改写.
- **`scan` 的候选簇只是字符层面的证据** (公共词缀/整词包含/字级相似), 语义近义 (如 记忆系统 vs 记忆架构) 脚本不猜, 由人判定后用 `merge` 固化.
- 新增规范 tag 时**同时**在 `desc` 里写清它管什么, 否则下一个沉淀的人分不清它和近义词的区别.
- 合理不需要 tag 的页面 (如 `001-关于`) 写进注册表的 `[settings] ignore_notes`, 不要为了凑规范硬加 tag.

### 注册表本身

- 位置: `ai-docs/.hx-tags.toml`, 随仓库一起版本管理; 隐藏文件不会被 sidebar 脚本与 Docusaurus 收录.
- 首次生成: `uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py init` (规范名取现有 tag, 别名留空, 由人类合并).
- 频率: 每次新增/合并 tag 后跑一次 `generate`; `check` 会对比内容指纹, 提醒你该重建了.

## 双链与引用

- 站内笔记互链用**相对链接**: 同分类下用 `[标题](../001-xxx/index.md)`, 跨分类用 `[标题](../../002-xxx/index.md)`.
- `docs/` 与 `ai-docs/` 是两棵独立文档树, 互链一样用相对路径.
- **链接语义要写清**: 说明那篇讲什么、与本文什么关系, 不要写"点击这里".
- 外链 (GitHub commit / 原文 / 视频) 放正文行内 + 末章汇总; GitHub 项目类笔记应关联对应 commit URL.
- 目录改名/移动会让相对链接失效, 必须同批修链.

## 存量体检

单篇笔记自身的合规体检 (文风/标点/侧车/双链/frontmatter) 走 [references/batch-audit.md](references/batch-audit.md) 的 8 项清单, 输出可执行的修复清单 (而不是只报问题).

**归类、身份与 tag 体系的整理不在这里** —— 重新分类、判断一个分类是否该重做、hxid 唯一 ID 与链接重算、tag 倒排树, 一律走 `hx-docs-organize`. 两套体检合起来才算完整.

## 参考文件

- [references/hxloli-md.md](references/hxloli-md.md) —— 什么时候读: 写正文前, 需要确认平台支持哪些语法 (Mermaid/公式/视频/图片参数/PPT 侧车) 与标点约定时.
- [references/authoring-style.md](references/authoring-style.md) —— 什么时候读: 需要写开头引入、布置图文、写结尾升华、定标题与 tag、区分读者可见与仅审核可见内容时.
- [references/batch-audit.md](references/batch-audit.md) —— 什么时候读: 要对存量 ai-docs 做全库体检或整理时.
- `scripts/format_cn_punct.py` —— 标点归一化/检查脚本. `--check` 只检查 (不合规 exit 1), `--diff` 先看差异, 无参数则原地改写.