# 迁移协议: hxid 唯一 ID 与安全执行

目录一动, 相对路径就断. 本文件规定怎么让链接在整理后依然正确, 以及整理本身的执行与验证.

## 一、问题

ai-docs 的跨文章引用现在写成 `[标题](../002-乙/index.md)`. 一旦 `002-乙` 改名或被移到别的分类, 这条链接就是死链. 站内没有 URL 重定向机制 (`docusaurus.config.ts` 未配置 `createRedirects`), 改名即换 URL.

**每一篇笔记各自占一个目录** (`NNN-标题/index.md`), 所以"同一目录内的笔记互相引用"这种情形并不存在. 指向另一篇笔记的路径, 只要那篇改名或被移走就会断; 而同目录的图片/侧车随目录一起走, 相对关系天然稳定.

## 二、解法: hxid 全局唯一 ID

每篇笔记在创建时分配一个永久不变的 ID, **写在 frontmatter 里**, 跨文章引用指向 ID 而非路径.

```yaml
---
hxid: "hx-3f9a2c71"
title: "..."
---
```

链接的两种形态:

```markdown
[标题](hxid:hx-3f9a2c71)                              <- 源码形态 (人写这个)
[标题](../002-乙/index.md "hxid:hx-3f9a2c71")          <- resolve 之后 (可渲染)
```

第二种形态**同时携带可渲染路径和持久身份**: 读者点得到, 脚本认得出. 所以 `resolve` 可以反复重跑, 每次把路径重算到最新, 而 ID 永不丢失.

## 三、脚本

```bash
# 1. 为缺 ID 的笔记分配 (先看计划, 再加 --write)
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py assign
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py assign --write

# 2. 校验: ID 唯一/合法 + 链接可达 + 路径是否已陈旧
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py check

# 3. 查看映射
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py index

# 4. 目录移动之后: 重算正文里的 hxid 链接 (先 dry-run)
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py resolve
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py resolve --write
```

**任何移动/改名操作之后都必须跑 `resolve --write`**, 否则链接停留在旧路径. `check` 会把陈旧路径报出来 (`X 应为 Y`), 这是它的主要价值.

## 三点五、不变性是怎么被保证的

ID 只在两个地方可能被改写, 两处都已封死:

| 风险 | 封堵方式 |
|---|---|
| 创建脚本覆盖已有笔记 (`makeDoc.py --force`) | 覆盖前先从旧文件头部读回 hxid 并沿用, 绝不新生成 |
| 有人手工改 frontmatter | `snapshot --check` 对比快照, ID 变了立即报 [GONE] |

解析优先级 (makeDoc.py): 显式 `--hxid` > 目标文件里已有的 hxid > 新生成. 后两者保证重跑创建命令不会换 ID.

### 证明 ID 没有变过

`snapshot` 把当前的 ID -> 文件位置 映射存成 `.hx-id-snapshot.json` (隐藏文件, 不进正文也不进侧边栏). 之后任何一次比对都能区分三种情况:

- `[MOVED]` —— ID 不变、位置变了. 这是**正常**的, 说明重命名/搬家成功且身份保住了.
- `[GONE]` —— 某个 ID 从库里消失. **这是警报**: 要么笔记被删, 要么 ID 被改写 (表现为 [NEW] 新 ID 与 [GONE] 旧 ID 同时出现).
- `[NEW]` —— 新出现的 ID, 正常对应新笔记.

```bash
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py snapshot          # 建基线快照
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py snapshot --check  # 改动前后比对
```

整理前后各跑一次 `snapshot --check`, 输出里**只应出现 [MOVED] 与 [NEW], 不应出现 [GONE]** —— 这是整理没有破坏身份的机器证明, 比口头保证可靠.
## 四、写入时机

新笔记的 hxid 由 `hx-docs-sediment/scripts/makeDoc.py` 在创建模板时生成, 不靠事后补. 整理存量时才用 `assign` 批量补齐.

## 五、单次整理的安全流程

1. **只读体检**: 按 [batch-audit](batch-audit.md) 的 10 项, 产出一张表 (文件 / 问题 / 建议动作 / 档位 / 风险). 这一步**不写任何文件、不跑任何 --write**. **不要边扫边改.**
2. **建基线**: 体检结果拿到人类确认、**确定要动手之后**, 在第一批改动之前建基线 —— `git add -A && git commit -m "chore: 整理前基线"`, 或用 `git stash create` 记一个 hash. 工作区已有未提交改动时**先问人类**这批改动要不要一并进基线, 不要替他决定.
3. **提交清单**: 档 A 说明即可动; 档 B/C 逐条列给人类.
4. **执行**: 按"先难后易"或"先易后难"无所谓, 但**同一批移动必须紧跟一次 `resolve --write`**.
5. **收尾闸门** (逐条过):
   - `node scripts/generateAiDocsSidebar.js` 跑过, 且 `sidebarsAiDocs.ts` 里能搜到改名后的新 id
   - `hx_docs_id.py check` 通过
   - `uv run .agents/skills/hx-docs-layout/scripts/format_cn_punct.py --check <改过的 md>` 通过
   - `hxloli_tags.py check` 通过, 且整理过的分类补了 curated `desc`
   - 全库 grep 一遍被移动过的路径字符串, 确认没有写死的引用残留
6. **交迁移映射表**: 旧路径 -> 新路径, 一行一条. 这是人类复核的唯一依据, 不要省略.

## 六、写死路径的例外

代码里可能写死了 ai-docs 路径 (当前只有一处: `src/hxdeck/decks.generated.ts`, 由 `scripts/generate-deck-registry.mjs` 生成). 移动相关笔记后必须重跑对应的生成脚本, 而不是手改生成物.