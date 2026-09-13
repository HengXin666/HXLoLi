# 存量体检: 可执行的巡检清单

整理的第一步是**只读体检**, 产出一张表, 不要边扫边改. 每项都给可直接跑的命令与判据.

只读体检阶段**不写任何文件、不跑任何带 --write 的命令**; 基线提交在人类确认后才做, 见 [migration](migration.md) 的安全流程.

## 清单

| # | 检查项 | 命令 / 判据 | 档位 |
|---|---|---|---|
| 1 | **散装文件** | 根下不该有 md: 所有笔记必须在 `NNN-目录/index.md` | B |
| 2 | **命名合法** | 目录名不许含空格或等号 —— 用 `find ai-docs -mindepth 2 -type d` 配合 grep 筛出. 注意改名会改 URL, 见 [decision-matrix](decision-matrix.md) | A |
| 3 | **层级一致** | 先看深度分布 `find ai-docs -name index.md` 输出后按斜杠数统计; 再按深度排序定位. 同一分类下深度不一致就是归类的症状 | B |
| 4 | **空目录** | `find ai-docs -type d -empty` | A |
| 5 | **散装 md** | 文章目录内**允许**存在: index.md; .hx-mitemite.md (答题卡, 每篇最多一个); tag.json (侧边栏标签与图标, 机制来自 `generateAiDocsSidebar.js` 的 getJsonTagConfig, `docs/` 树在大量使用); 侧车资源 (.html/.tsx/.spec.json/图片); 以及 index.md 显式引用的同目录分片 md. 这些之外的散装 md 才是问题 | B |
| 6 | **frontmatter 完整性** | 每篇须有 hxid / title / created_at / model / skill / authors / tags; skill 必须是列表且**指向当前存在的 skill 名** (引用已删除的 skill 也算 FAIL). 非笔记页面 (如 001-关于) 允许无 frontmatter, 但要在注册表 settings.ignore_notes 登记, 并让 `hx_docs_id.py` 跳过 | A |
| 7 | **正文过程痕迹** | 搜 "AI 辅助沉淀" / "需要用户 review" / "TODO(仅 AI" —— 只应出现在 .hx-mitemite.md, 不进正文 | A |
| 8 | **tag 体系** | `hxloli_tags.py check --health` —— 看单次出现占比 / desc 覆盖率 / 目录名当 tag, 而不是只看 PASS | C |
| 9 | **hxid 覆盖** | (跑 hx_docs_id.py 的 check 子命令) —— 缺 ID、ID 重复、链接悬空、路径陈旧都会报出来. 口径: "可被跨文章引用".

```text
笔记页面   = index.md, 或没有同级 index.md 的孤立 .md (如 ai-docs/deck-embed-test.md)
内容分片   = 与 index.md 同目录的其它 .md  -> 不给 hxid, 它们随 index.md 一起移动
答题卡     = .hx-mitemite.md             -> 不是笔记, 不参与
```

| A |
| 10 | **分类自洽** | 逐个数每个二级分类下的 index.md 数量, 统计时排除非笔记条目与登记过豁免的页面; 只有 1 篇的分类提交人类复核 | C |

## tag 健康度阈值

`check --health` 会给出以下指标. 没有唯一正确的数字, 但跨越这些线就该动手:

| 指标 | 该警惕 | 说明 |
|---|---|---|
| 只出现 1 次的 tag 占比 | 超过 50% | 说明 tag 是标题碎片而不是概念 |
| curated desc 覆盖率 | 低于 50% | 站点标签页没有解释可展示 |
| 声明 parent 的占比 | 低于 30% | 树没有建立, 无法判断归属 |
| 单篇 tag 数 | 超过 6 或少于 3 | 过多说明没想清楚, 过少检索不到 |

阈值是判据不是目标, 不要为了让数字好看而删 tag.

## 脚本 exit code (CI 判断用)

| 命令 | 0 | 非 0 |
|---|---|---|
| assign | 无缺 ID 的笔记 | 1 = 存在格式非法或重复的 ID |
| check | ID 唯一 + 链接可达 + 路径最新 | 1 = 有错误 |
| resolve | 无需重算 | **2 = 有需要重算的链接 (dry-run 的正常结果, 不是失败)**; 1 = 索引不健康, 拒绝执行 |
| tags check | 名字全部合法 | 1 = 有非规范 tag |
| tags check --health | 同上 | **健康度判定为 FAIL 时仍返回 0** —— 它是诊断输出, 别拿它当 CI 闸门 |

## 输出格式

体检的产物是一张表, 不是一段散文. 每行必须能被独立执行:

```
文件                        问题                     建议动作          档位  风险
ai-docs/deck-embed-test.md  根下散装测试页            移出 ai-docs       B     低
005-Agent平台/              与 005-从打野 前缀重号    改为 006-          A     低
001-想法探索/               一级分类下只有 1 篇       升格 / 并入        C     中
```

## 与 hx-docs-layout 的分工

layout 的 batch-audit (8 项) 管的是**单篇笔记本身合不合规** —— 文风 / 标点 / PPT 侧车 / 双链 / frontmatter 字段. 本文件的 10 项管的是**这批笔记的归类与身份** —— 该放哪 / 叫什么 / 怎么被引用. **两份清单都要跑**, 覆盖不同, 不是同一份.

## 不归本 skill 管的检查项

PPT 侧车是否合规 (禁外部 CDN / 必须与 index.md 同目录 / 是否自包含) 由 hx-docs-ppt 与 layout 的 batch-audit 项 4 定义, **本 skill 不重复规定**. 整理时只需要注意一件事: 侧车必须跟着它的 index.md 一起移动, 一旦分家就失效.

## 收尾闸门

改完后逐条确认, 一条不落地写进交付说明:

```bash
cd HXLoLi
node scripts/generateAiDocsSidebar.js                                       # 1. 侧边栏重建, 且能搜到改名后的 id
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py check          # 2. hxid 唯一 + hxid 链接最新
uv run .agents/skills/hx-docs-organize/scripts/hx_docs_id.py links          # 3. 本地引用逐条可达 (含图片/侧车)
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py check --health  # 4. tag 规范 + 健康度
uv run .agents/skills/hx-docs-layout/scripts/format_cn_punct.py --check <md>  # 5. 标点
node scripts/generateAiDocsSidebar.js && node scripts/generate-deck-registry.mjs
node scripts/generate-docs-graph.mjs                                        # 6. 三个派生索引重建
npx docusaurus build                                                        # 7. 整站构建真的过
grep -rn <旧路径片段> ai-docs src scripts plugins                           # 8. 无写死引用残留
```

**为什么 2 和 3 都要跑**: `check` 校验的是 hxid 链接的**身份**(ID 唯一、没被改写、指向的笔记还在), 它**看不见普通相对路径**. 搬完目录后, 一条 `../005-旧名/index.md` 会静默失效 —— 站点照样构建、页面照样打开, 只有读者点那一下才发现 404. `links` 把所有本地引用真去磁盘上 walk 一遍, 并区分"引用"和"图片".

### 派生索引必须重建, 且容易漏

`sidebars.ts` / `sidebarsAiDocs.ts` / `data/aiDocTags.ts` / `src/hxdeck/decks.generated.ts` 都是**生成物且入库**. 改了目录或 tag 却忘记重建, 站点就会拿旧索引渲染: 侧边栏指向已经不存在的 id, 标签页显示的还是旧词表.

`static/docs-links-graph.json` 是 `.gitignore` 里的生成物, 由 `generate-docs-graph.mjs` 产出, 不进 git.

**踩过的坑**: tag 治理时只跑了 `hxloli_tags.py generate` 而没重建 `data/aiDocTags.ts`, 于是读衍生数据做判断时拿到的是**上一轮的 tag 集合**, 差点据此得出错误结论. 改完任何被索引的东西, 先重建再读.

最后交出**迁移映射表** (旧路径 -> 新路径, 一行一条, 含被改名的 URL) 与**跳过的项及原因**.
