---
name: hx-docs-sediment
description: "把外部素材与本地变更沉淀成 HXLoLi ai-docs 笔记素材 —— B站/视频 (字幕或 ASR 转写 + 关键画面截图识别)、公开文章 HTML、需登录或特定解析方式的页面 (如 GPT 对话)、本地 code/git 变更经验、多源调研模式. 负责建文章目录并用 makeDoc.py 初始化 index.md 模板. Use when 要把一个链接/一篇文章/一段视频/一次代码改动/一次调研变成笔记初稿, 或用户说 `沉淀`, `沉淀这篇`, `把这个视频写成笔记`, `根据这次改动写篇笔记`."
---

# hx-docs-sediment

按素材类型选一条**解析路径**, 产出可引用的**素材与初稿**. 规定"怎么写好看"的是 hx-docs-layout; 规定"先跟人类谈成共识"的是 hx-docs-grill.

## 每次沉淀的固定契约

1. **先确认方向再动笔**: 目标/读者/边界没达成共识时, 先按 hx-docs-grill 谈. 素材齐全不等于方向明确, **禁止自作主张开始沉淀**.
2. **建目录**: `ai-docs/002-知识沉淀/<分类>/<子类>/NNN-标题/`. 目录命名、分类归属、新建**分类**目录是否允许, 一律按 hx-docs-layout. 注意: 这里只负责**新建**; 把已有笔记改归属/搬家/整理 tag 属于 hx-docs-organize, 不要在沉淀流程里顺手重构. 模板里的 `hxid` 由 makeDoc.py 自动生成, 不要手填.
3. **模板初始化**: 必须先跑 `makeDoc.py`, 禁止手建空 `index.md` 再手写.

   ```bash
   uv run .agents/skills/hx-docs-sediment/scripts/makeDoc.py \
     --title "标题" --tag "现代C++" \
     --skill hx-to-ai-docs --skill hx-docs-sediment \
     --output "ai-docs/002-知识沉淀/001-现代C++/001-日常探索/001-标题/index.md"
   ```

   - `--skill` 可重复或逗号分隔, 写入 frontmatter 的 `skill` YAML 列表; 不传时默认 `["hx-to-ai-docs"]`.
   - `--tag` 只填**注册表里的规范 tag** (`ai-docs/.hx-tags.toml`): 先用 `hxloli_tags.py suggest` 查, 不要现编近义词 (见 hx-docs-layout 的 HXLoLiTag 一节).
   - `--model` 写 frontmatter 的模型名. 能确定就显式传; 不确定时允许 `Unknown`, **禁止编造**当前会话模型.
   - 目标 `index.md` 已存在时脚本会拒绝覆盖 (除非 `--force`): 先读原文, 在其基础上编辑.
   - 生成后 AI 只替换/扩展 TODO 与章节内容, **不删** `created_at`/`model`/`skill`/`authors`/`tags` 字段.
4. **来源留痕**: 每个结论都要能指回来源 (视频时间戳 / 文章 URL / commit / 实测记录). 中间产物 (转写、截图、抓取原文) 不写进正文, 正文末章只列**外部来源**.
5. **收尾**: 定稿前按 hx-docs-layout 的成文结构与标点规范过一遍, 新增目录后跑 sidebar 脚本.

## 选路径

| 素材 | 读 | 一句话 |
|---|---|---|
| B站/YouTube/本地视频音频/字幕 | [references/video.md](references/video.md) | 转写成 transcript, 再按 transcript 写; 重点画面另做截图识别 |
| 公开文章 / 文档页 URL | [references/article.md](references/article.md) | 抓正文 → 提炼观点 → 保留出处 |
| 需登录或特殊解析的页面 (GPT 对话等) | [references/custom.md](references/custom.md) | 显式声明获取方式, 留 provenance, 不伪装成公开来源 |
| 本地代码 / git 变更 / 脚本配置 | [references/local.md](references/local.md) | 从 diff 与源码提炼经验, 结论必须有代码依据 |
| "调研一下业界怎么做" / 多源对比 | [references/research.md](references/research.md) | 多源交叉 + 来源账本, 先出结论表再沉淀 |

**视频类素材的第二步**: 只要用到口播内容, 都先交给 `hx-look-video` skill 拿 transcript, 不要自己凭标题或简介编内容.

## 全程红线

- **不臆造**: 查不到就说查不到; 无法确认的人名/术语/数字标 `[不确定]`.
- **不越权**: 目标没有明确、上下文不足时先问人类, 一次只问当前最关键的一个问题并附推荐答案 (按 hx-docs-grill).
- **不替人类拍板**: 方向与责任在人类, 你只是辅助.
- **不把过程写进正文**: 工具链、命令、本地路径、验证步骤、ASR 转写细节一律不进 `index.md`.

## 参考文件

- [references/video.md](references/video.md) —— 什么时候读: 素材是视频/音频/字幕, 或需要用视频关键画面做图文时.
- [references/article.md](references/article.md) —— 什么时候读: 素材是一篇公开文章或文档页 URL 时.
- [references/custom.md](references/custom.md) —— 什么时候读: 素材在登录墙后, 或需要特定解析方式 (如 GPT 对话) 时.
- [references/local.md](references/local.md) —— 什么时候读: 素材是本仓库/本机的代码变更、git 历史、脚本配置时.
- [references/research.md](references/research.md) —— 什么时候读: 任务是"调研/对比/看看别人怎么做", 需要多源交叉时.
- `scripts/makeDoc.py` —— 初始化 `index.md` 模板的脚本, 用法见上文契约第 3 条.
