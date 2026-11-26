---
name: hx-ai-docs
description: "HXLoLi ai-docs 写作流程: 先找出让细节都能推出来的一个核心模型, 再把每个特色机制按 翻车场景, 朴素做法, 为什么坏, 怎么修 的顺序写, 配 KBNote 组件和结构 SVG 图, 最后过 check-kb-note 与真实渲染截图. Use when 用户说 写一篇 ai-docs, 沉淀到 ai-docs, 把这个项目/文章/视频整理成知识页, 重写或精简某篇 ai-docs 笔记, 讲清某个工具为什么好用或怎么实现的时候."
license: MIT
disable-model-invocation: true
metadata:
  author: Heng_Xin
  version: "1.1"
---

# hx-ai-docs

一篇 ai-docs 是一个 `index.md` (mdx 格式, 用 KBNote 组件) 加同目录的结构图 SVG. 不再有 .hx-info.md 和可视化侧车

范文: `ai-docs/003-AI/003-Agent-Harness/001-oh-my-pi原生调试与代码审查/index.md`, 照它的密度和节奏写

## 为什么这样写

笔记的毛病是看过就忘, 没有一条能用下来. 2026-11-22 重写 oh-my-pi 那篇时连着失败了三次, 每次都是一种典型病

- 功能清单: 列出它能做什么, 读者记不住也用不上
- 文档摘抄: 函数名和毫秒数堆在一起, 没有一个能把它们串起来的图景
- 一节一句: 删到只剩结论, 读起来像敷衍, 看过去一点印象都没有

能留下来的是两样东西: 一个核心模型 (记住它, 细节能自己推出来), 和一个个具体的翻车场景 (读者见过这种坑, 才记得住为什么要这样修). 每节都从场景讲到修法, 配一张图或一个组件让结构一眼看清

## 四步

每步一个文件夹, `index.md` 是契约 (只做什么 / 产物 / 过关条件), 做法在 `impl/`

- `steps/1-model/index.md`: 读一手来源, 找出核心模型和要讲的几个机制, 每个机制配一个翻车场景. 这一步不写正文
- `steps/2-write/index.md`: 按骨架写 index.md, 每节 场景 -> 朴素做法 -> 为什么坏 -> 怎么修. 组件用法看 `HXLoLi/src/components/KBNote/CATALOG.md`
- `steps/3-figures/index.md`: 画结构图. 用 `scripts/svgfig.py` 生成同目录 SVG, 只画真结构
- `steps/4-verify/index.md`: 跑门禁, 在用户的 dev server 上实际渲染, 截下每张图和组件, 亲眼看过再交

## 红线

- **先有模型再动笔.** 说不出 "先记住一件事" 是什么, 就回到第一步
- **每节都有场景.** 一节只有标题加一句话, 等于没写. 场景要具体到命令, 代码或数字
- **只讲特色机制.** 别家也有的功能不讲, 能从核心模型推出来的细节不单独列
- **数字只留承载设计决定的.** 例如 "诊断等 500 ms 再转后台" 要留, 内部重试次数不留
- **不编.** 机制和数字来自固定提交的源码或文档, 链接带提交号. 示意性的例子要写明是示意, 拿不准的标 [不确定]
- **图是真结构.** 节点和边对应真实的模块, 调用, 时序. 没有结构可画就用 Compare 或 Flow, 都不合适就不配
- **不放 Markdown 表格, 不写大段.** 一段不超过 100 字
- **落位先看邻居.** 建目录前先 `ls` 目标分类. 同一主题会连着读的笔记 (含这次要写的) 到 2 篇以上, 就收进一个子目录, 例如 `001-购物/002-手机选购/` 下放指标篇和各期横评. 已有笔记跟着搬, 搬完跑 `hx_docs_id.py resolve --write` 和侧边栏重建. 判据见 `hx-note/entries/organize/impl/decision-matrix.md`. 新建一级或二级分类仍要用户同意
- **要换整体写法先出样本.** 写法由用户定, 改之前先拿一节样稿给用户看

## 脚本

- `scripts/svgfig.py`: 结构图生成库. 用 Python 写节点, 连线, 色带, 输出套用 kb-note.css 类名的 SVG. 用法见 `steps/3-figures/index.md`
- `scripts/render_check.ts`: 打开用户 dev server 上的笔记页, 报渲染错误, 并把每张图和组件截成 PNG. `node scripts/render_check.ts <url> <输出前缀> [dark|light]`

## 参考

- `references/anti-patterns.md`: 三种失败写法的片段和病因, 自查时读
- `HXLoLi/src/components/KBNote/CATALOG.md`: 组件目录和范例, 写正文时读

## 文本规范

本 skill 的文档和它生成的 index.md 遵守 `hx-make-skill/references/prose-rules.md`. 门禁 `uv run .agents/skills/hx-make-skill/scripts/prose_rules.py --check <路径>`. 已知冲突: 多行 JSX 属性会被 P4 报折行, 组件属性里的折行不算违规
