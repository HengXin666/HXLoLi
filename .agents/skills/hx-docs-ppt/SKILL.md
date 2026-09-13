---
name: hx-docs-ppt
description: "为 HXLoLi ai-docs 的 [#PPT]() 内嵌生成自包含单文件 16:9 演示页 HTML 侧车 —— 统一暗色主题、键盘/点击翻页、页码与进度、封面/要点/表格/对照/代码/收尾等版式, 并负责按平台 `ppt-html-assets` 插件规则校验 (必须与 index.md 同目录、单文件无外部依赖). Use when 笔记需要 PPT/演示页/幻灯片/overview-ppt.html 侧车, 或要检查已有 #ppt 侧车是否合规 (外部 CDN、放错目录、尺寸不对)."
---

# hx-docs-ppt

产出**与 `index.md` 同目录的单个 .html 文件**, 供正文 `[标题 #ppt](xxx.html)` 内嵌.

## 三条硬约束 (违反任一条就不显示)

1. **同目录**: 平台插件只拷贝"同目录存在 .md"的 .html. 放到 `research-output/`、`assets/` 或父目录都**不会**被发布.
2. **单文件自包含**: 内联全部 CSS/JS; **禁止外部 CDN** (字体/FontAwesome/图床都不行), 禁止相对路径引用图片与脚本. 内嵌资源用 data URI.
3. **16:9 画布**: 基准 `1600x900`, 加 `max-width:100%` 适配 iframe.

```markdown
[标题 #ppt](xxx.html)              <!-- 默认 80% 宽 -->
[标题 #ppt ##w100%##](xxx.html)    <!-- 指定宽度 -->
```

## 产出流程

1. **先定叙事, 再写页面**: 从 `index.md` 抽出 6~12 个"一页一个论点"的标题; 页数超过 12 说明主线没收住.
2. 复制 [assets/deck-template.html](assets/deck-template.html) 为侧车文件, 逐页替换 `<section class="slide">`.
3. 需要**架构图/流程图/时序图**时, 不要手写 SVG: 用 `archify` skill 生成图, 再用 `<iframe src="./xxx.html">` 嵌进某一页 (archify 产物是自包含 HTML, iframe 内嵌后交互与动效保留). 同目录的 archify 产物同样会被插件发布.
4. 按 [references/theme-spec.md](references/theme-spec.md) 的版式与自检清单过一遍.
5. 在 md 里加 `#ppt` 链接, 然后**构建或本地起站确认渲染**; 只看文件存在不算验证通过.

命名约定: `overview-ppt.html` (总览) / `<主题>-ppt.html` (专题), 一律小写连字符.

## 自检清单

- [ ] 与 `index.md` **同目录**, 文件名小写连字符结尾 `.html`
- [ ] grep 不到 `https://` / `http://` / `//cdn` / `src=".` 等外部或相对引用
- [ ] 画布基准 1600x900, 无横向滚动
- [ ] 每页一个论点, 正文用的是读者能看懂的话 (不堆内部术语)
- [ ] 键盘 ←/→、空格、点击均能翻页; 页码与总页数正确
- [ ] md 里有 `[标题 #ppt](xxx.html)` 链接, 且宽度标记写在链接文字里

## 参考文件

- [references/theme-spec.md](references/theme-spec.md) —— 什么时候读: 需要配色变量、各版式的 HTML 结构 (封面/要点/表格/对照/代码/引用/收尾)、或排查"内嵌后样式错乱/翻页失效"时.
- `assets/deck-template.html` —— 什么时候读: 开始产出时复制它作为起点 (含翻页运行时与全部版式示例).
