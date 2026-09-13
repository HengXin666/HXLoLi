---
name: hx-docs-ppt
description: "为 HXLoLi ai-docs 笔记产出演示页: 默认走平台原生的 .tsx 内联演示页 ([标题 ##PPT##](x.tsx), React 渲染、跟随站点主题、可换主题), 需要完全自包含单文件时才走 .html 侧车 ([标题 #ppt](x.html), 复用 deck-template.html + ppt-html-assets 插件规则). 负责版式选择、slide 切分、archify 图复用与渲染校验. Use when 笔记需要 PPT/演示页/幻灯片/演示侧车, 或要检查已有 #ppt/##PPT 演示页是否合规 (外部 CDN、放错目录、未注册 deck、尺寸不对)."
---

# hx-docs-ppt

产出**与 `index.md` 同目录的演示页**, 供正文内嵌.

## 先选形态

| 形态 | 引用语法 | 渲染 | 选它的理由 |
|---|---|---|---|
| **`.tsx` 内联演示页 (默认)** | `[标题 ##PPT##](x-deck.tsx)` | React 内联, **跟随站点主题, 可实时换主题** | 平台原生形态, 观感与站点一致; 无自包含约束 |
| `.html` 侧车 | `[标题 #ppt](x.html)` | iframe 独立文档 | 已有外部 HTML, 或确实需要能独立分发的单文件 |

**默认走 `.tsx`**; 只有"手上已经是一个 .html"或"必须单文件自包含"时才走 `.html`. 完整写法、组件清单与坑见 [references/tsx-deck.md](references/tsx-deck.md).

## 三条硬约束 (针对 .html 侧车; 违反任一条就不显示)

1. **同目录**: 平台插件只拷贝"同目录存在 .md"的 .html. 放到 `research-output/`、`assets/` 或父目录都**不会**被发布.
2. **单文件自包含**: 内联全部 CSS/JS; **禁止外部 CDN** (字体/FontAwesome/图床都不行), 禁止相对路径引用图片与脚本. 内嵌资源用 data URI.
3. **16:9 画布**: 基准 `1600x900`, 加 `max-width:100%` 适配 iframe.

```markdown
[标题 #ppt](xxx.html)              <!-- 默认 80% 宽 -->
[标题 #ppt ##w100%##](xxx.html)    <!-- 指定宽度 -->
```

## 密度底线 (两种形态都适用)

"能翻页"不等于"像 PPT". 最常见的失败形态是把正文段落搬进卡片: 每屏一两行字,
缩到 0.4~0.7 倍投屏后一片糊. 每屏都要满足:

1. 一屏一个判断, 且**有视觉宾语** —— 至少一个 `Diagram` / 图表 / `CompareTable` /
   `Timeline` / `Steps` / `Stat` 行 / `CodeBlock`; 全屏只有 `Bullets` 加 `Callout` 的屏要重做.
2. **字号要有量级差** —— 关键数字与标题要明显大于正文, 不要整屏 15~21px 的中等字号.
3. **投放前实测溢出, 不要靠眼看** —— 内容区只有 796px 高 (1600x900 画布扣掉 44/56/60/56 内边距),
   超出部分会被 `overflow: hidden` 静默裁掉. 量测方法见 references.

## 产出流程

1. **先定叙事, 再写页面**: 从 `index.md` 抽出 6~12 个"一页一个论点"的标题; 页数超过 12 说明主线没收住.
2. 复制 [assets/deck-template.html](assets/deck-template.html) 为侧车文件, 逐页替换 `<section class="slide">`.
3. 需要**架构图/流程图/时序图**时, 不要手写 SVG: 用 `archify` skill 生成图, 再用 `<iframe src="./xxx.html">` 嵌进某一页 (archify 产物是自包含 HTML, iframe 内嵌后交互与动效保留). 同目录的 archify 产物同样会被插件发布.
4. 按 [references/theme-spec.md](references/theme-spec.md) 的版式与自检清单过一遍.
5. 在 md 里加 `#ppt` 链接, 然后**构建或本地起站确认渲染**; 只看文件存在不算验证通过.
   - 注意: `ppt-html-assets` 插件在 **dev server 启动时**才枚举 `.html`. 先写 md、后补侧车时, 开发服务器上会 404 —— **重启 dev server** 即可 (构建不受影响, 每次都重新枚举).

命名约定: `overview-ppt.html` (总览) / `<主题>-ppt.html` (专题), 一律小写连字符.

## 自检清单

`.tsx` 形态:

- [ ] 与 `index.md` **同目录**, 导出 `slides()`, 已跑过 `npm run decks`
- [ ] 正文链接写的是 `[标题 ##PPT##](x-deck.tsx)`, 路径与文件名一致
- [ ] `npx tsc --noEmit` 不因这个 deck 报错 (属性里别裸嵌双引号)
- [ ] **实测无溢出** (任一屏 bottom > 840 即为溢出), 且没有屏是"只有文字"的
- [ ] 图注/表头 ≥ 13px, 正文 ≥ 17px (画布坐标); 图上的小字若读不清, 改字号下限**不要**给 SVG 加 `zoom`

`.html` 形态:

- [ ] 与 `index.md` **同目录**, 文件名小写连字符结尾 `.html`
- [ ] grep 不到 `https://` / `http://` / `//cdn` / `src=".` 等外部或相对引用
- [ ] 画布基准 1600x900, 无横向滚动
- [ ] 每页一个论点, 正文用的是读者能看懂的话 (不堆内部术语)
- [ ] 键盘 ←/→、空格、点击均能翻页; 页码与总页数正确
- [ ] md 里有 `[标题 #ppt](xxx.html)` 链接, 且宽度标记写在链接文字里

## 参考文件

- [references/tsx-deck.md](references/tsx-deck.md) —— 什么时候读: 要走 `.tsx` 内联演示页 (默认形态) 时, 含 slides() 骨架、可用组件清单、archify 图复用与常见坑.
- [references/theme-spec.md](references/theme-spec.md) —— 什么时候读: 走 `.html` 侧车时, 需要配色变量与各版式的 HTML 结构, 或排查"内嵌后样式错乱/翻页失效".
- `assets/deck-template.html` —— 什么时候读: 开始产出时复制它作为起点 (含翻页运行时与全部版式示例).