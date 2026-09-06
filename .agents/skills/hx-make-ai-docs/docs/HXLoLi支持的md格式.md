# HXLoLi支持的md格式

## 支持的语法

除了通用的 md 格式外. HXLoLi 还支持以下语法格式

- 支持 **Mermaid 图表** — 代码块标注 `mermaid`
- 支持 **archify 架构/流程图** — 调 `.agents/skills/hx-archify` (archify skill): typed JSON spec -> validate -> deliver 出自包含 .html, 再按下方 `#ppt` 侧车内嵌; 自带主题切换/缩放/导览/动效, iframe 内嵌后保留.
- 支持 **KaTeX 数学公式** — `$...$` 行内, `$$...$$` 块级
- 支持 **GitHub Alerts** — `> [!NOTE]` `> [!WARNING]` 等
- 支持 **任务列表** — `- [ ]` `- [x]`
- 组合代码块: `"```cpp [组名A-title1]" 和 "```cpp [组名A-title2]"` 可形成 title1 和 title2 两个代码块 (可 tab 切换)
- 支持 vscode 编辑器 "```cpp vscode" 即可让代码块为编辑器
- 支持 图片宽度 与 图片圆角 `![text ##w200##r50##](xxx.jpg)`
- 可在线编辑的 .drawio.svg - 要求文件后缀是 `![](xxx.drawio.svg)`
- 视频拓展:

```bilibili ##BV1Js411o76u##w90%##h600##danmaku=false##p=2##
BV1Js411o76u (代码块需填写内容)
```

## PPT / HTML 侧车内嵌

平台支持把**与 Markdown 同目录的独立 .html** 作为 PPT/演示页内嵌到笔记中:

- 在 md 中写 `[标题 #ppt](xxx.html)` 即可(链接文字必须含 `#ppt` 标记, 链接指向同目录的 .html 侧车文件).
- 可加宽度: `[标题 #ppt ##w100%##](xxx.html)` (不写默认 80%).
- 侧车文件会被自动发布到该笔记页面路由下, 因此必须是**自包含单文件**(内联 CSS/JS, 无外部相对资源, 不依赖当前页面的 DOM), 建议 16:9 画布 (如 1600x900).
- 构建时插件 `ppt-html-assets` 会把 `docs/ ai-docs/ blog/` 下所有 .html 自动拷贝到对应路由, 与 md 是否引用无关; 但只有用 `#ppt` 链接才会渲染成 PPT 查看器.

真实范例: `ai-docs/002-知识沉淀/002-项目学习/002-react-bits-classified-study/` 下的 `index.md` + `overview-ppt.html`.

## 要求

如下为合理的通用解析格式:

```md

## C++

- **x** `x` *x*

```cpp
[]() { }();
```

```

### 标点习惯 (HXLoLi 用户约定, 依据现有手写 ai-docs 观察)

- 正文统一用**英文标点**: `,` `.` `:` `;` `?` `!` 和半角括号 `()`, 后接中文时保持一个空格: `工具, 建议` `注意: 这里`.
- 顿号 `、` 与书名号/引号 `《》` `“”`、破折号 `—` 按原文保留.
- 行内代码 ``...``、围栏代码块、URL、frontmatter 内一律不做转换.
- AI 生成正文后可用归一化脚本检查/改写:

```bash
uv run .agents/skills/hx-make-ai-docs/scripts/format_cn_punct.py --check <note.md>   # 检查
uv run .agents/skills/hx-make-ai-docs/scripts/format_cn_punct.py --diff <note.md>   # 预览差异
uv run .agents/skills/hx-make-ai-docs/scripts/format_cn_punct.py <note.md>          # 原地规范化
```
