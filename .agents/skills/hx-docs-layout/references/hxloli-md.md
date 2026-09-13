# HXLoLi 支持的 Markdown 格式

## 通用语法

除标准 Markdown 外, 平台还支持:

- **Mermaid 图表** —— 代码块标注 `mermaid`
- **KaTeX 数学公式** —— `$...$` 行内, `$$...$$` 块级
- **GitHub Alerts** —— `> [!NOTE]` `> [!WARNING]` 等
- **任务列表** —— `- [ ]` / `- [x]`
- **组合代码块** —— 围栏标注写成 `cpp [组名A-title1]` 与 `cpp [组名A-title2]`, 生成可 tab 切换的两个代码块
- **vscode 编辑器代码块** —— 围栏标注 `cpp vscode`
- **图片宽度与圆角** —— `![text ##w200##r50##](xxx.jpg)`
- **可在线编辑的 drawio** —— 文件后缀必须是 `.drawio.svg`, 写作 `![](xxx.drawio.svg)`
- **B 站视频**:

```bilibili ##BV1Js411o76u##w90%##h600##danmaku=false##p=2##
BV1Js411o76u (代码块需填写内容)
```

- **架构图/流程图/时序图/数据流图/状态图** —— 调用 `archify` skill (vendored 于 `.agents/skills/hx-archify`): typed JSON spec -> validate -> deliver 出自包含 .html, 再按下方 `#ppt` 侧车规范内嵌. 自带主题切换/缩放/导览/动效, iframe 内嵌后保留; 不要手绘零散 SVG.

## PPT / HTML 侧车内嵌

平台支持把**与 Markdown 同目录的独立 .html** 作为演示页内嵌到笔记中:

- 正文写 `[标题 #ppt](xxx.html)`; **链接文字必须含 `#ppt` 标记**, 链接指向同目录 .html 侧车文件.
- 可加宽度: `[标题 #ppt ##w100%##](xxx.html)` (不写默认 80%).
- 侧车文件会被自动发布到该笔记页面路由下, 因此必须是**自包含单文件** (内联 CSS/JS, 无外部相对资源, 不依赖宿主页面 DOM), 建议 16:9 画布 (如 1600x900).
- 构建插件 `ppt-html-assets` 会把 `docs/ ai-docs/ blog/` 下所有 .html 拷到对应路由 (与 md 是否引用无关); 但**只有用 `#ppt` 链接才会渲染成 PPT 查看器**.

侧车文件的主题模板与产出流程见 `hx-docs-ppt` skill.

真实范例: `ai-docs/002-知识沉淀/002-项目学习/002-react-bits-classified-study/` 下的 `index.md` + `overview-ppt.html`.

## 标点习惯 (HXLoLi 用户约定, 依据现有手写 ai-docs 观察)

- 正文统一用**英文标点**: `,` `.` `:` `;` `?` `!` 和半角括号 `()`, 后接中文时保持一个空格: `工具, 建议` `注意: 这里`.
- 顿号 `、` 与书名号/引号 `《》` `“”`、破折号 `—` 按原文保留.
- 行内代码、围栏代码块、URL、frontmatter 内一律不做转换.
- 归一化脚本: `scripts/format_cn_punct.py` (用法见 hx-docs-layout 的 SKILL.md).
