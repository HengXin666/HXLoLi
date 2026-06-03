# HXLoLi支持的md格式

## 支持的语法

除了通用的 md 格式外. HXLoLi 还支持以下语法格式

- 支持 **Mermaid 图表** — 代码块标注 `mermaid`
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

## 要求

如下为合理的通用解析格式:

````md

## C++

- **x** `x` *x*

```cpp
[]() { }();
```

````

用户习惯: 使用 **英文标点符号** 如 `,:"!?;` 而不是 `，：“”！？；`