# Agent Note: ~~删除线~~ 包住链接时, 链接本体看不到删除线

Status: implemented

- 引入于: 本次改动

- 影响: `src/css/custom.css`(新增 del/s 内链接的补线规则)

## Problem

`~~[老鱼简历](https://www.laoyujianli.com/my_resume)~~` 这种"给链接打删除线"的写法整站失效:
文字照常显示, 颜色照常, 悬停下划线动画也正常, 唯独**一条删除线都看不到**。

伪装的点在于**同一页里纯文本的删除线是正常的**: 同一篇 `docs/000-关于/index.md` 里,
`~~[链接](url)~~` 无效而旁边的 `~~智联招聘~~` 正常。很容易误判成"这个 markdown 写法不被支持",
于是去换语法或改成 HTML 标签 —— 但构建产物里 `<del>` 一直都在, GFM 也一直在解析:

```html
<p>简历编辑: <del><span class="tailwind"><a href="...">老鱼简历<span ...></span><span ...></span></a></span></del> 不如...</p>
```

## Root cause

三个事实叠在一起, 每个单独看都正常。用真实构建 CSS 做 2x2 对照即可分离 (固定 `<del><span><a>`
结构, 只改 `display` 与 `text-decoration`):

```text
c1 a{display:inline}                  -> line=underline
c2 a{display:inline-block}            -> line=underline
c3 a{display:inline;text-decoration:inherit}       -> line=none
c4 a{display:inline-block;text-decoration:inherit} -> line=none
```

1. `src/components/HXLink/index.tsx` 用 `inline-block` 做悬停下划线动画。`inline-block` 是
   **atomic inline**, 祖先 `<del>` 的 `text-decoration` **不向它传播** —— 这是 CSS 规范行为,
   不是浏览器 bug。
2. `<a>` 外面还套了一层 `<span class="tailwind">` (`src/theme/MDXComponents/A.tsx`), 它触发
   tailwind preflight 的 `.tailwind a{color:inherit;text-decoration:inherit}`。
   `inherit` 把一个**已经被 atomic inline 截断**的 `none` 显式抄到链接上, 于是连
   "祖先线也没传下来"这个状态都被固化了。
3. 知识库页面还有 `.ai-kb-page .markdown a:not(...){text-decoration:none}` (`src/css/ai-kb.css`),
   特异性比任何朴素修复都高。

也就是说: 第 1 条让线传不下来, 第 2/3 条负责把任何"本来还能渲染出来的线"抹平。只修其中一条都不够。

## Decision

**在 `src/css/custom.css` 里为 `<del>`/`<s>` 内的链接显式补回删除线, 并带 `!important`。**

```css
.markdown del a,
.markdown s a {
  text-decoration-line: line-through !important;
}
```

- 用 `text-decoration-line` 而不是 `text-decoration` 简写: 简写会把颜色与样式一并重置成
  默认值, 在知识库那种带自己配色/虚线下边框的页面上会顺手改掉视觉。
- `!important` 不是偷懒: 不带它时实测在知识库页面**被压回 `line=none`**(见 Alternatives 第一条),
  因为 `.ai-kb-page .markdown a:not(...)` 的特异性高于 `.markdown del a`。
- 作用域限定在 `.markdown` 内的 `del`/`s` 后代 link, 不碰 `HXLink` 自身的悬停动画,
  也不影响 `del` 外的普通链接(见 Consequences 的实测反例)。

## Alternatives considered

- **什么都不做, 让作者改用 `<del>` 手写或干脆不划线**: 零成本。否决理由: 纯文本
  `~~删除~~` 是好的, 而这个缺口恰好只吃"最常用的那个组合"(给一个链接打删除线表示"这条路我否决了"),
  `docs/000-关于/index.md` 里一次就用了四处。把一条渲染契约变成"某些语法组合别用", 是把坑留给下一次写作。
- **只加朴素规则 `.markdown del a{text-decoration-line:line-through}`, 不带 `!important`**: 改动最小,
  文档页面也确实变好了。否决理由: **实测在知识库页面失效** ——
  `.ai-kb-page .markdown a:not(.table-of-contents__link):not(.menu__link){text-decoration:none}`
  (0,3,1 + 两个 `:not` 内的类) 压过它, `getComputedStyle` 读回 `line=none`。
  这类"看起来修好了"的改动最危险: 文档页截图过审, 知识库静默回退。
- **改 `HXLink`, 把 `inline-block` 换成 `inline`**: 治本方向 —— 线能自然传播。否决理由:
  `relative inline-block` 是那个悬停下划线动画(两条绝对定位的 `span`)的定位基准, 去掉它动画会塌掉,
  等于用一个视觉回归换另一个视觉修复。
- **在 `HXLink` 里读父级 `<del>` 并加类**: 不改 CSS 优先级、语义最干净。否决理由: 要在 React 层
  探测祖先元素, 得引入 ref + DOM 遍历, 而这本质是**渲染期的样式问题**; 让样式层解决它,
  不必把一个纯 CSS 的优先级问题升级成组件逻辑。
- **去掉 `.tailwind a{text-decoration:inherit}` 这条 preflight 规则**: 只删一行。否决理由:
  它是 tailwind 的既有基座, 站内大量 `.tailwind` 容器依赖它来避免继承外层的链接线;
  为这个 bug 拆基座会波及无法预估的范围。

## Consequences

- `~~[文字](url)~~` 与 `~~[文字](url)~~` 的变体在文档页与知识库页都正常划线, 实测
  `getComputedStyle` 为 `line-through`, 颜色仍取站点链接色。
- **反例已实测**: `del` 外的普通链接 `line=none`, 未被这条规则波及 —— 说明作用域没有过度扩大。
- 规则落在 `custom.css` 的 Markdown 段; 它约束的是"删除线内的链接", 与 `HXLink` 自身的
  悬停动画互不干涉(后者画的是绝对定位的 `span`, 不是 `text-decoration`)。
- 代价: `.markdown del a` 上挂了一个 `!important`。这是本仓库第二处为"压过知识库页面
  `.ai-kb-page` 规则"而付出的优先级成本; 若将来出现第三处, 该考虑的是收紧
  `.ai-kb-page .markdown a` 的选择器, 而不是继续加 `!important`。
