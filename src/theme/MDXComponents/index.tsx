import MDXComponents from '@theme-original/MDXComponents';
import NoteReferences from '@site/src/components/NoteReferences';
import { kbNoteComponents } from '@site/src/components/KBNote';

/**
 * 站点自己的 MDX 组件表。
 *
 * Docusaurus 的 MDXContent 用 `MDXProvider components={MDXComponents}` 把这张表发给每个
 * 页面, 所以在这里多挂一个键, 就等于让所有 markdown 都能直接写该组件  不需要在每篇里
 * import。
 *
 * 这份文件是 theme swizzle: `@theme-original/MDXComponents` 指向被包装的原版
 * (theme-classic 的那张内置表), 展开它再覆盖, 才不会丢掉 h1/a/img/admonition 等内置映射。
 */
const components = {
    ...MDXComponents,
    /**
     * 引用关系方框 (本文引用 / 本文被引用 / 站外来源)。
     *
     * 为什么走组件表而不是让 remark 注入 `import`: MDX 3 只认处理前就在文档顶层的 ESM
     * 节点, remark 阶段追加的 import 会被静默丢弃, 渲染期直接报
     * "Expected component `NoteReferences` to be defined"。
     *
     * (see )
     */
    NoteReferences,
    /**
     * 知识库笔记组件 (Lead / Stats / Layers / Fork / Flow / Compare / Evidence / Apply / Boundary / Figure / Chart).
     * 只在 frontmatter 写了 `mdx: { format: mdx }` 的笔记里生效; 普通 .md 仍是 CommonMark.
     * 范例: src/components/KBNote/CATALOG.md, 检查: scripts/check-kb-note.mjs
     */
    ...kbNoteComponents,
};

export default components;
