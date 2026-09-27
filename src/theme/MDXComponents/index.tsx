import MDXComponents from '@theme-original/MDXComponents';
import NoteReferences from '@site/src/components/NoteReferences';

/**
 * 站点自己的 MDX 组件表。
 *
 * Docusaurus 的 MDXContent 用 `MDXProvider components={MDXComponents}` 把这张表发给每个
 * 页面, 所以在这里多挂一个键, 就等于让所有 markdown 都能直接写该组件 —— 不需要在每篇里
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
     * (see .agents/notes/implemented/architecture/2026-09-26-note-references-auto-rendered.md)
     */
    NoteReferences,
};

export default components;
