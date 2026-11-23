/**
 * 知识库正文分节 (rehype): 把「一个 h2 + 到下一个 h2 之前的顶层内容」包成
 * <section class="kb-sec">, 让每一节在视觉上是一块 (src/css/ai-kb.css 给它铺淡底色).
 * 节与节靠底色块和间距分开, 不用横线 (作者不喜欢正文里有太多横线, 2026-11-22).
 *
 * 只挂在 ai-docs 这个 docs 实例上 (docusaurus.config.ts), 主文档和博客的 DOM 不变.
 * 第一个 h2 之前的导语留在节外; 正文末尾由 note-references 注入的 <NoteReferences>
 * 也不进最后一节.
 */
const STOP_JSX = new Set(['NoteReferences']);

export default function rehypeKbSections () {
    return (tree) => {
        const out = [];
        let cur = null;
        const flush = () => {
            if (!cur) return;
            out.push({
                type: 'element',
                tagName: 'section',
                properties: { className: ['kb-sec'] },
                children: cur,
            });
            cur = null;
        };
        for (const node of tree.children) {
            if (node.type === 'element' && node.tagName === 'h2') {
                flush();
                cur = [node];
            } else if ((node.type === 'mdxJsxFlowElement' && STOP_JSX.has(node.name)) || node.type === 'mdxjsEsm') {
                flush();
                out.push(node);
            } else if (cur) {
                cur.push(node);
            } else {
                out.push(node);
            }
        }
        flush();
        tree.children = out;
    };
}
