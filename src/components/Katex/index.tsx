import React from 'react';
import katex from 'katex';
import 'katex/dist/katex.min.css';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
interface KatexProps {
    content: string;
}

const Katex: React.FC<KatexProps> = ({ content }) => {
    const htmlContent = katex.renderToString(content, {
        throwOnError: false, // 遇到错误时不抛出异常
    });

    return <span dangerouslySetInnerHTML={{ __html: htmlContent }} />;
};

export default Katex;