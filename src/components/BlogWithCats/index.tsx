import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import './BlogWithCats.css';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
interface Props {
    style?: React.CSSProperties;
    children: React.ReactNode;
}

const BlogWithCats: React.FC<Props> = ({ style, children }) => {
    return (
        <div className="blog-container">
            <div className="blog-content-wrapper">
                <img
                    className="neko neko-left"
                    src={useBaseUrl('/default-img/neko_left.png')}
                    alt="左猫娘"
                />
                <img
                    className="neko neko-right"
                    src={useBaseUrl('/default-img/neko_right.png')}
                    alt="右猫娘"
                />
                <div className="blog-content" style={style}>
                    {children}
                </div>
            </div>
        </div>
    );
};

export default BlogWithCats;
