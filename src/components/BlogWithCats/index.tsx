import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import './BlogWithCats.css';

interface Props {
    style?: React.CSSProperties;
    children: React.ReactNode;
}

const BlogWithCats: React.FC<Props> = ({ style, children }) => {
    return (
        <div className="blog-container">
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
    );
};

export default BlogWithCats;
