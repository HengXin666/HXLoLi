import React from 'react';
import './BlogWithCats.css';

interface Props {
    children: React.ReactNode;
}

const BlogWithCats: React.FC<Props> = ({ children }) => {
    return (
        <div className="blog-container">
            <img
                className="neko neko-left"
                src="default-img/neko_left.png"
                alt="左猫娘"
            />
            <img
                className="neko neko-right"
                src="default-img/neko_right.png"
                alt="右猫娘"
            />
            <div className="blog-content">{children}</div>
        </div>
    );
};

export default BlogWithCats;
