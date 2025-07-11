import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useBlogPost } from '@docusaurus/plugin-content-blog/client';
import type { Props } from '@theme/BlogPostItem/Header/Title';

import styles from './styles.module.css';

export default function BlogPostItemHeaderTitle ({ className }: Props): ReactNode {
    const { metadata, isBlogPostPage } = useBlogPost();
    const { permalink, title } = metadata;
    const TitleHeading = isBlogPostPage ? 'h1' : 'h2';
    return (
        <TitleHeading
            className={clsx(styles.title, className)}
            style={isBlogPostPage ? { textAlign: 'center', whiteSpace: 'nowrap' } : {}}
        >
            {isBlogPostPage ? title : <Link to={permalink}>{title}</Link>}
        </TitleHeading>
    );
}
