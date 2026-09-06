import type { BlogSidebar } from '@docusaurus/plugin-content-blog';
import {
    BlogPostProvider,
    useBlogPost,
} from '@docusaurus/plugin-content-blog/client';
import { HtmlClassNameProvider, ThemeClassNames } from '@docusaurus/theme-common';
import BlogLayout from '@theme/BlogLayout';
import BlogPostItem from '@theme/BlogPostItem';
import type { Props } from '@theme/BlogPostPage';
import BlogPostPageMetadata from '@theme/BlogPostPage/Metadata';
import BlogPostPageStructuredData from '@theme/BlogPostPage/StructuredData';
import BlogPostPaginator from '@theme/BlogPostPaginator';
import ContentVisibility from '@theme/ContentVisibility';
import TOC from '@theme/TOC';
import clsx from 'clsx';
import React, { type ReactNode } from 'react';

import config from '@generated/docusaurus.config';
import BlogWithCats from '@site/src/components/BlogWithCats';
import HXGiscus from '@site/src/components/Giscus';
import InlineEditor from '@site/src/components/DevTools/InlineEditor';
import MDXA from '../MDXComponents/A';

function BlogPostPageContent ({
    sidebar,
    children,
}: {
    sidebar: BlogSidebar;
    children: ReactNode;
}): ReactNode {
    const { metadata, toc } = useBlogPost();
    const { nextItem, prevItem, frontMatter } = metadata;
    const {
        hide_table_of_contents: hideTableOfContents,
        toc_min_heading_level: tocMinHeadingLevel,
        toc_max_heading_level: tocMaxHeadingLevel,
    } = frontMatter;
    return (
        <BlogLayout
            sidebar={sidebar}
            toc={
                !hideTableOfContents && toc.length > 0 ? (
                    <TOC
                        toc={toc}
                        minHeadingLevel={tocMinHeadingLevel}
                        maxHeadingLevel={tocMaxHeadingLevel}
                    />
                ) : undefined
            }>
            <ContentVisibility metadata={metadata} />

            <BlogWithCats>
                {/* 本地开发专属: VS Code 跳转 (选中右键) — 博客文章同样支持 (仅 dev-edit-server 可达时渲染) */}
                <InlineEditor source={metadata.source} />
                <BlogPostItem>
                    {children}
                </BlogPostItem>
                <div style={{ display: 'flex', justifyContent: 'flex-end', marginTop: '20px', flexWrap: 'wrap', alignItems: 'center', gap: '4px' }}>
                    请作者喝奶茶:
                    <div className="icon-container" style={{ marginLeft: '10px' }}>
                        <img src={`${config.baseUrl}default-icons/alipay.svg`} alt="Alipay Icon" className="icon" />
                        <img src={`${config.baseUrl}img/alipay_qr_code.png`} alt="QR Code" className="qr-code" />
                    </div>
                    <div className="icon-container" style={{ marginLeft: '10px' }}>
                        <img src={`${config.baseUrl}default-icons/wechat.svg`} alt="Alipay Icon" className="icon" />
                        <img src={`${config.baseUrl}img/wechat_qr_code.png`} alt="QR Code" className="qr-code" />
                    </div>
                </div>
                <div style={{ display: 'flex', justifyContent: 'flex-end', flexWrap: 'wrap' }}>
                    <span style={{ fontSize: '12px' }}>本文遵循 <img src={`${config.baseUrl}default-icons/cc.svg`} alt="CC" style={{ width: '14px', verticalAlign: 'middle' }} /> <MDXA href='https://creativecommons.org/licenses/by-sa/4.0/'>CC 4.0 BY-SA</MDXA> 版权协议, 转载请标明出处</span>
                </div>
                {(nextItem || prevItem) && (
                    <BlogPostPaginator nextItem={nextItem} prevItem={prevItem} />
                )}
                <HXGiscus />
            </BlogWithCats>
        </BlogLayout>
    );
}

export default function BlogPostPage (props: Props): ReactNode {
    const BlogPostContent = props.content;
    return (
        <BlogPostProvider content={props.content} isBlogPostPage>
            <HtmlClassNameProvider
                className={clsx(
                    ThemeClassNames.wrapper.blogPages,
                    ThemeClassNames.page.blogPostPage,
                )}>
                <BlogPostPageMetadata />
                <BlogPostPageStructuredData />
                <BlogPostPageContent sidebar={props.sidebar}>
                    <BlogPostContent />
                </BlogPostPageContent>
            </HtmlClassNameProvider>
        </BlogPostProvider>
    );
}
