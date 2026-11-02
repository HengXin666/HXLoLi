import { PageMetadata, HtmlClassNameProvider, ThemeClassNames } from '@docusaurus/theme-common';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import BlogListPaginator from '@theme/BlogListPaginator';
import SearchMetadata from '@theme/SearchMetadata';
import type { Props } from '@theme/BlogListPage';
import BlogListPageStructuredData from '@theme/BlogListPage/StructuredData';
import BlogWithCats from '@site/src/components/BlogWithCats';
import clsx from 'clsx';
import React, { type ReactNode } from 'react';
import styles from './styles.module.css';

/** 正文宽度; 两侧猫娘按它算留白 */
const CONTENT_WIDTH = 880;

type Post = Props['items'][number]['content']['metadata'];

function BlogListPageMetadata (props: Props): ReactNode {
    const { metadata } = props;
    const { blogDescription, blogTitle } = metadata;
    return (
        <>
            <PageMetadata title={blogTitle} description={blogDescription} />
            <SearchMetadata tag="blog_posts_list" />
        </>
    );
}

const dateText = (d: string | Date): { full: string; day: string; ym: string } => {
    const t = new Date(d);
    const y = t.getFullYear();
    const m = String(t.getMonth() + 1).padStart(2, '0');
    const day = String(t.getDate()).padStart(2, '0');
    return { full: `${y}-${m}-${day}`, day, ym: `${y}.${m}` };
};

function PostRow ({ post }: { post: Post }): ReactNode {
    const d = dateText(post.date);
    const minutes = post.readingTime ? Math.max(1, Math.round(post.readingTime)) : null;
    return (
        <li className={styles.post}>
            <time className={styles.date} dateTime={d.full}>
                <span className={styles.day}>{d.day}</span>
                <span className={styles.ym}>{d.ym}</span>
            </time>
            <div className={styles.body}>
                <h2 className={styles.title}>
                    <Link to={post.permalink}>{post.title}</Link>
                </h2>
                {post.description && <p className={styles.desc}>{post.description}</p>}
                <div className={styles.meta}>
                    {minutes !== null && <span>约 {minutes} 分钟</span>}
                    {post.tags.map((t) => (
                        <Link key={t.permalink} to={t.permalink} className={styles.tag}>#{t.label}</Link>
                    ))}
                </div>
            </div>
        </li>
    );
}
/**
 * 博客列表: 不再用 BlogLayout 的侧栏 (侧栏"最近文章"与列表本身重复), 改为 BlogWithCats 包住的单栏,
 * 页头 (标题 / 简介 / 篇数 + 本页标签云) + 时间轴式条目 (左日期块, 右标题 / 摘要 / 阅读时长 / 标签) + 原分页器.
 * 条目只用 metadata (标题 / 描述 / 日期 / 标签), 不渲染正文, 列表页因此不加载每篇的 MDX 组件.
 * .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md
 */
function BlogListPageContent (props: Props): ReactNode {
    const { metadata, items } = props;
    const posts = items.map((i) => i.content.metadata);
    const tagCounts = new Map<string, { label: string; permalink: string; n: number }>();
    for (const p of posts) {
        for (const t of p.tags) {
            const cur = tagCounts.get(t.permalink);
            if (cur) cur.n++;
            else tagCounts.set(t.permalink, { label: t.label, permalink: t.permalink, n: 1 });
        }
    }
    const tags = [...tagCounts.values()].sort((a, b) => b.n - a.n || a.label.localeCompare(b.label));
    const total = metadata.totalCount;
    return (
        <Layout>
            <BlogWithCats contentWidth={CONTENT_WIDTH} style={{ padding: 0, background: 'transparent', minHeight: 0 }}>
                <div className={styles.page}>
                    <header className={styles.head}>
                        <p className={styles.kicker}>BLOG</p>
                        <h1 className={styles.heading}>{metadata.blogTitle || '博客'}</h1>
                        {metadata.blogDescription && <p className={styles.lead}>{metadata.blogDescription}</p>}
                        <p className={styles.count}>
                            共 {total} 篇{metadata.totalPages > 1 ? ` · 第 ${metadata.page}/${metadata.totalPages} 页` : ''}
                        </p>
                        {tags.length > 0 && (
                            <nav className={styles.tags} aria-label="标签">
                                {tags.map((t) => (
                                    <Link key={t.permalink} to={t.permalink} className={styles.tagChip}>
                                        {t.label}<span className={styles.tagN}>{t.n}</span>
                                    </Link>
                                ))}
                                <Link to="/blog/tags" className={styles.allTags}>全部标签 →</Link>
                            </nav>
                        )}
                    </header>
                    <ol className={styles.list}>
                        {posts.map((p) => <PostRow key={p.permalink} post={p} />)}
                    </ol>
                    <BlogListPaginator metadata={metadata} />
                </div>
            </BlogWithCats>
        </Layout>
    );
}

export default function BlogListPage (props: Props): ReactNode {
    return (
        <HtmlClassNameProvider
            className={clsx(
                ThemeClassNames.wrapper.blogPages,
                ThemeClassNames.page.blogListPage,
            )}>
            <BlogListPageMetadata {...props} />
            <BlogListPageStructuredData {...props} />
            <BlogListPageContent {...props} />
        </HtmlClassNameProvider>
    );
}
