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

const dateText = (d: string | Date): { full: string; md: string; y: number } => {
    const t = new Date(d);
    const y = t.getFullYear();
    const m = String(t.getMonth() + 1).padStart(2, '0');
    const day = String(t.getDate()).padStart(2, '0');
    return { full: `${y}-${m}-${day}`, md: `${m}.${day}`, y };
};

/** 动效延迟最多错开到第 8 个, 再往后同时出现, 免得长列表等太久 */
const stagger = (i: number): React.CSSProperties => ({ '--i': Math.min(i, 8) } as React.CSSProperties);

/**
 * 进入视口渐显: 页面加载时还在视口下方的 data-reveal 元素先标 data-pending (CSS 里是透明 + 下移),
 * 进入视口时去掉标记, 由 transition 渐显上移. 首屏内的元素不动 (它们走 CSS 的加载入场动画), 所以 SSR / 无 JS 时内容始终可见, 不会闪.
 * prefers-reduced-motion 时什么都不做.
 */
function useReveal (): React.RefObject<HTMLDivElement | null> {
    const ref = React.useRef<HTMLDivElement>(null);
    React.useEffect(() => {
        const root = ref.current;
        if (!root || typeof IntersectionObserver === 'undefined') return;
        if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
        const io = new IntersectionObserver((entries) => {
            for (const e of entries) {
                if (e.isIntersecting) {
                    delete (e.target as HTMLElement).dataset.pending;
                    io.unobserve(e.target);
                }
            }
        }, { rootMargin: '0px 0px -8% 0px', threshold: 0.08 });
        root.querySelectorAll<HTMLElement>('[data-reveal]').forEach((el) => {
            if (el.getBoundingClientRect().top < window.innerHeight) return;
            el.dataset.pending = 'true';
            io.observe(el);
        });
        return () => io.disconnect();
    }, []);
    return ref;
}

/** 卡片光晕跟随鼠标: 只写两个 CSS 变量, 不触发 React 渲染; 绘制交给 CSS (radial-gradient 在 ::before 上, 只改 opacity 与位置) */
function onSpot (e: React.PointerEvent<HTMLElement>): void {
    const el = e.currentTarget;
    const r = el.getBoundingClientRect();
    el.style.setProperty('--mx', `${e.clientX - r.left}px`);
    el.style.setProperty('--my', `${e.clientY - r.top}px`);
}

/**
 * 单篇: 编辑排版的一行 —— 左边等宽序号 + 日期, 右边大标题 / 摘要 / 阅读时长 + 标签.
 * featured (本页第一篇) 标题放大, 带"最新"标记.
 * 整行可点 (标题链接 ::after 铺满), 标签浮在上面单独可点.
 */
function PostRow ({ post, index, featured }: { post: Post; index: number; featured: boolean }): ReactNode {
    const d = dateText(post.date);
    const minutes = post.readingTime ? Math.max(1, Math.round(post.readingTime)) : null;
    return (
        <li className={clsx(styles.post, featured && styles.featured)} data-reveal style={stagger(index)} onPointerMove={onSpot}>
            <div className={styles.side}>
                <span className={styles.no}>{String(index + 1).padStart(2, '0')}</span>
                <time className={styles.date} dateTime={d.full}>{d.md}</time>
            </div>
            <div className={styles.body}>
                {featured && <span className={styles.latest}>最新</span>}
                <h3 className={styles.title}>
                    <Link to={post.permalink} className={styles.titleLink}>{post.title}</Link>
                </h3>
                {post.description && <p className={styles.desc}>{post.description}</p>}
                {(minutes !== null || post.tags.length > 0) && (
                    <div className={styles.meta}>
                        {minutes !== null && <span className={styles.read}>约 {minutes} 分钟</span>}
                        {post.tags.map((t) => (
                            <Link key={t.permalink} to={t.permalink} className={styles.tag}>#{t.label}</Link>
                        ))}
                    </div>
                )}
            </div>
            <span className={styles.arrow} aria-hidden="true">↗</span>
        </li>
    );
}

/** 按年份分组, 保持原顺序 (Docusaurus 已按日期倒序) */
function byYear (posts: Post[]): { year: number; posts: Post[] }[] {
    const out: { year: number; posts: Post[] }[] = [];
    for (const p of posts) {
        const y = new Date(p.date).getFullYear();
        const last = out[out.length - 1];
        if (last && last.year === y) last.posts.push(p);
        else out.push({ year: y, posts: [p] });
    }
    return out;
}
/**
 * 博客列表, 三个方向做深:
 * 1. 编辑排版: 超大标题 + 等宽数字 (篇数 / 页码 / 标签数), 网格底纹只铺页头并向下淡出.
 * 2. 时间线: 每年一节, 左侧 sticky 描边大年份 + 竖线, 右侧条目以发丝线分隔, 序号与日期等宽.
 * 3. 动效编排: 页头逐行入场 (stagger), 条目进入视口渐显上移 (IntersectionObserver), 悬停时光晕跟随鼠标 + 标题右移 + 箭头浮现.
 * 全部 CSS transition / animation, 只动 opacity 与 transform; prefers-reduced-motion 时全部关闭.
 * 单栏 880px, 两侧留给 SideCats. 条目只用 metadata, 不渲染正文.
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
    const groups = byYear(posts);
    const firstPage = metadata.page === 1;
    const reveal = useReveal();
    let n = 0;
    return (
        <Layout>
            <BlogWithCats contentWidth={CONTENT_WIDTH} style={{ padding: 0, minHeight: 0 }}>
                <div className={styles.page} ref={reveal}>
                    <header className={styles.head}>
                        <p className={styles.kicker} style={stagger(0)}>
                            <span className={styles.dot} aria-hidden="true" />Blog
                        </p>
                        <h1 className={styles.heading} style={stagger(1)}>{metadata.blogTitle || '博客'}</h1>
                        {metadata.blogDescription && <p className={styles.lead} style={stagger(2)}>{metadata.blogDescription}</p>}
                        <dl className={styles.stats} style={stagger(3)}>
                            <div><dd>{String(metadata.totalCount).padStart(2, '0')}</dd><dt>篇文章</dt></div>
                            <div><dd>{metadata.page}<small>/{metadata.totalPages}</small></dd><dt>页</dt></div>
                            <div><dd>{String(tags.length).padStart(2, '0')}</dd><dt>本页标签</dt></div>
                        </dl>
                        {tags.length > 0 && (
                            <nav className={styles.tags} aria-label="标签" style={stagger(4)}>
                                {tags.map((t) => (
                                    <Link key={t.permalink} to={t.permalink} className={styles.chip}>
                                        {t.label}<span className={styles.chipN}>{t.n}</span>
                                    </Link>
                                ))}
                                <Link to="/blog/tags" className={styles.allTags}>全部标签 →</Link>
                            </nav>
                        )}
                    </header>
                    {groups.map((g) => (
                        <section key={g.year} className={styles.year} aria-label={`${g.year} 年`}>
                            <div className={styles.yearRail}>
                                <h2 className={styles.yearNum} data-reveal>{g.year}</h2>
                                <span className={styles.yearCount}>{g.posts.length} 篇</span>
                            </div>
                            <ol className={styles.list}>
                                {g.posts.map((p, i) => {
                                    const idx = n++;
                                    return <PostRow key={p.permalink} post={p} index={i} featured={firstPage && idx === 0} />;
                                })}
                            </ol>
                        </section>
                    ))}
                    <div className={styles.pager}>
                        <BlogListPaginator metadata={metadata} />
                    </div>
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
