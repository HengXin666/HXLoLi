import Link from '@docusaurus/Link';
import BlogWithCats from '@site/src/components/BlogWithCats';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { latestAiDocs, latestBlogs, latestDocs, type LatestDoc } from '@site/data/latestDocs';
import {
    PROFILE_AS_OF,
    profileLede,
    profileLinks,
    profileStats,
    stackGroups,
} from '@site/data/profile';
import { PROJECT_SINCE, projects } from '@site/data/projectData';
import Layout from '@theme/Layout';
import React, { lazy, useEffect, useState } from 'react';
import { FaGithub, FaEnvelope } from 'react-icons/fa6';
import { SiBilibili, SiLeetcode } from 'react-icons/si';

import ProjectCarousel from '../ProjectCarousel';
import Tooltip from '../common/Tooltip';
import LazyMount from './LazyMount';
import {
    loadEcosystemPanel,
    loadHomeAnime,
    loadWakaPulse,
    loadWakaTimeChart,
    loadWordCountChart,
    loadWordStats,
} from './lazyChunks';
import styles from './page.module.css';

// 图表与关系图各自拆成独立 chunk, 首屏 HTML 只带文字与占位
// 字数数据 (约 35 KB) 跟图表一起懒加载, 不进首页入口
// import() 集中在 lazyChunks.js, 原因见该文件顶部注释 (nodenext 下 .tsx 里的 import() 过不了 tsc)
const WordCountChart = lazy(() =>
    Promise.all([loadWordCountChart(), loadWordStats()]).then(([chart, data]) => ({
        default: () => <chart.default rawData={data.stats} />,
    })),
);
const WakaPulse = lazy(loadWakaPulse);
const WakaTimeChart = lazy(loadWakaTimeChart);
const EcosystemPanel = lazy(loadEcosystemPanel);
// 番剧块要拉约 2.4 MB (gzip) 的追番记录, 与组件一起滚到附近才加载
const HomeAnime = lazy(loadHomeAnime);

const SECTIONS = [
    { id: 'writing', label: '写作' },
    { id: 'notes', label: '最近更新' },
    { id: 'time', label: '时间' },
    { id: 'projects', label: '项目' },
    { id: 'anime', label: '追番' },
    { id: 'about', label: '关于' },
] as const;

/** "最近更新"的三栏: 笔记 (docs/)、博客 (blog/)、AI 文章 (ai-docs/), 数据都由 docs-rss-plugin 生成 */
const FEEDS: { id: string; label: string; items: LatestDoc[]; all: string; allLabel: string }[] = [
    { id: 'blog', label: '博客', items: latestBlogs, all: '/blog', allLabel: '全部博客' },
    { id: 'ai', label: 'AI 文章', items: latestAiDocs, all: '/knowledge-base/关于/', allLabel: '全部 AI 文章' },
    { id: 'docs', label: '笔记', items: latestDocs, all: '/docs/关于', allLabel: '全部笔记' },
];

const LINK_ICONS: Record<string, React.ReactNode> = {
    github: <FaGithub aria-hidden="true" />,
    bilibili: <SiBilibili aria-hidden="true" />,
    leetcode: <SiLeetcode aria-hidden="true" />,
    mail: <FaEnvelope aria-hidden="true" />,
};

/** 左栏导航高亮: 视口上 1/3 处落在哪一节就点亮哪一节 */
function useActiveSection (): string {
    const [active, setActive] = useState<string>(SECTIONS[0].id);
    useEffect(() => {
        if (typeof IntersectionObserver === 'undefined') return;
        const io = new IntersectionObserver((entries) => {
            const hit = entries.filter((e) => e.isIntersecting).sort((a, b) => a.boundingClientRect.top - b.boundingClientRect.top)[0];
            if (hit) setActive(hit.target.id);
        }, { rootMargin: '-30% 0px -60% 0px' });
        SECTIONS.forEach((s) => {
            const el = document.getElementById(s.id);
            if (el) io.observe(el);
        });
        return () => io.disconnect();
    }, []);
    return active;
}

/**
 * 首页: 左栏是固定的个人卡 (头像 + 名字 / 一句话 / 在做的项目 / 数字 / 分节导航),
 * 右栏依次是字数、最近更新 (博客 / AI 文章 / 笔记)、WakaTime (摘要 + 更多: 趋势图)、近一年项目 (+ 更多: 关系图浮层)、追番、关于.
 * 悬停说明统一走 common/Tooltip, 不用原生 title.
 * .agents/notes/implemented/feature/2026-10-31-homepage-profile-and-project-index.md
 */
export default function HomePage (): React.ReactElement {
    const { siteConfig } = useDocusaurusContext();
    const active = useActiveSection();
    const [graphOpen, setGraphOpen] = useState(false);
    const [wakaFull, setWakaFull] = useState(false);
    // 趋势图第一次展开后才挂载 (才下载 recharts), 之后收起只是折叠, 再展开不用重新加载
    const [wakaMounted, setWakaMounted] = useState(false);
    const [feed, setFeed] = useState(FEEDS[0].id);
    const avatar = useBaseUrl('/img/ecosystem/profile.webp');
    const siteBase = useBaseUrl('/');

    const nowBuilding = projects
        .filter((p) => p.pushedAt >= PROJECT_SINCE)
        .sort((a, b) => b.pushedAt.localeCompare(a.pushedAt))
        .slice(0, 5);

    return (
        <Layout
            title={`いにゃさい ${siteConfig.title}`}
            description="Heng_Xin 的笔记、轮子和应用: 字数曲线、WakaTime 统计与近一年项目"
        >
            <BlogWithCats contentWidth={1180}>
            <div className={styles.page}>
                <aside className={styles.aside}>
                    <div className={styles.card}>
                        <div className={styles.identity}>
                            <img className={styles.avatar} src={avatar} alt="Heng_Xin" width={48} height={48} fetchPriority="high" />
                            <div>
                                <h1 className={styles.name}>{siteConfig.title}</h1>
                                <p className={styles.tagline}>{siteConfig.tagline}</p>
                            </div>
                        </div>
                        <p className={styles.lede}>{profileLede}</p>

                        <div className={styles.building}>
                            <span className={styles.label}>在做</span>
                            {nowBuilding.map((p) => (
                                <a key={p.title} className={styles.chip} href={p.url} target="_blank" rel="noopener noreferrer">
                                    {p.title}
                                </a>
                            ))}
                        </div>

                        <dl className={styles.stats}>
                            {profileStats.map((item) => (
                                <Tooltip key={item.label} width="auto" content={<span className={styles.tipText}>{item.hint}</span>}>
                                    <div className={styles.stat} tabIndex={0}>
                                        <dd>{item.value}</dd>
                                        <dt>{item.label}</dt>
                                    </div>
                                </Tooltip>
                            ))}
                        </dl>

                        <div className={styles.actions}>
                            <Link className={styles.primary} to="/docs/关于">开始阅读</Link>
                            <a className={styles.ghost} href="https://github.com/HengXin666" target="_blank" rel="noopener noreferrer">GitHub</a>
                        </div>
                    </div>

                    <nav className={styles.toc} aria-label="首页分节">
                        {SECTIONS.map((s) => (
                            <a key={s.id} href={`#${s.id}`} className={active === s.id ? styles.tocOn : styles.tocItem}>
                                {s.label}
                            </a>
                        ))}
                    </nav>
                </aside>

                <main className={styles.main}>
                    <section id="writing" className={styles.section}>
                        <header className={styles.head}>
                            <h2>写了多少</h2>
                            <p>每次提交的累计字数, 下方条是单次增删. 拖动底部滑块看任意区间.</p>
                        </header>
                        <div className={styles.panel}>
                            <LazyMount minHeight={330} rootMargin="0px" label="字数图加载中">
                                <WordCountChart />
                            </LazyMount>
                        </div>
                    </section>

                    <section id="notes" className={styles.section}>
                        <header className={styles.head}>
                            <h2>最近更新</h2>
                            <div className={styles.tabs} role="tablist" aria-label="最近更新的分类">
                                {FEEDS.map((f) => (
                                    <button
                                        key={f.id}
                                        type="button"
                                        role="tab"
                                        id={`feed-tab-${f.id}`}
                                        aria-selected={feed === f.id}
                                        aria-controls={`feed-${f.id}`}
                                        className={feed === f.id ? styles.tabOn : styles.tab}
                                        onClick={() => setFeed(f.id)}
                                    >
                                        {f.label}
                                        <span className={styles.tabCount}>{f.items.length}</span>
                                    </button>
                                ))}
                            </div>
                        </header>
                        {FEEDS.map((f) => (
                            <div
                                key={f.id}
                                id={`feed-${f.id}`}
                                role="tabpanel"
                                aria-labelledby={`feed-tab-${f.id}`}
                                hidden={feed !== f.id}
                                className={styles.feed}
                            >
                                <ol className={styles.notes}>
                                    {f.items.slice(0, 6).map((doc) => (
                                        <li key={doc.to}>
                                            <Link to={siteBase + doc.to.replace(/^\//, '')}>
                                                <span className={styles.noteCat}>{doc.category}</span>
                                                <span className={styles.noteTitle}>{doc.title}</span>
                                                <time className={styles.noteDate}>{doc.date}</time>
                                            </Link>
                                        </li>
                                    ))}
                                </ol>
                                <Link className={styles.feedAll} to={f.all}>{f.allLabel} →</Link>
                            </div>
                        ))}
                    </section>

                    <section id="time" className={styles.section}>
                        <header className={styles.head}>
                            <h2>时间花在哪</h2>
                            <p>编码时长 = 编辑器里写代码的时间; 使用设备 = 各系统上开着编辑器的时间, 含挂机.</p>
                        </header>
                        <div className={styles.panel}>
                            <LazyMount minHeight={300} label="WakaTime 加载中">
                                <WakaPulse />
                            </LazyMount>
                            <button
                                type="button"
                                className={styles.magicBtn}
                                data-open={wakaFull}
                                aria-expanded={wakaFull}
                                aria-controls="waka-trend"
                                onClick={() => {
                                    setWakaMounted(true);
                                    setWakaFull((v) => !v);
                                }}
                            >
                                <span className={styles.magicGlyph} aria-hidden="true">
                                    <svg viewBox="0 0 24 24" width="16" height="16"><path d="M3 17l5-6 4 3 6-8 3 3" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" /></svg>
                                </span>
                                {wakaFull ? '收起趋势图' : '更多 · 展开趋势图'}
                            </button>
                            <div id="waka-trend" className={styles.magic} data-open={wakaFull} inert={!wakaFull}>
                                <div className={styles.magicInner}>
                                    {wakaMounted && (
                                        <LazyMount minHeight={460} rootMargin="200px" label="趋势图加载中">
                                            <WakaTimeChart />
                                        </LazyMount>
                                    )}
                                </div>
                            </div>
                        </div>
                    </section>

                    <section id="projects" className={styles.section}>
                        <header className={styles.head}>
                            <h2>近一年在做</h2>
                            <p>{PROJECT_SINCE} 之后仍有推送的公开仓库, 手工维护. 悬停暂停并看完整简介, 点卡片进 GitHub.</p>
                        </header>
                        <ProjectCarousel more={{ open: graphOpen, onToggle: () => setGraphOpen((o) => !o) }} />
                        {graphOpen && (
                            <LazyMount minHeight={0} rootMargin="400px" label="关系图加载中">
                                <EcosystemPanel onClose={() => setGraphOpen(false)} />
                            </LazyMount>
                        )}
                    </section>

                    <section id="anime" className={styles.section}>
                        <header className={styles.head}>
                            <h2>在追的番</h2>
                            <p>BanGuMi 追番记录, 列的是最近更新进度的在看.</p>
                        </header>
                        <LazyMount minHeight={360} rootMargin="300px" label="追番记录加载中">
                            <HomeAnime />
                        </LazyMount>
                    </section>

                    <section id="about" className={styles.section}>
                        <header className={styles.head}>
                            <h2>关于</h2>
                        </header>
                        <div className={styles.about}>
                            {stackGroups.map((g) => (
                                <div key={g.group} className={styles.stackRow}>
                                    <span className={styles.label}>{g.group}</span>
                                    <ul>
                                        {g.items.map((t) => <li key={t}>{t}</li>)}
                                    </ul>
                                </div>
                            ))}
                            <div className={styles.stackRow}>
                                <span className={styles.label}>找到我</span>
                                <ul className={styles.links}>
                                    {profileLinks.map((l) => (
                                        <li key={l.label}>
                                            <a href={l.href} target="_blank" rel="noopener noreferrer" data-icon={l.icon}>
                                                <span className={styles.linkIcon}>{LINK_ICONS[l.icon]}</span>
                                                <span className={styles.linkText}>
                                                    <b>{l.label}</b>
                                                    <small>{l.note}</small>
                                                </span>
                                            </a>
                                        </li>
                                    ))}
                                </ul>
                            </div>
                        </div>
                    </section>
                </main>
            </div>
            </BlogWithCats>
        </Layout>
    );
}
