import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { stats } from '@site/data/wordStats';
import Layout from '@theme/Layout';
import React, { useCallback, useState } from 'react';

import {
    ECOSYSTEM_SINCE,
    featuredProjects,
    graphProjects,
    projects,
} from '@site/data/ecosystem';
import LatestDocsSection from '../LatestDocsSection';
import WakaTimeChart from '../chart/WakaTimeChart';
import WordCountChart from '../chart/WordCountChart';
import EcosystemGraph from './EcosystemGraph';
import ProjectDetail from './ProjectDetail';
import styles from './home.module.css';

const posts = [
    {
        date: '2026-10-07',
        title: '自用AI代码质量规范',
        excerpt: '把 AI 当同事来管: 用 Hook 把代码质量红线装进仓库, 而不是靠它自己发挥.',
        to: '/blog/2026/10/07/01-AI代码质量',
    },
    {
        date: '2026-09-06',
        title: '接受审判: I',
        excerpt: '转正面试的记录. 问的是业务, 没有算法和八股.',
        to: '/blog/2026/09/06/01_部门答辩',
    },
    {
        date: '2026-05-17',
        title: 'Codex Free 号池搭建',
        excerpt: '没事找事折腾出来的号池, 步骤和花掉的钱都记在这篇里.',
        to: '/blog/2026/05/17/01_codexFreePool',
    },
    {
        date: '2026-05-07',
        title: '最近的项目',
        excerpt: '按时间吹了一下水, 包括后来上线的简历台 HX-Resume.',
        to: '/blog/2026/05/07/01_最近的项目',
    },
];

const stack = ['C++20', 'Python', 'TypeScript', 'React', 'Qt / QML', 'Cloudflare', 'GitHub Actions', 'Linux', 'Docker'];

function FeatureCard({
    project,
    onSelect,
}: {
    project: (typeof featuredProjects)[number];
    onSelect: (id: string) => void;
}): React.ReactElement {
    const image = useBaseUrl(project.image);
    return (
        <button type="button" className={styles.feature} onClick={() => onSelect(project.id)}>
            <img
                src={image}
                alt=""
                className={project.image.endsWith('.svg') ? styles.contain : undefined}
            />
            <div className={styles.featureBody}>
                <h3>{project.name}</h3>
                <p>{project.summary}</p>
            </div>
        </button>
    );
}

const patterns = [
    {
        id: 'hx-cdn-forge',
        kicker: '博客要快',
        title: '加载痛了, 就做成 CDN 轮子',
        body: '主站的 package.json 依赖 HX-CDN-Forge. 音乐和番剧数据留在独立仓库, 由它选节点、分块拉取.',
    },
    {
        id: 'hx-git-db',
        kicker: '状态要有地方放',
        title: '私有仓库变成数据库',
        body: 'HX-Workflows 的 B 站接口、HX-Recall 和 HX-Video-Summary 都把 token 与进度写进 HX-Git-DB.',
    },
    {
        id: 'hx-houtiku',
        kicker: '邮件推送太疼',
        title: '清灰工具改走加密推送',
        body: 'HX-Recall 还留着邮件通知, 真正发出去的那条路是 HX-HouTiKu 的端到端推送.',
    },
];

/**
 * 首页以可编辑的项目生态图为中心
 * .agents/notes/implemented/feature/2026-10-31-homepage-ecosystem-graph.md
 */
export default function HomePage(): React.ReactElement {
    const { siteConfig } = useDocusaurusContext();
    const [selectedId, setSelectedId] = useState<string | null>(null);
    const close = useCallback(() => setSelectedId(null), []);
    const latest = stats[stats.length - 1];
    const words = latest ? Math.round(latest.wordCount / 10000) : 0;
    const wheels = graphProjects.filter((project) => project.layer === 'wheel').length;
    const hero = useBaseUrl('/img/main_menu_misaka.png');
    const catLeft = useBaseUrl('/default-img/neko_left.png');
    const catRight = useBaseUrl('/default-img/neko_right.png');

    return (
        <Layout
            title={siteConfig.title}
            description="Heng_Xin 的笔记、轮子和应用. 首页是一张可点击的项目生态图."
        >
            <div className={styles.page}>
                <header className={styles.hero}>
                    <div className={styles.heroInner}>
                        <div>
                            <p className={styles.kicker}>Heng_Xin 的实验室</p>
                            <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
                            <p className={styles.tagline}>{siteConfig.tagline}</p>
                            <p className={styles.lede}>
                                笔记、自研轮子和还在用的应用放在同一张图上. 应用里卡住的地方, 会被抽成下一层还能再用的库.
                            </p>
                            <div className={styles.actions}>
                                <a className={styles.primary} href="#ecosystem">看项目图</a>
                                <Link className={styles.ghost} to="/docs/关于">读笔记</Link>
                            </div>
                            <ul className={styles.stats}>
                                <li>
                                    <strong>{words} 万</strong>
                                    <span>笔记字数</span>
                                </li>
                                <li>
                                    <strong>{projects.length}</strong>
                                    <span>{ECOSYSTEM_SINCE} 之后仍在更新的仓库</span>
                                </li>
                                <li>
                                    <strong>{wheels}</strong>
                                    <span>抽出来的轮子</span>
                                </li>
                            </ul>
                        </div>
                        <div className={styles.heroFrame}>
                            <img src={hero} alt="博客首页插画" />
                        </div>
                    </div>
                </header>

                <section className={styles.section} id="ecosystem">
                    <div className={styles.sectionHead}>
                        <div>
                            <p className={styles.kicker}>自下而上</p>
                            <h2>项目生态</h2>
                        </div>
                        <p>
                            底层是学习和模板, 往上是轮子、平台、应用, 最上面是把它们收在一起的入口. 点任何一个名字.
                        </p>
                    </div>
                    <EcosystemGraph selectedId={selectedId} onSelect={setSelectedId} />
                    <p className={styles.caption}>
                        要加项目或改连线, 编辑 <code>data/ecosystem.ts</code>. 换封面只改对应的 <code>image</code> 路径.
                        虚线是推断, 实线能在仓库里指出文件.
                    </p>
                </section>

                <section className={styles.section} id="pattern">
                    <div className={styles.sectionHead}>
                        <div>
                            <p className={styles.kicker}>同一件事发生了三次</p>
                            <h2>痛点长成轮子</h2>
                        </div>
                    </div>
                    <div className={styles.patternGrid}>
                        {patterns.map((item) => (
                            <button
                                key={item.id}
                                type="button"
                                className={styles.pattern}
                                onClick={() => setSelectedId(item.id)}
                            >
                                <span>{item.kicker}</span>
                                <h3>{item.title}</h3>
                                <p>{item.body}</p>
                            </button>
                        ))}
                    </div>
                </section>

                <section className={styles.section} id="featured">
                    <div className={styles.sectionHead}>
                        <div>
                            <p className={styles.kicker}>先看这些</p>
                            <h2>代表项目</h2>
                        </div>
                    </div>
                    <div className={styles.featureGrid}>
                        {featuredProjects.map((project) => (
                            <FeatureCard key={project.id} project={project} onSelect={setSelectedId} />
                        ))}
                    </div>
                </section>

                <section className={styles.section} id="posts">
                    <div className={styles.sectionHead}>
                        <div>
                            <p className={styles.kicker}>博客</p>
                            <h2>最近的文章</h2>
                        </div>
                        <Link to="/blog">全部文章</Link>
                    </div>
                    <div className={styles.postGrid}>
                        {posts.map((post) => (
                            <Link key={post.to} className={styles.post} to={post.to}>
                                <time dateTime={post.date}>{post.date}</time>
                                <h3>{post.title}</h3>
                                <p>{post.excerpt}</p>
                            </Link>
                        ))}
                    </div>
                </section>

                <section className={styles.section} id="notes">
                    <div className={styles.sectionHead}>
                        <div>
                            <p className={styles.kicker}>知识库</p>
                            <h2>最近更新的笔记</h2>
                        </div>
                        <Link to="/docs/关于">关于这些笔记</Link>
                    </div>
                    <div className={styles.notesWrap}>
                        <LatestDocsSection />
                    </div>
                </section>

                <section className={styles.section} id="writing">
                    <div className={styles.sectionHead}>
                        <div>
                            <p className={styles.kicker}>还在写</p>
                            <h2>字数与编码时间</h2>
                        </div>
                    </div>
                    <div className={styles.chartGrid}>
                        <WordCountChart rawData={stats} />
                        <WakaTimeChart />
                    </div>
                </section>

                <section className={styles.section} id="about">
                    <div className={styles.aboutLayout}>
                        <div className={styles.aboutCard}>
                            <p className={styles.kicker}>关于</p>
                            <h3>Heng_Xin</h3>
                            <p>
                                写 C++ 网络库, 也写把 GitHub 当后端的小服务. 这份站从 HXLoLiHub 的主题长出来, 笔记、追番和带 ASS 歌词的播放器都留在这里.
                            </p>
                            <ul className={styles.stack}>
                                {stack.map((item) => (
                                    <li key={item}>{item}</li>
                                ))}
                            </ul>
                            <div className={styles.links}>
                                <a href="https://github.com/HengXin666" target="_blank" rel="noopener noreferrer">GitHub</a>
                                <a href="https://github.com/HengXin666/HXLoLi" target="_blank" rel="noopener noreferrer">本站仓库</a>
                                <a href="https://space.bilibili.com/478917126" target="_blank" rel="noopener noreferrer">哔哩哔哩</a>
                                <a href="https://leetcode.cn/u/heng_xin/" target="_blank" rel="noopener noreferrer">LeetCode</a>
                            </div>
                        </div>
                        <div className={styles.cats} aria-hidden="true">
                            <img src={catLeft} alt="" />
                            <img src={catRight} alt="" />
                        </div>
                    </div>
                </section>

                <section className={styles.section} id="roster">
                    <div className={styles.sectionHead}>
                        <div>
                            <p className={styles.kicker}>名录</p>
                            <h2>没有放进主图的仓库</h2>
                        </div>
                        <p>只有配置或测试桩. 点开仍能看到简介和仓库链接.</p>
                    </div>
                    <div className={styles.roster}>
                        {projects.filter((project) => !project.graph).map((project) => (
                            <button key={project.id} type="button" onClick={() => setSelectedId(project.id)}>
                                {project.name}
                            </button>
                        ))}
                    </div>
                </section>
            </div>
            <ProjectDetail projectId={selectedId} onSelect={setSelectedId} onClose={close} />
        </Layout>
    );
}
