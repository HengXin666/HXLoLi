import useBaseUrl from '@docusaurus/useBaseUrl';
import React from 'react';
import { PROJECT_SINCE, projects, Tags, type Project } from '@site/data/projectData';

import Tooltip from '../common/Tooltip';
import './css.css';

type MoreEntry = {
    open: boolean;
    onToggle: () => void;
};

/** 卡片的悬浮内容: 完整简介 (卡片里截成三行)、标签含义、徽章数字和最近推送 */
function CardHover ({ project }: { project: Project }) {
    return (
        <div className="pc-hover">
            <strong>{project.title}</strong>
            <p>{project.description}</p>
            <ul>
                {project.tags.map((tag) => (
                    <li key={tag}>
                        <i style={{ background: Tags[tag].color }} />
                        <b>{Tags[tag].label}</b> {Tags[tag].description}
                    </li>
                ))}
            </ul>
            <span className="pc-hover__meta">
                {(project.stats ?? []).filter((s) => s !== project.pushedAt).join(' · ')}
                {project.stats?.some((s) => s !== project.pushedAt) ? ' · ' : ''}最近推送 {project.pushedAt}
            </span>
        </div>
    );
}

function Card ({ project, hidden = false }: { project: Project; hidden?: boolean }) {
    const cover = useBaseUrl(project.preview ? `/${project.preview.replace(/^\//, '')}` : '/');
    const card = (
        <a className="pc-card" href={project.url} target="_blank" rel="noopener noreferrer" tabIndex={hidden ? -1 : undefined}>
            {project.preview ? (
                <img className="pc-cover" src={cover} alt="" loading="lazy" decoding="async" width={240} height={120} />
            ) : (
                <span className="pc-cover pc-cover--blank" aria-hidden="true">{project.title.slice(0, 1)}</span>
            )}
            <span className="pc-body">
                <span className="pc-title">{project.title}</span>
                <span className="pc-desc">{project.description}</span>
            </span>
            <span className="pc-foot">
                <span className="pc-tags">
                    {project.tags.slice(0, 2).map((tag) => (
                        <span key={tag} className="pc-tag">
                            <i style={{ background: Tags[tag].color }} />
                            {Tags[tag].label}
                        </span>
                    ))}
                </span>
                <span className="pc-meta">
                    <span className="pc-stars" aria-label={`GitHub 星标 ${project.stars}`}>★ {project.stars}</span>
                    <time className="pc-date">{project.pushedAt}</time>
                </span>
            </span>
        </a>
    );
    // 复制出来做无缝循环的那一份不挂悬浮卡片, 也不进 Tab 顺序
    if (hidden) return card;
    return (
        <Tooltip width={300} content={<CardHover project={project} />}>
            {card}
        </Tooltip>
    );
}

/**
 * 近一年项目的连续滚动条: 纯 CSS 跑马灯 (两份列表 translateX -50%), 不再依赖 swiper.
 * 卡片按 pushedAt 倒序, 早于 PROJECT_SINCE 的不上条带. 悬停或键盘聚焦时暂停,
 * prefers-reduced-motion 下改为可横向滑动的静态列表.
 * 关系图入口是条带下方常驻的"更多"横条, 不放进循环滚动的卡片里.
 * 悬停 / 聚焦卡片出统一悬浮卡片 (完整简介、标签含义、徽章), 原来卡片和标签上的原生 title 删掉.
 * 卡片底栏直接显示 GitHub 星数, 读 projectData 的 stars 字段 (不从 stats 字符串里解析), 0 星也显示 "★ 0".
 * .agents/notes/implemented/feature/2026-10-31-homepage-profile-and-project-index.md
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
const ProjectCarousel: React.FC<{ more?: MoreEntry }> = ({ more }) => {
    const recent = projects
        .filter((p) => p.pushedAt >= PROJECT_SINCE)
        .sort((a, b) => b.pushedAt.localeCompare(a.pushedAt));
    // 每张卡 4 秒, 卡越多转得越慢, 速度恒定
    const duration = `${recent.length * 4}s`;

    return (
        <div className="pc">
            <div className="pc-viewport">
                <div className="pc-track" style={{ animationDuration: duration }}>
                    {recent.map((p) => <Card key={p.title} project={p} />)}
                    <span className="pc-dup" aria-hidden="true">
                        {recent.map((p) => <Card key={`dup-${p.title}`} project={p} hidden />)}
                    </span>
                </div>
            </div>
            {more && (
                <button type="button" className="pc-more" aria-expanded={more.open} onClick={more.onToggle}>
                    <span className="pc-more__hint">{recent.length} 个近一年在推的公开仓库 · 60+ 个仓库的依赖、子模块与派生关系</span>
                    <span className="pc-more__action">{more.open ? '收起关系图' : '更多 · 项目关系图'}</span>
                </button>
            )}
        </div>
    );
};

export default ProjectCarousel;
