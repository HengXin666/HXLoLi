import useBaseUrl from '@docusaurus/useBaseUrl';
import React, { useEffect, useRef } from 'react';

import {
    edgeKinds,
    layers,
    projectById,
    relationsOf,
} from '@site/data/ecosystem';

import styles from './home.module.css';

type Props = {
    projectId: string | null;
    onSelect: (id: string) => void;
    onClose: () => void;
};

function relationLabel(direction: 'out' | 'in', kind: keyof typeof edgeKinds, otherName: string): string {
    if (kind === 'depends') {
        return direction === 'out' ? `依赖 ${otherName}` : `被 ${otherName} 依赖`;
    }
    if (kind === 'submodule') {
        return direction === 'out' ? `聚合 ${otherName}` : `被 ${otherName} 收为子模块`;
    }
    if (kind === 'derived') {
        return direction === 'out' ? `由 ${otherName} 演化而来` : `演化出 ${otherName}`;
    }
    return `与 ${otherName} 功能相关`;
}

export default function ProjectDetail({ projectId, onSelect, onClose }: Props): React.ReactElement | null {
    const closeRef = useRef<HTMLButtonElement>(null);
    const project = projectId ? projectById[projectId] : undefined;
    const image = useBaseUrl(project?.image ?? '/');

    useEffect(() => {
        if (!project) return undefined;
        const previous = document.body.style.overflow;
        document.body.style.overflow = 'hidden';
        closeRef.current?.focus();
        const onKey = (event: KeyboardEvent) => {
            if (event.key === 'Escape') onClose();
        };
        window.addEventListener('keydown', onKey);
        return () => {
            document.body.style.overflow = previous;
            window.removeEventListener('keydown', onKey);
        };
    }, [project, onClose]);

    if (!project) return null;

    const layer = layers.find((item) => item.id === project.layer);
    const relations = relationsOf(project.id);
    const placeholder = project.image.endsWith('.svg');

    return (
        <>
            <button type="button" className={styles.backdrop} aria-label="关闭项目详情" onClick={onClose} />
            <aside className={styles.panel} role="dialog" aria-modal="true" aria-labelledby="project-detail-title">
                <div className={styles.panelBar}>
                    <span>{layer?.label} · {layer?.hint}</span>
                    <button ref={closeRef} type="button" className={styles.close} onClick={onClose} aria-label="关闭">
                        ×
                    </button>
                </div>
                <img
                    src={image}
                    alt={project.name}
                    className={placeholder ? styles.contain : undefined}
                />
                <div className={styles.panelBody}>
                    <h2 id="project-detail-title">{project.name}</h2>
                    <p>{project.summary}</p>
                    {project.stack.length > 0 && (
                        <ul className={styles.chips}>
                            {project.stack.map((item) => (
                                <li key={item}>{item}</li>
                            ))}
                        </ul>
                    )}
                    <div className={styles.panelLinks}>
                        <a href={project.repo} target="_blank" rel="noopener noreferrer">GitHub</a>
                        {project.demo && (
                            <a href={project.demo} target="_blank" rel="noopener noreferrer">演示</a>
                        )}
                        {project.docs && (
                            <a href={project.docs} target="_blank" rel="noopener noreferrer">文档</a>
                        )}
                    </div>
                    <p className={styles.caption}>最近推送 {project.pushedAt}</p>
                    {relations.length > 0 && (
                        <ul className={styles.relations}>
                            {relations.map((relation) => {
                                const other = projectById[relation.otherId];
                                if (!other) return null;
                                return (
                                    <li key={`${relation.direction}-${relation.kind}-${relation.otherId}`}>
                                        <button type="button" onClick={() => onSelect(other.id)}>
                                            <strong>
                                                {relationLabel(relation.direction, relation.kind, other.name)}
                                                {relation.inferred ? ' · 推断' : ''}
                                            </strong>
                                            <small>{relation.evidence}</small>
                                        </button>
                                    </li>
                                );
                            })}
                        </ul>
                    )}
                </div>
            </aside>
        </>
    );
}
