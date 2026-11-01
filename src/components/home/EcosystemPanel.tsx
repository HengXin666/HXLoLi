import React, { useCallback, useEffect, useRef, useState } from 'react';

import { edges, graphProjects } from '@site/data/ecosystem';

import EcosystemGraph from './EcosystemGraph';
import ProjectDetail from './ProjectDetail';
import styles from './graph.module.css';

type Props = {
    onClose: () => void;
};

/**
 * 关系图浮层: 满宽 (最宽 1320px)、高度不超过 92vh 的对话框, 里面是关系图, 点节点再叠一层详情卡.
 * 关系图、详情卡和 data/ecosystem.ts 打成一个懒加载块, 只在点开"更多"后才下载.
 * Esc 先关详情卡, 再关浮层; 打开时锁住页面滚动, 关闭后焦点回到"更多"按钮 (由 HomePage 负责).
 * 关闭也有过渡: 先播 180ms 淡出下沉 (data-closing), 动画结束再真正卸载; 减弱动效时直接关.
 * 首页关系图的决策见同目录 HomePage.tsx 顶部注释
 * .agents/notes/implemented/feature/2026-11-01-home-graph-close-transition.md
 */
export default function EcosystemPanel ({ onClose }: Props): React.ReactElement {
    const [selectedId, setSelectedId] = useState<string | null>(null);
    const closeDetail = useCallback(() => setSelectedId(null), []);
    const sheetRef = useRef<HTMLDivElement>(null);
    const [closing, setClosing] = useState(false);

    const requestClose = useCallback(() => {
        const reduce = typeof window !== 'undefined' && window.matchMedia?.('(prefers-reduced-motion: reduce)').matches;
        if (reduce) onClose();
        else setClosing(true);
    }, [onClose]);

    // animationend 万一没来 (标签页在后台等), 兜底卸载
    useEffect(() => {
        if (!closing) return undefined;
        const t = window.setTimeout(onClose, 320);
        return () => window.clearTimeout(t);
    }, [closing, onClose]);

    useEffect(() => {
        const previous = document.body.style.overflow;
        document.body.style.overflow = 'hidden';
        sheetRef.current?.focus();
        return () => {
            document.body.style.overflow = previous;
        };
    }, []);

    useEffect(() => {
        if (selectedId) return undefined; // 详情卡自己处理 Esc
        const onKey = (e: KeyboardEvent) => {
            if (e.key === 'Escape') requestClose();
        };
        window.addEventListener('keydown', onKey);
        return () => window.removeEventListener('keydown', onKey);
    }, [selectedId, requestClose]);

    return (
        <>
            <div className={styles.scrim} data-closing={closing || undefined} onClick={requestClose} aria-hidden="true" />
            <div
                ref={sheetRef}
                className={styles.sheet}
                role="dialog"
                aria-modal="true"
                aria-labelledby="eco-title"
                tabIndex={-1}
                data-closing={closing || undefined}
                onAnimationEnd={(e) => { if (closing && e.target === e.currentTarget) onClose(); }}
            >
                <header className={styles.sheetHead}>
                    <div>
                        <h2 id="eco-title">项目关系图</h2>
                        <p>{graphProjects.length} 个仓库 · {edges.length} 条依赖、子模块与派生关系 · 行是产品线, 列从源头到门户</p>
                    </div>
                    <button type="button" className={styles.sheetClose} onClick={requestClose} aria-label="关闭关系图">
                        <svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true">
                            <path d="M6 6l12 12M18 6L6 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                        </svg>
                    </button>
                </header>
                <EcosystemGraph selectedId={selectedId} onSelect={setSelectedId} />
                <p className={styles.sheetFoot}>悬停或用 Tab 聚焦节点看简介与关系依据, 点击看全部关系 · Esc 关闭</p>
            </div>
            <ProjectDetail projectId={selectedId} onSelect={setSelectedId} onClose={closeDetail} />
        </>
    );
}
