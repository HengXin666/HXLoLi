import React, { useEffect, useState } from 'react';
import styles from './graph.module.css';

// 关系图的加载 / 错误 / 空态覆盖层. 数据仓 (animeStore) 不暴露字节进度, 这里用不定进度条 + 已用时 + 阶段文案给出进度感

interface LoadingProps {
    /** 当前阶段文案, 如 "正在下载番剧数据" / "正在布局关系图" */
    stage: string;
    /** 补充说明, 如数据大小 */
    detail?: string;
}

export const GraphLoading: React.FC<LoadingProps> = ({ stage, detail }) => {
    const [seconds, setSeconds] = useState(0);
    useEffect(() => {
        const started = Date.now();
        const timer = window.setInterval(() => setSeconds(Math.floor((Date.now() - started) / 1000)), 500);
        return () => window.clearInterval(timer);
    }, []);

    return (
        <div className={styles.status} role="status" aria-live="polite">
            <div className={styles.statusCard}>
                <div className={styles.statusTitle}>{stage}</div>
                <div className={styles.progressTrack} aria-hidden="true">
                    <div className={styles.progressBar} />
                </div>
                <div className={styles.statusMeta}>
                    {detail && <span>{detail}</span>}
                    <span className={styles.tabular}>已用时 {seconds}s</span>
                </div>
                {seconds >= 15 && (
                    <div className={styles.statusHint}>数据较大, 首次加载会慢一些; 加载完成后会缓存在本页会话里.</div>
                )}
            </div>
        </div>
    );
};

interface ErrorProps {
    message: string | null;
    onRetry: () => void;
}

export const GraphError: React.FC<ErrorProps> = ({ message, onRetry }) => (
    <div className={styles.status} role="alert">
        <div className={styles.statusCard}>
            <div className={styles.statusTitle}>番剧数据加载失败</div>
            {message && <div className={styles.statusError}>{message}</div>}
            <div className={styles.statusActions}>
                <button type="button" className={styles.button} onClick={onRetry}>重试</button>
            </div>
        </div>
    </div>
);

interface EmptyProps {
    title: string;
    detail?: string;
    actionLabel?: string;
    onAction?: () => void;
}

export const GraphEmpty: React.FC<EmptyProps> = ({ title, detail, actionLabel, onAction }) => (
    <div className={styles.status}>
        <div className={styles.statusCard}>
            <div className={styles.statusTitle}>{title}</div>
            {detail && <div className={styles.statusMeta}><span>{detail}</span></div>}
            {actionLabel && onAction && (
                <div className={styles.statusActions}>
                    <button type="button" className={styles.button} onClick={onAction}>{actionLabel}</button>
                </div>
            )}
        </div>
    </div>
);
