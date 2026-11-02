/**
 * Docusaurus Root 组件
 *
 * 全局 wrapper, 用于挂载需要跨页面持久化的组件
 * - 音乐播放器初始化
 * - ASS 歌词悬浮窗
 * - 按路由探头的立绘 (首页 / 番剧库 / 番剧详情 / 关系图; 博客等页面自己挂 SideCats, 不在路由表里)
 */
import { RouteCats } from '@site/src/components/BlogWithCats';
import AssLyrics from '@site/src/components/MusicPlayer/AssLyrics';
import { useMusicStore } from '@site/src/utils/music/musicStore';
import React, { useEffect } from 'react';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
export default function Root({ children }: { children: React.ReactNode }): React.ReactElement {
    const init = useMusicStore((s) => s.init);
    const initialized = useMusicStore((s) => s.initialized);
    const pl = useMusicStore((s) => s.playlist);

    useEffect(() => {
        if (!initialized) {
            init();
        }
    }, [init, initialized]);

    return (
        <>
            {children}
            {/* 歌词悬浮窗 (全局挂载, 不随路由变化) */}
            {pl.length > 0 && <AssLyrics />}
            <RouteCats />
        </>
    );
}
