/**
 * Docusaurus Root 组件
 *
 * 全局 wrapper, 用于挂载需要跨页面持久化的组件
 * - 音乐播放器初始化
 * - ASS 歌词悬浮窗
 */
import AssLyrics from '@site/src/components/MusicPlayer/AssLyrics';
import { useMusicStore } from '@site/src/utils/music/musicStore';
import React, { useEffect } from 'react';

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
        </>
    );
}
