/**
 * Docusaurus Root 组件
 *
 * 全局 wrapper, 用于挂载需要跨页面持久化的组件
 * - 音乐播放器初始化
 * - ASS 歌词悬浮窗
 * - 按路由探头的立绘 RouteCats: 全站一份, 路由表 (含 off 与 404 回落) 在 BlogWithCats/index.tsx;
 *   页面自己挂了 SideCats 的 (博客 / cv / 友链 / 分享 / 番剧详情) 由挂载登记让位, 同页不会叠两组
 */
import { RouteCats } from '@site/src/components/BlogWithCats';
import AssLyrics from '@site/src/components/MusicPlayer/AssLyrics';
import { useMusicStore } from '@site/src/utils/music/musicStore';
import React, { useEffect } from 'react';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 * 立绘挂载点 (RouteCats 放在 children 之后, 不随路由卸载重建 Root)
 * .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md
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
            {/* 立绘: 放最后, fixed 定位, 不影响文档流 */}
            <RouteCats />
        </>
    );
}
