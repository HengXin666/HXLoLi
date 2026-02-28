/**
 * 音乐播放器 Navbar 按钮 + 下拉面板
 *
 * 放在 Navbar 右侧 "更多" 按钮的前面
 */
import { useMusicStore } from '@site/src/utils/music/musicStore';
import React, { useCallback, useEffect, useRef } from 'react';
import styles from './MusicPlayerBar.module.css';

/** 触发 ASS 歌词悬浮窗位置重置 */
function resetLyricsPosition(): void {
    window.dispatchEvent(new CustomEvent('hxloli-lyrics-reset-position'));
}

/** 格式化时间 mm:ss */
function formatTime(seconds: number): string {
    if (!seconds || !isFinite(seconds)) return '0:00';
    const m = Math.floor(seconds / 60);
    const s = Math.floor(seconds % 60);
    return `${m}:${s.toString().padStart(2, '0')}`;
}

/** Navbar 上的播放按钮 (精简) */
export function MusicNavbarButton(): React.ReactElement | null {
    const pl = useMusicStore((s) => s.playlist);
    const isPlaying = useMusicStore((s) => s.isPlaying);
    const toggle = useMusicStore((s) => s.toggle);
    const togglePanel = useMusicStore((s) => s.togglePanel);
    const trackIndex = useMusicStore((s) => s.trackIndex);
    const panelExpanded = useMusicStore((s) => s.panelExpanded);

    // 没有歌曲时不显示
    if (pl.length === 0) return null;

    const currentTrack = pl[trackIndex];

    return (
        <div className={styles.navbarBtn}>
            {/* 播放/暂停按钮 */}
            <button
                className={styles.playToggle}
                onClick={toggle}
                title={isPlaying ? '暂停' : '播放'}
            >
                {isPlaying ? (
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                        <rect x="6" y="4" width="4" height="16" rx="1" />
                        <rect x="14" y="4" width="4" height="16" rx="1" />
                    </svg>
                ) : (
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M8 5v14l11-7z" />
                    </svg>
                )}
            </button>

            {/* 歌曲标题 (点击展开面板) */}
            <button
                className={styles.trackTitle}
                onClick={togglePanel}
                title="展开播放器"
            >
                <span className={styles.trackTitleText}>
                    {currentTrack?.title || '未知'}
                </span>
                {isPlaying && <span className={styles.playingDot} />}
            </button>

            {/* 下拉面板 */}
            {panelExpanded && <MusicPanel />}
        </div>
    );
}

/** 播放器下拉面板 */
function MusicPanel(): React.ReactElement {
    const pl = useMusicStore((s) => s.playlist);
    const trackIndex = useMusicStore((s) => s.trackIndex);
    const currentTime = useMusicStore((s) => s.currentTime);
    const duration = useMusicStore((s) => s.duration);
    const isPlaying = useMusicStore((s) => s.isPlaying);
    const volume = useMusicStore((s) => s.volume);
    const showLyrics = useMusicStore((s) => s.showLyrics);

    const toggle = useMusicStore((s) => s.toggle);
    const next = useMusicStore((s) => s.next);
    const prev = useMusicStore((s) => s.prev);
    const seek = useMusicStore((s) => s.seek);
    const setVolume = useMusicStore((s) => s.setVolume);
    const setTrack = useMusicStore((s) => s.setTrack);
    const toggleLyrics = useMusicStore((s) => s.toggleLyrics);
    const closePanel = useMusicStore((s) => s.closePanel);

    const panelRef = useRef<HTMLDivElement>(null);
    const currentTrack = pl[trackIndex] ?? null;

    // 点击外部关闭面板
    useEffect(() => {
        const handler = (e: MouseEvent) => {
            if (panelRef.current && !panelRef.current.contains(e.target as Node)) {
                // 检查是否点击了 navbar 按钮区域
                const btn = (e.target as HTMLElement).closest(`.${styles.navbarBtn}`);
                if (!btn) {
                    closePanel();
                }
            }
        };
        // 延迟添加，防止当前点击立即触发
        const timer = setTimeout(() => {
            document.addEventListener('mousedown', handler);
        }, 100);
        return () => {
            clearTimeout(timer);
            document.removeEventListener('mousedown', handler);
        };
    }, [closePanel]);

    // 进度条拖拽
    const handleSeek = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
        seek(parseFloat(e.target.value));
    }, [seek]);

    // 音量拖拽
    const handleVolume = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
        setVolume(parseFloat(e.target.value));
    }, [setVolume]);

    return (
        <div ref={panelRef} className={styles.panel}>
            {/* 当前曲目信息 */}
            <div className={styles.trackInfo}>
                {currentTrack?.coverUrl ? (
                    <img src={currentTrack.coverUrl} alt="" className={styles.cover} />
                ) : (
                    <div className={styles.coverPlaceholder}>🎵</div>
                )}
                <div className={styles.trackMeta}>
                    <div className={styles.trackName}>{currentTrack?.title || '无曲目'}</div>
                    <div className={styles.trackArtist}>{currentTrack?.artist || ''}</div>
                </div>
            </div>

            {/* 进度条 */}
            <div className={styles.progressRow}>
                <span className={styles.timeLabel}>{formatTime(currentTime)}</span>
                <input
                    type="range"
                    min={0}
                    max={duration || 0}
                    step={0.1}
                    value={currentTime}
                    onChange={handleSeek}
                    className={styles.progressBar}
                />
                <span className={styles.timeLabel}>{formatTime(duration)}</span>
            </div>

            {/* 控制按钮 */}
            <div className={styles.controls}>
                <button onClick={prev} className={styles.controlBtn} title="上一曲">
                    <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M6 6h2v12H6zm3.5 6l8.5 6V6z" />
                    </svg>
                </button>
                <button onClick={toggle} className={styles.controlBtnMain} title={isPlaying ? '暂停' : '播放'}>
                    {isPlaying ? (
                        <svg width="22" height="22" viewBox="0 0 24 24" fill="currentColor">
                            <rect x="6" y="4" width="4" height="16" rx="1" />
                            <rect x="14" y="4" width="4" height="16" rx="1" />
                        </svg>
                    ) : (
                        <svg width="22" height="22" viewBox="0 0 24 24" fill="currentColor">
                            <path d="M8 5v14l11-7z" />
                        </svg>
                    )}
                </button>
                <button onClick={next} className={styles.controlBtn} title="下一曲">
                    <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M6 18l8.5-6L6 6v12zm2 0h2V6h-2v12z" transform="scale(-1,1) translate(-24,0)" />
                    </svg>
                </button>
            </div>

            {/* 音量 + 歌词按钮 */}
            <div className={styles.bottomRow}>
                <div className={styles.volumeRow}>
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" opacity={0.6}>
                        <path d="M3 9v6h4l5 5V4L7 9H3zm13.5 3c0-1.77-1.02-3.29-2.5-4.03v8.05c1.48-.73 2.5-2.25 2.5-4.02z" />
                    </svg>
                    <input
                        type="range"
                        min={0}
                        max={1}
                        step={0.01}
                        value={volume}
                        onChange={handleVolume}
                        className={styles.volumeBar}
                    />
                </div>
                {currentTrack?.assUrl && (
                    <>
                        <button
                            onClick={toggleLyrics}
                            className={`${styles.lyricsBtn} ${showLyrics ? styles.lyricsBtnActive : ''}`}
                            title={showLyrics ? '隐藏歌词' : '显示歌词'}
                        >
                            词
                        </button>
                        {showLyrics && (
                            <button
                                onClick={resetLyricsPosition}
                                className={styles.lyricsBtn}
                                title="重置歌词悬浮窗位置"
                            >
                                ↺
                            </button>
                        )}
                    </>
                )}
            </div>

            {/* 播放列表 */}
            {pl.length >= 1 && (
                <div className={styles.playlistSection}>
                    <div className={styles.playlistTitle}>播放列表</div>
                    <div className={styles.playlistItems}>
                        {pl.map((track, i) => (
                            <button
                                key={track.id}
                                className={`${styles.playlistItem} ${i === trackIndex ? styles.playlistItemActive : ''}`}
                                onClick={() => setTrack(i)}
                            >
                                <span className={styles.playlistItemIndex}>{i + 1}</span>
                                <span className={styles.playlistItemTitle}>{track.title}</span>
                                <span className={styles.playlistItemArtist}>{track.artist}</span>
                            </button>
                        ))}
                    </div>
                </div>
            )}
        </div>
    );
}
