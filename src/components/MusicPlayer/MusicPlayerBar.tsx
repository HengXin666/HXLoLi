/**
 * 音乐播放器 Navbar 按钮 + 下拉面板
 *
 * 放在 Navbar 右侧 "更多" 按钮的前面
 */
import { useMusicStore, type PlayMode } from '@site/src/utils/music/musicStore';
import React, { useCallback, useEffect, useRef, useState } from 'react';
import {
    FaLock, FaLockOpen, FaMusic,
    FaPause,
    FaPlay,
    FaRandom,
    FaRedo,
    FaRetweet,
    FaStepBackward, FaStepForward,
    FaVolumeDown,
    FaVolumeMute,
    FaVolumeUp
} from 'react-icons/fa';
import styles from './MusicPlayerBar.module.css';

/** 歌词悬浮窗锁定状态的 localStorage key (与 AssLyrics.tsx 共享) */
const LOCK_KEY = 'hxloli-lyrics-locked';

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

/** 播放模式图标和描述 */
const PLAY_MODE_INFO: Record<PlayMode, { icon: React.ReactNode; title: string }> = {
    'list-loop': {
        icon: <FaRetweet size={14} />,
        title: '列表循环',
    },
    'single-loop': {
        icon: (
            <span style={{ position: 'relative', display: 'inline-flex', alignItems: 'center', justifyContent: 'center' }}>
                <FaRetweet size={14} />
                <span style={{ position: 'absolute', fontSize: 7, fontWeight: 'bold', top: '50%', left: '50%', transform: 'translate(-50%, -50%)' }}>1</span>
            </span>
        ),
        title: '单曲循环',
    },
    'shuffle': {
        icon: <FaRandom size={14} />,
        title: '随机播放',
    },
};

/** Navbar 上的播放按钮 (精简) */
export function MusicNavbarButton(): React.ReactElement | null {
    const pl = useMusicStore((s) => s.playlist);
    const isPlaying = useMusicStore((s) => s.isPlaying);
    const toggle = useMusicStore((s) => s.toggle);
    const togglePanel = useMusicStore((s) => s.togglePanel);
    const trackIndex = useMusicStore((s) => s.trackIndex);
    const panelExpanded = useMusicStore((s) => s.panelExpanded);

    const currentTrack = pl[trackIndex] ?? null;

    // 检测标题文字是否溢出（比较子元素宽度与父容器宽度）
    const navTitleContainerRef = useRef<HTMLButtonElement>(null);
    const navTitleRef = useRef<HTMLSpanElement>(null);
    const [navTitleOverflow, setNavTitleOverflow] = useState(false);

    useEffect(() => {
        const checkOverflow = () => {
            const container = navTitleContainerRef.current;
            const textEl = navTitleRef.current;
            if (container && textEl) {
                // 比较文字实际宽度和容器可用宽度（减去 playingDot 和 padding）
                setNavTitleOverflow(textEl.scrollWidth > container.clientWidth - 20);
            }
        };
        checkOverflow();
        window.addEventListener('resize', checkOverflow);
        return () => window.removeEventListener('resize', checkOverflow);
    }, [trackIndex, currentTrack?.title]);

    // 没有歌曲时不显示
    if (pl.length === 0) return null;

    return (
        <div className={styles.navbarBtn}>
            {/* 播放/暂停按钮 */}
            <button
                className={styles.playToggle}
                onClick={toggle}
                title={isPlaying ? '暂停' : '播放'}
            >
                {isPlaying ? <FaPause size={12} /> : <FaPlay size={12} style={{ marginLeft: 2 }} />}
            </button>

            {/* 歌曲标题 (点击展开面板) */}
            <button
                ref={navTitleContainerRef}
                className={`${styles.trackTitle} ${navTitleOverflow ? styles.navMarquee : ''}`}
                onClick={togglePanel}
                title="展开播放器"
            >
                <span
                    ref={navTitleRef}
                    className={`${styles.trackTitleText} ${navTitleOverflow ? styles.navMarqueeInner : ''}`}
                >
                    {currentTrack?.title || '未知'}
                </span>
                {/* 双份文本实现无缝循环滚动 */}
                {navTitleOverflow && (
                    <span
                        className={`${styles.trackTitleText} ${styles.navMarqueeInner}`}
                        aria-hidden="true"
                    >
                        {currentTrack?.title || '未知'}
                    </span>
                )}
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
    const playMode = useMusicStore((s) => s.playMode);

    const toggle = useMusicStore((s) => s.toggle);
    const next = useMusicStore((s) => s.next);
    const prev = useMusicStore((s) => s.prev);
    const seek = useMusicStore((s) => s.seek);
    const setVolume = useMusicStore((s) => s.setVolume);
    const setTrack = useMusicStore((s) => s.setTrack);
    const toggleLyrics = useMusicStore((s) => s.toggleLyrics);
    const closePanel = useMusicStore((s) => s.closePanel);
    const cyclePlayMode = useMusicStore((s) => s.cyclePlayMode);

    const panelRef = useRef<HTMLDivElement>(null);
    const currentTrack = pl[trackIndex] ?? null;

    // 歌词悬浮窗锁定状态 (从 localStorage 读取，与 AssLyrics 共享)
    const [lyricsLocked, setLyricsLocked] = useState(() => {
        try { return localStorage.getItem(LOCK_KEY) === 'true'; } catch { return false; }
    });

    // 静音功能：记录静音前的音量
    const [muted, setMuted] = useState(false);
    const prevVolumeRef = useRef(volume);

    // 监听 localStorage 变化同步锁定状态
    useEffect(() => {
        const handleStorage = (e: StorageEvent) => {
            if (e.key === LOCK_KEY && e.newValue !== null) {
                setLyricsLocked(e.newValue === 'true');
            }
        };
        window.addEventListener('storage', handleStorage);
        return () => window.removeEventListener('storage', handleStorage);
    }, []);

    // 点击外部关闭面板
    useEffect(() => {
        const handler = (e: MouseEvent) => {
            if (panelRef.current && !panelRef.current.contains(e.target as Node)) {
                const btn = (e.target as HTMLElement).closest(`.${styles.navbarBtn}`);
                if (!btn) {
                    closePanel();
                }
            }
        };
        const timer = setTimeout(() => {
            document.addEventListener('mousedown', handler);
        }, 100);
        return () => {
            clearTimeout(timer);
            document.removeEventListener('mousedown', handler);
        };
    }, [closePanel]);

    // 进度条拖拽
    const seekingRef = useRef(false);
    const seekValueRef = useRef(0);

    const handleSeekInput = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
        seekingRef.current = true;
        seekValueRef.current = parseFloat(e.target.value);
    }, []);

    const handleSeekChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
        const time = parseFloat(e.target.value);
        seekingRef.current = false;
        seek(time);
    }, [seek]);

    // 音量拖拽
    const handleVolume = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
        const vol = parseFloat(e.target.value);
        setVolume(vol);
        if (vol > 0) {
            setMuted(false);
            prevVolumeRef.current = vol;
        }
    }, [setVolume]);

    // 一键静音/取消静音
    const toggleMute = useCallback(() => {
        if (muted || volume === 0) {
            // 取消静音：恢复之前的音量
            const restoreVol = prevVolumeRef.current > 0 ? prevVolumeRef.current : 0.7;
            setVolume(restoreVol);
            setMuted(false);
        } else {
            // 静音
            prevVolumeRef.current = volume;
            setVolume(0);
            setMuted(true);
        }
    }, [muted, volume, setVolume]);

    // 切换歌词悬浮窗锁定状态
    const toggleLyricsLock = useCallback(() => {
        const next = !lyricsLocked;
        setLyricsLocked(next);
        try { localStorage.setItem(LOCK_KEY, JSON.stringify(next)); } catch { /* ignore */ }
        // 触发 storage 事件让 AssLyrics 同步（同一标签页需要手动 dispatch）
        window.dispatchEvent(new StorageEvent('storage', {
            key: LOCK_KEY,
            newValue: JSON.stringify(next),
        }));
    }, [lyricsLocked]);

    // 获取音量图标
    const VolumeIcon = volume === 0 || muted ? FaVolumeMute : volume < 0.5 ? FaVolumeDown : FaVolumeUp;

    // 判断文字是否需要滚动（用于歌名和歌手）
    // 用 hidden span 测量文本真实宽度，再与父容器宽度对比
    const nameRef = useRef<HTMLDivElement>(null);
    const artistRef = useRef<HTMLDivElement>(null);
    const nameTextRef = useRef<HTMLSpanElement>(null);
    const artistTextRef = useRef<HTMLSpanElement>(null);
    const [nameOverflow, setNameOverflow] = useState(false);
    const [artistOverflow, setArtistOverflow] = useState(false);

    useEffect(() => {
        // 延迟一帧检测，确保 DOM 已渲染
        const raf = requestAnimationFrame(() => {
            if (nameRef.current && nameTextRef.current) {
                setNameOverflow(nameTextRef.current.scrollWidth > nameRef.current.clientWidth);
            }
            if (artistRef.current && artistTextRef.current) {
                setArtistOverflow(artistTextRef.current.scrollWidth > artistRef.current.clientWidth);
            }
        });
        return () => cancelAnimationFrame(raf);
    }, [trackIndex, currentTrack?.title, currentTrack?.artist]);

    return (
        <div ref={panelRef} className={styles.panel}>
            {/* 当前曲目信息 */}
            <div className={styles.trackInfo}>
                {currentTrack?.coverUrl ? (
                    <img src={currentTrack.coverUrl} alt="" className={styles.cover} />
                ) : (
                    <div className={styles.coverPlaceholder}>
                        <FaMusic size={20} />
                    </div>
                )}
                <div className={styles.trackMeta}>
                    <div
                        ref={nameRef}
                        className={`${styles.trackName} ${nameOverflow ? styles.marquee : ''}`}
                    >
                        <span ref={nameTextRef} className={nameOverflow ? styles.marqueeInner : undefined}>
                            {currentTrack?.title || '无曲目'}
                        </span>
                        {nameOverflow && (
                            <span className={styles.marqueeInner} aria-hidden="true">
                                {currentTrack?.title || '无曲目'}
                            </span>
                        )}
                    </div>
                    <div
                        ref={artistRef}
                        className={`${styles.trackArtist} ${artistOverflow ? styles.marquee : ''}`}
                    >
                        <span ref={artistTextRef} className={artistOverflow ? styles.marqueeInner : undefined}>
                            {currentTrack?.artist || ''}
                        </span>
                        {artistOverflow && (
                            <span className={styles.marqueeInner} aria-hidden="true">
                                {currentTrack?.artist || ''}
                            </span>
                        )}
                    </div>
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
                    value={seekingRef.current ? seekValueRef.current : currentTime}
                    onInput={handleSeekInput}
                    onChange={handleSeekChange}
                    className={styles.progressBar}
                />
                <span className={styles.timeLabel}>{formatTime(duration)}</span>
            </div>

            {/* 控制按钮 */}
            <div className={styles.controls}>
                <button
                    onClick={cyclePlayMode}
                    className={styles.controlBtn}
                    title={PLAY_MODE_INFO[playMode].title}
                >
                    {PLAY_MODE_INFO[playMode].icon}
                </button>
                <button onClick={prev} className={styles.controlBtn} title="上一曲">
                    <FaStepBackward size={14} />
                </button>
                <button onClick={toggle} className={styles.controlBtnMain} title={isPlaying ? '暂停' : '播放'}>
                    {isPlaying ? <FaPause size={18} /> : <FaPlay size={18} style={{ marginLeft: 2 }} />}
                </button>
                <button onClick={next} className={styles.controlBtn} title="下一曲">
                    <FaStepForward size={14} />
                </button>
                {/* 歌词悬浮窗锁定按钮 */}
                {showLyrics && (
                    <button
                        onClick={toggleLyricsLock}
                        className={`${styles.controlBtn} ${lyricsLocked ? styles.controlBtnActive : ''}`}
                        title={lyricsLocked ? '解锁歌词悬浮窗' : '锁定歌词悬浮窗（全透明穿透）'}
                    >
                        {lyricsLocked ? <FaLock size={12} /> : <FaLockOpen size={12} />}
                    </button>
                )}
            </div>

            {/* 音量 + 歌词按钮 */}
            <div className={styles.bottomRow}>
                <div className={styles.volumeRow}>
                    <button
                        onClick={toggleMute}
                        className={styles.volumeBtn}
                        title={muted || volume === 0 ? '取消静音' : '静音'}
                    >
                        <VolumeIcon size={14} />
                    </button>
                    <input
                        type="range"
                        min={0}
                        max={1}
                        step={0.01}
                        value={volume}
                        onChange={handleVolume}
                        className={styles.volumeBar}
                    />
                    <span className={styles.volumeLabel}>{Math.round(volume * 100)}%</span>
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
                                <FaRedo size={10} />
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
