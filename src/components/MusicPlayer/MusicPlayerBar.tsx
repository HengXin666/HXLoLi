/**
 * 音乐播放器 Navbar 按钮 + 下拉面板
 *
 * 放在 Navbar 右侧 "更多" 按钮的前面. 视觉沿用首页: 1px emphasis-200 细线, 6-12px 圆角,
 * 粉色 (--ifm-color-primary) 是唯一强调色, 数字用等宽 tabular, 11px 小标签, 无渐变无发光.
 */
import type { MusicTrack } from '@site/src/config/musicData';
import { pickLyric } from '@site/src/utils/music/musicDataLoader';
import { useMusicStore, type PlayMode } from '@site/src/utils/music/musicStore';
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
    FaLock, FaLockOpen, FaMusic,
    FaPause,
    FaPlay,
    FaRandom,
    FaRedo,
    FaRetweet,
    FaSearch,
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

/** 格式化时间 m:ss */
function formatTime(seconds: number): string {
    if (!seconds || !isFinite(seconds) || seconds < 0) return '0:00';
    const m = Math.floor(seconds / 60);
    const s = Math.floor(seconds % 60);
    return `${m}:${s.toString().padStart(2, '0')}`;
}

/** 文件扩展名 -> 展示用格式名 */
function formatOf(url: string | undefined): string {
    const ext = url?.split(/[?#]/)[0].split('.').pop()?.toLowerCase() ?? '';
    return ({ flac: 'FLAC', mp3: 'MP3', m4a: 'AAC', aac: 'AAC', ogg: 'OGG', opus: 'OPUS', wav: 'WAV', webm: 'OPUS' } as Record<string, string>)[ext] ?? '';
}

const PLAY_MODE_INFO: Record<PlayMode, { icon: React.ReactNode; title: string }> = {
    'list-loop': { icon: <FaRetweet size={14} />, title: '列表循环' },
    'single-loop': {
        icon: (
            <span className={styles.modeOne}>
                <FaRetweet size={14} />
                <span>1</span>
            </span>
        ),
        title: '单曲循环',
    },
    'shuffle': { icon: <FaRandom size={13} />, title: '随机播放' },
};

/** 正在播放的跳动小条 (暂停时静止) */
function EqBars({ playing }: { playing: boolean }): React.ReactElement {
    return (
        <span className={styles.eq} data-playing={playing || undefined} aria-hidden="true">
            <i /><i /><i />
        </span>
    );
}

/**
 * 横向拖动条: 进度和音量共用.
 * 拖动时只改本地值, 松手才提交 (live 时边拖边提交, 给音量用); 悬停显示该位置的值.
 */
function Scrubber({ value, max, buffered = 0, onCommit, live = false, format, label, disabled = false }: {
    value: number;
    max: number;
    buffered?: number;
    onCommit: (v: number) => void;
    live?: boolean;
    format: (v: number) => string;
    label: string;
    disabled?: boolean;
}): React.ReactElement {
    const ref = useRef<HTMLDivElement>(null);
    const [drag, setDrag] = useState<number | null>(null);
    const [hover, setHover] = useState<number | null>(null);
    const safeMax = max > 0 && isFinite(max) ? max : 0;
    const shown = drag ?? value;
    const ratio = safeMax ? Math.min(1, Math.max(0, shown / safeMax)) : 0;
    const bufRatio = safeMax ? Math.min(1, Math.max(0, buffered / safeMax)) : 0;

    const valueAt = (clientX: number): number => {
        const r = ref.current?.getBoundingClientRect();
        if (!r || r.width === 0) return 0;
        return Math.min(1, Math.max(0, (clientX - r.left) / r.width)) * safeMax;
    };

    const onPointerDown = (e: React.PointerEvent<HTMLDivElement>): void => {
        if (disabled || !safeMax || e.button !== 0) return;
        e.currentTarget.setPointerCapture(e.pointerId);
        const v = valueAt(e.clientX);
        setDrag(v);
        if (live) onCommit(v);
    };
    const onPointerMove = (e: React.PointerEvent<HTMLDivElement>): void => {
        if (!safeMax) return;
        const v = valueAt(e.clientX);
        setHover(v);
        if (drag !== null) {
            setDrag(v);
            if (live) onCommit(v);
        }
    };
    const onPointerUp = (e: React.PointerEvent<HTMLDivElement>): void => {
        if (drag === null) return;
        const v = valueAt(e.clientX);
        setDrag(null);
        onCommit(v);
    };
    const onKeyDown = (e: React.KeyboardEvent<HTMLDivElement>): void => {
        if (disabled || !safeMax) return;
        const step = safeMax <= 1 ? 0.05 : 5;
        if (e.key === 'ArrowRight' || e.key === 'ArrowUp') { e.preventDefault(); onCommit(Math.min(safeMax, value + step)); }
        if (e.key === 'ArrowLeft' || e.key === 'ArrowDown') { e.preventDefault(); onCommit(Math.max(0, value - step)); }
    };

    const tip = drag ?? hover;
    return (
        <div
            ref={ref}
            className={styles.scrub}
            data-dragging={drag !== null || undefined}
            data-disabled={disabled || !safeMax || undefined}
            role="slider"
            tabIndex={disabled ? -1 : 0}
            aria-label={label}
            aria-valuemin={0}
            aria-valuemax={safeMax}
            aria-valuenow={shown}
            aria-valuetext={format(shown)}
            onPointerDown={onPointerDown}
            onPointerMove={onPointerMove}
            onPointerUp={onPointerUp}
            onPointerCancel={() => setDrag(null)}
            onPointerLeave={() => setHover(null)}
            onKeyDown={onKeyDown}
        >
            <span className={styles.scrubTrack}>
                <span className={styles.scrubBuf} style={{ width: `${bufRatio * 100}%` }} />
                <span className={styles.scrubFill} style={{ width: `${ratio * 100}%` }} />
            </span>
            <span className={styles.scrubThumb} style={{ left: `${ratio * 100}%` }} />
            {tip !== null && safeMax > 0 && (
                <span className={styles.scrubTip} style={{ left: `${Math.min(1, tip / safeMax) * 100}%` }}>{format(tip)}</span>
            )}
        </div>
    );
}

/** 宽度不够时无缝横向滚动的文字 */
function Marquee({ text, className }: { text: string; className?: string }): React.ReactElement {
    const boxRef = useRef<HTMLDivElement>(null);
    const textRef = useRef<HTMLSpanElement>(null);
    const [overflow, setOverflow] = useState(false);
    useEffect(() => {
        const check = (): void => {
            if (boxRef.current && textRef.current) setOverflow(textRef.current.scrollWidth > boxRef.current.clientWidth + 1);
        };
        const raf = requestAnimationFrame(check);
        window.addEventListener('resize', check);
        return () => { cancelAnimationFrame(raf); window.removeEventListener('resize', check); };
    }, [text]);
    return (
        <div ref={boxRef} className={`${className ?? ''} ${styles.marqueeBox} ${overflow ? styles.marquee : ''}`} title={text}>
            <span ref={textRef} className={overflow ? styles.marqueeInner : undefined}>{text}</span>
            {overflow && <span className={styles.marqueeInner} aria-hidden="true">{text}</span>}
        </div>
    );
}

/**
 * Navbar 上的播放按钮 (胶囊: 播放键 + 歌名 + 底部细进度线), 点歌名展开面板.
 * .agents/notes/implemented/feature/2026-11-24-music-player-v2.md
 */
export function MusicNavbarButton(): React.ReactElement | null {
    const pl = useMusicStore((s) => s.playlist);
    const isPlaying = useMusicStore((s) => s.isPlaying);
    const toggle = useMusicStore((s) => s.toggle);
    const togglePanel = useMusicStore((s) => s.togglePanel);
    const trackIndex = useMusicStore((s) => s.trackIndex);
    const panelExpanded = useMusicStore((s) => s.panelExpanded);
    const currentTime = useMusicStore((s) => s.currentTime);
    const duration = useMusicStore((s) => s.duration);
    const audioStatus = useMusicStore((s) => s.audioStatus);

    const currentTrack = pl[trackIndex] ?? null;
    if (pl.length === 0) return null;
    const ratio = duration > 0 ? Math.min(1, currentTime / duration) : 0;
    const busy = audioStatus === 'loading' || audioStatus === 'downloading';

    return (
        <div className={styles.navbarBtn} data-open={panelExpanded || undefined}>
            <button
                type="button"
                className={styles.playToggle}
                onClick={toggle}
                title={isPlaying ? '暂停' : '播放'}
                aria-label={isPlaying ? '暂停' : '播放'}
                data-busy={(busy && isPlaying) || undefined}
            >
                {isPlaying ? <FaPause size={10} /> : <FaPlay size={10} style={{ marginLeft: 2 }} />}
            </button>
            <button
                type="button"
                className={styles.navTitle}
                onClick={togglePanel}
                title="展开播放器"
                aria-expanded={panelExpanded}
            >
                <Marquee text={currentTrack?.title || '未知'} className={styles.navTitleText} />
                <EqBars playing={isPlaying} />
                <span className={styles.navProgress} style={{ transform: `scaleX(${ratio})` }} aria-hidden="true" />
            </button>
            {panelExpanded && <MusicPanel />}
        </div>
    );
}

interface TrackGroup {
    key: string;
    title: string;
    cover?: string;
    items: { track: MusicTrack; index: number }[];
}

const OTHER_KEY = '\u0000other';

/** 按 anime.title 分组; 组序 = 组内第一首在原列表中的位置, "其它" (anime 为空) 放最后; 不改歌曲原顺序 */
function groupByAnime(pl: readonly MusicTrack[]): TrackGroup[] {
    const map = new Map<string, TrackGroup>();
    pl.forEach((track, index) => {
        const title = track.anime?.title;
        const key = title ?? OTHER_KEY;
        let g = map.get(key);
        if (!g) {
            g = { key, title: title ?? '其它', items: [] };
            map.set(key, g);
        }
        if (!g.cover && track.coverUrl) g.cover = track.coverUrl;
        g.items.push({ track, index });
    });
    const groups = [...map.values()];
    const other = groups.findIndex((g) => g.key === OTHER_KEY);
    if (other >= 0) groups.push(groups.splice(other, 1)[0]);
    return groups;
}

/**
 * 播放列表树: 一级 = 番剧 (封面缩略图 / 番名 / 曲数 / 箭头), 二级 = 该番的歌 (原序号).
 * 默认只展开正在播放的组, 切到别的组的歌时自动展开那组; 只有一组时退化为平铺列表.
 * 有搜索词时平铺显示命中的歌 (标题 / 歌手 / 番名).
 * .agents/notes/implemented/feature/2026-11-02-music-anime-tree.md
 */
function PlaylistTree({ playlist, trackIndex, isPlaying, onPick }: {
    playlist: readonly MusicTrack[];
    trackIndex: number;
    isPlaying: boolean;
    onPick: (i: number) => void;
}): React.ReactElement {
    const groups = useMemo(() => groupByAnime(playlist), [playlist]);
    const currentKey = playlist[trackIndex]?.anime?.title ?? OTHER_KEY;
    const [open, setOpen] = useState<ReadonlySet<string>>(() => new Set([currentKey]));
    const [query, setQuery] = useState('');
    const activeRef = useRef<HTMLButtonElement>(null);
    useEffect(() => {
        setOpen((s) => (s.has(currentKey) ? s : new Set(s).add(currentKey)));
    }, [currentKey]);
    // 打开面板时把正在播放的那首滚进视野
    useEffect(() => {
        activeRef.current?.scrollIntoView({ block: 'nearest' });
    }, []);
    const flip = (key: string): void => setOpen((s) => {
        const n = new Set(s);
        if (n.has(key)) n.delete(key); else n.add(key);
        return n;
    });
    const q = query.trim().toLowerCase();
    const hits = useMemo(() => (q
        ? playlist.map((track, index) => ({ track, index }))
            .filter(({ track }) => [track.title, track.artist, track.anime?.title ?? ''].some((t) => t.toLowerCase().includes(q)))
        : []), [playlist, q]);

    const row = ({ track, index }: { track: MusicTrack; index: number }): React.ReactElement => {
        const active = index === trackIndex;
        return (
            <button
                key={track.id}
                ref={active ? activeRef : undefined}
                type="button"
                className={styles.row}
                data-active={active || undefined}
                onClick={() => onPick(index)}
            >
                <span className={styles.rowIndex}>{active ? <EqBars playing={isPlaying} /> : index + 1}</span>
                <span className={styles.rowTitle}>{track.title}</span>
                {(track.lyrics?.length ?? 0) > 0 && <span className={styles.rowTag} title="有歌词">词</span>}
                {track.artist && track.artist !== 'Unknown' && <span className={styles.rowArtist}>{track.artist}</span>}
            </button>
        );
    };
    const flat = groups.length <= 1;
    return (
        <div className={styles.list}>
            <div className={styles.listHead}>
                <span className={styles.label}>播放列表</span>
                <span className={styles.listCount}>{flat ? `${playlist.length} 首` : `${groups.length} 部 · ${playlist.length} 首`}</span>
            </div>
            {playlist.length > 6 && (
                <label className={styles.search}>
                    <FaSearch size={10} aria-hidden="true" />
                    <input
                        type="search"
                        value={query}
                        placeholder="搜索歌名 / 歌手 / 番剧"
                        onChange={(e) => setQuery(e.target.value)}
                        onKeyDown={(e) => e.stopPropagation()}
                    />
                </label>
            )}
            <div className={styles.listBody}>
                {q ? (hits.length ? hits.map(row) : <div className={styles.empty}>没有匹配的歌</div>)
                    : flat ? playlist.map((track, index) => row({ track, index })) : groups.map((g) => {
                        const isOpen = open.has(g.key);
                        const playing = g.key === currentKey;
                        return (
                            <div key={g.key} className={styles.group} data-open={isOpen || undefined}>
                                <button
                                    type="button"
                                    className={styles.groupHead}
                                    data-playing={playing || undefined}
                                    aria-expanded={isOpen}
                                    onClick={() => flip(g.key)}
                                >
                                    {g.cover
                                        ? <img className={styles.groupCover} src={g.cover} alt="" loading="lazy" decoding="async" />
                                        : <span className={styles.groupCover} aria-hidden="true">♪</span>}
                                    <span className={styles.groupTitle}>{g.title}</span>
                                    <span className={styles.groupCount}>{g.items.length}</span>
                                    <span className={styles.groupChevron} aria-hidden="true">›</span>
                                </button>
                                <div className={styles.groupBody} inert={!isOpen}>
                                    <div className={styles.groupInner}>{g.items.map(row)}</div>
                                </div>
                            </div>
                        );
                    })}
            </div>
        </div>
    );
}

/** 播放器下拉面板 */
function MusicPanel(): React.ReactElement {
    const pl = useMusicStore((s) => s.playlist);
    const trackIndex = useMusicStore((s) => s.trackIndex);
    const currentTime = useMusicStore((s) => s.currentTime);
    const duration = useMusicStore((s) => s.duration);
    const bufferedEnd = useMusicStore((s) => s.bufferedEnd);
    const isPlaying = useMusicStore((s) => s.isPlaying);
    const volume = useMusicStore((s) => s.volume);
    const showLyrics = useMusicStore((s) => s.showLyrics);
    const playMode = useMusicStore((s) => s.playMode);
    const audioStatus = useMusicStore((s) => s.audioStatus);
    const downloadProgress = useMusicStore((s) => s.downloadProgress);
    const isLeader = useMusicStore((s) => s.isLeader);
    const lossless = useMusicStore((s) => s.lossless);
    const lyricChoice = useMusicStore((s) => s.lyricChoice);

    const toggle = useMusicStore((s) => s.toggle);
    const next = useMusicStore((s) => s.next);
    const prev = useMusicStore((s) => s.prev);
    const seek = useMusicStore((s) => s.seek);
    const setVolume = useMusicStore((s) => s.setVolume);
    const setTrack = useMusicStore((s) => s.setTrack);
    const toggleLyrics = useMusicStore((s) => s.toggleLyrics);
    const closePanel = useMusicStore((s) => s.closePanel);
    const cyclePlayMode = useMusicStore((s) => s.cyclePlayMode);
    const setLyric = useMusicStore((s) => s.setLyric);
    const toggleLossless = useMusicStore((s) => s.toggleLossless);

    const panelRef = useRef<HTMLDivElement>(null);
    const currentTrack = pl[trackIndex] ?? null;
    const lyrics = currentTrack?.lyrics ?? [];
    const lyric = pickLyric(currentTrack, lyricChoice);

    // 歌词悬浮窗锁定状态 (从 localStorage 读取, 与 AssLyrics 共享)
    const [lyricsLocked, setLyricsLocked] = useState(() => {
        try { return localStorage.getItem(LOCK_KEY) === 'true'; } catch { return false; }
    });
    const prevVolumeRef = useRef(volume > 0 ? volume : 0.7);

    useEffect(() => {
        const handleStorage = (e: StorageEvent): void => {
            if (e.key === LOCK_KEY && e.newValue !== null) setLyricsLocked(e.newValue === 'true');
        };
        window.addEventListener('storage', handleStorage);
        return () => window.removeEventListener('storage', handleStorage);
    }, []);

    // 点击外部关闭; Esc 关闭; 面板内空格播放暂停, 左右键快退快进 5 秒
    useEffect(() => {
        const onDown = (e: MouseEvent): void => {
            if (panelRef.current && !panelRef.current.contains(e.target as Node)) {
                if (!(e.target as HTMLElement).closest(`.${styles.navbarBtn}`)) closePanel();
            }
        };
        const onKey = (e: KeyboardEvent): void => {
            if (e.key === 'Escape') { closePanel(); return; }
            if (!panelRef.current?.contains(document.activeElement)) return;
            const tag = (document.activeElement as HTMLElement | null)?.tagName;
            if (tag === 'INPUT' || (document.activeElement as HTMLElement | null)?.getAttribute('role') === 'slider') return;
            if (e.key === ' ') { e.preventDefault(); toggle(); }
        };
        const timer = setTimeout(() => document.addEventListener('mousedown', onDown), 100);
        document.addEventListener('keydown', onKey);
        return () => {
            clearTimeout(timer);
            document.removeEventListener('mousedown', onDown);
            document.removeEventListener('keydown', onKey);
        };
    }, [closePanel, toggle]);

    const commitVolume = useCallback((vol: number) => {
        const v = Math.round(vol * 100) / 100;
        setVolume(v);
        if (v > 0) prevVolumeRef.current = v;
    }, [setVolume]);

    const toggleMute = useCallback(() => {
        if (volume === 0) setVolume(prevVolumeRef.current > 0 ? prevVolumeRef.current : 0.7);
        else { prevVolumeRef.current = volume; setVolume(0); }
    }, [volume, setVolume]);

    const toggleLyricsLock = useCallback(() => {
        const nextLocked = !lyricsLocked;
        setLyricsLocked(nextLocked);
        try { localStorage.setItem(LOCK_KEY, JSON.stringify(nextLocked)); } catch { /* ignore */ }
        // 同一标签页不会收到 storage 事件, 手动派发让 AssLyrics 同步
        window.dispatchEvent(new StorageEvent('storage', { key: LOCK_KEY, newValue: JSON.stringify(nextLocked) }));
    }, [lyricsLocked]);

    const VolumeIcon = volume === 0 ? FaVolumeMute : volume < 0.5 ? FaVolumeDown : FaVolumeUp;
    const playingUrl = currentTrack?.audioWebUrl && !lossless ? currentTrack.audioWebUrl : currentTrack?.audioUrl;
    const fmt = formatOf(playingUrl);
    const sub = [currentTrack?.artist && currentTrack.artist !== 'Unknown' ? currentTrack.artist : '', currentTrack?.anime?.title ?? '']
        .filter(Boolean).join(' · ');

    const downloading = isLeader && audioStatus === 'downloading';
    // 状态行: 只有 Leader 知道真实加载进度; Follower 只显示格式
    let status: React.ReactNode = null;
    if (isLeader && audioStatus === 'downloading') {
        status = <span className={styles.statusBusy}>大文件分片下载 {Math.round(downloadProgress * 100)}%</span>;
    } else if (isLeader && audioStatus === 'loading' && isPlaying) {
        status = <span className={styles.statusBusy}>缓冲中</span>;
    } else if (isLeader && audioStatus === 'error') {
        status = <span className={styles.statusErr}>加载失败, 点下一首或重试</span>;
    }

    return (
        <div ref={panelRef} className={styles.panel} role="dialog" aria-label="音乐播放器">
            {/* 当前曲目 */}
            <div className={styles.now}>
                <div className={styles.coverBox}>
                    {currentTrack?.coverUrl
                        ? <img src={currentTrack.coverUrl} alt="" className={styles.cover} />
                        : <span className={styles.coverEmpty}><FaMusic size={18} /></span>}
                </div>
                <div className={styles.nowMeta}>
                    <Marquee text={currentTrack?.title || '无曲目'} className={styles.nowTitle} />
                    {sub && <Marquee text={sub} className={styles.nowSub} />}
                    <div className={styles.nowTags}>
                        {fmt && <span className={styles.tag}>{fmt}</span>}
                        {currentTrack?.audioWebUrl && (
                            <button
                                type="button"
                                className={styles.tagBtn}
                                data-on={lossless || undefined}
                                onClick={toggleLossless}
                                title={lossless ? '正在播原始无损文件, 点击改播网页版' : '正在播网页版, 点击改播原始无损文件'}
                            >
                                无损
                            </button>
                        )}
                        {status}
                    </div>
                </div>
            </div>

            {/* 进度 */}
            <div className={styles.progress}>
                <Scrubber
                    value={downloading ? downloadProgress : currentTime}
                    max={downloading ? 1 : duration}
                    buffered={bufferedEnd}
                    onCommit={seek}
                    format={downloading ? (v) => `${Math.round(v * 100)}%` : formatTime}
                    label="播放进度"
                    disabled={downloading}
                />
                <div className={styles.times}>
                    <span>{formatTime(currentTime)}</span>
                    <span>{formatTime(duration)}</span>
                </div>
            </div>

            {/* 控制 */}
            <div className={styles.controls}>
                <button type="button" onClick={cyclePlayMode} className={styles.ctl} title={PLAY_MODE_INFO[playMode].title} aria-label={PLAY_MODE_INFO[playMode].title}>
                    {PLAY_MODE_INFO[playMode].icon}
                </button>
                <button type="button" onClick={prev} className={styles.ctl} title="上一曲" aria-label="上一曲">
                    <FaStepBackward size={14} />
                </button>
                <button type="button" onClick={toggle} className={styles.ctlMain} title={isPlaying ? '暂停' : '播放'} aria-label={isPlaying ? '暂停' : '播放'}>
                    {isPlaying ? <FaPause size={16} /> : <FaPlay size={16} style={{ marginLeft: 3 }} />}
                </button>
                <button type="button" onClick={next} className={styles.ctl} title="下一曲" aria-label="下一曲">
                    <FaStepForward size={14} />
                </button>
                <button
                    type="button"
                    onClick={toggleLyrics}
                    className={styles.ctl}
                    data-on={showLyrics || undefined}
                    disabled={!lyric}
                    title={!lyric ? '这首歌没有歌词' : showLyrics ? '隐藏歌词' : '显示歌词'}
                    aria-label="歌词"
                >
                    <span className={styles.ctlWord}>词</span>
                </button>
            </div>

            {/* 歌词: 多份时可切换; 显示中可锁定 / 复位悬浮窗 */}
            {lyric && (showLyrics || lyrics.length > 1) && (
                <div className={styles.lyricRow}>
                    <span className={styles.label}>歌词</span>
                    <div className={styles.seg} role="radiogroup" aria-label="选择歌词">
                        {lyrics.map((l) => (
                            <button
                                key={l.key}
                                type="button"
                                role="radio"
                                aria-checked={l.key === lyric.key}
                                className={styles.segItem}
                                data-on={l.key === lyric.key || undefined}
                                onClick={() => currentTrack && setLyric(currentTrack.id, l.key)}
                            >
                                {l.label}
                            </button>
                        ))}
                    </div>
                    {showLyrics && (
                        <span className={styles.lyricTools}>
                            <button
                                type="button"
                                onClick={toggleLyricsLock}
                                className={styles.mini}
                                data-on={lyricsLocked || undefined}
                                title={lyricsLocked ? '解锁歌词悬浮窗' : '锁定歌词悬浮窗 (全透明穿透)'}
                                aria-label="锁定歌词悬浮窗"
                            >
                                {lyricsLocked ? <FaLock size={10} /> : <FaLockOpen size={10} />}
                            </button>
                            <button type="button" onClick={resetLyricsPosition} className={styles.mini} title="重置歌词悬浮窗位置" aria-label="重置歌词悬浮窗位置">
                                <FaRedo size={9} />
                            </button>
                        </span>
                    )}
                </div>
            )}

            {/* 音量 */}
            <div className={styles.volume}>
                <button type="button" onClick={toggleMute} className={styles.mini} title={volume === 0 ? '取消静音' : '静音'} aria-label="静音">
                    <VolumeIcon size={13} />
                </button>
                <Scrubber value={volume} max={1} onCommit={commitVolume} live format={(v) => `${Math.round(v * 100)}%`} label="音量" />
                <span className={styles.volNum}>{Math.round(volume * 100)}</span>
            </div>

            {pl.length >= 1 && <PlaylistTree playlist={pl} trackIndex={trackIndex} isPlaying={isPlaying} onPick={setTrack} />}
        </div>
    );
}
