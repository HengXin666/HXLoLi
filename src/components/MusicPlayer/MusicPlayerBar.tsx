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
function Scrubber({ value, max, buffered = 0, onCommit, live = false, format, label, disabled = false, playing = false, loading = false }: {
    value: number;
    max: number;
    buffered?: number;
    onCommit: (v: number) => void;
    live?: boolean;
    format: (v: number) => string;
    label: string;
    disabled?: boolean;
    /** 播放中: 两次 timeupdate 之间按真实时间外推, 让填充连续前进而不是 4Hz 跳格 */
    playing?: boolean;
    /** 加载中: 轨道上跑一条流光, 不可拖 */
    loading?: boolean;
}): React.ReactElement {
    const trackRef = useRef<HTMLSpanElement>(null);
    const fillRef = useRef<HTMLSpanElement>(null);
    const thumbRef = useRef<HTMLSpanElement>(null);
    const [drag, setDrag] = useState<number | null>(null);
    const [hover, setHover] = useState<number | null>(null);
    const safeMax = max > 0 && isFinite(max) ? max : 0;
    const shown = drag ?? value;
    const clamp01 = (x: number): number => Math.min(1, Math.max(0, x));
    const ratio = safeMax ? clamp01(shown / safeMax) : 0;
    const bufRatio = safeMax ? clamp01(buffered / safeMax) : 0;

    // 外推: 记下最近一次 value 的到达时刻, rAF 里直接写 style, 不触发 React 重渲染
    const anchor = useRef({ v: value, t: 0 });
    useEffect(() => { anchor.current = { v: value, t: performance.now() }; }, [value]);
    useEffect(() => {
        if (!playing || drag !== null || !safeMax) return;
        let raf = 0;
        const tick = (): void => {
            const { v, t } = anchor.current;
            // 最多外推 1s, timeupdate 停了 (卡顿/缓冲) 就停在原地
            const r = clamp01((v + Math.min(1, (performance.now() - t) / 1000)) / safeMax) * 100;
            if (fillRef.current) fillRef.current.style.width = `${r}%`;
            if (thumbRef.current) thumbRef.current.style.left = `${r}%`;
            raf = requestAnimationFrame(tick);
        };
        raf = requestAnimationFrame(tick);
        return () => cancelAnimationFrame(raf);
    }, [playing, drag, safeMax]);

    const valueAt = (clientX: number): number => {
        const r = trackRef.current?.getBoundingClientRect();
        if (!r || r.width === 0) return 0;
        return clamp01((clientX - r.left) / r.width) * safeMax;
    };
    const off = disabled || loading || !safeMax;

    const onPointerDown = (e: React.PointerEvent<HTMLDivElement>): void => {
        if (off || e.button !== 0) return;
        e.preventDefault();
        e.currentTarget.setPointerCapture(e.pointerId);
        const v = valueAt(e.clientX);
        setDrag(v);
        if (live) onCommit(v);
    };
    const onPointerMove = (e: React.PointerEvent<HTMLDivElement>): void => {
        if (off) return;
        const v = valueAt(e.clientX);
        if (e.pointerType === 'mouse') setHover(v);
        if (drag !== null) {
            setDrag(v);
            if (live) onCommit(v);
        }
    };
    const onPointerUp = (e: React.PointerEvent<HTMLDivElement>): void => {
        if (drag === null) return;
        const v = valueAt(e.clientX);
        setDrag(null);
        if (e.pointerType !== 'mouse') setHover(null);
        onCommit(v);
    };
    const onKeyDown = (e: React.KeyboardEvent<HTMLDivElement>): void => {
        if (off) return;
        const step = safeMax <= 1 ? 0.05 : 5;
        const keys: Record<string, number> = { ArrowRight: step, ArrowUp: step, ArrowLeft: -step, ArrowDown: -step, PageUp: step * 6, PageDown: -step * 6 };
        if (e.key in keys) { e.preventDefault(); onCommit(Math.min(safeMax, Math.max(0, value + keys[e.key]))); }
        if (e.key === 'Home') { e.preventDefault(); onCommit(0); }
        if (e.key === 'End' && safeMax > 1) { e.preventDefault(); onCommit(safeMax * 0.999); }
    };

    const tip = drag ?? hover;
    const tipRatio = tip !== null && safeMax ? clamp01(tip / safeMax) : null;
    return (
        <div
            className={styles.scrub}
            data-dragging={drag !== null || undefined}
            data-disabled={off || undefined}
            data-loading={loading || undefined}
            role="slider"
            tabIndex={off ? -1 : 0}
            aria-label={label}
            aria-valuemin={0}
            aria-valuemax={safeMax}
            aria-valuenow={shown}
            aria-valuetext={format(shown)}
            onPointerDown={onPointerDown}
            onPointerMove={onPointerMove}
            onPointerUp={onPointerUp}
            onPointerCancel={() => { setDrag(null); setHover(null); }}
            onPointerLeave={() => setHover(null)}
            onKeyDown={onKeyDown}
        >
            <span ref={trackRef} className={styles.scrubTrack}>
                <span className={styles.scrubBar}>
                    <span className={styles.scrubBuf} style={{ width: `${bufRatio * 100}%` }} />
                    {tipRatio !== null && drag === null && (
                        <span className={styles.scrubGhost} style={{ width: `${tipRatio * 100}%` }} />
                    )}
                    <span ref={fillRef} className={styles.scrubFill} style={{ width: `${ratio * 100}%` }} />
                </span>
                <span ref={thumbRef} className={styles.scrubThumb} style={{ left: `${ratio * 100}%` }} />
                {tipRatio !== null && (
                    <span className={styles.scrubTip} style={{ left: `${tipRatio * 100}%` }}>{format(tip as number)}</span>
                )}
            </span>
        </div>
    );
}

/**
 * 音量: 控制栏右端一个图标, 悬停 / 聚焦时向上弹出竖向小滑条, 和横向的进度条在形态上彻底区分.
 * 点图标静音, 在图标上滚轮 ±5%.
 */
function VolumeControl({ volume, onChange, onToggleMute }: {
    volume: number;
    onChange: (v: number) => void;
    onToggleMute: () => void;
}): React.ReactElement {
    const [open, setOpen] = useState(false);
    const [drag, setDrag] = useState(false);
    const railRef = useRef<HTMLSpanElement>(null);
    const closeTimer = useRef<number | undefined>(undefined);
    const Icon = volume === 0 ? FaVolumeMute : volume < 0.5 ? FaVolumeDown : FaVolumeUp;
    const show = (): void => { window.clearTimeout(closeTimer.current); setOpen(true); };
    const hide = (): void => { closeTimer.current = window.setTimeout(() => setOpen(false), 220); };
    useEffect(() => () => window.clearTimeout(closeTimer.current), []);

    const at = (clientY: number): number => {
        const r = railRef.current?.getBoundingClientRect();
        if (!r || r.height === 0) return volume;
        return Math.min(1, Math.max(0, (r.bottom - clientY) / r.height));
    };
    const onWheel = (e: React.WheelEvent): void => {
        e.preventDefault();
        onChange(Math.min(1, Math.max(0, volume + (e.deltaY < 0 ? 0.05 : -0.05))));
    };

    return (
        <div
            className={styles.vol}
            data-open={open || drag || undefined}
            onPointerEnter={(e) => { if (e.pointerType === 'mouse') show(); }}
            onPointerLeave={(e) => { if (e.pointerType === 'mouse' && !drag) hide(); }}
            onFocus={show}
            onBlur={(e) => { if (!e.currentTarget.contains(e.relatedTarget as Node)) hide(); }}
        >
            <div className={styles.volPop}>
                <span className={styles.volNum}>{Math.round(volume * 100)}</span>
                <div
                    className={styles.volHit}
                    role="slider"
                    tabIndex={0}
                    aria-label="音量"
                    aria-orientation="vertical"
                    aria-valuemin={0}
                    aria-valuemax={100}
                    aria-valuenow={Math.round(volume * 100)}
                    onPointerDown={(e) => {
                        if (e.button !== 0) return;
                        e.preventDefault();
                        e.currentTarget.setPointerCapture(e.pointerId);
                        setDrag(true);
                        onChange(at(e.clientY));
                    }}
                    onPointerMove={(e) => { if (drag) onChange(at(e.clientY)); }}
                    onPointerUp={() => setDrag(false)}
                    onPointerCancel={() => setDrag(false)}
                    onWheel={onWheel}
                    onKeyDown={(e) => {
                        const d = e.key === 'ArrowUp' || e.key === 'ArrowRight' ? 0.05 : e.key === 'ArrowDown' || e.key === 'ArrowLeft' ? -0.05 : 0;
                        if (d) { e.preventDefault(); onChange(Math.min(1, Math.max(0, volume + d))); }
                    }}
                >
                    <span ref={railRef} className={styles.volRail}>
                        <span className={styles.volFill} style={{ height: `${volume * 100}%` }} />
                        <span className={styles.volKnob} style={{ bottom: `${volume * 100}%` }} />
                    </span>
                </div>
            </div>
            <button
                type="button"
                onClick={onToggleMute}
                onWheel={onWheel}
                className={styles.ctl}
                data-on={volume === 0 || undefined}
                title={volume === 0 ? '取消静音' : `音量 ${Math.round(volume * 100)} (点击静音, 滚轮调节)`}
                aria-label={volume === 0 ? '取消静音' : '静音'}
            >
                <Icon size={15} />
            </button>
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

interface TrackItem { track: MusicTrack; index: number }

/** 播放列表树的一个目录节点; key = 相对 static/music/ 的目录路径 */
interface DirNode {
    key: string;
    name: string;
    dirs: DirNode[];
    items: TrackItem[];
    count: number;
    /** 目录自带封面 (playlist 的 dirCovers), 没有时为 undefined */
    cover?: string;
    /** 子树内前 4 张不同的歌曲封面, 用来拼图 */
    covers: string[];
}

const MUSIC_ROOT = '/static/music/';

/** 歌曲所在目录的各级名字: "/static/music/番剧A/OST/x.flac" -> ["番剧A", "OST"] */
function dirSegments(track: MusicTrack): string[] {
    const raw = track.audioRelativePath ?? track.audioUrl ?? '';
    const i = raw.indexOf(MUSIC_ROOT);
    if (i < 0) return [];
    const segs = raw.slice(i + MUSIC_ROOT.length).split('/').slice(0, -1).filter(Boolean);
    return segs.map((x) => { try { return decodeURIComponent(x); } catch { return x; } });
}

/**
 * 按音频所在目录建树, 层数不限. 目录顺序 = 目录内第一首在原列表的位置, 不改歌曲原顺序 (上一首 / 下一首仍按原列表).
 * 只有一个子目录且自己没有歌的目录与子目录合并成 "A / B" 一行, 少点一层展开.
 */
function buildDirTree(pl: readonly MusicTrack[]): DirNode {
    const root: DirNode = { key: '', name: '', dirs: [], items: [], count: 0, covers: [] };
    const dirCovers = new Map<string, string>();
    pl.forEach((track, index) => {
        for (const [k, v] of Object.entries(track.dirCovers ?? {})) if (v && !dirCovers.has(k)) dirCovers.set(k, v);
        let node = root;
        for (const seg of dirSegments(track)) {
            const key = node.key ? `${node.key}/${seg}` : seg;
            let next = node.dirs.find((d) => d.key === key);
            if (!next) {
                next = { key, name: seg, dirs: [], items: [], count: 0, covers: [] };
                node.dirs.push(next);
            }
            node = next;
        }
        node.items.push({ track, index });
    });
    const finish = (n: DirNode): DirNode => {
        n.dirs = n.dirs.map(finish);
        while (n !== root && n.items.length === 0 && n.dirs.length === 1) {
            const only = n.dirs[0];
            n = { ...only, name: `${n.name} / ${only.name}`, cover: only.cover ?? dirCovers.get(n.key) };
        }
        n.cover ??= dirCovers.get(n.key);
        n.count = n.items.length + n.dirs.reduce((x, d) => x + d.count, 0);
        const covers = new Set<string>();
        for (const it of n.items) if (it.track.coverUrl) covers.add(it.track.coverUrl);
        for (const d of n.dirs) for (const c of d.covers) covers.add(c);
        n.covers = [...covers].slice(0, 4);
        return n;
    };
    return finish(root);
}

/** 当前歌曲所在的各级目录 key (用于自动展开) */
function ancestorKeys(track: MusicTrack | undefined): string[] {
    if (!track) return [];
    const segs = dirSegments(track);
    return segs.map((_, i) => segs.slice(0, i + 1).join('/'));
}

/**
 * 目录封面: 目录自带的 cover.* 优先; 否则用子树里的歌曲封面拼图, 张数决定版式不重复贴图:
 * 1 张铺满, 2 张左右各半, 3 张左大右二, 4 张 2x2; 一张都没有画音符.
 */
function DirCover({ node }: { node: DirNode }): React.ReactElement {
    if (node.cover) return <img className={styles.groupCover} src={node.cover} alt="" loading="lazy" decoding="async" />;
    if (node.covers.length === 0) return <span className={styles.groupCover} aria-hidden="true">♪</span>;
    if (node.covers.length === 1) return <img className={styles.groupCover} src={node.covers[0]} alt="" loading="lazy" decoding="async" />;
    return (
        <span className={`${styles.groupCover} ${styles.mosaic}`} data-n={node.covers.length} aria-hidden="true">
            {node.covers.map((c, i) => <img key={i} src={c} alt="" loading="lazy" decoding="async" />)}
        </span>
    );
}

/**
 * 播放列表树: 按音频所在目录分组 (层数不限), 目录行 = 封面 / 名字 / 曲数 / 箭头, 歌曲行带封面缩略图.
 * 默认只展开正在播放的那首所在的各级目录, 切歌时自动展开新路径, 手动展开的保持.
 * 有搜索词时平铺显示命中的歌 (标题 / 歌手 / 目录名); 没有子目录时退化为平铺列表.
 * .agents/notes/implemented/feature/2026-11-02-music-anime-tree.md
 */
function PlaylistTree({ playlist, trackIndex, isPlaying, onPick }: {
    playlist: readonly MusicTrack[];
    trackIndex: number;
    isPlaying: boolean;
    onPick: (i: number) => void;
}): React.ReactElement {
    const tree = useMemo(() => buildDirTree(playlist), [playlist]);
    const currentPath = useMemo(() => ancestorKeys(playlist[trackIndex]), [playlist, trackIndex]);
    const currentSet = useMemo(() => new Set(currentPath), [currentPath]);
    const [open, setOpen] = useState<ReadonlySet<string>>(() => new Set(currentPath));
    const [query, setQuery] = useState('');
    const activeRef = useRef<HTMLButtonElement>(null);
    useEffect(() => {
        setOpen((s) => (currentPath.every((k) => s.has(k)) ? s : new Set([...s, ...currentPath])));
    }, [currentPath]);
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
            .filter(({ track }) => [track.title, track.artist, dirSegments(track).join('/')].some((t) => t.toLowerCase().includes(q)))
        : []), [playlist, q]);
    // 合并过的目录 ("A / B") 的 key 是最深一级那层的路径, 同样在当前歌的祖先路径里
    const holdsCurrent = (key: string): boolean => currentSet.has(key);

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
                {track.coverUrl
                    ? <img className={styles.rowCover} src={track.coverUrl} alt="" loading="lazy" decoding="async" />
                    : <span className={styles.rowCover} aria-hidden="true" />}
                <span className={styles.rowTitle}>{track.title}</span>
                {(track.lyrics?.length ?? 0) > 0 && <span className={styles.rowTag} title="有歌词">词</span>}
                {track.artist && track.artist !== 'Unknown' && <span className={styles.rowArtist}>{track.artist}</span>}
            </button>
        );
    };
    const flat = tree.dirs.length === 0;
    const renderDir = (node: DirNode, depth: number): React.ReactElement => {
        const isOpen = open.has(node.key);
        return (
            <div key={node.key} className={styles.group} data-open={isOpen || undefined}>
                <button
                    type="button"
                    className={styles.groupHead}
                    data-playing={holdsCurrent(node.key) || undefined}
                    aria-expanded={isOpen}
                    title={node.key}
                    onClick={() => flip(node.key)}
                >
                    <DirCover node={node} />
                    <span className={styles.groupTitle}>{node.name}</span>
                    <span className={styles.groupCount}>{node.count}</span>
                    <span className={styles.groupChevron} aria-hidden="true">›</span>
                </button>
                <div className={styles.groupBody} inert={!isOpen}>
                    <div className={styles.groupInner}>
                        {node.dirs.map((d) => renderDir(d, depth + 1))}
                        {node.items.map(row)}
                    </div>
                </div>
            </div>
        );
    };
    const topDirs = tree.dirs.length;
    return (
        <div className={styles.list}>
            <div className={styles.listHead}>
                <span className={styles.label}>播放列表</span>
                <span className={styles.listCount}>{flat ? `${playlist.length} 首` : `${topDirs} 个目录 · ${playlist.length} 首`}</span>
            </div>
            {playlist.length > 6 && (
                <label className={styles.search}>
                    <FaSearch size={10} aria-hidden="true" />
                    <input
                        type="search"
                        value={query}
                        placeholder="搜索歌名 / 歌手 / 目录"
                        onChange={(e) => setQuery(e.target.value)}
                        onKeyDown={(e) => e.stopPropagation()}
                    />
                </label>
            )}
            <div className={styles.listBody}>
                {q ? (hits.length ? hits.map(row) : <div className={styles.empty}>没有匹配的歌</div>)
                    : flat ? playlist.map((track, index) => row({ track, index }))
                        : <>{tree.dirs.map((d) => renderDir(d, 0))}{tree.items.map(row)}</>}
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

    const playingUrl = currentTrack?.audioWebUrl && !lossless ? currentTrack.audioWebUrl : currentTrack?.audioUrl;
    const fmt = formatOf(playingUrl);
    const sub = [currentTrack?.artist && currentTrack.artist !== 'Unknown' ? currentTrack.artist : '', currentTrack?.anime?.title ?? '']
        .filter(Boolean).join(' · ');

    const downloading = isLeader && audioStatus === 'downloading';
    const [showRemain, setShowRemain] = useState(false);
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
                <span className={styles.time}>{formatTime(currentTime)}</span>
                <Scrubber
                    value={downloading ? 0 : currentTime}
                    max={downloading ? 1 : duration}
                    buffered={downloading ? downloadProgress : bufferedEnd}
                    onCommit={seek}
                    format={formatTime}
                    label="播放进度"
                    playing={isPlaying && !downloading && audioStatus !== 'loading'}
                    loading={downloading || (isLeader && audioStatus === 'loading' && isPlaying)}
                />
                <button
                    type="button"
                    className={styles.timeBtn}
                    onClick={() => setShowRemain((v) => !v)}
                    title={showRemain ? '显示总时长' : '显示剩余时间'}
                >
                    {showRemain && duration > 0 ? `-${formatTime(Math.max(0, duration - currentTime))}` : formatTime(duration)}
                </button>
            </div>

            {/* 控制: 左 = 播放模式, 中 = 上一首 / 播放 / 下一首, 右 = 歌词开关 + 音量 */}
            <div className={styles.controls}>
                <div className={styles.ctlSide}>
                    <button type="button" onClick={cyclePlayMode} className={styles.ctl} data-on={playMode !== 'list-loop' || undefined} title={PLAY_MODE_INFO[playMode].title} aria-label={PLAY_MODE_INFO[playMode].title}>
                        {PLAY_MODE_INFO[playMode].icon}
                    </button>
                </div>
                <div className={styles.ctlCenter}>
                    <button type="button" onClick={prev} className={styles.ctl} title="上一曲" aria-label="上一曲">
                        <FaStepBackward size={14} />
                    </button>
                    <button type="button" onClick={toggle} className={styles.ctlMain} title={isPlaying ? '暂停' : '播放'} aria-label={isPlaying ? '暂停' : '播放'}>
                        {isPlaying ? <FaPause size={16} /> : <FaPlay size={16} style={{ marginLeft: 3 }} />}
                    </button>
                    <button type="button" onClick={next} className={styles.ctl} title="下一曲" aria-label="下一曲">
                        <FaStepForward size={14} />
                    </button>
                </div>
                <div className={`${styles.ctlSide} ${styles.ctlRight}`}>
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
                    <VolumeControl volume={volume} onChange={commitVolume} onToggleMute={toggleMute} />
                </div>
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

            {pl.length >= 1 && <PlaylistTree playlist={pl} trackIndex={trackIndex} isPlaying={isPlaying} onPick={setTrack} />}
        </div>
    );
}
