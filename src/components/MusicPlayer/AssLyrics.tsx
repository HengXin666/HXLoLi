/**
 * ASS 歌词渲染悬浮窗组件
 *
 * 使用 libass-wasm (JavascriptSubtitlesOctopus) 渲染 ASS 歌词
 * 支持拖拽移动、位置记忆、全屏观看、锁定(全透明)、位置重置
 */
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useMusicStore } from '@site/src/utils/music/musicStore';
import React, { useCallback, useEffect, useRef, useState } from 'react';

/** 全局重置位置的事件名 */
const RESET_POSITION_EVENT = 'hxloli-lyrics-reset-position';

const POSITION_KEY = 'hxloli-lyrics-position';
const SIZE_KEY = 'hxloli-lyrics-size';

/** 默认位置 */
function defaultPosition(): { x: number; y: number } {
    if (typeof window === 'undefined') return { x: 100, y: 80 };
    return { x: Math.max(0, window.innerWidth - 520), y: 80 };
}

/** 默认尺寸 */
const DEFAULT_SIZE = { w: 500, h: 350 };

/** 加载保存的位置 */
function loadPosition(): { x: number; y: number } {
    try {
        const raw = localStorage.getItem(POSITION_KEY);
        if (raw) return JSON.parse(raw);
    } catch { /* ignore */ }
    return defaultPosition();
}

/** 保存位置 */
function savePosition(pos: { x: number; y: number }): void {
    try { localStorage.setItem(POSITION_KEY, JSON.stringify(pos)); } catch { /* ignore */ }
}

/** 加载保存的尺寸 */
function loadSize(): { w: number; h: number } {
    try {
        const raw = localStorage.getItem(SIZE_KEY);
        if (raw) return JSON.parse(raw);
    } catch { /* ignore */ }
    return { ...DEFAULT_SIZE };
}

/** 保存尺寸 */
function saveSize(size: { w: number; h: number }): void {
    try { localStorage.setItem(SIZE_KEY, JSON.stringify(size)); } catch { /* ignore */ }
}

export default function AssLyrics(): React.ReactElement | null {
    const showLyrics = useMusicStore((s) => s.showLyrics);
    const lyricsFullscreen = useMusicStore((s) => s.lyricsFullscreen);
    const currentTime = useMusicStore((s) => s.currentTime);
    const trackIndex = useMusicStore((s) => s.trackIndex);
    const pl = useMusicStore((s) => s.playlist);
    const toggleLyrics = useMusicStore((s) => s.toggleLyrics);
    const toggleLyricsFullscreen = useMusicStore((s) => s.toggleLyricsFullscreen);
    const { siteConfig } = useDocusaurusContext();
    const baseUrl = siteConfig.baseUrl; // e.g. '/HXLoLi/'

    const currentTrack = pl[trackIndex] ?? null;

    const containerRef = useRef<HTMLDivElement>(null);
    const canvasContainerRef = useRef<HTMLDivElement>(null);
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const octopusRef = useRef<any>(null);
    const [position, setPosition] = useState(loadPosition);
    const [size, setSize] = useState(loadSize);
    const [locked, setLocked] = useState(false); // 锁定 (全透明穿透)
    const [, forceUpdate] = useState(0);
    const isDragging = useRef(false);
    const isResizing = useRef(false);
    const dragOffset = useRef({ x: 0, y: 0 });
    const lastTimeRef = useRef(-1);
    const initRetryRef = useRef<ReturnType<typeof setTimeout> | null>(null);

    // 位置重置
    const resetPosition = useCallback(() => {
        const pos = defaultPosition();
        const sz = { ...DEFAULT_SIZE };
        setPosition(pos);
        setSize(sz);
        savePosition(pos);
        saveSize(sz);
    }, []);

    // 切换锁定
    const toggleLock = useCallback(() => {
        setLocked((prev) => !prev);
    }, []);

    // 监听外部触发的重置位置事件（从播放器面板发出）
    useEffect(() => {
        const handler = () => {
            const pos = defaultPosition();
            const sz = { ...DEFAULT_SIZE };
            setPosition(pos);
            setSize(sz);
            savePosition(pos);
            saveSize(sz);
            forceUpdate((n) => n + 1);
        };
        window.addEventListener(RESET_POSITION_EVENT, handler);
        return () => window.removeEventListener(RESET_POSITION_EVENT, handler);
    }, []);

    // ---- 初始化 / 切换曲目时重建 octopus ----
    useEffect(() => {
        if (!showLyrics || !currentTrack?.assUrl) {
            if (octopusRef.current) {
                try { octopusRef.current.dispose(); } catch { /* ignore */ }
                octopusRef.current = null;
            }
            return;
        }

        let disposed = false;

        const initOctopus = async () => {
            // 等待 canvas 实际渲染到 DOM 并获得尺寸
            const canvas = canvasRef.current;
            const container = canvasContainerRef.current;
            if (!canvas || !container) {
                // canvas 还未挂载, 重试
                if (!disposed) {
                    initRetryRef.current = setTimeout(initOctopus, 100);
                }
                return;
            }

            // 获取容器的实际像素尺寸
            const rect = container.getBoundingClientRect();
            const pixelW = Math.round(rect.width) || size.w;
            const pixelH = Math.round(rect.height) || (size.h - 32); // 减去标题栏高度

            // 设置 canvas 像素尺寸
            canvas.width = pixelW;
            canvas.height = pixelH;

            try {
                const SubtitlesOctopus = (await import('libass-wasm')).default;
                if (disposed) return;

                // 销毁旧实例
                if (octopusRef.current) {
                    try { octopusRef.current.dispose(); } catch { /* ignore */ }
                    octopusRef.current = null;
                }

                const workerUrl = `${baseUrl}music/ass-worker/subtitles-octopus-worker.js`;
                const legacyWorkerUrl = `${baseUrl}music/ass-worker/subtitles-octopus-worker-legacy.js`;

                const instance = new SubtitlesOctopus({
                    canvas: canvas,
                    subUrl: currentTrack.assUrl,
                    fonts: currentTrack.fonts || [],
                    workerUrl,
                    legacyWorkerUrl,
                    renderMode: 'wasm-blend',
                    targetFps: 24, // 性能优先, 24fps 足够
                    prescaleFactor: 0.8,
                    prescaleHeightLimit: 1080,
                    maxRenderHeight: 720,
                    debug: false,
                });

                if (!disposed) {
                    octopusRef.current = instance;
                    // 初始化完成后立即同步当前时间
                    if (currentTime > 0) {
                        instance.setCurrentTime(currentTime);
                    }
                } else {
                    instance.dispose();
                }
            } catch (err) {
                console.error('[MusicPlayer] 加载 ASS 渲染器失败:', err);
            }
        };

        // 延迟一帧确保 DOM 已渲染
        const raf = requestAnimationFrame(() => {
            initOctopus();
        });

        return () => {
            disposed = true;
            cancelAnimationFrame(raf);
            if (initRetryRef.current) {
                clearTimeout(initRetryRef.current);
                initRetryRef.current = null;
            }
            if (octopusRef.current) {
                try { octopusRef.current.dispose(); } catch { /* ignore */ }
                octopusRef.current = null;
            }
        };
        // 注意: 不依赖 lyricsFullscreen, 因为全屏只改尺寸不重建
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [showLyrics, currentTrack?.assUrl, currentTrack?.fonts]);

    // ---- 同步播放时间到歌词渲染器 ----
    useEffect(() => {
        if (octopusRef.current && Math.abs(currentTime - lastTimeRef.current) > 0.03) {
            octopusRef.current.setCurrentTime(currentTime);
            lastTimeRef.current = currentTime;
        }
    }, [currentTime]);

    // ---- 全屏/尺寸变化时调整 canvas ----
    useEffect(() => {
        const canvas = canvasRef.current;
        const container = canvasContainerRef.current;
        if (!canvas || !container || !octopusRef.current) return;

        let w: number, h: number;
        if (lyricsFullscreen) {
            w = window.innerWidth;
            h = window.innerHeight;
        } else {
            const rect = container.getBoundingClientRect();
            w = Math.round(rect.width) || size.w;
            h = Math.round(rect.height) || (size.h - 32);
        }

        if (canvas.width !== w || canvas.height !== h) {
            canvas.width = w;
            canvas.height = h;
            octopusRef.current.resize(w, h);
        }
    }, [lyricsFullscreen, size]);

    // ---- 拖拽逻辑 ----
    const handlePointerDown = useCallback((e: React.PointerEvent) => {
        if (locked) return;
        if ((e.target as HTMLElement).dataset.dragHandle !== 'true') return;
        isDragging.current = true;
        dragOffset.current = {
            x: e.clientX - position.x,
            y: e.clientY - position.y,
        };
        (e.target as HTMLElement).setPointerCapture(e.pointerId);
    }, [position, locked]);

    const handlePointerMove = useCallback((e: React.PointerEvent) => {
        if (isDragging.current) {
            const newPos = {
                x: e.clientX - dragOffset.current.x,
                y: e.clientY - dragOffset.current.y,
            };
            setPosition(newPos);
        }
    }, []);

    const handlePointerUp = useCallback(() => {
        if (isDragging.current) {
            isDragging.current = false;
            savePosition(position);
        }
    }, [position]);

    // ---- 缩放逻辑 ----
    const handleResizePointerDown = useCallback((e: React.PointerEvent) => {
        if (locked) return;
        e.stopPropagation();
        e.preventDefault();
        isResizing.current = true;
        (e.target as HTMLElement).setPointerCapture(e.pointerId);
    }, [locked]);

    const handleResizePointerMove = useCallback((e: React.PointerEvent) => {
        if (isResizing.current && containerRef.current) {
            const rect = containerRef.current.getBoundingClientRect();
            const newW = Math.max(300, e.clientX - rect.left);
            const newH = Math.max(200, e.clientY - rect.top);
            setSize({ w: newW, h: newH });
        }
    }, []);

    const handleResizePointerUp = useCallback(() => {
        if (isResizing.current) {
            isResizing.current = false;
            saveSize(size);
            // 调整 canvas
            const canvas = canvasRef.current;
            const container = canvasContainerRef.current;
            if (canvas && container && octopusRef.current) {
                const rect = container.getBoundingClientRect();
                canvas.width = Math.round(rect.width);
                canvas.height = Math.round(rect.height);
                octopusRef.current.resize(canvas.width, canvas.height);
            }
        }
    }, [size]);

    // ---- 全屏切换（浏览器页面内全屏，非 Fullscreen API）----
    const handleFullscreen = useCallback(() => {
        toggleLyricsFullscreen();
    }, [toggleLyricsFullscreen]);

    if (!showLyrics || !currentTrack?.assUrl) return null;

    // ---- 全屏模式 ----
    if (lyricsFullscreen) {
        return (
            <div
                ref={containerRef}
                style={{
                    position: 'fixed',
                    inset: 0,
                    zIndex: 10000,
                    background: '#000',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                }}
            >
                <div style={{
                    position: 'absolute',
                    top: 12,
                    right: 12,
                    zIndex: 10001,
                    display: 'flex',
                    gap: 8,
                    opacity: 0.6,
                    transition: 'opacity 0.2s',
                }}
                    onMouseEnter={(e) => { (e.currentTarget as HTMLElement).style.opacity = '1'; }}
                    onMouseLeave={(e) => { (e.currentTarget as HTMLElement).style.opacity = '0.6'; }}
                >
                    <button onClick={handleFullscreen} style={controlBtnStyle} title="退出全屏">✕</button>
                </div>
                <div ref={canvasContainerRef} style={{ width: '100%', height: '100%', position: 'relative' }}>
                    <canvas ref={canvasRef} style={{ width: '100%', height: '100%', display: 'block' }} />
                </div>
            </div>
        );
    }

    // ---- 悬浮窗模式 ----
    return (
        <div
            ref={containerRef}
            onPointerMove={(e) => { handlePointerMove(e); handleResizePointerMove(e); }}
            onPointerUp={() => { handlePointerUp(); handleResizePointerUp(); }}
            style={{
                position: 'fixed',
                left: position.x,
                top: position.y,
                width: size.w,
                height: size.h,
                zIndex: 9999,
                background: locked ? 'transparent' : 'rgba(0, 0, 0, 0.85)',
                borderRadius: locked ? 0 : 8,
                boxShadow: locked ? 'none' : '0 4px 24px rgba(0,0,0,0.5)',
                display: 'flex',
                flexDirection: 'column',
                overflow: 'hidden',
                border: locked ? 'none' : '1px solid rgba(255,255,255,0.1)',
                userSelect: 'none',
                pointerEvents: locked ? 'none' : 'auto',
                transition: 'background 0.2s, box-shadow 0.2s, border 0.2s',
            }}
        >
            {/* 标题栏 (可拖拽) - 锁定时隐藏 */}
            {!locked && (
                <div
                    data-drag-handle="true"
                    onPointerDown={handlePointerDown}
                    style={{
                        height: 32,
                        minHeight: 32,
                        background: 'rgba(255,255,255,0.08)',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'space-between',
                        padding: '0 8px',
                        cursor: 'grab',
                        fontSize: 12,
                        color: 'rgba(255,255,255,0.7)',
                    }}
                >
                    <span data-drag-handle="true" style={{ pointerEvents: 'none' }}>
                        🎵 {currentTrack.title} - ASS 歌词
                    </span>
                    <div style={{ display: 'flex', gap: 4 }}>
                        <button onClick={toggleLock} style={controlBtnStyle} title="锁定 (全透明穿透)">🔓</button>
                        <button onClick={handleFullscreen} style={controlBtnStyle} title="页面全屏">⛶</button>
                        <button onClick={toggleLyrics} style={controlBtnStyle} title="关闭">✕</button>
                    </div>
                </div>
            )}

            {/* Canvas 区域 */}
            <div ref={canvasContainerRef} style={{ flex: 1, position: 'relative', overflow: 'hidden' }}>
                <canvas
                    ref={canvasRef}
                    style={{ width: '100%', height: '100%', display: 'block' }}
                />
            </div>

            {/* 锁定时: 悬浮的解锁按钮 (需要 pointer-events) */}
            {locked && (
                <button
                    onClick={toggleLock}
                    style={{
                        ...controlBtnStyle,
                        position: 'absolute',
                        top: 4,
                        right: 4,
                        pointerEvents: 'auto',
                        opacity: 0.3,
                        zIndex: 10,
                    }}
                    onMouseEnter={(e) => { (e.currentTarget as HTMLElement).style.opacity = '1'; }}
                    onMouseLeave={(e) => { (e.currentTarget as HTMLElement).style.opacity = '0.3'; }}
                    title="解锁"
                >
                    🔒
                </button>
            )}

            {/* 右下角缩放手柄 - 锁定时隐藏 */}
            {!locked && (
                <div
                    onPointerDown={handleResizePointerDown}
                    style={{
                        position: 'absolute',
                        right: 0,
                        bottom: 0,
                        width: 16,
                        height: 16,
                        cursor: 'nwse-resize',
                        background: 'linear-gradient(135deg, transparent 50%, rgba(255,255,255,0.3) 50%)',
                    }}
                />
            )}
        </div>
    );
}

const controlBtnStyle: React.CSSProperties = {
    background: 'rgba(255,255,255,0.15)',
    border: 'none',
    color: '#fff',
    width: 24,
    height: 24,
    borderRadius: 4,
    cursor: 'pointer',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    fontSize: 14,
    padding: 0,
    lineHeight: 1,
    transition: 'background 0.15s',
};
