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

/** 默认尺寸 */
const DEFAULT_SIZE = { w: 500, h: 350 };

/** 默认位置 (浏览器居中) */
function defaultPosition(): { x: number; y: number } {
    if (typeof window === 'undefined') return { x: 100, y: 100 };
    const x = Math.max(0, Math.round((window.innerWidth - DEFAULT_SIZE.w) / 2));
    const y = Math.max(0, Math.round((window.innerHeight - DEFAULT_SIZE.h) / 2));
    return { x, y };
}

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
    const [locked, setLocked] = useState(false);
    const [, forceUpdate] = useState(0);
    const isDragging = useRef(false);
    const isResizing = useRef(false);
    const dragOffset = useRef({ x: 0, y: 0 });
    const rafIdRef = useRef<number | null>(null);
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

    // 监听外部触发的重置位置事件
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

    // ---- 持续推送 currentTime 到 octopus 的 RAF 循环 ----
    useEffect(() => {
        if (!showLyrics) return;

        let lastPushedTime = -1;

        const tick = () => {
            const oct = octopusRef.current;
            if (oct) {
                // 直接从 store 读取最新 currentTime (不通过 React 订阅，避免重渲染)
                const ct = useMusicStore.getState().currentTime;
                // 只在时间变化时推送 (阈值 20ms)
                if (Math.abs(ct - lastPushedTime) > 0.02) {
                    try {
                        oct.setCurrentTime(ct);
                    } catch {
                        // octopus 可能已销毁
                    }
                    lastPushedTime = ct;
                }
            }
            rafIdRef.current = requestAnimationFrame(tick);
        };

        rafIdRef.current = requestAnimationFrame(tick);

        return () => {
            if (rafIdRef.current !== null) {
                cancelAnimationFrame(rafIdRef.current);
                rafIdRef.current = null;
            }
        };
    }, [showLyrics]);

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

        const initOctopus = () => {
            const canvas = canvasRef.current;
            const container = canvasContainerRef.current;
            if (!canvas || !container) {
                // canvas 还未挂载, 重试
                if (!disposed) {
                    initRetryRef.current = setTimeout(initOctopus, 150);
                }
                return;
            }

            // 获取容器的实际像素尺寸
            const rect = container.getBoundingClientRect();
            const pixelW = Math.round(rect.width);
            const pixelH = Math.round(rect.height);

            if (pixelW <= 0 || pixelH <= 0) {
                // 容器还没有尺寸，重试
                console.warn('[ASS] 容器尺寸为 0，等待布局完成...');
                if (!disposed) {
                    initRetryRef.current = setTimeout(initOctopus, 200);
                }
                return;
            }

            // 设置 canvas 像素尺寸（关键！必须在 SubtitlesOctopus 初始化前设置）
            canvas.width = pixelW;
            canvas.height = pixelH;

            console.log(`[ASS] 初始化 canvas: ${pixelW}x${pixelH}, assUrl: ${currentTrack.assUrl}`);

            // 销毁旧实例
            if (octopusRef.current) {
                try { octopusRef.current.dispose(); } catch { /* ignore */ }
                octopusRef.current = null;
            }

            const workerUrl = `${baseUrl}music/ass-worker/subtitles-octopus-worker.js`;
            const legacyWorkerUrl = `${baseUrl}music/ass-worker/subtitles-octopus-worker-legacy.js`;

            try {
                // 直接使用全局构造函数或通过 require
                // libass-wasm 的 subtitles-octopus.js 是一个传统的构造函数
                const SubtitlesOctopus = require('libass-wasm');

                const instance = new SubtitlesOctopus({
                    canvas: canvas,
                    subUrl: currentTrack.assUrl,
                    fonts: currentTrack.fonts || [],
                    workerUrl,
                    legacyWorkerUrl,
                    renderMode: 'wasm-blend',
                    targetFps: 24,
                    prescaleFactor: 0.8,
                    prescaleHeightLimit: 1080,
                    maxRenderHeight: 720,
                    debug: false,
                    onReady: () => {
                        console.log('[ASS] SubtitlesOctopus 就绪');
                    },
                    onError: (err: any) => {
                        console.error('[ASS] SubtitlesOctopus 错误:', err);
                    },
                });

                if (!disposed) {
                    octopusRef.current = instance;
                    // 立即同步当前时间
                    const ct = useMusicStore.getState().currentTime;
                    if (ct > 0) {
                        instance.setCurrentTime(ct);
                    }
                    console.log('[ASS] SubtitlesOctopus 实例创建成功');
                } else {
                    instance.dispose();
                }
            } catch (err) {
                console.error('[ASS] 加载 ASS 渲染器失败:', err);
                // 尝试通过 script 标签加载
                if (!disposed) {
                    loadOctopusViaScript(canvas, workerUrl, legacyWorkerUrl, disposed);
                }
            }
        };

        /** 通过 script 标签加载 (备用方案) */
        const loadOctopusViaScript = (
            canvas: HTMLCanvasElement,
            workerUrl: string,
            legacyWorkerUrl: string,
            isDisposed: boolean,
        ) => {
            // 检查全局是否已经有 SubtitlesOctopus
            if ((window as any).SubtitlesOctopus) {
                createFromGlobal(canvas, workerUrl, legacyWorkerUrl, isDisposed);
                return;
            }

            const script = document.createElement('script');
            script.src = `${baseUrl}music/ass-worker/subtitles-octopus.js`;
            script.onload = () => {
                if (!isDisposed) {
                    createFromGlobal(canvas, workerUrl, legacyWorkerUrl, isDisposed);
                }
            };
            script.onerror = () => {
                console.error('[ASS] 加载 subtitles-octopus.js 脚本失败');
            };
            document.head.appendChild(script);
        };

        const createFromGlobal = (
            canvas: HTMLCanvasElement,
            workerUrl: string,
            legacyWorkerUrl: string,
            isDisposed: boolean,
        ) => {
            const Ctor = (window as any).SubtitlesOctopus;
            if (!Ctor) return;

            const instance = new Ctor({
                canvas: canvas,
                subUrl: currentTrack!.assUrl,
                fonts: currentTrack!.fonts || [],
                workerUrl,
                legacyWorkerUrl,
                renderMode: 'wasm-blend',
                targetFps: 24,
                prescaleFactor: 0.8,
                prescaleHeightLimit: 1080,
                maxRenderHeight: 720,
                debug: false,
            });

            if (!isDisposed) {
                octopusRef.current = instance;
                console.log('[ASS] SubtitlesOctopus 实例创建成功 (via script)');
            } else {
                instance.dispose();
            }
        };

        // 延迟一帧以确保 DOM 完成布局
        const raf = requestAnimationFrame(() => {
            // 再延迟一帧，双重保险
            requestAnimationFrame(() => {
                if (!disposed) {
                    initOctopus();
                }
            });
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
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [showLyrics, currentTrack?.assUrl, currentTrack?.fonts, baseUrl]);

    // ---- 全屏/尺寸变化时调整 canvas ----
    useEffect(() => {
        const canvas = canvasRef.current;
        const container = canvasContainerRef.current;
        if (!canvas || !container || !octopusRef.current) return;

        // 延迟一帧，等布局完成
        const raf = requestAnimationFrame(() => {
            let w: number, h: number;
            if (lyricsFullscreen) {
                w = window.innerWidth;
                h = window.innerHeight;
            } else {
                const rect = container.getBoundingClientRect();
                w = Math.round(rect.width) || size.w;
                h = Math.round(rect.height) || (size.h - 32);
            }

            if (w > 0 && h > 0 && (canvas.width !== w || canvas.height !== h)) {
                canvas.width = w;
                canvas.height = h;
                try {
                    octopusRef.current?.resize(w, h);
                } catch { /* ignore */ }
            }
        });

        return () => cancelAnimationFrame(raf);
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
                const w = Math.round(rect.width);
                const h = Math.round(rect.height);
                if (w > 0 && h > 0) {
                    canvas.width = w;
                    canvas.height = h;
                    try {
                        octopusRef.current.resize(w, h);
                    } catch { /* ignore */ }
                }
            }
        }
    }, [size]);

    // ---- 全屏切换 ----
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

            {/* 锁定时: 悬浮的解锁按钮 */}
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
