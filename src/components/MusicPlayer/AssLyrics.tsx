/**
 * ASS 歌词渲染悬浮窗组件
 *
 * 使用 libass-wasm (JavascriptSubtitlesOctopus) 渲染 ASS 歌词
 * 支持拖拽移动、位置记忆、全屏观看、锁定(全透明)、位置重置
 *
 * 注意: SubtitlesOctopus 通过 <script> 标签加载到全局变量，
 * 不使用 require/import，因为 webpack 的 CJS/ESM interop 会破坏构造函数。
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

function loadPosition(): { x: number; y: number } {
    try {
        const raw = localStorage.getItem(POSITION_KEY);
        if (raw) return JSON.parse(raw);
    } catch { /* ignore */ }
    return defaultPosition();
}

function savePosition(pos: { x: number; y: number }): void {
    try { localStorage.setItem(POSITION_KEY, JSON.stringify(pos)); } catch { /* ignore */ }
}

function loadSize(): { w: number; h: number } {
    try {
        const raw = localStorage.getItem(SIZE_KEY);
        if (raw) return JSON.parse(raw);
    } catch { /* ignore */ }
    return { ...DEFAULT_SIZE };
}

function saveSize(size: { w: number; h: number }): void {
    try { localStorage.setItem(SIZE_KEY, JSON.stringify(size)); } catch { /* ignore */ }
}

// ========== Script loader (单例) ==========
let scriptLoaded = false;
let scriptLoading = false;
let scriptCallbacks: Array<() => void> = [];

/**
 * 通过 <script> 标签加载 subtitles-octopus.js
 * SubtitlesOctopus 会注册到全局变量 (因为源码末尾没有 UMD wrapper 来挂载到 window)
 * 不过它有: if (typeof exports !== 'undefined') { exports = module.exports = SubtitlesOctopus }
 * 在浏览器 <script> 标签下, exports 和 module 都是 undefined, 所以
 * SubtitlesOctopus 会留在闭包内但不会挂到 window 上。
 *
 * 解决方案: 我们手动把它放到 window 上。
 */
function loadSubtitlesOctopusScript(baseUrl: string, onReady: () => void): void {
    if ((window as any).SubtitlesOctopus) {
        scriptLoaded = true;
        onReady();
        return;
    }
    if (scriptLoaded) {
        onReady();
        return;
    }
    scriptCallbacks.push(onReady);
    if (scriptLoading) return;
    scriptLoading = true;

    // 通过 fetch 获取脚本内容并 eval, 同时将 module/exports mock 掉来捕获构造函数
    const scriptUrl = `${baseUrl}music/ass-worker/subtitles-octopus.js`;
    fetch(scriptUrl)
        .then((res) => res.text())
        .then((code) => {
            // 使用 Function 构造器来执行, 同时传入 mock 的 module/exports
            const fakeModule: any = { exports: {} };
            const fakeExports = fakeModule.exports;
            // 包装成函数执行，这样 subtitles-octopus.js 中的
            // `if (typeof exports !== 'undefined') { exports = module.exports = SubtitlesOctopus }`
            // 会把构造函数赋值给 fakeModule.exports
            const wrapper = new Function('module', 'exports', code);
            wrapper(fakeModule, fakeExports);

            // fakeModule.exports 现在应该是 SubtitlesOctopus 构造函数
            const Ctor = fakeModule.exports;
            if (typeof Ctor === 'function') {
                (window as any).SubtitlesOctopus = Ctor;
                console.log('[ASS] SubtitlesOctopus 构造函数加载成功');
            } else {
                console.error('[ASS] SubtitlesOctopus 加载后不是函数:', typeof Ctor);
            }

            scriptLoaded = true;
            scriptLoading = false;
            const cbs = scriptCallbacks.slice();
            scriptCallbacks = [];
            cbs.forEach((cb) => cb());
        })
        .catch((err) => {
            console.error('[ASS] 加载 subtitles-octopus.js 失败:', err);
            scriptLoading = false;
        });
}

export default function AssLyrics(): React.ReactElement | null {
    const showLyrics = useMusicStore((s) => s.showLyrics);
    const lyricsFullscreen = useMusicStore((s) => s.lyricsFullscreen);
    const trackIndex = useMusicStore((s) => s.trackIndex);
    const pl = useMusicStore((s) => s.playlist);
    const toggleLyrics = useMusicStore((s) => s.toggleLyrics);
    const toggleLyricsFullscreen = useMusicStore((s) => s.toggleLyricsFullscreen);
    const { siteConfig } = useDocusaurusContext();
    const baseUrl = siteConfig.baseUrl;

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

    const resetPosition = useCallback(() => {
        const pos = defaultPosition();
        const sz = { ...DEFAULT_SIZE };
        setPosition(pos);
        setSize(sz);
        savePosition(pos);
        saveSize(sz);
    }, []);

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
                const ct = useMusicStore.getState().currentTime;
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
                if (!disposed) {
                    initRetryRef.current = setTimeout(initOctopus, 150);
                }
                return;
            }

            const rect = container.getBoundingClientRect();
            const pixelW = Math.round(rect.width);
            const pixelH = Math.round(rect.height);

            if (pixelW <= 0 || pixelH <= 0) {
                console.warn('[ASS] 容器尺寸为 0，等待布局...');
                if (!disposed) {
                    initRetryRef.current = setTimeout(initOctopus, 200);
                }
                return;
            }

            // 设置 canvas 像素尺寸
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

            // 通过全局构造函数创建实例
            const Ctor = (window as any).SubtitlesOctopus;
            if (!Ctor) {
                console.error('[ASS] SubtitlesOctopus 构造函数未找到!');
                return;
            }

            try {
                const instance = new Ctor({
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
                    debug: true, // 打开调试以便排查
                    onReady: () => {
                        console.log('[ASS] SubtitlesOctopus 就绪, canvas:', canvas.width, 'x', canvas.height);
                        // 就绪后立即同步当前时间
                        const ct = useMusicStore.getState().currentTime;
                        try { instance.setCurrentTime(ct); } catch { /* ignore */ }
                    },
                    onError: (err: any) => {
                        console.error('[ASS] SubtitlesOctopus 错误:', err);
                    },
                });

                if (!disposed) {
                    octopusRef.current = instance;
                    console.log('[ASS] SubtitlesOctopus 实例创建成功');
                } else {
                    instance.dispose();
                }
            } catch (err) {
                console.error('[ASS] 创建 SubtitlesOctopus 实例失败:', err);
            }
        };

        // 先确保脚本已加载，然后延迟两帧确保 DOM 布局完成
        loadSubtitlesOctopusScript(baseUrl, () => {
            if (disposed) return;
            requestAnimationFrame(() => {
                requestAnimationFrame(() => {
                    if (!disposed) {
                        initOctopus();
                    }
                });
            });
        });

        return () => {
            disposed = true;
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
