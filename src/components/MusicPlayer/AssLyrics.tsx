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
const LOCK_KEY = 'hxloli-lyrics-locked';

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

function loadLocked(): boolean {
    try {
        return localStorage.getItem(LOCK_KEY) === 'true';
    } catch { /* ignore */ }
    return false;
}

function saveLocked(locked: boolean): void {
    try { localStorage.setItem(LOCK_KEY, JSON.stringify(locked)); } catch { /* ignore */ }
}

// ========== Script loader (单例) ==========
let scriptLoaded = false;
let scriptLoading = false;
let scriptCallbacks: Array<() => void> = [];

/**
 * 通过 <script> 标签加载 subtitles-octopus.js
 * 
 * subtitles-octopus.js 中用 `var SubtitlesOctopus = function(options) { ... };` 声明，
 * 在 <script> 标签中 `var` 声明会自动注册到 window 全局对象上。
 */
function loadSubtitlesOctopusScript(baseUrl: string, onReady: () => void): void {
    if ((window as any).SubtitlesOctopus) {
        console.log('[ASS] SubtitlesOctopus 已存在于 window');
        scriptLoaded = true;
        onReady();
        return;
    }
    if (scriptLoaded) {
        console.log('[ASS] 脚本已加载但 window.SubtitlesOctopus 不存在!');
        onReady();
        return;
    }
    scriptCallbacks.push(onReady);
    if (scriptLoading) return;
    scriptLoading = true;

    const scriptUrl = `${baseUrl}music/ass-worker/subtitles-octopus.js`;
    console.log('[ASS] 开始加载脚本:', scriptUrl);

    const script = document.createElement('script');
    script.src = scriptUrl;
    script.async = true;

    script.onload = () => {
        console.log('[ASS] 脚本加载完成, window.SubtitlesOctopus:', typeof (window as any).SubtitlesOctopus);

        if (typeof (window as any).SubtitlesOctopus !== 'function') {
            console.error('[ASS] 脚本加载后 window.SubtitlesOctopus 不是函数! 尝试 fetch+eval 方式...');
            // 兜底: 用 fetch + eval 方式
            fetch(scriptUrl)
                .then(r => r.text())
                .then(code => {
                    const fakeModule: any = { exports: {} };
                    const wrapper = new Function('module', 'exports', code);
                    wrapper(fakeModule, fakeModule.exports);
                    if (typeof fakeModule.exports === 'function') {
                        (window as any).SubtitlesOctopus = fakeModule.exports;
                        console.log('[ASS] fetch+eval 方式成功!');
                    } else {
                        console.error('[ASS] fetch+eval 也失败了:', typeof fakeModule.exports);
                    }
                    finishLoading();
                })
                .catch(err => {
                    console.error('[ASS] fetch+eval 失败:', err);
                    finishLoading();
                });
        } else {
            finishLoading();
        }
    };

    script.onerror = (err) => {
        console.error('[ASS] 脚本加载失败:', err);
        scriptLoading = false;
    };

    function finishLoading() {
        scriptLoaded = true;
        scriptLoading = false;
        const cbs = scriptCallbacks.slice();
        scriptCallbacks = [];
        cbs.forEach(cb => cb());
    }

    document.head.appendChild(script);
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
    const [locked, setLocked] = useState(loadLocked);
    const [, forceUpdate] = useState(0);
    // 每次递增此值会触发 octopus 重建（用于缩放结束后重新初始化以适应新尺寸）
    const [rebuildToken, setRebuildToken] = useState(0);
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
        setLocked((prev) => {
            const next = !prev;
            saveLocked(next);
            return next;
        });
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

    // 跨标签页同步：监听 localStorage 的 storage 事件，实时同步位置/大小/锁定状态
    useEffect(() => {
        const handleStorage = (e: StorageEvent) => {
            if (!e.key || !e.newValue) return;
            try {
                switch (e.key) {
                    case POSITION_KEY:
                        setPosition(JSON.parse(e.newValue));
                        break;
                    case SIZE_KEY:
                        setSize(JSON.parse(e.newValue));
                        break;
                    case LOCK_KEY:
                        setLocked(e.newValue === 'true');
                        break;
                }
            } catch { /* ignore */ }
        };
        window.addEventListener('storage', handleStorage);
        return () => window.removeEventListener('storage', handleStorage);
    }, []);

    // ---- 页面可见性：仅当页面可见时渲染 ASS ----
    // 不再要求必须是 Leader 才渲染，Follower 也可以渲染（通过时间插值实现流畅）
    const [pageVisible, setPageVisible] = useState(() => typeof document !== 'undefined' ? !document.hidden : true);

    useEffect(() => {
        const handler = () => setPageVisible(!document.hidden);
        document.addEventListener('visibilitychange', handler);
        return () => document.removeEventListener('visibilitychange', handler);
    }, []);

    // ---- 持续推送 currentTime 到 octopus 的 RAF 循环 ----
    // Leader 和 Follower 都渲染 ASS：
    //   - Leader: 使用 audio.currentTime (精确)
    //   - Follower: 使用 getInterpolatedTime() (基于同步时间 + 本地时间推算，流畅)
    const shouldRender = showLyrics && pageVisible;

    useEffect(() => {
        if (!shouldRender) return;

        // 上次推送给 octopus 的时间 (秒)
        // 确保只前进不后退，避免 libass 重复渲染同一区间导致抽搐
        let lastPushedTime = -1;

        const tick = () => {
            const oct = octopusRef.current;
            if (oct) {
                const ct = useMusicStore.getState().getInterpolatedTime();
                // 只在时间前进超过 16ms（约1帧@60fps）时才推送
                // 如果 ct < lastPushedTime（时间倒退），忽略——让时间自然追上来
                // 除非倒退幅度 > 0.5s（说明是 seek/换曲），此时需要跳转
                const delta = ct - lastPushedTime;
                if (delta > 0.016 || delta < -0.5) {
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
    }, [shouldRender]);

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
            const canvas = canvasRef.current;
            if (!canvas) {
                if (!disposed) {
                    initRetryRef.current = setTimeout(initOctopus, 150);
                }
                return;
            }

            // 直接使用状态值计算 canvas 尺寸（避免 getBoundingClientRect 布局延迟）
            let pixelW: number, pixelH: number;
            if (lyricsFullscreen) {
                pixelW = window.innerWidth;
                pixelH = window.innerHeight;
            } else {
                const titleBarHeight = locked ? 0 : 32;
                pixelW = Math.round(size.w);
                pixelH = Math.round(size.h - titleBarHeight);
            }

            if (pixelW <= 0 || pixelH <= 0) {
                console.warn('[ASS] 计算的 canvas 尺寸无效:', pixelW, pixelH);
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

            // 注意: Worker 中 fetch 相对路径会相对于 Worker 脚本位置，所以必须用完整绝对 URL
            const origin = typeof window !== 'undefined' ? window.location.origin : '';

            const workerUrl = `${origin}${baseUrl}music/ass-worker/subtitles-octopus-worker.js`;
            const legacyWorkerUrl = `${origin}${baseUrl}music/ass-worker/subtitles-octopus-worker-legacy.js`;

            // 通过全局构造函数创建实例
            const Ctor = (window as any).SubtitlesOctopus;
            if (!Ctor) {
                console.error('[ASS] SubtitlesOctopus 构造函数未找到!');
                return;
            }

            // 字体配置：Worker 通过 fetch 下载字体文件，需要完整可访问的 URL
            // 核心策略: 用 fallbackFont 指向 CJK 字体，所有 ASS 中未找到的字体都会 fallback 到它
            // 不在 availableFonts 中映射大量字体名 → 避免 Worker 对同一个 17MB 文件重复下载导致 OOM
            const cjkFallbackUrl = `${origin}${baseUrl}music/fonts/NotoSansSC-Regular.ttf`;

            // availableFonts 保持空对象：所有字体都走 fallbackFont
            const availableFonts: Record<string, string> = {};

            // 辅助函数：对 URL 进行编码（处理日文/中文/空格等特殊字符）
            const safeUrl = (url: string): string => {
                // 如果已经编码过（包含 %xx）则不重复编码
                if (/%[0-9A-Fa-f]{2}/.test(url)) return url;
                return encodeURI(url);
            };

            // fonts 数组置空: fallbackFont 已指向 CJK 字体，无需重复加载
            const fontsFullUrls: string[] = [];

            // subUrl 用完整绝对 URL + 编码
            const assUrlRaw = currentTrack.assUrl!;
            const subFullUrl = safeUrl(
                assUrlRaw.startsWith('http') ? assUrlRaw : `${origin}${assUrlRaw}`
            );

            // 先在前端 fetch ASS 文件内容，避免 Worker 中同步 XHR 对特殊字符 URL 的编码问题
            console.log('[ASS] 正在 fetch ASS 文件:', subFullUrl);
            let subContent: string | null = null;
            try {
                const resp = await fetch(subFullUrl);
                if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
                subContent = await resp.text();
                console.log(`[ASS] ASS 文件加载成功, 大小: ${subContent.length} 字符`);
            } catch (fetchErr) {
                console.error('[ASS] fetch ASS 文件失败:', fetchErr);
                return;
            }

            if (disposed) return;

            try {
                const instance = new Ctor({
                    canvas: canvas,
                    subContent: subContent,
                    fonts: fontsFullUrls,
                    availableFonts,
                    fallbackFont: safeUrl(cjkFallbackUrl),
                    // 注意: lazyFileLoading 需要服务器支持 Range 请求，dev server 可能不支持
                    lazyFileLoading: false,
                    workerUrl: safeUrl(workerUrl),
                    legacyWorkerUrl: safeUrl(legacyWorkerUrl),
                    renderMode: 'wasm-blend',
                    targetFps: 24,
                    prescaleFactor: 0.8,
                    prescaleHeightLimit: 1080,
                    maxRenderHeight: 720,
                    debug: true, // 打开调试以便排查
                    onReady: () => {
                        console.log('[ASS] SubtitlesOctopus 就绪, canvas:', canvas.width, 'x', canvas.height);
                        // 就绪后立即同步当前时间
                        const ct = useMusicStore.getState().getInterpolatedTime();
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
    }, [showLyrics, currentTrack?.assUrl, currentTrack?.fonts, baseUrl, lyricsFullscreen, size, locked, rebuildToken]);

    // ---- 全屏切换 / 浏览器窗口 resize 时，通过 rebuildToken 触发 octopus 重建 ----
    // (SubtitlesOctopus.resize() 不会重新计算 ASS 内部缩放比例，必须销毁重建才能真正自适应)
    useEffect(() => {
        if (!showLyrics) return;
        const handleWindowResize = () => {
            // 全屏模式下窗口尺寸即 canvas 尺寸，需要重建
            if (useMusicStore.getState().lyricsFullscreen) {
                setRebuildToken((n) => n + 1);
            }
        };
        window.addEventListener('resize', handleWindowResize);
        return () => window.removeEventListener('resize', handleWindowResize);
    }, [showLyrics]);

    // 全屏切换时也触发重建
    const prevFullscreen = useRef(lyricsFullscreen);
    useEffect(() => {
        if (prevFullscreen.current !== lyricsFullscreen) {
            prevFullscreen.current = lyricsFullscreen;
            setRebuildToken((n) => n + 1);
        }
    }, [lyricsFullscreen]);

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
            // 缩放结束后，通过 rebuildToken 触发 octopus 重建以适应新的 canvas 尺寸
            setRebuildToken((n) => n + 1);
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
