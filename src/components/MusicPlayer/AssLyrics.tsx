/**
 * ASS 歌词渲染悬浮窗组件
 *
 * 使用 libass-wasm (JavascriptSubtitlesOctopus) 渲染 ASS 歌词
 * 支持拖拽移动、位置记忆、全屏观看、锁定(全透明)、位置重置
 *
 * 重构: 操作栏浮动在 canvas 上方, 不占据布局空间
 * 锁定/解锁不会改变 canvas 尺寸, 避免位置漂移
 *
 * 注意: SubtitlesOctopus 通过 <script> 标签加载到全局变量，
 * 不使用 require/import，因为 webpack 的 CJS/ESM interop 会破坏构造函数。
 */
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useMusicStore } from '@site/src/utils/music/musicStore';
import React, { useCallback, useEffect, useRef, useState } from 'react';
import {
    FaAlignCenter,
    FaBackward,
    FaExpand,
    FaFastBackward,
    FaForward,
    FaLock,
    FaLockOpen,
    FaPause,
    FaPlay,
    FaStepBackward,
    FaStepForward,
    FaTimes,
    FaUndo,
} from 'react-icons/fa';

/** 全局重置位置的事件名 */
const RESET_POSITION_EVENT = 'hxloli-lyrics-reset-position';

const POSITION_KEY = 'hxloli-lyrics-position';
const SIZE_KEY = 'hxloli-lyrics-size';
const LOCK_KEY = 'hxloli-lyrics-locked';
const SUBTITLE_OFFSET_KEY = 'hxloli-lyrics-subtitle-offset';

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

function loadSubtitleOffset(): number {
    try {
        const raw = localStorage.getItem(SUBTITLE_OFFSET_KEY);
        if (raw) return parseFloat(raw) || 0;
    } catch { /* ignore */ }
    return 0;
}

function saveSubtitleOffset(offset: number): void {
    try { localStorage.setItem(SUBTITLE_OFFSET_KEY, JSON.stringify(offset)); } catch { /* ignore */ }
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

/** 字幕时间偏移量 (模块级变量, 供 RAF 循环读取) */
let globalSubtitleOffset = loadSubtitleOffset();

export default function AssLyrics(): React.ReactElement | null {
    const showLyrics = useMusicStore((s) => s.showLyrics);
    const lyricsFullscreen = useMusicStore((s) => s.lyricsFullscreen);
    const trackIndex = useMusicStore((s) => s.trackIndex);
    const pl = useMusicStore((s) => s.playlist);
    const isPlaying = useMusicStore((s) => s.isPlaying);
    const toggleLyrics = useMusicStore((s) => s.toggleLyrics);
    const toggleLyricsFullscreen = useMusicStore((s) => s.toggleLyricsFullscreen);
    const toggle = useMusicStore((s) => s.toggle);
    const next = useMusicStore((s) => s.next);
    const prev = useMusicStore((s) => s.prev);
    const seek = useMusicStore((s) => s.seek);
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
    const [subtitleOffset, setSubtitleOffset] = useState(loadSubtitleOffset);
    const [toolbarVisible, setToolbarVisible] = useState(false);
    const [, forceUpdate] = useState(0);
    const [rebuildToken, setRebuildToken] = useState(0);
    const isDragging = useRef(false);
    const isResizing = useRef(false);
    const dragOffset = useRef({ x: 0, y: 0 });
    const rafIdRef = useRef<number | null>(null);
    const initRetryRef = useRef<ReturnType<typeof setTimeout> | null>(null);
    const toolbarTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

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

    /** 字幕慢 0.5s (字幕提前显示, 即时间偏移减少) */
    const subtitleSlower = useCallback(() => {
        setSubtitleOffset((prev) => {
            const next = +(prev - 0.5).toFixed(1);
            globalSubtitleOffset = next;
            saveSubtitleOffset(next);
            return next;
        });
    }, []);

    /** 字幕快 0.5s (字幕延后显示, 即时间偏移增加) */
    const subtitleFaster = useCallback(() => {
        setSubtitleOffset((prev) => {
            const next = +(prev + 0.5).toFixed(1);
            globalSubtitleOffset = next;
            saveSubtitleOffset(next);
            return next;
        });
    }, []);

    /** 重置字幕时间偏移 */
    const resetSubtitleOffset = useCallback(() => {
        setSubtitleOffset(0);
        globalSubtitleOffset = 0;
        saveSubtitleOffset(0);
    }, []);

    /** 回到开头 */
    const seekToStart = useCallback(() => {
        seek(0);
    }, [seek]);

    /** 窗口水平居中 */
    const centerHorizontally = useCallback(() => {
        if (typeof window === 'undefined') return;
        const x = Math.max(0, Math.round((window.innerWidth - size.w) / 2));
        const newPos = { x, y: position.y };
        setPosition(newPos);
        savePosition(newPos);
    }, [size.w, position.y]);

    // 工具栏自动隐藏逻辑
    const showToolbar = useCallback(() => {
        setToolbarVisible(true);
        if (toolbarTimerRef.current) clearTimeout(toolbarTimerRef.current);
        toolbarTimerRef.current = setTimeout(() => {
            if (!isDragging.current) {
                setToolbarVisible(false);
            }
        }, 3000);
    }, []);

    const hideToolbar = useCallback(() => {
        if (toolbarTimerRef.current) clearTimeout(toolbarTimerRef.current);
        toolbarTimerRef.current = setTimeout(() => {
            if (!isDragging.current) {
                setToolbarVisible(false);
            }
        }, 500);
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

    // 跨标签页同步
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
                    case SUBTITLE_OFFSET_KEY: {
                        const val = parseFloat(e.newValue) || 0;
                        setSubtitleOffset(val);
                        globalSubtitleOffset = val;
                        break;
                    }
                }
            } catch { /* ignore */ }
        };
        window.addEventListener('storage', handleStorage);
        return () => window.removeEventListener('storage', handleStorage);
    }, []);

    // 页面可见性
    const [pageVisible, setPageVisible] = useState(() => typeof document !== 'undefined' ? !document.hidden : true);

    useEffect(() => {
        const handler = () => setPageVisible(!document.hidden);
        document.addEventListener('visibilitychange', handler);
        return () => document.removeEventListener('visibilitychange', handler);
    }, []);

    // ---- 持续推送 currentTime 到 octopus 的 RAF 循环 (含字幕偏移) ----
    const shouldRender = showLyrics && pageVisible;

    useEffect(() => {
        if (!shouldRender) return;

        let lastPushedTime = -1;

        const tick = () => {
            const oct = octopusRef.current;
            if (oct) {
                const ct = useMusicStore.getState().getInterpolatedTime() + globalSubtitleOffset;
                const adjustedCt = Math.max(0, ct);
                const delta = adjustedCt - lastPushedTime;
                if (delta > 0.016 || delta < -0.5) {
                    try {
                        oct.setCurrentTime(adjustedCt);
                    } catch {
                        // octopus 可能已销毁
                    }
                    lastPushedTime = adjustedCt;
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
    // 注意: canvas 尺寸不再受 locked 影响, 始终使用 size.w x size.h
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

            let pixelW: number, pixelH: number;
            if (lyricsFullscreen) {
                pixelW = window.innerWidth;
                pixelH = window.innerHeight;
            } else {
                // 不再减去标题栏高度, canvas 始终占满整个容器
                pixelW = Math.round(size.w);
                pixelH = Math.round(size.h);
            }

            if (pixelW <= 0 || pixelH <= 0) {
                console.warn('[ASS] 计算的 canvas 尺寸无效:', pixelW, pixelH);
                if (!disposed) {
                    initRetryRef.current = setTimeout(initOctopus, 200);
                }
                return;
            }

            canvas.width = pixelW;
            canvas.height = pixelH;

            console.log(`[ASS] 初始化 canvas: ${pixelW}x${pixelH}, assUrl: ${currentTrack.assUrl}`);

            if (octopusRef.current) {
                try { octopusRef.current.dispose(); } catch { /* ignore */ }
                octopusRef.current = null;
            }

            const origin = typeof window !== 'undefined' ? window.location.origin : '';
            const workerUrl = `${origin}${baseUrl}music/ass-worker/subtitles-octopus-worker.js`;
            const legacyWorkerUrl = `${origin}${baseUrl}music/ass-worker/subtitles-octopus-worker-legacy.js`;

            const Ctor = (window as any).SubtitlesOctopus;
            if (!Ctor) {
                console.error('[ASS] SubtitlesOctopus 构造函数未找到!');
                return;
            }

            const cjkFallbackUrl = `${origin}${baseUrl}music/fonts/NotoSansSC-Regular.ttf`;
            const availableFonts: Record<string, string> = {};

            const safeUrl = (url: string): string => {
                if (/%[0-9A-Fa-f]{2}/.test(url)) return url;
                return encodeURI(url);
            };

            const fontsFullUrls: string[] = [];

            const assUrlRaw = currentTrack.assUrl!;
            const subFullUrl = safeUrl(
                assUrlRaw.startsWith('http') ? assUrlRaw : `${origin}${assUrlRaw}`
            );

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
                    lazyFileLoading: false,
                    workerUrl: safeUrl(workerUrl),
                    legacyWorkerUrl: safeUrl(legacyWorkerUrl),
                    renderMode: 'wasm-blend',
                    targetFps: 24,
                    prescaleFactor: 0.8,
                    prescaleHeightLimit: 1080,
                    maxRenderHeight: 720,
                    debug: true,
                    onReady: () => {
                        console.log('[ASS] SubtitlesOctopus 就绪, canvas:', canvas.width, 'x', canvas.height);
                        const ct = useMusicStore.getState().getInterpolatedTime() + globalSubtitleOffset;
                        try { instance.setCurrentTime(Math.max(0, ct)); } catch { /* ignore */ }
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
    }, [showLyrics, currentTrack?.assUrl, currentTrack?.fonts, baseUrl, lyricsFullscreen, size, rebuildToken]);

    // ---- 全屏切换 / 浏览器窗口 resize 时重建 ----
    useEffect(() => {
        if (!showLyrics) return;
        const handleWindowResize = () => {
            if (useMusicStore.getState().lyricsFullscreen) {
                setRebuildToken((n) => n + 1);
            }
        };
        window.addEventListener('resize', handleWindowResize);
        return () => window.removeEventListener('resize', handleWindowResize);
    }, [showLyrics]);

    const prevFullscreen = useRef(lyricsFullscreen);
    useEffect(() => {
        if (prevFullscreen.current !== lyricsFullscreen) {
            prevFullscreen.current = lyricsFullscreen;
            setRebuildToken((n) => n + 1);
        }
    }, [lyricsFullscreen]);

    // ---- 拖拽逻辑 (整个工具栏可拖拽) ----
    const handlePointerDown = useCallback((e: React.PointerEvent) => {
        if (locked) return;
        // 只有拖拽手柄区域才启动拖拽 (按钮上不触发)
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
            setRebuildToken((n) => n + 1);
        }
    }, [size]);

    const handleFullscreen = useCallback(() => {
        toggleLyricsFullscreen();
    }, [toggleLyricsFullscreen]);

    // 清理工具栏定时器
    useEffect(() => {
        return () => {
            if (toolbarTimerRef.current) clearTimeout(toolbarTimerRef.current);
        };
    }, []);

    if (!showLyrics || !currentTrack?.assUrl) return null;

    // ---- 工具栏内容 ----
    const toolbarContent = (
        <div style={{
            display: 'flex',
            alignItems: 'center',
            gap: 2,
            flexWrap: 'wrap',
            justifyContent: 'center',
        }}>
            {/* 第一组: 播放控制 */}
            <button onClick={prev} style={toolbarBtnStyle} title="上一首"><FaStepBackward size={11} /></button>
            <button onClick={toggle} style={toolbarBtnStyle} title={isPlaying ? '暂停' : '播放'}>
                {isPlaying ? <FaPause size={11} /> : <FaPlay size={11} />}
            </button>
            <button onClick={next} style={toolbarBtnStyle} title="下一首"><FaStepForward size={11} /></button>
            <button onClick={seekToStart} style={toolbarBtnStyle} title="回到开头"><FaFastBackward size={11} /></button>

            <span style={separatorStyle}>│</span>

            {/* 第二组: 字幕调整 */}
            <button onClick={subtitleSlower} style={toolbarBtnStyle} title="字幕慢 0.5s (提前显示)">
                <FaBackward size={10} />
            </button>
            <button onClick={subtitleFaster} style={toolbarBtnStyle} title="字幕快 0.5s (延后显示)">
                <FaForward size={10} />
            </button>
            {subtitleOffset !== 0 && (
                <span style={{ color: 'rgba(255,255,255,0.6)', fontSize: 10, margin: '0 2px', whiteSpace: 'nowrap' }}>
                    {subtitleOffset > 0 ? '+' : ''}{subtitleOffset.toFixed(1)}s
                </span>
            )}
            <button onClick={resetSubtitleOffset} style={toolbarBtnStyle} title="重置字幕时间偏移">
                <FaUndo size={10} />
            </button>
            <button
                onClick={toggleLock}
                style={{
                    ...toolbarBtnStyle,
                    background: locked ? 'rgba(255, 180, 0, 0.4)' : toolbarBtnStyle.background,
                }}
                title={locked ? '解锁 (恢复可交互)' : '锁定 (全透明穿透)'}
            >
                {locked ? <FaLock size={11} /> : <FaLockOpen size={11} />}
            </button>

            <span style={separatorStyle}>│</span>

            {/* 第三组: 窗口控制 */}
            <button onClick={centerHorizontally} style={toolbarBtnStyle} title="窗口水平居中">
                <FaAlignCenter size={11} />
            </button>
            <button onClick={handleFullscreen} style={toolbarBtnStyle} title="页面全屏"><FaExpand size={11} /></button>
            <button onClick={toggleLyrics} style={toolbarBtnStyle} title="关闭"><FaTimes size={11} /></button>
        </div>
    );

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
                onMouseEnter={showToolbar}
                onMouseMove={showToolbar}
                onMouseLeave={hideToolbar}
            >
                {/* 全屏模式工具栏 */}
                <div style={{
                    position: 'absolute',
                    top: 0,
                    left: 0,
                    right: 0,
                    zIndex: 10001,
                    display: 'flex',
                    justifyContent: 'center',
                    padding: '8px 12px',
                    background: 'linear-gradient(to bottom, rgba(0,0,0,0.7) 0%, transparent 100%)',
                    opacity: toolbarVisible ? 1 : 0,
                    transition: 'opacity 0.3s',
                    pointerEvents: toolbarVisible ? 'auto' : 'none',
                }}>
                    {toolbarContent}
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
            onMouseEnter={showToolbar}
            onMouseMove={showToolbar}
            onMouseLeave={hideToolbar}
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
                overflow: 'hidden',
                border: locked ? 'none' : '1px solid rgba(255,255,255,0.1)',
                userSelect: 'none',
                pointerEvents: locked ? 'none' : 'auto',
                transition: 'background 0.2s, box-shadow 0.2s, border 0.2s',
            }}
        >
            {/* Canvas 区域 - 始终占满整个容器 */}
            <div ref={canvasContainerRef} style={{ width: '100%', height: '100%', position: 'relative' }}>
                <canvas
                    ref={canvasRef}
                    style={{ width: '100%', height: '100%', display: 'block' }}
                />
            </div>

            {/* 浮动工具栏 - 绝对定位在顶部, 不影响布局 */}
            {!locked && (
                <div
                    data-drag-handle="true"
                    onPointerDown={handlePointerDown}
                    style={{
                        position: 'absolute',
                        top: 0,
                        left: 0,
                        right: 0,
                        zIndex: 10,
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        padding: '4px 8px',
                        background: 'linear-gradient(to bottom, rgba(0,0,0,0.8) 0%, rgba(0,0,0,0.4) 70%, transparent 100%)',
                        cursor: 'grab',
                        opacity: toolbarVisible ? 1 : 0,
                        transition: 'opacity 0.3s',
                        pointerEvents: toolbarVisible ? 'auto' : 'none',
                    }}
                >
                    {toolbarContent}
                </div>
            )}

            {/* 锁定时: 悬浮的解锁按钮 */}
            {locked && (
                <button
                    onClick={toggleLock}
                    style={{
                        ...toolbarBtnStyle,
                        position: 'absolute',
                        top: 4,
                        right: 4,
                        pointerEvents: 'auto',
                        opacity: 0.15,
                        zIndex: 10,
                        width: 28,
                        height: 28,
                    }}
                    onMouseEnter={(e) => { (e.currentTarget as HTMLElement).style.opacity = '1'; }}
                    onMouseLeave={(e) => { (e.currentTarget as HTMLElement).style.opacity = '0.15'; }}
                    title="解锁"
                >
                    <FaLock size={12} />
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
                        zIndex: 11,
                    }}
                />
            )}
        </div>
    );
}

/** 工具栏按钮样式 */
const toolbarBtnStyle: React.CSSProperties = {
    background: 'rgba(255,255,255,0.15)',
    border: 'none',
    color: '#fff',
    width: 26,
    height: 26,
    borderRadius: 4,
    cursor: 'pointer',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    fontSize: 13,
    padding: 0,
    lineHeight: 1,
    transition: 'background 0.15s',
    flexShrink: 0,
};

/** 分隔符样式 */
const separatorStyle: React.CSSProperties = {
    color: 'rgba(255,255,255,0.25)',
    fontSize: 14,
    margin: '0 2px',
    userSelect: 'none',
    lineHeight: 1,
};
