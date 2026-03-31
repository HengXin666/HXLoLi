/**
 * ASS 歌词渲染悬浮窗组件 — SubtitlesOctopus Worker 模式
 *
 * 使用 HXLoLi 定制版 libass-wasm (SubtitlesOctopus) 渲染 ASS 歌词,
 * 原生支持 VSFilterMod 扩展 (\1img, \2img, \3img, \4img, \fsc, \fsvp),
 * 不再需要手动叠加图片。
 *
 * 保留:
 * - 悬浮窗拖拽移动、缩放、位置记忆
 * - ASS 预处理裁剪逻辑 (bounds timeline)
 * - 全屏模式
 * - 锁定(全透明)
 * - 字幕时间偏移
 *
 * 注意: SubtitlesOctopus 通过 <script> 标签加载到全局变量，
 * 不使用 require/import，因为 webpack 的 CJS/ESM interop 会破坏构造函数。
 *
 * \1img 图片通过 onReady 后 writeFile 写入 Worker FS，然后 setTrack 重新加载实现原生渲染。
 */
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import type { MusicTrackDetail } from '@site/src/config/musicData';
import { toMusicCdnUrl, toMusicLocalUrl, reqMusicByCDN } from '@site/src/utils/cdn/linkJsDelivr';
import { loadTrackDetail, shouldUseLocal } from '@site/src/utils/music/musicDataLoader';
import { useMusicStore } from '@site/src/utils/music/musicStore';
import React, { useCallback, useEffect, useRef, useState } from 'react';
import {
  FaAlignCenter,
  FaBackward,
  FaCrop,
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

// ========== SubtitlesOctopus 类型声明 (Worker 模式) ==========
interface SubtitlesOctopusInstance {
  canvas: HTMLCanvasElement;
  setCurrentTime(time: number): void;
  setTrack(content: string): void;
  setTrackByUrl(url: string): void;
  freeTrack(): void;
  resize(width?: number, height?: number): void;
  setChannelImage(channel: number, imageData: Uint8Array | null): void;
  writeFile(path: string, data: Uint8Array): void;
  dispose(): void;
  setIsPaused(isPaused: boolean, currentTime: number): void;
  worker: Worker;
}

// ========== 常量 & 工具 ==========

/** 全局重置位置的事件名 */
const RESET_POSITION_EVENT = 'hxloli-lyrics-reset-position';

const POSITION_KEY = 'hxloli-lyrics-position';
const SIZE_KEY = 'hxloli-lyrics-size';
const LOCK_KEY = 'hxloli-lyrics-locked';
const SUBTITLE_OFFSET_KEY = 'hxloli-lyrics-subtitle-offset';
const PREPROCESS_ASS_KEY = 'hxloli-lyrics-preprocess-ass';

const DEFAULT_SIZE = { w: 500, h: 350 };

function defaultPosition(): { x: number; y: number } {
  if (typeof window === 'undefined') return { x: 100, y: 100 };
  const x = Math.max(0, Math.round((window.innerWidth - DEFAULT_SIZE.w) / 2));
  const y = Math.max(0, Math.round((window.innerHeight - DEFAULT_SIZE.h) / 2));
  return { x, y };
}

function loadPosition(): { x: number; y: number } {
  try { const raw = localStorage.getItem(POSITION_KEY); if (raw) return JSON.parse(raw); } catch {}
  return defaultPosition();
}
function savePosition(pos: { x: number; y: number }) {
  try { localStorage.setItem(POSITION_KEY, JSON.stringify(pos)); } catch {}
}
function loadSize(): { w: number; h: number } {
  try { const raw = localStorage.getItem(SIZE_KEY); if (raw) return JSON.parse(raw); } catch {}
  return { ...DEFAULT_SIZE };
}
function saveSize(size: { w: number; h: number }) {
  try { localStorage.setItem(SIZE_KEY, JSON.stringify(size)); } catch {}
}
function loadLocked(): boolean {
  try { return localStorage.getItem(LOCK_KEY) === 'true'; } catch { return false; }
}
function saveLocked(locked: boolean) {
  try { localStorage.setItem(LOCK_KEY, JSON.stringify(locked)); } catch {}
}
function loadSubtitleOffset(): number {
  try { const raw = localStorage.getItem(SUBTITLE_OFFSET_KEY); if (raw) return parseFloat(raw) || 0; } catch {}
  return 0;
}
function saveSubtitleOffset(offset: number) {
  try { localStorage.setItem(SUBTITLE_OFFSET_KEY, JSON.stringify(offset)); } catch {}
}
function loadPreprocessAss(): boolean {
  try { const raw = localStorage.getItem(PREPROCESS_ASS_KEY); if (raw !== null) return raw === 'true'; } catch {}
  return true;
}
function savePreprocessAss(enabled: boolean) {
  try { localStorage.setItem(PREPROCESS_ASS_KEY, JSON.stringify(enabled)); } catch {}
}

// ========== ASS 预处理裁剪逻辑 ==========
interface TwoBlockBounds {
  topYMin: number; topYMax: number;
  btmYMin: number; btmYMax: number;
  left: number; right: number;
  leftT?: number; rightT?: number;
  leftB?: number; rightB?: number;
}

interface BoundsTimelinePoint extends TwoBlockBounds {
  t: number;
}

function boundsHasTop(b: TwoBlockBounds): boolean { return b.topYMax > 0; }
function boundsHasBtm(b: TwoBlockBounds): boolean { return b.btmYMax > 0; }
function boundsHasContent(b: TwoBlockBounds): boolean { return boundsHasTop(b) || boundsHasBtm(b) || b.right > 0; }

function interpolateBoundsAtTime(timeline: BoundsTimelinePoint[], time: number): TwoBlockBounds | null {
  const n = timeline.length;
  if (n === 0) return null;
  if (n === 1 || time <= timeline[0].t) {
    const p = timeline[0];
    return (p.topYMax > 0 || p.btmYMax > 0 || p.right > 0) ? p : null;
  }
  if (time >= timeline[n - 1].t) {
    const p = timeline[n - 1];
    return (p.topYMax > 0 || p.btmYMax > 0 || p.right > 0) ? p : null;
  }

  let lo = 0, hi = n - 1;
  while (lo < hi - 1) {
    const mid = (lo + hi) >> 1;
    if (timeline[mid].t <= time) lo = mid;
    else hi = mid;
  }

  const a = timeline[lo];
  const b = timeline[hi];
  const aHas = a.topYMax > 0 || a.btmYMax > 0 || a.right > 0;
  const bHas = b.topYMax > 0 || b.btmYMax > 0 || b.right > 0;

  if (!aHas && !bHas) return null;
  if (!aHas) return b;
  if (!bHas) return a;

  const dt = b.t - a.t;
  if (dt <= 0) return a;
  const ratio = (time - a.t) / dt;
  const lerp = (v1: number, v2: number) => Math.round(v1 + (v2 - v1) * ratio);
  const lerpOpt = (v1: number | undefined, v2: number | undefined) => {
    if (v1 == null || v2 == null) return undefined;
    return Math.round(v1 + (v2 - v1) * ratio);
  };
  return {
    topYMin: lerp(a.topYMin, b.topYMin),
    topYMax: lerp(a.topYMax, b.topYMax),
    btmYMin: lerp(a.btmYMin, b.btmYMin),
    btmYMax: lerp(a.btmYMax, b.btmYMax),
    left: lerp(a.left, b.left),
    right: lerp(a.right, b.right),
    leftT: lerpOpt((a as any).leftT, (b as any).leftT),
    rightT: lerpOpt((a as any).rightT, (b as any).rightT),
    leftB: lerpOpt((a as any).leftB, (b as any).leftB),
    rightB: lerpOpt((a as any).rightB, (b as any).rightB),
  };
}

const BOUNDS_PADDING_X = 30;
const BOUNDS_PADDING_Y = 10;

function cropAndDraw(srcCanvas: HTMLCanvasElement, dstCanvas: HTMLCanvasElement, bounds: TwoBlockBounds) {
  const hasTop = boundsHasTop(bounds);
  const hasBtm = boundsHasBtm(bounds);
  if (!hasTop && !hasBtm) return;

  const canvasW = srcCanvas.width;
  const canvasH = srcCanvas.height;

  const pad = (val: number, delta: number, min: number, max: number) =>
    Math.max(min, Math.min(max, val + delta));

  let topLeftX = (bounds.leftT != null && bounds.rightT != null && bounds.rightT > 0) ? bounds.leftT : bounds.left;
  let topRightX = (bounds.leftT != null && bounds.rightT != null && bounds.rightT > 0) ? bounds.rightT : bounds.right;
  let btmLeftX = (bounds.leftB != null && bounds.rightB != null && bounds.rightB > 0) ? bounds.leftB : bounds.left;
  let btmRightX = (bounds.leftB != null && bounds.rightB != null && bounds.rightB > 0) ? bounds.rightB : bounds.right;

  topLeftX = pad(topLeftX, -BOUNDS_PADDING_X, 0, canvasW);
  topRightX = pad(topRightX, BOUNDS_PADDING_X, 0, canvasW);
  btmLeftX = pad(btmLeftX, -BOUNDS_PADDING_X, 0, canvasW);
  btmRightX = pad(btmRightX, BOUNDS_PADDING_X, 0, canvasW);
  const topYMin = hasTop ? pad(bounds.topYMin, -BOUNDS_PADDING_Y, 0, canvasH) : 0;
  const topYMax = hasTop ? pad(bounds.topYMax, BOUNDS_PADDING_Y, 0, canvasH) : 0;
  const btmYMin = hasBtm ? pad(bounds.btmYMin, -BOUNDS_PADDING_Y, 0, canvasH) : 0;
  const btmYMax = hasBtm ? pad(bounds.btmYMax, BOUNDS_PADDING_Y, 0, canvasH) : 0;

  const topW = topRightX - topLeftX;
  const btmW = btmRightX - btmLeftX;
  const topH = hasTop ? (topYMax - topYMin) : 0;
  const btmH = hasBtm ? (btmYMax - btmYMin) : 0;
  const totalH = topH + btmH;
  if (totalH <= 0) return;

  const dstCtx = dstCanvas.getContext('2d');
  if (!dstCtx) return;
  dstCtx.clearRect(0, 0, dstCanvas.width, dstCanvas.height);

  const maxContentW = Math.max(topW, btmW, 1);
  const scaleX = dstCanvas.width / maxContentW;
  const scaleY = dstCanvas.height / totalH;
  const scale = Math.min(scaleX, scaleY, 1);

  const drawTotalH = totalH * scale;
  const offsetY = (dstCanvas.height - drawTotalH) / 2;

  if (hasTop && topH > 0 && topW > 0) {
    const drawTopW = topW * scale;
    const topOffsetX = (dstCanvas.width - drawTopW) / 2;
    dstCtx.drawImage(srcCanvas, topLeftX, topYMin, topW, topH, topOffsetX, offsetY, drawTopW, topH * scale);
  }

  if (hasBtm && btmH > 0 && btmW > 0) {
    const drawBtmW = btmW * scale;
    const btmOffsetX = (dstCanvas.width - drawBtmW) / 2;
    dstCtx.drawImage(srcCanvas, btmLeftX, btmYMin, btmW, btmH, btmOffsetX, offsetY + topH * scale, drawBtmW, btmH * scale);
  }
}

// ========== SubtitlesOctopus 模块加载 ==========

let jsoScriptLoaded = false;
let jsoScriptLoading = false;
let jsoScriptCallbacks: Array<() => void> = [];

function loadJSOScript(baseUrl: string, onReady: () => void) {
  if ((window as any).SubtitlesOctopus) {
    console.log('[JSO] SubtitlesOctopus 已存在于 window');
    jsoScriptLoaded = true;
    onReady();
    return;
  }
  if (jsoScriptLoaded) {
    console.log('[JSO] 脚本已加载但 window.SubtitlesOctopus 不存在!');
    onReady();
    return;
  }
  jsoScriptCallbacks.push(onReady);
  if (jsoScriptLoading) return;
  jsoScriptLoading = true;

  const scriptUrl = `${baseUrl}music/ass-worker/subtitles-octopus.js`;
  console.log('[JSO] 开始加载脚本:', scriptUrl);

  const script = document.createElement('script');
  script.src = scriptUrl;
  script.async = true;
  script.onload = () => {
    console.log('[JSO] subtitles-octopus.js 加载完成, window.SubtitlesOctopus:', typeof (window as any).SubtitlesOctopus);

    if (typeof (window as any).SubtitlesOctopus !== 'function') {
      console.error('[JSO] 脚本加载后 window.SubtitlesOctopus 不是函数! 尝试 fetch+eval 方式...');
      fetch(scriptUrl)
        .then(r => r.text())
        .then(code => {
          const fakeModule: any = { exports: {} };
          const wrapper = new Function('module', 'exports', code);
          wrapper(fakeModule, fakeModule.exports);
          if (typeof fakeModule.exports === 'function') {
            (window as any).SubtitlesOctopus = fakeModule.exports;
            console.log('[JSO] fetch+eval 方式成功!');
          } else {
            console.error('[JSO] fetch+eval 也失败了:', typeof fakeModule.exports);
          }
          finishLoading();
        })
        .catch(err => {
          console.error('[JSO] fetch+eval 失败:', err);
          finishLoading();
        });
    } else {
      finishLoading();
    }
  };
  script.onerror = () => {
    console.error('[JSO] subtitles-octopus.js 加载失败');
    jsoScriptLoading = false;
  };

  function finishLoading() {
    jsoScriptLoaded = true;
    jsoScriptLoading = false;
    const cbs = jsoScriptCallbacks.slice();
    jsoScriptCallbacks = [];
    cbs.forEach(cb => cb());
  }

  document.head.appendChild(script);
}

/**
 * 创建 SubtitlesOctopus 实例 (Worker 模式)
 */
function createJSOInstance(options: {
  canvas: HTMLCanvasElement;
  workerUrl: string;
  legacyWorkerUrl?: string;
  subContent?: string;
  subUrl?: string;
  fonts?: string[];
  fallbackFont?: string;
  availableFonts?: Record<string, string>;
  onReady?: () => void;
  onError?: (error: any) => void;
  debug?: boolean;
}): SubtitlesOctopusInstance {
  const SubOctopusCtor = (window as any).SubtitlesOctopus;
  if (!SubOctopusCtor) {
    throw new Error('SubtitlesOctopus 全局变量未找到, subtitles-octopus.js 可能未加载成功');
  }

  const instance = new SubOctopusCtor({
    canvas: options.canvas,
    workerUrl: options.workerUrl,
    legacyWorkerUrl: options.legacyWorkerUrl,
    subContent: options.subContent || undefined,
    subUrl: options.subUrl || undefined,
    fonts: options.fonts || [],
    fallbackFont: options.fallbackFont || 'default.woff2',
    availableFonts: options.availableFonts || {},
    onReady: options.onReady,
    onError: options.onError,
    debug: options.debug ?? true,
    renderMode: 'wasm-blend',
    targetFps: 24,
    prescaleFactor: 0.8,
    prescaleHeightLimit: 1080,
    maxRenderHeight: 720,
  }) as SubtitlesOctopusInstance;

  return instance;
}

// ========== 全局偏移 ==========
let globalSubtitleOffset = loadSubtitleOffset();

// ========== 组件 ==========
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
  // 主渲染 canvas (SubtitlesOctopus Worker 渲染到此)
  const renderCanvasRef = useRef<HTMLCanvasElement>(null);
  // 显示用的 canvas (裁剪后绘制到此, 预处理模式下使用)
  const displayCanvasRef = useRef<HTMLCanvasElement>(null);

  const rendererRef = useRef<SubtitlesOctopusInstance | null>(null);
  const [position, setPosition] = useState(loadPosition);
  const [size, setSize] = useState(loadSize);
  const [locked, setLocked] = useState(loadLocked);
  const [subtitleOffset, setSubtitleOffset] = useState(loadSubtitleOffset);
  const [toolbarVisible, setToolbarVisible] = useState(false);
  const [, forceUpdate] = useState(0);
  const [rebuildToken, setRebuildToken] = useState(0);
  const [preprocessAss, setPreprocessAss] = useState(loadPreprocessAss);
  const [initError, setInitError] = useState<string | null>(null);
  const cachedBoundsRef = useRef<TwoBlockBounds | null>(null);
  const timelineRef = useRef<BoundsTimelinePoint[] | null>(null);
  const isDragging = useRef(false);
  const isResizing = useRef(false);
  const dragOffset = useRef({ x: 0, y: 0 });
  const rafIdRef = useRef<number | null>(null);
  const toolbarTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const resetPosition = useCallback(() => {
    const pos = defaultPosition();
    const sz = { ...DEFAULT_SIZE };
    setPosition(pos); setSize(sz);
    savePosition(pos); saveSize(sz);
  }, []);

  const toggleLock = useCallback(() => {
    setLocked((prev) => { const next = !prev; saveLocked(next); return next; });
  }, []);

  const subtitleSlower = useCallback(() => {
    setSubtitleOffset((prev) => { const next = +(prev - 0.5).toFixed(1); globalSubtitleOffset = next; saveSubtitleOffset(next); return next; });
  }, []);

  const subtitleFaster = useCallback(() => {
    setSubtitleOffset((prev) => { const next = +(prev + 0.5).toFixed(1); globalSubtitleOffset = next; saveSubtitleOffset(next); return next; });
  }, []);

  const resetSubtitleOffset = useCallback(() => {
    setSubtitleOffset(0); globalSubtitleOffset = 0; saveSubtitleOffset(0);
  }, []);

  const seekToStart = useCallback(() => { seek(0); }, [seek]);

  const togglePreprocessAss = useCallback(() => {
    setPreprocessAss((prev) => { const next = !prev; savePreprocessAss(next); setRebuildToken((n) => n + 1); return next; });
  }, []);

  const centerHorizontally = useCallback(() => {
    if (typeof window === 'undefined') return;
    const x = Math.max(0, Math.round((window.innerWidth - size.w) / 2));
    const newPos = { x, y: position.y };
    setPosition(newPos); savePosition(newPos);
  }, [size.w, position.y]);

  const showToolbar = useCallback(() => {
    setToolbarVisible(true);
    if (toolbarTimerRef.current) clearTimeout(toolbarTimerRef.current);
    toolbarTimerRef.current = setTimeout(() => { if (!isDragging.current) setToolbarVisible(false); }, 3000);
  }, []);

  const hideToolbar = useCallback(() => {
    if (toolbarTimerRef.current) clearTimeout(toolbarTimerRef.current);
    toolbarTimerRef.current = setTimeout(() => { if (!isDragging.current) setToolbarVisible(false); }, 500);
  }, []);

  // 监听外部触发的重置位置事件
  useEffect(() => {
    const handler = () => {
      const pos = defaultPosition();
      const sz = { ...DEFAULT_SIZE };
      setPosition(pos); setSize(sz);
      savePosition(pos); saveSize(sz);
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
          case POSITION_KEY: setPosition(JSON.parse(e.newValue)); break;
          case SIZE_KEY: setSize(JSON.parse(e.newValue)); break;
          case LOCK_KEY: setLocked(e.newValue === 'true'); break;
          case SUBTITLE_OFFSET_KEY: {
            const val = parseFloat(e.newValue) || 0;
            setSubtitleOffset(val); globalSubtitleOffset = val; break;
          }
          case PREPROCESS_ASS_KEY:
            setPreprocessAss(e.newValue === 'true');
            setRebuildToken((n) => n + 1); break;
        }
      } catch {}
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

  // ========== RAF 渲染循环: 驱动 SubtitlesOctopus 的 setCurrentTime 并处理裁剪 ==========
  const shouldRender = showLyrics && pageVisible;

  useEffect(() => {
    if (!shouldRender) return;

    let lastTime = -1;
    let debugCounter = 0;

    const tick = () => {
      const jso = rendererRef.current;
      if (!jso) {
        if (debugCounter % 300 === 0) {
          console.log('[JSO] RAF: 实例为 null, 等待初始化...');
        }
        debugCounter++;
        rafIdRef.current = requestAnimationFrame(tick);
        return;
      }
      const ct = useMusicStore.getState().getInterpolatedTime() + globalSubtitleOffset;
      const adjustedCt = Math.max(0, ct);

      const delta = adjustedCt - lastTime;
      if (delta > 0.016 || delta < -0.5 || lastTime < 0) {
          // SubtitlesOctopus Worker 自动渲染到 canvas，只需更新时间
          jso.setCurrentTime(adjustedCt);

          // 预处理模式: 从 renderCanvas 裁剪到 displayCanvas
          if (preprocessAss && renderCanvasRef.current && displayCanvasRef.current) {
            const renderCanvas = renderCanvasRef.current;
            const tl = timelineRef.current;
            const currentBounds = tl
              ? interpolateBoundsAtTime(tl, adjustedCt)
              : cachedBoundsRef.current;

            if (currentBounds && boundsHasContent(currentBounds)) {
              cropAndDraw(renderCanvas, displayCanvasRef.current, currentBounds);
            } else if (tl) {
              // 空区间: 缩放复制
              const dCtx = displayCanvasRef.current.getContext('2d');
              if (dCtx) {
                dCtx.clearRect(0, 0, displayCanvasRef.current.width, displayCanvasRef.current.height);
                const scale = Math.min(
                  displayCanvasRef.current.width / renderCanvas.width,
                  displayCanvasRef.current.height / renderCanvas.height,
                  1
                );
                const dw = renderCanvas.width * scale;
                const dh = renderCanvas.height * scale;
                const dx = (displayCanvasRef.current.width - dw) / 2;
                const dy = (displayCanvasRef.current.height - dh) / 2;
                dCtx.drawImage(renderCanvas, 0, 0, renderCanvas.width, renderCanvas.height, dx, dy, dw, dh);
              }
            }
          }

          lastTime = adjustedCt;
        }
      debugCounter++;
      rafIdRef.current = requestAnimationFrame(tick);
    };

    rafIdRef.current = requestAnimationFrame(tick);
    return () => {
      if (rafIdRef.current !== null) {
        cancelAnimationFrame(rafIdRef.current);
        rafIdRef.current = null;
      }
    };
  }, [shouldRender, preprocessAss]);

  // ========== 初始化 / 切换曲目时重建 SubtitlesOctopus 实例 ==========
  useEffect(() => {
    if (!showLyrics || !currentTrack?.assUrl) {
      if (rendererRef.current) {
        try { rendererRef.current.dispose(); } catch {}
        rendererRef.current = null;
      }
      return;
    }

    let disposed = false;
    setInitError(null);

    const initRenderer = async () => {
      try {
        // 销毁旧实例
        if (rendererRef.current) {
          try { rendererRef.current.dispose(); } catch {}
          rendererRef.current = null;
        }

        // 计算渲染尺寸
        let pixelW: number, pixelH: number;
        if (preprocessAss) {
          pixelW = 1920;
          pixelH = 1080;
        } else if (lyricsFullscreen) {
          pixelW = window.innerWidth;
          pixelH = window.innerHeight;
        } else {
          pixelW = Math.round(size.w);
          pixelH = Math.round(size.h);
        }

        if (pixelW <= 0 || pixelH <= 0) {
          if (!disposed) setTimeout(initRenderer, 200);
          return;
        }

        console.log(`[JSO] 初始化 SubtitlesOctopus: ${pixelW}x${pixelH}, 预处理: ${preprocessAss}, 悬浮窗 size: ${size.w}x${size.h}`);

        // 设置 render canvas 尺寸
        const renderCanvas = renderCanvasRef.current;
        if (renderCanvas) {
          renderCanvas.width = pixelW;
          renderCanvas.height = pixelH;
        }

        // 设置 display canvas 尺寸 (预处理模式)
        if (preprocessAss && displayCanvasRef.current) {
          if (lyricsFullscreen) {
            displayCanvasRef.current.width = window.innerWidth;
            displayCanvasRef.current.height = window.innerHeight;
          } else {
            displayCanvasRef.current.width = Math.round(size.w);
            displayCanvasRef.current.height = Math.round(size.h);
          }
        }

        // 按需加载歌曲详细配置
        console.log(`[JSO] 加载歌曲详细配置: ${currentTrack.id}`);
        let trackDetail: MusicTrackDetail | null = null;
        try {
          trackDetail = await loadTrackDetail(currentTrack.id);
        } catch (detailErr) {
          console.warn('[JSO] 加载详细配置失败, 继续使用基础模式:', detailErr);
        }
        if (disposed) return;

        // 使用 shouldUseLocal() 正确检测本地服务器是否可用，
        // 而非仅凭 isLocalDev() 判断（serve.py 未运行时需 fallback 到 CDN，
        // 否则 Worker 加载字体 URL 失败会导致整个渲染引擎卡死）
        const useLocal = await shouldUseLocal();
        const toFontUrl = useLocal ? toMusicLocalUrl : toMusicCdnUrl;
        const cjkFallbackUrl = toFontUrl('/static/music/fonts/NotoSansSC-Regular.ttf');

        const safeUrl = (url: string): string => {
          if (/%[0-9A-Fa-f]{2}/.test(url)) return url;
          return encodeURI(url);
        };

        // 构建 availableFonts 映射 (字体名 -> URL)
        const availableFonts: Record<string, string> = {};
        if (trackDetail?.assFontMap) {
          for (const [fontName, fontUrl] of Object.entries(trackDetail.assFontMap)) {
            availableFonts[fontName] = safeUrl(fontUrl);
          }
        }

        // 收集所有需要预加载的字体文件 URL
        const fontUrls: string[] = [];
        if (trackDetail?.fonts) {
          for (const fontUrl of trackDetail.fonts) {
            fontUrls.push(safeUrl(fontUrl));
          }
        }

        // 加载 ASS 字幕内容 — 通过 ForgeEngine 切片并行下载
        console.log('[JSO] 正在加载 ASS 文件...');
        let subContent: string;
        try {
          if (useLocal) {
            // 本地开发: 直接 fetch
            const assUrlRaw = currentTrack.assUrl!;
            const origin = typeof window !== 'undefined' ? window.location.origin : '';
            const subFullUrl = safeUrl(
              assUrlRaw.startsWith('http') ? assUrlRaw : `${origin}${assUrlRaw}`
            );
            console.log('[JSO] 本地 fetch ASS:', subFullUrl);
            const resp = await fetch(subFullUrl);
            if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
            subContent = await resp.text();
          } else {
            // 生产: reqMusicByCDN 自动检测切片 (info.yaml → 并行下载 → 拼接)
            const relativePath = currentTrack.assRelativePath ?? currentTrack.assUrl!;
            console.log('[JSO] reqMusicByCDN ASS:', relativePath);
            const result = await reqMusicByCDN(relativePath);
            subContent = await result.blob.text();
            console.log(`[JSO] ASS 加载完成: ${subContent.length} 字符, 切片模式: ${result.usedSplitMode}, 耗时: ${result.totalTime.toFixed(0)}ms`);
          }
        } catch (fetchErr) {
          console.error('[JSO] ASS 文件加载失败:', fetchErr);
          setInitError(`加载 ASS 文件失败: ${fetchErr}`);
          return;
        }
        if (disposed) return;

        // 创建 SubtitlesOctopus 实例 (Worker 模式)
        if (!renderCanvas) {
          console.error('[JSO] renderCanvas 未就绪');
          return;
        }

        const workerUrl = safeUrl(`${origin}${baseUrl}music/ass-worker/subtitles-octopus-worker.js`);
        const legacyWorkerUrl = safeUrl(`${origin}${baseUrl}music/ass-worker/subtitles-octopus-worker-legacy.js`);

        const jso = createJSOInstance({
          canvas: renderCanvas,
          workerUrl: workerUrl,
          legacyWorkerUrl: legacyWorkerUrl,
          subContent: subContent,
          fonts: fontUrls,
          fallbackFont: safeUrl(cjkFallbackUrl),
          availableFonts: availableFonts,
          debug: false,
          onReady: () => {
            console.log('[JSO] Worker 就绪 ✅');

            // 写入图片到 Worker FS
            let hasImages = false;
            if (trackDetail?.assImageData) {
              for (const [filePath, dataUri] of Object.entries(trackDetail.assImageData)) {
                try {
                  const base64 = dataUri.split(',')[1];
                  const binary = atob(base64);
                  const data = new Uint8Array(binary.length);
                  for (let i = 0; i < binary.length; i++) {
                    data[i] = binary.charCodeAt(i);
                  }
                  jso.writeFile(filePath, data);
                  hasImages = true;
                  console.log(`[JSO] 写入图片到 Worker FS: ${filePath} (${data.length} bytes)`);
                } catch (imgErr) {
                  console.error(`[JSO] 图片写入失败: ${filePath}`, imgErr);
                }
              }
            }

            // 如果写入了图片, 需要重新加载 track, 使 libass 重新解析 ASS 时能找到 \1img 引用的图片
            if (hasImages && subContent) {
              console.log('[JSO] 图片已写入, 重新加载 track 以使 \\1img 生效...');
              jso.setTrack(subContent);
            }

            // 确保 Worker 知道正确的 canvas 尺寸
            jso.resize(pixelW, pixelH);

            // 设置预处理 bounds
            if (preprocessAss) {
              if (trackDetail?.assBoundsTimeline && trackDetail.assBoundsTimeline.length > 0) {
                timelineRef.current = trackDetail.assBoundsTimeline as BoundsTimelinePoint[];
                cachedBoundsRef.current = null;
                console.log(`[JSO] 使用时间轴 bounds: ${trackDetail.assBoundsTimeline.length} 个关键点`);
              } else if (trackDetail?.assBounds) {
                const b = trackDetail.assBounds;
                const bounds: TwoBlockBounds = {
                  topYMin: b.topYMin, topYMax: b.topYMax,
                  btmYMin: b.btmYMin, btmYMax: b.btmYMax,
                  left: b.left, right: b.right,
                };
                if (boundsHasContent(bounds)) {
                  cachedBoundsRef.current = bounds;
                  console.log('[JSO] 使用全局 assBounds:', bounds);
                } else {
                  cachedBoundsRef.current = null;
                }
                timelineRef.current = null;
              }
            }

            // 立即渲染当前帧
            const ct = useMusicStore.getState().getInterpolatedTime() + globalSubtitleOffset;
            const currentTime = Math.max(0, ct);
            jso.setCurrentTime(currentTime);
            console.log(`[JSO] 开始渲染: time=${currentTime.toFixed(3)}, size=${pixelW}x${pixelH}`);
          },
          onError: (error: any) => {
            console.error('[JSO] Worker 错误:', error);
            setInitError(`SubtitlesOctopus Worker 错误: ${error}`);
          },
        });

        if (disposed) {
          try { jso.dispose(); } catch {}
          return;
        }

        rendererRef.current = jso;
        console.log('[JSO] SubtitlesOctopus 实例创建完成 ✅');

      } catch (err) {
        console.error('[JSO] 初始化失败:', err);
        setInitError(`SubtitlesOctopus 初始化失败: ${err}`);
      }
    };

    loadJSOScript(baseUrl, () => {
      if (!disposed) initRenderer();
    });

    return () => {
      disposed = true;
      cachedBoundsRef.current = null;
      timelineRef.current = null;
      if (rendererRef.current) {
        try { rendererRef.current.dispose(); } catch {}
        rendererRef.current = null;
      }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [showLyrics, currentTrack?.assUrl, baseUrl, lyricsFullscreen, rebuildToken, preprocessAss,
        preprocessAss ? 0 : size.w, preprocessAss ? 0 : size.h]);

  // 全屏 resize 监听
  useEffect(() => {
    if (!showLyrics) return;
    const handleResize = () => {
      if (useMusicStore.getState().lyricsFullscreen) setRebuildToken((n) => n + 1);
    };
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [showLyrics]);

  const prevFullscreen = useRef(lyricsFullscreen);
  useEffect(() => {
    if (prevFullscreen.current !== lyricsFullscreen) {
      prevFullscreen.current = lyricsFullscreen;
      setRebuildToken((n) => n + 1);
    }
  }, [lyricsFullscreen]);

  // 拖拽逻辑
  const handlePointerDown = useCallback((e: React.PointerEvent) => {
    if (locked) return;
    if ((e.target as HTMLElement).dataset.dragHandle !== 'true') return;
    isDragging.current = true;
    dragOffset.current = { x: e.clientX - position.x, y: e.clientY - position.y };
    (e.target as HTMLElement).setPointerCapture(e.pointerId);
  }, [position, locked]);

  const handlePointerMove = useCallback((e: React.PointerEvent) => {
    if (isDragging.current) setPosition({ x: e.clientX - dragOffset.current.x, y: e.clientY - dragOffset.current.y });
  }, []);

  const handlePointerUp = useCallback(() => {
    if (isDragging.current) { isDragging.current = false; savePosition(position); }
  }, [position]);

  // 缩放逻辑
  const handleResizePointerDown = useCallback((e: React.PointerEvent) => {
    if (locked) return;
    e.stopPropagation(); e.preventDefault();
    isResizing.current = true;
    (e.target as HTMLElement).setPointerCapture(e.pointerId);
  }, [locked]);

  const handleResizePointerMove = useCallback((e: React.PointerEvent) => {
    if (isResizing.current && containerRef.current) {
      const rect = containerRef.current.getBoundingClientRect();
      setSize({ w: Math.max(300, e.clientX - rect.left), h: Math.max(200, e.clientY - rect.top) });
    }
  }, []);

  const handleResizePointerUp = useCallback(() => {
    if (isResizing.current) { isResizing.current = false; saveSize(size); setRebuildToken((n) => n + 1); }
  }, [size]);

  useEffect(() => {
    return () => { if (toolbarTimerRef.current) clearTimeout(toolbarTimerRef.current); };
  }, []);

  if (!showLyrics || !currentTrack?.assUrl) return null;

  // 工具栏
  const toolbarContent = (
    <div style={{ display: 'flex', alignItems: 'center', gap: 2, flexWrap: 'wrap', justifyContent: 'center' }}>
      {/* JSO-WASM 标识 */}
      <span style={{ fontSize: 9, color: '#00ff88', marginRight: 4, fontWeight: 700, letterSpacing: 0.5 }}>
        JSO-WASM
      </span>
      <button onClick={prev} style={toolbarBtnStyle} title="上一首"><FaStepBackward size={11} /></button>
      <button onClick={toggle} style={toolbarBtnStyle} title={isPlaying ? '暂停' : '播放'}>
        {isPlaying ? <FaPause size={11} /> : <FaPlay size={11} />}
      </button>
      <button onClick={next} style={toolbarBtnStyle} title="下一首"><FaStepForward size={11} /></button>
      <button onClick={seekToStart} style={toolbarBtnStyle} title="回到开头"><FaFastBackward size={11} /></button>
      <span style={separatorStyle}>│</span>
      <button onClick={subtitleSlower} style={toolbarBtnStyle} title="字幕慢 0.5s"><FaBackward size={10} /></button>
      <button onClick={subtitleFaster} style={toolbarBtnStyle} title="字幕快 0.5s"><FaForward size={10} /></button>
      {subtitleOffset !== 0 && (
        <span style={{ color: 'rgba(255,255,255,0.6)', fontSize: 10, margin: '0 2px', whiteSpace: 'nowrap' }}>
          {subtitleOffset > 0 ? '+' : ''}{subtitleOffset.toFixed(1)}s
        </span>
      )}
      <button onClick={resetSubtitleOffset} style={toolbarBtnStyle} title="重置时间偏移"><FaUndo size={10} /></button>
      <button onClick={toggleLock} style={{ ...toolbarBtnStyle, background: locked ? 'rgba(255, 180, 0, 0.4)' : toolbarBtnStyle.background }} title={locked ? '解锁' : '锁定'}>
        {locked ? <FaLock size={11} /> : <FaLockOpen size={11} />}
      </button>
      <span style={separatorStyle}>│</span>
      <button onClick={centerHorizontally} style={toolbarBtnStyle} title="水平居中"><FaAlignCenter size={11} /></button>
      <button onClick={togglePreprocessAss} style={{ ...toolbarBtnStyle, background: preprocessAss ? 'rgba(0, 200, 100, 0.4)' : toolbarBtnStyle.background }} title={preprocessAss ? '关闭裁剪' : '开启裁剪'}>
        <FaCrop size={11} />
      </button>
      <button onClick={() => toggleLyricsFullscreen()} style={toolbarBtnStyle} title="全屏"><FaExpand size={11} /></button>
      <button onClick={toggleLyrics} style={toolbarBtnStyle} title="关闭"><FaTimes size={11} /></button>
    </div>
  );

  // 错误提示
  const errorOverlay = initError ? (
    <div style={{
      position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center',
      background: 'rgba(0,0,0,0.8)', color: '#ff6b6b', fontSize: 12, padding: 16, textAlign: 'center',
      zIndex: 5,
    }}>
      {initError}
    </div>
  ) : null;

  // 全屏模式
  if (lyricsFullscreen) {
    return (
      <div ref={containerRef} style={{ position: 'fixed', inset: 0, zIndex: 10000, background: '#000', display: 'flex', alignItems: 'center', justifyContent: 'center' }}
        onMouseEnter={showToolbar} onMouseMove={showToolbar} onMouseLeave={hideToolbar}>
        <div style={{ position: 'absolute', top: 0, left: 0, right: 0, zIndex: 10001, display: 'flex', justifyContent: 'center', padding: '8px 12px', background: 'linear-gradient(to bottom, rgba(0,0,0,0.7) 0%, transparent 100%)', opacity: toolbarVisible ? 1 : 0, transition: 'opacity 0.3s', pointerEvents: toolbarVisible ? 'auto' : 'none' }}>
          {toolbarContent}
        </div>
        <div style={{ width: '100%', height: '100%', position: 'relative' }}>
          <canvas ref={preprocessAss ? displayCanvasRef : renderCanvasRef} style={{ width: '100%', height: '100%', display: 'block' }} />
          {preprocessAss && <canvas ref={renderCanvasRef} style={{ display: 'none' }} />}
          {errorOverlay}
        </div>
      </div>
    );
  }

  // 悬浮窗模式
  return (
    <div ref={containerRef}
      onPointerMove={(e) => { handlePointerMove(e); handleResizePointerMove(e); }}
      onPointerUp={() => { handlePointerUp(); handleResizePointerUp(); }}
      onMouseEnter={showToolbar} onMouseMove={showToolbar} onMouseLeave={hideToolbar}
      style={{
        position: 'fixed', left: position.x, top: position.y, width: size.w, height: size.h,
        zIndex: 9999, background: locked ? 'transparent' : 'rgba(0, 0, 0, 0.85)',
        borderRadius: locked ? 0 : 8, boxShadow: locked ? 'none' : '0 4px 24px rgba(0,0,0,0.5)',
        overflow: 'hidden', border: locked ? 'none' : '1px solid rgba(255,255,255,0.1)',
        userSelect: 'none', pointerEvents: locked ? 'none' : 'auto',
        transition: 'background 0.2s, box-shadow 0.2s, border 0.2s',
      }}>
      <div style={{ width: '100%', height: '100%', position: 'relative' }}>
        <canvas ref={preprocessAss ? displayCanvasRef : renderCanvasRef} style={{ width: '100%', height: '100%', display: 'block' }} />
        {preprocessAss && <canvas ref={renderCanvasRef} style={{ display: 'none' }} />}
        {errorOverlay}
      </div>
      {!locked && (
        <div data-drag-handle="true" onPointerDown={handlePointerDown}
          style={{
            position: 'absolute', top: 0, left: 0, right: 0, zIndex: 10,
            display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '4px 8px',
            background: 'linear-gradient(to bottom, rgba(0,0,0,0.8) 0%, rgba(0,0,0,0.4) 70%, transparent 100%)',
            cursor: 'grab', opacity: toolbarVisible ? 1 : 0, transition: 'opacity 0.3s',
            pointerEvents: toolbarVisible ? 'auto' : 'none',
          }}>
          {toolbarContent}
        </div>
      )}
      {locked && (
        <button onClick={toggleLock}
          style={{ ...toolbarBtnStyle, position: 'absolute', top: 4, right: 4, pointerEvents: 'auto', opacity: 0.15, zIndex: 10, width: 28, height: 28 }}
          onMouseEnter={(e) => { (e.currentTarget as HTMLElement).style.opacity = '1'; }}
          onMouseLeave={(e) => { (e.currentTarget as HTMLElement).style.opacity = '0.15'; }}
          title="解锁">
          <FaLock size={12} />
        </button>
      )}
      {!locked && (
        <div onPointerDown={handleResizePointerDown}
          style={{ position: 'absolute', right: 0, bottom: 0, width: 16, height: 16, cursor: 'nwse-resize', background: 'linear-gradient(135deg, transparent 50%, rgba(255,255,255,0.3) 50%)', zIndex: 11 }} />
      )}
    </div>
  );
}

const toolbarBtnStyle: React.CSSProperties = {
  background: 'rgba(255,255,255,0.15)', border: 'none', color: '#fff',
  width: 26, height: 26, borderRadius: 4, cursor: 'pointer',
  display: 'flex', alignItems: 'center', justifyContent: 'center',
  fontSize: 13, padding: 0, lineHeight: 1, transition: 'background 0.15s', flexShrink: 0,
};

const separatorStyle: React.CSSProperties = {
  color: 'rgba(255,255,255,0.25)', fontSize: 14, margin: '0 2px', userSelect: 'none', lineHeight: 1,
};
