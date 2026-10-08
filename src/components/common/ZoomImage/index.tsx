import React, {
    useCallback, useEffect, useLayoutEffect, useRef, useState,
} from 'react';
import { createPortal } from 'react-dom';
import styles from './ZoomImage.module.css';

/**
 * 点开看大图的查看器.
 *
 * 坐标系 (整份实现只有这一套, 动手前先确认没有引入第二套):
 *
 *   fit 系 = 图片按舞台 "contain" 放下后的完整尺寸 (fitW x fitH), 由 JS 量出.
 *   显示尺寸 = fit * f;  f = 1 表示"整张刚好装下", 放大就是 f > 1.
 *   p = 图片**中心**相对舞台中心的偏移, 单位是**屏幕像素** (不是 fit 系).
 *
 * 于是渲染只有一条公式, 且图片中心的屏幕位置恰好是 舞台中心 + p:
 *
 *   transform: translate(p.x, p.y) scale(f)
 *
 * 下面三条约束都从这一套坐标里长出来, 动它们之前先把这一节读完  它们各自对应一个踩过的坑:
 *
 * 1. p 一旦与 fit 系混用, 锚点和拖拽都会按 f 缩水: 实测拖 100px 只走 42px、滚轮锚点
 *    u 从 0.25 漂到 0.37. 所以本文件里凡是要加减的量 (锚点、拖拽增量、边界) 一律是屏幕像素,
 *    只有跟图片尺寸相乘比较时才换成 fit 系.
 * 2. 居中**不能**用 `translate(-50%, -50%)`. 那个百分比按未缩放的元素尺寸取, scale() 之后
 *    它不再把中心放回原位. 改用 JS 写的负 margin 居中, transform 里就只剩 translate(p) scale(f).
 * 3. 中心锚定让拖拽 / 缩放 / 滚轮锚点共用同一个不变量: 只要图片中心的屏幕位置没变,
 *    图片上的那个点就不会动. 换成左上角做锚点, 这三处各要各的换算.
 *
 * 变换内联写在 style 上而不是回写 state: 指针移动是高频事件, 走 state 会每帧重渲染整棵树.
 *
 * 与坐标系无关、但同样踩过的三条:
 *
 * 1. **wheel 必须原生监听.** React 17 起把 wheel 注册为 passive, onWheel 里的
 *    preventDefault() 会被浏览器静默忽略  表现为"在查看器里滚轮同时滚动了背后的文章".
 * 2. **样式全部收在本目录的 module 里.** 查看器 Portal 到 document.body, 宿主组件的
 *    CSS Module 类名在 Portal 子节点里解析成 undefined, 会塌成没有样式的裸元素.
 * 3. **点没点中图片要看 pointerdown 的 target.** 舞台调了 setPointerCapture 之后,
 *    合成 click 的 target 恒为舞台  拿 click.target 判断会让单击直接关掉查看器.
 */

export type ZoomImageProps = React.ImgHTMLAttributes<HTMLImageElement> & {
    /** 可选: 全屏时的自定义类名 */
    overlayClassName?: string;
    /** 可选: 全屏时图片的自定义样式 */
    fullScreenImageStyle?: React.CSSProperties;
};

type View = {
    /** 缩放倍数, 1 = 整张装下 */
    f: number;
    /** 图片中心相对舞台中心的偏移, 单位屏幕像素 */
    x: number;
    y: number;
};

type Fit = { w: number; h: number };

/** 放大上限按"一个图像像素最多铺到几个设备像素"算, 与图片原始分辨率挂钩 */
const MAX_DEVICE_PX_PER_IMAGE_PX = 4;
const MAX_F = 64;
/** 低于这个倍数就算回到"整张装下", 直接吸附归位  否则用户永远退不回严格的 f = 1 */
const SNAP_F = 1.02;
/** 双击的两档: 1.8x, 再双击 3.5x, 第三下复位 */
const DOUBLE_ZOOM_STEPS = [1.8, 3.5];
/** 关闭动画时长, 与 ZoomImage.module.css 里 .overlay 的 transition 对齐 */
const CLOSE_MS = 220;
/** 图片上单击的关闭延迟: 留出一个双击窗口, 否则"双击放大"的第一下就先把它关了 */
const SINGLE_CLICK_MS = 240;
/** 超过这个位移就不算"点一下" */
const MOVE_TOLERANCE = 4;

/** 全站只提示一次操作方式. 每张图都弹一遍那条浮条是纯噪音 */
let hintShown = false;

const clamp = (v: number, min: number, max: number) => Math.min(max, Math.max(min, v));

/** 上限 = min(按设备像素密度算出的天花板, 硬上限); 小图也必须能放到看得清 */
function maxF (fit: Fit | null): number {
    if (!fit || typeof window === 'undefined') return 4;
    const dpr = window.devicePixelRatio || 1;
    const byPixels = (window.innerWidth * dpr * MAX_DEVICE_PX_PER_IMAGE_PX) / fit.w;
    return clamp(byPixels, 3, MAX_F);
}

export default function ZoomImage ({
    style,
    className,
    overlayClassName,
    fullScreenImageStyle,
    onClick,
    ...props
}: ZoomImageProps): React.ReactElement {
    const [isOpen, setIsOpen] = useState(false);
    const [isAnimating, setIsAnimating] = useState(false);
    const [ready, setReady] = useState(false);
    const [scalePct, setScalePct] = useState(100);
    const [dragging, setDragging] = useState(false);
    const [broken, setBroken] = useState(false);

    const stageRef = useRef<HTMLDivElement>(null);
    const imgRef = useRef<HTMLImageElement>(null);
    const fitRef = useRef<Fit | null>(null);
    const measureRef = useRef<() => void>(() => {});
    /** 权威视角: 高频手势直接改它 + 手写 style, 只有倍率的变化才回写 state (工具条读数) */
    const view = useRef<View>({ f: 1, x: 0, y: 0 });
    /** 每次手势都从"按下那一刻的视角"重算, 而不是累加增量  累加会把丢事件和浮点误差一并算进去 */
    const gestureStart = useRef<View>({ f: 1, x: 0, y: 0 });
    const gesture = useRef<{
        kind: 'pan' | 'pinch' | null;
        pointerId: number;
        sx: number;
        sy: number;
        dist: number;
        pointers: Map<number, { x: number; y: number }>;
    }>({ kind: null, pointerId: -1, sx: 0, sy: 0, dist: 0, pointers: new Map() });
    /** 上一次手势是否构成"拖动". click 在 pointerup 之后到达, 只能这样把位移传给它 */
    const lastDragMoved = useRef(false);
    /**
     * 本次手势是否按在图片上.
     *
     * 只能取 pointerdown 的 target: 舞台调了 setPointerCapture, pointerup 会被重定向到舞台,
     * 于是合成出来的 click 的 target **永远是舞台**  拿 click.target 判断"点的是图片还是背景"
     * 会得到恒 false, 单击就直接把查看器关掉 (双击放大也因此被打断).
     */
    const downOnImage = useRef(false);
    const closeTimer = useRef<number | null>(null);
    const clickTimer = useRef<number | null>(null);

    const clearTimers = useCallback(() => {
        if (closeTimer.current !== null) { window.clearTimeout(closeTimer.current); closeTimer.current = null; }
        if (clickTimer.current !== null) { window.clearTimeout(clickTimer.current); clickTimer.current = null; }
    }, []);

    /**
     * 把当前 view 画到 DOM 上.
     *
     * 越界收敛是这里的核心: 图片比舞台小时中心必须钉死 (否则"放大过再缩回来, 图被拖出了屏幕"),
     * 比舞台大时才允许平移, 且边界停在图片边缘  即永远拖不出空白.
     *
     * 下面这条 transform 与上面那段坐标系数是同一个决策的两半, 改动前先读
     * (see ):
     * 居中靠负 margin 而不是 `translate(-50%, -50%)`, 单位一律屏幕像素.
     */
    const apply = useCallback(/**
                               * 图片查看器自己管缩放与平移, 不复用幻灯架构图那套
                               * .agents/notes/implemented/feature/2026-09-27-image-viewer-zoom-and-pan.md
                               */
                              () => {
        const img = imgRef.current;
        const stage = stageRef.current;
        const fit = fitRef.current;
        if (!img || !stage || !fit) return;
        const v = view.current;
        // 边界也是屏幕像素: 图片比舞台大多少, 中心就能往两边各走一半
        const mx = Math.max(0, fit.w * v.f - stage.clientWidth) / 2;
        const my = Math.max(0, fit.h * v.f - stage.clientHeight) / 2;
        v.x = clamp(v.x, -mx, mx);
        v.y = clamp(v.y, -my, my);
        img.style.transform = 'translate(' + v.x + 'px, ' + v.y + 'px) scale(' + v.f + ')';
    }, []);

    /** 平滑过渡只给按钮/双击这类一次性动作; 拖拽与捏合上一律关掉, 否则手感发飘 */
    const animate = useCallback((on: boolean) => {
        const img = imgRef.current;
        if (img) img.style.transition = on ? 'transform 0.2s cubic-bezier(0.22, 0.61, 0.36, 1)' : 'none';
    }, []);

    /**
     * 缩放. 锚点 (ax, ay) 是**舞台中心坐标系下的屏幕像素偏移** (即 clientX 减去舞台中心).
     *
     * 让锚点处对应到图片上的那个点保持不动, 解出来就是这条式子  与当前倍率、与是否拖过都无关.
     */
    const zoomAt = useCallback((nextF: number, ax: number, ay: number) => {
        const v = view.current;
        const f1 = clamp(nextF, 1, maxF(fitRef.current));
        if (Math.abs(f1 - v.f) < 1e-4) return;
        v.x = ax + (v.x - ax) * (f1 / v.f);
        v.y = ay + (v.y - ay) * (f1 / v.f);
        v.f = f1;
        setScalePct(Math.round(f1 * 100));
        apply();
    }, [apply]);

    /** 复位 = 倍数回 1 且回正. 同时作废 fit: 旋转屏幕/改窗口后旧值已经不准了 */
    const reset = useCallback(() => {
        view.current = { f: 1, x: 0, y: 0 };
        setScalePct(100);
        fitRef.current = null;
        setReady(false);
        requestAnimationFrame(() => measureRef.current());
    }, []);

    const handleOpen = (e: React.MouseEvent<HTMLImageElement>) => {
        onClick?.(e);
        clearTimers();
        view.current = { f: 1, x: 0, y: 0 };
        fitRef.current = null;
        setBroken(false);
        setReady(false);
        setScalePct(100);
        setIsOpen(true);
        requestAnimationFrame(() => setIsAnimating(true));
    };

    const handleClose = useCallback(() => {
        clearTimers();
        setIsAnimating(false);
        closeTimer.current = window.setTimeout(() => {
            closeTimer.current = null;
            setIsOpen(false);
            setDragging(false);
        }, CLOSE_MS);
    }, [clearTimers]);

    useEffect(() => clearTimers, [clearTimers]);

    /**
     * 量出 contain 后的尺寸, 写进 img 的内联样式, 再应用当前视角.
     *
     * 尺寸由 JS 算而不是交给 CSS object-fit: 平移边界需要知道**图片实际占了多少舞台**,
     * 而 object-fit 的盒子永远等于容器, 拿不到真实边界  那正是"拖不出边界"的来源.
     * 这里用 position:absolute + left/top:50% + translate(-50%,-50%) 把图片钉在舞台中心,
     * 中心即唯一锚点.
     */
    useLayoutEffect(() => {
        if (!isOpen) return;
        const stage = stageRef.current;
        const img = imgRef.current;
        if (!stage || !img) return;

        const measure = () => {
            const sw = stage.clientWidth;
            const sh = stage.clientHeight;
            const nw = img.naturalWidth;
            const nh = img.naturalHeight;
            if (!sw || !sh || !nw || !nh) return;
            const k = Math.min(sw / nw, sh / nh);
            const w = Math.max(1, Math.round(nw * k));
            const h = Math.max(1, Math.round(nh * k));
            const prev = fitRef.current;
            if (!prev || prev.w !== w || prev.h !== h) {
                fitRef.current = { w, h };
                img.style.width = w + 'px';
                img.style.height = h + 'px';
                // 负 margin 把图片中心钉在舞台中心, 于是 (x, y) 就是"中心相对中心的偏移"
                img.style.marginLeft = -(w / 2) + 'px';
                img.style.marginTop = -(h / 2) + 'px';
                setReady(true);
            }
            apply();
        };
        measureRef.current = measure;
        measure();
        const raf = requestAnimationFrame(measure);
        const ro = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(measure) : null;
        if (ro) ro.observe(stage);
        window.addEventListener('resize', measure);
        return () => {
            cancelAnimationFrame(raf);
            ro?.disconnect();
            window.removeEventListener('resize', measure);
        };
    }, [isOpen, apply]);

    /**
     * 打开期间锁住宿主页面滚动, 并接管键盘.
     *
     * 记下原值再写 hidden 而不是关闭时无条件清空: 宿主 (例如幻灯片的卡片弹层) 可能
     * 自己就把 body 设成了 hidden, 无条件清空会把它的锁一起拆掉.
     */
    useEffect(() => {
        if (!isOpen) return;
        const prevOverflow = document.body.style.overflow;
        document.body.style.overflow = 'hidden';
        const onKeyDown = (e: KeyboardEvent) => {
            if (e.key === 'Escape') {
                e.preventDefault();
                e.stopPropagation();
                handleClose();
                return;
            }
            if (!['+', '=', '-', '_', '0'].includes(e.key)) return;
            e.preventDefault();
            e.stopPropagation();
            animate(true);
            if (e.key === '0') { reset(); return; }
            zoomAt(view.current.f * (e.key === '-' || e.key === '_' ? 1 / 1.4 : 1.4), 0, 0);
        };
        // 捕获阶段: 查看器打开时, 这些键不该再被宿主页面的快捷键抢走
        window.addEventListener('keydown', onKeyDown, true);
        return () => {
            window.removeEventListener('keydown', onKeyDown, true);
            document.body.style.overflow = prevOverflow;
        };
    }, [isOpen, handleClose, animate, reset, zoomAt]);

    /**
     * 滚轮缩放, 以光标为锚点.
     *
     * 两个必须点:
     * · 原生绑定 + { passive: false }  React 17 起 wheel 是 passive 的, onWheel 里
     *   preventDefault() 无效, 事件会继续去滚宿主页面.
     * · 打开期间在 window 捕获阶段无条件吃掉 wheel, 否则光标滑出图片一步, 背后的文章就跟着滚了.
     */
    useEffect(() => {
        if (!isOpen) return;
        const onWheel = (e: WheelEvent) => {
            e.preventDefault();
            e.stopPropagation();
            const stage = stageRef.current;
            const fit = fitRef.current;
            if (!stage || !fit) return;
            const r = stage.getBoundingClientRect();
            if (e.clientX < r.left || e.clientX > r.right
                || e.clientY < r.top || e.clientY > r.bottom) return;
            // deltaMode: 0 像素 / 1 行 / 2 页  不换算, 触控板与 Firefox 的步长会差一个量级
            const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? r.height : 1;
            const dy = clamp(e.deltaY * unit, -120, 120);
            animate(false);
            zoomAt(
                view.current.f * Math.exp(-dy * 0.0018),
                e.clientX - r.left - r.width / 2,
                e.clientY - r.top - r.height / 2,
            );
        };
        window.addEventListener('wheel', onWheel, { passive: false, capture: true });
        return () => window.removeEventListener('wheel', onWheel, { capture: true });
    }, [isOpen, animate, zoomAt]);

    /** 指针按下: 单指进平移, 双指进捏合. 两者都以按下时的视角为基准 */
    const onPointerDown = (e: React.PointerEvent<HTMLDivElement>) => {
        const g = gesture.current;
        g.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
        e.currentTarget.setPointerCapture?.(e.pointerId);
        gestureStart.current = { ...view.current };
        lastDragMoved.current = false;
        downOnImage.current = e.target === imgRef.current;
        animate(false);
        if (g.pointers.size >= 2) {
            const pts = [...g.pointers.values()];
            g.kind = 'pinch';
            g.pointerId = e.pointerId;
            g.dist = Math.max(1, Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y));
            lastDragMoved.current = true;
            return;
        }
        g.kind = 'pan';
        g.pointerId = e.pointerId;
        g.sx = e.clientX;
        g.sy = e.clientY;
    };

    const onPointerMove = (e: React.PointerEvent<HTMLDivElement>) => {
        const g = gesture.current;
        if (!g.pointers.has(e.pointerId)) return;
        g.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
        const fit = fitRef.current;
        if (!fit) return;

        if (g.kind === 'pinch' && g.pointers.size >= 2) {
            const stage = stageRef.current;
            if (!stage) return;
            const pts = [...g.pointers.values()];
            const dist = Math.max(1, Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y));
            const r = stage.getBoundingClientRect();
            const send = gestureStart.current;
            const ax = (pts[0].x + pts[1].x) / 2 - r.left - r.width / 2;
            const ay = (pts[0].y + pts[1].y) / 2 - r.top - r.height / 2;
            const f1 = clamp(send.f * (dist / g.dist), 1, maxF(fit));
            const v = view.current;
            v.x = ax + (send.x - ax) * (f1 / send.f);
            v.y = ay + (send.y - ay) * (f1 / send.f);
            v.f = f1;
            setScalePct(Math.round(f1 * 100));
            apply();
            return;
        }

        if (g.kind === 'pan' && e.pointerId === g.pointerId) {
            const dx = e.clientX - g.sx;
            const dy = e.clientY - g.sy;
            if (Math.abs(dx) > MOVE_TOLERANCE || Math.abs(dy) > MOVE_TOLERANCE) {
                lastDragMoved.current = true;
                setDragging(true);
            }
            const send = gestureStart.current;
            const v = view.current;
            // 屏幕像素直接叠加, 不按倍率折算  手移动多少, 图就该走多少
            v.x = send.x + dx;
            v.y = send.y + dy;
            apply();
        }
    };

    const endPointer = (e: React.PointerEvent<HTMLDivElement>) => {
        const g = gesture.current;
        g.pointers.delete(e.pointerId);
        e.currentTarget.releasePointerCapture?.(e.pointerId);
        if (g.kind === 'pinch' && g.pointers.size === 1) {
            // 松开一根手指后接着拖: 剩下的那根成为新的平移起点
            const only = [...g.pointers.entries()][0];
            g.kind = 'pan';
            g.pointerId = only[0];
            g.sx = only[1].x;
            g.sy = only[1].y;
            gestureStart.current = { ...view.current };
            return;
        }
        if (g.pointers.size === 0) {
            g.kind = null;
            g.pointerId = -1;
            setDragging(false);
        }
    };

    /**
     * 单击关闭.
     *
     * 判据两条, 缺一不可:
     * · 拖过 (位移超过阈值) 不算点击  否则每次拖完松手都会把查看器关掉;
     * · 已经放大过也不算  放大之后"点一下"是放开鼠标, 想退出得点背景 / Esc / × 或双击复位.
     * 点图片时还要等一个双击窗口, 不然"双击放大"的第一下就把它关了.
     */
    const onStageClick = (e: React.MouseEvent<HTMLDivElement>) => {
        if (lastDragMoved.current) return;
        if ((e.target as HTMLElement).closest('button')) return;
        const onImage = downOnImage.current;
        if (view.current.f > SNAP_F) return;
        if (!onImage) { handleClose(); return; }
        if (clickTimer.current !== null) window.clearTimeout(clickTimer.current);
        clickTimer.current = window.setTimeout(() => {
            clickTimer.current = null;
            handleClose();
        }, SINGLE_CLICK_MS);
    };

    /** 双击: 光标处逐档放大; 已到最大档、或双击背景, 则复位 */
    const onDoubleClick = (e: React.MouseEvent<HTMLDivElement>) => {
        if (clickTimer.current !== null) { window.clearTimeout(clickTimer.current); clickTimer.current = null; }
        const stage = stageRef.current;
        const fit = fitRef.current;
        if (!stage || !fit) return;
        const v = view.current;
        const onImage = downOnImage.current;
        lastDragMoved.current = false;
        const maxStep = DOUBLE_ZOOM_STEPS[DOUBLE_ZOOM_STEPS.length - 1];
        let next = 1;
        if (onImage && v.f <= maxStep + 1e-3) {
            next = DOUBLE_ZOOM_STEPS.find((s) => s > v.f + 1e-3) || 1;
        }
        animate(true);
        if (next === 1) { reset(); return; }
        const r = stage.getBoundingClientRect();
        zoomAt(next, e.clientX - r.left - r.width / 2, e.clientY - r.top - r.height / 2);
    };

    /** 工具条上的事件不许冒泡到舞台 (否则点"复位"会顺带被判成单击/双击) */
    const stop = (e: React.MouseEvent) => e.stopPropagation();

    const thumbnailStyle: React.CSSProperties = { cursor: 'zoom-in', ...style };
    const overlayClass = [styles.overlay, overlayClassName, isAnimating ? styles.overlayOpen : '']
        .filter(Boolean).join(' ');

    return (
        <>
            <img
                {...props}
                className={className}
                style={thumbnailStyle}
                onClick={handleOpen}
            />

            {isOpen && createPortal(
                <div
                    className={overlayClass}
                    role="dialog"
                    aria-modal="true"
                    aria-label={props.alt || '图片预览'}
                >
                    <div
                        ref={stageRef}
                        className={styles.stage}
                        style={{ cursor: dragging ? 'grabbing' : scalePct > 102 ? 'grab' : 'zoom-in' }}
                        onPointerDown={onPointerDown}
                        onPointerMove={onPointerMove}
                        onPointerUp={endPointer}
                        onPointerCancel={endPointer}
                        onClick={onStageClick}
                        onDoubleClick={onDoubleClick}
                    >
                        <img
                            ref={imgRef}
                            src={props.src}
                            alt={props.alt}
                            className={styles.image}
                            draggable={false}
                            style={{ ...fullScreenImageStyle, opacity: ready ? 1 : 0 }}
                            onLoad={() => requestAnimationFrame(() => measureRef.current())}
                            onError={() => setBroken(true)}
                        />
                    </div>

                    {broken
                        ? <div className={styles.broken}>图片加载失败 (外链资源可能当前不可达)</div>
                        : (
                            <div
                                className={styles.hint}
                                style={{ display: hintShown ? 'none' : undefined }}
                                ref={() => { hintShown = true; }}
                            >
                                滚轮缩放 · 拖拽平移 · 双击放大 · Esc 关闭
                            </div>
                        )}

                    <div className={styles.toolbar} onClick={stop} onDoubleClick={stop} onPointerDown={stop}>
                        <button
                            type="button"
                            className={styles.button}
                            aria-label="缩小"
                            disabled={scalePct <= 100}
                            onClick={() => { animate(true); zoomAt(view.current.f / 1.4, 0, 0); }}
                        >
                            −
                        </button>
                        <span className={styles.scale}>{scalePct + '%'}</span>
                        <button
                            type="button"
                            className={styles.button}
                            aria-label="放大"
                            onClick={() => { animate(true); zoomAt(view.current.f * 1.4, 0, 0); }}
                        >
                            +
                        </button>
                        <button
                            type="button"
                            className={styles.button + ' ' + styles.text}
                            aria-label="复位"
                            onClick={() => { animate(true); reset(); }}
                        >
                            复位
                        </button>
                        <button
                            type="button"
                            className={styles.button}
                            aria-label="关闭"
                            onClick={handleClose}
                        >
                            ×
                        </button>
                    </div>
                </div>,
                document.body,
            )}
        </>
    );
}
