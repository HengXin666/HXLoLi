import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { themeToVars, type DeckTheme } from './theme/types';
import { DeckThemeProvider } from './theme/context';
import { SlideActiveProvider, DeckInteractiveProvider, DeckPageProvider } from './slide-state';
import { DeckLayerProvider } from './deck-layer';
import { Nav } from './Nav';
import { ThemePicker } from './ThemePicker';
import { loadTheme } from './theme/registry';
import { usePageParam, readThemeParam, writeThemeParam } from './usePageParam';
import { isWheelConsumed } from './wheel-lock';
import { Brand } from './brand';
import './fonts.css';
import './deck.css';
import './ui.css';

const STAGE_W = 1600;
const STAGE_H = 900;
const PAGE_DURATION = 860;   // 与 --hxd-motion-page 对齐
const LOCK_TAIL = 90;        // 动画尾部静默期, 吸收惯性滚轮
const WHEEL_THRESHOLD = 18;
const TOUCH_THRESHOLD = 55;
/**
 * 重置与重播之间的间隔.
 * 作用: 先移除 is-entered 让元素回到基态, 隔一帧再加回来,
 * 浏览器才会把这次变化识别为**新的过渡**并播放动画.
 * 没有这 40ms, 连续翻页时动画不会重播 (参考包同样用 40ms).
 */
const RESET_GUARD = 40;

export interface DeckProps {
    theme: DeckTheme;
    children: React.ReactNode;
    index?: number;
    onIndexChange?: (i: number) => void;
    showNav?: boolean;
    showDots?: boolean;
    showPager?: boolean;
    showBrand?: boolean;
    /** 由宿主给定高度 (避免出现页面级滚动条) */
    fill?: boolean;
    /** 把当前页码同步到地址栏 ?page=N (刷新/分享可回到同一页) */
    syncUrl?: boolean;
    /**
     * 是否接管交互 (滚轮翻页 / 键盘 / 拖拽).
     *
     * 为什么需要它:
     *   卡片在**预览态**时, deck 铺在正文里. 若此时接管滚轮, 用户在卡片上滚
     *   会被拿去翻演示页, 而页面纹丝不动 —— 违反直觉 ("页面滚不动了").
     *   因此预览态传 false (纯静态展示), 打开后的放大态才传 true.
     */
    interactive?: boolean;
    /** 是否在前端提供主题切换器 */
    themeSwitcher?: boolean;
    /** 初始主题 id (会尝试从 static/themes 按名加载) */
    themeId?: string;
    /** 主题切换后的回调 */
    onThemeChange?: (id: string) => void;
    className?: string;
}

/**
 * Deck 运行时 —— **整屏纵向位移**, 不是淡入淡出.
 *
 * 核心机制 (与答辩模板一致):
 *   1. 所有屏在纵向**依次排开** (每屏 100% 高), 组成一个 N 屏高的长条
 *   2. 翻页 = 对这个长条做 translateY(-index * 100%) —— 真实的"向上滚动"
 *   3. 加动画锁, 切换进行中忽略新输入, 防止惯性滚动连跳
 *   4. 舞台缩放仍在: 每屏内部是固定 1600x900, 用 scale 适配容器
 *
 * 这样"向下移动"的观感才成立; 单纯切 opacity 是没有位移的, 谈不上滚动.
 */
export function Deck({
    theme,
    children,
    index,
    onIndexChange,
    showNav = true,
    showDots = true,
    showPager = true,
    showBrand = true,
    fill = false,
    syncUrl = false,
    interactive = true,
    themeSwitcher = false,
    themeId,
    onThemeChange,
    className,
}: DeckProps): React.ReactElement {
    /**
     * 展开 children -> 屏列表.
     *
     * 必须递归展开 Fragment: deck 常由 `<>{...}</>` 包裹 (如注册表里的 render()),
     * 此时 Children.toArray 只给到**一个** Fragment, total 恒为 1,
     * 于是导航/进度/翻页全部不渲染 —— 表现为"嵌入了但什么都没有".
     */
    const items = useMemo(() => {
        const out: { node: React.ReactNode; title: string; chapter: string }[] = [];

        /** 是否像是 "一屏" (有 title/chapter 声明, 或本身就是 Slide) */
        const looksLikeSlide = (el: React.ReactElement): boolean => {
            const p = (el.props ?? {}) as { title?: string; chapter?: string };
            return p.title !== undefined || p.chapter !== undefined;
        };

        const walk = (nodes: React.ReactNode): void => {
            React.Children.forEach(nodes, (child) => {
                if (!React.isValidElement(child)) return;
                // Fragment: 拆开继续走
                if (child.type === React.Fragment) {
                    walk((child.props as { children?: React.ReactNode }).children);
                    return;
                }
                /*
                  注意: 这里**不能**直接调用函数组件来"展开"它.
                  曾经写过 if (typeof child.type === 'function') walk(createElement(Comp, props)),
                  结果是无限递归: 组件渲染出的还是它自己 -> Maximum call stack.
                  正确做法: deck 组件由 PptEmbed 先渲染成 Fragment<Slide...>, 再交给 Deck;
                  Deck 只负责展开 Fragment.
                */
                const p = (child.props ?? {}) as { title?: string; chapter?: string };
                out.push({
                    node: child,
                    title: p.title ?? `第 ${out.length + 1} 屏`,
                    chapter: p.chapter ?? '',
                });
            });
        };
        walk(children);
        return out;
    }, [children]);

    const total = items.length;
    // syncUrl: 把页码写进地址栏 (?page=N), 刷新/分享都能回到同一页.
    // 编辑器等"内嵌画布"默认关闭, 免得污染当前 URL.
    const [urlIndex, pushUrlIndex] = usePageParam(0);
    /*
      页码以**内部 state 为准**, props.index 只做"初始定位 + 外部改值".

      这是"指定页码后侧边栏就不更新了"的根因:
      之前 current 直接由 props.index 算出来, 一旦链接写成 [##PPT 3##],
      index 恒为 2; 点侧栏/圆点/翻页按钮时 applyGo 改了内部 state,
      current 却仍等于 props —— 长条滚走了, 侧栏高亮/圆点/进度条纹丝不动.
    */
    const [innerIndex, setInnerIndex] = useState(() => (index ?? (syncUrl ? urlIndex : 0)));
    useEffect(() => {
        if (index !== undefined) setInnerIndex(index);
    }, [index]);
    const current = Math.max(0, Math.min(total - 1, innerIndex));

    // 运行时主题: 初始用传入的 theme; 允许前端切换后覆盖
    const [activeTheme, setActiveTheme] = useState<DeckTheme>(theme);
    useEffect(() => setActiveTheme(theme), [theme]);

    // 主题来源优先级: URL 的 ?theme= > props.themeId > props.theme
    useEffect(() => {
        const fromUrl = syncUrl ? readThemeParam() : null;
        const want = fromUrl || themeId;
        if (!want) return;
        let alive = true;
        loadTheme(want).then((t) => {
            if (alive && t) setActiveTheme(t);
        });
        return () => { alive = false; };
    }, [themeId, syncUrl]);

    const hostRef = useRef<HTMLDivElement>(null);
    /**
     * 演示页根节点同时作为"页内浮层挂载点"下发给控件 (见 deck-layer.tsx).
     *
     * 为什么用 state 而不是只留 ref: 浮层要 createPortal(inner, host),
     * 而首帧 host 还没挂载, Portal 需要一个真实的 DOM 节点. 用回调 ref 把节点
     * 提升成 state, 挂载后重渲染一次, 控件才拿得到.
     */
    const [layerHost, setLayerHost] = useState<HTMLElement | null>(null);
    /** 有控件正在占用整页 (如架构图放大) —— 让 deck 自己的装饰+层级让位 */
    const [layerFull, setLayerFull] = useState(false);
    const attachHost = useCallback((el: HTMLDivElement | null) => {
        hostRef.current = el;
        setLayerHost(el);
    }, []);
    const layer = useMemo(() => ({ host: layerHost, setLayerFull }), [layerHost]);
    const stripRef = useRef<HTMLDivElement>(null);
    const currentRef = useRef(current);
    currentRef.current = current;
    const interactiveRef = useRef(interactive);
    interactiveRef.current = interactive;
    const lockedRef = useRef(false);
    const lockTimer = useRef(0);
    /** 正在播放入场动画的屏; -1 表示当前没有屏处于"已入场"态 */
    const [entered, setEntered] = useState(-1);
    const enterTimer = useRef(0);

    const applyGo = useCallback(
        (next: number, instant = false) => {
            const clamped = Math.max(0, Math.min(total - 1, next));
            if (clamped === currentRef.current && !instant) return;
            currentRef.current = clamped;

            // 动画锁: 期间忽略新输入
            lockedRef.current = true;
            window.clearTimeout(lockTimer.current);
            lockTimer.current = window.setTimeout(() => {
                lockedRef.current = false;
            }, PAGE_DURATION + LOCK_TAIL);

            const strip = stripRef.current;
            if (strip) {
                if (instant) strip.classList.add('is-instant');
                // 注意: translateY 的 % 基准是元素**自身高度** (= N 屏),
                // 所以单屏位移是 -(1/N*100)%, 不能写 -100% (那会跳过 N 屏)
                strip.style.transform = `translate3d(0, -${(clamped / Math.max(total, 1)) * 100}%, 0)`;
                if (instant) {
                    void strip.offsetHeight;   // 强制回流, 使后续切换恢复动画
                    strip.classList.remove('is-instant');
                }
            }

            // 出场: 立刻清掉 entered, 旧屏元素依 transition 回退到基态 (有动画)
            setEntered(-1);
            window.clearTimeout(enterTimer.current);
            // 入场: 隔 RESET_GUARD 再加回, 确保被识别为新过渡
            enterTimer.current = window.setTimeout(() => setEntered(clamped), instant ? 0 : RESET_GUARD);
            // 无论是否受控都要更新内部 state, 否则 props.index 固定时侧栏不会跟着动
            setInnerIndex(clamped);
            onIndexChange?.(clamped);
        },
        [index, onIndexChange, total],
    );

    /**
     * 翻页入口.
     *
     * 注意 (一个真实 bug): 当外部通过 `index` 受控传值时, 之前的实现只调用
     * onIndexChange 而不更新内部 state —— 若外部没提供 onIndexChange (PptEmbed 就是这样),
     * 点了侧栏也不会变页.
     * 现在: 无论是否受控, 都同步内部 state; 受控时额外通知外部.
     */
    const go = useCallback(
        (next: number) => {
            applyGo(next);
            if (index !== undefined) onIndexChange?.(Math.max(0, Math.min(total - 1, next)));
            if (syncUrl) pushUrlIndex(next);
        },
        [applyGo, index, onIndexChange, syncUrl, pushUrlIndex, total],
    );

    // 首次挂载: 无动画定位 + 首屏入场
    useEffect(() => {
        const strip = stripRef.current;
        if (!strip) return;
        strip.style.transform = `translate3d(0, -${(currentRef.current / Math.max(total, 1)) * 100}%, 0)`;
        enterTimer.current = window.setTimeout(() => setEntered(currentRef.current), RESET_GUARD);
        return () => window.clearTimeout(enterTimer.current);
    }, [total]);

    // 浏览器前进/后退 -> 跟随 URL 回到对应页
    useEffect(() => {
        if (!syncUrl) return;
        const onPop = () => {
            const i = Math.max(0, Math.min(total - 1, urlIndex));
            applyGo(i, true);
        };
        window.addEventListener('popstate', onPop);
        return () => window.removeEventListener('popstate', onPop);
    }, [syncUrl, urlIndex, total, applyGo]);

    // 滚轮: 累计阈值 + 锁
    useEffect(() => {
        const host = hostRef.current;
        if (!host || total <= 1) return;
        let accum = 0;
        let resetTimer = 0;

        const inScrollable = (target: EventTarget | null): boolean => {
            let n = target as HTMLElement | null;
            while (n && n !== host) {
                const cs = getComputedStyle(n);
                if (/(auto|scroll)/.test(cs.overflowY) && n.scrollHeight > n.clientHeight + 2) return true;
                n = n.parentElement;
            }
            return false;
        };

        /** deck 是否还有足够面积留在视口内 —— 滚出视野后就不该再接管滚轮 */
        const visibleEnough = (): boolean => {
            const r = host.getBoundingClientRect();
            const vh = window.innerHeight || 0;
            const shown = Math.min(r.bottom, vh) - Math.max(r.top, 0);
            return shown > Math.min(r.height, vh) * 0.5;
        };

        const onWheel = (e: WheelEvent) => {
            // 预览态不接管滚轮: 让页面正常滚动
            if (!interactiveRef.current) return;
            // 内层控件 (如图) 已在原生监听里消费掉这次滚轮 —— 不再翻页.
            // 这是"缩放到极限后滚轮变成翻页"那个 bug 的修法.
            if (isWheelConsumed(e)) return;
            // 内部可滚动区 (代码块等) 优先
            if (inScrollable(e.target)) return;
            // 滚出视野时不再接管: 否则会与页面滚动互相打架, 出现"元素整体上移且回不来"
            if (!visibleEnough()) return;
            // 动画锁期间别急着 preventDefault: 抢了默认行为又不翻页, 页面会"被吃掉一格"
            if (lockedRef.current) return;
            e.preventDefault();
            accum += e.deltaY;
            window.clearTimeout(resetTimer);
            resetTimer = window.setTimeout(() => { accum = 0; }, 140);
            if (Math.abs(accum) < WHEEL_THRESHOLD) return;
            const dir = accum > 0 ? 1 : -1;
            accum = 0;
            applyGo(currentRef.current + dir);
        };

        host.addEventListener('wheel', onWheel, { passive: false });
        return () => { host.removeEventListener('wheel', onWheel); window.clearTimeout(resetTimer); };
    }, [applyGo, total]);

    // 键盘
    useEffect(() => {
        const onKey = (e: KeyboardEvent) => {
            // 预览态不抢键盘: 否则会影响正文里的翻页/滚动快捷键
            if (!interactiveRef.current) return;
            const t = e.target as HTMLElement | null;
            if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT' || t.isContentEditable)) return;
            const k = e.key;
            if (k === 'ArrowDown' || k === 'ArrowRight' || k === 'PageDown' || k === ' ') {
                if (t && (t.tagName === 'BUTTON' || t.tagName === 'A')) return;
                e.preventDefault();
                applyGo(currentRef.current + 1);
            } else if (k === 'ArrowUp' || k === 'ArrowLeft' || k === 'PageUp') {
                if (t && (t.tagName === 'BUTTON' || t.tagName === 'A')) return;
                e.preventDefault();
                applyGo(currentRef.current - 1);
            } else if (k === 'Home') { e.preventDefault(); applyGo(0); }
            else if (k === 'End') { e.preventDefault(); applyGo(total - 1); }
        };
        window.addEventListener('keydown', onKey);
        return () => window.removeEventListener('keydown', onKey);
    }, [applyGo, total]);

    // 触屏
    useEffect(() => {
        const host = hostRef.current;
        if (!host || total <= 1) return;
        let startY: number | null = null;
        const onStart = (e: TouchEvent) => { if (!interactiveRef.current) return; startY = e.touches[0]?.clientY ?? null; };
        const onEnd = (e: TouchEvent) => {
            if (!interactiveRef.current) return;
            if (startY === null) return;
            const dy = startY - (e.changedTouches[0]?.clientY ?? startY);
            startY = null;
            if (Math.abs(dy) < TOUCH_THRESHOLD) return;
            applyGo(currentRef.current + (dy > 0 ? 1 : -1));
        };
        host.addEventListener('touchstart', onStart, { passive: true });
        host.addEventListener('touchend', onEnd, { passive: true });
        return () => {
            host.removeEventListener('touchstart', onStart);
            host.removeEventListener('touchend', onEnd);
        };
    }, [applyGo, total]);

    // 舞台缩放: 按每屏容器尺寸算
    const [scale, setScale] = useState(1);
    useEffect(() => {
        const el = hostRef.current;
        if (!el) return;
        const apply = () => {
            // 注意: --hxd-nav-reserve 是 calc() 表达式, parseFloat 会得到 NaN.
            // 必须量**已解析**的 paddingLeft, 否则预留量被当 0, 画布按整宽缩放而与侧栏重叠.
            const screenEl = el.querySelector<HTMLElement>('.hxd-screen');
            const reserve = screenEl ? parseFloat(getComputedStyle(screenEl).paddingLeft) || 0 : 0;
            const availW = Math.max(120, el.clientWidth - reserve);
            const availH = Math.max(120, el.clientHeight);
            setScale(Math.min(availW / STAGE_W, availH / STAGE_H));
        };
        apply();
        const ro = new ResizeObserver(apply);
        ro.observe(el);
        return () => ro.disconnect();
    }, [fill]);

    const styleVars = useMemo(
        () => ({
            ...themeToVars(activeTheme),
            '--hxd-total': String(total),
            // 侧栏没显示时不留白, 否则画布会被无故缩窄
            '--hxd-screen-pad-left': showNav && total > 1 ? 'var(--hxd-nav-reserve)' : '0px',
        } as React.CSSProperties),
        [activeTheme, total, showNav],
    );

    return (
        <DeckThemeProvider theme={activeTheme}>
            {/*
              层端口向下发: 页内浮层 (架构图放大等) 挂在 deck 根节点里而不是 body,
              于是"全屏"的边界天然就是演示页本身 —— 见 deck-layer.tsx.
            */}
            <DeckLayerProvider value={layer}>
            <div
                ref={attachHost}
                className={['hxd-deck', fill ? 'hxd-deck--fill' : '', className].filter(Boolean).join(' ')}
                data-theme={activeTheme.id}
                data-interactive={interactive ? 'true' : 'false'}
                data-skin-pattern={activeTheme.assets?.pattern ? 'true' : 'false'}
                data-layer-full={layerFull ? 'true' : 'false'}
                style={styleVars}
                tabIndex={-1}
            >
                {activeTheme.css ? <style>{activeTheme.css}</style> : null}

                {/* 整屏长条: N 屏纵向堆叠, 位移它 = 滚动 */}
                <div ref={stripRef} className="hxd-strip" style={{ height: `${total * 100}%` }}>
                    {items.map((it, i) => (
                        <section
                            className="hxd-screen"
                            data-active={i === current}
                            data-entered={i === entered ? 'true' : 'false'}
                            key={i}
                            aria-hidden={i !== current}
                        >
                            <div className="hxd-screen__inner" style={{ width: STAGE_W, height: STAGE_H, transform: `scale(${scale})` }}>
                                {/*
                                  交互态随内容一起下发: 预览卡片里的控件据此降级
                                  (如 Diagram 不提供页内全屏, 避免被 stage scale() 裁住 —— 见 slide-state 注释)
                                */}
                                <DeckInteractiveProvider interactive={interactive}>
                                    {/* 页码下发给控件: 分享链接要能带上"此刻在第几页" */}
                                    <DeckPageProvider value={{ index: current, total }}>
                                        <SlideActiveProvider active={i === current}>{it.node}</SlideActiveProvider>
                                    </DeckPageProvider>
                                </DeckInteractiveProvider>
                            </div>
                        </section>
                    ))}
                </div>

                {/* 固定层: 不随长条移动 */}
                {showBrand ? <Brand /> : null}

                {themeSwitcher ? (
                    <div className="hxd-deck__switcher">
                        <ThemePicker
                            value={activeTheme.id}
                            onChange={(t, id) => {
                                setActiveTheme(t);
                                // 主题也写进 URL, 分享链接能带上主题
                                if (syncUrl) writeThemeParam(id);
                                onThemeChange?.(id);
                            }}
                        />
                    </div>
                ) : null}
                {showNav && total > 1 ? (
                    <Nav items={items.map((it, i) => ({ title: it.title, chapter: it.chapter, index: i }))} current={current} onGo={go} />
                ) : null}
                {showDots && total > 1 ? (
                    <div className="hxd-dots" role="tablist" aria-label="页码">
                        {items.map((it, i) => (
                            <button type="button" key={i} role="tab" aria-selected={i === current}
                                aria-label={it.title} title={it.title}
                                className={`hxd-dots__dot${i === current ? ' is-active' : ''}`}
                                onClick={() => go(i)} />
                        ))}
                    </div>
                ) : null}
                {showPager && total > 1 ? (
                    <div className="hxd-pager">
                        <button type="button" className="hxd-pager__btn" onClick={() => go(current - 1)} disabled={current === 0} aria-label="上一页">↑</button>
                        <button type="button" className="hxd-pager__btn" onClick={() => go(current + 1)} disabled={current >= total - 1} aria-label="下一页">↓</button>
                    </div>
                ) : null}
                <div className="hxd-deck__title-now" aria-hidden="true">{items[current]?.title}</div>
                <div className="hxd-deck__progress" style={{ width: `${total ? ((current + 1) / total) * 100 : 0}%` }} />
                {/*
                  这里**不放**任何常驻的署名/页脚.
                  曾经在每页底部挂过 CC 署名链接, 结果 14 页每页都吊着两条外链, 像广告还挡内容.
                  署名属于"元信息", 归主题数据 (theme.credits) 与文档/仓库说明, 不占演示页版面.
                */}
            </div>
            </DeckLayerProvider>
        </DeckThemeProvider>
    );
}

export default Deck;