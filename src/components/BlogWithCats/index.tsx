import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';
import './BlogWithCats.css';

/**
 * 立绘对: static/img/catgirl/pairN-left.png / pairN-right.png, RGBA 透明底无损 PNG 原生分辨率, 约 1:2.4, 角色在下 55%.
 * left 图的竖直裁切线在右边 (身体藏在右侧墙后), right 图的裁切线在左边.
 * 以用户素材 static/img/catgirl-source/ 的角色为参考生成探头构图 (绿幕抠出 + 去溢色 + 边缘 0.6px 羽化), 素材原图保留.
 * 不动 static/default-img/neko_*.png (原组件素材).
 */
const NEKO_PAIRS = [
    { left: { w: 721, h: 1730 }, right: { w: 737, h: 1768 } },
    { left: { w: 732, h: 1757 }, right: { w: 725, h: 1740 } },
    { left: { w: 640, h: 1535 }, right: { w: 768, h: 1844 } },
    { left: { w: 765, h: 1837 }, right: { w: 705, h: 1691 } },
    { left: { w: 774, h: 1857 }, right: { w: 619, h: 1486 } },
    { left: { w: 703, h: 1688 }, right: { w: 723, h: 1735 } },
    { left: { w: 763, h: 1831 }, right: { w: 904, h: 2170 } },
    { left: { w: 710, h: 1704 }, right: { w: 728, h: 1746 } },
    { left: { w: 845, h: 2028 }, right: { w: 611, h: 1466 } },
    { left: { w: 758, h: 1819 }, right: { w: 700, h: 1680 } },
    { left: { w: 768, h: 1842 }, right: { w: 796, h: 1910 } },
    { left: { w: 775, h: 1860 }, right: { w: 736, h: 1766 } },
    { left: { w: 783, h: 1880 }, right: { w: 713, h: 1711 } },
] as const;

/**
 * 探头变体:
 * - pair: 左右对称, 正文块当墙 (z 2), 立绘 z 1 在墙后, 内侧裁切线藏进正文; 滚动越深身体越往上探 + 向外侧微倾.
 * - corner: 从视口右下角斜探 (视口右边当墙), 滚动时沿斜线升起更多.
 * - edges: 视口左右两边从底部升起半身 (全宽页面没有固定正文宽时用), 滚动升起更多.
 * - side: 只在右侧探头; 快速滚动时缩回墙后, 停下约 0.5 秒再探出来.
 * 所有变体: 入场从墙后滑出, 待机轻微呼吸; 滚动位移只写 CSS 变量 --neko-p (0..1, 滚过一屏到 1), 由 transform 消费.
 * prefers-reduced-motion: 不监听滚动, 无入场 / 呼吸 / 躲藏, 静态探出幅度不变.
 */
export type NekoVariant = 'pair' | 'corner' | 'edges' | 'side';

/** 上一次抽中的索引: 换页时避开它, 保证连续两页不是同一对 */
let lastPair = -1;
function drawPair (): number {
    const n = NEKO_PAIRS.length;
    if (n <= 1) return 0;
    let i = Math.floor(Math.random() * (n - 1));
    if (i >= lastPair && lastPair >= 0) i += 1;
    lastPair = i;
    return i;
}

/**
 * 每个页面一对: 以 pathname 为依赖在 effect 里抽, 整页刷新与站内换页都会重抽; 同页内的 re-render / 滚动不会重抽.
 * SSR 与首帧返回 null (不渲染立绘), 不会有水合不一致. 左右共用同一索引.
 */
function usePairForPage (): number | null {
    const { pathname } = useLocation();
    const [idx, setIdx] = React.useState<number | null>(null);
    React.useEffect(() => {
        setIdx(drawPair());
    }, [pathname]);
    return idx;
}

/** 媒体查询是否命中; SSR 与首帧为 false */
function useMedia (query: string): boolean {
    const [ok, setOk] = React.useState(false);
    React.useEffect(() => {
        const mq = window.matchMedia(query);
        const update = (): void => setOk(mq.matches);
        update();
        mq.addEventListener('change', update);
        return () => mq.removeEventListener('change', update);
    }, [query]);
    return ok;
}

/**
 * 滚动联动: passive scroll + requestAnimationFrame, 每帧最多写一次 --neko-p (滚动进度, 一屏封顶为 1), 不触发 React 渲染.
 * shy 为 true 时 (side 变体) 单帧滚动超过 24px 打上 data-shy, 停止 480ms 后移除.
 * reduced-motion 时什么都不做.
 */
function useScrollPeek (ref: React.RefObject<HTMLDivElement | null>, active: boolean, shy: boolean, token: number): void {
    React.useEffect(() => {
        const el = ref.current;
        if (!el || !active) return;
        if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
        let raf = 0;
        let lastY = window.scrollY;
        let timer: number | undefined;
        const frame = (): void => {
            raf = 0;
            const y = window.scrollY;
            const p = Math.min(1, Math.max(0, y / window.innerHeight));
            el.style.setProperty('--neko-p', p.toFixed(3));
            if (shy && Math.abs(y - lastY) > 24) {
                el.dataset.shy = '';
                window.clearTimeout(timer);
                timer = window.setTimeout(() => { delete el.dataset.shy; }, 480);
            }
            lastY = y;
        };
        const onScroll = (): void => {
            if (!raf) raf = window.requestAnimationFrame(frame);
        };
        frame();
        window.addEventListener('scroll', onScroll, { passive: true });
        return () => {
            window.removeEventListener('scroll', onScroll);
            if (raf) window.cancelAnimationFrame(raf);
            window.clearTimeout(timer);
        };
    }, [ref, active, shy, token]);
}

/** 单侧最小 / 最大宽度 (px, 仅 pair); 两侧空白放不下最小宽度时整组不渲染 */
const NEKO_MIN = 240;
const NEKO_MAX = 320;
/** pair 立绘外侧 (靠视口边) 至少留的空隙 */
const NEKO_GAP = 16;

interface StageProps {
    variant: NekoVariant;
    /** pair 用: 正文宽度, 立绘内侧贴着它的左右外沿 */
    contentWidth?: number;
}
/**
 * 立绘舞台: 一个页面只挂一组 (position: fixed). 外层 .neko-slot 吃滚动 transform, 内层 img 吃入场 / 呼吸动画, 两者不抢 transform.
 * .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md
 */
function NekoStage ({ variant, contentWidth = 960 }: StageProps): React.ReactElement | null {
    const query = variant === 'pair'
        ? `(min-width: ${contentWidth + 2 * (NEKO_MIN + NEKO_GAP)}px)`
        : '(min-width: 1100px)';
    const room = useMedia(query);
    const picked = usePairForPage();
    const ref = React.useRef<HTMLDivElement>(null);
    const idx = picked ?? 0;
    const pair = NEKO_PAIRS[idx];
    const left = useBaseUrl(`/img/catgirl/pair${idx + 1}-left.png`);
    const right = useBaseUrl(`/img/catgirl/pair${idx + 1}-right.png`);
    const show = room && picked !== null;
    // idx 变化时 key 让外层重挂载, token 让监听跟着换到新元素上
    useScrollPeek(ref, show, variant === 'side', idx);
    if (!show) return null;
    const style = variant === 'pair'
        ? {
            '--neko-w': `clamp(${NEKO_MIN}px, calc((100vw - ${contentWidth}px) / 2 - ${NEKO_GAP}px), ${NEKO_MAX}px)`,
            '--neko-cw': `${contentWidth}px`,
        } as React.CSSProperties
        : undefined;
    // left 图裁切线在右: 放在墙 (正文 / 视口右边) 的左侧; right 图裁切线在左: 放在墙的右侧
    const leftImg = <img className="neko-img" src={left} width={pair.left.w} height={pair.left.h} alt="" decoding="async" />;
    const rightImg = <img className="neko-img" src={right} width={pair.right.w} height={pair.right.h} alt="" decoding="async" />;
    return (
        <div className="neko-sides" data-variant={variant} key={idx} ref={ref} style={style} aria-hidden="true">
            {variant === 'pair' && <div className="neko-slot neko-slot-l">{leftImg}</div>}
            {variant === 'pair' && <div className="neko-slot neko-slot-r">{rightImg}</div>}
            {variant === 'edges' && <div className="neko-slot neko-slot-l">{rightImg}</div>}
            {variant !== 'pair' && <div className="neko-slot neko-slot-r">{leftImg}</div>}
        </div>
    );
}

/** 博客 / cv / 友链 / 分享页用: 左右对称墙后探头 */
export function SideCats ({ contentWidth = 960 }: { contentWidth?: number }): React.ReactElement | null {
    return <NekoStage variant="pair" contentWidth={contentWidth} />;
}

/**
 * 没有挂 SideCats 的页面按路由选变体 (挂在 src/theme/Root.tsx, 全站一份): 首页与关系图 edges, 番剧库 corner, 番剧详情 side.
 * 不在表里的路由不渲染, 因此不会和页面自己的 SideCats 叠成两组.
 */
const ROUTE_VARIANTS: Record<string, NekoVariant> = {
    '/': 'edges',
    '/anime': 'corner',
    '/anime/details': 'side',
    '/anime/graph': 'edges',
};
export function RouteCats (): React.ReactElement | null {
    const { pathname } = useLocation();
    const { siteConfig } = useDocusaurusContext();
    const base = siteConfig.baseUrl.replace(/\/$/, '');
    let path = pathname.startsWith(base) ? pathname.slice(base.length) : pathname;
    path = path.replace(/\/+$/, '') || '/';
    const variant = ROUTE_VARIANTS[path];
    if (!variant) return null;
    return <NekoStage key={variant} variant={variant} />;
}

interface Props {
    style?: React.CSSProperties;
    children: React.ReactNode;
    /** 正文最大宽度, 默认 960 (与 .blog-content-wrapper 一致) */
    contentWidth?: number;
}
/**
 * 两侧带猫娘的正文容器: cv / 友链 / 博客列表在用. 正文是带底色的圆角块 (墙), 猫娘见 NekoStage.
 * 决策记录见本文件 NekoStage 的注释 (同一目录只留一个锚点).
 */
const BlogWithCats: React.FC<Props> = ({ style, children, contentWidth = 960 }) => {
    return (
        <div className="blog-container">
            <SideCats contentWidth={contentWidth} />
            <div className="blog-content-wrapper" style={contentWidth === 960 ? undefined : { maxWidth: contentWidth }}>
                <div className="blog-content" style={style}>
                    {children}
                </div>
            </div>
        </div>
    );
};

export default BlogWithCats;
