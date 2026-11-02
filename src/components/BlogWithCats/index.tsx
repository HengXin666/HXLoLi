import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import './BlogWithCats.css';

/**
 * 猫娘立绘对: 放在 static/img/catgirl/, 每对 pairN-left.png / pairN-right.png, RGBA 透明底无损 PNG 原生分辨率.
 * "探头"构图: 角色贴图片内侧 (朝正文那边) 的竖直裁切线, 上方大面积透明; 左图朝右看, 右图朝左看.
 * 6 对角色取自用户素材 static/img/catgirl-source/ (以原图为参考重绘探头构图, 绿幕抠出 + 去溢色 + 边缘 0.6px 羽化), 原图保留.
 * 不动 static/default-img/neko_*.png (原组件素材).
 */
const NEKO_PAIRS = [
    { left: { w: 721, h: 1730 }, right: { w: 737, h: 1768 } },
    { left: { w: 732, h: 1757 }, right: { w: 725, h: 1740 } },
    { left: { w: 640, h: 1535 }, right: { w: 768, h: 1844 } },
    { left: { w: 765, h: 1837 }, right: { w: 705, h: 1691 } },
    { left: { w: 774, h: 1857 }, right: { w: 619, h: 1486 } },
    { left: { w: 703, h: 1688 }, right: { w: 723, h: 1735 } },
] as const;

/** 本次页面加载抽中的那一对: 模块级变量, SPA 站内跳转模块不重载所以不重抽, 刷新页面才重抽; 左右用同一索引 */
let pickedPair: number | null = null;
function pickPair (): number {
    if (pickedPair === null) pickedPair = Math.floor(Math.random() * NEKO_PAIRS.length);
    return pickedPair;
}

/** 单侧立绘的最小 / 最大宽度 (px); 两侧空白放不下最小宽度时整组不渲染 */
const NEKO_MIN = 240;
const NEKO_MAX = 320;
/** 立绘外侧 (靠视口边) 至少留的空隙; 内侧紧贴正文边缘 */
const NEKO_GAP = 16;

/** 视口够宽 (正文宽 + 两侧各一只最小宽度的猫娘 + 间隙) 才渲染. SSR 与首帧都不渲染, 窄屏永远不请求图片 */
function useRoomForCats (contentWidth: number): boolean {
    const [ok, setOk] = React.useState(false);
    React.useEffect(() => {
        const mq = window.matchMedia(`(min-width: ${contentWidth + 2 * (NEKO_MIN + NEKO_GAP)}px)`);
        const update = (): void => setOk(mq.matches);
        update();
        mq.addEventListener('change', update);
        return () => mq.removeEventListener('change', update);
    }, [contentWidth]);
    return ok;
}
/**
 * 两侧猫娘: fixed 竖直贴视口底 (滚动时一直在), 水平位置按正文算 —— 立绘内侧的竖直裁切线正好落在正文容器的左 / 右外沿,
 * 像从正文背后探出头来; 不贴视口边. 宽度 = 两侧空白减外侧间隙, 夹在 240 到 320px.
 * 层级: 立绘 z-index 3 高于正文 (.blog-content 为 2), pointer-events: none 不挡点击; 页脚 z-index 4 盖住立绘.
 * 顶部与底部都用 mask 渐隐 (顶部从虚无浮现, 底部半身裁切处淡出).
 * .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md
 */
export function SideCats ({ contentWidth = 960 }: { contentWidth?: number }): React.ReactElement | null {
    const room = useRoomForCats(contentWidth);
    // 只在客户端抽 (room 为 true 时已在 effect 之后), SSR 不渲染立绘, 不会有水合不一致
    const idx = room ? pickPair() : 0;
    const pair = NEKO_PAIRS[idx];
    const left = useBaseUrl(`/img/catgirl/pair${idx + 1}-left.png`);
    const right = useBaseUrl(`/img/catgirl/pair${idx + 1}-right.png`);
    if (!room) return null;
    const style = {
        '--neko-w': `clamp(${NEKO_MIN}px, calc((100vw - ${contentWidth}px) / 2 - ${NEKO_GAP}px), ${NEKO_MAX}px)`,
        '--neko-cw': `${contentWidth}px`,
    } as React.CSSProperties;
    return (
        <div className="neko-sides" style={style} aria-hidden="true">
            <img className="neko-side neko-side-left" src={left} width={pair.left.w} height={pair.left.h} alt="" decoding="async" />
            <img className="neko-side neko-side-right" src={right} width={pair.right.w} height={pair.right.h} alt="" decoding="async" />
        </div>
    );
}

interface Props {
    style?: React.CSSProperties;
    children: React.ReactNode;
    /** 正文最大宽度, 默认 960 (与 .blog-content-wrapper 一致) */
    contentWidth?: number;
}
/**
 * 两侧带猫娘的正文容器: cv / 友链 / 博客列表在用. 正文是带底色的圆角块, 猫娘见 SideCats.
 * 决策记录见本文件 SideCats 的注释 (同一目录只留一个锚点).
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
