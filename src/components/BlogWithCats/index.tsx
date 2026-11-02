import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import './BlogWithCats.css';

/** 猫娘立绘: RGBA 透明底无损 PNG (原生分辨率, 约 2x 以上显示宽度, 不缩不降质), 左边那只朝右看、右边那只朝左看, 都朝向正文 */
const NEKO = {
    left: { src: '/img/catgirl-neko-left.png', width: 753, height: 1128 },
    right: { src: '/img/catgirl-neko-right.png', width: 693, height: 1240 },
} as const;

/** 单侧立绘的最小 / 最大宽度 (px); 两侧空白放不下最小宽度时整组不渲染 */
const NEKO_MIN = 160;
const NEKO_MAX = 240;
/** 立绘与正文之间至少留的空隙 */
const NEKO_GAP = 24;

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
 * 两侧猫娘 (fixed, 贴视口左右下角): 宽度 = 两侧空白减间隙, 夹在 160 到 240px; 正文宽 contentWidth 不受影响.
 * 贴合方式: 立绘本身是 RGBA 透明底抠图 (轮廓 alpha 抗锯齿), 只有底部半身裁切处用 CSS 底部渐隐, 所以四周没有矩形边; 明暗主题共用同一张, 不需要底色匹配.
 * pointer-events: none, 不挡点击; 页脚盖在立绘之上, 滚到底时立绘沉到页脚后面.
 * 决策记录见本文件里 BlogWithCats 的注释.
 */
export function SideCats ({ contentWidth = 960 }: { contentWidth?: number }): React.ReactElement | null {
    const room = useRoomForCats(contentWidth);
    const left = useBaseUrl(NEKO.left.src);
    const right = useBaseUrl(NEKO.right.src);
    if (!room) return null;
    const style = {
        '--neko-w': `clamp(${NEKO_MIN}px, calc((100vw - ${contentWidth}px) / 2 - ${NEKO_GAP}px), ${NEKO_MAX}px)`,
    } as React.CSSProperties;
    return (
        <div className="neko-sides" style={style} aria-hidden="true">
            <img className="neko-side neko-side-left" src={left} width={NEKO.left.width} height={NEKO.left.height} alt="" decoding="async" />
            <img className="neko-side neko-side-right" src={right} width={NEKO.right.width} height={NEKO.right.height} alt="" decoding="async" />
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
 * .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md
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
