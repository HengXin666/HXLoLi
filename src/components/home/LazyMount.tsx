import React, { Suspense, useEffect, useRef, useState } from 'react';

/** chunk 下载失败 (断网 / 部署切换) 时只让这一块显示提示, 不让整页卸载 */
class ChunkBoundary extends React.Component<{ children: React.ReactNode }, { failed: boolean }> {
    state = { failed: false };
    static getDerivedStateFromError (): { failed: boolean } {
        return { failed: true };
    }
    render (): React.ReactNode {
        if (this.state.failed) {
            return <p style={{ margin: 0, fontSize: 13, color: 'var(--ifm-color-emphasis-600)' }}>这一块没加载出来, 刷新页面再试.</p>;
        }
        return this.props.children;
    }
}

type Props = {
    /** 占位高度, 与真实组件高度一致, 避免挂载时页面跳动 */
    minHeight: number;
    /** 提前多少像素开始加载 */
    rootMargin?: string;
    label?: string;
    children: React.ReactNode;
};

/**
 * 进入视口附近才挂载子树. 子树里用 React.lazy 引入的图表因此拆成独立 chunk,
 * 首屏 HTML 只有固定高度的占位, 不等 recharts 下载.
 */
export default function LazyMount({ minHeight, rootMargin = '240px', label = '加载中', children }: Props): React.ReactElement {
    const ref = useRef<HTMLDivElement>(null);
    const [show, setShow] = useState(false);

    useEffect(() => {
        const el = ref.current;
        if (!el) return;
        if (typeof IntersectionObserver === 'undefined') {
            setShow(true);
            return;
        }
        const io = new IntersectionObserver((entries) => {
            if (entries.some((e) => e.isIntersecting)) {
                setShow(true);
                io.disconnect();
            }
        }, { rootMargin });
        io.observe(el);
        return () => io.disconnect();
    }, [rootMargin]);

    const placeholder = (
        <div
            aria-busy="true"
            aria-label={label}
            style={{
                minHeight,
                borderRadius: 12,
                background: 'linear-gradient(90deg, rgba(255,255,255,0.03), rgba(255,255,255,0.06), rgba(255,255,255,0.03))',
            }}
        />
    );

    return (
        <div ref={ref} style={{ minHeight }}>
            {show ? <ChunkBoundary><Suspense fallback={placeholder}>{children}</Suspense></ChunkBoundary> : placeholder}
        </div>
    );
}
