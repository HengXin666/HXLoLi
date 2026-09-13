import React, { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';

export interface NavItem {
    title: string;
    chapter: string;
    index: number;
}

/**
 * 左侧章节目录条 (滑块式).
 *
 * 借鉴答辩模板的做法并做成 React 版:
 *   · **独立滑块元素**在当前标题之间平滑位移, 而不是给条目换背景色 (那样是跳变, 没有"味道")
 *   · 激活标题放大 + 右侧箭头淡入, 箭头延迟到滑块到位之后
 *   · 当前章节整组实色, 其余整组淡化; 指针悬停时整条恢复
 *   · 标题按字数分档降低放大倍率, 避免长标题与箭头重叠
 */
export function Nav({
    items,
    current,
    onGo,
}: {
    items: NavItem[];
    current: number;
    onGo: (i: number) => void;
}): React.ReactElement {
    const listRef = useRef<HTMLDivElement>(null);
    const itemRefs = useRef<(HTMLButtonElement | null)[]>([]);
    const [thumb, setThumb] = useState<{ top: number; height: number } | null>(null);
    const [first, setFirst] = useState(true);

    /**
     * 计算滑块位置.
     *
     * 踩坑: 曾经混用 offsetTop(相对 offsetParent) 与 getBoundingClientRect 差值,
     * 两者基准不同 -> 滑块与高亮项错位整整一项.
     * 现在统一只用 **相对同一容器(list)的 rect 差值**, 并减去 list 的滚动量,
     * 让滑块始终停留在内容坐标系里.
     */
    const measure = useCallback(() => {
        const el = itemRefs.current[current];
        const list = listRef.current;
        if (!el || !list) return;
        const r = el.getBoundingClientRect();
        const lr = list.getBoundingClientRect();
        // list 有 border 时 clientTop 需要计入, 否则差 1~2px
        setThumb({ top: r.top - lr.top + list.scrollTop - list.clientTop, height: r.height });
    }, [current]);

    useLayoutEffect(() => {
        // 首次渲染时 list 的高度还没稳定 (字体/换行都可能改变), 因此:
        // 立即测一次 -> 下一帧再测一次, 避免滑块停在错误位置或干脆不渲染.
        measure();
        const raf = requestAnimationFrame(() => measure());
        const t = window.setTimeout(() => { measure(); setFirst(false); }, 320);
        return () => { cancelAnimationFrame(raf); window.clearTimeout(t); };
    }, [measure, items.length]);

    // 当前项滚出视野时自动带回来 (长目录必需)
    useEffect(() => {
        const el = itemRefs.current[current];
        if (!el) return;
        const r = el.getBoundingClientRect();
        const vh = window.innerHeight;
        if (r.top < 80 || r.bottom > vh - 60) {
            el.scrollIntoView({ block: 'center', behavior: 'smooth' });
        }
    }, [current]);

    useEffect(() => {
        const list = listRef.current;
        window.addEventListener('resize', measure);
        list?.addEventListener('scroll', measure, { passive: true });
        // 字体加载完会改变标题宽高 -> 必须重新贴合, 否则滑块尺寸不匹配
        document.fonts?.ready?.then?.(measure);
        const ro = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(measure) : null;
        if (ro && list) ro.observe(list);
        return () => {
            window.removeEventListener('resize', measure);
            list?.removeEventListener('scroll', measure);
            ro?.disconnect();
        };
    }, [measure]);

    const chapterOf = (i: number) => items[i]?.chapter ?? '';

    // 按连续 chapter 分组
    const groups: { chapter: string; rows: NavItem[] }[] = [];
    items.forEach((it) => {
        const last = groups[groups.length - 1];
        if (last && last.chapter === it.chapter) last.rows.push(it);
        else groups.push({ chapter: it.chapter, rows: [it] });
    });

    const tierOf = (t: string) => (t.length >= 10 ? 'long' : t.length >= 7 ? 'mid' : 'short');

    return (
        <nav className="hxd-nav" aria-label="章节目录">
            <div className="hxd-nav__count">
                <b>{String(current + 1).padStart(2, '0')}</b>
                <span className="hxd-nav__slash">/</span>
                <span className="hxd-nav__total">{String(items.length).padStart(2, '0')}</span>
            </div>

            <div className="hxd-nav__list" ref={listRef}>
                {thumb ? (
                    <span
                        className="hxd-nav__thumb"
                        data-first={first}
                        style={{ top: thumb.top, height: thumb.height }}
                        aria-hidden="true"
                    />
                ) : null}

                {groups.map((g, gi) => {
                    const isCur = items[current]?.chapter === g.chapter;
                    // 单屏章节: 章节名自己承担跳转与高亮, 不再列二级标题
                    const merged = Boolean(g.chapter) && g.rows.length === 1;
                    if (merged) {
                        const it = g.rows[0];
                        return (
                            <div className={`hxd-nav__group${isCur ? ' is-current' : ''}`} key={gi}>
                                <button
                                    type="button"
                                    ref={(el) => { itemRefs.current[it.index] = el; }}
                                    className={`hxd-nav__item hxd-nav__item--chapter${it.index === current ? ' is-active' : ''}`}
                                    data-len={tierOf(g.chapter)}
                                    onClick={() => onGo(it.index)}
                                    title={g.chapter}
                                >
                                    <span className="hxd-nav__label">{g.chapter}</span>
                                    <span className="hxd-nav__arrow" aria-hidden="true">◀</span>
                                </button>
                            </div>
                        );
                    }
                    return (
                        <div className={`hxd-nav__group${isCur ? ' is-current' : ''}`} key={gi}>
                            {g.chapter ? <div className="hxd-nav__chapter">{g.chapter}</div> : null}
                            <div className="hxd-nav__items">
                                {g.rows.map((it) => (
                                    <button
                                        type="button"
                                        key={it.index}
                                        ref={(el) => { itemRefs.current[it.index] = el; }}
                                        className={`hxd-nav__item${g.chapter ? ' hxd-nav__item--sub' : ''}${it.index === current ? ' is-active' : ''}`}
                                        data-len={tierOf(it.title)}
                                        onClick={() => onGo(it.index)}
                                        title={it.title}
                                    >
                                        <span className="hxd-nav__label">{it.title}</span>
                                        <span className="hxd-nav__arrow" aria-hidden="true">◀</span>
                                    </button>
                                ))}
                            </div>
                        </div>
                    );
                })}
            </div>
        </nav>
    );
}

export default Nav;