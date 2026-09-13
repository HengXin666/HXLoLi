import { useCallback, useEffect, useState } from 'react';

/**
 * 页码与 URL 查询参数双向同步 (?page=3).
 *
 * 为什么需要:
 *   演示页刷新后要能回到同一页, 也要能把"第几页"直接发给别人.
 * 为什么不用 hash:
 *   Docusaurus 用 hash 做页内锚点跳转, 占用它会与文档自身的目录冲突.
 *
 * 约定: 对外是 1-based (人的直觉), 对内是 0-based.
 */
/** 读出 URL 里的主题名 (?theme=xxx) */
export function readThemeParam(): string | null {
    if (typeof window === 'undefined') return null;
    return new URLSearchParams(window.location.search).get('theme');
}

/** 写主题名到 URL (不产生历史记录) */
export function writeThemeParam(id: string | null): void {
    if (typeof window === 'undefined') return;
    const url = new URL(window.location.href);
    if (id) url.searchParams.set('theme', id);
    else url.searchParams.delete('theme');
    window.history.replaceState(window.history.state, '', url.toString());
}

export function usePageParam(initial = 0): [number, (i: number) => void] {
    const read = useCallback((): number => {
        if (typeof window === 'undefined') return initial;
        const raw = new URLSearchParams(window.location.search).get('page');
        if (!raw) return initial;
        const n = Number(raw);
        return Number.isFinite(n) && n > 0 ? Math.max(0, Math.floor(n) - 1) : initial;
    }, [initial]);

    const [index, setIndex] = useState<number>(read);

    // 外部点击前进/后退按钮时同步回来
    useEffect(() => {
        const onPop = () => setIndex(read());
        window.addEventListener('popstate', onPop);
        return () => window.removeEventListener('popstate', onPop);
    }, [read]);

    const go = useCallback((i: number) => {
        const next = Math.max(0, i);
        setIndex(next);
        if (typeof window === 'undefined') return;
        const url = new URL(window.location.href);
        if (next === 0) url.searchParams.delete('page');
        else url.searchParams.set('page', String(next + 1));
        // replaceState: 翻页不该在浏览器历史里堆一堆条目
        window.history.replaceState(window.history.state, '', url.toString());
    }, []);

    return [index, go];
}