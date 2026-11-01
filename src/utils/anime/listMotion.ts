import React from "react";
import { flushSync } from "react-dom";

/**
 * 番剧列表筛选切换的过渡. 不引 framer-motion, 两条路:
 *   1. 支持 View Transitions API 时 (Chrome/Edge 111+, Safari 18+, Firefox 144+): runListTransition 包住一次筛选更新,
 *      视口附近的卡片临时挂 view-transition-name, 留下的平滑移到新位置, 离开的淡出, 新来的淡入; 其余区域整体交叉淡化.
 *   2. 不支持, 或这次变化没走 runListTransition (浏览器后退 / 前进): useListFlip 用 Web Animations 做 FLIP 重排 + 新项淡入,
 *      离开的项直接移除 (无退场).
 * prefers-reduced-motion: reduce 时两条都不跑, 硬切. CSS 那边也同时关掉 (AnimeIndexPage.module.css).
 * 只处理视口上下各一屏内的卡片, 几百部番时不会给每张卡都做快照.
 * .agents/notes/implemented/feature/2026-11-01-anime-filter-motion.md
 */
const ITEM_SELECTOR = "[data-anime-id]";
const VT_FLAG = "animeVt";
const DURATION = 260;
const EASING = "cubic-bezier(0.2, 0, 0, 1)";

export function prefersReducedMotion (): boolean {
    return typeof window !== "undefined"
        && typeof window.matchMedia === "function"
        && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
}

function nearViewport (r: DOMRect): boolean {
    const h = window.innerHeight;
    return r.bottom > -h && r.top < h * 2;
}

/** 给视口附近的卡片挂 view-transition-name, 名字按番剧 id, 新旧两次快照里同一部番是同一个名字 */
function nameItems (): void {
    document.querySelectorAll<HTMLElement>(ITEM_SELECTOR).forEach((el) => {
        el.style.viewTransitionName = nearViewport(el.getBoundingClientRect()) ? `anime-${el.dataset.animeId}` : "";
    });
}

function clearNames (): void {
    document.querySelectorAll<HTMLElement>(ITEM_SELECTOR).forEach((el) => { el.style.viewTransitionName = ""; });
}

type StartViewTransition = (cb: () => void) => { finished: Promise<unknown> };

/**
 * 用 View Transition 执行一次会改变列表的更新 (这里是 history.push / replace).
 * 不支持或要求减弱动效时直接执行. 返回值: 这次是否走了 View Transition.
 */
export function runListTransition (update: () => void): boolean {
    const start = (typeof document !== "undefined"
        ? (document as Document & { startViewTransition?: StartViewTransition }).startViewTransition
        : undefined);
    if (!start || prefersReducedMotion()) {
        update();
        return false;
    }
    const root = document.documentElement;
    root.dataset[VT_FLAG] = "1";
    nameItems();
    const t = start.call(document, () => {
        flushSync(update);
        nameItems();
    });
    t.finished.finally(() => {
        clearNames();
        delete root.dataset[VT_FLAG];
    });
    return true;
}

function isTransitioning (): boolean {
    return typeof document !== "undefined" && document.documentElement.dataset[VT_FLAG] === "1";
}

/**
 * View Transition 之外的降级: 每次 key 变化后对容器里的卡片做 FLIP.
 * 记录上一次布局的文档坐标, 这次布局后算位移, 用 transform 反推回去再动画回 0; 上次不在的卡片淡入.
 * 首次出现 (骨架 -> 列表) 不动画.
 */
export function useListFlip (container: React.RefObject<HTMLElement | null>, key: unknown): void {
    const prev = React.useRef<Map<string, { x: number; y: number }> | null>(null);

    React.useLayoutEffect(() => {
        const box = container.current;
        if (!box) return;
        const now = new Map<string, { x: number; y: number }>();
        const els: HTMLElement[] = [];
        box.querySelectorAll<HTMLElement>(ITEM_SELECTOR).forEach((el) => {
            const r = el.getBoundingClientRect();
            if (!nearViewport(r)) return;
            const id = el.dataset.animeId ?? "";
            now.set(id, { x: r.left + window.scrollX, y: r.top + window.scrollY });
            els.push(el);
        });
        const before = prev.current;
        prev.current = now;
        if (!before || before.size === 0 || isTransitioning() || prefersReducedMotion()) return;
        if (typeof els[0]?.animate !== "function") return;

        for (const el of els) {
            const id = el.dataset.animeId ?? "";
            const to = now.get(id);
            const from = before.get(id);
            if (!to) continue;
            if (from) {
                const dx = from.x - to.x;
                const dy = from.y - to.y;
                if (Math.abs(dx) < 1 && Math.abs(dy) < 1) continue;
                el.animate(
                    [{ transform: `translate(${dx}px, ${dy}px)` }, { transform: "translate(0, 0)" }],
                    { duration: DURATION, easing: EASING },
                );
            } else {
                el.animate(
                    [{ opacity: 0, transform: "translateY(8px) scale(0.98)" }, { opacity: 1, transform: "none" }],
                    { duration: DURATION, easing: EASING, fill: "backwards" },
                );
            }
        }
    }, [container, key]);
}
