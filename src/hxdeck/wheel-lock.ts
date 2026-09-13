/**
 * 滚轮事件的"已被消费"标记.
 *
 * 背景 (一个真实踩过的坑):
 *   Diagram 用 React 的 onWheel 做缩放. React 事件是**合成事件**, 挂在 root 上委托处理;
 *   而 Deck 用的是原生 `addEventListener('wheel', ...)`. 原生监听在冒泡链上先于 React 委托,
 *   因此 Diagram 里调 stopPropagation() **拦不住 Deck** —— 结果是:
 *     · 图缩放到极限后, 滚轮仍被 Deck 当成翻页信号 → 整屏被翻走
 *
 * 解决: 内层控件在**原生**监听里先打标记, 外层据此跳过.
 * 用 WeakSet 而非事件属性, 避免污染用户事件对象.
 */
const consumed = new WeakSet<Event>();

export function markWheelConsumed(e: Event): void {
    consumed.add(e);
}

export function isWheelConsumed(e: Event): boolean {
    return consumed.has(e);
}

/**
 * 判断某个可滚动容器是否真的还能往该方向滚.
 * 用于"缩放到极限/滚到边界"时把信号交还给外层, 或就地阻断.
 */
export function canScrollFurther(el: HTMLElement | null, deltaY: number): boolean {
    if (!el) return false;
    const max = el.scrollHeight - el.clientHeight;
    if (max <= 0) return false;
    if (deltaY > 0) return el.scrollTop < max - 1;
    return el.scrollTop > 1;
}
