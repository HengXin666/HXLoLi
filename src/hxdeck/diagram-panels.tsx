import React from 'react';
import type { DiagramFacts } from './diagram-focus';

/**
 * 架构图的"语义面板"族 —— 图例/透镜/查找器/指南.
 *
 * 这些都是**关于这张图的元信息**, 不是图本身, 所以贴在图上层的浮层里,
 * 与工具条同属 __stage. 交互只改 SVG 上的语义属性, 视觉由 archify 自带 CSS 决定.
 */

export interface KindCount { kind: string; count: number }

/** 图例 / 语义透镜共用的类型条: 点选 = 透镜聚焦, 悬停 = 邻域预览 */
export function DiagramLens({
    kinds, active, onPick, onHover, onLeave,
}: {
    kinds: KindCount[];
    active: string | null;
    onPick: (kind: string) => void;
    onHover: (kind: string | null) => void;
    onLeave: () => void;
}): React.ReactElement | null {
    if (!kinds.length) return null;
    return (
        <div className="hxd-panel hxd-panel--lens" role="group" aria-label="语义透镜">
            <div className="hxd-panel__head">
                <span className="hxd-panel__eyebrow">语义透镜</span>
                <span className="hxd-panel__hint">按角色看这张图</span>
            </div>
            <div className="hxd-panel__kinds">
                {kinds.map((k) => (
                    <button
                        key={k.kind}
                        type="button"
                        className="hxd-kind"
                        data-kind={k.kind}
                        data-on={active === k.kind ? 'true' : 'false'}
                        onClick={() => onPick(k.kind)}
                        onMouseEnter={() => onHover(k.kind)}
                        onFocus={() => onHover(k.kind)}
                        onMouseLeave={onLeave}
                        onBlur={onLeave}
                        aria-pressed={active === k.kind}
                    >
                        <span className="hxd-kind__dot" aria-hidden="true" />
                        <span className="hxd-kind__name">{k.kind}</span>
                        <span className="hxd-kind__count">{k.count}</span>
                    </button>
                ))}
            </div>
        </div>
    );
}

/** 节点查找器: 按标签 / 子标签 / 稳定 ID 搜 */
export function DiagramFinder({
    query, onQuery, results, onGo, onClose,
}: {
    query: string;
    onQuery: (v: string) => void;
    results: { id: string; label: string; sublabel?: string; context?: string }[];
    onGo: (id: string) => void;
    onClose: () => void;
}): React.ReactElement {
    return (
        <div className="hxd-panel hxd-panel--finder" role="dialog" aria-label="查找节点">
            <div className="hxd-panel__head">
                <span className="hxd-panel__eyebrow">查找节点</span>
                <button type="button" className="hxd-panel__close" onClick={onClose} aria-label="关闭查找">×</button>
            </div>
            <input
                className="hxd-panel__input"
                value={query}
                autoFocus
                placeholder="标签 / 子标签 / 稳定 ID"
                onChange={(e) => onQuery(e.target.value)}
                onKeyDown={(e) => {
                    // 输入框里的按键不该被 Deck 拿去翻页
                    e.stopPropagation();
                    if (e.key === 'Escape') { e.preventDefault(); onClose(); }
                    if (e.key === 'Enter' && results[0]) { e.preventDefault(); onGo(results[0].id); }
                }}
            />
            <div className="hxd-panel__results">
                {query && !results.length ? <span className="hxd-panel__empty">没有匹配的节点</span> : null}
                {results.map((n) => (
                    <button key={n.id} type="button" className="hxd-panel__row" onClick={() => onGo(n.id)}>
                        <strong>{n.label}</strong>
                        {n.sublabel ? <small>{n.sublabel}</small> : null}
                        <code>{n.id}</code>
                    </button>
                ))}
            </div>
        </div>
    );
}

/** 路径探测面板: 显示选择进度与结果 */
export function DiagramRoute({
    facts, route, picking, onStart, onClear, onGo,
}: {
    facts: DiagramFacts;
    route: string[] | null;
    picking: 'source' | 'target' | null;
    onStart: () => void;
    onClear: () => void;
    onGo: (id: string) => void;
}): React.ReactElement | null {
    if (!picking && !route) return null;
    const name = (id: string) => facts.nodes.get(id)?.label || id;
    return (
        <div className="hxd-panel hxd-panel--route" role="status" aria-live="polite">
            <div className="hxd-panel__head">
                <span className="hxd-panel__eyebrow">路径探测</span>
                <button type="button" className="hxd-panel__close" onClick={onClear} aria-label="清除路径">×</button>
            </div>
            {picking === 'source' ? (
                <p className="hxd-panel__msg">点一个节点作为<b>起点</b>.</p>
            ) : null}
            {picking === 'target' ? (
                <p className="hxd-panel__msg">起点 <b>{name(route?.[0] || '')}</b> — 再点一个<b>终点</b>.</p>
            ) : null}
            {route && route.length > 1 ? (
                <ol className="hxd-panel__steps">
                    {route.map((id, i) => (
                        <li key={id}>
                            <button type="button" onClick={() => onGo(id)}>
                                <span className="hxd-panel__stepNo">{i + 1}</span>
                                {name(id)}
                            </button>
                        </li>
                    ))}
                </ol>
            ) : null}
            {!picking && route && route.length > 1 ? (
                <p className="hxd-panel__msg hxd-panel__msg--ok">
                    共 {route.length - 1} 跳. 这是作者写下的有向关系, 非推断.
                </p>
            ) : null}
        </div>
    );
}

/** 图表指南: 说明这张图能做什么 (对应 archify 的 Diagram Guide) */
