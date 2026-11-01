import React, { useLayoutEffect, useMemo, useRef, useState } from 'react';

import {
    type EcosystemProject,
    type EdgeKind,
    type LayerId,
    edgeKinds,
    edges,
    graphProjects,
    lanes,
    layers,
    projectById,
    relationsOf,
} from '@site/data/ecosystem';

import Tooltip from '../common/Tooltip';
import styles from './graph.module.css';

type Box = { x: number; y: number; w: number; h: number };

type Props = {
    selectedId: string | null;
    onSelect: (id: string) => void;
};

const KIND_ORDER = Object.keys(edgeKinds) as EdgeKind[];

/**
 * 每层的列宽按该层节点最多的那一格分配: 应用层一格最多 10 个, 门户层最多 1 个,
 * 等宽分会让应用层挤成 2 列、门户层大片空白, 行就被撑高.
 */
const LAYER_WEIGHT: Record<LayerId, number> = {
    source: 1.6,
    wheel: 0.95,
    platform: 0.9,
    app: 1.45,
    portal: 0.6,
};

function curve (from: Box, to: Box): string {
    const fx = from.x + from.w / 2;
    const fy = from.y + from.h / 2;
    const tx = to.x + to.w / 2;
    const ty = to.y + to.h / 2;
    // 同一行: 从侧边出, 弧线往上拱一点, 不压住同行的节点
    if (Math.abs(fy - ty) < 6) {
        const right = fx < tx;
        const ax = right ? from.x + from.w : from.x;
        const bx = right ? to.x : to.x + to.w;
        const lift = Math.min(28, Math.abs(bx - ax) / 3);
        return `M ${ax} ${fy} C ${ax + (right ? lift : -lift)} ${fy - lift}, ${bx - (right ? lift : -lift)} ${ty - lift}, ${bx} ${ty}`;
    }
    // 跨行: 从上下边出, 竖向 S 形
    const down = ty > fy;
    const ay = down ? from.y + from.h : from.y;
    const by = down ? to.y : to.y + to.h;
    const mid = (ay + by) / 2;
    return `M ${fx} ${ay} C ${fx} ${mid}, ${tx} ${mid}, ${tx} ${by}`;
}

function relationText (direction: 'out' | 'in', kind: EdgeKind, other: string): string {
    if (kind === 'depends') return direction === 'out' ? `依赖 ${other}` : `被 ${other} 依赖`;
    if (kind === 'submodule') return direction === 'out' ? `聚合 ${other}` : `被 ${other} 收为子模块`;
    if (kind === 'derived') return direction === 'out' ? `由 ${other} 演化而来` : `演化出 ${other}`;
    return `与 ${other} 功能相关`;
}

/** 节点的悬浮卡片: 简介、技术栈、最近推送, 以及前三条关系和它们的依据 (原来连线上的 SVG <title>) */
function NodeCard ({ project }: { project: EcosystemProject }): React.ReactElement {
    const rels = relationsOf(project.id);
    const layer = layers.find((l) => l.id === project.layer);
    const lane = lanes.find((l) => l.id === project.lane);
    return (
        <div className={styles.card}>
            <div className={styles.cardHead}>
                <strong>{project.name}</strong>
                <span>{lane?.label} · {layer?.label}</span>
            </div>
            <p>{project.summary}</p>
            {project.stack.length > 0 && <div className={styles.cardStack}>{project.stack.slice(0, 4).join(' · ')}</div>}
            {rels.length > 0 && (
                <ul className={styles.cardRels}>
                    {rels.slice(0, 3).map((r) => (
                        <li key={`${r.direction}-${r.kind}-${r.otherId}`} data-kind={r.kind}>
                            <b>{relationText(r.direction, r.kind, projectById[r.otherId]?.name ?? r.otherId)}{r.inferred ? ' · 推断' : ''}</b>
                            <small>{r.evidence}</small>
                        </li>
                    ))}
                    {rels.length > 3 && <li className={styles.cardMore}>还有 {rels.length - 3} 条, 点开看全部</li>}
                </ul>
            )}
            <div className={styles.cardFoot}>最近推送 {project.pushedAt} · 点击看详情</div>
        </div>
    );
}

/**
 * 项目关系图 (第三轮重做). 行 = 产品线 (7 行), 列 = 层级 (源头 -> 门户 5 列), 节点是一行高 24px 的胶囊,
 * 同一格里横向换行. 旧版是"层级做行、产品线做列", 7 列挤在约 760px 的右栏里, 每格只放得下一个节点, 只能往下堆, 所以又窄又高.
 * 现在整张图放在满宽的浮层里 (最宽 1320px), 列宽按每层最多节点数加权, 胶囊最宽 168px (超长名字省略, 全名在悬浮卡片里), 1440x900 下整张图约 560px 高.
 * 连线默认是淡色细线; 悬停 / 聚焦节点时只点亮与它相连的线并让线流动, 其余节点变暗.
 * 悬停内容用统一的 Tooltip 卡片, 不再用原生 title 与 SVG <title>.
 * 首页关系图的决策见同目录 HomePage.tsx 顶部注释
 */
export default function EcosystemGraph ({ selectedId, onSelect }: Props): React.ReactElement {
    const [hoverId, setHoverId] = useState<string | null>(null);
    const [hiddenKinds, setHiddenKinds] = useState<Set<EdgeKind>>(new Set());
    const focusId = hoverId ?? selectedId;
    const stageRef = useRef<HTMLDivElement>(null);
    const nodes = useRef(new Map<string, HTMLButtonElement>());
    const [boxes, setBoxes] = useState<Record<string, Box>>({});
    const [size, setSize] = useState({ w: 0, h: 0 });

    const cells = useMemo(() => {
        const map = new Map<string, EcosystemProject[]>();
        for (const p of graphProjects) {
            const key = `${p.lane}|${p.layer}`;
            const list = map.get(key) ?? [];
            list.push(p);
            map.set(key, list);
        }
        for (const list of map.values()) list.sort((a, b) => b.pushedAt.localeCompare(a.pushedAt));
        return map;
    }, []);

    const neighbors = useMemo(() => {
        if (!focusId) return null;
        const set = new Set<string>([focusId]);
        for (const e of edges) {
            if (e.from === focusId) set.add(e.to);
            if (e.to === focusId) set.add(e.from);
        }
        return set;
    }, [focusId]);

    useLayoutEffect(() => {
        const stage = stageRef.current;
        if (!stage) return undefined;
        const measure = () => {
            const origin = stage.getBoundingClientRect();
            const next: Record<string, Box> = {};
            nodes.current.forEach((el, id) => {
                const r = el.getBoundingClientRect();
                next[id] = { x: r.left - origin.left, y: r.top - origin.top, w: r.width, h: r.height };
            });
            setBoxes(next);
            setSize({ w: stage.scrollWidth, h: stage.scrollHeight });
        };
        measure();
        const ro = new ResizeObserver(measure);
        ro.observe(stage);
        return () => ro.disconnect();
    }, []);

    const columns = `112px ${layers.map((l) => `minmax(0, ${LAYER_WEIGHT[l.id]}fr)`).join(' ')}`;

    const toggleKind = (kind: EdgeKind) => {
        setHiddenKinds((prev) => {
            const next = new Set(prev);
            if (next.has(kind)) next.delete(kind);
            else next.add(kind);
            return next;
        });
    };

    return (
        <div className={styles.graph}>
            <div className={styles.legend} role="group" aria-label="连线类型, 点击隐藏或显示">
                {KIND_ORDER.map((kind) => (
                    <Tooltip key={kind} width="auto" content={<span className={styles.hint}>{edgeKinds[kind].hint}</span>}>
                        <button
                            type="button"
                            className={styles.kind}
                            data-kind={kind}
                            aria-pressed={!hiddenKinds.has(kind)}
                            onClick={() => toggleKind(kind)}
                        >
                            <i />
                            {edgeKinds[kind].label}
                        </button>
                    </Tooltip>
                ))}
                <span className={styles.legendNote}>虚线 = 推断 (仓库间没有直接引用, 按用途串起来)</span>
            </div>

            <div className={styles.scroller}>
                <div className={styles.stage} ref={stageRef} style={{ gridTemplateColumns: columns }} data-focus={focusId ? 'true' : undefined}>
                    <svg className={styles.edges} width={size.w} height={size.h} aria-hidden="true">
                        {edges.map((edge) => {
                            const from = boxes[edge.from];
                            const to = boxes[edge.to];
                            if (!from || !to || hiddenKinds.has(edge.kind)) return null;
                            const hot = focusId === edge.from || focusId === edge.to;
                            return (
                                <path
                                    key={`${edge.from}-${edge.to}-${edge.kind}`}
                                    d={curve(from, to)}
                                    className={styles.edge}
                                    data-kind={edge.kind}
                                    data-inferred={edge.inferred ? 'true' : undefined}
                                    data-hot={hot ? 'true' : focusId ? 'dim' : undefined}
                                />
                            );
                        })}
                    </svg>

                    <div className={styles.corner} />
                    {layers.map((layer) => (
                        <div key={layer.id} className={styles.colHead}>
                            <b>{layer.label}</b>
                            <span>{layer.hint}</span>
                        </div>
                    ))}

                    {lanes.map((lane) => (
                        <React.Fragment key={lane.id}>
                            <div className={styles.rowHead}>{lane.label}</div>
                            {layers.map((layer) => (
                                <div key={layer.id} className={styles.cell}>
                                    {(cells.get(`${lane.id}|${layer.id}`) ?? []).map((p) => {
                                        const state = selectedId === p.id ? 'on' : neighbors ? (neighbors.has(p.id) ? 'near' : 'dim') : undefined;
                                        return (
                                            <Tooltip key={p.id} width={280} content={<NodeCard project={p} />}>
                                                <button
                                                    type="button"
                                                    className={styles.node}
                                                    data-state={state}
                                                    data-featured={p.featured ? 'true' : undefined}
                                                    aria-pressed={selectedId === p.id}
                                                    ref={(el) => {
                                                        if (el) nodes.current.set(p.id, el);
                                                        else nodes.current.delete(p.id);
                                                    }}
                                                    onMouseEnter={() => setHoverId(p.id)}
                                                    onMouseLeave={() => setHoverId(null)}
                                                    onFocus={() => setHoverId(p.id)}
                                                    onBlur={() => setHoverId(null)}
                                                    onClick={() => onSelect(p.id)}
                                                >
                                                    {p.name}
                                                </button>
                                            </Tooltip>
                                        );
                                    })}
                                </div>
                            ))}
                        </React.Fragment>
                    ))}
                </div>
            </div>
        </div>
    );
}
