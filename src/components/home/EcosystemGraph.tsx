import React, { useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';

import {
    type EdgeKind,
    type EcosystemProject,
    type LaneId,
    edgeKinds,
    edges,
    graphProjects,
    lanes,
    layers,
} from '@site/data/ecosystem';

import styles from './home.module.css';

type Box = { x: number; y: number; w: number; h: number };

type Props = {
    selectedId: string | null;
    onSelect: (id: string) => void;
};

const kindClass: Record<EdgeKind, string> = {
    depends: styles.edgeDepends,
    submodule: styles.edgeSubmodule,
    derived: styles.edgeDerived,
    related: styles.edgeRelated,
};

const swatchClass: Record<EdgeKind, string> = {
    depends: styles.swatchDepends,
    submodule: styles.swatchSubmodule,
    derived: styles.swatchDerived,
    related: styles.swatchRelated,
};

function curve(from: Box, to: Box): string {
    const x1 = from.x + from.w / 2;
    const y1c = from.y + from.h / 2;
    const x2 = to.x + to.w / 2;
    const y2c = to.y + to.h / 2;
    if (Math.abs(y1c - y2c) < 28) {
        const left = x1 < x2;
        const ax = left ? from.x + from.w : from.x;
        const bx = left ? to.x : to.x + to.w;
        const mid = (ax + bx) / 2;
        return `M ${ax} ${y1c} C ${mid} ${y1c}, ${mid} ${y2c}, ${bx} ${y2c}`;
    }
    const down = y2c >= y1c;
    const ay = down ? from.y + from.h : from.y;
    const by = down ? to.y : to.y + to.h;
    const mid = (ay + by) / 2;
    return `M ${x1} ${ay} C ${x1} ${mid}, ${x2} ${mid}, ${x2} ${by}`;
}

export default function EcosystemGraph({ selectedId, onSelect }: Props): React.ReactElement {
    const [lane, setLane] = useState<LaneId | 'all'>('all');
    const [markerBase, setMarkerBase] = useState('');
    const stageRef = useRef<HTMLDivElement>(null);
    const nodes = useRef(new Map<string, HTMLButtonElement>());
    const [boxes, setBoxes] = useState<Record<string, Box>>({});

    const byLayer = useMemo(() => {
        const grouped = new Map<string, EcosystemProject[]>();
        for (const project of graphProjects) {
            const list = grouped.get(project.layer) ?? [];
            list.push(project);
            grouped.set(project.layer, list);
        }
        return grouped;
    }, []);

    useEffect(() => {
        setMarkerBase(`${window.location.pathname}${window.location.search}`);
    }, []);

    useLayoutEffect(() => {
        const stage = stageRef.current;
        if (!stage) return undefined;

        const measure = () => {
            const origin = stage.getBoundingClientRect();
            const next: Record<string, Box> = {};
            nodes.current.forEach((el, id) => {
                const rect = el.getBoundingClientRect();
                next[id] = {
                    x: rect.left - origin.left,
                    y: rect.top - origin.top,
                    w: rect.width,
                    h: rect.height,
                };
            });
            setBoxes(next);
        };

        measure();
        const observer = new ResizeObserver(measure);
        observer.observe(stage);
        return () => observer.disconnect();
    }, [lane]);

    const visualLayers = [...layers].reverse();

    return (
        <div className={styles.graphShell}>
            <div className={styles.legend}>
                {(Object.keys(edgeKinds) as EdgeKind[]).map((kind) => (
                    <span key={kind} className={styles.legendItem} title={edgeKinds[kind].hint}>
                        <i className={`${styles.swatch} ${swatchClass[kind]}`} />
                        {edgeKinds[kind].label}
                    </span>
                ))}
                <span className={styles.legendItem} title="仓库之间没有直接引用, 按用途串起来">
                    <i className={`${styles.swatch} ${styles.swatchInferred}`} />
                    推断, 更淡的虚线
                </span>
            </div>
            <div className={styles.filters} role="toolbar" aria-label="按产品线筛选">
                <button
                    type="button"
                    className={lane === 'all' ? styles.filterOn : styles.filter}
                    aria-pressed={lane === 'all'}
                    onClick={() => setLane('all')}
                >
                    全部
                </button>
                {lanes.map((item) => (
                    <button
                        key={item.id}
                        type="button"
                        className={lane === item.id ? styles.filterOn : styles.filter}
                        aria-pressed={lane === item.id}
                        onClick={() => setLane(item.id)}
                    >
                        {item.label}
                    </button>
                ))}
            </div>
            <div className={styles.scroller}>
                <div className={styles.stage} ref={stageRef}>
                    <svg className={styles.edges} aria-hidden="true">
                        <defs>
                            {(Object.keys(edgeKinds) as EdgeKind[]).map((kind) => (
                                <marker
                                    key={kind}
                                    id={`arrow-${kind}`}
                                    viewBox="0 0 10 10"
                                    refX="8"
                                    refY="5"
                                    markerWidth="7"
                                    markerHeight="7"
                                    orient="auto-start-reverse"
                                >
                                    <path d="M 0 1.2 L 8 5 L 0 8.8 z" fill="context-stroke" />
                                </marker>
                            ))}
                        </defs>
                        {edges.map((edge) => {
                            const from = boxes[edge.from];
                            const to = boxes[edge.to];
                            if (!from || !to) return null;
                            const fromProject = graphProjects.find((item) => item.id === edge.from);
                            const toProject = graphProjects.find((item) => item.id === edge.to);
                            const involved = selectedId === edge.from || selectedId === edge.to;
                            const laneHit = lane === 'all'
                                || fromProject?.lane === lane
                                || toProject?.lane === lane;
                            const dim = Boolean(selectedId) ? !involved : !laneHit;
                            const classes = [
                                styles.edge,
                                kindClass[edge.kind],
                                edge.inferred ? styles.edgeInferred : '',
                                involved ? styles.edgeHot : '',
                                dim ? styles.edgeDim : '',
                            ].filter(Boolean).join(' ');
                            return (
                                <path
                                    key={`${edge.from}-${edge.to}-${edge.kind}`}
                                    d={curve(from, to)}
                                    className={classes}
                                    markerEnd={markerBase ? `url(${markerBase}#arrow-${edge.kind})` : undefined}
                                >
                                    <title>{edge.evidence}</title>
                                </path>
                            );
                        })}
                    </svg>
                    <div className={styles.laneHead}>
                        <span />
                        {lanes.map((item) => (
                            <span key={item.id}>{item.label}</span>
                        ))}
                    </div>
                    {visualLayers.map((layer, index) => (
                        <div
                            key={layer.id}
                            className={`${styles.band} ${index % 2 === 0 ? styles.bandAlt : ''}`}
                        >
                            <div>
                                <div className={styles.bandLabel}>{layer.label}</div>
                                <span className={styles.bandHint}>{layer.hint}</span>
                            </div>
                            {lanes.map((laneItem) => {
                                const cell = (byLayer.get(layer.id) ?? [])
                                    .filter((project) => project.lane === laneItem.id)
                                    .sort((a, b) => a.name.localeCompare(b.name));
                                return (
                                    <div key={laneItem.id} className={styles.cell}>
                                        {cell.map((project) => {
                                            const dim = lane !== 'all' && project.lane !== lane;
                                            const on = selectedId === project.id;
                                            const cls = on ? styles.nodeOn : dim ? styles.nodeDim : styles.node;
                                            return (
                                                <button
                                                    key={project.id}
                                                    type="button"
                                                    className={cls}
                                                    ref={(el) => {
                                                        if (el) nodes.current.set(project.id, el);
                                                        else nodes.current.delete(project.id);
                                                    }}
                                                    aria-pressed={on}
                                                    onClick={() => onSelect(project.id)}
                                                >
                                                    <span className={styles.nodeName}>{project.name}</span>
                                                    <span className={styles.nodeMeta}>
                                                        {project.stack.slice(0, 2).join(' · ') || project.pushedAt}
                                                    </span>
                                                </button>
                                            );
                                        })}
                                    </div>
                                );
                            })}
                        </div>
                    ))}
                </div>
            </div>
            <p className={styles.scrollHint}>窄屏可以左右滑动这张图.</p>
        </div>
    );
}
