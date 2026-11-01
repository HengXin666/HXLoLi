import React, { useMemo, useState, useCallback, useRef, useEffect } from 'react';
import type { ForceGraphMethods, NodeObject, LinkObject } from 'react-force-graph-2d';
import { useActorMap, useAnimeRecords } from "@site/src/utils/anime/animeStore";
import { ANiMeRecord } from "@site/src/utils/anime/types";
import BrowserOnly from '@docusaurus/BrowserOnly';
import {
    buildFocusGraph,
    buildOverviewGraph,
    buildSeriesIndex,
    type GraphLinkType,
    type GraphNodeData,
    type GraphNodeType,
} from './buildGraph';
import styles from './graph.module.css';

// ================= 1. 类型定义 =================

type NodeType = GraphNodeType;

/** 图库会在节点对象上挂 x / y / vx / vy 等运行时字段 */
type GraphNode = NodeObject & GraphNodeData;

interface GraphLink extends LinkObject {
    source: string | GraphNode;
    target: string | GraphNode;
    type: GraphLinkType;
    label?: string;
    weight?: number;
}

interface GraphData {
    nodes: GraphNode[];
    links: GraphLink[];
}

export type GraphLayer = 'overview' | 'focus';

// ================= 2. 样式常量 (Dark Theme) =================

export const THEME = {
    // 与站点暗色底 (#121212) 一致; 强调色取站点主色 #ff88ff, 只用于选中 / 高亮描边
    background: '#121212',
    panelBg: 'rgba(24, 24, 24, 0.88)',
    textMain: 'rgba(255, 255, 255, 0.9)',
    textSub: 'rgba(255, 255, 255, 0.55)',
    border: 'rgba(255, 255, 255, 0.1)',
    accent: '#ff88ff',
    colors: {
        // 三类节点用低饱和色区分: 番剧中性米白, 角色灰青, 声优灰金
        Anime: '#d9d4c7',
        Character: '#7fa6a1',
        Actor: '#b9a37a',
        Sequel: '#d9d4c7',
        Default: '#8a8a8a',
        TextBg: 'rgba(14, 14, 16, 0.82)',
        Text: '#ece9e2',
        Dim: '#3a3a3a',
        Ghost: '#5c5a55',
    }
};

/** 边按类型区分: 颜色 / 线宽 / 线型. 续作实线最醒目, 包含细实线, 配音与共享声优虚线, 其它作品点线 */
export const EDGE_STYLE: Record<GraphLinkType, { color: string; width: number; dash: number[] | null; label: string }> = {
    Sequel: { color: 'rgba(217, 212, 199, 0.75)', width: 2, dash: null, label: '续作 / 同系列' },
    SharedCV: { color: 'rgba(185, 163, 122, 0.5)', width: 1, dash: [4, 3], label: '共享声优 (越粗越多)' },
    Include: { color: 'rgba(127, 166, 161, 0.4)', width: 1, dash: null, label: '番剧 — 角色' },
    VoicedBy: { color: 'rgba(185, 163, 122, 0.45)', width: 1, dash: [3, 3], label: '角色 — 声优' },
    Appears: { color: 'rgba(255, 255, 255, 0.18)', width: 1, dash: [1, 3], label: '声优的其它作品' },
    Related: { color: 'rgba(255, 255, 255, 0.3)', width: 1, dash: [1, 3], label: '相关' },
};

const LAYER_EDGES: Record<GraphLayer, GraphLinkType[]> = {
    overview: ['Sequel', 'SharedCV'],
    focus: ['Sequel', 'Include', 'VoicedBy', 'Appears'],
};

export const NODE_LEGEND: { type: NodeType; label: string }[] = [
    { type: 'Anime', label: '番剧' },
    { type: 'Character', label: '角色' },
    { type: 'Actor', label: '声优' },
];

function linkEndId (end: string | GraphNode | undefined): string {
    if (end && typeof end === 'object') return end.id;
    return end ?? '';
}

function linkWidthOf (link: GraphLink): number {
    const base = EDGE_STYLE[link.type].width;
    if (link.type === 'SharedCV' && link.weight) return Math.min(base + (link.weight - 1) * 0.6, 4);
    return base;
}

function roundRectPath (ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number) {
    const rr = Math.min(r, w / 2, h / 2);
    ctx.beginPath();
    ctx.moveTo(x + rr, y);
    ctx.arcTo(x + w, y, x + w, y + h, rr);
    ctx.arcTo(x + w, y + h, x, y + h, rr);
    ctx.arcTo(x, y + h, x, y, rr);
    ctx.arcTo(x, y, x + w, y, rr);
    ctx.closePath();
}

// 图片缓存
const imgCache = new Map<string, HTMLImageElement>();
/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
function getImage (url: string): HTMLImageElement {
    if (imgCache.has(url)) return imgCache.get(url)!;
    const img = new Image();
    img.src = url;
    img.crossOrigin = 'Anonymous';
    imgCache.set(url, img);
    return img;
}

// ================= 3. 控制器组件 =================

interface GraphControllerProps {
    layer: GraphLayer;
    hiddenRelations: string[];
    setHiddenRelations: (val: string[]) => void;
    preferCn: boolean;
    setPreferCn: (val: boolean) => void;
    showImages: boolean;
    setShowImages: (val: boolean) => void;
    minShared: number;
    setMinShared: (val: number) => void;

    chargeStrength: number;
    setChargeStrength: (val: number) => void;
    linkDistance: number;
    setLinkDistance: (val: number) => void;
}

const RELATION_OPTIONS = ['主角', '配角', '客串', '闲角', '旁白'];

export const AnimeGraphController: React.FC<GraphControllerProps> = ({
    layer,
    hiddenRelations, setHiddenRelations,
    preferCn, setPreferCn,
    showImages, setShowImages,
    minShared, setMinShared,
    chargeStrength, setChargeStrength,
    linkDistance, setLinkDistance
}) => {
    const [isCollapsed, setIsCollapsed] = useState(false);

    // 勾选 = 显示该角色定位 (URL 里仍存被隐藏的那些, 兼容旧链接)
    const toggleRelation = (rel: string) => {
        if (hiddenRelations.includes(rel)) {
            setHiddenRelations(hiddenRelations.filter(r => r !== rel));
        } else {
            setHiddenRelations([...hiddenRelations, rel]);
        }
    };

    return (
        <div className={isCollapsed ? styles.controllerCollapsed : styles.controller}>
            <button
                type="button"
                className={styles.controllerToggle}
                onClick={() => setIsCollapsed(!isCollapsed)}
                aria-expanded={!isCollapsed}
                title={isCollapsed ? "展开设置" : "收起设置"}
            >
                {isCollapsed ? '设置' : '»'}
            </button>

            <div className={styles.panel}>
                <div>
                    <span className={styles.sectionTitle}>
                        {layer === 'overview' ? '共享声优统计的角色' : '显示的角色'}
                    </span>
                    <div className={styles.chips}>
                        {RELATION_OPTIONS.map(rel => {
                            const shown = !hiddenRelations.includes(rel);
                            return (
                                <button
                                    key={rel}
                                    type="button"
                                    className={shown ? styles.chipOn : styles.chip}
                                    aria-pressed={shown}
                                    onClick={() => toggleRelation(rel)}
                                >
                                    {rel}
                                </button>
                            );
                        })}
                    </div>
                </div>

                {layer === 'overview' && (
                    <div>
                        <div className={styles.rangeHead}>
                            <span>共享声优至少</span>
                            <span className={styles.tabular}>{minShared} 位</span>
                        </div>
                        <input
                            className={styles.range}
                            type="range"
                            min="1"
                            max="6"
                            step="1"
                            value={minShared}
                            onChange={e => setMinShared(Number(e.target.value))}
                        />
                    </div>
                )}

                <div className={styles.toggles}>
                    <label>
                        <input type="checkbox" checked={preferCn} onChange={e => setPreferCn(e.target.checked)} />
                        中文优先
                    </label>
                    <label>
                        <input type="checkbox" checked={showImages} onChange={e => setShowImages(e.target.checked)} />
                        显示头像
                    </label>
                </div>

                <details className={styles.advanced}>
                    <summary>物理参数</summary>
                    <div className={styles.rangeHead}>
                        <span>斥力</span>
                        <span className={styles.tabular}>{chargeStrength}</span>
                    </div>
                    <input
                        className={styles.range}
                        type="range"
                        min="-500"
                        max="-10"
                        step="10"
                        value={chargeStrength}
                        onChange={(e) => setChargeStrength(Number(e.target.value))}
                    />
                    <div className={styles.rangeHead}>
                        <span>间距</span>
                        <span className={styles.tabular}>{linkDistance}</span>
                    </div>
                    <input
                        className={styles.range}
                        type="range"
                        min="10"
                        max="200"
                        step="5"
                        value={linkDistance}
                        onChange={(e) => setLinkDistance(Number(e.target.value))}
                    />
                </details>
            </div>
        </div>
    );
};

// ================= 面包屑 =================

interface BreadcrumbProps {
    focusTitle?: string;
    onOverview: () => void;
}

export const GraphBreadcrumb: React.FC<BreadcrumbProps> = ({ focusTitle, onOverview }) => (
    <nav className={styles.breadcrumb} aria-label="关系图层级">
        {focusTitle ? (
            <>
                <button type="button" className={styles.crumbLink} onClick={onOverview}>全部番剧</button>
                <span className={styles.crumbSep} aria-hidden="true">›</span>
                <span className={styles.crumbCurrent} title={focusTitle}>{focusTitle}</span>
            </>
        ) : (
            <span className={styles.crumbCurrent}>全部番剧</span>
        )}
        <span className={styles.crumbHint}>
            {focusTitle ? '单击选中 · 双击其它作品切换 · Esc 返回' : '单击选中 · 双击番剧展开角色与声优'}
        </span>
    </nav>
);

// ================= 4. 主图表组件 =================

interface ContextMenuState {
    visible: boolean;
    x: number;
    y: number;
    node: GraphNode | null;
}

interface AnimeForceGraphProps {
    baseUrl: string;
    /** 有值 = 聚焦层 (以该番剧所在系列为中心), 无值 = 概览层 */
    targetAnimeId?: number;
    hiddenRelations?: string[];
    preferCn?: boolean;
    showImages?: boolean;
    chargeStrength?: number;
    linkDistance?: number;
    /** 概览层共享声优边的最小共享人数 */
    minShared?: number;
    /** 进入 / 切换聚焦层 (传 undefined 回到概览层) */
    onFocusAnime?: (animeId: number | undefined) => void;
}

const DOUBLE_CLICK_MS = 320;

/**
 * 番剧关系图重构: 概览层 / 聚焦层两层视图, 续作改用 relations, 物理参数首屏生效
 * .agents/notes/implemented/feature/2026-11-01-anime-graph-refactor.md
 */
export const AnimeForceGraph: React.FC<AnimeForceGraphProps> = ({
    baseUrl,
    targetAnimeId,
    hiddenRelations = [],
    preferCn = false,
    showImages = false,
    chargeStrength = -60,
    linkDistance = 60,
    minShared = 2,
    onFocusAnime,
}) => {
    // Hooks
    const records = useAnimeRecords(baseUrl);
    const actorMap = useActorMap(baseUrl);
    const fgRef = useRef<ForceGraphMethods | undefined>(undefined);
    // 图实例挂载后才有值; 用 state 承载, 物理参数 effect 才能在首次挂载时触发 (ref.current 变化不会触发渲染)
    const [fgInstance, setFgInstance] = useState<ForceGraphMethods | undefined>(undefined);
    const attachGraph = useCallback((instance: ForceGraphMethods | undefined | null) => {
        fgRef.current = instance ?? undefined;
        setFgInstance(instance ?? undefined);
    }, []);
    const containerRef = useRef<HTMLDivElement>(null);

    // State
    const [dimensions, setDimensions] = useState({ width: 0, height: 0 });
    const [hoverNode, setHoverNode] = useState<GraphNode | null>(null);
    const [selectedId, setSelectedId] = useState<string | null>(null);
    const [contextMenu, setContextMenu] = useState<ContextMenuState>({ visible: false, x: 0, y: 0, node: null });
    const lastClickRef = useRef<{ id: string; time: number } | null>(null);
    const needFitRef = useRef(true);
    /** 当前帧可见的图坐标范围, 由 onRenderFramePre 更新, 用于视口裁剪 */
    const viewRef = useRef<{ minX: number; maxX: number; minY: number; maxY: number } | null>(null);
    /** 上一版节点对象, 重建图数据时沿用坐标, 过滤切换时图不会整体炸开重排 */
    const prevNodesRef = useRef<Map<string, GraphNode>>(new Map());

    // ------------------- 初始化逻辑 -------------------

    useEffect(() => {
        if (!containerRef.current) return;
        const resizeObserver = new ResizeObserver((entries) => {
            for (let entry of entries) {
                setDimensions({
                    width: entry.contentRect.width,
                    height: entry.contentRect.height
                });
            }
        });
        resizeObserver.observe(containerRef.current);
        return () => resizeObserver.disconnect();
    }, []);

    // 响应物理引擎参数变化 (含首次挂载: fgInstance 从 undefined 变为实例时也会执行)
    useEffect(() => {
        if (!fgInstance) return;
        fgInstance.d3Force('charge')?.strength(chargeStrength);
        fgInstance.d3Force('link')?.distance(linkDistance);
        fgInstance.d3ReheatSimulation();
    }, [fgInstance, chargeStrength, linkDistance]);

    // ------------------- 数据处理逻辑 -------------------

    // 0. 系列 (续作) 索引: relations 优先, 名字前缀兜底, 只依赖 records
    const seriesIndex = useMemo(() => buildSeriesIndex(records), [records]);

    const layer: GraphLayer = targetAnimeId !== undefined && records.some(r => r.anime_data.id === targetAnimeId)
        ? 'focus' : 'overview';

    // 1. 构建图数据 (概览层 / 聚焦层)
    const graphData = useMemo<GraphData>(() => {
        const plain = layer === 'focus'
            ? buildFocusGraph(records, actorMap, seriesIndex, targetAnimeId!, hiddenRelations)
            : buildOverviewGraph(records, seriesIndex, hiddenRelations, minShared);
        const prev = prevNodesRef.current;
        const nodes: GraphNode[] = plain.nodes.map(n => {
            const old = prev.get(n.id);
            return old ? { ...n, x: old.x, y: old.y } : { ...n };
        });
        prevNodesRef.current = new Map(nodes.map(n => [n.id, n]));
        return { nodes, links: plain.links.map(l => ({ ...l })) };
    }, [layer, records, actorMap, seriesIndex, targetAnimeId, hiddenRelations, minShared]);

    // 换层或换中心后重新适配视图
    useEffect(() => {
        needFitRef.current = true;
        setSelectedId(null);
        setContextMenu(p => ({ ...p, visible: false }));
    }, [layer, targetAnimeId]);

    // 2. 邻接表
    const neighborMap = useMemo(() => {
        const map = new Map<string, Set<string>>();
        graphData.links.forEach(link => {
            const sId = linkEndId(link.source);
            const tId = linkEndId(link.target);
            if (!map.has(sId)) map.set(sId, new Set());
            if (!map.has(tId)) map.set(tId, new Set());
            map.get(sId)!.add(tId);
            map.get(tId)!.add(sId);
        });
        return map;
    }, [graphData]);

    // 3. 高亮集合: 选中优先于悬停, 只取一跳邻居
    const highlightNodes = useMemo(() => {
        const set = new Set<string>();
        const targetId = selectedId || hoverNode?.id;
        if (targetId) {
            set.add(targetId);
            neighborMap.get(targetId)?.forEach(n => set.add(n));
        }
        return set;
    }, [selectedId, hoverNode, neighborMap]);

    // ------------------- 交互回调 -------------------

    const focusAnime = useCallback((animeId: number | undefined) => {
        onFocusAnime?.(animeId);
    }, [onFocusAnime]);

    const handleNodeClick = useCallback((node: GraphNode) => {
        setContextMenu(p => ({ ...p, visible: false }));
        const now = Date.now();
        const last = lastClickRef.current;
        const isDouble = !!last && last.id === node.id && now - last.time < DOUBLE_CLICK_MS;
        lastClickRef.current = isDouble ? null : { id: node.id, time: now };

        if (isDouble) {
            // 双击番剧: 概览层进入聚焦层; 聚焦层里双击其它作品切换中心
            if (node.type === 'Anime' && node.rawAnimeId !== undefined
                && (layer === 'overview' || node.ghost)) {
                focusAnime(node.rawAnimeId);
            }
            return;
        }
        setSelectedId(prev => (prev === node.id ? null : node.id));
    }, [layer, focusAnime]);

    const handleBackgroundClick = useCallback(() => {
        setContextMenu(p => ({ ...p, visible: false }));
        setSelectedId(null);
    }, []);

    const handleNodeRightClick = useCallback((node: GraphNode, event: MouseEvent) => {
        setContextMenu({ visible: true, x: event.clientX, y: event.clientY, node });
    }, []);

    // Esc: 关菜单 > 取消选中 > 回概览层
    useEffect(() => {
        const onKey = (e: KeyboardEvent) => {
            if (e.key !== 'Escape') return;
            const el = e.target as HTMLElement | null;
            if (el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA')) return;
            if (contextMenu.visible) setContextMenu(p => ({ ...p, visible: false }));
            else if (selectedId) setSelectedId(null);
            else if (layer === 'focus') focusAnime(undefined);
        };
        window.addEventListener('keydown', onKey);
        return () => window.removeEventListener('keydown', onKey);
    }, [contextMenu.visible, selectedId, layer, focusAnime]);

    const zoomBy = (factor: number) => {
        const fg = fgRef.current;
        if (!fg) return;
        fg.zoom(fg.zoom() * factor, 300);
    };

    // ------------------- Canvas 渲染 -------------------

    const onRenderFramePre = useCallback((ctx: CanvasRenderingContext2D, globalScale: number) => {
        // 由当前变换反推可见的图坐标范围 (变换含设备像素比, 与 canvas 像素尺寸对应)
        const t = ctx.getTransform();
        if (!t.a) return;
        const margin = 80 / globalScale;
        const x0 = -t.e / t.a;
        const y0 = -t.f / t.d;
        viewRef.current = {
            minX: x0 - margin,
            maxX: x0 + ctx.canvas.width / t.a + margin,
            minY: y0 - margin,
            maxY: y0 + ctx.canvas.height / t.d + margin,
        };
    }, []);

    const nodeCanvasObject = useCallback((node: GraphNode, ctx: CanvasRenderingContext2D, globalScale: number) => {
        if (node.x === undefined || node.y === undefined) return;
        const view = viewRef.current;
        if (view && (node.x < view.minX || node.x > view.maxX || node.y < view.minY || node.y > view.maxY)) return;

        const label = preferCn ? (node.name_cn || node.name) : node.name;
        const isHover = hoverNode?.id === node.id;
        const isSelected = selectedId === node.id;
        const isHighlight = highlightNodes.has(node.id);
        const dimmed = highlightNodes.size > 0 && !isHighlight && !isHover;
        const baseColor = node.ghost ? THEME.colors.Ghost : (THEME.colors[node.type] || THEME.colors.Default);

        // 头像只在放大到能看清时画, 缩小时只画色块, 省绘制
        const img = (showImages && node.image_url && globalScale >= 1.2) ? getImage(node.image_url) : null;
        const hasImage = !!img && img.complete && img.naturalWidth > 0;

        ctx.save();
        ctx.globalAlpha = dimmed ? 0.18 : (node.ghost ? 0.6 : 1);
        const strokeColor = isHover || isSelected ? THEME.accent : 'rgba(255, 255, 255, 0.7)';

        let halfH: number;
        if (node.type === 'Anime') {
            // 番剧: 圆角矩形, 有封面时按封面比例
            const w = Math.sqrt(node.val) * 5;
            const h = hasImage ? w * (img!.naturalHeight / img!.naturalWidth) : w * 1.4;
            const x = node.x - w / 2;
            const y = node.y - h / 2;
            halfH = h / 2;
            roundRectPath(ctx, x, y, w, h, 3);
            ctx.fillStyle = dimmed ? THEME.colors.Dim : baseColor;
            ctx.fill();
            if (hasImage) {
                ctx.save();
                ctx.clip();
                ctx.drawImage(img!, x, y, w, h);
                ctx.restore();
            }
            if (isHover || isHighlight) {
                roundRectPath(ctx, x, y, w, h, 3);
                ctx.lineWidth = 2 / globalScale;
                ctx.strokeStyle = strokeColor;
                ctx.stroke();
            }
        } else {
            // 角色 / 声优: 圆形
            const r = Math.sqrt(node.val) * 2.6;
            halfH = r;
            ctx.beginPath();
            ctx.arc(node.x, node.y, r, 0, Math.PI * 2);
            ctx.fillStyle = dimmed ? THEME.colors.Dim : baseColor;
            ctx.fill();
            if (hasImage) {
                ctx.save();
                ctx.clip();
                const side = r * 2;
                const ratio = img!.naturalHeight / img!.naturalWidth;
                // 角色图多为竖图, 取上部 (脸) 对齐
                ctx.drawImage(img!, node.x - r, node.y - r, side, side * ratio);
                ctx.restore();
            }
            if (isHover || isHighlight) {
                ctx.beginPath();
                ctx.arc(node.x, node.y, r, 0, Math.PI * 2);
                ctx.lineWidth = 1.5 / globalScale;
                ctx.strokeStyle = strokeColor;
                ctx.stroke();
            }
        }

        // 标签: 番剧常显 (占位作品放大后才显示); 角色 / 声优在放大或被高亮时显示
        const showLabel = !dimmed && (
            isHover || isHighlight
            || (node.type === 'Anime' && (!node.ghost || globalScale >= 1.2))
            || globalScale >= 1.6
        );
        if (showLabel && label) {
            const fontSize = Math.max(11 / globalScale, 2.5);
            ctx.font = `${fontSize}px system-ui, -apple-system, "Segoe UI", sans-serif`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            const padX = fontSize * 0.4;
            const padY = fontSize * 0.25;
            const textW = ctx.measureText(label).width;
            const textY = node.y + halfH + fontSize * 0.5 + padY + 1 / globalScale;
            roundRectPath(ctx, node.x - textW / 2 - padX, textY - fontSize / 2 - padY, textW + padX * 2, fontSize + padY * 2, fontSize * 0.3);
            ctx.fillStyle = THEME.colors.TextBg;
            ctx.fill();
            ctx.fillStyle = THEME.colors.Text;
            ctx.fillText(label, node.x, textY);
        }
        ctx.restore();
    }, [preferCn, showImages, hoverNode, highlightNodes, selectedId]);

    const isActiveLink = useCallback((link: GraphLink) =>
        highlightNodes.has(linkEndId(link.source)) && highlightNodes.has(linkEndId(link.target)), [highlightNodes]);

    // ------------------- 组件返回 -------------------

    const menuNode = contextMenu.node;
    const menuLabel = menuNode ? (preferCn ? (menuNode.name_cn || menuNode.name) : menuNode.name) : '';
    const closeMenu = () => setContextMenu(p => ({ ...p, visible: false }));

    return (
        <div
            ref={containerRef}
            className={styles.canvasWrap}
            onContextMenu={e => e.preventDefault()}
        >
            {dimensions.width > 0 && (
                <BrowserOnly>
                    {() => {
                        const ForceGraph2D = require('react-force-graph-2d').default;
                        return <ForceGraph2D
                            ref={attachGraph}
                            width={dimensions.width}
                            height={dimensions.height}
                            backgroundColor={THEME.background}
                            graphData={graphData}
                            nodeLabel={() => ''} // 禁用默认 Label, 完全由 nodeCanvasObject 接管
                            nodeCanvasObject={nodeCanvasObject as any}
                            onRenderFramePre={onRenderFramePre}

                            linkColor={(link: GraphLink) => {
                                if (highlightNodes.size > 0 && !isActiveLink(link)) return 'rgba(255, 255, 255, 0.03)';
                                return EDGE_STYLE[link.type].color;
                            }}
                            linkWidth={(link: GraphLink) => {
                                const width = linkWidthOf(link);
                                return (highlightNodes.size > 0 && isActiveLink(link)) ? width + 1 : width;
                            }}
                            linkLineDash={(link: GraphLink) => EDGE_STYLE[link.type].dash}
                            linkLabel={(link: GraphLink) => link.label ?? ''}

                            onNodeHover={(node: GraphNode | null) => setHoverNode(node || null)}
                            onNodeClick={handleNodeClick}
                            onNodeRightClick={handleNodeRightClick}
                            onBackgroundClick={handleBackgroundClick}
                            onEngineStop={() => {
                                if (!needFitRef.current) return;
                                needFitRef.current = false;
                                fgRef.current?.zoomToFit(500, 60);
                            }}

                            d3VelocityDecay={0.35}
                            cooldownTicks={layer === 'overview' ? 200 : 150}
                        />;
                    }}
                </BrowserOnly>
            )}

            <div className={styles.bottomLeft}>
                <div className={styles.zoomControls}>
                    <button type="button" className={styles.zoomButton} onClick={() => zoomBy(1.4)} aria-label="放大">+</button>
                    <button type="button" className={styles.zoomButton} onClick={() => zoomBy(1 / 1.4)} aria-label="缩小">−</button>
                    <button type="button" className={styles.zoomButton} onClick={() => fgRef.current?.zoomToFit(400, 60)} aria-label="适应全部">⤢</button>
                </div>
                <GraphLegend edgeTypes={LAYER_EDGES[layer]} nodeTypes={layer === 'overview' ? ['Anime'] : undefined} />
            </div>

            {contextMenu.visible && menuNode && (
                <div className={styles.menu} style={{ top: contextMenu.y, left: contextMenu.x }} role="menu">
                    <div className={styles.menuHead}>「{menuLabel}」</div>
                    <MenuOption onClick={() => {
                        if (menuNode.x !== undefined && menuNode.y !== undefined) {
                            fgRef.current?.centerAt(menuNode.x, menuNode.y, 600);
                            fgRef.current?.zoom(3, 800);
                        }
                        closeMenu();
                    }}>
                        放大到这里
                    </MenuOption>
                    {menuNode.type === 'Anime' && menuNode.rawAnimeId !== undefined && (
                        layer === 'overview' || menuNode.ghost
                    ) && (
                        <MenuOption onClick={() => { focusAnime(menuNode.rawAnimeId); closeMenu(); }}>
                            展开角色与声优
                        </MenuOption>
                    )}
                    {menuNode.type === 'Anime' && (
                        <MenuOption onClick={() => {
                            window.open(`${baseUrl}anime/details?id=${menuNode.rawAnimeId}`, '_blank');
                            closeMenu();
                        }}>
                            打开番剧详情 ↗
                        </MenuOption>
                    )}
                </div>
            )}
        </div>
    );
};

// 图例: 节点类型 + 边类型 (颜色与线型和画布一致)
const GraphLegend: React.FC<{ edgeTypes: GraphLinkType[]; nodeTypes?: NodeType[] }> = ({ edgeTypes, nodeTypes }) => (
    <div className={styles.legend} aria-label="图例">
        {NODE_LEGEND.filter(item => !nodeTypes || nodeTypes.includes(item.type)).map(item => (
            <span key={item.type} className={styles.legendItem}>
                <i
                    className={item.type === 'Anime' ? styles.legendRect : styles.legendDot}
                    style={{ background: THEME.colors[item.type] }}
                />
                {item.label}
            </span>
        ))}
        {edgeTypes.map(type => {
            const style = EDGE_STYLE[type];
            return (
                <span key={type} className={styles.legendItem}>
                    <svg width="22" height="8" aria-hidden="true">
                        <line
                            x1="1" y1="4" x2="21" y2="4"
                            stroke={style.color}
                            strokeWidth={Math.max(style.width, 1.5)}
                            strokeDasharray={style.dash ? style.dash.join(' ') : undefined}
                        />
                    </svg>
                    {style.label}
                </span>
            );
        })}
    </div>
);

// 简单的菜单组件
const MenuOption: React.FC<{ onClick: () => void, children: React.ReactNode }> = ({ onClick, children }) => (
    <button type="button" role="menuitem" className={styles.menuItem} onClick={onClick}>
        {children}
    </button>
);

/** 聚焦层面包屑用的标题: 中心番剧名 */
export function focusTitleOf (records: readonly ANiMeRecord[], animeId: number | undefined, preferCn: boolean): string | undefined {
    if (animeId === undefined) return undefined;
    const anime = records.find(r => r.anime_data.id === animeId)?.anime_data;
    if (!anime) return undefined;
    return preferCn ? (anime.name_cn || anime.name) : anime.name;
}
