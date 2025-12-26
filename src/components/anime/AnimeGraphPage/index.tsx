import React, { useMemo, useState, useCallback, useRef, useEffect } from 'react';
import type { ForceGraphMethods, NodeObject, LinkObject } from 'react-force-graph-2d';
import { useActorMap, useAnimeRecords } from "@site/src/utils/anime/animeStore";
import { ANiMeRecord, Character } from "@site/src/utils/anime/types";
import BrowserOnly from '@docusaurus/BrowserOnly';
import { toJsDelivrUrl } from '@site/src/utils/cdn/linkJsDelivr';

// ================= 1. 类型定义 =================

type NodeType = 'Anime' | 'Character' | 'Actor';

interface GraphNode extends NodeObject {
    id: string;
    type: NodeType;
    name: string;
    name_cn?: string;
    image_url?: string;
    val: number;
    // 原始数据
    rawCharacter?: Character;
    rawAnimeId?: number;
    rawActorId?: number;
}

interface GraphLink extends LinkObject {
    source: string | GraphNode;
    target: string | GraphNode;
    type: 'Include' | 'VoicedBy' | 'Sequel' | 'Related';
    label?: string;
}

interface GraphData {
    nodes: GraphNode[];
    links: GraphLink[];
}

// ================= 2. 样式常量 (Dark Theme) =================

export const THEME = {
    background: '#0b0b0b',
    panelBg: 'rgba(20, 20, 20, 0.85)',
    textMain: '#ffffff',
    textSub: '#aaaaaa',
    border: 'rgba(255, 255, 255, 0.1)',
    colors: {
        Anime: '#ff6b6b',
        Character: '#4ecdc4',
        Actor: '#ffe66d',
        Sequel: '#ff9f43',
        Default: '#888888',
        TextBg: 'rgba(0, 0, 0, 0.7)',
    }
};

// 图片缓存
const imgCache = new Map<string, HTMLImageElement>();
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
    hiddenRelations: string[];
    setHiddenRelations: (val: string[]) => void;
    preferCn: boolean;
    setPreferCn: (val: boolean) => void;
    showImages: boolean;
    setShowImages: (val: boolean) => void;
    targetAnimeId: number | undefined;
    setTargetAnimeId: (val: number | undefined) => void;
    allRecords: readonly ANiMeRecord[];

    chargeStrength: number;
    setChargeStrength: (val: number) => void;
    linkDistance: number;
    setLinkDistance: (val: number) => void;
}

export const AnimeGraphController: React.FC<GraphControllerProps> = ({
    hiddenRelations, setHiddenRelations,
    preferCn, setPreferCn,
    showImages, setShowImages,
    targetAnimeId, setTargetAnimeId,
    allRecords,
    chargeStrength, setChargeStrength,
    linkDistance, setLinkDistance
}) => {
    const [isCollapsed, setIsCollapsed] = useState(false);
    const relationOptions = ['主角', '配角', '客串', '闲角', '旁白'];

    const containerStyle: React.CSSProperties = {
        position: 'absolute',
        bottom: '20px',
        right: '20px',
        width: '320px',
        zIndex: 10,
        // 动画核心属性
        transition: 'transform 0.3s cubic-bezier(0.25, 0.8, 0.25, 1)',
        transform: isCollapsed ? 'translateX(calc(100% + 20px))' : 'translateX(0)',
    };

    const toggleRelation = (rel: string) => {
        if (hiddenRelations.includes(rel)) {
            setHiddenRelations(hiddenRelations.filter(r => r !== rel));
        } else {
            setHiddenRelations([...hiddenRelations, rel]);
        }
    };

    const panelStyle: React.CSSProperties = {
        backgroundColor: THEME.panelBg,
        backdropFilter: 'blur(12px)',
        WebkitBackdropFilter: 'blur(12px)',
        borderRadius: '12px',
        padding: '20px',
        boxShadow: '0 8px 32px rgba(0, 0, 0, 0.6)',
        border: `1px solid ${THEME.border}`,
        color: THEME.textMain,
        fontSize: '14px',
        display: 'flex',
        flexDirection: 'column',
        gap: '16px',
        position: 'relative', // 为了定位 toggle 按钮
    };

    const toggleBtnStyle: React.CSSProperties = {
        position: 'absolute',
        left: '-25px', // 移出面板左侧
        bottom: '0',
        width: '32px',
        height: '32px',
        backgroundColor: THEME.panelBg,
        border: `1px solid ${THEME.border}`,
        borderRadius: '8px 0 0 8px', // 左侧圆角
        color: '#fff',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        cursor: 'pointer',
        boxShadow: '-4px 4px 10px rgba(0,0,0,0.3)',
        fontSize: '18px',
        backdropFilter: 'blur(12px)',
    };

    const sectionTitleStyle: React.CSSProperties = {
        fontWeight: '600',
        marginBottom: '8px',
        color: THEME.textSub,
        fontSize: '12px',
        textTransform: 'uppercase',
        letterSpacing: '1px',
        display: 'block'
    };

    const selectStyle: React.CSSProperties = {
        width: '100%',
        padding: '8px 12px',
        borderRadius: '6px',
        border: `1px solid ${THEME.border}`,
        background: 'rgba(255,255,255,0.05)',
        color: 'white',
        outline: 'none'
    };

    return (
        <div style={containerStyle}>
            {/* 折叠/展开按钮 */}
            <div
                style={toggleBtnStyle}
                onClick={() => setIsCollapsed(!isCollapsed)}
                title={isCollapsed ? "展开菜单" : "折叠菜单"}
            >
                {isCollapsed ? '⚙️' : '»'}
            </div>

            {/* 面板内容 */}
            <div style={panelStyle}>
                <div>
                    <span style={sectionTitleStyle}>Focus / 聚焦番剧</span>
                    <select
                        value={targetAnimeId || ''}
                        onChange={e => setTargetAnimeId(e.target.value ? Number(e.target.value) : undefined)}
                        style={selectStyle}
                    >
                        <option value="">🪐 全部番剧</option>
                        {allRecords.map(r => (
                            <option key={r.anime_data.id} value={r.anime_data.id}>
                                {r.anime_data.name_cn || r.anime_data.name}
                            </option>
                        ))}
                    </select>
                    {targetAnimeId && (
                        <div style={{ fontSize: '12px', marginTop: '5px', color: '#ff9f43' }}>
                            * 已自动加载前作/续作关系网
                        </div>
                    )}
                </div>

                <div style={{ borderTop: `1px solid ${THEME.border}` }} />

                <div>
                    <span style={sectionTitleStyle}>Filter / 过滤角色</span>
                    <div style={{ display: 'flex', gap: '10px', flexWrap: 'wrap' }}>
                        {relationOptions.map(rel => (
                            <label key={rel} style={{ cursor: 'pointer', display: 'flex', alignItems: 'center', fontSize: '13px' }}>
                                <input
                                    type="checkbox"
                                    checked={hiddenRelations.includes(rel)}
                                    onChange={() => toggleRelation(rel)}
                                    style={{ marginRight: '6px', accentColor: THEME.colors.Character }}
                                />
                                {rel}
                            </label>
                        ))}
                    </div>
                </div>

                <div>
                    <span style={sectionTitleStyle}>View / 视图设置</span>
                    <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <label style={{ cursor: 'pointer', fontSize: '13px' }}>
                            <input
                                type="checkbox"
                                checked={preferCn}
                                onChange={e => setPreferCn(e.target.checked)}
                                style={{ marginRight: '6px' }}
                            />
                            中文优先
                        </label>
                        <label style={{ cursor: 'pointer', fontSize: '13px' }}>
                            <input
                                type="checkbox"
                                checked={showImages}
                                onChange={e => setShowImages(e.target.checked)}
                                style={{ marginRight: '6px' }}
                            />
                            显示头像
                        </label>
                    </div>
                </div>
                <div style={{ borderTop: `1px solid ${THEME.border}` }} />
                <div>
                    <span style={sectionTitleStyle}>Physics / 物理引擎</span>

                    {/* 斥力控制 */}
                    <div style={{ marginBottom: '10px' }}>
                        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '12px', color: '#ccc' }}>
                            <span>斥力 (Charge)</span>
                            <span>{chargeStrength}</span>
                        </div>
                        <input
                            type="range"
                            min="-500"
                            max="-10"
                            step="10"
                            value={chargeStrength}
                            onChange={(e) => setChargeStrength(Number(e.target.value))}
                            style={{ width: '100%', cursor: 'pointer' }}
                        />
                    </div>

                    {/* 连线距离控制 */}
                    <div>
                        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '12px', color: '#ccc' }}>
                            <span>间距 (Distance)</span>
                            <span>{linkDistance}</span>
                        </div>
                        <input
                            type="range"
                            min="10"
                            max="200"
                            step="5"
                            value={linkDistance}
                            onChange={(e) => setLinkDistance(Number(e.target.value))}
                            style={{ width: '100%', cursor: 'pointer' }}
                        />
                    </div>
                </div>
            </div>
        </div>
    );
};

// ================= 4. 主图表组件 =================

// 新增: 右键菜单类型
interface ContextMenuState {
    visible: boolean;
    x: number;
    y: number;
    node: GraphNode | null;
}

interface AnimeForceGraphProps {
    baseUrl: string;
    targetAnimeId?: number;
    hiddenRelations?: string[];
    preferCn?: boolean;
    showImages?: boolean;
    chargeStrength?: number;
    linkDistance?: number;
}

export const AnimeForceGraph: React.FC<AnimeForceGraphProps> = ({
    baseUrl,
    targetAnimeId,
    hiddenRelations = [],
    preferCn = false,
    showImages = false,
    chargeStrength = -60,
    linkDistance = 60
}) => {
    // Hooks
    const records = useAnimeRecords(baseUrl);
    const actorMap = useActorMap(baseUrl);
    const fgRef = useRef<ForceGraphMethods | undefined>(undefined);
    const containerRef = useRef<HTMLDivElement>(null);

    // State
    const [dimensions, setDimensions] = useState({ width: 0, height: 0 });
    const [hoverNode, setHoverNode] = useState<GraphNode | null>(null);
    const [pinnedNodeId, setPinnedNodeId] = useState<string | null>(null);
    const [contextMenu, setContextMenu] = useState<ContextMenuState>({ visible: false, x: 0, y: 0, node: null });

    // ------------------- 初始化逻辑 -------------------

    // 监听容器尺寸变化
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

    // 响应物理引擎参数变化
    useEffect(() => {
        if (fgRef.current) {
            fgRef.current.d3Force('charge')?.strength(chargeStrength);
            fgRef.current.d3Force('link')?.distance(linkDistance);
            fgRef.current.d3ReheatSimulation();
        }
    }, [fgRef.current, chargeStrength, linkDistance]);

    // ------------------- 数据处理逻辑 -------------------

    // 1. 构建图数据
    const graphData = useMemo<GraphData>(() => {
        const nodes: Map<string, GraphNode> = new Map();
        const links: GraphLink[] = [];
        const addNode = (node: GraphNode) => {
            if (!nodes.has(node.id)) nodes.set(node.id, node);
        };

        // 筛选逻辑
        let activeRecords: Set<ANiMeRecord> = new Set();
        if (targetAnimeId) {
            const targetRecord = records.find(r => r.anime_data.id === targetAnimeId);
            if (targetRecord) {
                const queue = [targetRecord];
                const visitedIds = new Set<number>();
                while (queue.length > 0) {
                    const current = queue.shift()!;
                    if (visitedIds.has(current.anime_data.id)) continue;
                    visitedIds.add(current.anime_data.id);
                    activeRecords.add(current);
                    records.forEach(r => {
                        if (visitedIds.has(r.anime_data.id)) return;
                        const n1 = current.anime_data.name || "";
                        const n2 = r.anime_data.name || "";
                        if (n1.length > 4 && n2.startsWith(n1.substring(0, 4))) queue.push(r);
                    });
                }
            }
        } else {
            records.forEach(r => activeRecords.add(r));
        }

        // 构建节点
        const activeRecordsArray = Array.from(activeRecords);
        activeRecordsArray.forEach(record => {
            const anime = record.anime_data;
            const animeNodeId = `anime_${anime.id}`;
            addNode({
                id: animeNodeId,
                type: 'Anime',
                name: anime.name,
                name_cn: anime.name_cn,
                image_url: toJsDelivrUrl(`/py/anime/data/anime/${anime.id}.jpg`),
                val: 80,
                rawAnimeId: anime.id
            });

            anime.characters.forEach(char => {
                if (hiddenRelations.includes(char.relation)) return;
                const charNodeId = `char_${char.id}`;
                addNode({
                    id: charNodeId,
                    type: 'Character',
                    name: char.name,
                    name_cn: char.name_cn,
                    image_url: toJsDelivrUrl(`/py/anime/data/kyara/${char.id}.jpg`),
                    val: 20,
                    rawCharacter: char
                });
                links.push({ source: animeNodeId, target: charNodeId, type: 'Include' });

                char.actor_ids.forEach(actorId => {
                    const actor = actorMap.get(actorId);
                    if (actor) {
                        const actorNodeId = `actor_${actor.id}`;
                        addNode({
                            id: actorNodeId,
                            type: 'Actor',
                            name: actor.name,
                            image_url: toJsDelivrUrl(`/py/anime/data/cv/${actor.id}.jpg`),
                            val: 12,
                            rawActorId: actor.id
                        });
                        links.push({ source: charNodeId, target: actorNodeId, type: 'VoicedBy' });
                    }
                });
            });
        });

        // 构建连线 (续作关系)
        for (let i = 0; i < activeRecordsArray.length; i++) {
            for (let j = i + 1; j < activeRecordsArray.length; j++) {
                const r1 = activeRecordsArray[i];
                const r2 = activeRecordsArray[j];
                const n1 = r1.anime_data.name || "";
                const n2 = r2.anime_data.name || "";
                if (n1.length > 3 && n2.startsWith(n1.substring(0, 4))) {
                    links.push({
                        source: `anime_${r1.anime_data.id}`,
                        target: `anime_${r2.anime_data.id}`,
                        type: 'Sequel'
                    });
                }
            }
        }
        return { nodes: Array.from(nodes.values()), links };
    }, [records, actorMap, targetAnimeId, hiddenRelations]);

    // 2. 构建邻居索引表 (性能核心)
    const neighborMap = useMemo(() => {
        const map = new Map<string, Set<string>>();
        graphData.links.forEach(link => {
            const sId = typeof link.source === 'object' ? (link.source as any).id : link.source;
            const tId = typeof link.target === 'object' ? (link.target as any).id : link.target;
            if (!map.has(sId)) map.set(sId, new Set());
            if (!map.has(tId)) map.set(tId, new Set());
            map.get(sId)!.add(tId);
            map.get(tId)!.add(sId);
        });
        return map;
    }, [graphData]);

    // 3. 计算当前高亮节点集合 (Pinned > Hover)
    const highlightNodes = useMemo(() => {
        const set = new Set<string>();
        const targetId = pinnedNodeId || hoverNode?.id;
        if (targetId) {
            set.add(targetId);
            const layer1 = neighborMap.get(targetId);
            if (layer1) {
                layer1.forEach(n1 => {
                    set.add(n1);
                    const layer2 = neighborMap.get(n1);
                    if (layer2) layer2.forEach(n2 => set.add(n2));
                });
            }
        }
        return set;
    }, [pinnedNodeId, hoverNode, neighborMap]);

    // ------------------- 交互回调 -------------------

    // 拦截中键默认行为
    const handleContainerMouseDown = useCallback((e: React.MouseEvent) => {
        // Button 1 是中键
        if (e.button === 1) {
            e.preventDefault(); // 阻止浏览器默认的自动滚动圆圈
            e.stopPropagation();

            // 直接根据当前悬停的节点 (hoverNode) 来判断
            if (hoverNode) {
                // 如果鼠标正指着一个节点 -> 切换该节点的锁定状态
                setPinnedNodeId(prev => (prev === hoverNode.id ? null : hoverNode.id));
            } else {
                // 如果鼠标指着空白处 -> 取消所有锁定
                setPinnedNodeId(null);
            }
        }
    }, [hoverNode]);

    const handleNodeClick = useCallback((node: GraphNode, event: MouseEvent) => {
        setContextMenu(p => ({ ...p, visible: false }));

        // 这里的 event.button === 1 通常不会触发, 因为库过滤了
        // 只保留左键逻辑即可
        fgRef.current?.centerAt(node.x, node.y, 1000);
        fgRef.current?.zoom(3, 2000);
    }, []);

    const handleBackgroundClick = useCallback((event: MouseEvent) => {
        setContextMenu(p => ({ ...p, visible: false }));
        // 任意点击空白处取消锁定
        setPinnedNodeId(null);
    }, []);

    const handleNodeRightClick = useCallback((node: GraphNode, event: MouseEvent) => {
        setContextMenu({
            visible: true,
            x: event.clientX,
            y: event.clientY,
            node: node
        });
    }, []);

    // ------------------- Canvas 渲染 -------------------

    const nodeCanvasObject = useCallback((node: GraphNode, ctx: CanvasRenderingContext2D, globalScale: number) => {
        const label = preferCn ? (node.name_cn || node.name) : node.name;
        const fontSize = 12 / globalScale;
        const displayFontSize = Math.max(fontSize, 4);
        const baseWidth = Math.sqrt(node.val) * 6;

        let w = baseWidth;
        let h = baseWidth;

        const img = (showImages && node.image_url) ? getImage(node.image_url) : null;
        const hasImage = img && img.complete && img.naturalWidth > 0;

        if (hasImage) {
            const ratio = img.naturalHeight / img.naturalWidth;
            h = w * ratio;
        }
        const x = node.x! - w / 2;
        const y = node.y! - h / 2;
        const isHover = hoverNode === node;
        const isHighlight = highlightNodes.has(node.id);

        // 1. 绘制背景
        ctx.beginPath();
        ctx.rect(x, y, w, h);
        const baseColor = THEME.colors[node.type] || THEME.colors.Default;
        if (highlightNodes.size > 0 && !isHighlight && !isHover) {
            ctx.fillStyle = '#222';
            ctx.globalAlpha = 0.2;
        } else {
            ctx.fillStyle = baseColor;
            ctx.globalAlpha = 1;
        }
        ctx.fill();

        // 2. 绘制图片
        if (hasImage) {
            ctx.save();
            ctx.beginPath();
            ctx.rect(x, y, w, h);
            ctx.clip();
            ctx.drawImage(img, x, y, w, h);
            ctx.restore();
        }

        // 3. 边框
        if (isHover || isHighlight) {
            ctx.lineWidth = 2 / globalScale;
            ctx.strokeStyle = '#990099';
            ctx.stroke();
        }

        // 4. 文字
        const textY = node.y! + h / 2 + displayFontSize * 0.5 + 2;
        ctx.font = `${displayFontSize}px Sans-Serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        const textWidth = ctx.measureText(label).width;
        const bkgDimensions = [textWidth + 4, displayFontSize + 4];
        ctx.beginPath();
        ctx.rect(
            node.x! - bkgDimensions[0] / 2,
            textY - bkgDimensions[1] / 2,
            bkgDimensions[0],
            bkgDimensions[1]
        );
        ctx.fill();
        ctx.fillStyle = '#990099';
        ctx.fillText(label, node.x!, textY);
        ctx.globalAlpha = 1;
    }, [preferCn, showImages, hoverNode, highlightNodes]);

    // ------------------- 组件返回 -------------------

    return (
        <div
            ref={containerRef}
            style={{ width: '100%', height: '100%', overflow: 'hidden', background: THEME.background }}
            onContextMenu={e => e.preventDefault()}
            onMouseDown={handleContainerMouseDown} // 拦截中键默认行为
        >
            {dimensions.width > 0 && (
                <BrowserOnly>
                    {() => {
                        const ForceGraph2D = require('react-force-graph-2d').default;
                        return <ForceGraph2D
                            ref={fgRef}
                            width={dimensions.width}
                            height={dimensions.height}
                            graphData={graphData}
                            nodeLabel={() => ''} // 禁用默认 Label, 完全由 nodeCanvasObject 接管
                            nodeCanvasObject={nodeCanvasObject as any}

                            // 连线样式
                            linkColor={(link: any) => {
                                const s = typeof link.source === 'object' ? link.source.id : link.source;
                                const t = typeof link.target === 'object' ? link.target.id : link.target;
                                const isActive = highlightNodes.has(s) && highlightNodes.has(t);
                                if (highlightNodes.size > 0 && !isActive) return 'rgba(255,255,255,0.02)';
                                return isActive ? 'rgba(255, 255, 255, 0.6)' : 'rgba(255, 255, 255, 0.15)';
                            }}
                            linkWidth={(link: any) => {
                                const s = typeof link.source === 'object' ? link.source.id : link.source;
                                const t = typeof link.target === 'object' ? link.target.id : link.target;
                                return (highlightNodes.has(s) && highlightNodes.has(t)) ? 2 : 1;
                            }}

                            // 交互绑定
                            onNodeHover={(node: any) => setHoverNode(node || null)}
                            onNodeClick={handleNodeClick}
                            onNodeRightClick={handleNodeRightClick}
                            onBackgroundClick={handleBackgroundClick}

                            // 物理参数
                            d3VelocityDecay={0.1}
                            cooldownTicks={100}
                        />;
                    }}
                </BrowserOnly>
            )}

            {/* 自定义右键菜单 */}
            {contextMenu.visible && contextMenu.node && (
                <div style={{
                    position: 'fixed',
                    top: contextMenu.y,
                    left: contextMenu.x,
                    backgroundColor: 'rgba(30, 30, 30, 0.95)',
                    backdropFilter: 'blur(10px)',
                    border: '1px solid rgba(255,255,255,0.2)',
                    borderRadius: '8px',
                    padding: '6px 0',
                    minWidth: '160px',
                    boxShadow: '0 4px 12px rgba(0,0,0,0.5)',
                    zIndex: 1000,
                    color: '#fff',
                    fontSize: '13px',
                    fontFamily: 'sans-serif'
                }}>
                    <div style={{ padding: '8px 16px', borderBottom: '1px solid rgba(255,255,255,0.1)', color: '#aaa', fontSize: '12px' }}>
                        {contextMenu.node.type}: 「{preferCn ? (contextMenu.node.name_cn || contextMenu.node.name) : contextMenu.node.name}」
                    </div>

                    <MenuOption onClick={() => {
                        fgRef.current?.centerAt(contextMenu.node!.x!, contextMenu.node!.y!, 1000);
                        fgRef.current?.zoom(4, 2000);
                        setContextMenu(p => ({ ...p, visible: false }));
                    }}>
                        🔍 聚焦 (Focus)
                    </MenuOption>

                    {contextMenu.node.type === 'Anime' && (
                        <MenuOption onClick={() => {
                            window.open(`${baseUrl}anime/details?id=${contextMenu.node?.rawAnimeId}`, '_blank');
                            setContextMenu(p => ({ ...p, visible: false }));
                        }}>
                            🔗 跳转番剧详情
                        </MenuOption>
                    )}

                    <MenuOption onClick={() => setContextMenu(p => ({ ...p, visible: false }))}>
                        ❌ 关闭 (Close)
                    </MenuOption>
                </div>
            )}
        </div>
    );
};

// 简单的菜单组件
const MenuOption: React.FC<{ onClick: () => void, children: React.ReactNode }> = ({ onClick, children }) => (
    <div
        onClick={onClick}
        style={{ padding: '8px 16px', cursor: 'pointer', transition: 'background 0.2s' }}
        onMouseEnter={e => e.currentTarget.style.backgroundColor = 'rgba(255,255,255,0.1)'}
        onMouseLeave={e => e.currentTarget.style.backgroundColor = 'transparent'}
    >
        {children}
    </div>
);