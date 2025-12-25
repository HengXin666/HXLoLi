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

const THEME = {
    background: '#0b0b0b', // 更深邃的背景
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
}

const AnimeGraphController: React.FC<GraphControllerProps> = ({
    hiddenRelations, setHiddenRelations,
    preferCn, setPreferCn,
    showImages, setShowImages,
    targetAnimeId, setTargetAnimeId,
    allRecords
}) => {
    const relationOptions = ['主角', '配角', '客串', '闲角', '旁白'];

    const toggleRelation = (rel: string) => {
        if (hiddenRelations.includes(rel)) {
            setHiddenRelations(hiddenRelations.filter(r => r !== rel));
        } else {
            setHiddenRelations([...hiddenRelations, rel]);
        }
    };

    const panelStyle: React.CSSProperties = {
        position: 'absolute',
        bottom: '20px',
        right: '20px',
        width: '320px',
        backgroundColor: THEME.panelBg,
        backdropFilter: 'blur(12px)',
        WebkitBackdropFilter: 'blur(12px)',
        borderRadius: '12px',
        padding: '20px',
        boxShadow: '0 8px 32px rgba(0, 0, 0, 0.6)',
        zIndex: 10,
        border: `1px solid ${THEME.border}`,
        color: THEME.textMain,
        fontSize: '14px',
        display: 'flex',
        flexDirection: 'column',
        gap: '16px',
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
        </div>
    );
};

// ================= 4. 主图表组件 =================

interface AnimeForceGraphProps {
    baseUrl: string;
    targetAnimeId?: number;
    hiddenRelations?: string[];
    preferCn?: boolean;
    showImages?: boolean;
}

export const AnimeForceGraph: React.FC<AnimeForceGraphProps> = ({
    baseUrl,
    targetAnimeId,
    hiddenRelations = [],
    preferCn = false,
    showImages = false
}) => {
    const records = useAnimeRecords(baseUrl);
    const actorMap = useActorMap(baseUrl);
    const fgRef = useRef<ForceGraphMethods | undefined>(undefined);
    const containerRef = useRef<HTMLDivElement>(null);

    // 状态
    const [highlightNodes, setHighlightNodes] = useState(new Set<string>());
    const [hoverNode, setHoverNode] = useState<GraphNode | null>(null);
    const [dimensions, setDimensions] = useState({ width: 0, height: 0 });

    // 精准的尺寸监听, 解决坐标偏移问题
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

    // 增大节点间距 (物理引擎调整)
    useEffect(() => {
        if (fgRef.current) {
            // Charge: 斥力, 负数越小, 斥力越大 (-30 -> -600)
            fgRef.current.d3Force('charge')?.strength(-60);
            // Link: 连线距离, 设大一点
            fgRef.current.d3Force('link')?.distance(60);
        }
    }, [fgRef.current]); // 当 ref 挂载后执行

    // ---------------- 数据处理核心逻辑 ----------------

    const graphData = useMemo<GraphData>(() => {
        const nodes: Map<string, GraphNode> = new Map();
        const links: GraphLink[] = [];
        const addNode = (node: GraphNode) => {
            if (!nodes.has(node.id)) {
                nodes.set(node.id, node);
            }
        };

        let activeRecords: Set<ANiMeRecord> = new Set();

        // === 关联遍历逻辑 ===
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
                        let isRelated = false;

                        // 简单的名字相似度关联
                        const curName = current.anime_data.name || "";
                        const rName = r.anime_data.name || "";
                        if (curName.length > 4 && rName.startsWith(curName.substring(0, 4))) {
                            isRelated = true;
                        }

                        if (isRelated) queue.push(r);
                    });
                }
            }
        } else {
            records.forEach(r => activeRecords.add(r));
        }

        const activeRecordsArray = Array.from(activeRecords);

        // 构建图数据
        activeRecordsArray.forEach(record => {
            const anime = record.anime_data;
            const animeNodeId = `anime_${anime.id}`;

            addNode({
                id: animeNodeId,
                type: 'Anime',
                name: anime.name,
                name_cn: anime.name_cn,
                image_url: toJsDelivrUrl(`/py/anime/data/anime/${anime.id}.jpg`),
                val: 80, // 稍微加大番剧节点
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

        // 建立番剧间连线
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

    // ---------------- 渲染 ----------------

    const nodeCanvasObject = useCallback((node: GraphNode, ctx: CanvasRenderingContext2D, globalScale: number) => {
        const label = preferCn ? (node.name_cn || node.name) : node.name;
        const fontSize = 12 / globalScale;
        const displayFontSize = Math.max(fontSize, 4);

        // 尺寸放大逻辑
        // 基础宽度: val 开根号 * 放大倍数 (这里设为 6, 原本是 2 左右, 放大了3倍)
        const baseWidth = Math.sqrt(node.val) * 6;

        let w = baseWidth;
        let h = baseWidth; // 默认正方形

        const img = (showImages && node.image_url) ? getImage(node.image_url) : null;
        const hasImage = img && img.complete && img.naturalWidth > 0;

        // 如果有图片, 根据图片原比例调整高度
        if (hasImage) {
            const ratio = img.naturalHeight / img.naturalWidth;
            h = w * ratio;
        }

        // 居中坐标 (ForceGraph 的 x,y 是中心点, Canvas 绘制通常需要左上角)
        const x = node.x! - w / 2;
        const y = node.y! - h / 2;

        const isHover = hoverNode === node;
        const isHighlight = highlightNodes.has(node.id);

        // 1. 绘制背景/占位符
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

        // 2. 绘制图片 (完全原比例, 不剪裁)
        if (hasImage) {
            ctx.save();
            // 为了防止图片溢出背景(如果计算有误), 可以 clip 一下
            ctx.beginPath();
            ctx.rect(x, y, w, h);
            ctx.clip();
            ctx.drawImage(img, x, y, w, h);

            ctx.restore();
        }

        // 3. 边框 (高亮)
        if (isHover || isHighlight) {
            ctx.lineWidth = 2 / globalScale;
            ctx.strokeStyle = '#990099';
            ctx.stroke();
        }

        // 4. 文字 (【修复 3】始终显示, 不依赖 globalScale 判断)
        // 无论大小如何都绘制, 但会根据 globalScale 调整文字背景框的大小
        const textY = node.y! + h / 2 + displayFontSize * 0.5 + 2;

        ctx.font = `${displayFontSize}px Sans-Serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';

        const textWidth = ctx.measureText(label).width;
        const bkgDimensions = [textWidth + 4, displayFontSize + 4]; // padding

        // 背景
        ctx.beginPath();
        // 使用 rect 兼容性更好
        ctx.rect(
            node.x! - bkgDimensions[0] / 2,
            textY - bkgDimensions[1] / 2,
            bkgDimensions[0],
            bkgDimensions[1]
        );
        ctx.fill();

        // 字体
        ctx.fillStyle = '#990099';
        ctx.fillText(label, node.x!, textY);

        ctx.globalAlpha = 1;
    }, [preferCn, showImages, hoverNode, highlightNodes]);
    return (
        <div ref={containerRef} style={{ width: '100%', height: '100%', overflow: 'hidden', background: THEME.background }}>
            {/* 只有当获取到尺寸时才渲染 Graph, 否则会因为 width=0 导致初始化错误 */}
            {dimensions.width > 0 && (
                <BrowserOnly>
                    {() => {
                        const ForceGraph2D = require('react-force-graph-2d').default;
                        return <ForceGraph2D
                            ref={fgRef}
                            width={dimensions.width}
                            height={dimensions.height}
                            graphData={graphData}

                            nodeLabel={() => ''}
                            nodeCanvasObject={nodeCanvasObject as any}

                            linkColor={() => 'rgba(255,255,255,0.15)'}
                            linkWidth={(link: GraphLink) => (highlightNodes.has((link.source as GraphNode).id) && highlightNodes.has((link.target as GraphNode).id)) ? 2 : 1}

                            onNodeHover={(node: GraphNode | null) => {
                                setHoverNode((node as GraphNode) || null);
                                const newHighlight = new Set<string>();
                                if (node) {
                                    newHighlight.add(node.id as string);
                                    graphData.links.forEach(link => {
                                        const s = (link.source as GraphNode).id;
                                        const t = (link.target as GraphNode).id;
                                        if (s === node.id) newHighlight.add(t);
                                        if (t === node.id) newHighlight.add(s);
                                    });
                                }
                                setHighlightNodes(newHighlight);
                            }}

                            onNodeClick={(node: GraphNode) => {
                                fgRef.current?.centerAt(node.x, node.y, 1000);
                                fgRef.current?.zoom(3, 2000);
                            }}

                            d3VelocityDecay={0.1}
                            cooldownTicks={100}
                        />;
                    }}
                </BrowserOnly>)}
        </div>
    );
};

// ================= 5. 页面整合组件 =================

export default function AnimeGraphPage ({ baseUrl }: { baseUrl: string }) {
    const records = useAnimeRecords(baseUrl);

    const [hiddenRelations, setHiddenRelations] = useState<string[]>(["配角", "客串", "闲角", "旁白"]);
    const [preferCn, setPreferCn] = useState(true);
    const [showImages, setShowImages] = useState(true);
    const [targetAnimeId, setTargetAnimeId] = useState<number | undefined>(undefined);

    return (
        <div style={{ position: 'relative', width: '100%', height: '100vh', background: THEME.background }}>
            <AnimeForceGraph
                baseUrl={baseUrl}
                targetAnimeId={targetAnimeId}
                hiddenRelations={hiddenRelations}
                preferCn={preferCn}
                showImages={showImages}
            />
            <AnimeGraphController
                hiddenRelations={hiddenRelations}
                setHiddenRelations={setHiddenRelations}
                preferCn={preferCn}
                setPreferCn={setPreferCn}
                showImages={showImages}
                setShowImages={setShowImages}
                targetAnimeId={targetAnimeId}
                setTargetAnimeId={setTargetAnimeId}
                allRecords={records}
            />
        </div>
    );
}
