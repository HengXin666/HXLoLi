import BrowserOnly from "@docusaurus/BrowserOnly";
import { useHistory, useLocation } from "@docusaurus/router";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { ForceGraphMethods } from "react-force-graph-2d";

const THEME = {
  background: '#0b0b0b',
  panelBg: 'rgba(20, 20, 20, 0.85)',
  textMain: '#ffffff',
  textSub: '#aaaaaa',
  border: 'rgba(255, 255, 255, 0.1)',
  colors: {
    internal: '#4ecdc4',
    external: '#ff6b6b',
    current: '#ffe66d',
    linkInternal: 'rgba(78, 205, 196, 0.4)',
    linkExternal: 'rgba(255, 107, 107, 0.25)',
  }
};

interface DocNode {
  id: string;
  title: string;
  val: number;
  isCurrent: boolean;
  isExternal: boolean;
  linkCount: number;
  x?: number;
  y?: number;
}

interface DocLink {
  source: string | DocNode;
  target: string | DocNode;
  external: boolean;
}

interface GraphData {
  nodes: DocNode[];
  links: DocLink[];
}

interface RawGraphData {
  nodes: { id: string; title: string }[];
  links: { source: string; target: string; external: boolean }[];
}

type ViewMode = 'current' | 'global';
type GraphMode = '2d' | '3d';

export default function DocsGraphPage() {
  const { siteConfig } = useDocusaurusContext();
  const history = useHistory();
  const location = useLocation();

  const [rawData, setRawData] = useState<RawGraphData | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const params = new URLSearchParams(location.search);
  const docParamRaw = params.get('doc') || undefined;
  // URL 参数可能是编码后的中文路径，需要解码以匹配图数据中的原始中文 id
  const docParam = docParamRaw ? decodeURIComponent(docParamRaw) : undefined;
  const [viewMode, setViewMode] = useState<ViewMode>(docParam ? 'current' : 'global');
  const [showExternal, setShowExternal] = useState(
    params.has('ext') ? params.get('ext') !== 'false' : false
  );
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [chargeStrength, setChargeStrength] = useState(
    Number(params.get('charge')) || -60
  );
  const [linkDistance, setLinkDistance] = useState(
    Number(params.get('dist')) || 30
  );
  const [graphMode, setGraphMode] = useState<GraphMode>(
    (params.get('dim') === '3d') ? '3d' : '2d'
  );
  const [gravity, setGravity] = useState(
    Number(params.get('grav')) || 0.05
  );

  // 3D 组件需要延迟注入数据：先让组件以空数据挂载完成（layout 初始化），再注入真实数据
  const [graph3dReady, setGraph3dReady] = useState(graphMode === '2d');
  const emptyGraphData = useMemo<GraphData>(() => ({ nodes: [], links: [] }), []);

  // 切换到 3D 时重置 ready 状态
  useEffect(() => {
    if (graphMode === '3d') {
      setGraph3dReady(false);
    } else {
      setGraph3dReady(true);
    }
  }, [graphMode]);

  const focusDoc = viewMode === 'current' ? docParam : undefined;

  const fgRef2d = useRef<ForceGraphMethods | undefined>(undefined);
  const fgRef3d = useRef<any>(undefined);
  const containerRef = useRef<HTMLDivElement>(null);
  const [dimensions, setDimensions] = useState({ width: 0, height: 0 });
  const [hoverNode, setHoverNode] = useState<DocNode | null>(null);

  // 加载数据
  useEffect(() => {
    const url = `${siteConfig.baseUrl}docs-links-graph.json`;
    fetch(url)
      .then(r => {
        if (!r.ok) throw new Error(`HTTP ${r.status}: ${url}`);
        return r.json();
      })
      .then((data: RawGraphData) => {
        setRawData(data);
        setLoading(false);
      })
      .catch((e) => {
        setError(`${e.message}`);
        setLoading(false);
      });
  }, [siteConfig.baseUrl]);

  // 同步 URL
  useEffect(() => {
    const p = new URLSearchParams();
    if (docParam) p.set('doc', docParam);
    p.set('mode', viewMode);
    p.set('ext', String(showExternal));
    p.set('charge', String(chargeStrength));
    p.set('dist', String(linkDistance));
    p.set('dim', graphMode);
    p.set('grav', String(gravity));
    const newSearch = p.toString();
    if (location.search !== `?${newSearch}`) {
      history.replace({ pathname: location.pathname, search: newSearch });
    }
  }, [docParam, viewMode, showExternal, chargeStrength, linkDistance, graphMode, gravity, history, location.pathname]);

  // 监听容器尺寸 (和 AnimeForceGraph 完全一致)
  useEffect(() => {
    if (!containerRef.current) return;
    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        setDimensions({
          width: entry.contentRect.width,
          height: entry.contentRect.height,
        });
      }
    });
    resizeObserver.observe(containerRef.current);
    return () => resizeObserver.disconnect();
  }, []);

  // 响应物理参数
  useEffect(() => {
    const fg = graphMode === '3d' ? fgRef3d.current : fgRef2d.current;
    if (!fg) return;
    // 3D 模式下，等 layout 就绪后再操作力引擎
    if (graphMode === '3d' && !graph3dReady) return;
    try {
      fg.d3Force('charge')?.strength(chargeStrength);
      fg.d3Force('link')?.distance(linkDistance);
      if (graphMode === '3d') {
        const d3 = require('d3-force-3d');
        fg.d3Force('x', d3.forceX(0).strength(gravity));
        fg.d3Force('y', d3.forceY(0).strength(gravity));
        fg.d3Force('z', d3.forceZ(0).strength(gravity));
      } else {
        const d3 = require('d3-force');
        fg.d3Force('x', d3.forceX(0).strength(gravity));
        fg.d3Force('y', d3.forceY(0).strength(gravity));
      }
      fg.d3ReheatSimulation();
    } catch (e) {
      // layout 可能尚未初始化，忽略
    }
  }, [chargeStrength, linkDistance, graphMode, graph3dReady, gravity]);

  // 构建图数据
  const graphData = useMemo<GraphData>(() => {
    if (!rawData) return { nodes: [], links: [] };

    let filteredLinks = rawData.links;
    const relevantNodeIds = new Set<string>();

    if (focusDoc) {
      filteredLinks = rawData.links.filter(l =>
        l.source === focusDoc || l.target === focusDoc
      );
      if (!showExternal) {
        filteredLinks = filteredLinks.filter(l => !l.external);
      }
      filteredLinks.forEach(l => {
        relevantNodeIds.add(l.source);
        relevantNodeIds.add(l.target);
      });
      relevantNodeIds.add(focusDoc);
    } else {
      if (!showExternal) {
        filteredLinks = filteredLinks.filter(l => !l.external);
      }
      filteredLinks.forEach(l => {
        relevantNodeIds.add(l.source);
        relevantNodeIds.add(l.target);
      });
      rawData.nodes.forEach(n => relevantNodeIds.add(n.id));
    }

    const linkCountMap = new Map<string, number>();
    filteredLinks.forEach(l => {
      linkCountMap.set(l.source, (linkCountMap.get(l.source) || 0) + 1);
      linkCountMap.set(l.target, (linkCountMap.get(l.target) || 0) + 1);
    });

    const nodeMap = new Map(rawData.nodes.map(n => [n.id, n]));
    const nodes: DocNode[] = [];

    relevantNodeIds.forEach(id => {
      const raw = nodeMap.get(id);
      const lc = linkCountMap.get(id) || 0;
      const isExt = !raw;
      nodes.push({
        id,
        title: raw?.title || extractDomain(id),
        val: id === focusDoc ? 30 : Math.max(5, Math.min(25, 5 + lc * 2)),
        isCurrent: id === focusDoc,
        isExternal: isExt || /^https?:\/\//i.test(id),
        linkCount: lc,
      });
    });

    return {
      nodes,
      links: filteredLinks.map(l => ({ ...l })),
    };
  }, [rawData, focusDoc, showExternal]);

  // 邻居索引
  const neighborMap = useMemo(() => {
    const map = new Map<string, Set<string>>();
    graphData.links.forEach(link => {
      const s = typeof link.source === 'object' ? (link.source as any).id : link.source;
      const t = typeof link.target === 'object' ? (link.target as any).id : link.target;
      if (!map.has(s)) map.set(s, new Set());
      if (!map.has(t)) map.set(t, new Set());
      map.get(s)!.add(t);
      map.get(t)!.add(s);
    });
    return map;
  }, [graphData]);

  // 高亮集合
  const highlightNodes = useMemo(() => {
    const set = new Set<string>();
    const targetId = hoverNode?.id;
    if (targetId) {
      set.add(targetId);
      neighborMap.get(targetId)?.forEach(n => set.add(n));
    }
    return set;
  }, [hoverNode, neighborMap]);

  // 节点渲染
  const nodeCanvasObject = useCallback((node: DocNode, ctx: CanvasRenderingContext2D, globalScale: number) => {
    const r = Math.sqrt(node.val) * 2;
    const x = node.x!;
    const y = node.y!;
    const isHover = hoverNode?.id === node.id;
    const isHighlight = highlightNodes.has(node.id);

    const color = node.isCurrent ? THEME.colors.current
      : node.isExternal ? THEME.colors.external
      : THEME.colors.internal;

    ctx.beginPath();
    ctx.arc(x, y, r, 0, 2 * Math.PI);

    if (highlightNodes.size > 0 && !isHighlight) {
      ctx.fillStyle = '#333';
      ctx.globalAlpha = 0.2;
    } else {
      ctx.fillStyle = color;
      ctx.globalAlpha = node.isCurrent ? 1 : 0.85;
    }
    ctx.fill();

    if (isHover || node.isCurrent) {
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.5 / globalScale;
      ctx.stroke();
    }

    const fontSize = Math.max(10 / globalScale, 3);
    if (globalScale > 0.8 || isHighlight || node.isCurrent) {
      const label = node.title.length > 20 ? node.title.slice(0, 18) + '…' : node.title;
      ctx.font = `${fontSize}px sans-serif`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      const tw = ctx.measureText(label).width;
      ctx.fillStyle = 'rgba(0,0,0,0.7)';
      ctx.globalAlpha = 1;
      ctx.fillRect(x - tw / 2 - 1, y + r + 1, tw + 2, fontSize + 2);
      ctx.fillStyle = node.isCurrent ? THEME.colors.current : '#ddd';
      ctx.fillText(label, x, y + r + 2);
    }
    ctx.globalAlpha = 1;
  }, [hoverNode, highlightNodes]);

  // 3D 节点渲染
  const nodeThreeObject = useCallback((node: DocNode) => {
    const THREE = require('three');
    const SpriteText = require('three-spritetext').default;

    const group = new THREE.Group();

    const color = node.isCurrent ? THEME.colors.current
      : node.isExternal ? THEME.colors.external
      : THEME.colors.internal;

    const r = Math.sqrt(node.val) * 0.8;
    const geometry = new THREE.SphereGeometry(r, 16, 12);
    const material = new THREE.MeshLambertMaterial({
      color,
      transparent: true,
      opacity: 0.9,
    });
    const sphere = new THREE.Mesh(geometry, material);
    group.add(sphere);

    const label = node.title.length > 16 ? node.title.slice(0, 14) + '…' : node.title;
    const sprite = new SpriteText(label);
    sprite.color = node.isCurrent ? THEME.colors.current : '#cccccc';
    sprite.textHeight = Math.max(2, r * 0.8);
    sprite.position.y = -(r + 3);
    sprite.backgroundColor = 'rgba(0,0,0,0.6)';
    sprite.padding = 1;
    sprite.borderRadius = 1;
    group.add(sprite);

    return group;
  }, []);

  // 点击节点
  const handleNodeClick = useCallback((node: DocNode) => {
    if (node.isExternal) {
      window.open(node.id, '_blank');
    } else {
      window.open(`${siteConfig.baseUrl}${node.id.replace(/^\//, '')}`, '_blank');
    }
  }, [siteConfig.baseUrl]);

  // 不使用 early return, 始终渲染 containerRef 以确保 ResizeObserver 能正确绑定
  const showGraph = !loading && !error && rawData;

  const currentDocTitle = focusDoc && rawData
    ? rawData.nodes.find(n => n.id === focusDoc)?.title || focusDoc
    : null;

  const panelStyle: React.CSSProperties = {
    backgroundColor: THEME.panelBg,
    backdropFilter: 'blur(12px)',
    WebkitBackdropFilter: 'blur(12px)',
    borderRadius: '12px',
    padding: '16px',
    boxShadow: '0 8px 32px rgba(0, 0, 0, 0.6)',
    border: `1px solid ${THEME.border}`,
    color: THEME.textMain,
    fontSize: '13px',
    display: 'flex',
    flexDirection: 'column' as const,
    gap: '12px',
    position: 'relative' as const,
  };

  const sectionTitle: React.CSSProperties = {
    fontWeight: '600',
    marginBottom: '4px',
    color: THEME.textSub,
    fontSize: '10px',
    textTransform: 'uppercase' as const,
    letterSpacing: '1px',
  };

  const tabStyle = (active: boolean): React.CSSProperties => ({
    flex: 1,
    padding: '6px 0',
    border: 'none',
    borderRadius: '6px',
    background: active ? 'rgba(255, 136, 255, 0.2)' : 'transparent',
    color: active ? '#ff88ff' : THEME.textSub,
    fontSize: '12px',
    fontWeight: active ? 600 : 400,
    cursor: 'pointer',
    transition: 'all 0.2s',
  });

  return (
    <div style={{
      position: 'relative',
      width: '100%',
      height: '100vh',
      background: THEME.background,
      overflow: 'hidden',
    }}>
      {/* 图表容器 — 始终渲染以确保 ResizeObserver 能正确绑定 */}
      <div
        ref={containerRef}
        style={{ width: '100%', height: '100%', overflow: 'hidden', background: THEME.background }}
        onContextMenu={e => e.preventDefault()}
      >
        {showGraph && dimensions.width > 0 && graphData.nodes.length > 0 && (
          <BrowserOnly key={graphMode}>
            {() => {
              if (graphMode === '3d') {
                const ForceGraph3D = require('react-force-graph-3d').default;
                return (
                  <ForceGraph3D
                    ref={(el: any) => {
                      fgRef3d.current = el;
                      // 组件挂载后，延迟注入真实数据
                      if (el && !graph3dReady) {
                        requestAnimationFrame(() => {
                          setGraph3dReady(true);
                        });
                      }
                    }}
                    width={dimensions.width}
                    height={dimensions.height}
                    graphData={graph3dReady ? graphData : emptyGraphData}
                    nodeLabel={(node: DocNode) => node.title}
                    nodeThreeObject={nodeThreeObject}
                    nodeThreeObjectExtend={false}
                    linkColor={(link: any) => {
                      const s = typeof link.source === 'object' ? link.source.id : link.source;
                      const t = typeof link.target === 'object' ? link.target.id : link.target;
                      const isActive = highlightNodes.has(s) && highlightNodes.has(t);
                      if (highlightNodes.size > 0 && !isActive) return 'rgba(255,255,255,0.03)';
                      return link.external ? THEME.colors.linkExternal : THEME.colors.linkInternal;
                    }}
                    linkWidth={(link: any) => {
                      const s = typeof link.source === 'object' ? link.source.id : link.source;
                      const t = typeof link.target === 'object' ? link.target.id : link.target;
                      return (highlightNodes.has(s) && highlightNodes.has(t)) ? 1.5 : 0.3;
                    }}
                    linkDirectionalArrowLength={3}
                    linkDirectionalArrowRelPos={1}
                    linkOpacity={0.6}
                    onNodeHover={(node: any) => setHoverNode(node || null)}
                    onNodeClick={handleNodeClick}
                    d3VelocityDecay={0.15}
                    cooldownTicks={150}
                    backgroundColor={THEME.background}
                  />
                );
              }

              const ForceGraph2D = require('react-force-graph-2d').default;
              return (
                <ForceGraph2D
                  ref={fgRef2d}
                  width={dimensions.width}
                  height={dimensions.height}
                  graphData={graphData}
                  nodeLabel={() => ''}
                  nodeCanvasObject={nodeCanvasObject as any}
                  linkColor={(link: any) => {
                    const s = typeof link.source === 'object' ? link.source.id : link.source;
                    const t = typeof link.target === 'object' ? link.target.id : link.target;
                    const isActive = highlightNodes.has(s) && highlightNodes.has(t);
                    if (highlightNodes.size > 0 && !isActive) return 'rgba(255,255,255,0.02)';
                    return link.external ? THEME.colors.linkExternal : THEME.colors.linkInternal;
                  }}
                  linkWidth={(link: any) => {
                    const s = typeof link.source === 'object' ? link.source.id : link.source;
                    const t = typeof link.target === 'object' ? link.target.id : link.target;
                    return (highlightNodes.has(s) && highlightNodes.has(t)) ? 2 : 0.5;
                  }}
                  linkDirectionalArrowLength={3}
                  linkDirectionalArrowRelPos={1}
                  onNodeHover={(node: any) => setHoverNode(node || null)}
                  onNodeClick={handleNodeClick}
                  d3VelocityDecay={0.15}
                  cooldownTicks={150}
                />
              );
            }}
          </BrowserOnly>
        )}
      </div>

      {/* 加载/错误覆盖层 */}
      {loading && (
        <div style={{ position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center', color: '#fff', fontSize: '16px', zIndex: 5 }}>
          加载笔记关系数据...
        </div>
      )}
      {!loading && (error || !rawData) && (
        <div style={{ position: 'absolute', inset: 0, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', color: '#fff', gap: '12px', zIndex: 5 }}>
          <div style={{ fontSize: '16px', color: '#ff6b6b' }}>{error || '无法加载笔记关系数据'}</div>
          <div style={{ fontSize: '12px', color: '#888' }}>请确保已构建 docs-links-graph.json</div>
        </div>
      )}

      {/* 控制面板 */}
      <div style={{
        position: 'absolute',
        bottom: '20px',
        right: '20px',
        width: '280px',
        zIndex: 10,
        transition: 'transform 0.3s cubic-bezier(0.25, 0.8, 0.25, 1)',
        transform: isCollapsed ? 'translateX(calc(100% + 20px))' : 'translateX(0)',
      }}>
        <div
          style={{
            position: 'absolute',
            left: '-25px',
            bottom: '0',
            width: '32px',
            height: '32px',
            backgroundColor: THEME.panelBg,
            border: `1px solid ${THEME.border}`,
            borderRadius: '8px 0 0 8px',
            color: '#fff',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            cursor: 'pointer',
            boxShadow: '-4px 4px 10px rgba(0,0,0,0.3)',
            fontSize: '18px',
            backdropFilter: 'blur(12px)',
          }}
          onClick={() => setIsCollapsed(!isCollapsed)}
          title={isCollapsed ? "展开菜单" : "折叠菜单"}
        >
          {isCollapsed ? '⚙' : '»'}
        </div>

        <div style={panelStyle}>
          <div>
            <span style={sectionTitle}>View / 视图</span>
            <div style={{ display: 'flex', gap: '4px', background: 'rgba(255,255,255,0.05)', borderRadius: '8px', padding: '3px' }}>
              {docParam && (
                <button style={tabStyle(viewMode === 'current')} onClick={() => setViewMode('current')}>
                  当前文章
                </button>
              )}
              <button style={tabStyle(viewMode === 'global')} onClick={() => setViewMode('global')}>
                全站
              </button>
            </div>
            <div style={{ display: 'flex', gap: '4px', background: 'rgba(255,255,255,0.05)', borderRadius: '8px', padding: '3px', marginTop: '6px' }}>
              <button style={tabStyle(graphMode === '2d')} onClick={() => setGraphMode('2d')}>
                2D
              </button>
              <button style={tabStyle(graphMode === '3d')} onClick={() => setGraphMode('3d')}>
                3D
              </button>
            </div>
            {viewMode === 'current' && currentDocTitle && (
              <div style={{ fontSize: '11px', marginTop: '4px', color: '#ff88ff', overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                {currentDocTitle}
              </div>
            )}
          </div>

          <div style={{ borderTop: `1px solid ${THEME.border}` }} />

          <div>
            <span style={sectionTitle}>Filter / 过滤</span>
            <label style={{ cursor: 'pointer', fontSize: '12px', display: 'flex', alignItems: 'center' }}>
              <input
                type="checkbox"
                checked={showExternal}
                onChange={e => setShowExternal(e.target.checked)}
                style={{ marginRight: '6px', accentColor: THEME.colors.external }}
              />
              显示外部链接
            </label>
          </div>

          <div>
            <span style={sectionTitle}>Legend / 图例</span>
            <div style={{ display: 'flex', gap: '10px', fontSize: '11px', flexWrap: 'wrap' }}>
              <span style={{ color: THEME.colors.internal }}>● 站内</span>
              <span style={{ color: THEME.colors.external }}>● 外部</span>
              {focusDoc && <span style={{ color: THEME.colors.current }}>● 当前</span>}
            </div>
          </div>

          <div style={{ borderTop: `1px solid ${THEME.border}` }} />

          <div>
            <span style={sectionTitle}>Physics / 物理引擎</span>
            <div style={{ marginBottom: '6px' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px', color: '#ccc' }}>
                <span>斥力</span><span>{chargeStrength}</span>
              </div>
              <input type="range" min="-500" max="-10" step="10" value={chargeStrength}
                onChange={e => setChargeStrength(Number(e.target.value))}
                style={{ width: '100%', cursor: 'pointer' }} />
            </div>
            <div style={{ marginBottom: '6px' }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px', color: '#ccc' }}>
                <span>间距</span><span>{linkDistance}</span>
              </div>
              <input type="range" min="10" max="200" step="5" value={linkDistance}
                onChange={e => setLinkDistance(Number(e.target.value))}
                style={{ width: '100%', cursor: 'pointer' }} />
            </div>
            <div>
              <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '11px', color: '#ccc' }}>
                <span>向心力</span><span>{gravity.toFixed(2)}</span>
              </div>
              <input type="range" min="0" max="0.2" step="0.005" value={gravity}
                onChange={e => setGravity(Number(e.target.value))}
                style={{ width: '100%', cursor: 'pointer' }} />
            </div>
          </div>

          <div style={{ borderTop: `1px solid ${THEME.border}`, paddingTop: '6px', fontSize: '10px', color: THEME.textSub }}>
            节点: {graphData.nodes.length} | 链接: {graphData.links.length} | 左键: 打开笔记
          </div>
        </div>
      </div>
    </div>
  );
}

function extractDomain(url: string): string {
  try {
    return new URL(url).hostname;
  } catch {
    return url.split('/').pop() || url;
  }
}
