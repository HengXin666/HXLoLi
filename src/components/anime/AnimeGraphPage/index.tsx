import React, { useMemo, useState, useCallback, useRef } from 'react';
import ForceGraph3D, { ForceGraphMethods } from 'react-force-graph-3d'; // 引入类型 ForceGraphMethods
import * as THREE from 'three';
import SpriteText from 'three-spritetext'; // 【新增】用于显示3D文字
import { useActorMap, useAnimeRecords } from "@site/src/utils/anime/animeStore";
import { ANiMeRecord, EpisodeType, WatchStatus, Character, SubjectType } from "@site/src/utils/anime/types";

// ================= 类型定义 =================

type NodeType = 'Anime' | 'Character' | 'Actor';

interface GraphNode {
  id: string;
  type: NodeType;
  name: string;
  name_cn?: string;
  image_url?: string;
  val: number; // 节点大小
  // 原始数据引用，方便点击事件等
  rawCharacter?: Character;
  rawAnimeId?: number;
  rawActorId?: number;
}

interface GraphLink {
  source: string;
  target: string;
  type: 'Include' | 'VoicedBy' | 'Sequel';
  label?: string;
}

interface GraphData {
  nodes: GraphNode[];
  links: GraphLink[];
}

// ================= 控制器组件 =================

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

export const AnimeGraphController: React.FC<GraphControllerProps> = ({
  hiddenRelations,
  setHiddenRelations,
  preferCn,
  setPreferCn,
  showImages,
  setShowImages,
  targetAnimeId,
  setTargetAnimeId,
  allRecords
}) => {
  const relationOptions = ['主角', '配角', '客串'];

  const toggleRelation = (rel: string) => {
    if (hiddenRelations.includes(rel)) {
      setHiddenRelations(hiddenRelations.filter(r => r !== rel));
    } else {
      setHiddenRelations([...hiddenRelations, rel]);
    }
  };

  return (
    <div style={{ padding: '10px', borderBottom: '1px solid #ccc', background: '#f5f5f5' }}>
      <div style={{ marginBottom: '10px', display: 'flex', gap: '20px', alignItems: 'center', flexWrap: 'wrap' }}>
        {/* 1. 角色筛选 */}
        <div>
          <strong>隐藏角色类型: </strong>
          {relationOptions.map(rel => (
            <label key={rel} style={{ marginLeft: '8px', cursor: 'pointer' }}>
              <input
                type="checkbox"
                checked={hiddenRelations.includes(rel)}
                onChange={() => toggleRelation(rel)}
              />
              {rel}
            </label>
          ))}
        </div>

        {/* 2. 显示设置 */}
        <div>
          <label style={{ cursor: 'pointer', marginRight: '15px' }}>
            <input
              type="checkbox"
              checked={preferCn}
              onChange={e => setPreferCn(e.target.checked)}
            /> 优先展示中文名
          </label>
          <label style={{ cursor: 'pointer' }}>
            <input
              type="checkbox"
              checked={showImages}
              onChange={e => setShowImages(e.target.checked)}
            /> 展示图片 (3D Sprite)
          </label>
        </div>
      </div>

      {/* 3. 番剧选择 (可选) */}
      <div>
        <strong>聚焦番剧: </strong>
        <select 
          value={targetAnimeId || ''} 
          onChange={e => setTargetAnimeId(e.target.value ? Number(e.target.value) : undefined)}
          style={{ padding: '4px', maxWidth: '300px' }}
        >
          <option value="">全部展示</option>
          {allRecords.map(r => (
            <option key={r.anime_data.id} value={r.anime_data.id}>
              {r.anime_data.name_cn || r.anime_data.name}
            </option>
          ))}
        </select>
      </div>
    </div>
  );
};

// ================= 主图表组件 =================

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
  
  // 【修复 1】useRef 提供初始值 undefined，并指定正确的类型 ForceGraphMethods
  const fgRef = useRef<ForceGraphMethods | undefined>(undefined);

  // 高亮状态
  const [highlightNodes, setHighlightNodes] = useState(new Set<string>());
  const [highlightLinks, setHighlightLinks] = useState(new Set<string>());
  const [hoverNode, setHoverNode] = useState<GraphNode | null>(null);

  // 1. 数据转换逻辑 (保持不变)
  const graphData = useMemo<GraphData>(() => {
    // ... (你的原有逻辑保持不变) ...
    // 为了节省篇幅，这里假设原有逻辑未变
    // 实际代码中请保留你原来的 graphData useMemo 内容
    const nodes: Map<string, GraphNode> = new Map();
    const links: GraphLink[] = [];
    
    // 这里简单 Mock 一下原有逻辑的结构，请替换回你原来的完整代码
    const addNode = (node: GraphNode) => { if (!nodes.has(node.id)) nodes.set(node.id, node); };
    let activeRecords = targetAnimeId ? records.filter(r => r.anime_data.id === targetAnimeId) : records;
    
    activeRecords.forEach(record => {
       const anime = record.anime_data;
       const animeNodeId = `anime_${anime.id}`;
       addNode({ id: animeNodeId, type: 'Anime', name: anime.name, name_cn: anime.name_cn, image_url: anime.image_url, val: 20, rawAnimeId: anime.id });
       
       anime.characters.forEach(char => {
         if (hiddenRelations.includes(char.relation)) return;
         const charNodeId = `char_${char.id}`;
         addNode({ id: charNodeId, type: 'Character', name: char.name, name_cn: char.name_cn, image_url: char.image_url, val: 10, rawCharacter: char });
         links.push({ source: animeNodeId, target: charNodeId, type: 'Include' });
         
         char.actor_ids.forEach(actorId => {
           const actor = actorMap.get(actorId);
           if(actor) {
             const actorNodeId = `actor_${actor.id}`;
             addNode({ id: actorNodeId, type: 'Actor', name: actor.name, image_url: actor.image_url, val: 5, rawActorId: actor.id });
             links.push({ source: charNodeId, target: actorNodeId, type: 'VoicedBy' });
           }
         });
       });
       // ... 续集逻辑保持不变
    });
    return { nodes: Array.from(nodes.values()), links: links };
  }, [records, actorMap, targetAnimeId, hiddenRelations]);

  // 2. 交互处理 (保持不变)
  const handleNodeHover = useCallback((node: GraphNode | null) => {
    // ... (保持不变) ...
    setHoverNode(node || null);
    const newHighlightNodes = new Set<string>();
    const newHighlightLinks = new Set<string>();
    if (node) {
      newHighlightNodes.add(node.id);
      graphData.links.forEach(link => {
        const sourceId = typeof link.source === 'object' ? (link.source as any).id : link.source;
        const targetId = typeof link.target === 'object' ? (link.target as any).id : link.target;
        if (sourceId === node.id || targetId === node.id) {
          newHighlightLinks.add((link as any).id || link);
          newHighlightNodes.add(sourceId);
          newHighlightNodes.add(targetId);
        }
      });
    }
    setHighlightNodes(newHighlightNodes);
    setHighlightLinks(newHighlightLinks);
  }, [graphData]);

  // 3. 样式与渲染配置
  const getNodeColor = useCallback((node: GraphNode) => {
    if (highlightNodes.size > 0 && !highlightNodes.has(node.id)) {
      return 'rgba(200, 200, 200, 0.2)'; 
    }
    switch (node.type) {
      case 'Anime': return '#ff6b6b';
      case 'Character': return '#4ecdc4';
      case 'Actor': return '#ffe66d';
      default: return '#ccc';
    }
  }, [highlightNodes]);

  // 【修复 2 & 新功能】自定义节点渲染：始终显示文字 + (图片 或 默认球体)
  const nodeThreeObject = useCallback((node: any) => {
    const group = new THREE.Group();

    // A. 创建文字 (始终显示)
    const labelText = preferCn ? (node.name_cn || node.name) : node.name;
    const spriteText = new SpriteText(labelText);
    spriteText.color = '#fff'; // 文字颜色，可以根据需要调整
    spriteText.textHeight = 4; // 文字大小
    spriteText.position.y = -8; // 将文字放在节点下方
    group.add(spriteText);

    // B. 创建节点主体 (图片 或 球体)
    if (showImages && node.image_url) {
      // B1. 图片模式
      const imgTexture = new THREE.TextureLoader().load(node.image_url);
      const material = new THREE.SpriteMaterial({ map: imgTexture });
      const sprite = new THREE.Sprite(material);
      const scale = node.val * 0.8; 
      sprite.scale.set(scale, scale, 1);
      group.add(sprite);
    } else {
      // B2. 默认几何体模式 (因为如果不返回 Object3D，文字也没法显示，所以这里要手动画一个球)
      const color = getNodeColor(node);
      const geometry = new THREE.SphereGeometry(Math.cbrt(node.val) * 2); // 估算一个合适的大小
      const material = new THREE.MeshLambertMaterial({ 
        color, 
        transparent: true, 
        opacity: 0.8 
      });
      const sphere = new THREE.Mesh(geometry, material);
      group.add(sphere);
    }

    return group; // 始终返回一个 Group (Object3D)，解决了类型错误
  }, [showImages, preferCn, getNodeColor]); 

  // 连线样式 (保持不变)
  const getLinkColor = (link: any) => {
     // ... (保持不变)
     const isHovered = highlightNodes.has(link.source.id) && highlightNodes.has(link.target.id);
     if (highlightNodes.size > 0 && !isHovered) return 'rgba(200, 200, 200, 0.1)';
     if (link.type === 'Sequel') return '#ff9f43';
     return 'rgba(150, 150, 150, 0.5)';
  };

  return (
    <div style={{ width: '100%', height: '600px', background: '#1a1a1a' }}>
      <ForceGraph3D
        ref={fgRef}
        graphData={graphData}
        
        // 节点相关
        // nodeLabel 仍然保留，用于鼠标悬浮时的提示（可选）
        nodeLabel={(node: any) => preferCn ? (node.name_cn || node.name) : node.name}
        // nodeColor={getNodeColor} // 由于使用了 nodeThreeObject，nodeColor 实际上不会生效，颜色在上面手动控制了
        nodeVal="val"
        nodeOpacity={1}
        nodeResolution={16}

        // 【关键】使用自定义对象渲染，实现“图片/球体 + 文字”
        nodeThreeObject={nodeThreeObject}

        // 连线相关
        linkColor={getLinkColor}
        linkWidth={link => (highlightNodes.has((link.source as any).id) && highlightNodes.has((link.target as any).id)) ? 2 : 0.5}
        linkDirectionalArrowLength={link => link.type === 'Sequel' ? 3.5 : 0}
        linkDirectionalArrowRelPos={1}

        // 交互
        onNodeHover={handleNodeHover as any}
        onNodeClick={node => {
          if (node) {
            // 【修复 3】添加非空/默认值检查，解决 undefined 报错
            const x = node.x || 0;
            const y = node.y || 0;
            const z = node.z || 0;

            const distance = 40;
            // Math.hypot 处理坐标
            const distRatio = 1 + distance / (Math.hypot(x, y, z) || 1); // 防止除以0

            fgRef.current?.cameraPosition(
              { x: x * distRatio, y: y * distRatio, z: z * distRatio }, 
              { x, y, z }, // lookAt
              3000
            );
          }
        }}
      />
    </div>
  );
};

// ================= 页面整合组件 (Usage Example) =================

export default function AnimeGraphPage({ baseUrl}: { baseUrl: string }) {
  const records = useAnimeRecords(baseUrl);
  
  // 状态提升到页面层级
  const [hiddenRelations, setHiddenRelations] = useState<string[]>([]);
  const [preferCn, setPreferCn] = useState(true);
  const [showImages, setShowImages] = useState(false);
  const [targetAnimeId, setTargetAnimeId] = useState<number | undefined>(undefined);

  return (
    <div>
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
      
      <AnimeForceGraph 
        baseUrl={baseUrl}
        targetAnimeId={targetAnimeId}
        hiddenRelations={hiddenRelations}
        preferCn={preferCn}
        showImages={showImages}
      />
    </div>
  );
}
