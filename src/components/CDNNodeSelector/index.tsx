/**
 * CDN 节点选择器 — Navbar 下拉面板
 */
import { ensureInit, mainEngine, onCdnReady, selectNodeAll } from '@site/src/utils/cdn/linkJsDelivr';
import React, { useCallback, useEffect, useRef, useState } from 'react';

const CDN_STORAGE_KEY = 'hxloli-cdn-node';

interface NodeInfo {
  id: string;
  name: string;
  region: string;
  latency: number | null;
  selected: boolean;
  testing: boolean;
}

function getNodeList(markAllTesting = false): NodeInfo[] {
  if (!mainEngine) return [];
  const nodes = mainEngine.getNodes();
  const current = mainEngine.getCurrentNode();
  const latencyMap = mainEngine.getLatencyResults();

  return nodes.map((n: any) => {
    const result = latencyMap.get(n.id);
    return {
      id: n.id,
      name: n.name,
      region: n.region ?? 'global',
      latency: markAllTesting ? null : (result?.latency ?? null),
      selected: current?.id === n.id,
      testing: markAllTesting,
    };
  });
}

function CDNNodePanel({ onUpdate }: { onUpdate: () => void }): React.ReactElement {
  const [nodes, setNodes] = useState<NodeInfo[]>(() => getNodeList());
  const [testing, setTesting] = useState(false);

  /** 执行一次完整测速 */
  const runSpeedTest = useCallback(async () => {
    if (!mainEngine) return;
    setTesting(true);
    setNodes(getNodeList(true));
    try {
      await mainEngine.testAllNodesStreaming((result: any) => {
        setNodes((prev) =>
          prev.map((n) =>
            n.id === result.nodeId
              ? { ...n, latency: result.latency, testing: false }
              : n,
          ),
        );
        onUpdate();
      });
      // 测速完成后同步最优节点到所有引擎
      const best = mainEngine.getCurrentNode();
      if (best) selectNodeAll(best.id);
      setNodes(getNodeList());
      onUpdate();
    } finally {
      setTesting(false);
    }
  }, [onUpdate]);

  useEffect(() => {
    setNodes(getNodeList());
    ensureInit().then(() => {
      const currentNodes = getNodeList();
      setNodes(currentNodes);
      onUpdate();
      // 如果没有测速数据 (所有节点延迟为 null), 自动触发一次测速
      const hasLatency = currentNodes.some((n) => n.latency != null);
      if (!hasLatency && mainEngine) {
        runSpeedTest();
      }
    });
  }, [onUpdate, runSpeedTest]);

  const handleSelect = useCallback((nodeId: string) => {
    selectNodeAll(nodeId);
    setNodes(getNodeList());
    onUpdate();
  }, [onUpdate]);

  return (
    <div style={panelStyle}>
      <div style={panelHeader}>CDN 节点</div>
      {nodes.map((node) => (
        <div
          key={node.id}
          onClick={() => handleSelect(node.id)}
          style={{
            ...nodeStyle,
            background: node.selected ? 'rgba(99, 102, 241, 0.12)' : 'transparent',
            borderLeft: node.selected ? '3px solid #6366f1' : '3px solid transparent',
          }}
          onMouseEnter={(e) => {
            if (!node.selected) e.currentTarget.style.background = 'rgba(255,255,255,0.05)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.background = node.selected ? 'rgba(99, 102, 241, 0.12)' : 'transparent';
          }}
        >
          <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <span style={{ fontSize: 10 }}>{node.region === 'china' ? '🇨🇳' : '🌐'}</span>
            <span style={{ fontSize: 13 }}>{node.name}</span>
            {node.selected && <span style={{ fontSize: 10, color: '#6366f1' }}>✓</span>}
          </div>
          <span style={{
            ...latencyFont,
            color: node.testing ? '#888' : node.latency != null ? latencyColor(node.latency) : '#666',
          }}>
            {node.testing ? '测速中...' : node.latency != null ? formatLatency(node.latency) : '---'}
          </span>
        </div>
      ))}
      <button onClick={runSpeedTest} disabled={testing} style={btnStyle}>
        {testing ? '⏳ 测速中...' : '🔄 重新测速'}
      </button>
    </div>
  );
}

export function CDNNavbarButton(): React.ReactElement {
  const [open, setOpen] = useState(false);
  const [label, setLabel] = useState('CDN');
  const [color, setColor] = useState('var(--ifm-navbar-link-color)');
  const ref = useRef<HTMLDivElement>(null);

  const refreshLabel = useCallback(() => {
    if (!mainEngine) return;
    const node = mainEngine.getCurrentNode();
    if (node) {
      const latencyMap = mainEngine.getLatencyResults();
      const result = latencyMap.get(node.id);
      if (result?.latency != null) {
        setLabel(formatLatency(result.latency));
        setColor(latencyColor(result.latency));
        return;
      }
    }
    setLabel('CDN');
    setColor('var(--ifm-navbar-link-color)');
  }, []);

  useEffect(() => {
    ensureInit().then(refreshLabel);
    // 监听测速完成事件, 自动刷新标签
    const unsub = onCdnReady(refreshLabel);
    return unsub;
  }, [refreshLabel]);

  // 跨页面同步
  useEffect(() => {
    const handler = (e: StorageEvent) => {
      if (e.key === CDN_STORAGE_KEY && e.newValue) {
        selectNodeAll(e.newValue);
        refreshLabel();
      }
    };
    window.addEventListener('storage', handler);
    return () => window.removeEventListener('storage', handler);
  }, [refreshLabel]);

  useEffect(() => {
    if (!open) return;
    const handler = (e: MouseEvent) => {
      if (ref.current && !ref.current.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, [open]);

  return (
    <div ref={ref} style={{ position: 'relative' }} className="navbar__item">
      <button
        className="navbar__link"
        onClick={() => setOpen(!open)}
        style={{
          background: 'none', border: 'none', cursor: 'pointer',
          display: 'flex', alignItems: 'center', gap: 4,
          fontSize: 13, padding: '4px 8px',
          color: 'var(--ifm-navbar-link-color)',
        }}
        title="CDN 节点选择"
      >
        <span style={{ fontSize: 11 }}>⚡</span>
        <span style={{ ...latencyFont, color }}>{label}</span>
      </button>
      {open && <CDNNodePanel onUpdate={refreshLabel} />}
    </div>
  );
}

function formatLatency(ms: number): string {
  if (ms < 0) return '超时';
  return `${ms}ms`;
}

function latencyColor(ms: number): string {
  if (ms < 0) return '#ef4444';
  if (ms < 100) return '#22c55e';
  if (ms < 300) return '#eab308';
  if (ms < 600) return '#f97316';
  return '#ef4444';
}

const latencyFont: React.CSSProperties = {
  fontFamily: 'ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace',
  fontSize: 12,
  fontVariantNumeric: 'tabular-nums',
  letterSpacing: -0.3,
};

const panelStyle: React.CSSProperties = {
  position: 'absolute', top: '100%', right: 0, marginTop: 4,
  background: 'var(--ifm-background-surface-color, #1e1e2e)',
  border: '1px solid var(--ifm-color-emphasis-300, #3a3a4a)',
  borderRadius: 8, boxShadow: '0 8px 24px rgba(0,0,0,0.5)',
  minWidth: 240, zIndex: 1000, overflow: 'hidden',
};

const panelHeader: React.CSSProperties = {
  padding: '8px 12px', fontSize: 11,
  color: 'var(--ifm-color-emphasis-600, #999)',
  borderBottom: '1px solid var(--ifm-color-emphasis-200, #2a2a3a)',
};

const nodeStyle: React.CSSProperties = {
  display: 'flex', justifyContent: 'space-between', alignItems: 'center',
  padding: '8px 12px', cursor: 'pointer', transition: 'background 0.15s',
  color: 'var(--ifm-font-color-base, #e0e0e0)',
};

const btnStyle: React.CSSProperties = {
  width: '100%', padding: '8px 12px', border: 'none',
  borderTop: '1px solid var(--ifm-color-emphasis-200, #2a2a3a)',
  background: 'transparent', color: 'var(--ifm-color-primary, #6366f1)',
  cursor: 'pointer', fontSize: 12, fontWeight: 600,
};

export default CDNNodePanel;
