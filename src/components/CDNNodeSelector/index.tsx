/**
 * CDN 节点选择器 — Navbar 下拉面板
 */
import React, { useCallback, useEffect, useRef, useState } from 'react';
import { mainEngine, ensureInit } from '@site/src/utils/cdn/linkJsDelivr';

interface NodeInfo {
  id: string;
  name: string;
  region: string;
  latency: number | null; // null = 未测速, -1 = 超时
  selected: boolean;
  testing: boolean;
}

function CDNNodePanel(): React.ReactElement {
  const [nodes, setNodes] = useState<NodeInfo[]>([]);
  const [testing, setTesting] = useState(false);

  const refreshNodes = useCallback((markAllTesting = false) => {
    if (!mainEngine.isInitialized()) return;
    const sorted = mainEngine.getSortedNodes();
    const current = mainEngine.getCurrentNode();
    setNodes(
      sorted.map((n) => ({
        id: n.id,
        name: n.name,
        region: n.region ?? 'global',
        latency: markAllTesting ? null : (n.latency ?? null),
        selected: current?.id === n.id,
        testing: markAllTesting,
      })),
    );
  }, []);

  useEffect(() => {
    ensureInit().then(() => refreshNodes());
  }, [refreshNodes]);

  const handleTest = useCallback(async () => {
    setTesting(true);
    // 先把所有节点标记为「测速中」
    refreshNodes(true);

    try {
      // 流式测速：每个节点完成立即更新
      await mainEngine.testAllNodesStreaming((result) => {
        setNodes((prev) =>
          prev.map((n) =>
            n.id === result.nodeId
              ? { ...n, latency: result.latency, testing: false }
              : n,
          ),
        );
      });
      // 测速完成后刷新选中状态
      const current = mainEngine.getCurrentNode();
      setNodes((prev) =>
        prev
          .map((n) => ({ ...n, selected: current?.id === n.id }))
          .sort((a, b) => {
            const la = a.latency == null || a.latency < 0 ? Infinity : a.latency;
            const lb = b.latency == null || b.latency < 0 ? Infinity : b.latency;
            return la - lb;
          }),
      );
    } finally {
      setTesting(false);
    }
  }, [refreshNodes]);

  const handleSelect = useCallback(
    (nodeId: string) => {
      mainEngine.selectNode(nodeId);
      refreshNodes();
    },
    [refreshNodes],
  );

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
            <span style={{ fontSize: 10 }}>
              {node.region === 'china' ? '🇨🇳' : '🌐'}
            </span>
            <span style={{ fontSize: 13 }}>{node.name}</span>
            {node.selected && <span style={{ fontSize: 10, color: '#6366f1' }}>✓</span>}
          </div>
          <span style={{
            fontSize: 12,
            fontFamily: 'monospace',
            color: node.testing
              ? '#888'
              : node.latency != null
                ? latencyColor(node.latency)
                : '#666',
          }}>
            {node.testing
              ? '测速中...'
              : node.latency != null
                ? formatLatency(node.latency)
                : '---'}
          </span>
        </div>
      ))}
      <button onClick={handleTest} disabled={testing} style={btnStyle}>
        {testing ? '⏳ 测速中...' : '🔄 重新测速'}
      </button>
    </div>
  );
}

/** Navbar 按钮 */
export function CDNNavbarButton(): React.ReactElement {
  const [open, setOpen] = useState(false);
  const [currentLatency, setCurrentLatency] = useState<number | null>(null);
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    ensureInit().then(() => {
      const node = mainEngine.getCurrentNode();
      if (node) {
        const sorted = mainEngine.getSortedNodes();
        const found = sorted.find((n) => n.id === node.id);
        if (found?.latency != null) setCurrentLatency(found.latency);
      }
    });
  }, [open]);

  // 点击外部关闭
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
        <span style={{
          fontFamily: 'monospace',
          fontSize: 12,
          color: currentLatency != null ? latencyColor(currentLatency) : 'var(--ifm-navbar-link-color)',
        }}>
          {currentLatency != null ? formatLatency(currentLatency) : 'CDN'}
        </span>
      </button>
      {open && <CDNNodePanel />}
    </div>
  );
}

function formatLatency(ms: number): string {
  if (ms < 0) return '超时';
  return `${ms}ms`;
}

function latencyColor(ms: number): string {
  if (ms < 0) return '#ef4444';  // 超时 = 红色
  if (ms < 100) return '#22c55e';
  if (ms < 300) return '#eab308';
  if (ms < 600) return '#f97316';
  return '#ef4444';
}

const panelStyle: React.CSSProperties = {
  position: 'absolute',
  top: '100%',
  right: 0,
  marginTop: 4,
  background: 'var(--ifm-background-surface-color, #1e1e2e)',
  border: '1px solid var(--ifm-color-emphasis-300, #3a3a4a)',
  borderRadius: 8,
  boxShadow: '0 8px 24px rgba(0,0,0,0.5)',
  minWidth: 240,
  zIndex: 1000,
  overflow: 'hidden',
};

const panelHeader: React.CSSProperties = {
  padding: '8px 12px',
  fontSize: 11,
  color: 'var(--ifm-color-emphasis-600, #999)',
  borderBottom: '1px solid var(--ifm-color-emphasis-200, #2a2a3a)',
};

const nodeStyle: React.CSSProperties = {
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
  padding: '8px 12px',
  cursor: 'pointer',
  transition: 'background 0.15s',
  color: 'var(--ifm-font-color-base, #e0e0e0)',
};

const btnStyle: React.CSSProperties = {
  width: '100%',
  padding: '8px 12px',
  border: 'none',
  borderTop: '1px solid var(--ifm-color-emphasis-200, #2a2a3a)',
  background: 'transparent',
  color: 'var(--ifm-color-primary, #6366f1)',
  cursor: 'pointer',
  fontSize: 12,
  fontWeight: 600,
};

export default CDNNodePanel;
