/**
 * CDN 节点选择器 — Navbar 下拉面板
 */
import React, { useCallback, useEffect, useRef, useState } from 'react';
import { mainEngine, ensureInit } from '@site/src/utils/cdn/linkJsDelivr';

interface NodeInfo {
  id: string;
  name: string;
  region: string;
  latency: number | null;
  selected: boolean;
}

function CDNNodePanel({ onClose }: { onClose: () => void }): React.ReactElement {
  const [nodes, setNodes] = useState<NodeInfo[]>([]);
  const [testing, setTesting] = useState(false);

  const refreshNodes = useCallback(() => {
    if (!mainEngine.isInitialized()) return;
    const sorted = mainEngine.getSortedNodes();
    const current = mainEngine.getCurrentNode();
    setNodes(
      sorted.map((n) => ({
        id: n.id,
        name: n.name,
        region: n.region ?? 'global',
        latency: n.latency ?? null,
        selected: current?.id === n.id,
      })),
    );
  }, []);

  useEffect(() => {
    ensureInit().then(refreshNodes);
  }, [refreshNodes]);

  const handleTest = useCallback(async () => {
    setTesting(true);
    try {
      await mainEngine.testAndSelectBest();
      refreshNodes();
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
      <div style={{ padding: '8px 12px', fontSize: 11, opacity: 0.5, borderBottom: '1px solid var(--ifm-color-emphasis-200, #333)' }}>
        CDN 节点 (点击切换)
      </div>
      {nodes.map((node) => (
        <div
          key={node.id}
          onClick={() => handleSelect(node.id)}
          style={{
            ...nodeStyle,
            background: node.selected ? 'var(--ifm-color-primary-lightest, rgba(99,102,241,0.15))' : 'transparent',
            fontWeight: node.selected ? 600 : 400,
          }}
        >
          <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <span style={{ fontSize: 10 }}>
              {node.region === 'china' ? '🇨🇳' : '🌐'}
            </span>
            <span style={{ fontSize: 13 }}>{node.name}</span>
            {node.selected && <span style={{ fontSize: 10, color: 'var(--ifm-color-primary)' }}>✓</span>}
          </div>
          <span style={{
            fontSize: 12,
            fontFamily: 'monospace',
            color: node.latency != null ? latencyColor(node.latency) : 'var(--ifm-color-emphasis-500)',
          }}>
            {node.latency != null ? `${node.latency}ms` : '---'}
          </span>
        </div>
      ))}
      <button onClick={handleTest} disabled={testing} style={btnStyle}>
        {testing ? '⏳ 测速中...' : '🔄 重新测速'}
      </button>
    </div>
  );
}

/** Navbar 按钮 — 点击弹出 CDN 节点面板 */
export function CDNNavbarButton(): React.ReactElement {
  const [open, setOpen] = useState(false);
  const [currentName, setCurrentName] = useState('CDN');
  const [currentLatency, setCurrentLatency] = useState<number | null>(null);
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    ensureInit().then(() => {
      const node = mainEngine.getCurrentNode();
      if (node) {
        setCurrentName(node.name);
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
          fontSize: 14, padding: '4px 8px',
          color: 'var(--ifm-navbar-link-color)',
        }}
      >
        <span style={{ fontSize: 12 }}>⚡</span>
        <span>{currentName}</span>
        {currentLatency != null && (
          <span style={{ fontSize: 11, color: latencyColor(currentLatency), fontFamily: 'monospace' }}>
            {currentLatency}ms
          </span>
        )}
      </button>
      {open && <CDNNodePanel onClose={() => setOpen(false)} />}
    </div>
  );
}

function latencyColor(ms: number): string {
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
  border: '1px solid var(--ifm-color-emphasis-300, #444)',
  borderRadius: 8,
  boxShadow: '0 8px 24px rgba(0,0,0,0.4)',
  minWidth: 240,
  zIndex: 1000,
  overflow: 'hidden',
};

const nodeStyle: React.CSSProperties = {
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
  padding: '8px 12px',
  cursor: 'pointer',
  transition: 'background 0.15s',
  color: 'var(--ifm-font-color-base)',
};

const btnStyle: React.CSSProperties = {
  width: '100%',
  padding: '8px 12px',
  border: 'none',
  borderTop: '1px solid var(--ifm-color-emphasis-200, #333)',
  background: 'transparent',
  color: 'var(--ifm-color-primary)',
  cursor: 'pointer',
  fontSize: 12,
  fontWeight: 600,
};

export default CDNNodePanel;
