import { FloatingLayer } from '@site/src/components/common/Tooltip';
import {
  type CdnNodeView,
  getCdnServerSnapshot,
  getCdnSnapshot,
  onCdnReady,
  retestAllNodes,
  selectNodeAll,
} from '@site/src/utils/cdn/linkJsDelivr';
import React, { useEffect, useId, useRef, useState, useSyncExternalStore } from 'react';
import { FaCheck, FaEarthAsia, FaGlobe, FaRotateRight, FaSignal } from 'react-icons/fa6';
import styles from './styles.module.css';

function useCdnSnapshot() {
  return useSyncExternalStore(onCdnReady, getCdnSnapshot, getCdnServerSnapshot);
}

function formatLatency(ms: number | null): string {
  if (ms == null) return '—';
  if (ms < 0) return '超时';
  return `${ms}ms`;
}

/** 延迟分档 → CSS 类名 (颜色在 styles.module.css 里) */
function levelOf(ms: number | null): string {
  if (ms == null) return styles.lvIdle;
  if (ms < 0) return styles.lvBad;
  if (ms < 100) return styles.lvGood;
  if (ms < 300) return styles.lvOk;
  if (ms < 600) return styles.lvSlow;
  return styles.lvBad;
}

function NodeRow({ node, selected, best }: { node: CdnNodeView; selected: boolean; best: boolean }): React.ReactElement {
  const china = node.region === 'china';
  const RegionIcon = china ? FaEarthAsia : FaGlobe;
  return (
    <button
      type="button"
      className={styles.row}
      data-selected={selected ? 'true' : undefined}
      onClick={() => selectNodeAll(node.id, 'manual')}
      aria-pressed={selected}
    >
      <span className={styles.check} aria-hidden="true">{selected && <FaCheck size={11} />}</span>
      <RegionIcon size={12} className={styles.region} aria-label={china ? '国内节点' : '全球节点'} />
      <span className={styles.name}>{node.name}</span>
      {best && <span className={styles.best}>最优</span>}
      <span className={`${styles.latency} ${node.testing ? styles.lvIdle : levelOf(node.latency)}`}>
        {node.testing ? '测速中' : formatLatency(node.latency)}
      </span>
    </button>
  );
}

/**
 * CDN 节点选择器 — Navbar 一行状态指示 + 悬浮面板
 *
 * - 触发器: 信号图标 (颜色 = 当前节点延迟分档) + 延迟数字, 一行.
 * - 面板: 复用 common/Tooltip 的 FloatingLayer (与 Select 同一套 fixed 定位、翻转、视口夹取、淡入), 不另造弹层.
 * - 状态: 全部来自 linkJsDelivr 的单一 store (useSyncExternalStore); 服务端快照固定为 idle, 水合首帧一致.
 * .agents/notes/implemented/feature/2026-11-01-navbar-slim-and-cdn-first-load.md
 */
export function CDNNavbarButton(): React.ReactElement {
  const snap = useCdnSnapshot();
  const panelId = `hx-cdn-${useId().replace(/:/g, '')}`;
  const [open, setOpen] = useState(false);
  const [anchor, setAnchor] = useState<HTMLButtonElement | null>(null);
  const cardRef = useRef<HTMLDivElement | null>(null);

  const current = snap.nodes.find((n) => n.id === snap.currentId) ?? null;
  const testing = snap.phase === 'testing';
  const latency = current?.latency ?? null;
  const pending = testing && latency == null;
  const title = current ? `CDN 节点: ${current.name}` : 'CDN 节点';
  // 测完才标最优, 测速中不标, 避免 "最优" 在节点间跳
  const bestId = testing ? null : snap.bestId;
  const best = snap.nodes.find((n) => n.id === bestId) ?? null;
  const suggest = snap.source === 'manual' && best && best.id !== snap.currentId ? best : null;

  // 点外部 / Esc 关闭 (与 common/Tooltip/Select 同口径: pointerdown 捕获阶段)
  useEffect(() => {
    if (!open) return undefined;
    const onDown = (e: PointerEvent) => {
      const t = e.target as Node;
      if (anchor?.contains(t) || cardRef.current?.contains(t)) return;
      setOpen(false);
    };
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        setOpen(false);
        anchor?.focus();
      }
    };
    document.addEventListener('pointerdown', onDown, true);
    window.addEventListener('keydown', onKey);
    return () => {
      document.removeEventListener('pointerdown', onDown, true);
      window.removeEventListener('keydown', onKey);
    };
  }, [open, anchor]);

  const panel = (
    <div className={styles.panel} role="group" aria-label="CDN 节点">
      <div className={styles.header}>
        CDN 节点 · {testing ? '测速中' : snap.source === 'manual' ? '手动选择' : '自动优选'}
      </div>
      {snap.nodes.map((node) => (
        <NodeRow key={node.id} node={node} selected={node.id === snap.currentId} best={node.id === bestId} />
      ))}
      {suggest && (
        <button type="button" className={styles.suggest} onClick={() => selectNodeAll(suggest.id, 'auto')}>
          当前节点较慢, 切换到 {suggest.name} ({formatLatency(suggest.latency)})
        </button>
      )}
      <button
        type="button"
        className={styles.retest}
        onClick={() => { void retestAllNodes(); }}
        disabled={testing}
      >
        <FaRotateRight size={11} className={testing ? styles.spin : undefined} aria-hidden="true" />
        {testing ? '测速中' : '重新测速'}
      </button>
    </div>
  );

  return (
    <div className={`navbar__item ${styles.wrap}`}>
      <button
        ref={setAnchor}
        type="button"
        className={styles.trigger}
        onClick={() => setOpen((v) => !v)}
        title={title}
        aria-label={`${title} · ${pending ? '测速中' : formatLatency(latency)}`}
        aria-expanded={open}
        aria-controls={open ? panelId : undefined}
        data-open={open ? 'true' : undefined}
      >
        <FaSignal
          size={13}
          aria-hidden="true"
          className={`${levelOf(latency)} ${pending ? styles.pulse : ''}`.trim()}
        />
        <span className={styles.value}>{pending ? '···' : formatLatency(latency)}</span>
      </button>
      {open && anchor && (
        <FloatingLayer
          id={panelId}
          anchor={anchor}
          content={panel}
          role="presentation"
          prefer="bottom"
          arrow={false}
          interactive
          width="auto"
          cardRef={cardRef}
          style={{ padding: 4, minWidth: 248 }}
        />
      )}
    </div>
  );
}

export default CDNNavbarButton;
