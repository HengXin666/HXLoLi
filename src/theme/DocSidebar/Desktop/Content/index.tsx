/**
 * Swizzled DocSidebar/Desktop/Content — 在知识库侧边栏顶部加「重建侧边栏」工具栏 (仅本地 dev).
 *
 * 点击后:
 *   1. 先快照当前 DOM 里所有 [data-sidebar-path]
 *   2. POST /api/sidebar/rebuild -> 后端重扫 ai-docs/ 并重写 sidebarsAiDocs.ts, 返回 diff
 *   3. Docusaurus dev 热更新侧边栏 (本组件下方原样渲染)
 *   4. diff 里的 added/changed 路径映射到 DOM -> 播插入动画 (仅这些项)
 * 构建产物中 useDevService 探测失败 => 工具栏不渲染, 零痕迹.
 */
import React, { useEffect, useRef, useState, type ReactNode } from 'react';
import { translate } from '@docusaurus/Translate';
import OriginalDocSidebarDesktopContent from '@theme-original/DocSidebar/Desktop/Content';
import { useDevService } from '@site/src/components/DevTools/useDevService';
import { useLocation } from '@docusaurus/router';
import clsx from 'clsx';

import styles from './styles.module.css';
import { cancelSidebarFlash, flashSidebarChanges, rememberFlash } from './sidebarAnimator';

interface SidebarDiff {
  added: string[];
  changed: string[];
  removed: string[];
}

export default function DocSidebarDesktopContentWrapper (props: {
  path: string;
  sidebar: unknown;
  className?: string;
}): ReactNode {
  const { ready, port } = useDevService();
  const location = useLocation();
  const isAIDocs = location.pathname.includes('/knowledge-base');
  const [busy, setBusy] = useState(false);
  const [status, setStatus] = useState<'idle' | 'ok' | 'nochange' | 'error'>('idle');
  const [msg, setMsg] = useState('');
  const timerRef = useRef<number | null>(null);

  // 挂载时: 若存在重建后遗留的 flash (刚 reload 回来), 立即播放动画
  useEffect(() => {
    try {
      const raw = sessionStorage.getItem('hx-sidebar-flash');
      if (raw) {
        const t = window.setTimeout(() => flashSidebarChanges(), 300);
        return () => window.clearTimeout(t);
      }
    } catch { /* ignore */ }
  }, []);

  // 清理状态提示定时器 + 动画观察
  useEffect(() => () => {
    if (timerRef.current) window.clearTimeout(timerRef.current);
    cancelSidebarFlash();
  }, []);

  const show = (s: 'ok' | 'nochange' | 'error', text: string) => {
    setStatus(s);
    setMsg(text);
    if (timerRef.current) window.clearTimeout(timerRef.current);
    timerRef.current = window.setTimeout(() => { setStatus('idle'); setMsg(''); }, 4000);
  };

  const rebuild = async () => {
    if (busy || !ready) return;
    setBusy(true);
    setStatus('idle');
    try {
      const resp = await fetch('http://localhost:' + port + '/api/sidebar/rebuild', { method: 'POST' });
      const data = await resp.json();
      if (!data.ok) throw new Error(data.error || 'rebuild failed');
      if (data.unchanged) {
        show('nochange', '没有发现新内容, 侧边栏已是最新');
        return;
      }
      const diff: SidebarDiff = data.diff || { added: [], changed: [], removed: [] };
      // 记录需要高亮的路径 (sessionStorage 跨 reload 保留)
      rememberFlash({
        at: Date.now(),
        added: diff.added,
        changed: diff.changed,
        removed: diff.removed,
      });
      // 新增文档需要 Docusaurus 重新生成路由/元数据, 客户端侧边栏数据无法纯 HMR 热更,
      // 因此刷新页面; 刷新后 animator 依据 flash 状态只对新增/变化项播放插入动画.
      show('ok', '已重建 (+' + diff.added.length + '/~' + diff.changed.length + '), 刷新中…');
      // 给 webpack 留出编译时间 (sidebarsAiDocs.ts 变更 -> 重编译约 1-2s), 避免 reload 撞上编译中的半成品
      window.setTimeout(() => {
        window.location.reload();
      }, 1200);
    } catch (e) {
      show('error', '重建失败: ' + String((e as Error).message || e).slice(0, 60));
    } finally {
      setBusy(false);
    }
  };

  const showToolbar = isAIDocs && ready;

  return (
    <div className={styles.root}>
      {showToolbar && (
        <div className={styles.toolbar} data-hx-sidebar-toolbar="1">
          <button
            type="button"
            className={clsx(styles.rebuildBtn, busy && styles.rebuildBusy)}
            onClick={rebuild}
            disabled={busy}
            title="重新扫描 ai-docs/ 并重建侧边栏 (只高亮有变化的部分)">
            {busy ? '重建中…' : (
              <>
                <span className={styles.rebuildIcon} aria-hidden="true">⟳</span>
                重建侧边栏
              </>
            )}
          </button>
          {status !== 'idle' && msg && (
            <div
              className={clsx(styles.status, {
                [styles.statusOk]: status === 'ok',
                [styles.statusNochange]: status === 'nochange',
                [styles.statusError]: status === 'error',
              })}
              role="status">
              {msg}
            </div>
          )}
        </div>
      )}
      <OriginalDocSidebarDesktopContent {...props} />
    </div>
  );
}
