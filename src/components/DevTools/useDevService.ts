import { useEffect, useState } from 'react';

/**
 * useDevService — 探测本地 dev-edit-server (端口 3310) 是否可达.
 * 仅本地开发环境启动该服务; 构建产物中此探测永远失败 => 组件不渲染.
 * 返回值: { ready, port } — ready 表示本地编辑功能可用.
 */
const DEV_PORT = 3310;

export function useDevService (): { ready: boolean; port: number } {
  const [ready, setReady] = useState(false);
  useEffect(() => {
    let cancelled = false;
    const ctrl = new AbortController();
    const timer = window.setTimeout(() => ctrl.abort(), 1500);
    fetch(`http://localhost:${DEV_PORT}/health`, { signal: ctrl.signal })
      .then((r) => r.json().then((j) => j.ok === true))
      .then((ok) => { if (!cancelled) setReady(ok); })
      .catch(() => { if (!cancelled) setReady(false); })
      .finally(() => window.clearTimeout(timer));
    return () => { cancelled = true; ctrl.abort(); };
  }, []);
  return { ready, port: DEV_PORT };
}
