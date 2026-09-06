import React, { useCallback, useEffect, useRef, useState } from 'react';
import { FaCode, FaExternalLinkAlt, FaHighlighter } from 'react-icons/fa';
import { useDevService } from './useDevService';
import styles from './styles.module.css';

interface BlockInfo {
  id: string;
  kind: string;
  start: number;
  end: number;
  text: string;
  source: string;
}

interface InlineEditorProps {
  source?: string; // e.g. @site/ai-docs/.../index.md
}

/**
 * 本地开发专属 DevTools:
 * 1) 工具条「在 VS Code 中打开」: 复用已打开的 VS Code 窗口跳到本文件 H1 行。
 * 2) 选中正文文本 -> 右键 -> 自定义菜单「在 VS Code 中打开 第 N 行」:
 *    把选区所在块映射回源码行号, 精确跳到那一行 (仅本地可用, 否则放行浏览器右键菜单)。
 * 仅 dev-edit-server (localhost:3310) 可达时渲染; 生产构建/静态站永不出现。
 */

/** 文本相似度评分: 基于公共子串/前缀覆盖, 0..1 */
function scoreMatch (domText: string, candText: string): number {
  if (!domText || !candText) return 0;
  const d = domText;
  const c = candText;
  // 前缀公共长度
  let pre = 0;
  const maxPre = Math.min(d.length, c.length);
  while (pre < maxPre && d[pre] === c[pre]) pre++;
  // 后缀公共长度
  let suf = 0;
  const maxSuf = Math.min(d.length, c.length);
  while (suf < maxSuf && d[d.length - 1 - suf] === c[c.length - 1 - suf]) suf++;
  // 归一化: 前缀+后缀对短文本更关键
  const cover = Math.min(1, (pre + suf) / Math.min(d.length, c.length));
  const lenBonus = 1 - Math.abs(d.length - c.length) / Math.max(d.length, c.length, 1);
  return cover * 0.7 + lenBonus * 0.3;
}

export default function InlineEditor ({ source }: InlineEditorProps): React.ReactNode | null {
  const { ready, port } = useDevService();
  const [status, setStatus] = useState<'idle' | 'opening' | 'done' | 'error'>('idle');
  const [errMsg, setErrMsg] = useState('');
  const [blocks, setBlocks] = useState<BlockInfo[] | null>(null);
  const [menu, setMenu] = useState<{ x: number; y: number; line: number; col: number; kind: string } | null>(null);

  const rel = useCallback(() => {
    if (!source) return null;
    return source.replace(/^@site\//, '').replace(/^\/+/, '') || null;
  }, [source]);

  // 服务可达性探测
  const [reachable, setReachable] = useState<boolean>(false);
  useEffect(() => {
    let cancelled = false;
    const controller = new AbortController();
    const t = setTimeout(() => controller.abort(), 1500);
    fetch('http://localhost:' + port + '/health', { signal: controller.signal })
      .then((r) => r.json())
      .then((j) => { if (!cancelled) setReachable(!!j.ok); })
      .catch(() => { if (!cancelled) setReachable(false); })
      .finally(() => clearTimeout(t));
    return () => { cancelled = true; controller.abort(); clearTimeout(t); };
  }, [port]);

  // 拉取块映射 (用于选区 -> 行号)
  useEffect(() => {
    if (!ready) return;
    const r = rel();
    if (!r) return;
    const controller = new AbortController();
    fetch('http://localhost:' + port + '/api/blocks?rel=' + encodeURIComponent(r), { signal: controller.signal })
      .then((resp) => resp.json())
      .then((j) => { setBlocks(j.blocks ?? null); })
      .catch(() => { /* 服务不在, 菜单不出现 */ });
    return () => controller.abort();
  }, [ready, port, rel]);

  // 归一化文本: 去 fence 行/前导后导空白, 压缩空白, 便于比对
  const normText = useCallback((s: string): string => {
    return String(s || '')
      .split('\n')
      .filter((l) => !/^`{3,}/.test(l.trim())) // 去掉代码围栏行
      .map((l) => l.trim())
      .filter(Boolean)
      .join('\n')
      .replace(/[\s\u00a0]+/g, ' ')
      .trim();
  }, []);

  // 找选区所在块: 不再依赖 DOM 顺序索引, 而是用「文本内容匹配」精确定位到源码块
  // (代码块/表格/告警等渲染成 DIV/TABLE 包装, 顺序索引会漂移, 文本匹配才可靠)
  const findBlockForSelection = useCallback((): { line: number; col: number; kind: string } | null => {
    const sel = window.getSelection();
    if (!sel || sel.isCollapsed || !sel.anchorNode || !blocks || blocks.length === 0) return null;
    // 兼容: docs/knowledge-base 是 .theme-doc-markdown, blog 是 article > .markdown
    const md = document.querySelector('.theme-doc-markdown')
      ?? document.querySelector('article .markdown')
      ?? document.querySelector('article .theme-doc-markdown');
    if (!md) return null;
    // 从 anchorNode 向上找属于 md 的顶层块
    let el: Element | null = sel.anchorNode instanceof Element ? sel.anchorNode : (sel.anchorNode.parentElement ?? null);
    while (el && el !== md) {
      const parent: Element | null = el.parentElement;
      if (parent === md) break; // el 就是顶层块
      el = parent;
    }
    if (!el || el === md) return null;
    // 顶层块的渲染文本 (含代码块/表格内容)
    const domText = normText(el.textContent || '');

    // 1) 文本匹配: 与每个块的 text/source 归一化后比较, 选相似度最高者
    let best: BlockInfo | null = null;
    let bestScore = -1;
    for (const bl of blocks) {
      const a = normText(bl.text || '');
      const b = normText(bl.source || '');
      const cand = a.length >= b.length ? a : b; // source 通常更完整 (表格/代码保留原文)
      if (!cand) continue;
      const score = scoreMatch(domText, cand);
      if (score > bestScore) {
        bestScore = score;
        best = bl;
      }
    }
    // 要求: 至少 DOM 文本的相当部分被覆盖, 否则可能匹配到无关块
    if (best && bestScore >= 0.35 && domText.length > 2) {
      // 列号估算: anchorNode 在块内的字符偏移 -> 行内列 (最佳努力)
      let col = 1;
      try {
        const anchorText = sel.anchorNode?.textContent || '';
        const off = sel.anchorOffset || 0;
        const lineStart = anchorText.lastIndexOf('\n', Math.max(0, off - 1)) + 1;
        col = Math.max(1, off - lineStart + 1);
      } catch {}
      // 代码块内精确到行: 通过 token-line 行号定位实际源码行
      let line = best.start;
      if (best.kind === 'code' && el.querySelector('pre code')) {
        const codeEl = el.querySelector('pre code');
        const lineSpans = Array.from(codeEl.querySelectorAll('span[class*="token-line"]'));
        // anchorNode 所在的行 span 索引
        let anchorSpanIdx = -1;
        const anchor = sel.anchorNode;
        if (anchor instanceof Element) {
          let n: Element | null = anchor.closest('span[class*="token-line"]');
          if (n) anchorSpanIdx = lineSpans.indexOf(n as HTMLElement);
        } else if (anchor && anchor.parentElement) {
          const n = anchor.parentElement.closest('span[class*="token-line"]');
          if (n) anchorSpanIdx = lineSpans.indexOf(n as HTMLElement);
        }
        if (anchorSpanIdx >= 0) {
          // best.start 是围栏行, 内容从 start+1 开始
          line = best.start + 1 + anchorSpanIdx;
        }
      }
      return { line, col, kind: best.kind };
    }

    // 2) 兜底: 文本匹配失败 (空块/图片等) 时, 按顺序粗略定位
    //    用「选区所在顶层块在 md children 中的位置」在 blocks 里等比例估算
    const kids = Array.from(md.children) as HTMLElement[];
    const idx = kids.indexOf(el as HTMLElement);
    if (idx < 0) return null;
    const ratio = kids.length > 1 ? idx / (kids.length - 1) : 0;
    const approxIdx = Math.min(blocks.length - 1, Math.max(0, Math.round(ratio * (blocks.length - 1))));
    return { line: blocks[approxIdx].start, col: 1, kind: blocks[approxIdx].kind };
  }, [blocks, normText]);

  // 右键: 有选区 -> 自定义菜单; 无选区 -> 放行浏览器菜单
  useEffect(() => {
    if (!ready || !reachable) return;
    const onContextMenu = (e: MouseEvent) => {
      const info = findBlockForSelection();
      if (!info) { setMenu(null); return; } // 无选区, 放行
      e.preventDefault();
      setMenu({ x: e.clientX, y: e.clientY, line: info.line, col: info.col, kind: info.kind });
    };
    const onDown = (e: MouseEvent) => {
      const t = e.target as Node;
      if (menu && !(t instanceof Element && t.closest && t.closest('[data-hx-vsc-menu]'))) setMenu(null);
    };
    document.addEventListener('contextmenu', onContextMenu);
    document.addEventListener('mousedown', onDown);
    document.addEventListener('scroll', () => setMenu(null), true);
    return () => {
      document.removeEventListener('contextmenu', onContextMenu);
      document.removeEventListener('mousedown', onDown);
      document.removeEventListener('scroll', () => setMenu(null), true);
    };
  }, [ready, reachable, findBlockForSelection, menu]);

  const openAt = useCallback(async (line: number, col?: number) => {
    const r = rel();
    if (!r) return;
    setStatus('opening');
    try {
      const suffix = ':' + line + ':' + (col ?? 1);
      const resp = await fetch('http://localhost:' + port + '/api/open-in-vscode?rel=' + encodeURIComponent(r + suffix));
      const j = await resp.json().catch(() => ({}));
      if (!resp.ok || !j.ok) throw new Error(j.error || 'HTTP ' + resp.status);
      setStatus('done');
      setMenu(null);
      window.setTimeout(() => setStatus('idle'), 2500);
    } catch (e) {
      setStatus('error');
      setErrMsg(String((e as Error).message));
      setMenu(null);
      window.setTimeout(() => setStatus('idle'), 4000);
    }
  }, [rel, port]);

  const openTop = useCallback(() => void openAt(0), [openAt]);

  if (!ready || !reachable) return null;

  return (
    <>
      <div className={styles.vscodeBar}>
        <span className={styles.wysiwygLabel}><FaCode /> 本地开发</span>
        <button
          type="button"
          className={styles.wysiwygBtn + ' ' + styles.vscode}
          onClick={openTop}
          disabled={status === 'opening'}
          title="复用已打开的 VS Code 窗口, 跳到本文件标题行"
        >
          <FaExternalLinkAlt /> 在 VS Code 中打开
        </button>
        <span className={styles.vscodeHint}>
          选中正文文字后右键, 可精确跳到 VS Code 对应行
        </span>
        {status === 'done' && <span className={styles.wysiwygStatus + ' ' + styles.ok}>已跳转</span>}
        {status === 'error' && (
          <span className={styles.wysiwygStatus + ' ' + styles.err} title={errMsg}>跳转失败</span>
        )}
      </div>

      {menu && (
        <div
          data-hx-vsc-menu="1"
          className={styles.vscMenu}
          style={{ left: menu.x, top: menu.y }}
        >
          <button
            type="button"
            className={styles.vscMenuItem}
            onClick={() => void openAt(menu.line, menu.col)}
          >
            <FaHighlighter /> 跳转到 VS Code
            <span className={styles.vscMenuLoc}>L{menu.line}:C{menu.col}</span>
          </button>
        </div>
      )}
    </>
  );
}
