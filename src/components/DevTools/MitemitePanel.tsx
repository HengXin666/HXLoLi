import React, { useCallback, useEffect, useState } from 'react';
import { FaChevronDown, FaCode, FaPencilAlt, FaSave, FaStickyNote, FaUndo } from 'react-icons/fa';
import { useDevService } from './useDevService';
import styles from './styles.module.css';

interface MitemiteBlock {
  seq: string;
  hash: string;
  q: string;
  a: string;
  startLine: number;
  qLine: number;
  aLine: number;
}

interface MitemitePanelProps {
  source?: string; // @site/... 相对路径
}

/**
 * MitemitePanel — 文章顶部的答题卡 (.hx-mitemite.md).
 * 可折叠区域, 默认折叠: 点击标题栏展开, 再点收起.
 * 展开后每个 Q/A block 渲染为独立带边框的盒子 (区分不同问题).
 * 局部编辑: 每个答案块独立「编辑」-> 该答案变 textarea, 保存只写回该答案 (POST /api/mitemite/answer);
 * 问题区只读, 右上角「在 VS Code 中改问题」跳转到 .hx-mitemite.md 对应行.
 * 本地 dev-edit-server 可达时渲染; 构建时永不渲染.
 */
export default function MitemitePanel ({ source }: MitemitePanelProps): React.ReactNode | null {
  const { ready, port } = useDevService();
  const [open, setOpen] = useState(false); // 默认折叠
  const [loaded, setLoaded] = useState(false);
  const [content, setContent] = useState<string | null>(null);
  const [blocks, setBlocks] = useState<MitemiteBlock[]>([]);
  const [editSeq, setEditSeq] = useState<string | null>(null); // 正在编辑答案的 block seq
  const [draft, setDraft] = useState('');
  const [status, setStatus] = useState<'idle' | 'loading' | 'saving' | 'saved' | 'error'>('idle');
  const [jumpStatus, setJumpStatus] = useState<'idle' | 'opening' | 'done' | 'error'>('idle');
  const [errMsg, setErrMsg] = useState('');

  const rel = useCallback(() => {
    if (!source) return null;
    const m = source.replace(/^@site\//, '').replace(/^\/+/, '');
    return m || null;
  }, [source]);

  // .hx-mitemite.md 与文章同目录
  const mitemiteRel = useCallback(() => {
    const r = rel();
    if (!r) return null;
    const dir = r.split('/').slice(0, -1).join('/');
    return (dir ? dir + '/' : '') + '.hx-mitemite.md';
  }, [rel]);

  const load = useCallback(async () => {
    const r = rel();
    if (!r || !ready) return;
    setStatus('loading');
    try {
      const resp = await fetch('http://localhost:' + port + '/api/mitemite?rel=' + encodeURIComponent(r));
      if (resp.status === 404) {
        setContent(null); setBlocks([]); setDraft(''); setLoaded(true); setStatus('idle');
        return;
      }
      if (!resp.ok) throw new Error('HTTP ' + resp.status);
      const j = await resp.json();
      setContent(j.content ?? '');
      setBlocks(Array.isArray(j.blocks) ? j.blocks : []);
      setLoaded(true);
      setStatus('idle');
    } catch (e) {
      setStatus('error');
      setErrMsg(String((e as Error).message || e));
    }
  }, [rel, ready, port]);

  // 首次展开时惰性加载
  useEffect(() => {
    if (ready && open && !loaded) void load();
  }, [ready, open, loaded, load]);

  const toggle = useCallback(() => {
    setOpen((o) => !o);
  }, []);

  // 只保存当前编辑的答案 (题目文本绝不经前端改动)
  const saveAnswer = useCallback(async (seq: string, hash: string) => {
    const r = rel();
    if (!r) return;
    setStatus('saving');
    try {
      const resp = await fetch('http://localhost:' + port + '/api/mitemite/answer', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ rel: r, seq, hash, answer: draft }),
      });
      if (!resp.ok) {
        const j = await resp.json().catch(() => ({}));
        throw new Error((j as { error?: string }).error || 'HTTP ' + resp.status);
      }
      setEditSeq(null);
      setStatus('saved');
      window.setTimeout(() => setStatus('idle'), 1500);
      void load();
    } catch (e) {
      setStatus('error');
      setErrMsg(String((e as Error).message || e));
    }
  }, [rel, port, draft, load]);

  // 跳到 .hx-mitemite.md 的某行 (问题在 qLine; 整体查看用 startLine)
  const openMitemiteAt = useCallback(async (line: number) => {
    const m = mitemiteRel();
    if (!m) return;
    setJumpStatus('opening');
    try {
      const resp = await fetch('http://localhost:' + port + '/api/open-in-vscode?rel=' + encodeURIComponent(m + ':' + line));
      const j = await resp.json().catch(() => ({}));
      if (!resp.ok || !j.ok) throw new Error((j as { error?: string }).error || 'HTTP ' + resp.status);
      setJumpStatus('done');
      window.setTimeout(() => setJumpStatus('idle'), 2500);
    } catch (e) {
      setJumpStatus('error');
      setErrMsg(String((e as Error).message || e));
      window.setTimeout(() => setJumpStatus('idle'), 4000);
    }
  }, [mitemiteRel, port]);

  if (!ready) return null;

  const renderMd = (s: string): string => {
    return s
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
      .replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>')
      .replace(/\`([^\`]+)\`/g, '<code>$1</code>')
      .replace(/\[(.+?)\]\((.+?)\)/g, '<a href="$2" target="_blank" rel="noreferrer">$1</a>')
      .replace(/\n/g, '<br/>');
  };

  const answeredCount = blocks.filter((b) => b.a && b.a.trim().length > 0).length;
  const allAnswered = blocks.length > 0 && answeredCount === blocks.length;

  return (
    <div className={styles.mitemiteWrap}>
      {/* 标题栏: 整条可点击, 展开/收起 */}
      <button
        type="button"
        data-mitemite-bar="1"
        aria-expanded={open}
        className={styles.mitemiteBar + (open ? ' ' + styles.mitemiteBarOpen : '')}
        onClick={toggle}
        title={open ? '收起答题卡' : '展开答题卡 (仅本地)'}
      >
        <FaChevronDown className={styles.mitemiteChevron} style={{ transform: open ? 'rotate(0deg)' : 'rotate(-90deg)' }} />
        <FaStickyNote /> 答题卡 · .hx-mitemite.md
        {loaded && content !== null && blocks.length > 0 && (
          <span className={styles.mitemiteCount}>{answeredCount}/{blocks.length} 已答</span>
        )}
        <span className={styles.mitemiteBarMeta}>仅本地渲染</span>
      </button>

      {/* 折叠内容区 */}
      {open && (
        <div className={styles.mitemiteBody}>
          <div className={styles.mitemiteBodyHead}>
            <span className={styles.mitemiteBodyTitle}>
              {loaded && blocks.length > 0
                ? (allAnswered ? '全部已答, 可并入正文' : '还有问题待回答')
                : '审核事项落盘处'
              }
            </span>
            <span className={styles.mitemiteActions}>
              {status === 'saved' && <span className={styles.mitemiteSaved}>已保存</span>}
              {status === 'error' && (
                <span className={styles.mitemiteErr} title={errMsg}>保存失败</span>
              )}
              {jumpStatus === 'done' && <span className={styles.mitemiteSaved}>已在 VS Code 打开</span>}
              {jumpStatus === 'error' && (
                <span className={styles.mitemiteErr} title={errMsg}>跳转失败</span>
              )}
            </span>
          </div>

          {status === 'loading' ? (
            <div className={styles.mitemiteLoading}>加载中...</div>
          ) : loaded && content === null ? (
            <div className={styles.mitemiteEmpty}>
              <FaStickyNote /> 本文章还没有 .hx-mitemite.md
            </div>
          ) : (
            <div className={styles.mitemiteBlocks}>
              {blocks.length === 0 ? (
                <div className={styles.mitemiteEmpty}>文件无 Q/A block</div>
              ) : (
                blocks.map((b) => {
                  const hasA = !!(b.a && b.a.trim().length > 0);
                  const editing = editSeq === b.seq;
                  return (
                    <div key={b.seq + b.hash} data-mitemite-block="1" className={styles.mitemiteBlock}>
                      <div className={styles.mitemiteBlockHead}>
                        <span className={styles.mitemiteBlockTitle}>
                          <span className={styles.mitemiteSeq}>{b.seq}</span>
                          <span className={styles.mitemiteHash}>#{b.hash}</span>
                        </span>
                        <span className={styles.mitemiteActions}>
                          <span className={hasA ? styles.mitemiteAnswered : styles.mitemitePendingTag}>
                            {hasA ? '已答' : '待答'}
                          </span>
                          <button
                            type="button"
                            className={styles.mitemiteIconBtn}
                            onClick={() => void openMitemiteAt(b.qLine)}
                            disabled={jumpStatus === 'opening'}
                            title={'在 VS Code 中修改该问题 (跳到 .hx-mitemite.md 第' + b.qLine + '行)'}
                            data-hx-vsc-qedit="1"
                          >
                            <FaCode /> 改问题
                          </button>
                          {!editing ? (
                            <button
                              type="button"
                              className={styles.mitemiteIconBtn}
                              onClick={() => { setEditSeq(b.seq); setDraft(b.a ?? ''); }}
                              title="只编辑该答案"
                            >
                              <FaPencilAlt /> 编辑答案
                            </button>
                          ) : (
                            <>
                              <button
                                type="button"
                                className={styles.mitemiteIconBtn}
                                onClick={() => void saveAnswer(b.seq, b.hash)}
                                disabled={status === 'saving'}
                              >
                                <FaSave /> {status === 'saving' ? '保存中' : '保存'}
                              </button>
                              <button
                                type="button"
                                className={styles.mitemiteIconBtn}
                                onClick={() => { setEditSeq(null); setDraft(''); }}
                                title="取消"
                              >
                                <FaUndo />
                              </button>
                            </>
                          )}
                        </span>
                      </div>
                      <div className={styles.mitemiteQA}>
                        <div className={styles.mitemiteQLabel}>问题 (只读, 改问题请在 VS Code 中)</div>
                        <div className={styles.mitemiteQBody} dangerouslySetInnerHTML={{ __html: renderMd(b.q) }} />
                        <div className={styles.mitemiteALabel}>答案</div>
                        {editing ? (
                          <textarea
                            className={styles.mitemiteTextarea}
                            value={draft}
                            onChange={(e) => setDraft(e.target.value)}
                            spellCheck={false}
                            aria-label={b.seq + ' 答案'}
                          />
                        ) : hasA ? (
                          <div className={styles.mitemiteABody} dangerouslySetInnerHTML={{ __html: renderMd(b.a) }} />
                        ) : (
                          <div className={styles.mitemiteAPending}>待填写… (点「编辑答案」填写)</div>
                        )}
                      </div>
                    </div>
                  );
                })
              )}
            </div>
          )}
        </div>
      )}
    </div>
  );
}
