import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import React, { type ReactNode, useMemo, useState } from 'react';
import { FaExternalLinkAlt, FaArrowRight, FaArrowLeft } from 'react-icons/fa';

import { noteRefGraph, noteRefTitles, noteRefPermalinks } from '@site/data/noteReferences';
import styles from './styles.module.css';

/**
 * 笔记底部的「引用关系」方框。
 *
 * 取代原来手写的 `## 0x0A 参考来源` 章节 —— 手写的参考来源有三个问题:
 *   1. 读者读完就忘, 而它本来是**图**, 一眼能看出方向 (引了谁 / 谁引了这篇);
 *   2. 必须人工与正文同步, 必然漂移;
 *   3. 分不出站内与站外。
 *
 * 数据由 plugins/note-references-plugin.mjs 在构建期从正文抽出来。
 */
export default function NoteReferences (): ReactNode {
    const { frontMatter } = useDoc();
    const fm = frontMatter as Record<string, unknown>;
    const hxid = typeof fm.hxid === 'string' ? fm.hxid : undefined;
    const [tab, setTab] = useState<'out' | 'in'>('out');
    // Hook 必须在组件顶层调用 —— 放进 renderInternal 那种回调里会违反 hooks 规则。
    const withBase = useBaseUrl('/knowledge-base/');

    const data = useMemo(() => {
        if (!hxid || !noteRefGraph[hxid]) return { out: [], in: [], ext: [] };
        return noteRefGraph[hxid];
    }, [hxid]);

    const titleOf = (id: string) => noteRefTitles[id] ?? id;

    // 站内引用渲染成可点链接; 名字从 hxid 反查标题
    const renderInternal = (ids: string[]) => (
        <ul className={styles.list}>
            {ids.map((id) => {
                const slug = noteRefPermalinks[id];
                // slug 缺失说明那篇没有 index.md (只存在 .hx-info.md), 没有页面可跳 —— 退化成纯文本。
                return (
                    <li key={id}>
                        {slug
                            // 不写死 "/knowledge-base/...": 本站双平台部署, baseUrl 在
                            // GitHub Pages 是 "/HXLoLi"、Cloudflare 是 "/", 写死会在其中一边 404。
                            // 交给 Docusaurus 的 useBaseUrl 拼前缀。
                            ? <Link to={`${withBase}${slug}`} className={styles.link}>{titleOf(id)}</Link>
                            : <span className={styles.plain} title="该笔记没有对外页面">{titleOf(id)}</span>}
                    </li>
                );
            })}
        </ul>
    );

    const renderExternal = (urls: string[]) => (
        <ul className={styles.list}>
            {urls.map((u) => {
                let host = u;
                try { host = new URL(u).hostname.replace(/^www\./, ''); } catch {}
                return (
                    <li key={u}>
                        <a href={u} target="_blank" rel="noreferrer" className={styles.link}>
                            <FaExternalLinkAlt size={10} className={styles.icon} />
                            {host}
                        </a>
                        <span className={styles.url}>{u}</span>
                    </li>
                );
            })}
        </ul>
    );

    const internalCount = tab === 'out' ? data.out.length : data.in.length;
    if (data.out.length === 0 && data.in.length === 0 && data.ext.length === 0) return null;

    return (
        <section className={styles.box} aria-label="引用关系">
            <div className={styles.tabs} role="tablist">
                <button
                    role="tab"
                    aria-selected={tab === 'out'}
                    className={tab === 'out' ? styles.tabActive : styles.tab}
                    onClick={() => setTab('out')}
                >
                    <FaArrowRight size={10} /> 本文引用 ({data.out.length})
                </button>
                <button
                    role="tab"
                    aria-selected={tab === 'in'}
                    className={tab === 'in' ? styles.tabActive : styles.tab}
                    onClick={() => setTab('in')}
                >
                    <FaArrowLeft size={10} /> 本文被引用 ({data.in.length})
                </button>
            </div>

            <div className={styles.panel}>
                {internalCount > 0
                    ? renderInternal(tab === 'out' ? data.out : data.in)
                    : (
                        <p className={styles.empty}>
                            {tab === 'out' ? '本文没有引用站内其它笔记。' : '还没有别的笔记引用本文。'}
                        </p>
                    )}

                {tab === 'out' && data.ext.length > 0 && (
                    <>
                        <div className={styles.divider}>站外来源 ({data.ext.length})</div>
                        {renderExternal(data.ext)}
                    </>
                )}
            </div>
        </section>
    );
}