import Link from '@docusaurus/Link';
import { useBaseUrlUtils } from '@docusaurus/useBaseUrl';
import React, { type ReactNode, useState } from 'react';
import { FaExternalLinkAlt, FaArrowRight, FaArrowLeft, FaBullseye } from 'react-icons/fa';

import styles from './styles.module.css';

/** [渲染后的 permalink, 标题]  permalink 已由构建期解析好, 这里不再拼路径 */
type Edge = [string, string];
/** [URL, 锚文本]  锚文本空串表示原文没给 */
type ExternalEdge = [string, string];

export interface NoteReferencesData {
    out: Edge[];
    in: Edge[];
    ext: ExternalEdge[];
}

type TabKey = 'out' | 'in' | 'ext';

/**
 * 载荷经 remark 以 JSON 字符串注入, 这里解一次。
 *
 * 为什么不做成对象属性: 见 plugins/note-references-plugin.mjs 里对"为什么是字符串"的说明
 * (MDX 3 只在属性表达式自带 estree 时才序列化它, 否则静默输出空)。
 *
 * 解析失败时退化成空载荷而不是抛错  引用框是页面附属信息, 它不该让整篇文档白屏。
 */
function parseData (raw: unknown): NoteReferencesData {
    if (raw && typeof raw === 'object') return raw as NoteReferencesData;
    if (typeof raw !== 'string') return { out: [], in: [], ext: [] };
    try {
        const parsed = JSON.parse(raw);
        return parsed && typeof parsed === 'object' ? parsed as NoteReferencesData : { out: [], in: [], ext: [] };
    } catch {
        return { out: [], in: [], ext: [] };
    }
}

/**
 * 笔记底部的「引用关系」方框。
 *
 * 取代原来手写的 `## 0x0A 参考来源` 章节  手写的参考来源有三个问题:
 *   1. 读者读完就忘, 而它本来是**图**, 一眼能看出方向 (引了谁 / 谁引了这篇);
 *   2. 必须人工与正文同步, 必然漂移;
 *   3. 分不出站内与站外, 也就分不出"延伸阅读"和"结论的出处"。
 *
 * 三个方向各占一个页签, 各自带自己那个数字:
 *   本文引用 (出边, 站内) / 本文被引用 (入边) / 站外来源 (出边, 站外)。
 *
 * 为什么站外来源要独立成页签而不是塞在"本文引用"下面: 它们过去共用一个数字, 于是
 * 出现"本文引用 (0)"底下挂着九条来源的自相矛盾  计数和内容说的是两件事, 读者只能
 * 怀疑功能坏了。数字必须数它下面真正列出来的东西。
 *
 * 数据由 plugins/note-references-plugin.mjs 在构建期从正文抽出, 以 props 注入 
 * 不 import 全站表, 见该文件的说明。
 */
export default function NoteReferences ({ data }: { data?: NoteReferencesData | string }): ReactNode {
    const [tab, setTab] = useState<TabKey>('out');
    // Hook 必须在组件顶层调用  放进回调里会违反 hooks 规则。
    /**
     * 引用关系方框改为全站注入, 站外来源按「域名@标题」显示
     * .agents/notes/implemented/architecture/2026-09-26-note-references-auto-rendered.md
     */
    const { withBaseUrl } = useBaseUrlUtils();

    const payload = parseData(data);
    const out = payload.out ?? [];
    const incoming = payload.in ?? [];
    const external = payload.ext ?? [];

    const tabs: { key: TabKey; label: string; icon: ReactNode; count: number }[] = [
        { key: 'out', label: '本文引用', icon: <FaArrowRight size={10} />, count: out.length },
        { key: 'in', label: '本文被引用', icon: <FaArrowLeft size={10} />, count: incoming.length },
        { key: 'ext', label: '站外来源', icon: <FaExternalLinkAlt size={10} />, count: external.length },
    ];

    const renderInternal = (edges: Edge[]) => (
        <ul className={styles.list}>
            {edges.map(([permalink, title], index) => (
                <li key={`${permalink}#${index}`}>
                    {/* permalink 是构建期算好的绝对路径, 交给 withBaseUrl 拼部署前缀:
                        本站双平台部署, baseUrl 在 GitHub Pages 是 "/HXLoLi"、Cloudflare 是 "/",
                        写死会在其中一边 404。 */}
                    <Link to={withBaseUrl(permalink)} className={styles.link}>{title || permalink}</Link>
                </li>
            ))}
        </ul>
    );

    /**
     * 站外来源显示的锚文本: `域名@标题`。
     *
     * 为什么不是整条 URL: 一条带 commit hash 的 GitHub 链接能有 90 字符, 列八条就把方框
     * 撑满, 而且真正有信息量的部分 (哪个域名、这篇文章叫什么) 全被路径噪声埋掉。
     * 完整 URL 仍然可达  挪进 title 属性, 悬停可见, 链接本身不变。
     */
    const renderExternal = (edges: ExternalEdge[]) => (
        <ul className={styles.list}>
            {edges.map(([url, label], index) => {
                let host = url;
                try { host = new URL(url).hostname.replace(/^www\./, ''); } catch {}
                const text = label ? `${host}@${label}` : host;
                return (
                    <li key={`${url}#${index}`}>
                        <a
                            href={url}
                            target="_blank"
                            rel="noreferrer"
                            className={styles.link}
                            title={url}
                        >
                            <FaExternalLinkAlt size={10} className={styles.icon} />
                            <span className={styles.extText}>{text}</span>
                        </a>
                    </li>
                );
            })}
        </ul>
    );

    const active = tabs.find((t) => t.key === tab) ?? tabs[0];
    const emptyHint: Record<TabKey, string> = {
        out: '本文没有引用站内其它文档。',
        in: '还没有别的文档引用本文。',
        ext: '本文没有引用站外来源。',
    };

    return (
        <section className={styles.box} aria-label="引用关系">
            <div className={styles.tabs} role="tablist">
                {tabs.map((item) => (
                    <button
                        key={item.key}
                        role="tab"
                        aria-selected={tab === item.key}
                        className={tab === item.key ? styles.tabActive : styles.tab}
                        onClick={() => setTab(item.key)}
                    >
                        {item.icon} {item.label} ({item.count})
                    </button>
                ))}
            </div>

            <div className={styles.panel}>
                {active.count > 0 ? (
                    active.key === 'ext'
                        ? renderExternal(external)
                        : renderInternal(active.key === 'out' ? out : incoming)
                ) : (
                    <p className={styles.empty}>
                        <FaBullseye size={10} className={styles.icon} /> {emptyHint[active.key]}
                    </p>
                )}
            </div>
        </section>
    );
}
