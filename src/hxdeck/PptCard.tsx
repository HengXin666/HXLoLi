import React, { useEffect, useRef, useState } from 'react';
import ReactDOM from 'react-dom';
import { FaCompress, FaExpand, FaExternalLinkAlt, FaPlay, FaTimes } from 'react-icons/fa';
/*
  自带的完整样式 (不再复用 PptHtmlViewer 的 CSS module).
  原因见 PptCard.module.css 顶部: 放大弹层是 Portal 到 body 的, 那种模块化 class
  在 Portal 子节点里解析成 undefined, 弹层会整个失去外观。
*/
import styles from './PptCard.module.css';
import { PptCardIdProvider } from './deck-layer';

/**
 * 演示页卡片 —— 与老 #ppt 的 PptHtmlViewer **外观一致**的 .tsx 版本.
 *
 * 两类内容共用同一套卡片 UI: 同样的工具栏、同样的"打开"遮罩、同样的放大弹层、
 * 同样的新标签页按钮. 唯一区别: .tsx 的工具栏右侧多一个**主题下拉**
 * (因为 .tsx 是内联 React, 可以实时换主题; iframe 里的内容换不了).
 *
 * 外观现在由自带的 PptCard.module.css 提供 (而非 import PptHtmlViewer 的 module),
 * 原因见该 CSS 顶部注释 —— 一句话: 弹层走 Portal, 别人的 module class 在里面会失效.
 */
/**
 * 已被 URL 的 ?ppt= 认领过的卡片标识.
 *
 * 为什么需要: 页面里可能有多张卡片**标识相同** (同一份演示页在一篇笔记里被
 * 引用多次). 只用标识比对时, 刷新会命中所有同标识卡片, 于是"一次打开全部弹层".
 *
 * 规则: 同一个标识只允许**第一张**卡片认领, 其余保持关闭.
 *
 * 注意: 认领只能发生在 effect 里, 不能放在 useState 的初始化器里 ——
 * 初始化器属于渲染阶段, React 可能渲染多次而只提交一次 (hydration / 并发渲染),
 * 在渲染里改这个 Set 会让"第一次渲染认领成功、提交的那次却看到已被认领"
 * 从而判定为不打开 (实测: 直接带 ?ppt= 刷新, 一张都打不开).
 */
const claimedPptIds = new Set<string>();

export interface PptCardProps {
    title: string;
    /**
     * 卡片在 URL 里的唯一标识 (?ppt=<id>).
     *
     * 默认回退到 title —— 但**同名卡片必须由调用方传一个真正唯一的 id**
     * (如演示页路径 / iframe 的 src), 否则刷新会把同名的全部打开.
     */
    cardId?: string;
    /**
     * 放大的底页颜色.
     *
     * 为什么必须显式传:
     *   弹层用 Portal 挂到 document.body, 因此**继承不到** deck 上的 CSS 变量
     *   (`--hxd-color-bg` 之类). 之前写 `var(--hxd-color-bg, #0f172a)` 会拿不到值,
     *   底页实际是透明的 -> 架构图的半透明背景就透出了后面的正文.
     *   这里由调用方把当前主题的底色算好传进来.
     */
    backdrop?: string;
    /** 指向 .html 时用 iframe 渲染 */
    src?: string;
    /** 预览态的 .tsx 内容 (静态展示, 不接管滚轮) */
    children?: React.ReactNode;
    /** 工具栏右侧的主题下拉 (只有 .tsx 演示页会传; iframe 内容换不了主题) */
    themeSelect?: React.ReactNode;
    /**
     * 放大态的内容.
     *
     * 为什么要分两份:
     *   预览态必须"不接管滚轮", 否则用户在卡片上滚动会被 deck 拿去翻页,
     *   页面纹丝不动 (实测复现). 放大态反而需要接管.
     *   同一棵树没法同时满足, 所以按状态给不同的实例.
     */
    childrenActive?: React.ReactNode;
    /** 预览区是否可点击打开 */
    clickable?: boolean;
    /** "新标签页打开"的目标: .html 给文件本身; .tsx 给独立播放页 */
    newTabHref?: string;
}

export function PptCard({
    title, cardId, src, children, childrenActive, themeSelect, backdrop, clickable = true, newTabHref,
}: PptCardProps): React.ReactElement {
    // "新标签页打开"的目标: .html 给文件本身; .tsx 给独立播放页
    const openHref = newTabHref || src || '#';
    /** URL 里代表本卡片的标识: 优先用调用方给的唯一 id, 否则退回标题 */
    const pptId = cardId || title;

    const [isOpen, setIsOpen] = useState(false);

    /*
      支持用 URL 直接打开本卡片: ?ppt=<本卡片的标识>

      用途:
        · 分享"我已经翻到第 3 页了"这种状态
        · 从笔记跳到一个干净的放大视图

      互斥认领放在 effect 里 (每次提交只跑一次), 同一个标识只让第一张卡片打开.
      带 ?ppt= 的分享链接因此不会被"谁先渲染"这种不确定因素左右.
    */
    useEffect(() => {
        if (typeof window === 'undefined') return;
        const want = new URLSearchParams(window.location.search).get('ppt');
        if (!want) return;
        let asked = want;
        try { asked = decodeURI(want); } catch { /* 保留原样 */ }
        if (asked !== pptId && want !== pptId) return;
        if (claimedPptIds.has(pptId)) return;
        claimedPptIds.add(pptId);
        setIsOpen(true);
        // 卸载时归还认领名额, 免得 SPA 内导航回来后再也打不开
        return () => { claimedPptIds.delete(pptId); };
    }, [pptId]);
    const [isFullscreen, setIsFullscreen] = useState(false);
    const shellRef = useRef<HTMLDivElement | null>(null);
    /** 标记"这次退出全屏是本组件自己发起的" (Esc 分支), 用于区分外部撤销 */
    const selfExitRef = useRef(false);
    const isIframe = Boolean(src);

    useEffect(() => {
        if (!isOpen) return;
        const prev = document.body.style.overflow;
        const onKey = (e: KeyboardEvent) => {
            if (e.key !== 'Escape') return;
            /* 全屏优先: 第一次 Esc 退出浏览器全屏 (弹层保留), 第二次才关弹层。 */
            if (document.fullscreenElement) {
                void document.exitFullscreen();
                return;
            }
            setIsOpen(false);
            syncUrl(false);
        };
        const onFs = () => {
            const on = document.fullscreenElement === shellRef.current;
            setIsFullscreen(on);
            /*
              浏览器全屏被外部撤销 (Esc / F11 / 系统手势) 时, 弹层要跟着收掉。
              否则会出现"全屏已退出, 但仍有一层最大化的弹层压在页面上, 退出不明显" ——
              这正是用户报的那个隐患。
              但自己按 Esc 时由 onKey 先退全屏, 这里不能再关弹层, 否则一次 Esc 全没了,
              所以用这个 ref 标记"这次是脚本主动退出"。

              判据必须是"**没有任何元素**处于全屏", 而不是"弹层自己不是全屏元素":
              弹层里的 iframe (老 #ppt 语法) 或演示页自己请求全屏时, fullscreenElement
              会变成那个内层元素 —— 那不是撤销, 弹层不该跟着消失。
            */
            if (!document.fullscreenElement && !selfExitRef.current) {
                setIsOpen(false);
                syncUrl(false);
            }
            selfExitRef.current = false;
        };

        document.body.style.overflow = 'hidden';
        document.addEventListener('keydown', onKey);
        document.addEventListener('fullscreenchange', onFs);
        return () => {
            document.body.style.overflow = prev;
            document.removeEventListener('keydown', onKey);
            document.removeEventListener('fullscreenchange', onFs);
        };
    }, [isOpen]);

    /** 把"打开了哪张卡片"写进 URL —— 刷新/分享可复原 */
    const syncUrl = (open: boolean) => {
        if (typeof window === 'undefined') return;
        const url = new URL(window.location.href);
        // 写的是唯一标识, 刷新时才能精确只打开这一张
        if (open) url.searchParams.set('ppt', pptId);
        else url.searchParams.delete('ppt');
        window.history.replaceState(window.history.state, '', url.toString());
    };

    /*
      手动点开卡片时, 顺手清掉上一次分享链接留下的 ?page= / ?zoom=.

      这两个参数只在"从分享链接进来"时才有意义 (由下面的 effect 处理, 不经过这里).
      不清的话, 读者关掉卡片后再点开, 会莫名其妙跳到别人分享的那个页码、
      甚至架构图一进来就是放大态 —— 与他的点击意图不符.
    */
    const openModal = () => {
        if (typeof window !== 'undefined') {
            const url = new URL(window.location.href);
            url.searchParams.delete('page');
            url.searchParams.delete('zoom');
            window.history.replaceState(window.history.state, '', url.toString());
        }
        setIsOpen(true);
        syncUrl(true);
    };
    const closeModal = () => {
        // 全屏中先退全屏, 再关弹层 —— 与原组件一致 (标记为主动退出, 免得 onFs 重复处理)
        if (document.fullscreenElement === shellRef.current) {
            selfExitRef.current = true;
            void document.exitFullscreen();
        }
        setIsOpen(false);
        syncUrl(false);
    };

    /**
     * 全屏 —— 与原组件完全一致的实现.
     *
     * 两种内容 (.html 的 iframe / .tsx 的内联 deck) 走同一条路径:
     * 让 modalShell 自己 requestFullscreen.
     * 之前给 .tsx 另造了"页内浮层放大", 反而出现"整页变成演示页且退不出去"的问题.
     */
    const toggleExpand = () => {
        const shell = shellRef.current;
        if (!shell) return;
        if (document.fullscreenElement === shell) {
            selfExitRef.current = true;
            void document.exitFullscreen();
            return;
        }
        void shell.requestFullscreen();
    };

    /*
      卡片身份下发给内容: 图表 (Diagram) 生成分享链接时要知道 ?ppt= 写什么,
      否则"复制链接"只能给到页码, 对方打开还在正文缩略图上.
    */
    const withId = (node: React.ReactNode): React.ReactNode => (
        <PptCardIdProvider value={pptId}>{node}</PptCardIdProvider>
    );

    const body = isIframe ? (
        <iframe className={styles.iframe} src={src} title={title} loading="lazy" allow="fullscreen" allowFullScreen />
    ) : (
        withId(children)
    );

    return (
        <>
            <span className={styles.viewer}>
                <span className={styles.toolbar}>
                    <span className={styles.title} title={title}>{title}</span>
                    <span className={styles.actions}>
                        {/* 主题下拉: 与右侧按钮同风格, 不用浏览器原生外观 */}
                        {themeSelect ? <span className={styles.themeSelect}>{themeSelect}</span> : null}
                        <button
                            type="button"
                            className={styles.actionButton}
                            onClick={openModal}
                            title="打开"
                            aria-label={`打开 ${title}`}
                        >
                            <FaPlay aria-hidden="true" />
                        </button>
                        {/*
                          新标签页打开: .html 指向文件本身; .tsx 指向独立播放页.
                          两种内容都提供 —— 这是原组件就有的能力, 不该被省掉.
                        */}
                        <a
                            className={styles.actionButton}
                            href={openHref}
                            target="_blank"
                            rel="noopener noreferrer"
                            title="新标签页打开"
                            aria-label={`新标签页打开 ${title}`}
                        >
                            <FaExternalLinkAlt aria-hidden="true" />
                        </a>
                    </span>
                </span>

                <span
                    className={styles.preview}
                    onClick={clickable ? openModal : undefined}
                    role={clickable ? 'button' : undefined}
                    tabIndex={clickable ? 0 : undefined}
                    onKeyDown={(e) => {
                        if (!clickable) return;
                        if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); openModal(); }
                    }}
                    aria-label={`打开 ${title}`}
                    style={isIframe ? undefined : { cursor: 'default' }}
                >
                    {body}
                    <span className={styles.previewOverlay}>
                        <FaPlay aria-hidden="true" />
                        <span>打开</span>
                    </span>
                </span>
            </span>

            {isOpen
                ? ReactDOM.createPortal(
                    <div className={styles.modalOverlay} onMouseDown={closeModal}>
                        <div
                            ref={shellRef}
                            className={styles.modalShell}
                            /*
                              遮罩关闭挂在 overlay 上, 内容区自己吞掉 mousedown;
                              但 mousedown 不是点击 —— 从弹层里按下鼠标、拖到遮罩上再松手,
                              浏览器会把 click 派发给 overlay, 于是"一拖动就关掉"。
                              这里再补一层 click 拦截, 只关心从遮罩本身发起的点击。
                            */
                            onClick={(e) => e.stopPropagation()}
                            onMouseDown={(e) => e.stopPropagation()}
                        >
                            <div className={styles.modalToolbar}>
                                <span className={styles.modalTitle} title={title}>{title}</span>
                                <div className={styles.modalActions}>
                                    {themeSelect ? <span className={styles.themeSelect}>{themeSelect}</span> : null}
                                    <a
                                        className={styles.modalButton}
                                        href={openHref}
                                        target="_blank"
                                        rel="noopener noreferrer"
                                        title="新标签页打开"
                                        aria-label={`新标签页打开 ${title}`}
                                    >
                                        <FaExternalLinkAlt aria-hidden="true" />
                                    </a>
                                    <button
                                        type="button"
                                        className={styles.modalButton}
                                        onClick={toggleExpand}
                                        title={isFullscreen ? '退出放大' : '放大'}
                                        aria-label={isFullscreen ? '退出放大' : '放大'}
                                    >
                                        {isFullscreen ? <FaCompress aria-hidden="true" /> : <FaExpand aria-hidden="true" />}
                                    </button>
                                    <button
                                        type="button"
                                        className={styles.modalButton}
                                        onClick={closeModal}
                                        title="关闭"
                                        aria-label="关闭"
                                    >
                                        <FaTimes aria-hidden="true" />
                                    </button>
                                </div>
                            </div>
                            {/*
                              底页颜色直接给 modalFrame.

                              之前在这里叠了两个 position:absolute; inset:0 的层
                              (一个当"底页", 一个当"前景层"), 结果是:
                                · 它们脱离文档流, 盖住了真正的 deck ——
                                  鼠标事件全落在空层上, 于是"点击无反应、滚动失效"
                                · deck 被压在两层之间, 架构图的半透明背景仍不对
                              正确做法: 底页就是 modalFrame 自己的背景色, 不需要额外元素.
                            */}
                            <div className={styles.modalFrame} style={backdrop ? { background: backdrop } : undefined}>
                                {isIframe ? (
                                    <iframe className={styles.iframe} src={src} title={title} allow="fullscreen" allowFullScreen />
                                ) : (
                                    <span className={styles.deckHolder}>{withId(childrenActive ?? children)}</span>
                                )}
                            </div>
                        </div>
                    </div>,
                    document.body,
                )
                : null}
        </>
    );}