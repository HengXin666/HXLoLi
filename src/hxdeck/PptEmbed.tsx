import React, { useEffect, useMemo, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import { Deck } from './Deck';
import { Slide } from './Slide';
import { Cover } from './blocks';
import { loadTheme, loadThemeFile, getTheme, loadManifest, defaultTheme, listThemes } from './theme/registry';
import type { ThemeInfo } from './theme/registry';
import type { DeckTheme } from './theme/types';
import type { PptDirective } from './syntax';
import { deckModules, findDeckKey } from './decks.generated';
import { PptCard } from './PptCard';
import { Dropdown } from './Dropdown';
import './decks-builtin';

/**
 * 博客内嵌演示页.
 *
 * 两类内容, **同一套卡片 UI**:
 *   [标题 #ppt](x.html)     通用本地 HTML   -> 卡片里放 iframe
 *   [标题 ##PPT##](x.tsx)   我们的演示页     -> 卡片里放内联 React deck
 *
 * 两者的工具栏、打开遮罩、放大弹层、新标签页按钮完全一致
 * (外观由 PptCard.module.css 统一提供 —— 弹层走 Portal, 复用别人的 module 会失效,
 *  详见该 CSS 顶部说明).
 * 唯一区别: .tsx 的工具栏右侧多一个主题下拉 —— 因为它是内联 React, 可以实时换主题;
 * iframe 里的内容换不了.
 */

export interface PptEmbedProps extends PptDirective {
    /** 指向 .html 时用 iframe */
    src?: string;
    title?: string;
    width?: string;
    height?: string;
    /** 是否显示主题下拉 (默认显示) */
    switcher?: boolean;
    /** 是否显示左侧目录 */
    nav?: boolean;
    /** .tsx 演示页的路径 */
    deckName?: string;
    /** 兜底: 直接给 Slide 列表 */
    children?: React.ReactNode;
    /** 笔记所在目录 (相对 ai-docs) */
    noteDir?: string;
}

/** 从路由推出笔记目录 (相对 ai-docs) */
function useNoteDir(): string | undefined {
    const { pathname } = useLocation();
    const i = pathname.indexOf('/knowledge-base/');
    if (i < 0) return undefined;
    const rest = decodeURI(pathname.slice(i + '/knowledge-base/'.length));
    const segs = rest.split('/').filter(Boolean);
    segs.pop();
    return segs.join('/');
}

/** 顶栏显示的短名: 只取文件名并去扩展名 */
function shortName(p?: string): string {
    if (!p) return '';
    let s = p;
    try { s = decodeURI(p); } catch { /* 保留原样 */ }
    const last = s.split(/[\\/]/).filter(Boolean).pop() || s;
    return last.replace(/\.tsx$/i, '');
}

/** 站点前缀, 用于拼新标签页地址 */
function siteBase(): string {
    if (typeof window === 'undefined') return '';
    return window.location.pathname.split('/').slice(0, 2).join('/') || '';
}

/**
 * 把"笔记里的相对 .html 路径"解析成真正能取到文件的绝对 URL.
 *
 * 为什么必须显式解析 (一个真实踩坑):
 *   Docusaurus 文档的**规范 URL 不带尾斜杠**。浏览器对这种 URL 解析相对链接时,
 *   会先砍掉最后一段再拼接 ——
 *     /knowledge-base/…/001-CF过盾工程        + cf-gateway-pipeline.html
 *       -> /knowledge-base/…/cf-gateway-pipeline.html      (错了! 少了 001-… 这一层)
 *   而带尾斜杠时才是对的。于是同一篇笔记:
 *     · 从正文点进 (带尾斜杠) -> 卡片正常;
 *     · 刷新 / 从侧边栏直达 (无尾斜杠) -> iframe 404, 站点再把它 302 到首页,
 *       表现为"整块演示页变成网站首页"。
 *
 * 旧组件 PptHtmlViewer 里有一份 resolvePptSrc 专门处理这件事, 换成 PptEmbed 时丢了 ——
 * 这里补回来: 把相对路径按"当前笔记的目录"重新拼绝对地址。
 */
export function resolveSidecarSrc(src: string, pathname: string): string {
    // 协议 / 协议相对 / 纯锚点 / 已是绝对路径: 原样返回
    if (!src || /^(?:[a-z][a-z\d+.-]*:|\/\/|#|\/)/i.test(src)) return src;

    /*
      关键: 把当前 pathname 当作**目录**来拼, 即在末尾补一个斜杠.
      无尾斜杠时浏览器会把最后一段当文件名, 相对链接因此掉一层 —— 这正是 bug 本身.
    */
    const dir = pathname.replace(/\/$/, '') + '/';
    return dir + src;
}

export function PptEmbed({
    src, index, theme, title, deckName, noteDir, children, nav,
    width = '100%',
    switcher = true,
}: PptEmbedProps): React.ReactElement {
    const { pathname } = useLocation();
    const autoNoteDir = useNoteDir();
    const resolvedNoteDir = noteDir ?? autoNoteDir;
    /** 侧车 .html 的绝对地址 —— 必须按"当前笔记目录"解析, 见 resolveSidecarSrc */
    const resolvedSrc = useMemo(() => resolveSidecarSrc(src ?? '', pathname), [src, pathname]);

    const [t, setT] = useState<DeckTheme | null>(() => (theme ? getTheme(theme) ?? null : defaultTheme()));
    const [err, setErr] = useState('');
    const [slides, setSlides] = useState<React.ReactNode>(null);
    const [deckKey, setDeckKey] = useState('');
    const [themes, setThemes] = useState<ThemeInfo[]>([]);

    // 主题
    useEffect(() => {
        if (!theme) { setT(defaultTheme()); return; }
        const hit = getTheme(theme);
        if (hit) { setT(hit); return; }
        let alive = true;
        // loadTheme 内部会先确保清单就绪, 因此这里不必再串一遍
        loadTheme(theme)
            .then((got) => {
                if (!alive) return;
                if (got) setT(got);
                else { setErr(`主题 "${theme}" 未找到, 已用默认主题`); setT(defaultTheme()); }
            });
        return () => { alive = false; };
    }, [theme]);

    // 内容
    useEffect(() => {
        if (children || !deckName) return;
        const key = findDeckKey(deckName, resolvedNoteDir);
        if (!key) { setErr(`找不到演示页: ${deckName}`); return; }
        setDeckKey(key);
        const entry = deckModules[key];
        setSlides(entry.slides ? entry.slides() : null);
    }, [deckName, resolvedNoteDir, children]);

    // 主题清单 (下拉用)
    useEffect(() => {
        if (!switcher) return;
        let alive = true;
        loadManifest().then(() => { if (alive) setThemes(listThemes()); });
        return () => { alive = false; };
    }, [switcher]);

    const content = useMemo(() => {
        if (children) return children;
        if (slides) return slides;
        return (
            <Slide title="未注册" chapter="">
                <Cover
                    eyebrow="PPT"
                    title={deckName ? `未注册的演示页: ${deckName}` : '未指定演示页'}
                    subtitle="请检查链接里的路径"
                />
            </Slide>
        );
    }, [children, slides, deckName]);

    /** 主题下拉: 仅 .tsx 有 (iframe 内容换不了主题) */
    const pickTheme = async (id: string) => {
        const hit = getTheme(id);
        if (hit) { setT(hit); return; }
        const info = themes.find((x) => x.id === id);
        const got = info?.file ? await loadThemeFile(info.file) : await loadTheme(id);
        if (got) setT(got);
    };

    /*
      主题下拉 —— 预览外框的右上角 (在"打开"/"新标签页"按钮之前), 与用户确认的
      "在外边框那里放一个下拉菜单来选择主题"一致; 放大弹层的工具栏里同样保留一份.

      实现: 自绘组件 —— 原生 <select> 的展开列表由浏览器渲染, CSS 定制不了,
            达不到站点风格.
      仅 .tsx 有: iframe 里的内容换不了主题.
    */
    const themeSelect = switcher && !src && themes.length > 1 ? (
        <Dropdown
            value={t?.id ?? ''}
            options={themes.map((x) => ({ value: x.id, label: x.name }))}
            onChange={pickTheme}
            label="选择主题"
            title="选择主题"
        />
    ) : null;

    const displayTitle = title || shortName(deckName) || '演示页';

    /*
      卡片在 URL 里的标识 (?ppt=).

      必须**唯一**: 标题可能重复 (同一份演示页被引用两次 / 两篇笔记同名侧车),
      只用标题做标识时刷新会把同名的全部打开. 这里用"演示页路径 or iframe src",
      两者都指向具体的一份内容; 标题只作为最后的兜底.
    */
    const cardId = deckName || src || displayTitle;

    /*
      分享链接里的 ?page=N: 打开这张卡片时直接落在那一屏.

      必须在**任何提前 return 之前**求值 —— 它是个 hook, 而卡片会在
      "iframe 分支 / 主题还没加载完"时提前返回, 放在后面会让 hook 数量
      随分支变化 (Rendered more hooks than during the previous render).

      只在"这张卡片确实被 ?ppt= 点名"时才采用 —— 否则页面里其它卡片会跟着
      跳到同一个页码, 而它们本该停在自己的初始页.
    */
    const sharedPage = useMemo(() => {
        if (typeof window === 'undefined') return null;
        const params = new URLSearchParams(window.location.search);
        const want = params.get('ppt');
        if (!want) return null;
        let asked = want;
        try { asked = decodeURI(want); } catch { /* 保留原样 */ }
        if (asked !== cardId && want !== cardId) return null;
        const raw = params.get('page');
        const n = Number(raw);
        return Number.isFinite(n) && n > 0 ? Math.floor(n) - 1 : null;
    }, [cardId]);

    // 通用本地 HTML: 内容用 iframe, 卡片 UI 与 .tsx 完全一致
    if (src) {
        // .html: 新标签页指向文件本身 (原组件行为), 不加主题下拉
        return <PptCard title={displayTitle} cardId={cardId} src={resolvedSrc} themeSelect={null} backdrop="#0f172a" newTabHref={resolvedSrc} />;
    }

    if (!t) {
        return <PptCard title={displayTitle} cardId={cardId} themeSelect={null} backdrop="#0f172a"><div style={{ aspectRatio: '16 / 9', display: 'grid', placeItems: 'center' }}>主题加载中…</div></PptCard>;
    }

    /*
      两个实例, 各司其职:
        · 预览态 interactive={false} —— 不接管滚轮/键盘, 让页面正常滚动;
          同时这条通道也告诉内部控件"现在是预览", 于是架构图不给页内全屏
          (否则浮层会被卡片的 stage scale() 裁住, 退不出去).
        · 放大态 interactive       —— 接管, 可以翻页/快捷键/缩放看图.
      这是"卡片上滚不动页面"与"预览里放大退不出来"两个问题的正解.
    */
    const deckProps = {
        theme: t,
        index: sharedPage ?? index,
        showNav: nav !== false,
        showBrand: false,
        syncUrl: false,
    } as const;

    return (
        <PptCard
            title={displayTitle}
            cardId={cardId}
            themeSelect={themeSelect}
            newTabHref={deckKey ? siteBase() + '/ppt?' + new URLSearchParams({ deck: deckKey, page: String((index ?? 0) + 1), ...(t ? { theme: t.id } : {}) }).toString() : undefined}
            backdrop={t.colors.bg}
            // 预览态: 静态展示, 不接管滚轮
            children={<>{err ? <div style={{ padding: '6px 14px', fontSize: 12, opacity: 0.8 }}>{err}</div> : null}<Deck {...deckProps} interactive={false}>{content}</Deck></>}
            // 放大态: 接管交互 (翻页 / 快捷键)
            childrenActive={<Deck {...deckProps} interactive>{content}</Deck>}
        />
    );
}

export default PptEmbed;