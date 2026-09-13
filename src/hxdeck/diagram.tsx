import React, { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';
import { useDeckTheme } from './theme/context';
import { Rise } from './blocks';
import { markWheelConsumed } from './wheel-lock';
import { useDeckInteractive, useIsSlideActive, useDeckPage } from './slide-state';
import { useDeckLayer, usePptCardId } from './deck-layer';
import { createPortal } from 'react-dom';
import { exportSvg, pickCssVars, type ExportFormat } from './export';
import { parseFacts, applyViewToDom, DiagramPassport, type PassportPos } from './diagram-focus';
import { DiagramLens, DiagramFinder, DiagramRoute } from './diagram-panels';
import { FaShareAlt, FaFileDownload, FaCheck } from 'react-icons/fa';

/**
 * 架构图 / 流程图控件族.
 *
 * 三条被反复踩出来的设计约束:
 *
 * 1. **容器尺寸固定, 只有内容缩放.**
 *    早期做法是给 <svg> 设 width: zoom*100% + height:auto, 于是放大时容器被撑高,
 *    整个版面高度跟着变 —— deck 侦测到尺寸变化就翻页, 观感上"缩放变成了翻页".
 *    现在容器高度由 viewBox 宽高比算出并锁死; 缩放只作用于内部画布.
 *
 * 2. **滚动边界必须真实.**
 *    内部画布用 width/height = zoom*100% 参与布局, 因此
 *    scrollWidth/scrollHeight 恰好等于缩放后的尺寸, 拖拽能真正摸到四边.
 *    若改用 transform: scale(), 布局尺寸不变, 滚动区就永远是 0 —— 那正是"拖不到边界"的原因.
 *
 * 3. **放大 = 占满"当前演示页", 不是占满整个网页.**
 *    浮层挂在**演示页/弹层的舞台容器**里 (见 deck-layer.tsx), 用 inset:0 定位.
 *    于是它的边界天生就是父布局, 内部布局不可能比父布局还大;
 *    CSS 变量/字体/圆角也都在, 不会出现"透明底 + 盖住整页 + 退出不明显".
 *    为什么不 Portal 到 document.body: 那样 inset:0 是**视口**, 卡片预览里会盖住
 *    整篇正文; 宿主在浏览器全屏里时, 全屏元素处于 top layer, body 上的浮层还会被压在
 *    它下面 (点了放大反而什么都看不见).
 */

export type DiagramKind = 'architecture' | 'workflow' | 'sequence' | 'dataflow' | 'lifecycle';

export interface DiagramAsset {
    svg: string;
    css: string;
    title?: string;
    viewBox?: string;
}

/**
 * 归一化图资产.
 *
 * 坑: webpack 在 ESM 下 import 一个同时含 default 与具名导出的模块时,
 * 拿到的可能是 Module 命名空间对象 (带 default 字段), 直接读 a.viewBox 会是 undefined,
 * 于是自动缩放静默失效 (表现为永远 100%).
 */
export function resolveAsset(input: DiagramAsset | string): DiagramAsset {
    if (typeof input === 'string') return { svg: input, css: '' };
    const anyIn = input as unknown as { default?: DiagramAsset };
    if (!(input as DiagramAsset).svg && anyIn.default?.svg) return anyIn.default;
    return input as DiagramAsset;
}

/** 从 archify HTML 直接抽取 (运行时用; 构建期建议用 scripts/extract-diagram.mjs) */
export function extractSvg(html: string): string {
    const m = /<svg[\s\S]*?<\/svg>/i.exec(html);
    return m ? m[0] : '';
}

/** 从 viewBox 解析宽高比 */
export function aspectOf(viewBox?: string, fallback = 16 / 9): number {
    const m = /[\d.]+\s+[\d.]+\s+([\d.]+)\s+([\d.]+)/.exec(viewBox ?? '');
    if (!m) return fallback;
    const w = parseFloat(m[1]);
    const h = parseFloat(m[2]);
    return w > 0 && h > 0 ? w / h : fallback;
}

/**
 * 把主题 token 映射成 archify 的变量名.
 * 这是"主题 <-> 图"的唯一边界: 换主题只改这里, 图与控件都不动.
 */
export function diagramVars(theme: ReturnType<typeof useDeckTheme>): Record<string, string> {
    const c = theme.colors;
    return {
        '--bg': c.bg,
        '--grid': c.border,
        '--text': c.text,
        '--text-muted': c.textMuted,
        '--text-dim': c.textMuted,
        '--text-faint': c.textMuted,
        '--panel': c.surface,
        '--panel-border': c.border,
        '--lane-fill': c.surface,
        '--lane-stroke': c.border,
        '--arrow': c.textMuted,
        '--arrow-emphasis': c.accent,
        '--mask': c.bg,
        '--frontend-fill': c.primarySoft,
        '--frontend-stroke': c.primary,
        '--backend-fill': c.surface,
        '--backend-stroke': c.accent,
        '--database-fill': c.surfaceAlt ?? c.surface,
        '--database-stroke': c.primary,
        '--cloud-fill': c.surface,
        '--cloud-stroke': c.textMuted,
        '--security-fill': c.surface,
        '--security-stroke': c.danger,
        '--messagebus-fill': c.surface,
        '--messagebus-stroke': c.warn,
        '--external-fill': c.surface,
        '--external-stroke': c.textMuted,
        '--toolbar-bg': c.bg,
        '--toolbar-border': c.border,
        '--toolbar-text': c.text,
        '--toolbar-hover': c.surface,
        '--toolbar-menu-bg': c.bg,
    };
}

/**
 * 从 viewBox 推导合适的初始缩放.
 *
 * archify 的 viewBox 越宽, 图被压得越厉害, 里面的小字越看不清.
 * 目标: 让渲染后的像素密度接近原始尺度.
 */
export function autoZoom(viewBox?: string, minFont = 9, fallback = 1.5): number {
    if (!viewBox) return fallback;
    const m = /[\d.]+\s+[\d.]+\s+([\d.]+)\s+([\d.]+)/.exec(viewBox);
    const w = m ? parseFloat(m[1]) : 0;
    if (!w) return fallback;
    const byWidth = w / 900;
    const byFont = minFont < 10 ? 1.25 : minFont < 12 ? 1.1 : 1;
    return Math.max(1, Math.min(3, Math.round(byWidth * byFont * 100) / 100));
}

/** 从 SVG 里解析最小 font-size (用于可读性补偿) */
export function minFontSize(svg: string): number {
    const sizes = [...svg.matchAll(/font-size="([\d.]+)"/g)].map((m) => parseFloat(m[1]));
    return sizes.length ? Math.min(...sizes) : 12;
}

export interface DiagramProps {
    asset: DiagramAsset | string;
    /**
     * 图表所属卡片在 URL 里的标识 (?ppt=).
     *
     * 分享链接需要它才能"直接打开这个架构图" —— 光有 ?page= 只会落在正文的
     * 缩略图上, 对方还要自己点一下。由 PptEmbed 透传进来.
     */
    cardId?: string;
    kind?: DiagramKind;
    caption?: React.ReactNode;
    tone?: 'plain' | 'card' | 'glass';
    pad?: 'none' | 'sm' | 'md';
    /** 初始缩放; 不传则按 viewBox 自动推导 (见 autoZoom) */
    defaultZoom?: number;
    /** 是否提供缩放/拖拽/全屏查看器 */
    zoomable?: boolean;
    /** 覆盖宽高比, 如 '16/9'; 默认取 viewBox */
    ratio?: string;
    /** 容器高度上限 (px) */
    maxHeight?: number;
    i?: number;
}

export function Diagram({
    asset, kind, caption, tone = 'card', pad = 'md', i,
    defaultZoom, zoomable = true, ratio, maxHeight, cardId,
}: DiagramProps): React.ReactElement {
    const t = useDeckTheme();
    const vars = useMemo(() => diagramVars(t) as React.CSSProperties, [t]);
    const a = resolveAsset(asset);
    /*
      预览态 (卡片正文里) 一律降级为静态图:
      此时 deck 的 stage scale() 会成为 position:fixed 的包含块, 放大浮层会算错尺寸、
      被裁在卡片里且退不出去; 而卡片本身又是"点一下打开"的整体热区.
      要缩放/导出/全屏, 点开卡片即可 (那时 interactive=true)。
    */
    const deckInteractive = useDeckInteractive();
    const canZoom = zoomable && deckInteractive;

    /*
      初始缩放**固定 100%**: 整张图完整可见, 每个节点都能点到.

      之前按"像素密度"自动放大到 187%, 结果是 2761px 的图塞进 1064px 的框里 ——
      实测 10 个节点里 9 个落在可视区之外, 点谁都点不到 (路径探测"点了没用"的直接原因).
      要看细节用 +/- 或滚轮, 不该由默认值替读者做这个决定.
    */
    const derived = 1;
    const [zoom, setZoom] = useState(defaultZoom ?? derived);
    const touched = useRef(defaultZoom !== undefined);
    useEffect(() => {
        if (!touched.current && derived) setZoom(derived);
    }, [derived]);

    /*
      节点探查 (语义护照).

      图的"语义"来自 SVG 上的稳定钩子 (data-node-id / data-edge-from ...) ——
      与几何无关, 所以缩放/拖拽/放大之后仍然准确. 解析一次缓存, 交互时只改属性,
      具体的变暗/提亮交给 archify 自带的那段语义 CSS, 视觉语言与原始产物一致.
    */
    const facts = useMemo(() => parseFacts(a.svg), [a.svg]);
    /*
      初始聚焦直接从 URL 的 #focus=<id> 取 (惰性初始化).

      为什么不在 effect 里读: "聚焦 -> 写 URL" 那个 effect 声明在前面,
      首次渲染时 focusId 还是 null, 它会**先把 hash 清掉**,
      后面那个"读 hash"的 effect 就再也看不到深链了 (实测分享链接打不开).
      放在初始化器里读, 时序上不可能被覆盖.
    */
    const [focusId, setFocusId] = useState<string | null>(() => {
        if (typeof window === 'undefined') return null;
        const m = /(?:^|[#&])focus=([^&]+)/.exec(window.location.hash);
        if (!m) return null;
        const id = decodeURIComponent(m[1]);
        return facts.nodes.has(id) ? id : null;
    });
    /*
      聚焦是**派生渲染**, 不是命令式改 DOM.

      坑: SVG 走 dangerouslySetInnerHTML, React 每次重渲染都会重建这棵子树 ——
      命令式 setAttribute 打上的标记会被下一次渲染抹掉 (点完节点紧接着摆卡片就是两次渲染).
      所以把聚焦态算进 SVG 字符串, 任何重渲染都只会把它画回正确状态.
    */
    /*
      读者的"语义视图" —— 聚焦 / 语义透镜 / 图例预览 / 路径探测.
      它们互斥 (同一时刻只有一种主导), 由 applyView 统一算进 SVG 字符串.
    */
    const [lensKind, setLensKind] = useState<string | null>(null);
    const [previewKind, setPreviewKind] = useState<string | null>(null);
    const [route, setRoute] = useState<string[] | null>(null);
    const [routePick, setRoutePick] = useState<'source' | 'target' | null>(null);
    // 放大态下的阅读深度 (map / read / full) —— 随缩放自动升降
    const [detailLevel, setDetailLevel] = useState<'map' | 'read' | 'full'>('read');
    // 查找器 / 演示模式
    const [finderOpen, setFinderOpen] = useState(false);
    const [presenting, setPresenting] = useState(false);
    const [query, setQuery] = useState('');

    /*
      SVG **只注入一次** (用稳定的 a.svg), 视图属性走命令式 applyViewToDom.

      这是用户报的两个问题的共同根因: 之前把视图算进字符串, 于是任何交互都重建
      SVG 子树 —— 动画被打回起点 (看着"倒着跑"), mousedown 后节点被销毁导致
      click 根本不触发 (路径探测点了没用).
    */
    const containerRef = useRef<HTMLDivElement>(null);
    /** 事件委托的监听器读它, 避免每次聚焦都重挂监听 */
    const focusRef = useRef<string | null>(null);
    focusRef.current = focusId;
    const [passportPos, setPassportPos] = useState<PassportPos>({ left: 12, top: 12 });
    const [linkCopied, setLinkCopied] = useState(false);
    const copyTimer = useRef(0);

    const aspect = useMemo(() => {
        if (ratio) {
            const [x, y] = ratio.split('/').map(Number);
            if (x > 0 && y > 0) return x / y;
        }
        return aspectOf(a.viewBox);
    }, [ratio, a.viewBox]);

    /*
      放大 (页内全屏, 非浏览器全屏).

      边界 = **当前演示页**. 浮层挂在 deck-layer 给出的宿主节点里 (演示页根节点,
      或卡片弹层的舞台容器), 用 inset:0 铺满它 —— 而不是铺满整个网页.
    */
    /*
      上下文读取放在最前: 下面多个 useCallback/useEffect 都依赖它们
      (分享链接要页码与卡片标识, 自动放大要 canZoom), 放在后面会被 TDZ 挡住.
    */
    const slideActive = useIsSlideActive();
    /** 当前页码 + 总屏数 (由 Deck 下发), 分享链接据此带上 ?page= */
    const { index: pageIndex, total: pageTotal } = useDeckPage();
    /** 所属卡片的 URL 标识: 有了它, 分享链接才能"直接打开这个架构图" */
    const cardIdFromCtx = usePptCardId();
    const pptCardId = cardId ?? cardIdFromCtx;

    const [full, setFull] = useState(false);
    /*
      ?zoom=1 —— "复制链接"带上的视角参数.

      带参数的链接打开后应**直接是放大态**, 否则对方还得自己点一次"放大",
      那就没达到"直接打开这个架构图"的目的. 只在可交互的图上生效 (见下方 effect).
    */
    const wantsZoom = useMemo(() => {
        if (typeof window === 'undefined') return false;
        const p = new URLSearchParams(window.location.search);
        return p.get('zoom') === '1' && p.get('ppt') !== null;
    }, []);
    const { host: layerHost, setLayerFull } = useDeckLayer();
    /**
     * 放大动画起点.
     *
     * 需求: 从**原图的位置**向四周放大到全屏, 收起时反向缩回.
     * 做法: 展开前记录原图 rect, 首帧把浮层 transform 成该 rect 的位置与尺寸 (FLIP),
     *       下一帧清掉 transform 让 CSS transition 把它补回全屏.
     * 时长 320ms (要求 0.5s 以内).
     *
     * 注意坐标系: 浮层未必挂在视口根上 (舞台容器可能自带 scale), 所以这里存的是
     * **浮层父节点坐标系**下的矩形, 不是 getBoundingClientRect 的视口坐标.
     * 之前直接拿视口坐标 + top:0;left:0 定位, 一旦宿主有 scale 就会飞出去.
     */
    const [flipFrom, setFlipFrom] = useState<{ x: number; y: number; w: number; h: number } | null>(null);
    const [exporting, setExporting] = useState(false);
    const [menuOpen, setMenuOpen] = useState(false);
    const wrapRef = useRef<HTMLElement>(null);
    const frameRef = useRef<HTMLDivElement>(null);
    /** 舞台 = 图框 + 悬浮工具条. 滚轮监听挂这里, 工具条上也一样生效 */
    const stageRef = useRef<HTMLDivElement>(null);
    const [frameH, setFrameH] = useState<number | null>(null);
    const timers = useRef<number[]>([]);

    /*
      浮层挂点:
        · 在演示页里 (deck-layer 给得出宿主) -> 挂在**演示页根节点**上.
          若宿主是浏览器全屏元素的后代, 浮层自然也进 top layer, 与宿主同生共死.
        · 独立使用 (没有 Deck) -> 退回 document.body + position: fixed 的视口浮层.
    */
    const layerNode = layerHost ?? (typeof document !== 'undefined' ? document.body : null);
    const viewportLayer = !layerHost;

    /**
     * 容器高度 = 可用宽 / 宽高比, 用可用高度封顶.
     * 只依赖宽度与比例, **与缩放无关** —— 这是"缩放不再引发版面高度变化"的关键.
     */
    const measureRef = useRef<() => void>(() => {});

    useLayoutEffect(() => {
        const measure = () => {
            const wrap = wrapRef.current;
            const frame = frameRef.current;
            if (!wrap || !frame) return;
            /*
              放大态: 高度交给 CSS (flex:1 撑满浮层剩余空间), JS 不再插手 ——
              之前这里按 window.innerHeight 算, 而 window 是**视口**而非演示页,
              卡片预览里算出的高度比父布局还大 (实测 900 vs 卡片 782), 直接把
              浮层内容顶出父边界 —— 正是"内部布局比父布局还大"的来源.
            */
            if (full) { setFrameH(null); return; }
            const w = frame.clientWidth || wrap.clientWidth;
            if (!w) return;
            const avail = maxHeight ?? Math.round(window.innerHeight * 0.58);
            setFrameH(Math.round(Math.min(w / aspect, avail)));
        };
        measureRef.current = measure;
        measure();
        // 下一帧再量一次, 覆盖"刚切换全屏/刚挂载"时布局未稳定的情况
        const raf = requestAnimationFrame(measure);
        const ro = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(measure) : null;
        if (ro && wrapRef.current) ro.observe(wrapRef.current);
        window.addEventListener('resize', measure);
        return () => {
            cancelAnimationFrame(raf);
            ro?.disconnect();
            window.removeEventListener('resize', measure);
        };
    }, [aspect, maxHeight, full]);

    /*
      切换放大态后强制重算高度.

      为什么单独来一次: 浮层走 Portal 渲染 (节点被搬到演示页根节点),
      上面那个 effect 里注册的 ResizeObserver 观察的仍是原节点, 未必会触发,
      高度会停在切换前的数值.
      这里等两帧 (Portal 挂载 + 样式生效) 再量.
    */
    useEffect(() => {
        const a = requestAnimationFrame(() => {
            const b = requestAnimationFrame(() => measureRef.current());
            timers.current.push(b);
        });
        timers.current.push(a);
        return () => {
            timers.current.forEach(cancelAnimationFrame);
            timers.current = [];
        };
    }, [full]);

    // 从放大态回到预览态 (或宿主关掉交互) 时, 别把全屏浮层留在屏幕上
    useEffect(() => {
        if (!canZoom && full) setFull(false);
    }, [canZoom, full]);

    /**
     * 语义透镜 / 图例预览用的类型清单.
     *
     * 直接来自图里真实存在的 data-node-kind —— 不预置一份写死的词表,
     * 换了图 (或 archify 换了角色集) 这里自动跟着变.
     */
    const kinds = useMemo(() => {
        const counts = new Map<string, number>();
        for (const n of facts.nodes.values()) {
            if (!n.kind) continue;
            counts.set(n.kind, (counts.get(n.kind) || 0) + 1);
        }
        return [...counts.entries()].map(([kind, count]) => ({ kind, count }))
            .sort((a, b) => b.count - a.count || a.kind.localeCompare(b.kind));
    }, [facts]);

    /** 查找器结果: 按标签 / 子标签 / 稳定 ID 匹配 */
    const found = useMemo(() => {
        const q = query.trim().toLowerCase();
        if (!q) return [];
        /*
          排序: 标签命中 > 稳定 ID > 子标签/上下文命中.
          更贴近"我找的是这个名字"的直觉 —— 否则搜 cookie 时,
          子标签里带 cookie 的节点会挤到真正的 CookieFetcher 前面.
        */
        const rank = (n: { id: string; label: string; sublabel?: string; context?: string }) => {
            if (n.label.toLowerCase().includes(q)) return 0;
            if (n.id.toLowerCase().includes(q)) return 1;
            return 2;
        };
        return [...facts.nodes.values()]
            .filter((n) => n.id.toLowerCase().includes(q)
                || n.label.toLowerCase().includes(q)
                || (n.sublabel || '').toLowerCase().includes(q)
                || (n.context || '').toLowerCase().includes(q))
            .sort((a, b) => rank(a) - rank(b) || a.label.localeCompare(b.label))
            .slice(0, 12);
    }, [facts, query]);

    /** 路径探测: 从起点出发的有向可达节点 (只用作者写下的边, 不猜) */
    const reachable = useCallback((start: string): Set<string> => {
        const seen = new Set<string>();
        const queue = [start];
        while (queue.length) {
            const cur = queue.shift() as string;
            for (const e of facts.out.get(cur) || []) {
                if (seen.has(e.to) || e.to === start) continue;
                seen.add(e.to);
                queue.push(e.to);
            }
        }
        return seen;
    }, [facts]);

    /** 两点间的**最短有向路径** (BFS, 无解返回 null) */
    const findRoute = useCallback((from: string, to: string): string[] | null => {
        if (from === to) return null;
        const prev = new Map<string, string>();
        const seen = new Set([from]);
        const queue = [from];
        while (queue.length) {
            const cur = queue.shift() as string;
            if (cur === to) break;
            for (const e of facts.out.get(cur) || []) {
                if (seen.has(e.to)) continue;
                seen.add(e.to);
                prev.set(e.to, cur);
                queue.push(e.to);
            }
        }
        if (!prev.has(to)) return null;
        const path = [to];
        let cur = to;
        while (prev.has(cur)) { cur = prev.get(cur) as string; path.unshift(cur); }
        return path;
    }, [facts]);

    /** 清掉所有"主动语义视图" (换图 / 关闭面板时用) */
    const clearViews = useCallback(() => {
        setLensKind(null);
        setPreviewKind(null);
        setRoute(null);
        setRoutePick(null);
    }, []);
    const clampZoom = useCallback((z: number) => Math.max(0.4, Math.min(8, Math.round(z * 100) / 100)), []);

    /*
      聚焦 / 取消聚焦.

      与 archify 的 set() 一致: 点节点 = 聚焦 (高亮它的直接邻域), 再点一次 = 取消,
      点空白 = 取消. 状态既写进 SVG 属性, 也同步到 URL 的 #focus=<id> —— 分享链接能复原.
    */
    const selectNode = useCallback((id: string | null) => {
        setFocusId((prev) => (prev === id ? null : id));
    }, []);

    /*
      聚焦状态 -> URL 的 #focus=<id>.

      放在 effect 里而不是 setState 的 updater 里: updater 必须是纯函数,
      在 StrictMode 下会被调用两次, 副作用放进去行为不可预期 (实测取消聚焦时
      hash 清不掉). effect 只随最终状态跑一次, 语义正确.

      用 replaceState 而不是 pushState: 键盘走查节点不该把浏览器回退键塞满.
    */
    useEffect(() => {
        if (typeof window === 'undefined') return;
        const url = new URL(window.location.href);
        const want = focusId ? 'focus=' + encodeURIComponent(focusId) : '';
        if (url.hash.replace(/^#/, '') === want) return;
        url.hash = want;
        window.history.replaceState(window.history.state, '', url.toString());
    }, [focusId]);

    /**
     * 复制"这张图在当前站点的链接".
     *
     * 链接要能**完整还原此刻的视角**, 因此带上三样东西:
     *   · ?ppt=<卡片标识>  —— 直接打开这张卡片的放大弹层 (否则只看到正文里的缩略图)
     *   · ?page=<页码>      —— 翻到了第几屏 (否则对方落在第 1 页还得自己翻)
     *   · #focus=<节点>     —— 聚焦到哪个节点 (可选)
     *
     * 为什么在**客户端**补而不是用当前 location: 卡片打开时 URL 里只有 ?ppt=;
     * 页码由 deck 内部维护 (props.syncUrl=false, 不往地址栏写), 所以要显式拼进去.
     */
    const copyLink = useCallback(async () => {
        if (typeof window === 'undefined') return;
        const url = new URL(window.location.href);
        if (pptCardId) url.searchParams.set('ppt', pptCardId);
        if (pageTotal > 0) url.searchParams.set('page', String(pageIndex + 1));
        if (focusId) url.hash = 'focus=' + encodeURIComponent(focusId);
        // 若此刻正把这张图放大着看, 链接也带上下 —— 对方打开就是同一个视角
        if (full) url.searchParams.set('zoom', '1');
        else url.searchParams.delete('zoom');
        const value = url.toString();
        let ok = false;
        try {
            if (navigator.clipboard?.writeText) {
                await navigator.clipboard.writeText(value);
                ok = true;
            }
        } catch { ok = false; }
        if (!ok) {
            // 无剪贴板权限 (http / 内嵌 iframe) 时的兜底
            const field = document.createElement('textarea');
            field.value = value;
            field.setAttribute('readonly', '');
            field.style.position = 'fixed';
            field.style.opacity = '0';
            document.body.appendChild(field);
            field.select();
            try { ok = document.execCommand('copy'); } catch { ok = false; }
            field.remove();
        }
        if (ok) {
            setLinkCopied(true);
            window.clearTimeout(copyTimer.current);
            copyTimer.current = window.setTimeout(() => setLinkCopied(false), 1600);
        }
    }, [focusId, pptCardId, pageIndex, pageTotal, full]);

    /** 护照卡片摆在被聚焦节点的旁边 (与 archify 一样"贴近被看的对象") */
    const placePassport = useCallback((id: string) => {
        const svg = frameRef.current?.querySelector('svg');
        // CSS.escape 兜住含特殊字符的 id; 老浏览器退回原样拼接
        const safe = typeof CSS !== 'undefined' && CSS.escape ? CSS.escape(id) : id;
        const node = svg?.querySelector('[data-node-id="' + safe + '"]');
        const stage = stageRef.current;
        if (!node || !stage) return;
        const nr = node.getBoundingClientRect();
        const sr = stage.getBoundingClientRect();
        const width = 268;
        const left = Math.max(8, Math.min(sr.width - width - 8, nr.right - sr.left + 10));
        const top = Math.max(8, Math.min(sr.height - 120, nr.top - sr.top));
        setPassportPos({ left: Math.round(left), top: Math.round(top) });
    }, []);

    /** 打开放大: 记录当前 rect 作为动画起点, 并告诉 deck "整页已被占用" */
    const openFull = useCallback(() => {
        const el = wrapRef.current;
        if (el) {
            const r = el.getBoundingClientRect();
            setFlipFrom({ x: r.left, y: r.top, w: r.width, h: r.height });
        }
        setFull(true);
    }, []);

    /*
      按 ?zoom=1 自动进入放大态.

      必须等 canZoom 为真 (即这张卡片真的被打开、内容切到交互实例) 才执行 ——
      预览卡片里铺满整页是个 bug, 不是需求. 用 ref 保证只自动做一次,
      读者手动退出后不该被再次拽回放大态.
    */
    const autoZoomed = useRef(false);
    useEffect(() => {
        /*
          必须同时满足 slideActive: deck 会把**所有**屏都挂载 (只是隐藏),
          少了这个条件, ?zoom=1 会让每一屏里的图都自动展开 (实测 2 张同时铺满).
          只有当前可见那一屏的图才该放大, 而且它必须已经量好尺寸 ——
          隐藏屏量出来是 0, FLIP 动画也会算错.
        */
        if (autoZoomed.current || !wantsZoom || !canZoom || !slideActive) return;
        autoZoomed.current = true;
        openFull();
    }, [wantsZoom, canZoom, slideActive, openFull]);

    /** 关闭: 同样从当前位置缩回原图 */
    const closeFull = useCallback(() => {
        const el = wrapRef.current;
        if (el) {
            const r = el.getBoundingClientRect();
            setFlipFrom({ x: r.left, y: r.top, w: r.width, h: r.height });
        }
        setFull(false);
    }, []);

    // 把"整页被占用"的语义同步给 deck (它据此隐藏自己的装饰/抬起层级)
    useEffect(() => {
        setLayerFull(full);
        return () => setLayerFull(false);
    }, [full, setLayerFull]);

    /*
      宿主消失 (弹层被卸载 / 换了演示页) 时收起浮层.

      为什么必须做: 浮层挂在宿主节点里, 宿主一卸载浮层也跟着没了 React 树,
      但内部 state 还停在 full=true —— 下次挂载会"一进来就是全屏".
    */
    useEffect(() => {
        if (!full) return;
        if (!layerHost) return;
        const host = layerHost;
        const check = () => { if (!host.isConnected) setFull(false); };
        const t = window.setInterval(check, 300);
        return () => window.clearInterval(t);
    }, [full, layerHost]);

    /*
      FLIP 补间.

      用 **getBoundingClientRect 的视口坐标**算: 浮层父节点自带的 scale 同时作用于
      起点目标与实测点, transform 也处在同一个坐标系里, 于是比值与平移量天然自洽,
      不需要手工做坐标系换算.
    */
    useLayoutEffect(() => {
        if (!flipFrom) return;
        const el = wrapRef.current;
        if (!el) return;
        const target = el.getBoundingClientRect();
        if (!target.width || !target.height) return;
        const sx = flipFrom.w / target.width;
        const sy = flipFrom.h / target.height;
        /*
          平移量要换算回**浮层父节点自己的坐标系**: 父节点若带 scale (演示页舞台),
          局部 translate(T) 在屏幕上会变成 scale*T. 尺寸比值不受影响 (两边同比例),
          但平移必须除以这个缩放, 否则动画从错误的位置起飞.
        */
        const parent = el.parentElement;
        const ps = parent && parent !== document.body && parent.offsetWidth
            ? (parent.getBoundingClientRect().width / parent.offsetWidth) || 1
            : 1;
        const dx = (flipFrom.x - target.left) / ps;
        const dy = (flipFrom.y - target.top) / ps;
        el.style.transition = 'none';
        el.style.transformOrigin = 'top left';
        el.style.transform = `translate(${dx}px, ${dy}px) scale(${sx}, ${sy})`;
        el.style.opacity = '0.4';
        const raf = requestAnimationFrame(() => {
            el.style.transition = 'transform 320ms cubic-bezier(0.22, 0.61, 0.36, 1), opacity 200ms ease';
            el.style.transform = 'none';
            el.style.opacity = '1';
        });
        const done = window.setTimeout(() => {
            el.style.transition = '';
            el.style.transform = '';
            el.style.transformOrigin = '';
            el.style.opacity = '';
            setFlipFrom(null);
        }, 360);
        return () => { cancelAnimationFrame(raf); window.clearTimeout(done); };
    }, [flipFrom]);

    /*
      事件委托: SVG 是 dangerouslySetInnerHTML 注进来的, 不能给每个节点挂 React 事件.
      所以在舞台容器上委托 —— 与 archify 在 svg 上挂一个监听的做法一致.
        · 点节点      -> 聚焦 / 再点取消 (archify: svg click -> set(id))
        · 点空白      -> 取消聚焦
        · Enter/Space -> 键盘聚焦 (节点本来就带 tabindex="0" role="button")
    */
    useEffect(() => {
        const stage = stageRef.current;
        /*
          只在"可交互"的图 (放大弹层 / 独立播放页) 上启用节点探查.
          预览卡片里的图是静态缩略图, 且整张卡片本身就是"点一下打开"的热区 ——
          在那里抢点击会让读者点不卡片. 与缩放/放大的降级规则一致.
        */
        if (!stage || !facts.nodes.size || !canZoom) return;

        const onStageClick = (e: MouseEvent) => {
            const target = e.target as Element | null;
            // 面板 / 工具条 / 分享菜单自己的点击不算"点空白"
            if (target?.closest?.('.hxd-diagram__tools, .hxd-passport, .hxd-panel, .hxd-diagram__export')) return;
            const node = target?.closest?.('[data-node-id]');

            /*
              路径探测优先: 选起点 -> 选终点 -> 显示最短有向路径.
              这条通道下点节点**不再**触发聚焦, 否则两种语义会互相打架
              (读者会看到路径高亮与邻域高亮同时挂在 svg 上).
            */
            if (routePick) {
                if (!node) return;
                const id = node.getAttribute('data-node-id') || '';
                if (routePick === 'source') {
                    setRoute([id]);
                    setRoutePick('target');
                } else {
                    const path = findRoute(route?.[0] || '', id);
                    if (path) { setRoute(path); setRoutePick(null); }
                    // 无解: 保持 target 选择态, 让读者另选一个
                }
                return;
            }

            if (node) {
                const id = node.getAttribute('data-node-id') || '';
                // 点节点 = 聚焦; 同时退出透镜/预览, 避免两种语义视图叠加
                setLensKind(null);
                setPreviewKind(null);
                selectNode(id);
                // 让卡片贴到刚点的那个节点旁边 (下一帧, 等属性生效)
                requestAnimationFrame(() => placePassport(id));
                return;
            }
            // 点画布空白处 = 取消聚焦 (工具条与卡片自己的点击不算)
            if (focusRef.current) selectNode(null);
        };
        const onStageKey = (e: KeyboardEvent) => {
            const target = e.target as Element | null;
            const node = target?.closest?.('[data-node-id]');
            if (!node) return;
            if (e.key !== 'Enter' && e.key !== ' ') return;
            e.preventDefault();
            // 不拦下来, Deck 的 window 级键盘会把空格当成"下一页"
            e.stopPropagation();
            const id = node.getAttribute('data-node-id') || '';
            selectNode(id);
            requestAnimationFrame(() => placePassport(id));
        };

        stage.addEventListener('click', onStageClick);
        stage.addEventListener('keydown', onStageKey);
        return () => {
            stage.removeEventListener('click', onStageClick);
            stage.removeEventListener('keydown', onStageKey);
        };
    /*
      full 必须在 deps 里: 放大/收起会切换"内联 <-> Portal", 那是一次**重新挂载** ——
      stageRef 指向的是新节点, 监听却还绑在已卸载的旧节点上, 于是放大后点节点毫无反应.
      (滚轮那条 effect 早就带了 full, 所以滚轮一直正常, 只有点击/键盘受影响.)
    */
    }, [facts, selectNode, placePassport, canZoom, routePick, route, findRoute, full]);

    /** 点菜单外面关掉分享菜单 (与 archify 的 document 级关闭一致) */
    useEffect(() => {
        if (!menuOpen) return;
        const onDoc = (e: MouseEvent) => {
            const target = e.target as Element | null;
            if (target?.closest?.('.hxd-diagram__export')) return;
            setMenuOpen(false);
        };
        document.addEventListener('mousedown', onDoc);
        return () => document.removeEventListener('mousedown', onDoc);
    }, [menuOpen]);

    /*
      初始聚焦: 从 URL 的 #focus=<id> 复原 (分享链接直达某个节点).
      只在 mounted 后跑一次, 且必须是图里真实存在的 id.
      另外: 换主题会重挂 SVG 内容 -> 属性被冲掉, 这里按 focusId 重新贴一遍.
    */
    /*
      把 SVG 写入容器.

      判据是 **(容器元素, 内容) 对**:
        · 内容变 (换图/换主题) -> 重写;
        · **容器变** (放大/收起时浮层会 Portal 出一个新的 div) -> 也要重写.

      踩过的坑: 一开始只比 a.svg, 于是放大后新挂载的容器永远是空的 ——
      表现就是"放大之后一片空白, 只有放大时才触发".
    */
/** 换图/换容器时 +1, 逼视图 effect 重新贴属性 */
    const [viewEpoch, setViewEpoch] = useState(0);
    const injectedRef = useRef<{ el: HTMLElement | null; svg: string }>({ el: null, svg: '' });
    useEffect(() => {
        const el = containerRef.current;
        if (!el || !a.svg) return;
        if (injectedRef.current.el === el && injectedRef.current.svg === a.svg) return;
        injectedRef.current = { el, svg: a.svg };
        el.innerHTML = a.svg;
        // 内容或容器变了: 视图属性要重新贴一遍
        setViewEpoch((n) => n + 1);
    }, [a.svg, full, viewEpoch]);
    /*
      把当前视图 (聚焦/透镜/预览/路径) 应用到 DOM.

      命令式设置属性, 不碰节点身份 —— 动画与事件监听都保留.
    */
    useEffect(() => {
        applyViewToDom(containerRef.current, facts, {
            focusId,
            lensKind,
            previewKind,
            route,
            routePicking: routePick,
        });
        if (focusId && slideActive) placePassport(focusId);
    }, [facts, focusId, lensKind, previewKind, route, routePick, placePassport, canZoom, full, slideActive, viewEpoch]);

    
    /** 节点集合变化 (换图) 时丢掉无效的聚焦 */
    useEffect(() => {
        if (focusId && !facts.nodes.has(focusId)) setFocusId(null);
    }, [facts, focusId]);

    /*
      轨迹动效 (trace) —— 对应 archify 的 ambient motion.

      设计稿的 SVG 里每个节点/边都带 data-animate + --step (作者编排的顺序).
      动效规则全部是 html[data-ambient-motion="running"] … 的形式, 也就是
      "由**文档根**上的一个属性统一放行" —— 这是 archify 的 Motion Governor:
        · 跑完一轮就把 data-ambient-motion 置为 settled, 动效自然停在终态;
        · 静态产物 (没有 data-animation="trace") 完全不参与.
      我们照搬这套协议, 不自己造动画.

      触发时机: 打开放大 / 进入独立播放页 (canZoom) 时跑一轮;
      读者点节点探查时**不**重放 —— 语义意图优先于氛围动效.
    */
    const hasTrace = useMemo(() => /data-animation="trace"/.test(a.svg), [a.svg]);
    const [motionRun, setMotionRun] = useState(0);
    const replayMotion = useCallback(() => setMotionRun((n) => n + 1), []);

    useEffect(() => {
        /*
          只在"可交互 + 当前这一屏"放行动效:
            · 预览卡片里的图是静态缩略图, 不该动 (与缩放/放大的降级规则一致);
            · deck 会把所有屏都挂上, 非当前屏动效既看不见又要烧 CPU,
              而且多张图同时往文档根写同一个属性会互相打架.
        */
        if (!hasTrace || !canZoom || !slideActive || typeof document === 'undefined') return;
        const root = document.documentElement;
        const reduce = typeof window.matchMedia === 'function'
            && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
        if (reduce) {
            // 尊重系统偏好: 不放行, 图停在已授权的静态形态
            root.removeAttribute('data-ambient-motion');
            return;
        }
        root.setAttribute('data-ambient-motion', 'running');
        /*
          跑完一轮后置 settled.
          时间取最长一条边的动画 (2.4s) + 最后一步的延迟 (12 * 160ms) + 余量,
          与 archify 的六秒捕获窗口同量级; 到点再兜底置位, 免得某些元素不触发
          animationend 时永久停在 running.
        */
        const total = 2400 + 12 * 160 + 400;
        const timer = window.setTimeout(() => {
            if (root.getAttribute('data-ambient-motion') === 'running') {
                root.setAttribute('data-ambient-motion', 'settled');
            }
        }, total);
        return () => {
            window.clearTimeout(timer);
            root.removeAttribute('data-ambient-motion');
        };
        // motionRun 变化 = 手动重播一轮
    }, [hasTrace, motionRun, canZoom, slideActive]);

    /** 导出当前图 */
    const doExport = useCallback(async (fmt: ExportFormat) => {
        const svg = frameRef.current?.querySelector('svg');
        if (!svg) return;
        setExporting(true);
        try {
            await exportSvg(svg as SVGSVGElement, a.css, pickCssVars(vars as Record<string, string>), {
                format: fmt,
                name: (a.title || 'diagram').replace(/[\\/:*?"<>|]/g, '_'),
                // 位图需要底色; svg 保持透明
                background: fmt === 'png' ? undefined : (t.colors.bg || '#ffffff'),
                scale: 2,
            });
        } finally {
            setExporting(false);
            setMenuOpen(false);
        }
    }, [a.css, a.title, vars, t.colors.bg]);

    // 缩放后把视口居中, 免得放大时只看到左上角
    useEffect(() => {
        const f = frameRef.current;
        if (!f) return;
        f.scrollLeft = (f.scrollWidth - f.clientWidth) / 2;
        f.scrollTop = (f.scrollHeight - f.clientHeight) / 2;
    }, [zoom, frameH, full]);

    // 拖拽平移
    const [panning, setPanning] = useState(false);
    const drag = useRef<{ x: number; y: number; sl: number; st: number; moved: boolean } | null>(null);

    /*
      按住拖拽 = 平移.

      踩过的坑 (用户报的"路径探测点了没用"的真根因):
        之前在 mousedown 里就 setPanning(true) —— 那是一次状态变更, 会让浏览器
        认为按下与抬起落在**不同的 DOM 状态**上, 于是**根本不派发 click 事件**.
        实测: 节点上拿到 mousedown + mouseup, 但 click 一次都没有 → 节点永远选不中.
      正解: mousedown 只记起点, **等指针真的移动了**才开始拖拽/置 panning.
      这样单纯的点击不产生任何状态变更, click 正常派发, 与节点探查并存.
    */
    const onDown = useCallback((e: React.MouseEvent) => {
        const el = frameRef.current;
        if (!el || !canZoom) return;
        drag.current = { x: e.clientX, y: e.clientY, sl: el.scrollLeft, st: el.scrollTop, moved: false };
    }, [canZoom]);

    useEffect(() => {
        const onMove = (e: MouseEvent) => {
            const el = frameRef.current;
            const d = drag.current;
            if (!el || !d) return;
            // 第一次真正移动才算拖拽 (阈值 3px, 避免手抖把点击吃掉)
            if (!d.moved) {
                if (Math.abs(e.clientX - d.x) < 3 && Math.abs(e.clientY - d.y) < 3) return;
                d.moved = true;
                setPanning(true);
            }
            // 直接写 scrollLeft/Top: 浏览器会自动夹在 [0, max] 内, 天然就是真实边界
            el.scrollLeft = d.sl - (e.clientX - d.x);
            el.scrollTop = d.st - (e.clientY - d.y);
        };
        const onUp = () => { setPanning(false); drag.current = null; };
        window.addEventListener('mousemove', onMove);
        window.addEventListener('mouseup', onUp);
        return () => {
            window.removeEventListener('mousemove', onMove);
            window.removeEventListener('mouseup', onUp);
        };
    }, [panning]);

    /*
      放大态 = 一个真正的模态: 键盘先由它接管.

      两个必须拦住的兄弟处理:
        · 宿主 (PptCard) 的 Esc —— 退浏览器全屏 / 关弹层;
        · Deck 的翻页键盘 —— 否则浮层开着, 底下的演示页还在被翻走.
      两者都挂在 document/window 的**冒泡**阶段, 所以这里用捕获阶段监听 +
      stopPropagation, 抢在它们之前. 这样"放大时按 Esc"只退放大, 不会一次退两层.
    */
    useEffect(() => {
        if (!full) return;
        const onKey = (e: KeyboardEvent) => {
            if (e.key === 'Escape') {
                e.preventDefault();
                e.stopPropagation();
                /*
                  分层退出 (与 archify 的"先关卡片, 再退层级"一致):
                  护照卡片开着 -> 先收卡片; 卡片收了 -> 才退放大.
                  否则一次 Esc 直接从放大态掉回演示页, 读者还要重点一次节点才知道看到了什么.
                */
                if (focusRef.current) selectNode(null);
                else closeFull();
                return;
            }
            // 放大期间不把翻页/滚动按键传给 deck
            if (['ArrowDown', 'ArrowRight', 'ArrowUp', 'ArrowLeft', 'PageDown', 'PageUp', 'Home', 'End', ' '].includes(e.key)) {
                e.preventDefault();
                e.stopPropagation();
            }
        };
        window.addEventListener('keydown', onKey, true);
        return () => window.removeEventListener('keydown', onKey, true);
    }, [full, closeFull, selectNode]);

    /*
      聚焦护照开着时, Esc 也要能收掉它 —— 上面那条只在**放大态**生效,
      而卡片预览/独立播放页里没有放大态, 只靠它收不了卡片.
      同样用捕获阶段, 抢在 Deck / PptCard 之前.
    */
    useEffect(() => {
        if (!focusId || full) return;
        const onKey = (e: KeyboardEvent) => {
            if (e.key !== 'Escape') return;
            e.preventDefault();
            e.stopPropagation();
            selectNode(null);
        };
        window.addEventListener('keydown', onKey, true);
        return () => window.removeEventListener('keydown', onKey, true);
    }, [focusId, full, selectNode]);

    /*
      阅读深度: 随缩放自动升降 (archify 的 Reading Depth).

        MAP  (< 100%): 只留结构 + 主标签
        READ (100-175%): 补上关系标签与节点上下文
        FULL (>= 175%): 再补细粒度注释
      实现方式与 archify 一致: 只改容器上的一个属性, 显隐交给自带 CSS.
    */
    useEffect(() => {
        const next = zoom >= 1.75 ? 'full' : zoom >= 1 ? 'read' : 'map';
        setDetailLevel(next);
    }, [zoom]);

    /*
      键盘快捷键 —— 与 archify 的键位保持一致.
      必须用**捕获阶段**并拦下来: Deck 的翻页监听在 window 冒泡阶段,
      不拦的话按 / 或 r 会被当成翻页快捷键.
    */
    useEffect(() => {
        if (!canZoom || !slideActive) return;
        const onKey = (e: KeyboardEvent) => {
            const t = e.target as HTMLElement | null;
            if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
            const k = e.key;
            const stop = () => { e.preventDefault(); e.stopPropagation(); };
            if (k === '/') { stop(); setFinderOpen((v) => !v); return; }
            if (k === 'l' || k === 'L') {
                stop();
                setLensKind((prev) => (prev ? null : kinds[0]?.kind ?? null));
                return;
            }
            if (k === 'r' || k === 'R') {
                stop();
                setLensKind(null); setPreviewKind(null); setFocusId(null);
                setRoutePick((p) => (p ? null : 'source'));
                setRoute(null);
                return;
            }
            if (k === 'p' || k === 'P') { stop(); setPresenting((v) => !v); return; }
            if (k === 'f' || k === 'F') { stop(); setFinderOpen((v) => !v); return; }
            if (k === '0') { stop(); setZoom(1); return; }
            if (k === '+' || k === '=') { stop(); setZoom((z) => clampZoom(z + 0.15)); return; }
            if (k === '-') { stop(); setZoom((z) => clampZoom(z - 0.15)); return; }
            if (k === 'e' || k === 'E') { stop(); setMenuOpen((v) => !v); return; }
        };
        window.addEventListener('keydown', onKey, true);
        return () => window.removeEventListener('keydown', onKey, true);
    }, [canZoom, slideActive, kinds, clampZoom]);

    // 演示模式: 隐去所有控件, 只留图; Esc 退出
    useEffect(() => {
        if (!presenting) return;
        const onKey = (e: KeyboardEvent) => {
            if (e.key !== 'Escape' && e.key !== 'p' && e.key !== 'P') return;
            e.preventDefault();
            e.stopPropagation();
            setPresenting(false);
        };
        window.addEventListener('keydown', onKey, true);
        return () => window.removeEventListener('keydown', onKey, true);
    }, [presenting]);
    /**
     * 滚轮 = 缩放.
     *
     * 必须用**原生**监听:
     *   React 的 onWheel 是合成事件 (委托在 root 上), 而 Deck 用的是原生 addEventListener,
     *   原生监听在冒泡链上更早触发 —— 合成事件的 stopPropagation 根本拦不住它.
     *
     * 另外: 图已经可滚动且还能滚 / 或者缩放已到极限时, 也要吃掉这次滚轮.
     * 否则"缩到最小再继续滚"会把信号放给 Deck, 整屏被翻走 —— 这正是报告的 bug.
     */
    useEffect(() => {
        const f = stageRef.current;
        if (!f || !canZoom) return;

        const onNativeWheel = (e: WheelEvent) => {
            /*
              只在**图已处于放大查看状态**时才用滚轮缩放.

              踩过的坑: 之前无条件 preventDefault + stopPropagation, 结果是
              只要页面里有架构图, 鼠标停在图上滚就永远被吃掉 —— 页面滚不动.
              这不是"图的边界"问题, 而是图不该在预览态抢滚轮:
              读者此时想滚的是**文章**, 不是图.
            */
            if (!full) return;

            markWheelConsumed(e);
            e.preventDefault();
            e.stopPropagation();

            const dir = e.deltaY > 0 ? -1 : 1;
            setZoom((z) => clampZoom(z + dir * 0.15));
        };

        // passive: false 才能 preventDefault
        f.addEventListener('wheel', onNativeWheel, { passive: false });
        return () => f.removeEventListener('wheel', onNativeWheel);
    }, [canZoom, clampZoom, full]);

    const inner = (
        <figure
            ref={wrapRef as React.RefObject<HTMLElement>}
            className="hxd-diagram"
            data-tone={tone}
            data-pad={pad}
            data-kind={kind}
            data-full={full ? 'true' : 'false'}
            data-layer={viewportLayer ? 'viewport' : 'deck'}
            data-presenting={presenting ? 'true' : 'false'}
            style={vars}
            onMouseDown={full ? (e) => e.stopPropagation() : undefined}
        >
            {a.css ? <style>{a.css}</style> : null}


            <div ref={stageRef} className="hxd-diagram__stage">
                {/*
                  放大态必须拦住发生在浮层上的 mousedown, 不能让它冒泡到宿主:
                  卡片弹层的 modalOverlay 用 onMouseDown 关闭自己 —— 放大后在图上按一下鼠标,
                  整个弹层 (连同浮层) 就被关掉了. 这也是"退出路径不明显"的一个来源.
                */}
                <div
                    ref={frameRef}
                    className="hxd-diagram__frame"
                    data-panning={panning ? 'true' : 'false'}
                    style={frameH ? { height: frameH } : undefined}
                    onMouseDown={onDown}
                    onClick={full ? (e) => e.stopPropagation() : undefined}
                >
                    {/*
                      SVG 由 effect **一次性写入**, React 不参与管理这棵子树.

                      为什么不用 dangerouslySetInnerHTML: 实测只要父组件重渲染
                      (缩放 / 点节点 / 开面板都会), React 就会重设 innerHTML ——
                      SVG 被整棵重建, 于是 (a) 动画被打回起点, 看着"倒着跑";
                      (b) mousedown 后节点被销毁, click 不派发, 节点选不中.
                      手动写入后 React 只管这个空 div, 内容与动画都不再被打断.
                    */}
                    <div
                        ref={containerRef}
                        className="hxd-diagram__svg diagram-container"
                        data-detail-level={detailLevel}
                        role="img"
                        aria-label={a.title || 'diagram'}
                        style={{ width: `${zoom * 100}%`, height: `${zoom * 100}%` }}
                    />
                </div>

                {canZoom ? (
                    <div className="hxd-diagram__tools" data-full={full ? 'true' : 'false'}>
                        <button type="button" className="hxd-diagram__btn" onClick={() => setZoom((z) => clampZoom(z - 0.15))} aria-label="缩小">−</button>
                        <span className="hxd-diagram__zoom">{Math.round(zoom * 100)}%</span>
                        <button type="button" className="hxd-diagram__btn" onClick={() => setZoom((z) => clampZoom(z + 0.15))} aria-label="放大">+</button>
                        <button type="button" className="hxd-diagram__btn" onClick={() => setZoom(1)} aria-label="复位">⟲</button>
                        {/* 重播轨迹动效: 只在图本身带 trace 时出现 */}
                        {hasTrace ? (
                            <button
                                type="button"
                                className="hxd-diagram__btn"
                                onClick={replayMotion}
                                aria-label="重播动效"
                                title="重播动效"
                            >
                                ▶
                            </button>
                        ) : null}
                        {/* 语义透镜: 按节点角色看这张图 */}
                        <button
                            type="button"
                            className="hxd-diagram__btn"
                            data-on={lensKind ? 'true' : 'false'}
                            onClick={() => setLensKind((v) => (v ? null : kinds[0]?.kind ?? null))}
                            aria-label="语义透镜"
                            aria-expanded={!!lensKind}
                            title="语义透镜 (L)"
                        >
                            透
                        </button>
                        {/* 路径探测: 两点间的有向路径 */}
                        <button
                            type="button"
                            className="hxd-diagram__btn"
                            data-on={routePick || route ? 'true' : 'false'}
                            onClick={() => {
                                setLensKind(null); setPreviewKind(null); setFocusId(null); setRoute(null);
                                setRoutePick((p) => (p ? null : 'source'));
                            }}
                            aria-label="路径探测"
                            aria-expanded={!!(routePick || route)}
                            title="路径探测 (R)"
                        >
                            路
                        </button>
                        {/* 查找节点 */}
                        <button
                            type="button"
                            className="hxd-diagram__btn"
                            data-on={finderOpen ? 'true' : 'false'}
                            onClick={() => setFinderOpen((v) => !v)}
                            aria-label="查找节点"
                            aria-expanded={finderOpen}
                            title="查找节点 (/)"
                        >
                            ⌕
                        </button>
                        {/* 演示模式 */}
                        <button
                            type="button"
                            className="hxd-diagram__btn"
                            onClick={() => setPresenting(true)}
                            aria-label="演示模式"
                            title="演示模式 (P)"
                        >
                            ▶
                        </button>
                        {/* 全屏按钮必须明确表达退出, 而不是一个可再次点的图标.
                            之前放大后按钮仍在, 再点会二次进入浮层遮住整个网页, 退出路径不明显. */}
                        <button
                            type="button"
                            className="hxd-diagram__btn"
                            data-variant={full ? 'exit' : 'enter'}
                            onClick={full ? closeFull : openFull}
                            aria-label={full ? '退出放大' : '放大查看'}
                            title={full ? '退出放大 (Esc)' : '放大查看'}
                        >
                            {full ? '✕ 退出' : '⛶'}
                        </button>
                        {/*
                          分享菜单 —— "带走这张图".

                          为什么不是一个裸的下载箭头: 之前那个 ⤓ 既不像导出, 也不像分享,
                          而且读者最想要的是"这张图在我们网站上的链接", 不是一份离线文件.
                          所以主入口是**分享**图标, 菜单里第一项就是复制链接
                          (带 #focus=<id> 时打开即回到同一视角), 下载格式排在下面.
                        */}
                        <span className="hxd-diagram__export">
                            <button
                                type="button"
                                className="hxd-diagram__btn"
                                data-variant={linkCopied ? 'done' : undefined}
                                onClick={() => setMenuOpen((v) => !v)}
                                aria-label="分享"
                                aria-expanded={menuOpen}
                                title="分享 / 复制链接"
                            >
                                {linkCopied ? <FaCheck aria-hidden="true" /> : <FaShareAlt aria-hidden="true" />}
                            </button>
                            {menuOpen ? (
                                <span className="hxd-diagram__menu">
                                    <button type="button" onClick={copyLink}>
                                        {linkCopied ? '已复制链接' : '复制链接'}
                                    </button>
                                    {/* 下载: 只在放大时出现 (按需求) */}
                                    {full ? (
                                        <>
                                            <span className="hxd-diagram__menu-sep" />
                                            <span className="hxd-diagram__menu-head">下载图片</span>
                                            {(['png', 'jpeg', 'webp', 'svg'] as ExportFormat[]).map((fmt) => (
                                                <button key={fmt} type="button" disabled={exporting} onClick={() => doExport(fmt)}>
                                                    <FaFileDownload aria-hidden="true" /> {fmt.toUpperCase()}
                                                </button>
                                            ))}
                                        </>
                                    ) : null}
                                </span>
                            ) : null}
                        </span>
                    </div>
                ) : null}

                {/*
                  语义护照: 点节点后贴在它旁边的信息卡.
                  它放在 stage 里 (与工具条同级), 因为节点本身在滚动画布里, 卡片跟着滚会永远错位.
                */}
                {focusId ? (
                    <DiagramPassport
                        facts={facts}
                        id={focusId}
                        pos={passportPos}
                        onClose={() => selectNode(null)}
                        onGo={(next) => { selectNode(next); requestAnimationFrame(() => placePassport(next)); }}
                    />
                ) : null}

                {/* 语义透镜: 按节点角色看这张图 */}
                {lensKind && kinds.length ? (
                    <DiagramLens
                        kinds={kinds}
                        active={lensKind}
                        onPick={(k) => { setFocusId(null); setLensKind((v) => (v === k ? null : k)); }}
                        onHover={(k) => setPreviewKind(k)}
                        onLeave={() => setPreviewKind(null)}
                    />
                ) : null}

                {/* 路径探测 */}
                {routePick || route ? (
                    <DiagramRoute
                        facts={facts}
                        route={route}
                        picking={routePick}
                        onStart={() => { setRoute(null); setRoutePick('source'); }}
                        onClear={() => { setRoute(null); setRoutePick(null); }}
                        onGo={(id) => { selectNode(id); requestAnimationFrame(() => placePassport(id)); }}
                    />
                ) : null}

                {/* 节点查找器 */}
                {finderOpen ? (
                    <DiagramFinder
                        query={query}
                        onQuery={setQuery}
                        results={found}
                        onClose={() => { setFinderOpen(false); setQuery(''); }}
                        onGo={(id) => {
                            selectNode(id);
                            requestAnimationFrame(() => placePassport(id));
                            setFinderOpen(false);
                        }}
                    />
                ) : null}

            </div>
            {caption ? <figcaption className="hxd-diagram__caption">{caption}</figcaption> : null}
        </figure>
    );

    /*
      放大: Portal 到**演示页根节点** (deck-layer 给的 host), 不是 document.body.

      历史教训 —— 三种"错的全屏"都出自 Portal 到 body + position: fixed:
        1. 定位相对**视口**: 卡片预览里 inset:0 就是整块网页, 盖住正文与站点 UI;
        2. 脱离 deck 的 CSS 变量: 背景色解析不出值, 浮层透明, 后面的内容透出来;
        3. 宿主进入浏览器全屏时, 全屏元素在 top layer, body 上的浮层被压在下面 ——
           点"放大"反而什么都看不见.
      Portal 只搬自己这一棵子树, 完全不动 deck 的 transform, 因此没有连带动画.

      为什么不用 :has() 把祖先 transform 改成 none:
        .hxd-strip 带着 0.86s 的 transform 过渡, 强行设成 none 会触发一次"回到第 1 页"的动画 ——
        那正是"放大时有页面切换特效"和"某些页加载不出来"的原因.
    */
    if (full) {
        return createPortal(inner, (layerNode ?? document.body) as Element);
    }
    return inner;
}

/** 图 + 旁注并排 */
export function DiagramWithNotes({
    asset, notes, kind, ratio = '1.6fr 1fr', side = 'right', caption,
    defaultZoom, pad = 'sm', maxHeight, i,
}: {
    asset: DiagramAsset | string;
    notes: React.ReactNode;
    kind?: DiagramKind;
    ratio?: string;
    side?: 'left' | 'right';
    caption?: React.ReactNode;
    defaultZoom?: number;
    pad?: DiagramProps['pad'];
    maxHeight?: number;
    i?: number;
}): React.ReactElement {
    const fig = <Diagram asset={asset} kind={kind} caption={caption} pad={pad} defaultZoom={defaultZoom} maxHeight={maxHeight} />;
    const note = <div className="hxd-diagram__notes">{notes}</div>;
    const body = (
        <div className="hxd-diagram-split" style={{ gridTemplateColumns: ratio }} data-side={side}>
            {side === 'left' ? <>{fig}{note}</> : <>{note}{fig}</>}
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/** 多图并排 */
export function DiagramGrid({
    items, cols, i,
}: {
    items: { asset: DiagramAsset | string; caption?: React.ReactNode; kind?: DiagramKind }[];
    cols?: number;
    i?: number;
}): React.ReactElement {
    const n = cols ?? Math.min(items.length, 2);
    const body = (
        <div className="hxd-diagram-grid" style={{ gridTemplateColumns: "repeat(" + n + ", minmax(0, 1fr))" }}>
            {items.map((it, k) => (
                <Diagram key={k} asset={resolveAsset(it.asset)} kind={it.kind} caption={it.caption} pad="sm" />
            ))}
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}

/** 占位 */
export function DiagramPlaceholder({ label = '架构图', i }: { label?: string; i?: number }): React.ReactElement {
    const body = (
        <div className="hxd-diagram hxd-diagram--placeholder" data-tone="card" data-pad="md">
            <div className="hxd-diagram__frame">
                <span>{label}</span>
            </div>
        </div>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}
