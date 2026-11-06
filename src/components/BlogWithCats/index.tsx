import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';
import { createPortal } from 'react-dom';
import './BlogWithCats.css';

/**
 * 立绘对: static/img/catgirl/pairN-left.png / pairN-right.png, RGBA 透明底无损 PNG 原生分辨率, 5:12, 角色在下 55%.
 * left 图的竖直裁切线在右边 (身体藏在右侧墙后), right 图的裁切线在左边. 几何规格见 .agents/research/catgirl-spec.md.
 * 以用户素材 static/img/catgirl-source/ 的角色为参考生成探头构图 (绿幕抠出 + 去溢色 + 边缘 0.6px 羽化), 素材原图保留.
 * 不动 static/default-img/neko_*.png (原组件素材).
 */
/**
 * fade: 底部开始淡出的位置 (占图片高度的比例, 淡到 100% 处全透明). 逐张看图定的: 落在躯干下部 / 裙摆 / 腿,
 * 不落在手、手臂、手指上 (这些细长轮廓一淡就像断了). 2026-11-03 复查 26 张, 3l / 5l / 5r / 10r / 8r 手臂垂到最底或扒错边, 已重新生成.
 */
/** 每对必须是两个不同角色, 禁止同一张图镜像成一对 (2026-11-03 删掉 pair14: 白草帽浅紫发角色的左右镜像). */
/** 内置 13 对: manifest.json 读不到 / 为空 / 全部不合规格时的回落 (2026-11-03 起素材优先读 manifest). */
const NEKO_PAIRS: readonly NekoFullPair[] = [
    { left: { w: 721, h: 1730, fade: 0.87 }, right: { w: 737, h: 1768, fade: 0.84 } },
    { left: { w: 732, h: 1757, fade: 0.88 }, right: { w: 725, h: 1740, fade: 0.93 } },
    { left: { w: 1102, h: 2646, fade: 0.88 }, right: { w: 768, h: 1844, fade: 0.86 } },
    { left: { w: 765, h: 1837, fade: 0.88 }, right: { w: 705, h: 1691, fade: 0.86 } },
    { left: { w: 714, h: 1713, fade: 0.85 }, right: { w: 796, h: 1911, fade: 0.88 } },
    { left: { w: 703, h: 1688, fade: 0.88 }, right: { w: 723, h: 1735, fade: 0.89 } },
    { left: { w: 763, h: 1831, fade: 0.91 }, right: { w: 904, h: 2170, fade: 0.86 } },
    { left: { w: 710, h: 1704, fade: 0.9 }, right: { w: 745, h: 1788, fade: 0.91 } },
    { left: { w: 845, h: 2028, fade: 0.9 }, right: { w: 611, h: 1466, fade: 0.94 } },
    { left: { w: 758, h: 1819, fade: 0.88 }, right: { w: 696, h: 1670, fade: 0.88 } },
    { left: { w: 768, h: 1842, fade: 0.9 }, right: { w: 796, h: 1910, fade: 0.86 } },
    { left: { w: 775, h: 1860, fade: 0.88 }, right: { w: 736, h: 1766, fade: 0.9 } },
    { left: { w: 783, h: 1880, fade: 0.9 }, right: { w: 716, h: 1719, fade: 0.93 } },
].map((p, i) => ({
    left: { ...p.left, file: `pair${i + 1}-left.png`, slug: `pair${i + 1}-l` },
    right: { ...p.right, file: `pair${i + 1}-right.png`, slug: `pair${i + 1}-r` },
    // 内置 13 对每对是两个不同角色 (2026-11-03 逐对核过), 角色标识直接用各自的图 slug, 保证左右不等
    leftSlug: `pair${i + 1}-l`,
    rightSlug: `pair${i + 1}-r`,
}));

/**
 * 角色在画面里的横向落位 = 竖直裁切线 (身体藏进墙的那条边) 在哪一侧.
 * pair 变体要求裁切线朝正文: 左槽 anchor 'right', 右槽 anchor 'left'. 不填时按文件名约定 (left 图 'right', right 图 'left').
 * 落位与槽位相反时自动水平镜像 (类名 neko-flip); noFlip 的图 (有文字 / 明显不对称的服饰) 不镜像, 该对整组跳过.
 * 2026-11-03 逐张量过 28 张 (.agents/research/catgirl-anchor.md): 全部与约定一致, 当前没有一张需要镜像.
 */
interface NekoImg { file: string; w: number; h: number; fade: number; slug: string; anchor?: 'left' | 'right'; noFlip?: boolean }
/** 一侧为 null = 单侧素材 (manifest 里另一侧还没生成, needsGen); 内置回落恒为双侧 */
interface NekoPair { left: NekoImg | null; right: NekoImg | null; leftSlug?: string; rightSlug?: string }
/** leftSlug / rightSlug: 左右各自是哪个角色 (镜像检查只认这两个字段, 不认条目 slug) */
interface NekoFullPair { left: NekoImg; right: NekoImg; leftSlug?: string; rightSlug?: string }

/** 双侧且左右角色 slug 相同 = 同一角色镜像成一对 (禁止上站). 单侧恒为 false */
function mirrorRiskOf (p: NekoPair): boolean {
    return !!(p.left && p.right && p.leftSlug !== undefined && p.leftSlug === p.rightSlug);
}

/** 该图放进某槽位是否需要镜像; null = 需要镜像但标了 noFlip, 不能用 */
function flipFor (img: NekoImg, file: 'left' | 'right', slot: 'l' | 'r'): boolean | null {
    const anchor = img.anchor ?? (file === 'left' ? 'right' : 'left');
    const want = slot === 'l' ? 'right' : 'left';
    if (anchor === want) return false;
    return img.noFlip ? null : true;
}

/** 图的竖直裁切线在哪侧; 不填时按文件名约定 */
function anchorOf (img: NekoImg, file: 'left' | 'right'): 'left' | 'right' {
    return img.anchor ?? (file === 'left' ? 'right' : 'left');
}

/**
 * 一对素材落到左右槽: l = 放墙左侧的图 (裁切线在右), r = 放墙右侧的图.
 * 双侧: left 进左槽, right 进右槽, 落位相反时 (pair 变体) 镜像, noFlip 的整对 blocked.
 * 单侧: 只有的那张按它自己的锚点进槽 (裁切线在右进左槽, 在左进右槽), 永不镜像, 不做镜像禁用检查; 另一槽空着.
 */
interface SlotImgs { l: NekoImg | null; r: NekoImg | null; flipL: boolean; flipR: boolean; blocked: boolean }
function slotImgs (p: NekoPair): SlotImgs {
    if (p.left && p.right) {
        const fl = flipFor(p.left, 'left', 'l');
        const fr = flipFor(p.right, 'right', 'r');
        return { l: p.left, r: p.right, flipL: fl === true, flipR: fr === true, blocked: fl === null || fr === null };
    }
    const only = p.left ?? p.right;
    if (!only) return { l: null, r: null, flipL: false, flipR: false, blocked: false };
    const toL = anchorOf(only, p.left ? 'left' : 'right') === 'right';
    return { l: toL ? only : null, r: toL ? null : only, flipL: false, flipR: false, blocked: false };
}

/** 这一页的变体需要哪一槽有图: any = 至少一侧; l = 必须有左槽图 (side / corner / pocket 只用它); r = 必须有右槽图 (只出右侧的 gutter) */
type SlotNeed = 'any' | 'l' | 'r';
function eligible (p: NekoPair, need: SlotNeed): boolean {
    const s = slotImgs(p);
    if (need === 'l') return !!s.l;
    if (need === 'r') return !!s.r;
    return !!(s.l || s.r);
}

/* ---------------- 素材: manifest 优先, 内置 13 对回落 ---------------- */

type AssetSource = 'manifest' | 'builtin';
interface NekoAssets { pairs: readonly NekoPair[]; source: AssetSource }

/** 规格卡硬性项 (catgirl-spec.md §1): 原生宽 >= 600, 5:12 (±0.002), fade 0.84-0.94 */
function specOk (img: NekoImg): boolean {
    return img.w >= 600 && img.h > 0 && Math.abs(img.w / img.h - 5 / 12) <= 0.002
        && img.fade >= 0.84 && img.fade <= 0.94;
}

/** manifest 单图: { file | src, w | width, h | height, fade, anchor?, noFlip? }; 宽容解析, 不合规返回 null */
function parseImg (raw: unknown, slug: string): NekoImg | null {
    if (!raw || typeof raw !== 'object') return null;
    const o = raw as Record<string, unknown>;
    const file = typeof o.file === 'string' ? o.file : typeof o.src === 'string' ? o.src : null;
    const w = Number(o.w ?? o.width);
    const h = Number(o.h ?? o.height);
    const fade = Number(o.fade);
    if (!file || !Number.isFinite(w) || !Number.isFinite(h) || !Number.isFinite(fade)) return null;
    // 2026-11-06: manifest 里标了 excluded 的图 (旧命名残留 / 与其他图重复或互为镜像) 一律不上站.
    // 之前 parseCharacters 不看这个字段, 8 张残留图照样被抽中, 会出现"左右两侧像同一张图"的情况.
    if (o.excluded === true) return null;
    const img: NekoImg = {
        file: file.replace(/^\/?(img\/catgirl\/)?/, ''),
        w, h, fade,
        slug: typeof o.slug === 'string' ? o.slug : slug,
        anchor: o.anchor === 'left' || o.anchor === 'right' ? o.anchor : undefined,
        noFlip: o.noFlip === true,
    };
    return specOk(img) ? img : null;
}

/**
 * manifest.json 形状: { status, entries: [{ slug?, leftSlug?, rightSlug?, left: {file, w, h, fade}, right: {...} }] }.
 * 左右各自的角色: 优先取条目显式的 leftSlug / rightSlug; 没给时兜底: 该侧图自带的 slug, 再不行取文件名
 * "角色slug-序号-left|right.png" 的角色段. 条目级 slug 是整条的 id, 左右共用, 不参与镜像判断.
 * 双侧且左右角色相同的条目无条件丢弃 (禁止同一角色镜像成一对, 不管条目有没有 slug).
 * 单侧条目 (一侧为 null / 不合规, needsGen) 不丢: 按单侧收, 另一侧 null; 两侧都没有才丢.
 */
function parseManifest (json: unknown): NekoPair[] {
    const entries = (json as { entries?: unknown } | null)?.entries;
    if (!Array.isArray(entries)) return [];
    const avoid = avoidSetOf(json);
    const slugOf = (img: NekoImg): string => img.slug.replace(/-\d+-(left|right)(\.png)?$/i, '').replace(/-(l|r)$/, '');
    const out: NekoPair[] = [];
    for (const e of entries) {
        if (!e || typeof e !== 'object') continue;
        const o = e as Record<string, unknown>;
        const base = typeof o.slug === 'string' ? o.slug : '';
        const left = parseImg(o.left, base || String((o.left as { file?: string } | undefined)?.file ?? ''));
        const right = parseImg(o.right, base || String((o.right as { file?: string } | undefined)?.file ?? ''));
        if (!left && !right) continue;
        const sideSlug = (explicit: unknown, raw: unknown, img: NekoImg | null): string | undefined => {
            if (typeof explicit === 'string' && explicit) return explicit;
            if (!img) return undefined;
            const own = (raw as { slug?: unknown } | undefined)?.slug;
            return typeof own === 'string' && own ? slugOf(img) : slugOf({ ...img, slug: img.file });
        };
        const leftSlug = sideSlug(o.leftSlug, o.left, left);
        const rightSlug = sideSlug(o.rightSlug, o.right, right);
        if (left && right && leftSlug === rightSlug) continue;
        if (left && right && leftSlug && rightSlug && avoid.has(avoidKey(leftSlug, rightSlug))) continue;
        out.push({ left, right, leftSlug, rightSlug });
    }
    return out;
}

/**
 * characters 形状: [{ slug, character, characterCn, work, images: [{file, w, h, fade, anchor}] }].
 * 转成候选对: 每个角色按 anchor 分成"能进左槽的图"(裁切线在右) 与"能进右槽的图"(裁切线在左),
 * 再对所有不同角色两两交叉, 用取模轮转逐张取图, 让每个姿势都能轮到 (2026-11-04: 只用 20/56 张被用户指出浪费).
 * 交叉天生保证 leftSlug !== rightSlug, 不可能出现镜像成对.
 */
/**
 * 不许互配的角色对 (疑似同一角色, 配成左右一对看起来就是"同一个人镜像"). 内置一份兜底,
 * manifest.avoidPairs ([[slugA, slugB], ...]) 可追加. 2026-11-06: manifest.selfCheck 早就写了
 * "komari 与 unk-strawhat 疑似同一角色, 已避免互配", 但 parseCharacters 的两两交叉没管这条, 照样配了出来.
 */
const AVOID_PAIRS: readonly (readonly [string, string])[] = [['komari', 'unk-strawhat']];
function avoidKey (a: string, b: string): string {
    return a < b ? `${a}|${b}` : `${b}|${a}`;
}
function avoidSetOf (json: unknown): Set<string> {
    const set = new Set(AVOID_PAIRS.map(([a, b]) => avoidKey(a, b)));
    const extra = (json as { avoidPairs?: unknown } | null)?.avoidPairs;
    if (Array.isArray(extra)) {
        for (const p of extra) {
            if (Array.isArray(p) && typeof p[0] === 'string' && typeof p[1] === 'string') set.add(avoidKey(p[0], p[1]));
        }
    }
    return set;
}

function parseCharacters (json: unknown): NekoPair[] {
    const chars = (json as { characters?: unknown } | null)?.characters;
    if (!Array.isArray(chars)) return [];
    const avoid = avoidSetOf(json);
    const pools: { slug: string; left: NekoImg[]; right: NekoImg[] }[] = [];
    for (const c of chars) {
        if (!c || typeof c !== 'object') continue;
        const o = c as Record<string, unknown>;
        const slug = typeof o.slug === 'string' ? o.slug : '';
        if (!slug) continue;
        const imgs = Array.isArray(o.images) ? o.images : [];
        const left: NekoImg[] = [];
        const right: NekoImg[] = [];
        for (const raw of imgs) {
            const img = parseImg(raw, slug);
            if (!img) continue;
            // anchor 'right' = 裁切线在右 -> 进左槽; 'left' -> 进右槽
            if (img.anchor === 'left') right.push(img);
            else left.push(img);
        }
        if (left.length || right.length) pools.push({ slug, left, right });
    }
    const out: NekoPair[] = [];
    let k = 0;
    for (const a of pools) {
        for (const b of pools) {
            if (a.slug === b.slug) continue;
            if (avoid.has(avoidKey(a.slug, b.slug))) continue;
            if (!a.left.length || !b.right.length) continue;
            out.push({
                left: a.left[k % a.left.length],
                right: b.right[k % b.right.length],
                leftSlug: a.slug,
                rightSlug: b.slug,
            });
            k += 1;
        }
    }
    return out;
}

const BUILTIN: NekoAssets = { pairs: NEKO_PAIRS, source: 'builtin' };
let assetsCache: NekoAssets | null = null;
let assetsPromise: Promise<NekoAssets> | null = null;
function loadAssets (manifestUrl: string): Promise<NekoAssets> {
    if (assetsCache) return Promise.resolve(assetsCache);
    if (!assetsPromise) {
        assetsPromise = fetch(manifestUrl, { cache: 'no-cache' })
            .then((r) => (r.ok ? r.json() : null))
            .then((j) => {
                // characters 池优先 (全量姿势), 没有才退回 entries 固定对
                const pooled = parseCharacters(j);
                const pairs = pooled.length > 0 ? pooled : parseManifest(j);
                return pairs.length > 0 ? { pairs, source: 'manifest' as const } : BUILTIN;
            })
            .catch(() => BUILTIN)
            .then((a) => { assetsCache = a; return a; });
    }
    return assetsPromise;
}

/** 素材集; SSR 与加载完成前为 null (此时不渲染立绘) */
function useAssets (): NekoAssets | null {
    const manifestUrl = useBaseUrl('/img/catgirl/manifest.json');
    const [assets, setAssets] = React.useState<NekoAssets | null>(assetsCache);
    React.useEffect(() => {
        let alive = true;
        loadAssets(manifestUrl).then((a) => { if (alive) setAssets(a); });
        return () => { alive = false; };
    }, [manifestUrl]);
    return assets;
}

/**
 * 探头变体:
 * - pair: 左右对称, 正文块当墙 (z 2), 立绘 z 1 在墙后, 内侧裁切线藏进正文; 滚动越深身体越往上探.
 *   留白实测: 墙外沿到障碍物 (博客侧边栏 / TOC / 文档侧边栏) 或视口边, 不再按 50% ± 正文宽/2 推算.
 * - gutter: 墙是透明的内容容器 (文档 / 搜索 / 404 / 标签页), 立绘零侵入地住在容器外的留白里, 内侧裁切线靠水平淡出收口.
 * - corner: 从视口右下角斜探 (视口右边当墙), 暂无路由使用, CSS 保留.
 * - edges: 视口左右两边从底部升起半身 (全宽页面没有固定正文宽时用), 滚动升起更多.
 * - screen: 屏幕左右边界当墙, 两侧各从屏幕外探进来半个身子 (番剧库卡片墙铺满), 滚动多探一点.
 * - side: 只在右侧探头; 快速滚动时缩回墙后, 停下约 0.5 秒再探出来.
 * 所有变体: <= 996px 宽或 <= 500px 高不出墙后探头; <= 768 宽改出手机端 pocket (右下角窄条, 只露头肩, 见 NekoPocket 与 peek-layout.md §3); 入场从墙后滑出, 待机轻微呼吸; 滚动位移只写 --neko-p.
 * prefers-reduced-motion: 不监听滚动, 无入场 / 呼吸 / 躲藏, 静态探出幅度不变.
 */
export type NekoVariant = 'pair' | 'gutter' | 'corner' | 'edges' | 'side' | 'screen';

/** 上一次抽中的索引: 换页时避开它, 保证连续两页不是同一对 */
let lastPair = -1;
/** 从候选索引池里抽一个 (避开上一次); 池空返回 -1 */
function drawPair (pool: readonly number[]): number {
    if (pool.length === 0) return -1;
    const cand = pool.length > 1 ? pool.filter((i) => i !== lastPair) : pool;
    const i = cand[Math.floor(Math.random() * cand.length)];
    lastPair = i;
    return i;
}

/**
 * 每个页面一对: 以 pathname 与素材集为依赖在 effect 里抽, 整页刷新与站内换页都会重抽; 同页 re-render / 滚动不会重抽.
 * SSR 与首帧返回 null (不渲染立绘), 不会有水合不一致. 只在该变体用得上的素材里抽 (need); 一个都没有时返回 -1.
 */
function usePairForPage (pairs: readonly NekoPair[] | null, need: SlotNeed): number | null {
    const { pathname } = useLocation();
    const [idx, setIdx] = React.useState<number | null>(null);
    React.useEffect(() => {
        if (!pairs) return;
        const pool: number[] = [];
        pairs.forEach((p, i) => { if (eligible(p, need)) pool.push(i); });
        setIdx(drawPair(pool));
    }, [pathname, pairs, need]);
    return idx !== null && pairs !== null && idx < pairs.length ? idx : null;
}

/** 媒体查询是否命中; SSR 与首帧为 false */
function useMedia (query: string): boolean {
    const [ok, setOk] = React.useState(false);
    React.useEffect(() => {
        const mq = window.matchMedia(query);
        const update = (): void => setOk(mq.matches);
        update();
        mq.addEventListener('change', update);
        return () => mq.removeEventListener('change', update);
    }, [query]);
    return ok;
}

/** 桌面布局门槛: Docusaurus 在 996 收起侧边栏 / TOC; 高 <= 500 是横屏手机. 两者任一不满足都不渲染 */
const DESKTOP_QUERY = '(min-width: 997px) and (min-height: 501px)';

/**
 * 滚动联动: passive scroll + requestAnimationFrame, 每帧最多写一次 --neko-p (滚动进度, 一屏封顶为 1), 不触发 React 渲染.
 * shy 为 true 时 (side 变体) 单帧滚动超过 24px 打上 data-shy, 停止 480ms 后移除.
 * reduced-motion 时什么都不做.
 */
function useScrollPeek (ref: React.RefObject<HTMLDivElement | null>, active: boolean, shy: boolean, token: number): void {
    React.useEffect(() => {
        const el = ref.current;
        if (!el || !active) return;
        if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
        let raf = 0;
        let lastY = window.scrollY;
        let timer: number | undefined;
        const frame = (): void => {
            raf = 0;
            const y = window.scrollY;
            const p = Math.min(1, Math.max(0, y / window.innerHeight));
            el.style.setProperty('--neko-p', p.toFixed(3));
            if (shy && Math.abs(y - lastY) > 24) {
                el.dataset.shy = '';
                window.clearTimeout(timer);
                timer = window.setTimeout(() => { delete el.dataset.shy; }, 480);
            }
            lastY = y;
        };
        const onScroll = (): void => {
            if (!raf) raf = window.requestAnimationFrame(frame);
        };
        frame();
        window.addEventListener('scroll', onScroll, { passive: true });
        return () => {
            window.removeEventListener('scroll', onScroll);
            if (raf) window.cancelAnimationFrame(raf);
            window.clearTimeout(timer);
        };
    }, [ref, active, shy, token]);
}

/* ---------------- 墙与留白实测 (pair / gutter) ---------------- */

/**
 * pair 单侧宽度 (px). 2026-11-04: 下限 130 -> 72. 博客文章页在 BlogLayout 三栏里 (博客侧边栏 + 正文列 + TOC 列),
 * 正文块两侧缝只有 80-160px, 下限 130 时整页不出 (reason no-room). CSS 里 --neko-w 的 clamp 下限直接用这个常量.
 */
const NEKO_MIN = 72;
/** pair 完整模式 (内侧侵入墙后 NEKO_IN) 的最小宽; 低于它改紧凑模式: 不侵入, 整张站在留白里, 内侧靠 --neko-mask-x 淡出收口 */
const NEKO_FULL = 130;
/**
 * 紧凑模式外侧空隙 (完整模式仍用 NEKO_GAP 16). 紧凑成立条件: room - 8 >= 72, 即单侧缝 >= 80px.
 * 博客文章页门槛 (BlogLayout: 侧栏 col--2 + 2rem, 正文列 = 主列 78.25%, 墙 = BlogPostPage contentWidth 900,
 * >= 1536 时 CSS 收成 min(900, 正文列 - 240)): >= 1536 两侧缝恒 >= 120 -> 双侧紧凑 (约 112px); 1920 缝约 163 -> 双侧完整 (约 157px);
 * < 1536 不留缝, 1440 缝约 7, 1280 为 0 -> 不渲染 (no-room). 若墙回到 960 且不收缝, 门槛会变成正文列 >= 1120 (侧栏 col--3 时 vw >= 1905).
 * 详见 .agents/research/peek-layout.md §2 博客文章页一行与 §2.2.
 */
const NEKO_GAP_COMPACT = 8;
const NEKO_MAX = 380;
/** gutter 单侧宽度 (px) */
const GUTTER_MIN = 120;
const GUTTER_MAX = 260;
/** 立绘外侧 (靠障碍物 / 视口边) 至少留的空隙 */
const NEKO_GAP = 16;
/** pair 内侧侵入墙后的比例 (与 CSS --neko-in 一致); 仅作 --neko-in-ratio 的兜底默认值 */
const NEKO_IN = 0.06;
/**
 * 画布最多藏进墙后的比例 (2026-11-04 用户嫌立绘小). 立绘 z 1 在正文块 z 2 后面, 压进墙的那部分完全看不见,
 * 所以画布可以放大到近两倍, 露出宽度仍只由留白决定: visible = room - GAP_COMPACT, w = visible / (1 - NEKO_IN_MAX).
 */
const NEKO_IN_MAX = 0.55;

/** 立绘不许盖住的东西: 文档侧边栏 / 博客侧边栏 (aside) / TOC */
const OBSTACLES = '.theme-doc-sidebar-container, aside, .table-of-contents, .theme-doc-toc-desktop';

interface WallCfg {
    /** gutter 的墙选择器 (按顺序找第一个可见的); pair 固定找同容器里的 .blog-content */
    wall?: readonly string[];
    /** 墙找不到时按视口居中的这个宽度推算 (px); 不给就不渲染 */
    fallbackWidth?: number;
    /** 出哪几侧 */
    sides: 'both' | 'r';
}

interface Fit { l: boolean; r: boolean; w: number; roomL: number; roomR: number; compact: boolean; ratio: number }

function visibleRect (el: Element): DOMRect | null {
    const r = el.getBoundingClientRect();
    if (r.width < 1 || r.height < 1) return null;
    const cs = window.getComputedStyle(el);
    if (cs.display === 'none' || cs.visibility === 'hidden') return null;
    return r;
}

/**
 * 量墙外沿与两侧留白, 写 --neko-wl / --neko-wr / --neko-w (视口 px); 返回哪几侧放得下.
 * 留白 = 墙外沿到最近障碍物 (完全在墙外侧的侧边栏 / TOC) 或视口边. 墙 / 障碍物尺寸变化 (ResizeObserver) 与窗口缩放时重量;
 * 墙晚挂载时 (路由切换) 每 250ms 重找一次, 最多 12 次.
 */
function useWallFit (
    ref: React.RefObject<HTMLDivElement | null>, variant: 'pair' | 'gutter', cfg: WallCfg, contentWidth: number,
    active: boolean, token: number, wantL: boolean, wantR: boolean,
): Fit | null {
    const [fit, setFit] = React.useState<Fit | null>(null);
    const wallKey = (cfg.wall ?? []).join('|');
    React.useEffect(() => {
        const el = ref.current;
        if (!el || !active) { setFit(null); return; }
        let raf = 0;
        let tries = 0;
        let retry: number | undefined;
        let ro: ResizeObserver | null = null;
        let observed: Element | null = null;
        let marked: Element | null = null;
        const findWall = (): Element | null => {
            if (variant === 'pair') return el.parentElement?.querySelector('.blog-content') ?? null;
            for (const sel of cfg.wall ?? []) {
                const hit = Array.from(document.querySelectorAll(sel)).find((n) => visibleRect(n));
                if (hit) return hit;
            }
            return null;
        };
        const measure = (): void => {
            raf = 0;
            const vw = document.documentElement.clientWidth;
            const wall = findWall();
            let left: number;
            let right: number;
            if (wall) {
                const r = wall.getBoundingClientRect();
                left = r.left;
                right = r.right;
                if (ro && observed !== wall) { if (observed) ro.unobserve(observed); ro.observe(wall); observed = wall; }
            } else if (cfg.fallbackWidth || variant === 'pair') {
                // INFIMA_CONTAINER: Infima .container 在 >= 1440 宽是 1320, 以下是 1140 (B14)
                const fw = cfg.fallbackWidth === INFIMA_CONTAINER && vw >= 1440 ? 1320 : (cfg.fallbackWidth ?? 0);
                const cw = variant === 'pair' ? contentWidth : fw;
                left = (vw - cw) / 2;
                right = (vw + cw) / 2;
                if (tries < 12 && variant === 'gutter') { tries += 1; retry = window.setTimeout(schedule, 250); }
            } else {
                setFit(null);
                if (tries < 12) { tries += 1; retry = window.setTimeout(schedule, 250); }
                return;
            }
            let limL = 0;
            let limR = vw;
            for (const ob of Array.from(document.querySelectorAll(OBSTACLES))) {
                if (el.contains(ob) || wall?.contains(ob) || ob.contains(wall as Node)) continue;
                const r = visibleRect(ob);
                if (!r) continue;
                if (r.right <= left + 1) limL = Math.max(limL, r.right);
                else if (r.left >= right - 1) limR = Math.min(limR, r.left);
            }
            const roomL = Math.max(0, left - limL);
            const roomR = Math.max(0, limR - right);
            const short = window.innerHeight <= 640 ? 0.85 : 1;
            let w: number;
            let l: boolean;
            let r: boolean;
            let compact = false;
            let ratio = NEKO_IN;
            if (variant === 'pair') {
                // 画布尽量大, 露出宽度由留白决定, 其余藏进墙后 (看不见). 单调: room 越大 w 越大, 无跳变.
                const pairW = (room: number): { w: number; compact: boolean; ratio: number } | null => {
                    const visible = Math.max(0, room - NEKO_GAP_COMPACT) * short;
                    if (visible < NEKO_MIN * short) return null;
                    if (visible >= NEKO_MAX) return { w: NEKO_MAX, compact: false, ratio: 0 };
                    const pw = Math.min(NEKO_MAX, visible / (1 - NEKO_IN_MAX));
                    const pr = Math.max(0, 1 - visible / pw);
                    return { w: pw, compact: pr > 0.2, ratio: pr };
                };
                let fitW: { w: number; compact: boolean; ratio: number } | null;
                if (wantL && wantR) {
                    // 双侧优先 (等宽, 按较窄一侧); 较窄一侧放不下时退化为单侧: 只出 room 大的那侧, 宽度按那侧算, 不被窄侧拖累
                    fitW = pairW(Math.min(roomL, roomR));
                    if (fitW) {
                        l = true;
                        r = true;
                    } else {
                        const goL = roomL > roomR;
                        fitW = pairW(goL ? roomL : roomR);
                        l = !!fitW && goL;
                        r = !!fitW && !goL;
                    }
                } else {
                    // 单侧素材只按出的那一侧算
                    fitW = pairW(wantL ? roomL : roomR);
                    l = !!fitW && wantL;
                    r = !!fitW && wantR;
                }
                w = fitW ? fitW.w : 0;
                compact = !!fitW && fitW.compact;
                ratio = fitW ? fitW.ratio : NEKO_IN;
            } else {
                const need = GUTTER_MIN * short;
                // B17: 放不下时 room - GAP 可能为负, 夹到 0 (show 为 false, reason = no-room)
                const wl = Math.max(0, Math.min(GUTTER_MAX, roomL - NEKO_GAP) * short);
                const wr = Math.max(0, Math.min(GUTTER_MAX, roomR - NEKO_GAP) * short);
                const canL = cfg.sides === 'both' && wantL && wl >= need;
                const canR = wantR && wr >= need;
                if (wantL && wantR) {
                    // 两侧都放得下才双侧等宽; 否则只出放得下的那一侧, 不再要求左侧必须跟着右侧
                    r = canR;
                    l = canL;
                    if (l && r) w = Math.min(wl, wr);
                    else if (r) w = wr;
                    else w = wl;
                } else {
                    // 单侧素材: 只看有图的那一侧
                    l = canL;
                    r = canR;
                    w = l ? wl : wr;
                }
            }
            // gutter 的墙原本透明: 真出立绘时才标记, 由 CSS 给底色 + 1px 边 + 上下外边距, 和 pair 的正文墙一个样子 (2026-11-06)
            const markOn = variant === 'gutter' && wall && (l || r) ? wall : null;
            if (marked !== markOn) { marked?.removeAttribute('data-neko-wall'); markOn?.setAttribute('data-neko-wall', ''); marked = markOn; }
            el.style.setProperty('--neko-wl', `${left}px`);
            el.style.setProperty('--neko-wr', `${right}px`);
            el.style.setProperty('--neko-w', `${Math.round(w)}px`);
            setFit((old) => (old && old.l === l && old.r === r && old.compact === compact
                && Math.abs(old.w - w) < 1 && Math.abs(old.ratio - ratio) < 0.01
                ? old
                : { l, r, w: Math.round(w), roomL: Math.round(roomL), roomR: Math.round(roomR), compact, ratio }));
        };
        function schedule (): void {
            if (!raf) raf = window.requestAnimationFrame(measure);
        }
        ro = typeof ResizeObserver === 'undefined' ? null : new ResizeObserver(schedule);
        ro?.observe(document.body);
        measure();
        window.addEventListener('resize', schedule);
        return () => {
            ro?.disconnect();
            marked?.removeAttribute('data-neko-wall');
            window.removeEventListener('resize', schedule);
            if (raf) window.cancelAnimationFrame(raf);
            window.clearTimeout(retry);
        };
    }, [ref, variant, wallKey, cfg.fallbackWidth, cfg.sides, contentWidth, active, token, wantL, wantR]);
    return fit;
}

/* ---------------- 挂载登记 (防叠组) 与 dev 自检 ---------------- */

/** 页面自己挂的 SideCats 实例; 非空时 RouteCats 不渲染 (peek-layout.md §5, 修 /anime/details 叠两组) */
const pageStages = new Set<symbol>();
const stageListeners = new Set<() => void>();
function subscribeStages (fn: () => void): () => void {
    stageListeners.add(fn);
    return () => { stageListeners.delete(fn); };
}
function emitStages (): void {
    stageListeners.forEach((fn) => fn());
}
function usePageOwnsStage (): boolean {
    return React.useSyncExternalStore(subscribeStages, () => pageStages.size > 0, () => false);
}
function useRegisterPageStage (): void {
    React.useEffect(() => {
        const id = Symbol('neko-page-stage');
        pageStages.add(id);
        emitStages();
        return () => { pageStages.delete(id); emitStages(); };
    }, []);
}

type Reason = 'no-asset' | 'rendered' | 'narrow-or-short' | 'pocket-blocked' | 'pocket-page-off' | 'pocket-wait-footer' | 'no-room' | 'loading' | 'off' | 'no-route' | 'page-owns' | 'img-error';
interface DebugState {
    variant: NekoVariant | 'off' | null;
    source: 'page' | 'route' | null;
    reason: Reason;
    pair: number | null;
    files: [string, string] | null;
    /** 当前这一对左右角色 slug 相同 (同一角色镜像成对) = true; 期望恒为 false */
    mirrorRisk?: boolean | null;
    assets: AssetSource | null;
    room: { l: number; r: number; w: number } | null;
    /** desktop = 墙后探头 (> 996 宽且 > 500 高); pocket = 手机端页尾小探头 */
    mode?: 'desktop' | 'pocket';
    /** pocket 实测: 槽矩形 (视口坐标, 量的那一刻) 与压到的元素 */
    pocket?: { w: number; h: number; bottom: number; blockedBy: string[] } | null;
}
const debugState: { route: DebugState | null; page: DebugState | null } = { route: null, page: null };

/** 可点击元素: 立绘不许盖住 (页脚里的除外, 页脚 z 4 在立绘之上; 正文块里的除外, pair 墙 z 2 在立绘之上) */
const CLICKABLE = 'a[href], button, input, select, textarea, [role="button"], .navbar, .theme-doc-sidebar-container, .table-of-contents, .theme-back-to-top-button';

/**
 * window.__nekoDebug(): 当前页立绘的自检报告 (只读, 无副作用). 给本机 / playwright 验收用.
 * 返回 { path, variant, source, rendered, reason, pair, files, mirrorRisk, assets, room, stages, slots, hScroll, overlaps, reducedMotion }.
 * slots 的矩形取人物所在的下 55% (规格卡: 上 45% 全透明); overlaps 期望为空, hScroll 期望 false, stages 恒为 0 或 1.
 */
function nekoDebug (): Record<string, unknown> {
    const st = debugState.page ?? debugState.route;
    const stages = document.querySelectorAll('.neko-sides[data-on], .neko-pocket[data-on]').length;
    // B16: 隐藏中的 pocket (未 data-on, visibility: hidden) 不算
    const slots = Array.from(document.querySelectorAll<HTMLElement>('.neko-slot, .neko-pocket[data-on]')).map((s) => {
        const r = s.getBoundingClientRect();
        // 墙后探头: 人物在下 55%; pocket 本身已裁到头肩, 整块都算
        const top = s.classList.contains('neko-pocket') ? r.top : r.top + r.height * 0.45;
        return {
            side: s.classList.contains('neko-slot-l') ? 'l' : 'r',
            rect: { left: Math.round(r.left), right: Math.round(r.right), top: Math.round(top), bottom: Math.round(r.bottom) },
        };
    });
    const vw = window.innerWidth;
    const overlaps: string[] = [];
    const pair = st?.variant === 'pair';
    for (const node of Array.from(document.querySelectorAll(CLICKABLE))) {
        if (node.closest('.footer') || node.closest('.neko-sides') || node.closest('.neko-pocket') || (pair && node.closest('.blog-content'))) continue;
        const r = visibleRect(node);
        if (!r) continue;
        for (const s of slots) {
            const a = s.rect;
            const vis = { left: Math.max(a.left, 0), right: Math.min(a.right, vw) };
            if (r.left < vis.right && r.right > vis.left && r.top < a.bottom && r.bottom > a.top) {
                const tag = node.tagName.toLowerCase();
                const cls = (node.getAttribute('class') ?? '').split(/\s+/).filter(Boolean).slice(0, 2).join('.');
                overlaps.push(`${s.side}:${tag}${cls ? `.${cls}` : ''}`);
                break;
            }
        }
    }
    return {
        path: window.location.pathname,
        variant: st?.variant ?? null,
        source: st?.source ?? null,
        rendered: st?.reason === 'rendered',
        reason: st?.reason ?? 'no-route',
        pair: st?.pair ?? null,
        files: st?.files ?? null,
        mirrorRisk: st?.mirrorRisk ?? null,
        assets: st?.assets ?? null,
        room: st?.room ?? null,
        stages,
        slots,
        hScroll: document.documentElement.scrollWidth > window.innerWidth,
        overlaps: Array.from(new Set(overlaps)),
        mode: st?.mode ?? null,
        pocket: st?.pocket ?? null,
        reducedMotion: window.matchMedia('(prefers-reduced-motion: reduce)').matches,
        viewport: { w: window.innerWidth, h: window.innerHeight },
    };
}
if (typeof window !== 'undefined') {
    (window as unknown as { __nekoDebug: typeof nekoDebug }).__nekoDebug = nekoDebug;
}

/** state 为 null 时不上报 (由子舞台自己上报) */
function useDebugReport (source: 'page' | 'route', state: DebugState | null): void {
    const key = JSON.stringify(state);
    React.useEffect(() => {
        if (!state) return;
        debugState[source] = state;
        return () => { if (debugState[source] === state) debugState[source] = null; };
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [source, key]);
}

/* ---------------- 手机端: 右下角窄条探头 (pocket) ---------------- */

/*
 * 2026-11-03 用户定稿的方向 A (peek-layout.md §3): <= 768 宽时在视口右下角放一条窄立绘, 只露头和肩.
 *   T 481-768: 宽 44px, 高 18vh, 底边抬 72px (返回顶部按钮之上)
 *   N <= 480 : 宽 36px, 高 14vh, 底边抬 64px
 *   S 769-996: 不出 (侧边栏已收起但正文仍宽, 用户定为保留不出)
 * 用 left 图 (裁切线在右), 右对齐贴视口右边 = 屏幕边当墙, 和番剧库 screen 变体同一套朝向.
 * 只在页面滚到页脚进入视口时出现 ("页脚上方"), 每次出现前用 getBoundingClientRect 实测:
 * 与正文文字 / 可点击元素 (含页脚链接) 有交集就不显示, 原因与压到的元素写进 __nekoDebug().pocket.
 * 有侧边栏 / TOC 的文档页与博客文章页不出 (用户定), reason = pocket-page-off.
 */
const POCKET_TEXT = 'p, li, h1, h2, h3, h4, h5, h6, pre, table, blockquote, figure, img:not(.neko-pocket-img)';
const POCKET_QUERY = '(max-width: 768px)';

function pocketBlockers (el: HTMLElement): string[] {
    const r = el.getBoundingClientRect();
    if (r.width < 1 || r.height < 1) return ['zero-size'];
    const hit: string[] = [];
    for (const n of Array.from(document.querySelectorAll(`${CLICKABLE}, ${POCKET_TEXT}`))) {
        if (el.contains(n) || n.contains(el)) continue;
        // B15: 页脚不是障碍: pocket 只在页脚进视口时出, 底边抬 64 / 72px 本来就叠在页脚上沿; 只豁免祖先含 .footer 的元素
        if (n.closest('.footer')) continue;
        if (n.matches('.navbar') || n.closest('.navbar')) continue; // 导航栏在顶部, 和右下角不可能相交; 跳过省开销
        const b = visibleRect(n);
        if (!b) continue;
        if (b.left < r.right && b.right > r.left && b.top < r.bottom && b.bottom > r.top) {
            const cls = (n.getAttribute('class') ?? '').split(/\s+/).filter(Boolean).slice(0, 2).join('.');
            hit.push(`${n.tagName.toLowerCase()}${cls ? `.${cls}` : ''}`);
        }
    }
    return Array.from(new Set(hit)).slice(0, 8);
}

/** 博客文章页 (有 TOC / 博客侧栏): 不出 pocket. 列表 / 分页 / 归档 / 标签 / 作者页照出 */
function isBlogPost (path: string): boolean {
    if (!path.startsWith('/blog/')) return false;
    return !/^\/blog\/(page\/|archive|tags|authors)/.test(path);
}

function NekoPocket ({ source, variant, pageOff }: { source: 'page' | 'route'; variant: NekoVariant; pageOff: boolean }): React.ReactElement | null {
    const band = useMedia(POCKET_QUERY);
    const assets = useAssets();
    // pocket 只用左槽图 (裁切线在右, 贴视口右边); 只在有左槽图的素材里抽
    const picked = usePairForPage(assets ? assets.pairs : null, 'l');
    const [footerIn, setFooterIn] = React.useState(false);
    const [blocked, setBlocked] = React.useState<string[] | null>(null);
    const [size, setSize] = React.useState<{ w: number; h: number; bottom: number } | null>(null);
    const [imgError, setImgError] = React.useState(false);
    const ref = React.useRef<HTMLDivElement>(null);
    const { pathname } = useLocation();
    const noAsset = picked === -1;
    const idx = picked !== null && picked >= 0 ? picked : 0;
    const pair: NekoPair = assets?.pairs[idx] ?? NEKO_PAIRS[0];
    const img = slotImgs(pair).l ?? NEKO_PAIRS[0].left;
    const src = useBaseUrl(`/img/catgirl/${img.file}`);
    const active = band && !pageOff && picked !== null && !noAsset && !imgError;
    // 页脚进入视口才考虑出现
    React.useEffect(() => {
        setFooterIn(false);
        setBlocked(null);
        if (!active) return;
        const footer = document.querySelector('.footer');
        if (!footer) { setFooterIn(true); return; } // 无页脚的页面: 视作一直在页尾
        const io = new IntersectionObserver((es) => setFooterIn(es.some((e) => e.isIntersecting)));
        io.observe(footer);
        return () => io.disconnect();
    }, [active, pathname]);
    // 实测: 在页脚可见期间, 滚动 / 尺寸变化后重新量一次是否压到文字或可点击元素
    React.useEffect(() => {
        const el = ref.current;
        if (!el || !active || !footerIn) return;
        let t = 0;
        const measure = (): void => {
            const r = el.getBoundingClientRect();
            setSize({ w: Math.round(r.width), h: Math.round(r.height), bottom: Math.round(window.innerHeight - r.bottom) });
            setBlocked(pocketBlockers(el));
        };
        const schedule = (): void => { window.clearTimeout(t); t = window.setTimeout(measure, 80); };
        measure();
        window.addEventListener('scroll', schedule, { passive: true });
        window.addEventListener('resize', schedule);
        return () => { window.clearTimeout(t); window.removeEventListener('scroll', schedule); window.removeEventListener('resize', schedule); };
    }, [active, footerIn, idx, pathname]);
    const ok = active && footerIn && blocked !== null && blocked.length === 0;
    let reason: Reason = 'rendered';
    if (!band) reason = 'narrow-or-short';
    else if (pageOff) reason = 'pocket-page-off';
    else if (imgError) reason = 'img-error';
    else if (picked === null) reason = 'loading';
    else if (noAsset) reason = 'no-asset';
    else if (!footerIn) reason = 'pocket-wait-footer';
    else if (blocked === null) reason = 'loading';
    else if (!ok) reason = 'pocket-blocked';
    useDebugReport(source, {
        variant, source, reason, mode: 'pocket',
        pair: picked, files: [img.file, img.file], mirrorRisk: mirrorRiskOf(pair), assets: assets?.source ?? null, room: null,
        pocket: size ? { ...size, blockedBy: blocked ?? [] } : null,
    });
    if (!active || typeof document === 'undefined') return null;
    // 量的时候 visibility: hidden 占位 (不吃点击, 不可见), 合规才 data-on
    const style = { '--neko-ratio': `${img.h / img.w}` } as React.CSSProperties;
    return createPortal(
        <div className="neko-pocket" ref={ref} style={style} data-on={ok ? '' : undefined} aria-hidden="true">
            <img className="neko-pocket-img" src={src} width={img.w} height={img.h} alt="" decoding="async"
                onError={() => setImgError(true)} />
        </div>,
        document.body,
    );
}

/* ---------------- 舞台 ---------------- */

interface StageProps {
    variant: NekoVariant;
    source: 'page' | 'route';
    /** pair 用: 正文宽度, 墙没量到时按它推算 */
    contentWidth?: number;
    /** gutter 用 */
    wall?: WallCfg;
    /** 手机端不出 pocket (有侧边栏 / TOC 的页) */
    pocketOff?: boolean;
}

const GUTTER_DEFAULT: WallCfg = { wall: ['main .container', 'main'], sides: 'both' };

/** 视口墙变体 (不量墙) 的最小视口宽 */
const VIEWPORT_QUERY: Partial<Record<NekoVariant, string>> = {
    corner: '(min-width: 1400px)', // 番剧库正文 1200 宽, 右侧留白 >= 100px 才放得下不压海报
    screen: '(min-width: 1360px)', // 番剧库卡片墙 1200 宽, 两侧留白 >= 80px, 探进来的身子才不大面积压海报
    side: '(min-width: 1200px)', // 博客标签 / 作者页右侧 21.75% 空列 >= 196px
    edges: '(min-width: 1100px)',
};

/**
 * 立绘舞台: 一个页面只挂一组 (槽 position: fixed). 外层 .neko-slot 吃滚动 transform, 内层 img 吃入场 / 呼吸动画, 两者不抢 transform.
 * pair / gutter 先挂一个空的 .neko-sides 量墙, 放得下才出槽; 其余变体按视口宽媒体查询.
 * .agents/notes/implemented/feature/2026-11-02-blog-catgirl-sides.md
 */
function NekoStage ({ variant, source, contentWidth = 960, wall = GUTTER_DEFAULT, pocketOff = false }: StageProps): React.ReactElement | null {
    const desktop = useMedia(DESKTOP_QUERY);
    const wide = useMedia(VIEWPORT_QUERY[variant] ?? '(min-width: 0px)');
    const assets = useAssets();
    // side / corner 只出右槽且用左槽图; 只出右侧的 gutter 只用右槽图; 其余至少有一侧即可
    const need: SlotNeed = variant === 'side' || variant === 'corner' ? 'l' : variant === 'gutter' && wall.sides === 'r' ? 'r' : 'any';
    const picked = usePairForPage(assets ? assets.pairs : null, need);
    const ref = React.useRef<HTMLDivElement>(null);
    const [imgError, setImgError] = React.useState(false);
    const noAsset = picked === -1;
    const idx = picked !== null && picked >= 0 ? picked : 0;
    const measured = variant === 'pair' || variant === 'gutter';
    const ready = desktop && picked !== null && !noAsset && !imgError;
    const pair: NekoPair = assets?.pairs[idx] ?? NEKO_PAIRS[0];
    const slots = slotImgs(pair);
    const fit = useWallFit(ref, variant === 'pair' ? 'pair' : 'gutter', variant === 'pair' ? { sides: 'both' } : wall,
        contentWidth, ready && measured, idx, !!slots.l, !!slots.r);
    const left = useBaseUrl(`/img/catgirl/${slots.l?.file ?? ''}`);
    const right = useBaseUrl(`/img/catgirl/${slots.r?.file ?? ''}`);
    const flipL = slots.flipL;
    const flipR = slots.flipR;
    // 镜像禁用只管双侧 (slotImgs 对单侧恒给 blocked = false)
    const flipBlocked = variant === 'pair' && slots.blocked;
    const showL = measured ? !!fit?.l : (variant === 'edges' || variant === 'screen') && !!slots.r;
    const showR = measured ? !!fit?.r : !!slots.l;
    const show = ready && !flipBlocked && (measured ? showL || showR : wide);
    // idx 变化时 key 让外层重挂载, token 让监听跟着换到新元素上
    useScrollPeek(ref, show, variant === 'side', idx);
    let reason: Reason = 'rendered';
    if (!desktop) reason = 'narrow-or-short'; // 手机端由 NekoPocket 接管上报 (下面 useDebugReport 传 null)
    else if (imgError) reason = 'img-error';
    else if (picked === null) reason = 'loading';
    else if (noAsset) reason = 'no-asset';
    else if (!show) reason = 'no-room';
    useDebugReport(source, desktop ? {
        variant, source, reason, mode: 'desktop',
        pair: picked,
        files: [slots.l?.file ?? '', slots.r?.file ?? ''],
        mirrorRisk: mirrorRiskOf(pair),
        assets: assets?.source ?? null,
        room: fit ? { l: fit.roomL, r: fit.roomR, w: fit.w } : null,
    } : null);
    React.useEffect(() => { setImgError(false); }, [idx]);
    if (!desktop) return <NekoPocket source={source} variant={variant} pageOff={pocketOff} />;
    if (!ready) return null;
    const style = variant === 'pair'
        ? {
            // 首帧推算值, 下限与 NEKO_MIN (72) 同步; 量完墙后 useWallFit 用 setProperty 覆盖成实测宽
            '--neko-w': `clamp(${NEKO_MIN}px, calc((100vw - ${contentWidth}px) / 2 - ${NEKO_GAP}px), ${NEKO_MAX}px)`,
            '--neko-cw': `${contentWidth}px`,
            // 量完墙后由 useWallFit 写入实测 ratio; 首帧用默认 6%
            '--neko-in-ratio': String(fit?.ratio ?? NEKO_IN),
        } as React.CSSProperties
        : undefined;
    const onError = (): void => setImgError(true);
    const leftImg = slots.l && <img className="neko-img" src={left} width={slots.l.w} height={slots.l.h} style={{ '--neko-fade': `${slots.l.fade * 100}%` } as React.CSSProperties} alt="" decoding="async" onError={onError} />;
    const rightImg = slots.r && <img className="neko-img" src={right} width={slots.r.w} height={slots.r.h} style={{ '--neko-fade': `${slots.r.fade * 100}%` } as React.CSSProperties} alt="" decoding="async" onError={onError} />;
    // pair / gutter: left 图 (裁切线在右) 放墙左侧, right 图放墙右侧. 视口墙变体: 视口右边放 left 图, 视口左边放 right 图, 不镜像
    return (
        <div className="neko-sides" data-variant={variant} data-on={show ? '' : undefined} data-compact={variant === 'pair' && fit?.compact ? '' : undefined} key={idx} ref={ref} style={style} aria-hidden="true">
            {show && measured && showL && leftImg && <div className={`neko-slot neko-slot-l${variant === 'pair' && flipL ? ' neko-flip' : ''}`}>{leftImg}</div>}
            {show && measured && showR && rightImg && <div className={`neko-slot neko-slot-r${variant === 'pair' && flipR ? ' neko-flip' : ''}`}>{rightImg}</div>}
            {show && showL && (variant === 'edges' || variant === 'screen') && <div className="neko-slot neko-slot-l">{rightImg}</div>}
            {show && !measured && showR && <div className="neko-slot neko-slot-r">{leftImg}</div>}
        </div>
    );
}

/** 博客 / cv / 友链 / 分享 / 番剧详情页用: 左右对称墙后探头. 挂载即登记, RouteCats 见到登记就让位 */
export function SideCats ({ contentWidth = 960 }: { contentWidth?: number }): React.ReactElement | null {
    useRegisterPageStage();
    const { pathname } = useLocation();
    const { siteConfig } = useDocusaurusContext();
    const base = siteConfig.baseUrl.replace(/\/$/, '');
    const path = pathname.startsWith(base) ? pathname.slice(base.length) : pathname;
    return <NekoStage variant="pair" source="page" contentWidth={contentWidth} pocketOff={isBlogPost(path)} />;
}

/* ---------------- 路由表 (peek-layout.md §2) ---------------- */

interface RouteRule {
    /** 精确路径, 或以 / 结尾的前缀 (前缀同时匹配不带尾斜杠的自身) */
    match: string;
    variant: NekoVariant | 'off';
    wall?: WallCfg;
}

const INFIMA_CONTAINER = 1140;
/**
 * 文档 / 知识库: 墙 = 文档容器 (含 TOC). 2026-11-04 起两侧: 左缝 = 容器左沿到文档侧边栏右沿 (侧边栏是障碍物, 实测扣掉),
 * 放得下就出左侧, 放不下自动只出右侧 (useWallFit gutter 分支). 原来只出右侧, 用户反馈"有的地方应该左右都有".
 */
const DOC_WALL: WallCfg = { wall: ['[class*="docMainContainer"] > .container', 'main .container'], sides: 'both' };
/** 标签页: 1120 内容列 (DocTag* 主题的 .inner) */
const TAG_WALL: WallCfg = { wall: ['[class*="inner_"]', 'main .container'], fallbackWidth: 1120, sides: 'both' };
/** 搜索 / 404: Docusaurus 居中容器 */
/** B14: 搜索 / 404 没有 <main>, 墙找不到时按视口中线 ± Infima 容器宽 / 2 推算 */
const CENTER_WALL: WallCfg = { wall: ['main .container', 'main.container'], fallbackWidth: INFIMA_CONTAINER, sides: 'both' };
/** B21: 番剧详情是卡片式布局, 墙 = .shell (CSS Module 哈希类名) 外沿 */
const DETAIL_WALL: WallCfg = { wall: ['[class*=shell]'], sides: 'both' };

/**
 * 没有挂 SideCats 的页面按路由选变体 (挂在 src/theme/Root.tsx, 全站一份). 顺序即优先级, 第一条命中即用.
 * 页面自己挂了 SideCats 的 (博客列表 / 文章 / 归档 / cv / 友链 / 分享) 由挂载登记让位, 不写进表.
 * off = 明确不渲染, 理由见 .agents/research/peek-layout.md §2 (#21-#24): 演示 / 编辑器 / 投屏页四周都是控件.
 * 表外路由 (含 404) 走 FALLBACK: gutter + 居中容器, 量不到墙或留白不够就不渲染.
 */
const ROUTE_RULES: readonly RouteRule[] = [
    { match: '/', variant: 'edges' },
    // 番剧库卡片墙铺满 1200px, 没有正文块可躲: 屏幕边当墙 (screen)
    { match: '/anime', variant: 'screen' },
    { match: '/anime/graph', variant: 'edges' },
    // B21: 详情页是卡片式布局 (shell 透明, 无 z-index), pair 的"身体藏进墙后"不成立; gutter, 墙取 shell 外沿
    { match: '/anime/details', variant: 'gutter', wall: DETAIL_WALL },
    { match: '/docs-graph', variant: 'edges' },
    { match: '/blog/tags/', variant: 'side' },
    { match: '/blog/authors/', variant: 'side' },
    { match: '/docs/tags/', variant: 'gutter', wall: TAG_WALL },
    { match: '/knowledge-base/tags/', variant: 'gutter', wall: TAG_WALL },
    { match: '/docs/', variant: 'gutter', wall: DOC_WALL },
    { match: '/knowledge-base/', variant: 'gutter', wall: DOC_WALL },
    { match: '/search', variant: 'gutter', wall: CENTER_WALL },
    { match: '/hxdeck', variant: 'off' },
    { match: '/hxdeck-editor', variant: 'off' },
    { match: '/ppt', variant: 'off' },
    { match: '/drawio', variant: 'off' },
];
const FALLBACK: RouteRule = { match: '*', variant: 'gutter', wall: CENTER_WALL };

function ruleFor (path: string): RouteRule {
    for (const r of ROUTE_RULES) {
        if (r.match.endsWith('/') && r.match !== '/') {
            if (path === r.match.slice(0, -1) || path.startsWith(r.match)) return r;
        } else if (path === r.match) return r;
    }
    return FALLBACK;
}

export function RouteCats (): React.ReactElement | null {
    const { pathname } = useLocation();
    const { siteConfig } = useDocusaurusContext();
    const owned = usePageOwnsStage();
    const base = siteConfig.baseUrl.replace(/\/$/, '');
    let path = pathname.startsWith(base) ? pathname.slice(base.length) : pathname;
    path = path.replace(/\/+$/, '') || '/';
    try { path = decodeURI(path); } catch { /* 保留原样 */ }
    const rule = ruleFor(path);
    const idle: DebugState = {
        variant: rule.variant, source: 'route', reason: owned ? 'page-owns' : 'off',
        pair: null, files: null, assets: null, room: null,
    };
    const idleOnly = owned || rule.variant === 'off';
    useDebugReport('route', idleOnly ? idle : null);
    const variant = rule.variant;
    if (owned || variant === 'off') return null;
    // 文档 / 知识库正文页 (DOC_WALL) 有侧边栏与 TOC: 手机端不出 pocket
    return <NekoStage key={`${variant}:${rule.match}`} variant={variant} source="route" wall={rule.wall} pocketOff={rule.wall === DOC_WALL} />;
}

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
interface Props {
    style?: React.CSSProperties;
    children: React.ReactNode;
    /** 正文最大宽度, 默认 960 (与 .blog-content-wrapper 一致) */
    contentWidth?: number;
}
/**
 * 两侧带猫娘的正文容器: cv / 友链 / 博客列表与文章 / 归档 / 番剧详情在用. 正文是带底色的圆角块 (墙), 猫娘见 NekoStage.
 * 决策记录见本文件 NekoStage 的注释 (同一目录只留一个锚点).
 */
const BlogWithCats: React.FC<Props> = ({ style, children, contentWidth = 960 }) => {
    return (
        <div className="blog-container">
            <SideCats contentWidth={contentWidth} />
            <div className="blog-content-wrapper" style={contentWidth === 960 ? undefined : { maxWidth: contentWidth }}>
                <div className="blog-content" style={style}>
                    {children}
                </div>
            </div>
        </div>
    );
};

export default BlogWithCats;
