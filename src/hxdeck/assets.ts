/**
 * 静态资源解析 + 主题素材槽位.
 *
 * 两种图片, 语义完全不同 —— 这是本系统的一条核心区分:
 *
 *   · 内容图片 (用户要看的东西, 如架构图、截图)
 *     -> 走普通的 <img>, 属于正文内容, 与主题无关.
 *
 *   · 皮肤素材 (主题自带的纹理/立绘/吉祥物)
 *     -> 不单独展示, 而是**嵌进控件的槽位里**由控件负责融合.
 *        用户换一张图, 系统仍然和谐; 这才是"主题皮肤".
 *
 * Webpack 5 在 ESM 下 import 静态资源返回 { default: url }, 直接当字符串用会得到
 * "[object Module]" (图片静默不显示), 因此统一经 assetUrl() 收敛.
 */
type AssetModule = string | { default?: string; src?: string };

export function assetUrl(mod: AssetModule | undefined | null): string | undefined {
    if (!mod) return undefined;
    if (typeof mod === 'string') return mod;
    return mod.default ?? mod.src ?? undefined;
}

export function assetMap(mods: Record<string, AssetModule>): Record<string, string> {
    const out: Record<string, string> = {};
    for (const [k, v] of Object.entries(mods)) {
        const u = assetUrl(v);
        if (u) out[k] = u;
    }
    return out;
}

/**
 * 一个可定位的皮肤槽.
 *
 * 为什么需要"位置": 皮肤素材是**融进版面**的, 而不是贴在中间.
 * 同一个立绘放在右下角、左下角或做成通栏底纹, 观感完全不同.
 * 因此槽位自带定位/尺寸/透明度/混合模式, 由主题编辑器可视化调好后导出.
 */
export interface SkinSlot {
    /** 图片地址 (png/webp/svg, 或 dataURL) */
    src?: string;
    /** 定位: 支持 px / % / 关键字 (auto/right/center) */
    x?: string;
    y?: string;
    /** 尺寸 */
    w?: string;
    h?: string;
    /** 0~1 */
    opacity?: number;
    /** 旋转角度 */
    rotate?: number;
    /** 与底色的融合方式: 方图用 luminosity/screen 能明显减少"贴纸感" */
    blend?: 'normal' | 'screen' | 'luminosity' | 'multiply' | 'overlay' | 'soft-light';
    radius?: string;
    /** 层级 */
    z?: number;
    /** 动效: 是否缓慢浮动 */
    float?: boolean;
    /** 适应方式 */
    fit?: 'cover' | 'contain';
}

/**
 * 皮肤素材槽位.
 * 全部可选 —— 缺省时控件必须优雅降级 (纯 CSS 装饰), 不允许报错.
 */
export interface DeckSkin {
    /** 吉祥物立绘槽 (封面右下角) */
    mascot?: string | SkinSlot;
    /** 角落探头小图槽 */
    mascotPeek?: string | SkinSlot;
    /** 背景纹样 (平铺/柔化后作底纹, 不直接展示原图) */
    pattern?: string | SkinSlot;
    /** 情绪贴图: 控件按语义取用 (如 Callout 的 tip/warn/danger) */
    emoji?: Record<string, string>;
    /** 品牌标记 (博客 logo), 用于右上角题头 */
    logo?: string | SkinSlot;
    /** 品牌名 */
    brand?: string;
}

/** 把 string | SkinSlot 归一成 SkinSlot */
export function normalizeSlot(v: string | SkinSlot | undefined): SkinSlot | undefined {
    if (!v) return undefined;
    return typeof v === 'string' ? { src: v } : v;
}

/** 槽位的 CSS 变量前缀 */
export function slotVars(prefix: string, slot?: SkinSlot): Record<string, string> {
    const s = normalizeSlot(slot);
    if (!s?.src) return {};
    const out: Record<string, string> = { [`--hxd-skin-${prefix}`]: `url(${s.src})` };
    if (s.x !== undefined) out[`--hxd-skin-${prefix}-x`] = s.x;
    if (s.y !== undefined) out[`--hxd-skin-${prefix}-y`] = s.y;
    if (s.w !== undefined) out[`--hxd-skin-${prefix}-w`] = s.w;
    if (s.h !== undefined) out[`--hxd-skin-${prefix}-h`] = s.h;
    if (s.opacity !== undefined) out[`--hxd-skin-${prefix}-opacity`] = String(s.opacity);
    if (s.rotate !== undefined) out[`--hxd-skin-${prefix}-rotate`] = `${s.rotate}deg`;
    if (s.radius !== undefined) out[`--hxd-skin-${prefix}-radius`] = s.radius;
    if (s.z !== undefined) out[`--hxd-skin-${prefix}-z`] = String(s.z);
    if (s.blend) out[`--hxd-skin-${prefix}-blend`] = s.blend;
    if (s.fit) out[`--hxd-skin-${prefix}-fit`] = s.fit;
    return out;
}

/** 语义化情绪名 —— 控件按语义要图, 不按文件名要图 */
export type EmojiKey = 'happy' | 'think' | 'warn' | 'error' | 'success' | 'sleep' | 'work' | 'rich';