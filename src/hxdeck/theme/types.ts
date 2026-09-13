/**
 * hxdeck 主题契约 (端口层)
 *
 * 设计原则: 控件只认 token, 不认具体主题.
 * 换主题 = 换一份 DeckTheme, 控件代码一行不改.
 *
 * 命名空间统一用 --hxd-* , 与博客自身的 --ifm-* 完全隔离.
 */

/** 语义化色板 —— 控件只允许引用这些语义名, 不允许出现字面色值 */
export interface DeckColors {
    /** 页面底色 (通常是渐变起点) */
    bg: string;
    /** 页面底色渐变终点; 无渐变时与 bg 相同 */
    bgAlt: string;
    /** 卡片/面板表面色 */
    surface: string;
    /** 表面色之上的次级表面 (表格斑马纹、嵌套面板) */
    surfaceAlt: string;
    /** 主文字 */
    text: string;
    /** 次要文字 (说明、脚注、坐标轴) */
    textMuted: string;
    /** 品牌主色 (强调、进度、选中态) */
    primary: string;
    /** 主色的柔和变体 (大面积填充) */
    primarySoft: string;
    /** 辅助强调色 (与 primary 拉开色相的第二个焦点色) */
    accent: string;
    /** 语义色 */
    success: string;
    warn: string;
    danger: string;
    /** 描边 */
    border: string;
}

/** 形状与间距 */
export interface DeckShape {
    radius: string;
    radiusSm: string;
    radiusLg: string;
    borderWidth: string;
    /** 卡片阴影 */
    shadow: string;
    /** 发光效果 (二次元主题用得上) */
    glow: string;
}

/**
 * 尺度梯度 —— 让"换主题"也能换排版气质, 而不只是换色.
 * 全部用 clamp() 做流体缩放, 保证投屏与笔电都可读.
 */
export interface DeckScale {
    /** 大标题 */
    display: string;
    h1: string;
    h2: string;
    body: string;
    sm: string;
    xs: string;
    /** 数字/指标 */
    num: string;
    /** 间距梯度 */
    sp2: string;
    sp3: string;
    sp4: string;
    sp5: string;
    sp7: string;
    /** 行高与字距 */
    lhTight: string;
    lhNormal: string;
    lsWide: string;
    /** CJK 字重描边量: 小字号下中文笔画密度高, 需描边补粗才不虚 */
    strokeW7: string;
    /** 更细的字阶 (标签/图注) */
    label: string;
    caption: string;
    /** 超大数字 (关键指标) */
    hero: string;
}

/** 间距梯度 (8pt 网格) */

/**
 * 高度层级 —— Material 3 的 elevation 思路.
 * 用"表面亮度 + 阴影"共同表达层级, 而不是只靠阴影 (深色主题下纯阴影几乎不可见).
 */
export interface DeckElevation {
    /** 0: 贴地 (页面底) */
    e0: string;
    /** 1: 卡片默认 */
    e1: string;
    /** 2: 悬浮/悬停 */
    e2: string;
    /** 3: 弹层/浮窗 */
    e3: string;
    /** 4: 模态 */
    e4: string;
    /** 对应的表面提亮量 (深色主题下表达层级的更可靠手段) */
    tint1: string;
    tint2: string;
    tint3: string;
}

/**
 * 状态层 —— hover/press 的统一反馈.
 * Apple 用"材质变化", Material 用"叠加半透明色", 这里取后者, 因为它与主题色自然联动.
 */
export interface DeckState {
    hover: string;
    press: string;
    focusRing: string;
}

/**
 * 关于字号的重要约束:
 *   舞台是**固定 1600x900 画布**, 再整体 scale 去适配容器.
 *   因此这里必须用**绝对 px 值** (相对画布), 绝不能用 vw/vh ——
 *   否则 vw 与 scale 会叠加两次缩放, 投屏时字会小到看不清.
 */
/** 字体栈 */
export interface DeckFonts {
    heading: string;
    body: string;
    mono: string;
    /** 数字/数据专用 (可选, 缺省回退 mono) */
    numeric?: string;
}

/** 动效曲线与时长 —— 用户可整体调"活泼度" */
export interface DeckMotion {
    /** 进场缓动 */
    ease: string;
    /** 强调/回弹缓动 */
    easeBounce: string;
    fast: string;
    base: string;
    slow: string;
    /** 每页元素级联进场的间隔 */
    stagger: string;
    /** 整屏翻页时长 (需与 Deck.tsx 的 PAGE_DURATION 对齐) */
    page: string;
    /** 整屏翻页缓动 */
    easePage: string;
}

export type { DeckSkin as DeckAssets } from '../assets';

/**
 * 署名 —— 合规必需.
 * CC BY-NC-SA 之类要求署名的素材, 必须通过这里声明, 由渲染器统一在页脚输出.
 */
export interface DeckCredit {
    text: string;
    url?: string;
}

import { slotVars, normalizeSlot, type DeckSkin } from '../assets';

/** 一份完整主题 = token (端口) + 皮肤素材 (实现) + 署名 (合规) */
export interface DeckTheme {
    /** 唯一 id, 用于选择与持久化 */
    id: string;
    /** 展示名 */
    name: string;
    description?: string;
    colors: DeckColors;
    shape: DeckShape;
    fonts: DeckFonts;
    motion: DeckMotion;
    scale: DeckScale;
    elevation: DeckElevation;
    state: DeckState;
    /** 皮肤素材槽 (见 assets.ts 的 DeckSkin) */
    assets?: DeckSkin;
    credits?: DeckCredit[];
    /** 主题独有的附加 CSS (字体导入、复杂装饰), 注入时加 hxd-theme-<id> 作用域 */
    css?: string;
}

/** 把主题编译成 CSS 变量表, 供注入 style 元素 */
export function themeToVars(theme: DeckTheme): Record<string, string> {
    const vars: Record<string, string> = {};
    for (const [k, v] of Object.entries(theme.colors)) vars[`--hxd-color-${kebab(k)}`] = v;
    for (const [k, v] of Object.entries(theme.shape)) vars[`--hxd-shape-${kebab(k)}`] = v;
    for (const [k, v] of Object.entries(theme.fonts)) vars[`--hxd-font-${kebab(k)}`] = v;
    for (const [k, v] of Object.entries(theme.motion)) vars[`--hxd-motion-${kebab(k)}`] = v;
    for (const [k, v] of Object.entries(theme.scale)) vars[`--hxd-scale-${kebab(k)}`] = v;
    for (const [k, v] of Object.entries(theme.elevation)) vars[`--hxd-elev-${kebab(k)}`] = v;
    for (const [k, v] of Object.entries(theme.state)) vars[`--hxd-state-${kebab(k)}`] = v;
    // 皮肤槽 -> CSS 变量. 位置/透明度/混合模式等一并导出, 控件按变量渲染即可.
    Object.assign(vars, slotVars('mascot', normalizeSlot(theme.assets?.mascot)));
    Object.assign(vars, slotVars('peek', normalizeSlot(theme.assets?.mascotPeek)));
    Object.assign(vars, slotVars('pattern', normalizeSlot(theme.assets?.pattern)));
    Object.assign(vars, slotVars('logo', normalizeSlot(theme.assets?.logo)));
    return vars;
}

/** camelCase -> kebab-case */
export function kebab(s: string): string {
    return s.replace(/([a-z0-9])([A-Z])/g, '$1-$2').toLowerCase();
}

/**
 * 同心圆角 (Apple 的 concentric corners).
 * 嵌套容器若用同一个圆角值, 视觉上内角会显得"比外角尖".
 * 规则: 内圆角 = 外圆角 - 内外间距.
 */
export function concentric(outerRadiusPx: number, paddingPx: number): string {
    return `${Math.max(0, outerRadiusPx - paddingPx)}px`;
}