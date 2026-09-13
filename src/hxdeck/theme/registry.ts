import type { DeckTheme } from './types';
import { whaleTheme } from './whale';
import { hxloliTheme } from './hxloli';
import { parseTheme } from './serialize';

/**
 * 站点 baseUrl.
 *
 * 坑: 博客部署在子路径 (/HXLoLi/), 直接 fetch('/themes/index.json') 会 404.
 * 这里从 Docusaurus 生成的配置读取, 保证任何部署前缀下都能取到.
 */
function sitePrefix(): string {
    if (typeof document === 'undefined') return '';
    // 优先用显式注入 (见下), 其次从 <base href> 推导, 最后退回当前路径
    const injected = (window as unknown as { __HXD_BASE_URL__?: string }).__HXD_BASE_URL__;
    if (injected) return injected.endsWith('/') ? injected.slice(0, -1) : injected;

    // require() 在浏览器不存在 —— 早期版本因此静默回退到 '/', 主题清单永远加载不到.
    const baseEl = document.querySelector('base[href]');
    if (baseEl) {
        const href = baseEl.getAttribute('href') || '';
        if (href) return href.endsWith('/') ? href.slice(0, -1) : href;
    }
    // 兜底: 用当前路径里已知的站点前缀 (去掉最后一段)
    const p = window.location.pathname;
    const i = p.indexOf('/', 1);
    return i > 0 ? p.slice(0, i) : '';
}

/**
 * 主题注册表 —— 让"主题"成为可运行时加载/切换的资源, 而不是编译期常量.
 *
 * 三种来源:
 *   1. 内置: 随包发布的主题 (whale / hxloli)
 *   2. 静态: 用户放到 `static/themes/*.yaml|json` 的主题, 由文档里的语法按名引用
 *   3. 临时: 用户在前端导入的文件 (仅本次会话有效)
 *
 * 关键: 三者在消费侧没有区别 —— 都是 DeckTheme 对象.
 */

/**
 * 默认主题.
 *
 * 演示页没显式指定主题时用它. 单独抽成常量而不是散落在各处,
 * 这样"默认主题是哪个"只有一个定义点.
 */
export const DEFAULT_THEME_ID = 'whale';

const builtin: Record<string, DeckTheme> = {
    whale: whaleTheme,
    hxloli: hxloliTheme,
    // 常用别名, 免得写错
    default: hxloliTheme,
    'whale-chan': whaleTheme,
};

const runtime = new Map<string, DeckTheme>();
const pending = new Map<string, Promise<DeckTheme | null>>();

export interface ThemeInfo {
    id: string;
    name: string;
    builtin: boolean;
    /** 磁盘上的文件名 (静态主题才有) */
    file?: string;
}

/** 内置 + 已加载 + 磁盘清单里的全部主题 */
export function listThemes(): ThemeInfo[] {
    const out: ThemeInfo[] = Object.values(builtin).map((t) => ({ id: t.id, name: t.name, builtin: true }));
    for (const t of runtime.values()) out.push({ id: t.id, name: t.name, builtin: false });
    for (const t of manifest) {
        if (!out.some((x) => x.id === t.id)) out.push({ id: t.id, name: t.name, builtin: false, file: t.file });
    }
    // 去重 (别名可能指向同一主题)
    const seen = new Set<string>();
    return out.filter((t) => (seen.has(t.id) ? false : (seen.add(t.id), true)));
}

/** 磁盘主题清单: 由 scripts/scan-themes.mjs 生成 */
let manifest: { id: string; name: string; file: string }[] = [];
let manifestLoaded = false;

/**
 * 加载主题清单 (static/themes/index.json).
 * 静态站无法列目录, 所以必须靠这份索引 —— 它让"用户放进去的主题"能被自动发现.
 */
/** 依次尝试若干候选路径, 返回第一个 2xx 的文本 */
async function fetchFirst(paths: string[]): Promise<string | null> {
    for (const p of paths) {
        try {
            const res = await fetch(p);
            if (res.ok) return await res.text();
        } catch {
            // 换下一个候选
        }
    }
    return null;
}

/**
 * 主题资源的候选路径.
 *
 * 不做单点路径推导 —— 直接给出全部可能位置, 谁先命中用谁.
 * 这样开发服务器 / 生产构建 / 任意 baseUrl 前缀都能取到, 不会因为推导错误而静默失联.
 */
export function themePaths(file: string, baseUrl?: string): string[] {
    const prefix = baseUrl !== undefined
        ? (baseUrl.endsWith('/') ? baseUrl.slice(0, -1) : baseUrl)
        : sitePrefix();
    return [...new Set([`${prefix}/themes/${file}`, `/themes/${file}`])];
}

export async function loadManifest(baseUrl?: string): Promise<ThemeInfo[]> {
    if (manifestLoaded) return listThemes();
    const text = await fetchFirst(themePaths('index.json', baseUrl));
    try {
        manifest = text
            ? ((JSON.parse(text) as { themes?: { id: string; name: string; file: string }[] }).themes ?? [])
            : [];
    } catch {
        manifest = [];
    }
    manifestLoaded = true;
    return listThemes();
}

/** 按文件名直接加载 (清单里给了 file, 比猜 <id>.yaml 更可靠) */
export async function loadThemeFile(file: string, baseUrl?: string): Promise<DeckTheme | null> {
    const cached = [...runtime.values()].find((t) => t.id === file);
    if (cached) return cached;
    const text = await fetchFirst(themePaths(file, baseUrl));
    if (!text) return null;
    try {
        const theme = parseTheme(text);
        runtime.set(theme.id, theme);
        return theme;
    } catch {
        return null;
    }
}

/** 同步取: 只在内置或已加载时命中 */
export function getTheme(id: string): DeckTheme | undefined {
    return runtime.get(id) ?? builtin[id];
}

/**
 * 按名加载主题: 先查内置, 再去 `/themes/<id>.yaml|json` 找.
 * 找到后缓存, 重复调用只发一次请求.
 */
export function loadTheme(id: string, baseUrl?: string): Promise<DeckTheme | null> {
    const hit = getTheme(id);
    if (hit) return Promise.resolve(hit);
    const flying = pending.get(id);
    if (flying) return flying;

    const task = (async () => {
        // 先确保清单就绪: 用户给主题文件起任意名字时, 只有清单知道它的文件名
        await loadManifest(baseUrl);
        // 清单里若有该 id 的文件名, 优先用它 (用户可能把文件命名为任意名字)
        const fromManifest = manifest.find((m) => m.id === id);
        const names = fromManifest ? [fromManifest.file] : [`${id}.yaml`, `${id}.yml`, `${id}.json`];
        const candidates = names.flatMap((n) => themePaths(n, baseUrl));
        for (const url of candidates) {
            try {
                const res = await fetch(url);
                if (!res.ok) continue;
                const theme = parseTheme(await res.text());
                runtime.set(id, theme);
                // 同时用其自身 id 注册, 便于后续按 id 引用
                runtime.set(theme.id, theme);
                return theme;
            } catch {
                // 换下一个候选
            }
        }
        return null;
    })();

    pending.set(id, task);
    task.finally(() => pending.delete(id));
    return task;
}

/** 前端导入的主题: 仅本次会话有效 */
export function registerTheme(theme: DeckTheme): void {
    runtime.set(theme.id, theme);
}

/** 取默认主题对象 (必须放在 builtin 之后, 因为要读它) */
export function defaultTheme(): DeckTheme {
    return builtin[DEFAULT_THEME_ID] ?? whaleTheme;
}