/**
 * HXLoLi 演示页内嵌语法.
 *
 * ── 两类本地文件, 按扩展名区分 ──────────────────────────────────
 *   [标题 #ppt](x.html)       通用本地 HTML  → iframe 独立渲染 (不套主题)
 *   [标题 ##PPT##](x.tsx)     我们的演示页    → React 内联渲染 (融入主题)
 *
 * ── 指令只接受两个参数, 都可选 ──────────────────────────────────
 *   [##PPT 3 whale##](x.tsx)   默认展示第 3 页, 主题 whale
 *   [##PPT 3##](x.tsx)         第 3 页, 用默认主题
 *   [##PPT whale##](x.tsx)     第 1 页, 主题 whale
 *   [##PPT##](x.tsx)           第 1 页, 默认主题
 *   也接受具名写法: [##PPT 页码=3 主题=whale##](x.tsx)
 *
 * ── 刻意不做的 ────────────────────────────────────────────────
 *   · 不接受 "标题" 参数 —— 标题一律取自链接原文
 *   · 不接受 宽/高/切换/目录 —— 这些应由 deck 自身与站点统一定义,
 *     放进每篇笔记的链接里会让写法迅速失控
 */

export interface PptDirective {
    /** 0-based 页码; 未指定则由组件从第 1 页开始 */
    index?: number;
    /** 主题 id; 未指定则用默认主题 */
    theme?: string;
}

/** 演示页文件类型 */
export type PptTarget = 'deck' | 'html' | 'none';

/** 按扩展名判定这是哪一类本地文件 */
export function classifyHref(href: string): PptTarget {
    const path = (href || '').split(/[?#]/, 1)[0].toLowerCase();
    if (path.endsWith('.tsx')) return 'deck';
    if (path.endsWith('.html') || path.endsWith('.htm')) return 'html';
    return 'none';
}

/**
 * 解析指令串.
 * 只认两个东西: 一个纯数字 (页码) 和一个非数字词 (主题).
 * 多余的内容会被忽略, 而不是当作标题 —— 避免出现"半个标题进了指令"这种模糊状态.
 */
export function parsePptArgs(raw: string): PptDirective {
    const out: PptDirective = {};

    for (const part of raw.trim().split(/\s+/).filter(Boolean)) {
        const eq = part.indexOf('=');
        if (eq > 0) {
            const k = part.slice(0, eq).toLowerCase();
            const v = part.slice(eq + 1);
            if (k === '页码' || k === 'page' || k === 'p') {
                const n = Number(v);
                if (Number.isFinite(n) && n > 0) out.index = Math.floor(n) - 1;
            } else if (k === '主题' || k === 'theme' || k === 't') {
                if (v) out.theme = v;
            }
            // 其它键一律忽略
            continue;
        }
        // 位置参数: 纯数字 = 页码, 其它 = 主题 (都只取第一个)
        if (/^\d+$/.test(part)) {
            if (out.index === undefined) out.index = Math.max(0, Number(part) - 1);
        } else if (!out.theme) {
            out.theme = part;
        }
    }
    return out;
}

/**
 * 从链接文字里抽出指令, 并返回去掉指令后的**标题原文**.
 *
 * 标题就是链接里除指令之外剩下的文字 —— 与旧语法
 * `[标题 #ppt](x.html)` 的行为一致.
 */
export function parsePptText(text: string): { directive: PptDirective; title: string; matched: boolean } {
    const m = /##\s*PPT\b([^#]*)##/i.exec(text);
    if (m) {
        return {
            directive: parsePptArgs(m[1]),
            title: text.replace(m[0], ' ').replace(/\s+/g, ' ').trim(),
            matched: true,
        };
    }
    // 旧语法: #ppt + 可选 ##w100%##
    if (/(^|\s)#ppt(?=\s|##|$)/i.test(text)) {
        return {
            directive: {},
            title: text
                .replace(/##[wW]\d+%?##/g, '')
                .replace(/(^|\s)#ppt(?=\s|##|$)/gi, ' ')
                .replace(/\s+/g, ' ')
                .trim(),
            matched: true,
        };
    }
    return { directive: {}, title: text.trim(), matched: false };
}

/** 旧语法里的宽度标记 (##w100%##) – 仅通用 HTML 用得上 */
export function legacyWidth(text: string): string | undefined {
    const m = /##[wW](\d+%?)##/.exec(text);
    if (!m) return undefined;
    return m[1].includes('%') ? m[1] : `${m[1]}px`;
}
