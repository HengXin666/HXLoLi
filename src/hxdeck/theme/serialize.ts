import yaml from 'js-yaml';
import type { DeckTheme } from './types';

/**
 * 主题的序列化 / 反序列化.
 *
 * 目的: 让主题成为**可导出的文件**, 而不是写死在代码里.
 * 用户在前端编辑 (配色 / 图片 / 位置) -> 导出 JSON 或 YAML -> 放进项目静态目录
 * -> 运行时加载 -> 继续修改. 整个过程不需要改代码.
 *
 * 格式选择:
 *   JSON  —— 机器友好, 适合被代码 import, 无歧义
 *   YAML  —— 人手写友好, 适合配置文件 (注释 / 多行字符串)
 * 两者互转, 内容完全等价 (loadTheme 两种都能吃).
 */

export type ThemeFormat = 'json' | 'yaml';

/** 主题 -> 字符串 */
export function serializeTheme(theme: DeckTheme, format: ThemeFormat = 'json'): string {
    // 去掉运行期字段 (函数等) —— 主题本身已是纯数据, 这里只保证键序稳定
    const plain = JSON.parse(JSON.stringify(theme)) as DeckTheme;
    return format === 'yaml'
        ? yaml.dump(plain, { indent: 2, lineWidth: 120, noRefs: true })
        : JSON.stringify(plain, null, 2);
}

/** 字符串 -> 主题 (自动识别 JSON / YAML) */
export function parseTheme(text: string): DeckTheme {
    const trimmed = text.trim();
    // JSON 能被 YAML 解析器吃下, 所以只用 yaml.load 即可覆盖两种;
    // 但 JSON 报错信息更准, 故优先试 JSON.
    let data: unknown;
    if (trimmed.startsWith('{')) {
        try {
            data = JSON.parse(trimmed);
        } catch {
            data = yaml.load(trimmed);
        }
    } else {
        data = yaml.load(trimmed);
    }
    return validateTheme(data);
}

/** 最小结构校验: 缺关键字段就报错, 而不是等到渲染时莫名其妙 */
export function validateTheme(data: unknown): DeckTheme {
    if (!data || typeof data !== 'object') {
        throw new Error('主题必须是对象');
    }
    const t = data as Partial<DeckTheme>;
    const need: (keyof DeckTheme)[] = ['id', 'name', 'colors', 'shape', 'fonts', 'motion', 'scale', 'elevation', 'state'];
    const missing = need.filter((k) => t[k] === undefined);
    if (missing.length) {
        throw new Error(`主题缺少必填字段: ${missing.join(', ')}`);
    }
    return t as DeckTheme;
}

/** 两份主题合并: 用于"在基础主题上只改几个值" */
export function mergeTheme(base: DeckTheme, patch: Partial<DeckTheme>): DeckTheme {
    const deep = <T>(a: T, b: unknown): T => {
        if (!b || typeof b !== 'object') return (b === undefined ? a : (b as T));
        const out = { ...(a as Record<string, unknown>) };
        for (const [k, v] of Object.entries(b as Record<string, unknown>)) {
            const cur = (a as Record<string, unknown>)?.[k];
            out[k] = cur && typeof cur === 'object' && !Array.isArray(cur) ? deep(cur, v) : v;
        }
        return out as T;
    };
    return deep(base, patch) as DeckTheme;
}

/** 下载为文件 (浏览器端) */
export function downloadTheme(theme: DeckTheme, format: ThemeFormat = 'json'): void {
    const text = serializeTheme(theme, format);
    const blob = new Blob([text], { type: format === 'yaml' ? 'text/yaml;charset=utf-8' : 'application/json;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${theme.id}.theme.${format === 'yaml' ? 'yaml' : 'json'}`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
}

/** 从文件读取 */
export async function readThemeFile(file: File): Promise<DeckTheme> {
    const text = await file.text();
    return parseTheme(text);
}

/** 生成一份空白主题骨架 (供"从零做主题") */
export function blankTheme(id = 'custom', name = '自定义主题', base?: DeckTheme): DeckTheme {
    if (base) {
        return { ...JSON.parse(JSON.stringify(base)) as DeckTheme, id, name };
    }
    throw new Error('blankTheme 需要 base 主题作为起点');
}
