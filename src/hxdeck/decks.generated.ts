/* 由 scripts/generate-deck-registry.mjs 自动生成, 请勿手改.
 *
 * 每个条目: ai-docs 下的相对路径 -> 演示页组件
 * 链接里写 [标题 ##PPT##](相对当前笔记的路径.tsx) 即可挂载.
 *
 * 运行: npm run decks   (build 前会自动执行)
 */
import type React from 'react';

export type DeckComponent = React.ComponentType<{ page?: number }>;

import deck0, { slides as slides0 } from '../../ai-docs/002-AI/008-AI逆向/001-CF过盾工程-从零实现Turnstile绕过/cf-gateway-deck';

export type DeckSlides = () => React.ReactNode;

export const deckModules: Record<string, { view: DeckComponent; slides?: DeckSlides }> = {
    '002-AI/008-AI逆向/001-CF过盾工程-从零实现Turnstile绕过/cf-gateway-deck.tsx': { view: deck0, slides: slides0 },
};

/** 规范化为"相对 ai-docs 的路径" */
export function normalizeDeckKey(p: string): string {
    return p.replace(/^\/?/, '').replace(/\/\/?/g, '/');
}

/** 处理 ../ 与 ./ */
export function resolveDotSegments(p: string): string {
    const out: string[] = [];
    for (const seg of p.split('/')) {
        if (!seg || seg === '.') continue;
        if (seg === '..') out.pop();
        else out.push(seg);
    }
    return out.join('/');
}

/**
 * 在注册表里查找演示页.
 * 依次尝试: 相对笔记目录解析 -> 直接当 ai-docs 相对路径 -> 只给文件名时取唯一匹配.
 */
export function findDeckKey(href: string, noteDir?: string): string | undefined {
    const raw = decodeURI((href || '').split(/[?#]/, 1)[0] || '');
    if (!raw) return undefined;
    const keys = Object.keys(deckModules);
    if (noteDir) {
        const joined = normalizeDeckKey(noteDir + '/' + raw);
        const resolved = resolveDotSegments(joined);
        if (deckModules[resolved]) return resolved;
    }
    const direct = resolveDotSegments(normalizeDeckKey(raw));
    if (deckModules[direct]) return direct;
    const base = raw.split('/').pop() || raw;
    const byName = keys.filter((k) => k === base || k.endsWith('/' + base));
    if (byName.length === 1) return byName[0];
    return undefined;
}
