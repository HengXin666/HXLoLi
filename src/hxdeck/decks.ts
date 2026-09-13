import type React from 'react';

/**
 * 演示页内容注册表.
 *
 * 新语法 [##PPT 页码 主题##](名) 只声明"怎么展示", 不承载内容.
 * 内容按名字从注册表取 —— 笔记正文保持干净, 演示页本身仍是可复用组件.
 *
 * 注册方式: 在任意 .tsx 里调用 registerDeck() (见 decks-builtin.tsx).
 * 注意: 本文件**不能含 JSX**, 扩展名是 .ts; 含 JSX 的注册请放 .tsx.
 */

export interface DeckEntry {
    title: string;
    /** 返回演示页的 <Slide> 列表 */
    render: () => React.ReactNode;
}

const registry = new Map<string, DeckEntry>();

export function registerDeck(name: string, entry: DeckEntry): void {
    registry.set(name, entry);
}

export function getDeck(name: string): DeckEntry | undefined {
    return registry.get(name);
}

export function listDecks(): string[] {
    return [...registry.keys()];
}
