#!/usr/bin/env node
/**
 * 扫描 ai-docs 下的演示页, 生成静态注册表.
 *
 * 演示页判定: 文件扩展名 .tsx —— 与"本地 .html 侧车"同一套思路, 按相对路径 + 扩展名识别.
 *
 * 为什么用**静态 import** 而不是动态 import():
 *   项目 tsconfig 是 moduleResolution: nodenext, 动态 import 的相对路径必须带扩展名,
 *   而 .tsx 扩展名又不被允许 (allowImportingTsExtensions 未开) —— 两头堵.
 *   静态 import 交给 webpack 解析, 没有这个限制.
 *
 * 用法: node scripts/generate-deck-registry.mjs
 */
import fs from 'node:fs';
import path from 'node:path';

const root = process.cwd();
const docsDir = path.join(root, 'ai-docs');
const outFile = path.join(root, 'src', 'hxdeck', 'decks.generated.ts');

if (!fs.existsSync(docsDir)) {
    console.log('ai-docs 不存在, 跳过');
    process.exit(0);
}

function walk(dir, acc = []) {
    for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
        if (e.name.startsWith('.')) continue;
        const full = path.join(dir, e.name);
        if (e.isDirectory()) walk(full, acc);
        else if (e.isFile() && e.name.endsWith('.tsx')) acc.push(full);
    }
    return acc;
}

const files = walk(docsDir);
fs.mkdirSync(path.dirname(outFile), { recursive: true });

const entries = files.map((f) => {
    const key = path.relative(docsDir, f).split(path.sep).join('/');
    const rel = path.relative(path.join(root, 'src', 'hxdeck'), f).split(path.sep).join('/');
    // import 路径不带扩展名: webpack 能解析, 而 TS 的 nodenext 不允许 .tsx 后缀
    const noExt = rel.replace(/\.tsx$/, '');
    return { key, importPath: noExt.startsWith('.') ? noExt : './' + noExt };
});

const L = [];
L.push('/* 由 scripts/generate-deck-registry.mjs 自动生成, 请勿手改.');
L.push(' *');
L.push(' * 每个条目: ai-docs 下的相对路径 -> 演示页组件');
L.push(' * 链接里写 [标题 ##PPT##](相对当前笔记的路径.tsx) 即可挂载.');
L.push(' *');
L.push(' * 运行: npm run decks   (build 前会自动执行)');
L.push(' */');
L.push("import type React from 'react';");
L.push('');
L.push('export type DeckComponent = React.ComponentType<{ page?: number }>;');
L.push('');
// 同时引入默认组件与 slides() 工厂
entries.forEach((e, i) => L.push("import deck" + i + ", { slides as slides" + i + " } from '" + e.importPath + "';"));
L.push('');
L.push('export type DeckSlides = () => React.ReactNode;');
L.push('');
L.push('export const deckModules: Record<string, { view: DeckComponent; slides?: DeckSlides }> = {');
entries.forEach((e, i) => L.push("    '" + e.key + "': { view: deck" + i + ", slides: slides" + i + ' },'));
L.push('};');
L.push('');
L.push('/** 规范化为"相对 ai-docs 的路径" */');
L.push('export function normalizeDeckKey(p: string): string {');
L.push("    return p.replace(/^\\/?/, '').replace(/\\/\\/?/g, '/');");
L.push('}');
L.push('');
L.push('/** 处理 ../ 与 ./ */');
L.push('export function resolveDotSegments(p: string): string {');
L.push('    const out: string[] = [];');
L.push("    for (const seg of p.split('/')) {");
L.push("        if (!seg || seg === '.') continue;");
L.push("        if (seg === '..') out.pop();");
L.push('        else out.push(seg);');
L.push('    }');
L.push("    return out.join('/');");
L.push('}');
L.push('');
L.push('/**');
L.push(' * 在注册表里查找演示页.');
L.push(' * 依次尝试: 相对笔记目录解析 -> 直接当 ai-docs 相对路径 -> 只给文件名时取唯一匹配.');
L.push(' */');
L.push('export function findDeckKey(href: string, noteDir?: string): string | undefined {');
L.push("    const raw = decodeURI((href || '').split(/[?#]/, 1)[0] || '');");
L.push('    if (!raw) return undefined;');
L.push('    const keys = Object.keys(deckModules);');
L.push('    if (noteDir) {');
L.push("        const joined = normalizeDeckKey(noteDir + '/' + raw);");
L.push('        const resolved = resolveDotSegments(joined);');
L.push('        if (deckModules[resolved]) return resolved;');
L.push('    }');
L.push('    const direct = resolveDotSegments(normalizeDeckKey(raw));');
L.push('    if (deckModules[direct]) return direct;');
L.push("    const base = raw.split('/').pop() || raw;");
L.push("    const byName = keys.filter((k) => k === base || k.endsWith('/' + base));");
L.push('    if (byName.length === 1) return byName[0];');
L.push('    return undefined;');
L.push('}');
L.push('');

fs.writeFileSync(outFile, L.join('\n'));
console.log('已生成 src/hxdeck/decks.generated.ts (' + entries.length + ' 个演示页)');
for (const e of entries) console.log('  -', e.key);