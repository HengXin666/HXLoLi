#!/usr/bin/env node
/**
 * 把 archify 产物 (.html) 转成可被 React 内联使用的 TS 模块.
 *
 * 为什么要转:
 *   archify 出的是**整页 HTML** (含工具栏/主题切换/186KB 样式), 直接内联会把
 *   演示页污染成一个'外来窗口'. 我们只要两样东西:
 *     1. <svg>...</svg>          —— 图形本体
 *     2. 语义 class 那段 CSS      —— .c-frontend / .t-muted 等, 配色全走变量
 *   其余 (工具栏、主题变量默认值) 由主题层接管, 这样图才真正跟随主题.
 *
 * 用法: node scripts/extract-diagram.mjs <in.html> <out.ts>
 */
import fs from 'node:fs';
import path from 'node:path';

const [, , inPath, outPath] = process.argv;
if (!inPath || !outPath) {
    console.error('用法: node scripts/extract-diagram.mjs <in.html> <out.ts>');
    process.exit(1);
}

const html = fs.readFileSync(inPath, 'utf8');

const svgMatch = /<svg[\s\S]*?<\/svg>/i.exec(html);
if (!svgMatch) {
    console.error('未找到 <svg>, 请确认这是 archify 产物');
    process.exit(1);
}
const svg = svgMatch[0];

const styles = [...html.matchAll(/<style[^>]*>([\s\S]*?)<\/style>/gi)].map((m) => m[1]);
if (!styles.length) {
    console.error('未找到 <style>, 无法取出语义 class');
    process.exit(1);
}

// 只保留 'SVG SEMANTIC CLASSES' 之后那段: 前面的主题变量与工具栏由主题层覆盖
const full = styles[0];
const marker = 'SVG SEMANTIC CLASSES';
const at = full.indexOf(marker);
const semantic = at >= 0 ? full.slice(at) : full;
// 去掉注释块头部残留
const cleaned = semantic.replace(/^[^{]*?\*\//, '').trim();

const title = (/<title[^>]*>([\s\S]*?)<\/title>/i.exec(svg) || [, ''])[1].trim();
const viewBox = (/(viewBox=")([^"]+)/.exec(svg) || [, '', ''])[2];

const out = [
    '/* 由 scripts/extract-diagram.mjs 自动生成, 请勿手改.',
    ' * 源: ' + path.basename(inPath),
    ' * 内容: 图形 SVG + archify 语义 class (配色走 --hxd 变量, 由 Diagram 控件注入)',
    ' */',
    '',
    'export const title = ' + JSON.stringify(title) + ';',
    'export const viewBox = ' + JSON.stringify(viewBox) + ';',
    '',
    'export const css = ' + JSON.stringify(cleaned) + ';',
    '',
    'export const svg = ' + JSON.stringify(svg) + ';',
    '',
    'export default { title, viewBox, css, svg };',
    '',
].join('\n');

fs.mkdirSync(path.dirname(outPath), { recursive: true });
fs.writeFileSync(outPath, out);
console.log('已生成', outPath);
console.log('  viewBox:', viewBox, '| SVG', svg.length, 'B | CSS', cleaned.length, 'B');
