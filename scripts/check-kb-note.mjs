#!/usr/bin/env node
/**
 * 知识库笔记检查 (用了 KBNote 组件的 ai-docs 笔记).
 *
 * 用法 (在 HXLoLi 根目录):
 *   node scripts/check-kb-note.mjs <笔记目录或 index.md> [...]
 *
 * 不启动博客. 做法: 用博客自带的 @mdx-js/mdx 编译正文, 再用"桩组件"执行一遍,
 * 桩组件只收集 props 做校验. 所以 MDX 写坏了、用了没注册的组件、props 缺字段,
 * 都在这里报出来, 而不是等到页面白屏.
 *
 * 规则 (能被脚本拦的就不写进提示词):
 *   1. 正文有组件就必须 frontmatter `mdx: { format: mdx }` (Docusaurus 只认 mdx.format, 顶层 format 会被忽略)
 *   2. 只能用 KBNote 的组件; 不允许小写 HTML 标签、style、className
 *   3. <Lead> 恰好一个, 且在第一个 ## 之前
 *   4. 必须有 `## 什么时候不成立` 一节
 *   5. 组件必填字段: Stats.source, Figure.alt/caption, Apply.path (且路径存在) ...
 *   6. 不允许 Markdown 表格
 *   7. 单段正文不超过 100 字
 */
import fs from 'node:fs';
import path from 'node:path';
import * as runtime from 'react/jsx-runtime';
import { evaluate } from '@mdx-js/mdx';
import remarkGfm from 'remark-gfm';

const ALLOWED = ['Lead', 'Stats', 'Layers', 'Fork', 'Flow', 'Compare', 'Evidence', 'Apply', 'Boundary', 'Figure', 'Chart', 'Red', 'Gold'];
const MAX_MARK = 30; // <Red>/<Gold> 里最多几个字
const TONES = new Set(['good', 'ok', 'bad', 'last']);
const MAX_PARA = 100;

// 笔记里图片写 require('./x.png').default; 检查时不打包, 给个桩, 顺便核对文件存在
let currentDir = '.';
const missingAssets = [];
globalThis.require = (p) => {
    if (!fs.existsSync(path.join(currentDir, p))) missingAssets.push(p);
    return { default: p };
};

const args = process.argv.slice(2);
const rootIdx = args.indexOf('--root');
const wsRoot = path.resolve(rootIdx >= 0 ? args.splice(rootIdx, 2)[1] : '..');
if (!args.length) {
    console.error('用法: node scripts/check-kb-note.mjs <笔记目录或 index.md> [...] [--root <HXLoLis 根>]');
    process.exit(2);
}

function splitFrontmatter (src) {
    const m = /^---\n([\s\S]*?)\n---\n/.exec(src);
    if (!m) return { fm: '', body: src, offset: 0 };
    return { fm: m[1], body: src.slice(m[0].length), offset: m[0].split('\n').length - 1 };
}

function textLen (s) {
    return [...s.replace(/<\/?(Red|Gold)>/g, '').replace(/`[^`]*`/g, 'x').replace(/\[([^\]]*)\]\([^)]*\)/g, '$1').replace(/\s+/g, '')].length;
}

/** 去掉代码块与 JSX 块, 剩下的才是"露在外面的正文" */
function proseLines (body) {
    const lines = body.split('\n');
    const out = [];
    let inFence = false;
    let jsxDepth = 0;
    lines.forEach((line, i) => {
        if (/^```/.test(line)) { inFence = !inFence; return; }
        if (inFence) return;
        if (jsxDepth === 0 && /^<[A-Z]/.test(line)) {
            if (!/\/>\s*$/.test(line) && !/^<([A-Z]\w*)[^>]*>.*<\/\1>\s*$/.test(line)) jsxDepth = 1;
            return;
        }
        if (jsxDepth > 0) {
            if (/^(\/>|<\/[A-Z]\w*>|\]\}\s*\/>|\}\}\s*\/>)\s*$/.test(line.trim()) || /\/>\s*$/.test(line)) jsxDepth = 0;
            return;
        }
        out.push({ line, no: i + 1 });
    });
    return out;
}

function req (errs, comp, obj, keys, where = '') {
    for (const k of keys) {
        const v = obj?.[k];
        if (v === undefined || v === null || v === '') errs.push(`<${comp}>${where} 缺少 ${k}`);
    }
}

function checkTone (errs, comp, x, where) {
    if (x?.tone !== undefined && !TONES.has(x.tone)) errs.push(`<${comp}>${where} tone 只能是 good / ok / bad / last, 实际是 ${x.tone}`);
}

function makeStubs (errs, seen) {
    const stub = (name, check) => (props) => {
        seen.push(name);
        if ('style' in props || 'className' in props) errs.push(`<${name}> 不接受 style / className`);
        try { check(props); } catch (e) { errs.push(`<${name}> props 结构不对: ${e.message}`); }
        return null;
    };
    const list = (comp, arr, keys, extra) => {
        if (!Array.isArray(arr) || !arr.length) { errs.push(`<${comp}> 需要非空数组`); return; }
        arr.forEach((x, i) => { req(errs, comp, x, keys, ` 第 ${i + 1} 项`); checkTone(errs, comp, x, ` 第 ${i + 1} 项`); extra?.(x, i); });
    };
    return {
        Lead: stub('Lead', (p) => req(errs, 'Lead', p, ['why', 'claim', 'use'])),
        Stats: stub('Stats', (p) => list('Stats', p.items, ['value', 'label', 'source'])),
        Layers: stub('Layers', (p) => list('Layers', p.items, ['name', 'weight'], (x, i) => {
            if (typeof x.weight !== 'number' || x.weight <= 0) errs.push(`<Layers> 第 ${i + 1} 项 weight 要是正数`);
        })),
        Fork: stub('Fork', (p) => { req(errs, 'Fork', p, ['from']); list('Fork', p.options, ['name']); }),
        Flow: stub('Flow', (p) => list('Flow', p.steps, ['name'])),
        Compare: stub('Compare', (p) => {
            for (const k of ['left', 'right']) {
                req(errs, 'Compare', p[k], ['title', 'items'], ` ${k}`);
                checkTone(errs, 'Compare', p[k], ` ${k}`);
            }
        }),
        Evidence: stub('Evidence', (p) => req(errs, 'Evidence', p, ['source', 'children'])),
        Apply: stub('Apply', (p) => list('Apply', p.items, ['path', 'now', 'next'], (x, i) => {
            if (typeof x.path === 'string' && !fs.existsSync(path.join(wsRoot, x.path))) {
                errs.push(`<Apply> 第 ${i + 1} 项路径不存在: ${x.path} (相对 ${wsRoot})`);
            }
            if (p.tiers === undefined) return;
            // 升档图: tiers 至少两档, 每项 from < to 且都在范围内, 胶囊短字不超过 16 字
            const n = Array.isArray(p.tiers) ? p.tiers.length : 0;
            if (i === 0 && n < 2) errs.push('<Apply> tiers 至少要两档');
            const ok = (v) => Number.isInteger(v) && v >= 0 && v < n;
            if (!ok(x.from) || !ok(x.to) || x.from >= x.to) errs.push(`<Apply> 第 ${i + 1} 项 from/to 要是 0..${n - 1} 的整数, 且 from < to`);
            for (const k of ['label', 'target']) {
                if (typeof x[k] !== 'string' || !x[k].trim()) errs.push(`<Apply> 第 ${i + 1} 项升档图需要 ${k}`);
                else if (textLen(x[k]) > 16) errs.push(`<Apply> 第 ${i + 1} 项 ${k} 超过 16 字: ${x[k]}`);
            }
        })),
        Boundary: stub('Boundary', (p) => list('Boundary', (p.items || []).map((t) => ({ t })), ['t'])),
        Figure: stub('Figure', (p) => req(errs, 'Figure', p, ['src', 'alt', 'caption'])),
        Chart: stub('Chart', (p) => {
            req(errs, 'Chart', p, ['kind', 'data', 'source', 'caption']);
            if (!['bar', 'range'].includes(p.kind)) errs.push(`<Chart> kind 只能是 bar / range, 实际是 ${p.kind}`);
            const keys = p.kind === 'range' ? ['name', 'from', 'to'] : ['name', 'value'];
            list('Chart', p.data, keys, (x, i) => {
                for (const k of keys.slice(1)) if (typeof x[k] !== 'number' || !Number.isFinite(x[k])) errs.push(`<Chart> 第 ${i + 1} 项 ${k} 要是数字`);
            });
            if (Array.isArray(p.data) && p.data.length < 4) errs.push('<Chart> 少于 4 项就别画图, 写进正文或用 <Stats>');
        }),
        Red: stub('Red', (p) => req(errs, 'Red', p, ['children'])),
        Gold: stub('Gold', (p) => req(errs, 'Gold', p, ['children'])),
    };
}

async function checkFile (file) {
    const errs = [];
    currentDir = path.dirname(file);
    missingAssets.length = 0;
    const src = fs.readFileSync(file, 'utf8');
    if (/^import\s/m.test(src)) errs.push("不要写 import, 组件已全局注册; 图片用 require('./x.png').default");
    const { fm, body, offset } = splitFrontmatter(src);
    const usesJsx = /^<[A-Za-z]/m.test(body);
    if (/^format:/m.test(fm)) errs.push('顶层 format 不生效, 要写成 mdx 下面缩进 format: mdx');
    if (usesJsx && !/^mdx:\s*\n\s+format:\s*mdx\s*$/m.test(fm)) errs.push('正文用了组件, frontmatter 需要 mdx 下面缩进 format: mdx');

    // 2. 标签白名单 (源码层面)
    const fence = body.replace(/```[\s\S]*?```/g, '').replace(/`[^`\n]*`/g, '');
    for (const m of fence.matchAll(/<([A-Za-z][\w.]*)/g)) {
        const tag = m[1];
        if (/^[a-z]/.test(tag)) errs.push(`不允许 HTML 标签 <${tag}>, 用 KBNote 组件或 Markdown`);
        else if (!ALLOWED.includes(tag)) errs.push(`未注册的组件 <${tag}>, 可用: ${ALLOWED.join(', ')}`);
    }

    // 6. 表格
    proseLines(body).forEach(({ line, no }) => {
        if (/^\s*\|.*\|\s*$/.test(line) && /^\s*\|?\s*:?-{3,}/.test(body.split('\n')[no] || '')) errs.push(`第 ${no + offset} 行: 不允许 Markdown 表格`);
    });

    // 7. 段落长度
    let para = [];
    const flush = () => {
        if (!para.length) return;
        const text = para.map((p) => p.line).join('');
        if (textLen(text) > MAX_PARA) errs.push(`第 ${para[0].no + offset} 行: 这段 ${textLen(text)} 字, 超过 ${MAX_PARA}`);
        para = [];
    };
    for (const pl of proseLines(body)) {
        const t = pl.line.trim();
        if (!t || /^(#|[-*] |\d+\. |>)/.test(t)) { flush(); continue; }
        para.push(pl);
    }
    flush();

    // 8. 行内标注: 只标短语, 一节 (## 之间) 最多一处
    for (const m of body.matchAll(/<(Red|Gold)>([\s\S]*?)<\/\1>/g)) {
        if (textLen(m[2]) > MAX_MARK) errs.push(`<${m[1]}> 只标短语, 这处 ${textLen(m[2])} 字, 超过 ${MAX_MARK}: ${m[2].slice(0, 20)}...`);
    }
    body.split(/^##\s.*$/m).forEach((sec, i) => {
        const n = (sec.match(/<(Red|Gold)>/g) || []).length;
        if (n > 1) errs.push(`第 ${i} 个 ## 节里有 ${n} 处 <Red>/<Gold>, 一节最多一处`);
    });

    // 4. 不成立一节
    if (!/^##\s*什么时候不成立\s*$/m.test(body)) errs.push('缺少 `## 什么时候不成立` 一节');

    // 3. Lead 位置
    const leadAt = body.search(/^<Lead\b/m);
    const h2At = body.search(/^##\s/m);
    if (leadAt < 0) errs.push('缺少 <Lead>');
    else if (h2At >= 0 && leadAt > h2At) errs.push('<Lead> 要放在第一个 ## 之前');

    // 1/5. 编译 + 桩执行
    const seen = [];
    try {
        const mod = await evaluate(body, { ...runtime, remarkPlugins: [remarkGfm], development: false });
        const Content = mod.default;
        const stubs = makeStubs(errs, seen);
        // 未注册的组件上面已经报过, 这里给个空桩, 让其余组件照常被检查
        for (const m of fence.matchAll(/<([A-Z][\w]*)/g)) if (!stubs[m[1]]) stubs[m[1]] = () => null;
        const tree = Content({ components: stubs });
        // 展开一层, 让桩组件真正被调用
        const walk = (n) => {
            if (!n || typeof n !== 'object') return;
            if (Array.isArray(n)) { n.forEach(walk); return; }
            if (typeof n.type === 'function') { walk(n.type(n.props || {})); return; }
            walk(n.props?.children);
        };
        walk(tree);
    } catch (e) {
        errs.push('MDX 编译或执行失败: ' + String(e.message || e).split('\n')[0]);
    }
    for (const a of missingAssets) errs.push(`图片不存在: ${a}`);
    const leads = seen.filter((x) => x === 'Lead').length;
    if (leads > 1) errs.push(`<Lead> 只能有一个, 实际 ${leads} 个`);

    return errs;
}

let failed = 0;
for (const a of args) {
    const file = fs.statSync(a).isDirectory() ? path.join(a, 'index.md') : a;
    const errs = await checkFile(file);
    if (errs.length) {
        failed++;
        console.log(`✗ ${file}`);
        for (const e of errs) console.log('  - ' + e);
    } else {
        console.log(`✓ ${file}`);
    }
}
process.exit(failed ? 1 : 0);
