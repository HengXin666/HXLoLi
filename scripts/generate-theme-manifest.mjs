#!/usr/bin/env node
/**
 * 扫描主题配置目录, 生成清单文件.
 *
 * 为什么需要: 静态站点无法列目录, 前端要知道"有哪些主题可用"必须有一份索引.
 * 扫描 static/themes/*.{yaml,yml,json} -> static/themes/index.json
 *
 * 用法: node scripts/generate-theme-manifest.mjs [static/themes]
 */
import fs from 'node:fs';
import path from 'node:path';

const dir = process.argv[2] || 'static/themes';
if (!fs.existsSync(dir)) {
    console.log('主题目录不存在, 跳过:', dir);
    process.exit(0);
}

const ext = /.(ya?ml|json)$/i;
const files = fs.readdirSync(dir).filter((f) => ext.test(f) && f !== 'index.json');

/** 从文件里粗略取 id/name (不引入 yaml 依赖, 只做首几行的轻量解析) */
function peek(file) {
    const text = fs.readFileSync(path.join(dir, file), 'utf8');
    const isJson = /\.json$/i.test(file);
    if (isJson) {
        try {
            const d = JSON.parse(text);
            return { id: d.id, name: d.name };
        } catch {
            return {};
        }
    }
    const id = /^id:\s*["']?([\w.-]+)/m.exec(text)?.[1];
    const name = /^name:\s*["']?(.+?)["']?\s*$/m.exec(text)?.[1];
    return { id, name };
}

const items = files.map((file) => {
    const { id, name } = peek(file);
    return {
        file,
        id: id || path.basename(file).replace(ext, ''),
        name: name || path.basename(file).replace(ext, ''),
    };
});

const out = { schema_version: 1, themes: items };
fs.writeFileSync(path.join(dir, 'index.json'), JSON.stringify(out, null, 2) + '\n');
console.log(`已生成 ${path.join(dir, 'index.json')} (${items.length} 个主题)`);
for (const t of items) console.log('  -', t.id, '|', t.name, '|', t.file);