#!/usr/bin/env node
/**
 * 校验 wrangler.toml 的静态资源部署配置。
 *
 * 为什么需要它: worker.js 通过 env.ASSETS 访问构建产物 (目录页 / 404 兜底 /
 * search-index 合并)。Wrangler 只有在 [assets] 里显式写了 binding 时才会注入
 * 这个绑定; 漏写时 deploy 不会报任何错, 但请求一旦落到脚本上就会抛
 * "TypeError: Cannot read properties of undefined (reading 'fetch')",
 * 线上表现为 Cloudflare Error 1101 —— 排查成本极高 (本次事故: 2026-09-13)。
 *
 * 用法: node scripts/check-cf-worker-config.mjs [--config wrangler.toml]
 */

import fs from 'fs';
import path from 'path';

const args = process.argv.slice(2);
const configIndex = args.indexOf('--config');
const configPath = path.resolve(
  configIndex === -1 || !args[configIndex + 1] ? 'wrangler.toml' : args[configIndex + 1],
);

if (!fs.existsSync(configPath)) {
  console.error(`❌ 找不到配置文件: ${configPath}`);
  process.exit(1);
}

const content = fs.readFileSync(configPath, 'utf8');

/** 取 [assets] 段里的 key = value (忽略注释与前后空白) */
function readAssetsSection(text) {
  const result = {};
  let inAssets = false;

  for (const rawLine of text.split(/\r?\n/)) {
    const line = rawLine.replace(/#.*$/, '').trim();
    if (!line) continue;

    if (/^\[/.test(line)) {
      inAssets = line === '[assets]';
      continue;
    }
    if (!inAssets) continue;

    const match = /^([A-Za-z0-9_-]+)\s*=\s*(.+?)\s*$/.exec(line);
    if (match) result[match[1]] = match[2].replace(/^["']|["']$/g, '');
  }

  return result;
}

const assets = readAssetsSection(content);
const problems = [];

if (!assets.directory) {
  problems.push('[assets] 缺少 directory —— Worker 没有静态资源可服务');
}

if (!assets.binding) {
  problems.push(
    '[assets] 缺少 binding —— worker.js 里的 env.ASSETS 会是 undefined, 线上直接 Error 1101',
  );
}

const mainMatch = /^\s*main\s*=\s*(.+?)\s*$/m.exec(content);
if (!mainMatch) {
  problems.push('缺少 main —— 没有 Worker 脚本入口');
}

if (problems.length > 0) {
  console.error('❌ Cloudflare Workers 配置校验失败:');
  for (const problem of problems) console.error(`   - ${problem}`);
  console.error('');
  console.error('   [assets] 段正确写法:');
  console.error('     [assets]');
  console.error('     directory = "./build"');
  console.error('     binding = "ASSETS"');
  process.exit(1);
}

console.log(`✅ Cloudflare Workers 配置校验通过 (${path.basename(configPath)})`);
console.log(`   main      = ${mainMatch ? mainMatch[1].replace(/^["']|["']$/g, '') : '(未声明)'}`);
console.log(`   directory = ${assets.directory}`);
console.log(`   binding   = ${assets.binding}`);
