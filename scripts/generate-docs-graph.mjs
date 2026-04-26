#!/usr/bin/env node
/**
 * 扫描 docs 目录中所有 md/mdx 文件, 提取链接关系, 生成 docs-links-graph.json
 *
 * 输出格式:
 * {
 *   nodes: [{ id: "/docs/xxx", title: "标题" }, ...],
 *   links: [{ source: "/docs/xxx", target: "/docs/yyy", external: false }, ...]
 * }
 *
 * 用法: node scripts/generate-docs-graph.mjs [--docs-dir ./docs] [--output ./static/docs-links-graph.json]
 */

import fs from 'fs';
import path from 'path';

const args = process.argv.slice(2);
function getArg(name, defaultValue) {
  const idx = args.indexOf(name);
  return idx !== -1 && args[idx + 1] ? args[idx + 1] : defaultValue;
}

const DOCS_DIR = getArg('--docs-dir', './docs');
const OUTPUT = getArg('--output', './static/docs-links-graph.json');

// Docusaurus 将 docs/xxx/yyy/index.md 映射为 /docs/xxx/yyy
// docs/xxx/yyy/zzz.md 映射为 /docs/xxx/yyy/zzz
function docPathToUrl(filePath) {
  let rel = path.relative(DOCS_DIR, filePath).replace(/\\/g, '/');
  // 去掉数字前缀 (如 001-xxx → xxx)
  rel = rel.split('/').map(seg => seg.replace(/^\d+-/, '')).join('/');
  // 去掉 .md / .mdx 扩展名
  rel = rel.replace(/\.(md|mdx)$/i, '');
  // 去掉末尾的 /index
  rel = rel.replace(/\/index$/, '');
  // 如果整个就是 index
  if (rel === 'index') rel = '';
  return '/docs/' + rel;
}

// 从 markdown 内容提取第一个 h1 标题
function extractTitle(content) {
  const match = content.match(/^#\s+(.+)$/m);
  return match ? match[1].trim() : null;
}

// 从 markdown 内容提取所有链接
// 匹配 [text](url) 和 <a href="url">
function extractLinks(content) {
  const links = [];
  // markdown 链接: [text](url)
  const mdLinkRegex = /\[([^\]]*)\]\(([^)]+)\)/g;
  let match;
  while ((match = mdLinkRegex.exec(content)) !== null) {
    const url = match[2].trim();
    // 跳过图片引用 (以 .png, .jpg, .svg, .gif, .webp 等结尾)
    if (/\.(png|jpe?g|svg|gif|webp|bmp|ico|drawio)(\?.*)?$/i.test(url)) continue;
    // 跳过锚点链接
    if (url.startsWith('#')) continue;
    links.push(url);
  }
  // HTML 链接: href="url"
  const htmlLinkRegex = /href=["']([^"']+)["']/g;
  while ((match = htmlLinkRegex.exec(content)) !== null) {
    const url = match[1].trim();
    if (url.startsWith('#')) continue;
    links.push(url);
  }
  return links;
}

// 判断是否为外部链接
function isExternal(url) {
  return /^https?:\/\//i.test(url) || url.startsWith('//');
}

// 将相对链接解析为 docs URL
function resolveDocLink(fromFile, rawUrl) {
  // 去掉锚点
  const url = rawUrl.split('#')[0].split('?')[0];
  if (!url) return null;

  if (isExternal(url)) {
    return { url: rawUrl.split('#')[0], external: true };
  }

  // 绝对路径 (以 / 开头)
  if (url.startsWith('/docs/')) {
    return { url: url, external: false };
  }
  if (url.startsWith('/')) {
    // 非 docs 路径 (如 /blog/xxx)
    return { url: url, external: true };
  }

  // 相对路径
  const fromDir = path.dirname(fromFile);
  const resolved = path.resolve(fromDir, url);

  // 检查是否在 docs 目录内
  const absDocsDir = path.resolve(DOCS_DIR);
  if (!resolved.startsWith(absDocsDir)) {
    return null;
  }

  // 检查文件是否存在
  let targetFile = resolved;
  if (fs.existsSync(resolved) && fs.statSync(resolved).isDirectory()) {
    // 目录 → 找 index.md 或 index.mdx
    if (fs.existsSync(path.join(resolved, 'index.md'))) {
      targetFile = path.join(resolved, 'index.md');
    } else if (fs.existsSync(path.join(resolved, 'index.mdx'))) {
      targetFile = path.join(resolved, 'index.mdx');
    }
  } else if (!fs.existsSync(resolved)) {
    // 尝试加 .md / .mdx
    if (fs.existsSync(resolved + '.md')) {
      targetFile = resolved + '.md';
    } else if (fs.existsSync(resolved + '.mdx')) {
      targetFile = resolved + '.mdx';
    } else {
      return null;
    }
  }

  return { url: docPathToUrl(targetFile), external: false };
}

// 检查文件是否属于私有笔记 (以 _ 开头的目录 或 frontmatter 中有 hx_protected)
function isPrivate(filePath, content) {
  const rel = path.relative(DOCS_DIR, filePath);
  if (rel.split(path.sep).some(seg => seg.startsWith('_'))) return true;
  // 检查 frontmatter 中的 hx_protected 标记
  const fmMatch = content.match(/^---\s*\n([\s\S]*?)\n---/);
  if (fmMatch && /hx_protected:\s*true/i.test(fmMatch[1])) return true;
  return false;
}

// 主逻辑
function main() {
  const nodesMap = new Map(); // url → { id, title }
  const linksSet = new Set(); // "source|target|external" 去重
  const links = [];

  // 递归扫描 docs 目录
  function scanDir(dir) {
    const items = fs.readdirSync(dir);
    for (const item of items) {
      const fullPath = path.join(dir, item);
      const stat = fs.statSync(fullPath);
      if (stat.isDirectory()) {
        scanDir(fullPath);
      } else if (/\.(md|mdx)$/i.test(item)) {
        const content = fs.readFileSync(fullPath, 'utf-8');
        if (isPrivate(fullPath, content)) continue;

        const url = docPathToUrl(fullPath);
        const title = extractTitle(content) || path.basename(path.dirname(fullPath)).replace(/^\d+-/, '');

        nodesMap.set(url, { id: url, title });

        const rawLinks = extractLinks(content);
        for (const rawLink of rawLinks) {
          const resolved = resolveDocLink(fullPath, rawLink);
          if (!resolved) continue;

          const key = `${url}|${resolved.url}|${resolved.external}`;
          if (linksSet.has(key)) continue;
          linksSet.add(key);

          // 确保外部链接的目标也作为节点存在
          if (!resolved.external && !nodesMap.has(resolved.url)) {
            nodesMap.set(resolved.url, { id: resolved.url, title: resolved.url.split('/').pop() || resolved.url });
          }

          links.push({
            source: url,
            target: resolved.url,
            external: resolved.external,
          });
        }
      }
    }
  }

  scanDir(DOCS_DIR);

  // 过滤: 只保留内部链接中 source 和 target 都存在于 nodesMap 中的
  const validLinks = links.filter(l => {
    if (l.external) return true;
    return nodesMap.has(l.source) && nodesMap.has(l.target);
  });

  const result = {
    nodes: Array.from(nodesMap.values()),
    links: validLinks,
  };

  // 确保输出目录存在
  const outDir = path.dirname(OUTPUT);
  if (!fs.existsSync(outDir)) {
    fs.mkdirSync(outDir, { recursive: true });
  }

  fs.writeFileSync(OUTPUT, JSON.stringify(result), 'utf-8');

  const internalLinks = validLinks.filter(l => !l.external).length;
  const externalLinks = validLinks.filter(l => l.external).length;
  console.log(`✅ docs-links-graph.json 生成完成`);
  console.log(`   节点: ${result.nodes.length}, 内部链接: ${internalLinks}, 外部链接: ${externalLinks}`);
}

main();
