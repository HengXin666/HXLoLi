#!/usr/bin/env node
/**
 * 将 search-index.json 拆分为多个 chunk 文件
 * 
 * search-index.json 的结构是一个 JSON 数组，每个元素包含 documents 和 index。
 * 本脚本将整个 JSON 字符串按字节大小拆分为多个 chunk，
 * 并生成 search-index-manifest.json 供 Worker 合并返回。
 * 
 * 用法: node scripts/split-search-index.mjs [--max-size <MB>] [--build-dir <dir>]
 */

import fs from 'fs';
import path from 'path';

const args = process.argv.slice(2);
function getArg(name, defaultValue) {
  const idx = args.indexOf(name);
  return idx !== -1 && args[idx + 1] ? args[idx + 1] : defaultValue;
}

const MAX_CHUNK_SIZE = parseInt(getArg('--max-size', '20'), 10) * 1024 * 1024; // 默认 20MB
const BUILD_DIR = getArg('--build-dir', './build');
const INDEX_FILE = path.join(BUILD_DIR, 'search-index.json');

if (!fs.existsSync(INDEX_FILE)) {
  console.log('⚠️  search-index.json 不存在, 跳过拆分');
  process.exit(0);
}

const stat = fs.statSync(INDEX_FILE);
const fileSizeMB = (stat.size / (1024 * 1024)).toFixed(2);

if (stat.size <= MAX_CHUNK_SIZE) {
  console.log(`✅ search-index.json (${fileSizeMB} MB) 未超过限制, 无需拆分`);
  process.exit(0);
}

console.log(`📦 search-index.json 大小: ${fileSizeMB} MB, 开始拆分...`);

const content = fs.readFileSync(INDEX_FILE, 'utf-8');
const totalBytes = Buffer.byteLength(content, 'utf-8');
const chunks = [];
let offset = 0;

while (offset < content.length) {
  // 按字符估算，找到不超过 MAX_CHUNK_SIZE 字节的切割点
  let end = offset;
  let byteCount = 0;
  
  while (end < content.length) {
    const charBytes = Buffer.byteLength(content[end], 'utf-8');
    if (byteCount + charBytes > MAX_CHUNK_SIZE) break;
    byteCount += charBytes;
    end++;
  }
  
  // 如果没有前进（单个字符就超过限制，不太可能），至少前进一个字符
  if (end === offset) end = offset + 1;
  
  chunks.push(content.slice(offset, end));
  offset = end;
}

console.log(`📄 拆分为 ${chunks.length} 个 chunk`);

const chunkFiles = [];
for (let i = 0; i < chunks.length; i++) {
  const chunkName = `search-index-chunk-${i}.txt`;
  const chunkPath = path.join(BUILD_DIR, chunkName);
  fs.writeFileSync(chunkPath, chunks[i], 'utf-8');
  const chunkSize = (Buffer.byteLength(chunks[i], 'utf-8') / (1024 * 1024)).toFixed(2);
  console.log(`  ✅ ${chunkName} (${chunkSize} MB)`);
  chunkFiles.push(chunkName);
}

// 写入 manifest
const manifest = {
  totalSize: totalBytes,
  chunks: chunkFiles,
};
fs.writeFileSync(
  path.join(BUILD_DIR, 'search-index-manifest.json'),
  JSON.stringify(manifest),
  'utf-8'
);

// 删除原始大文件
fs.unlinkSync(INDEX_FILE);
console.log(`🗑️  已删除原始 search-index.json`);
console.log(`✅ 拆分完成! manifest: search-index-manifest.json`);
