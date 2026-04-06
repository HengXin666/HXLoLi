#!/usr/bin/env node
/**
 * clean-private.mjs
 *
 * 清理开发模式下拷贝到 docs/ 和 blog/ 的私有页面文件
 * 通过对比私有仓库目录结构来精确删除, 不会影响公有仓库原有文件
 *
 * 用法:
 *   node scripts/clean-private.mjs [私有仓库本地路径]
 */

import { existsSync, readdirSync, statSync, unlinkSync, rmdirSync } from 'fs';
import { join, resolve, dirname } from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const projectRoot = resolve(__dirname, '..');

const defaultLocalPath = resolve(projectRoot, '..', 'HXLoLi-imouto');
const localPath = process.argv[2] || defaultLocalPath;

function getRelativePaths(dir, base = dir) {
  const results = [];
  if (!existsSync(dir)) return results;

  const items = readdirSync(dir);
  for (const item of items) {
    if (item.startsWith('.')) continue;
    const fullPath = join(dir, item);
    const stat = statSync(fullPath);
    if (stat.isDirectory()) {
      results.push(...getRelativePaths(fullPath, base));
      // 也记录目录本身 (用于清理空目录)
      results.push({ path: fullPath.slice(base.length + 1), isDir: true });
    } else {
      results.push({ path: fullPath.slice(base.length + 1), isDir: false });
    }
  }
  return results;
}

function tryRemove(filePath) {
  try {
    if (existsSync(filePath)) {
      const stat = statSync(filePath);
      if (stat.isDirectory()) {
        // 只删除空目录
        const items = readdirSync(filePath);
        if (items.length === 0) {
          rmdirSync(filePath);
          return true;
        }
      } else {
        unlinkSync(filePath);
        return true;
      }
    }
  } catch (e) {
    // ignore
  }
  return false;
}

function main() {
  console.log('🧹 清理私有页面文件');
  console.log('━'.repeat(50));

  if (!existsSync(localPath)) {
    console.log(`⚠️  私有仓库路径不存在: ${localPath}`);
    console.log('   无法确定需要清理的文件');
    return;
  }

  let cleaned = 0;

  // 清理 docs 相关文件
  const docsSource = join(localPath, 'docs');
  if (existsSync(docsSource)) {
    console.log('📁 清理 docs/ 中的私有文件...');
    const files = getRelativePaths(docsSource);
    // 先删除文件, 后删除目录
    const fileEntries = files.filter(f => !f.isDir);
    const dirEntries = files.filter(f => f.isDir).reverse(); // 从深到浅

    for (const entry of fileEntries) {
      // .md 可能被转为 .mdx
      const targetPath = join(projectRoot, 'docs', entry.path);
      const mdxPath = targetPath.replace(/\.md$/, '.mdx');
      if (tryRemove(targetPath)) {
        console.log(`  🗑️  docs/${entry.path}`);
        cleaned++;
      }
      if (tryRemove(mdxPath)) {
        console.log(`  🗑️  docs/${entry.path.replace(/\.md$/, '.mdx')}`);
        cleaned++;
      }
    }

    for (const entry of dirEntries) {
      const targetPath = join(projectRoot, 'docs', entry.path);
      if (tryRemove(targetPath)) {
        console.log(`  🗑️  docs/${entry.path}/`);
        cleaned++;
      }
    }
  }

  // 清理 blog 相关文件
  const blogSource = join(localPath, 'blog');
  if (existsSync(blogSource)) {
    console.log('📁 清理 blog/ 中的私有文件...');
    const files = getRelativePaths(blogSource);
    const fileEntries = files.filter(f => !f.isDir);
    const dirEntries = files.filter(f => f.isDir).reverse();

    for (const entry of fileEntries) {
      const targetPath = join(projectRoot, 'blog', entry.path);
      const mdxPath = targetPath.replace(/\.md$/, '.mdx');
      if (tryRemove(targetPath)) {
        console.log(`  🗑️  blog/${entry.path}`);
        cleaned++;
      }
      if (tryRemove(mdxPath)) {
        console.log(`  🗑️  blog/${entry.path.replace(/\.md$/, '.mdx')}`);
        cleaned++;
      }
    }

    for (const entry of dirEntries) {
      const targetPath = join(projectRoot, 'blog', entry.path);
      if (tryRemove(targetPath)) {
        console.log(`  🗑️  blog/${entry.path}/`);
        cleaned++;
      }
    }
  }

  console.log('\n' + '━'.repeat(50));
  console.log(`🎉 清理完成! 共删除 ${cleaned} 个文件/目录`);
}

main();
