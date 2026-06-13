#!/usr/bin/env node
/**
 * clean-private.mjs
 *
 * 清理开发模式下映射/拷贝到 docs/、blog/ 和 ai-docs/ 的私有页面文件
 * 通过对比私有仓库目录结构来精确删除, 不会影响公有仓库原有文件
 *
 * 用法:
 *   node scripts/clean-private.mjs [私有仓库本地路径]
 */

import { existsSync, lstatSync, readdirSync, statSync, unlinkSync, rmdirSync } from 'fs';
import { join, resolve, dirname, sep } from 'path';
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

function collectPrivateMappingSymlinks(sourceDir, targetDir, base = sourceDir) {
  const results = [];
  if (!existsSync(sourceDir)) return results;

  for (const item of readdirSync(sourceDir)) {
    if (item.startsWith('.')) continue;
    const sourcePath = join(sourceDir, item);
    const relPath = sourcePath.slice(base.length + 1);
    const targetPath = join(targetDir, relPath);

    if (existsSync(targetPath) && lstatSync(targetPath).isSymbolicLink()) {
      results.push(relPath);
      continue;
    }

    const sourceStat = statSync(sourcePath);
    if (sourceStat.isDirectory()) {
      results.push(...collectPrivateMappingSymlinks(sourcePath, targetDir, base));
    }
  }

  return results;
}

function isUnderPath(path, parent) {
  return path === parent || path.startsWith(`${parent}${sep}`);
}

function tryRemove(filePath) {
  try {
    if (existsSync(filePath)) {
      const linkStat = lstatSync(filePath);
      if (linkStat.isSymbolicLink()) {
        unlinkSync(filePath);
        return true;
      }

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

  for (const section of ['docs', 'blog', 'ai-docs']) {
    const sectionSource = join(localPath, section);
    if (!existsSync(sectionSource)) continue;

    console.log(`📁 清理 ${section}/ 中的私有文件...`);

    const sectionTarget = join(projectRoot, section);
    const symlinks = collectPrivateMappingSymlinks(sectionSource, sectionTarget);

    for (const relPath of symlinks) {
      const targetPath = join(sectionTarget, relPath);
      if (tryRemove(targetPath)) {
        console.log(`  🗑️  ${section}/${relPath} -> 映射`);
        cleaned++;
      }
    }

    const files = getRelativePaths(sectionSource)
      .filter(entry => !symlinks.some(linkPath => isUnderPath(entry.path, linkPath)));
    const fileEntries = files.filter(f => !f.isDir);
    const dirEntries = files.filter(f => f.isDir).reverse();

    for (const entry of readdirSync(sectionSource)) {
      if (entry.startsWith('.')) continue;
      const targetPath = join(sectionTarget, entry);
      if (existsSync(targetPath) && lstatSync(targetPath).isSymbolicLink()) {
        if (tryRemove(targetPath)) {
          console.log(`  🗑️  ${section}/${entry} -> 映射`);
          cleaned++;
        }
      }
    }

    for (const entry of fileEntries) {
      const targetPath = join(sectionTarget, entry.path);
      const mdxPath = targetPath.replace(/\.md$/, '.mdx');
      if (tryRemove(targetPath)) {
        console.log(`  🗑️  ${section}/${entry.path}`);
        cleaned++;
      }
      if (tryRemove(mdxPath)) {
        console.log(`  🗑️  ${section}/${entry.path.replace(/\.md$/, '.mdx')}`);
        cleaned++;
      }
    }

    for (const entry of dirEntries) {
      const targetPath = join(sectionTarget, entry.path);
      if (tryRemove(targetPath)) {
        console.log(`  🗑️  ${section}/${entry.path}/`);
        cleaned++;
      }
    }
  }

  console.log('\n' + '━'.repeat(50));
  console.log(`🎉 清理完成! 共删除 ${cleaned} 个文件/目录`);
}

main();
