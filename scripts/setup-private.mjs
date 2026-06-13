#!/usr/bin/env node
/**
 * setup-private.mjs
 *
 * 本地开发辅助脚本:
 *   1. 在 HXLoLis/HXLoLi-imouto 存在时, 直接把私有分区映射到本库
 *   2. 否则按指定路径/默认路径查找, 不存在时才 clone
 *   3. 映射 docs/blog/ai-docs, 本地编辑会直接写回 HXLoLi-imouto
 *
 * 用法:
 *   node scripts/setup-private.mjs [私有仓库本地路径]
 *
 *   如果不指定路径, 优先使用 HXLoLis/HXLoLi-imouto, 否则使用同级目录 ../HXLoLi-imouto
 *
 * 环境变量:
 *   GITHUB_TOKEN  - GitHub Personal Access Token (用于克隆私有仓库)
 *   PRIVATE_REPO  - 私有仓库 GitHub 路径 (默认: HengXin666/HXLoLi-imouto)
 */

import { execSync } from 'child_process';
import { existsSync, lstatSync, mkdirSync, readFileSync, readdirSync, rmSync, statSync, symlinkSync, writeFileSync } from 'fs';
import { basename, join, relative, resolve, dirname } from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const projectRoot = resolve(__dirname, '..');

const PRIVATE_REPO = process.env.PRIVATE_REPO || 'HengXin666/HXLoLi-imouto';
const GITHUB_TOKEN = process.env.GITHUB_TOKEN || '';

// 默认私有仓库本地路径: HXLoLis/HXLoLi + HXLoLis/HXLoLi-imouto
const defaultLocalPath = resolve(projectRoot, '..', 'HXLoLi-imouto');
const hxlolisLocalPath = basename(resolve(projectRoot, '..')) === 'HXLoLis'
  ? defaultLocalPath
  : '';
const localPath = resolve(process.argv[2] || (hxlolisLocalPath && existsSync(hxlolisLocalPath) ? hxlolisLocalPath : defaultLocalPath));

const privateSections = ['docs', 'blog', 'ai-docs'];
const excludeBegin = '# BEGIN HXLoLi-imouto private mappings';
const excludeEnd = '# END HXLoLi-imouto private mappings';

function run(cmd, opts = {}) {
  console.log(`  $ ${cmd}`);
  try {
    execSync(cmd, { stdio: 'inherit', ...opts });
  } catch (e) {
    if (!opts.allowFail) {
      console.error(`❌ 命令执行失败: ${cmd}`);
      process.exit(1);
    }
  }
}

function linkEntry(sourcePath, targetPath, displayPath, linkedPaths) {
  const sourceStat = statSync(sourcePath);

  if (existsSync(targetPath)) {
    const targetStat = lstatSync(targetPath);
    if (targetStat.isSymbolicLink()) {
      rmSync(targetPath);
    } else if (targetStat.isDirectory() && sourceStat.isDirectory()) {
      let linked = 0;
      for (const entry of readdirSync(sourcePath)) {
        if (entry.startsWith('.')) continue;
        linked += linkEntry(
          join(sourcePath, entry),
          join(targetPath, entry),
          `${displayPath}/${entry}`,
          linkedPaths,
        );
      }
      return linked;
    } else {
      console.warn(`⚠️  跳过 ${displayPath}: 目标已存在且不是兼容目录/映射`);
      return 0;
    }
  }

  const relSource = relative(dirname(targetPath), sourcePath);
  symlinkSync(relSource, targetPath);
  linkedPaths.push(displayPath);
  console.log(`  🔗 ${displayPath} -> ${relSource}`);
  return 1;
}

function getTopLevelPrivatePaths() {
  const paths = [];
  for (const section of privateSections) {
    const sourceDir = join(localPath, section);
    if (!existsSync(sourceDir)) continue;
    for (const entry of readdirSync(sourceDir)) {
      if (!entry.startsWith('.')) {
        paths.push(`${section}/${entry}`);
      }
    }
  }
  return paths;
}

function updateGitExclude(paths) {
  const gitDir = join(projectRoot, '.git');
  if (!existsSync(gitDir)) return;

  const excludePath = join(gitDir, 'info', 'exclude');
  mkdirSync(dirname(excludePath), { recursive: true });

  const current = existsSync(excludePath) ? readFileSync(excludePath, 'utf8') : '';
  const oldManagedPaths = new Set([excludeBegin, excludeEnd, '# HXLoLi-imouto private mappings', ...getTopLevelPrivatePaths()]);
  const currentLines = current.split('\n');
  const nextLines = [];
  let skipping = false;

  for (const line of currentLines) {
    if (line === excludeBegin || line === '# HXLoLi-imouto private mappings') {
      skipping = true;
      continue;
    }
    if (line === excludeEnd) {
      skipping = false;
      continue;
    }
    if (skipping || oldManagedPaths.has(line)) {
      continue;
    }
    nextLines.push(line);
  }

  while (nextLines.length > 0 && nextLines[nextLines.length - 1] === '') {
    nextLines.pop();
  }

  if (paths.length > 0) {
    nextLines.push('', excludeBegin, ...paths, excludeEnd);
  }

  writeFileSync(excludePath, `${nextLines.join('\n')}\n`, 'utf8');
}

function linkPrivateSection(section, linkedPaths) {
  const sourceDir = join(localPath, section);
  const targetDir = join(projectRoot, section);

  if (!existsSync(sourceDir)) {
    return 0;
  }

  mkdirSync(targetDir, { recursive: true });

  let linked = 0;
  for (const entry of readdirSync(sourceDir)) {
    if (entry.startsWith('.')) continue;

    const sourcePath = join(sourceDir, entry);
    const targetPath = join(targetDir, entry);
    linked += linkEntry(sourcePath, targetPath, `${section}/${entry}`, linkedPaths);
  }

  return linked;
}

function main() {
  console.log('🔧 HXLoLi 本地开发 - 私有页面设置');
  console.log('━'.repeat(50));

  // Step 1: 确保私有仓库存在
  if (existsSync(localPath)) {
    console.log(`📂 发现本地仓库: ${localPath}`);
    console.log('   本地开发使用映射模式, 不自动 git pull, 避免覆盖未提交修改');
  } else {
    console.log(`📂 本地仓库不存在, 开始克隆...`);
    console.log(`   仓库: ${PRIVATE_REPO}`);
    console.log(`   目标: ${localPath}`);

    if (GITHUB_TOKEN) {
      run(`git clone "https://x-access-token:${GITHUB_TOKEN}@github.com/${PRIVATE_REPO}.git" "${localPath}"`);
    } else {
      // 尝试 SSH 方式
      console.log('   💡 未设置 GITHUB_TOKEN, 尝试 SSH 方式...');
      run(`git clone "git@github.com:${PRIVATE_REPO}.git" "${localPath}"`);
    }
  }

  // Step 2: 映射私有分区
  console.log('\n📋 映射私有页面...');
  let linked = 0;
  const linkedPaths = [];
  for (const section of privateSections) {
    linked += linkPrivateSection(section, linkedPaths);
  }

  updateGitExclude(linkedPaths);

  if (linked === 0) {
    console.log('⚠️  没有发现可映射的 docs/blog/ai-docs 分区');
  }

  // Step 3: 重新生成 sidebar (因为可能有新的 docs/ai-docs 页面)
  console.log('\n📋 重新生成侧边栏...');
  run('node scripts/generateSidebar.js', { cwd: projectRoot });
  run('node scripts/generateAiDocsSidebar.js', { cwd: projectRoot });

  console.log('\n' + '━'.repeat(50));
  console.log('🎉 设置完成! 现在可以运行 npm start 预览所有页面');
  console.log('   (私有页面通过符号链接映射, 修改会写回 HXLoLi-imouto)');
}

main();
