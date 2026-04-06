#!/usr/bin/env node
/**
 * setup-private.mjs
 *
 * 本地开发辅助脚本:
 *   1. 如果本地没有私有仓库, 通过 git clone 拉取
 *   2. 如果已有, 执行 git pull 更新
 *   3. 以 dev 模式运行 encrypt-private.mjs (直接拷贝明文)
 *
 * 用法:
 *   node scripts/setup-private.mjs [私有仓库本地路径]
 *
 *   如果不指定路径, 默认使用同级目录 ../HXLoLi-imouto
 *
 * 环境变量:
 *   GITHUB_TOKEN  - GitHub Personal Access Token (用于克隆私有仓库)
 *   PRIVATE_REPO  - 私有仓库 GitHub 路径 (默认: HengXin666/HXLoLi-imouto)
 */

import { execSync } from 'child_process';
import { existsSync } from 'fs';
import { resolve, dirname } from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const projectRoot = resolve(__dirname, '..');

const PRIVATE_REPO = process.env.PRIVATE_REPO || 'HengXin666/HXLoLi-imouto';
const GITHUB_TOKEN = process.env.GITHUB_TOKEN || '';

// 默认私有仓库本地路径
const defaultLocalPath = resolve(projectRoot, '..', 'HXLoLi-imouto');
const localPath = process.argv[2] || defaultLocalPath;

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

function main() {
  console.log('🔧 HXLoLi 本地开发 - 私有页面设置');
  console.log('━'.repeat(50));

  // Step 1: 确保私有仓库存在
  if (existsSync(localPath)) {
    console.log(`📂 发现本地仓库: ${localPath}`);
    console.log('📥 拉取最新内容...');
    run('git pull', { cwd: localPath, allowFail: true });
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

  // Step 2: 以 dev 模式运行加密脚本 (实际是拷贝明文)
  console.log('\n📋 以开发模式拷贝私有页面...');
  run(`node scripts/encrypt-private.mjs --source "${localPath}" --mode dev`, { cwd: projectRoot });

  // Step 3: 重新生成 sidebar (因为可能有新的 docs 页面)
  console.log('\n📋 重新生成侧边栏...');
  run('node scripts/generateSidebar.js', { cwd: projectRoot });

  console.log('\n' + '━'.repeat(50));
  console.log('🎉 设置完成! 现在可以运行 npm start 预览所有页面');
  console.log('   (包括来自私有仓库的页面, 以明文显示)');
}

main();
