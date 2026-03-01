#!/usr/bin/env node

/**
 * 友链 PR 验证脚本
 *
 * 验证内容:
 * 1. PR 只修改了 data/friendLinks.ts
 * 2. 只允许新增记录 / 修改自己曾经提交的记录
 * 3. github 字段链接与 PR 提交人一致
 * 4. 数据格式合法 (所有必填字段存在且为非空字符串)
 * 5. 编译通过 (TypeScript 类型检查)
 * 6. 所有链接 HTTP 200 可达
 */

import { execSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const ROOT = resolve(__dirname, '..');

/* ========== 工具函数 ========== */

/** 彩色输出 */
const log = {
  info: (msg) => console.log(`\x1b[36mℹ ${msg}\x1b[0m`),
  ok: (msg) => console.log(`\x1b[32m✔ ${msg}\x1b[0m`),
  warn: (msg) => console.log(`\x1b[33m⚠ ${msg}\x1b[0m`),
  fail: (msg) => console.error(`\x1b[31m✖ ${msg}\x1b[0m`),
};

const errors = [];
function fail(msg) {
  log.fail(msg);
  errors.push(msg);
}

/* ========== 环境变量 ========== */
// PR 提交人的 GitHub 用户名 (由 CI 传入)
const prAuthor = (process.env.PR_AUTHOR || '').toLowerCase();
// 基准分支 (用于 diff)
const baseSha = process.env.BASE_SHA || 'origin/main';

/* ========== 1. 检查 PR 只修改了 data/friendLinks.ts ========== */
log.info('检查 PR 修改的文件范围...');

let changedFiles;
try {
  changedFiles = execSync(`git diff --name-only ${baseSha}...HEAD`, {
    cwd: ROOT,
    encoding: 'utf-8',
  })
    .trim()
    .split('\n')
    .filter(Boolean);
} catch {
  // 如果 merge-base 比较失败，尝试直接 diff
  changedFiles = execSync(`git diff --name-only ${baseSha} HEAD`, {
    cwd: ROOT,
    encoding: 'utf-8',
  })
    .trim()
    .split('\n')
    .filter(Boolean);
}

const ALLOWED_FILE = 'data/friendLinks.ts';
const disallowed = changedFiles.filter((f) => f !== ALLOWED_FILE);

if (disallowed.length > 0) {
  fail(`PR 只允许修改 ${ALLOWED_FILE}，但还修改了: ${disallowed.join(', ')}`);
}

if (!changedFiles.includes(ALLOWED_FILE)) {
  fail(`PR 没有修改 ${ALLOWED_FILE}，无需验证友链`);
}

/* ========== 2. 解析新旧友链数据 ========== */
log.info('解析友链数据...');

/**
 * 从 friendLinks.ts 源码中提取友链数组
 * 使用简单正则 + JSON 解析，避免引入额外依赖
 */
function parseFriendLinks(source) {
  // 把 TS 数组转换为合法 JSON
  // 移除 export / interface / type 等声明，只保留数组部分
  const arrayMatch = source.match(
    /export\s+const\s+friendLinks\s*(?::\s*FriendLink\[\])?\s*=\s*\[([\s\S]*?)\];/
  );
  if (!arrayMatch) return [];

  let body = arrayMatch[1];
  // 移除单行注释
  body = body.replace(/\/\/.*$/gm, '');
  // 移除多行注释
  body = body.replace(/\/\*[\s\S]*?\*\//g, '');
  // 给没有引号的 key 加引号 (name: -> "name":)
  body = body.replace(/(\w+)\s*:/g, '"$1":');
  // 把单引号换成双引号
  body = body.replace(/'/g, '"');
  // 移除末尾逗号 (JSON 不允许)
  body = body.replace(/,\s*([\]}])/g, '$1');

  try {
    return JSON.parse(`[${body}]`);
  } catch (e) {
    fail(`友链数据解析失败: ${e.message}`);
    return [];
  }
}

// 读取当前版本
const currentSource = readFileSync(resolve(ROOT, ALLOWED_FILE), 'utf-8');
const currentLinks = parseFriendLinks(currentSource);

// 读取基准版本
let baseLinks = [];
try {
  const baseSource = execSync(`git show ${baseSha}:${ALLOWED_FILE}`, {
    cwd: ROOT,
    encoding: 'utf-8',
  });
  baseLinks = parseFriendLinks(baseSource);
} catch {
  log.warn('无法获取基准版本的友链数据，跳过增量检查 (可能是首次添加)');
}

log.ok(`基准版本: ${baseLinks.length} 条友链, 当前版本: ${currentLinks.length} 条友链`);

/* ========== 3. 格式合法性检查 ========== */
log.info('检查数据格式...');

const REQUIRED_FIELDS = ['name', 'owner', 'url', 'github', 'avatar', 'description'];
const URL_FIELDS = ['url', 'github', 'avatar'];
const MAX_DESC_LEN = 50;

for (let i = 0; i < currentLinks.length; i++) {
  const link = currentLinks[i];
  const prefix = `friendLinks[${i}] (${link.name || '未知'})`;

  // 必填字段
  for (const field of REQUIRED_FIELDS) {
    if (!link[field] || typeof link[field] !== 'string' || !link[field].trim()) {
      fail(`${prefix}: 缺少必填字段 "${field}" 或值为空`);
    }
  }

  // URL 格式
  for (const field of URL_FIELDS) {
    if (link[field] && !/^https?:\/\/.+/.test(link[field])) {
      fail(`${prefix}: "${field}" 不是合法的 URL -> ${link[field]}`);
    }
  }

  // GitHub 链接格式
  if (link.github && !/^https:\/\/github\.com\/[\w-]+\/?$/.test(link.github)) {
    fail(`${prefix}: "github" 格式不正确，应为 https://github.com/用户名`);
  }

  // 描述长度
  if (link.description && link.description.length > MAX_DESC_LEN) {
    fail(`${prefix}: "description" 超过 ${MAX_DESC_LEN} 字 (当前 ${link.description.length} 字)`);
  }
}

/* ========== 4. 增量检查: 只能新增 / 修改自己的记录 ========== */
log.info('检查新增 / 修改权限...');

if (baseLinks.length > 0) {
  // 构建基准 map: github 链接 (小写) -> 记录
  const baseMap = new Map();
  for (const link of baseLinks) {
    if (link.github) {
      baseMap.set(link.github.toLowerCase().replace(/\/$/, ''), link);
    }
  }

  // 检查是否有删除
  for (const oldLink of baseLinks) {
    const key = oldLink.github?.toLowerCase().replace(/\/$/, '');
    const stillExists = currentLinks.some(
      (l) => l.github?.toLowerCase().replace(/\/$/, '') === key
    );
    if (!stillExists) {
      fail(`不允许删除已有友链: ${oldLink.name} (@${oldLink.owner})`);
    }
  }

  // 检查修改权限
  for (const newLink of currentLinks) {
    const key = newLink.github?.toLowerCase().replace(/\/$/, '');
    const oldLink = baseMap.get(key);

    if (oldLink) {
      // 已有记录 — 检查是否有修改
      const changed = REQUIRED_FIELDS.some((f) => oldLink[f] !== newLink[f]);
      if (changed && prAuthor) {
        // 从 github 链接提取用户名
        const linkOwner = key?.split('/').pop() || '';
        if (linkOwner !== prAuthor) {
          fail(
            `不允许修改他人的友链: ${oldLink.name} (链接所有者: ${linkOwner}, PR 提交人: ${prAuthor})`
          );
        }
      }
    }
    // 新增记录 — 下一步检查 github 与 PR 提交人一致性
  }
}

/* ========== 5. GitHub 链接与 PR 提交人一致性 ========== */
log.info('检查 GitHub 链接与提交人一致性...');

if (prAuthor) {
  // 找出新增的记录
  const baseGithubs = new Set(
    baseLinks.map((l) => l.github?.toLowerCase().replace(/\/$/, ''))
  );

  for (const link of currentLinks) {
    const key = link.github?.toLowerCase().replace(/\/$/, '');
    if (!baseGithubs.has(key)) {
      // 新增记录 — github 链接必须与 PR 提交人一致
      const linkOwner = key?.split('/').pop() || '';
      if (linkOwner !== prAuthor) {
        fail(
          `新增友链的 github 字段必须与 PR 提交人一致: github 用户 "${linkOwner}" ≠ PR 提交人 "${prAuthor}"`
        );
      }
    }
  }
} else {
  log.warn('未设置 PR_AUTHOR 环境变量，跳过提交人一致性检查 (本地测试模式)');
}

/* ========== 6. TypeScript 编译检查 ========== */
log.info('运行 TypeScript 类型检查...');

try {
  execSync('npx tsc --noEmit', { cwd: ROOT, encoding: 'utf-8', stdio: 'pipe' });
  log.ok('TypeScript 编译通过');
} catch (e) {
  fail(`TypeScript 编译失败:\n${e.stdout || e.stderr || e.message}`);
}

/* ========== 7. 链接可达性检查 (HTTP 200) ========== */
log.info('检查所有链接的可达性 (HTTP 200)...');

/**
 * 检测 URL 是否可达
 * 带重试机制，最多重试 2 次
 */
async function checkUrl(url, retries = 2) {
  for (let attempt = 0; attempt <= retries; attempt++) {
    try {
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 15000);

      const resp = await fetch(url, {
        method: 'HEAD',
        signal: controller.signal,
        redirect: 'follow',
        headers: {
          'User-Agent': 'Mozilla/5.0 (FriendLink-Bot/1.0)',
        },
      });
      clearTimeout(timeout);

      if (resp.ok) return { url, ok: true };

      // 某些站点不支持 HEAD，用 GET 再试
      if (resp.status === 405 || resp.status === 403) {
        const controller2 = new AbortController();
        const timeout2 = setTimeout(() => controller2.abort(), 15000);
        const resp2 = await fetch(url, {
          method: 'GET',
          signal: controller2.signal,
          redirect: 'follow',
          headers: {
            'User-Agent': 'Mozilla/5.0 (FriendLink-Bot/1.0)',
          },
        });
        clearTimeout(timeout2);
        if (resp2.ok) return { url, ok: true };
        return { url, ok: false, status: resp2.status };
      }

      return { url, ok: false, status: resp.status };
    } catch (e) {
      if (attempt === retries) {
        return { url, ok: false, error: e.message };
      }
      // 等一下再重试
      await new Promise((r) => setTimeout(r, 2000));
    }
  }
}

// 只检查新增 / 修改的记录中的链接
const baseGithubSet = new Set(
  baseLinks.map((l) => l.github?.toLowerCase().replace(/\/$/, ''))
);

const urlsToCheck = [];
for (const link of currentLinks) {
  const key = link.github?.toLowerCase().replace(/\/$/, '');
  const oldLink = baseLinks.find(
    (l) => l.github?.toLowerCase().replace(/\/$/, '') === key
  );

  // 新增记录: 检查所有链接
  if (!oldLink) {
    for (const field of URL_FIELDS) {
      if (link[field]) urlsToCheck.push({ url: link[field], name: link.name, field });
    }
    continue;
  }

  // 已有记录但有修改: 只检查变更的 URL 字段
  for (const field of URL_FIELDS) {
    if (link[field] && link[field] !== oldLink[field]) {
      urlsToCheck.push({ url: link[field], name: link.name, field });
    }
  }
}

if (urlsToCheck.length > 0) {
  // 去重
  const unique = [...new Map(urlsToCheck.map((u) => [u.url, u])).values()];
  log.info(`需要检查 ${unique.length} 个链接...`);

  const results = await Promise.all(unique.map((u) => checkUrl(u.url)));

  for (const r of results) {
    const info = unique.find((u) => u.url === r.url);
    if (r.ok) {
      log.ok(`${info.name}.${info.field}: ${r.url}`);
    } else {
      fail(
        `${info.name}.${info.field} 链接不可达: ${r.url} (${r.status ? `HTTP ${r.status}` : r.error})`
      );
    }
  }
} else {
  log.ok('无新增/变更链接需要检查');
}

/* ========== 输出结果 ========== */
console.log('\n' + '='.repeat(60));
if (errors.length > 0) {
  log.fail(`验证失败! 共 ${errors.length} 个错误:\n`);
  errors.forEach((e, i) => console.error(`  ${i + 1}. ${e}`));
  console.log('');
  process.exit(1);
} else {
  log.ok('所有验证通过! ✨');
  process.exit(0);
}
