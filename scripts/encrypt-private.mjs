#!/usr/bin/env node
/**
 * encrypt-private.mjs
 *
 * HXLoLi 私有页面 **混合加密** 脚本
 *
 * 安全架构:
 *   RSA-OAEP (2048-bit) + AES-256-GCM
 *
 *   1. 为每个页面生成一个 **随机 AES-256 密钥** (一次性)
 *   2. 用 AES-256-GCM 加密 Markdown 正文 (高效)
 *   3. 用 RSA-OAEP **公钥** 加密那个随机 AES 密钥 (安全)
 *   4. 将 (加密后的 AES 密钥 + AES 密文) 打包为 base64, 嵌入页面 HTML
 *
 *   解密时 (浏览器插件):
 *   1. 用 RSA **私钥** 解密出 AES 密钥
 *   2. 用 AES 密钥解密正文
 *
 * 密文格式 (base64 编码前):
 *   encKeyLen(2 bytes, big-endian) + encryptedAesKey(N bytes) + iv(12) + authTag(16) + ciphertext
 *
 * 优势:
 *   - 公钥可以公开放在仓库/CI 中, 泄露也无法解密
 *   - 每个页面的 AES 密钥独立随机, 互不关联
 *   - 只有持有 RSA 私钥的浏览器插件才能解密
 *   - 密文嵌入 DOM, 零网络请求
 *
 * 用法:
 *   # 部署模式 (CI 中使用, 需要 RSA 公钥)
 *   node scripts/encrypt-private.mjs --source ./private-pages-repo --mode deploy --pubkey <PEM文件路径>
 *
 *   # 本地开发模式: 直接拷贝明文到 docs/blog 目录
 *   node scripts/encrypt-private.mjs --source ./private-pages-repo --mode dev
 */

import { readFileSync, writeFileSync, mkdirSync, readdirSync, statSync, existsSync, copyFileSync, unlinkSync } from 'fs';
import { join, relative, dirname, basename } from 'path';
import { createCipheriv, randomBytes, publicEncrypt, constants } from 'crypto';

// ============ 配置 ============

/** 受保护页面的魔术标记 (供浏览器插件识别) */
const PROTECTED_MAGIC = 'HXLOLI_PROTECTED_V3';

// ============ 参数解析 ============

function parseArgs() {
  const args = process.argv.slice(2);
  const opts = {
    source: '',
    mode: 'deploy',       // deploy | dev
    docsOutput: './docs',
    blogOutput: './blog',
    pubkeyPath: '',       // RSA 公钥 PEM 文件路径
    pubkeyPem: '',        // RSA 公钥 PEM 字符串 (从环境变量)
  };

  for (let i = 0; i < args.length; i++) {
    switch (args[i]) {
      case '--source':
      case '-s':
        opts.source = args[++i];
        break;
      case '--mode':
      case '-m':
        opts.mode = args[++i];
        break;
      case '--docs-output':
        opts.docsOutput = args[++i];
        break;
      case '--blog-output':
        opts.blogOutput = args[++i];
        break;
      case '--pubkey':
      case '-p':
        opts.pubkeyPath = args[++i];
        break;
      case '--help':
      case '-h':
        console.log(`
用法: node scripts/encrypt-private.mjs [选项]

选项:
  --source, -s      私有仓库本地路径 (必需)
  --mode, -m        模式: deploy (加密嵌入) | dev (拷贝明文) (默认: deploy)
  --docs-output     docs 输出目录 (默认: ./docs)
  --blog-output     blog 输出目录 (默认: ./blog)
  --pubkey, -p      RSA 公钥 PEM 文件路径 (deploy 模式必需)
                    也可通过环境变量 HXLOLI_RSA_PUBLIC_KEY 设置 PEM 内容
  --help, -h        显示帮助

安全模型:
  CI 只需要 RSA 公钥 (用于加密) → 泄露也无法解密
  浏览器插件持有 RSA 私钥 (用于解密) → 只有博主才有

生成 RSA 密钥对:
  openssl genrsa -out hxloli_private.pem 2048
  openssl rsa -in hxloli_private.pem -pubout -out hxloli_public.pem

示例:
  # CI 部署
  node scripts/encrypt-private.mjs -s ./private-pages-repo -m deploy -p ./hxloli_public.pem

  # 通过环境变量传公钥
  HXLOLI_RSA_PUBLIC_KEY="$(cat hxloli_public.pem)" node scripts/encrypt-private.mjs -s ./private-pages-repo -m deploy

  # 本地开发 (不需要密钥)
  node scripts/encrypt-private.mjs -s ../HXLoLi-imouto -m dev
        `);
        process.exit(0);
    }
  }

  // 公钥也可以通过环境变量传入 (PEM 字符串)
  if (process.env.HXLOLI_RSA_PUBLIC_KEY) {
    opts.pubkeyPem = process.env.HXLOLI_RSA_PUBLIC_KEY;
  }

  if (!opts.source) {
    console.error('❌ 错误: 必须指定 --source 目录');
    process.exit(1);
  }

  if (opts.mode === 'deploy') {
    if (opts.pubkeyPath && existsSync(opts.pubkeyPath)) {
      opts.pubkeyPem = readFileSync(opts.pubkeyPath, 'utf8');
    }
    if (!opts.pubkeyPem) {
      console.error('❌ 错误: deploy 模式需要 RSA 公钥');
      console.error('   通过 --pubkey 指定 PEM 文件, 或设置 HXLOLI_RSA_PUBLIC_KEY 环境变量');
      console.error('');
      console.error('   生成密钥对:');
      console.error('     openssl genrsa -out hxloli_private.pem 2048');
      console.error('     openssl rsa -in hxloli_private.pem -pubout -out hxloli_public.pem');
      process.exit(1);
    }
  }

  return opts;
}

// ============ RSA-OAEP + AES-256-GCM 混合加密 ============

/**
 * 混合加密 Markdown 正文
 *
 * 流程:
 *   1. 生成随机 AES-256 密钥 (32 bytes)
 *   2. AES-256-GCM 加密正文
 *   3. RSA-OAEP 公钥加密 AES 密钥
 *   4. 打包为: encKeyLen(2) + encryptedAesKey(N) + iv(12) + authTag(16) + ciphertext
 *
 * @param {string} plaintext - 明文 (Markdown 正文)
 * @param {string} rsaPublicPem - RSA 公钥 (PEM 格式)
 * @returns {string} base64 编码的混合密文
 */
function hybridEncrypt(plaintext, rsaPublicPem) {
  // 1. 生成随机 AES-256 密钥
  const aesKey = randomBytes(32); // 256 bits

  // 2. AES-256-GCM 加密正文
  const iv = randomBytes(12); // GCM 推荐 12 bytes IV
  const cipher = createCipheriv('aes-256-gcm', aesKey, iv);
  const encrypted = Buffer.concat([
    cipher.update(plaintext, 'utf8'),
    cipher.final(),
  ]);
  const authTag = cipher.getAuthTag(); // 16 bytes

  // 3. RSA-OAEP 公钥加密 AES 密钥
  const encryptedAesKey = publicEncrypt(
    {
      key: rsaPublicPem,
      padding: constants.RSA_PKCS1_OAEP_PADDING,
      oaepHash: 'sha256',
    },
    aesKey
  );

  // 4. 打包: encKeyLen(2, big-endian) + encryptedAesKey + iv(12) + authTag(16) + ciphertext
  const encKeyLen = Buffer.alloc(2);
  encKeyLen.writeUInt16BE(encryptedAesKey.length);

  const packed = Buffer.concat([encKeyLen, encryptedAesKey, iv, authTag, encrypted]);
  return packed.toString('base64');
}

// ============ Frontmatter 解析 ============

/**
 * 从 Markdown 文件的 frontmatter 中提取元信息
 */
function parseFrontmatter(content) {
  const match = content.match(/^---\n([\s\S]*?)\n---\n?([\s\S]*)$/);
  if (!match) {
    return { frontmatter: {}, rawFrontmatter: '', body: content };
  }

  const rawFrontmatter = match[1];
  const body = match[2];

  // 简易 YAML 解析 (足够处理 frontmatter)
  const frontmatter = {};
  let currentKey = null;

  for (const line of rawFrontmatter.split('\n')) {
    const kvMatch = line.match(/^(\w[\w_]*)\s*:\s*(.*)$/);
    if (kvMatch) {
      currentKey = kvMatch[1];
      const val = kvMatch[2].trim();
      if (val === '') {
        frontmatter[currentKey] = val;
      } else {
        frontmatter[currentKey] = val.replace(/^['"]|['"]$/g, '');
      }
    } else if (line.match(/^\s+-\s+(.+)$/) && currentKey) {
      const item = line.match(/^\s+-\s+(.+)$/)[1].trim();
      if (!Array.isArray(frontmatter[currentKey])) {
        frontmatter[currentKey] = [];
      }
      frontmatter[currentKey].push(item);
    }
  }

  return { frontmatter, rawFrontmatter, body };
}

/**
 * 将 frontmatter 对象序列化为 YAML 字符串
 */
function serializeFrontmatter(fm) {
  const lines = [];
  for (const [key, value] of Object.entries(fm)) {
    if (Array.isArray(value)) {
      lines.push(`${key}:`);
      for (const item of value) {
        lines.push(`    - ${item}`);
      }
    } else if (value === true) {
      lines.push(`${key}: true`);
    } else if (value === false) {
      lines.push(`${key}: false`);
    } else {
      lines.push(`${key}: ${value}`);
    }
  }
  return lines.join('\n');
}

// ============ 文件遍历 ============

function getAllFiles(dir, filter = null) {
  const results = [];
  if (!existsSync(dir)) return results;

  const items = readdirSync(dir);
  for (const item of items) {
    if (item.startsWith('.')) continue;
    const fullPath = join(dir, item);
    const stat = statSync(fullPath);
    if (stat.isDirectory()) {
      results.push(...getAllFiles(fullPath, filter));
    } else {
      if (!filter || filter(item)) {
        results.push(fullPath);
      }
    }
  }
  return results;
}

function copyDirRecursive(src, dest) {
  if (!existsSync(src)) return;
  mkdirSync(dest, { recursive: true });

  const items = readdirSync(src);
  for (const item of items) {
    if (item.startsWith('.')) continue;
    const srcPath = join(src, item);
    const destPath = join(dest, item);
    const stat = statSync(srcPath);

    if (stat.isDirectory()) {
      copyDirRecursive(srcPath, destPath);
    } else {
      copyFileSync(srcPath, destPath);
    }
  }
}

// ============ 占位页面生成 (deploy 模式, 含密文) ============

/**
 * 生成 docs 类型的加密占位 MDX 页面
 */
function generateDocsPlaceholder({ frontmatter, body, rsaPublicPem }) {
  const title = frontmatter.title || '受保护页面';
  const ciphertext = hybridEncrypt(body, rsaPublicPem);

  const newFm = {
    title: `"${title}"`,
    hx_protected: true,
  };

  if (frontmatter.sidebar_label) {
    newFm.sidebar_label = `"${frontmatter.sidebar_label}"`;
  }

  return `---
${serializeFrontmatter(newFm)}
---

import ProtectedPage from '@site/src/components/ProtectedPage';

<ProtectedPage
  magic="${PROTECTED_MAGIC}"
  title="${title.replace(/"/g, '\\"')}"
  cipher="${ciphertext}"
/>
`;
}

/**
 * 生成 blog 类型的加密占位 MDX 页面
 */
function generateBlogPlaceholder({ frontmatter, rawFrontmatter, body, rsaPublicPem }) {
  const title = frontmatter.title || '受保护博客';
  const ciphertext = hybridEncrypt(body, rsaPublicPem);

  let newRawFm = rawFrontmatter;
  if (!newRawFm.includes('hx_protected')) {
    newRawFm += '\nhx_protected: true';
  }

  return `---
${newRawFm}
---

import ProtectedPage from '@site/src/components/ProtectedPage';

此博客内容受保护 🔒

{/* truncate */}

<ProtectedPage
  magic="${PROTECTED_MAGIC}"
  title="${title.replace(/"/g, '\\"')}"
  cipher="${ciphertext}"
/>
`;
}

// ============ 主流程 ============

function main() {
  const opts = parseArgs();

  console.log('🔐 HXLoLi 私有页面加密系统 v3 (RSA-OAEP + AES-256-GCM)');
  console.log('━'.repeat(60));
  console.log(`📂 源目录: ${opts.source}`);
  console.log(`🔧 模式: ${opts.mode}`);
  if (opts.mode === 'deploy') {
    console.log(`🔑 加密方式: RSA-OAEP(SHA-256) + AES-256-GCM 混合加密`);
    console.log(`🔓 公钥: ${opts.pubkeyPath || '(环境变量)'} (公钥可公开, 不影响安全)`);
  }
  console.log('━'.repeat(60));

  const docsSource = join(opts.source, 'docs');
  const blogSource = join(opts.source, 'blog');
  const hasDocsSrc = existsSync(docsSource);
  const hasBlogSrc = existsSync(blogSource);

  if (!hasDocsSrc && !hasBlogSrc) {
    console.log('⚠️  私有仓库中没有找到 docs/ 或 blog/ 目录');
    console.log('   期望的目录结构:');
    console.log('   private-repo/');
    console.log('   ├── docs/     (笔记页面)');
    console.log('   └── blog/     (博客文章)');
    return;
  }

  // ========== 开发模式: 直接拷贝明文 ==========
  if (opts.mode === 'dev') {
    console.log('\n📋 开发模式: 直接拷贝明文文件\n');

    if (hasDocsSrc) {
      console.log('📁 拷贝 docs/ ...');
      copyDirRecursive(docsSource, opts.docsOutput);
      const docFiles = getAllFiles(docsSource, f => /\.(md|mdx)$/i.test(f));
      console.log(`   ✅ ${docFiles.length} 个 Markdown 文件已拷贝到 ${opts.docsOutput}/`);
    }

    if (hasBlogSrc) {
      console.log('📁 拷贝 blog/ ...');
      copyDirRecursive(blogSource, opts.blogOutput);
      const blogFiles = getAllFiles(blogSource, f => /\.(md|mdx)$/i.test(f));
      console.log(`   ✅ ${blogFiles.length} 个 Markdown 文件已拷贝到 ${opts.blogOutput}/`);
    }

    console.log('\n🎉 开发模式: 所有文件已拷贝, 运行 npm start 即可预览');
    return;
  }

  // ========== 部署模式: RSA + AES 混合加密 ==========
  console.log('\n📋 部署模式: RSA-OAEP + AES-256-GCM 混合加密\n');

  let docsCount = 0;
  let blogCount = 0;
  let nonMdCount = 0;

  // --- 处理 docs ---
  if (hasDocsSrc) {
    console.log('📁 处理 docs/ ...');

    const allFiles = getAllFiles(docsSource);

    for (const filePath of allFiles) {
      const relPath = relative(docsSource, filePath);
      const outPath = join(opts.docsOutput, relPath);

      if (/\.(md|mdx)$/i.test(filePath)) {
        const content = readFileSync(filePath, 'utf8');
        const { frontmatter, body } = parseFrontmatter(content);

        const mdxOutPath = outPath.replace(/\.md$/, '.mdx');
        mkdirSync(dirname(mdxOutPath), { recursive: true });

        const placeholder = generateDocsPlaceholder({
          frontmatter,
          body,
          rsaPublicPem: opts.pubkeyPem,
        });

        writeFileSync(mdxOutPath, placeholder, 'utf8');

        // 如果源文件是 .md, 转换后产生了 .mdx, 则删除目标目录中可能残留的 .md 明文
        if (mdxOutPath !== outPath && existsSync(outPath)) {
          unlinkSync(outPath);
          console.log(`  🗑️  docs: 已删除残留明文 ${basename(outPath)}`);
        }

        console.log(`  🔒 docs: ${relPath} → ${basename(mdxOutPath)} (${body.length} chars encrypted)`);
        docsCount++;
      } else {
        // 非 Markdown 文件 (tag.json, 图片等): 直接拷贝
        mkdirSync(dirname(outPath), { recursive: true });
        copyFileSync(filePath, outPath);
        nonMdCount++;
      }
    }
  }

  // --- 处理 blog ---
  if (hasBlogSrc) {
    console.log('📁 处理 blog/ ...');

    const allFiles = getAllFiles(blogSource);

    for (const filePath of allFiles) {
      const relPath = relative(blogSource, filePath);
      const outPath = join(opts.blogOutput, relPath);

      if (/\.(md|mdx)$/i.test(filePath)) {
        const content = readFileSync(filePath, 'utf8');
        const { frontmatter, rawFrontmatter, body } = parseFrontmatter(content);

        const mdxOutPath = outPath.replace(/\.md$/, '.mdx');
        mkdirSync(dirname(mdxOutPath), { recursive: true });

        const placeholder = generateBlogPlaceholder({
          frontmatter,
          rawFrontmatter,
          body,
          rsaPublicPem: opts.pubkeyPem,
        });

        writeFileSync(mdxOutPath, placeholder, 'utf8');

        // 如果源文件是 .md, 转换后产生了 .mdx, 则删除目标目录中可能残留的 .md 明文
        if (mdxOutPath !== outPath && existsSync(outPath)) {
          unlinkSync(outPath);
          console.log(`  🗑️  blog: 已删除残留明文 ${basename(outPath)}`);
        }

        console.log(`  🔒 blog: ${relPath} → ${basename(mdxOutPath)} (${body.length} chars encrypted)`);
        blogCount++;
      } else {
        mkdirSync(dirname(outPath), { recursive: true });
        copyFileSync(filePath, outPath);
        nonMdCount++;
      }
    }
  }

  // --- 生成清单 ---
  const manifestPath = join(opts.docsOutput, '..', 'static', 'protected-pages.json');
  try {
    mkdirSync(dirname(manifestPath), { recursive: true });

    const manifest = {
      magic: PROTECTED_MAGIC,
      version: 3,
      encryption: 'RSA-OAEP(SHA-256) + AES-256-GCM',
      note: '公钥加密, 只有持有 RSA 私钥的浏览器插件才能解密',
      generatedAt: new Date().toISOString(),
      stats: {
        docs: docsCount,
        blog: blogCount,
        assets: nonMdCount,
      }
    };

    writeFileSync(manifestPath, JSON.stringify(manifest, null, 2), 'utf8');
    console.log(`\n📋 清单已写入: ${manifestPath}`);
  } catch (e) {
    console.warn(`⚠️  无法写入清单文件: ${e.message}`);
  }

  console.log('\n' + '━'.repeat(60));
  console.log(`🎉 完成! docs: ${docsCount}, blog: ${blogCount}, 资源: ${nonMdCount}`);
  console.log(`   所有内容已使用 RSA-OAEP + AES-256-GCM 混合加密`);
  console.log(`   🔓 公钥加密 → 即使泄露也无法解密`);
  console.log(`   🔐 私钥解密 → 只有浏览器插件持有`);
}

main();
