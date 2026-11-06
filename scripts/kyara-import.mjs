#!/usr/bin/env node
/**
 * kyara 命令行导入: 复用插件的解析器 (pixiv / x / 直链) 与命名规则, 直接落盘到 static/img/kyara/{name}/.
 *
 *   node scripts/kyara-import.mjs <name> <url...>     逐条解析 → 下载 → 按下一个序号写入 NNN-标题@作者.ext
 *   node scripts/kyara-import.mjs --list <name>       列出该角色当前的图片 (按渲染顺序)
 *   node scripts/kyara-import.mjs --selftest          命名规则与链接匹配的自测, 全过退出码 0, 不联网
 *
 * pixiv 原图必须带 Referer, 浏览器里拿不到, 只能走这里或 dev 下的 POST /__hx/kyara/{name}/import.
 */

import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { importKyaraUrl, scanKyaraCharacter } from '../plugins/kyara-gallery-plugin.mjs';
import { formatKyaraFilename, parseKyaraFilename } from '../src/utils/kyara/filename.ts';
import { findKyaraResolver } from '../src/utils/kyara/resolvers.ts';

const siteDir = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
// 本地落盘只关心文件, src 前缀用 GitHub Pages 的 baseUrl; 页面上的 src 由插件按实际 baseUrl 生成
const BASE_URL = process.env.DEPLOY_TARGET === 'cloudflare' ? '/' : '/HXLoLi/';

/**
 * --selftest: 冻结的命名规则用例 + 解析器链接匹配用例, 任何一条不过退出码 1.
 * .agents/notes/implemented/feature/2026-11-06-anime-kyara-gallery.md
 */
function selftest() {
  let fail = 0;
  const eq = (label, got, want) => {
    const ok = JSON.stringify(got) === JSON.stringify(want);
    if (!ok) fail++;
    console.log(`${ok ? 'ok  ' : 'FAIL'} ${label}${ok ? '' : `\n     got  ${JSON.stringify(got)}\n     want ${JSON.stringify(want)}`}`);
  };
  const pick = (r) => (r ? { seq: r.seq, title: r.title, author: r.author } : r);

  eq('parse 001-春日@ふーみ.png', pick(parseKyaraFilename('001-春日@ふーみ.png')), { seq: 1, title: '春日', author: 'ふーみ' });
  eq('parse 010-a-b@x@y.webp (按最后一个 @ 拆)', pick(parseKyaraFilename('010-a-b@x@y.webp')), { seq: 10, title: 'a-b@x', author: 'y' });
  eq('parse abc.png → null', parseKyaraFilename('abc.png'), null);
  eq('parse 0001-无作者.JPG', pick(parseKyaraFilename('0001-无作者.JPG')), { seq: 1, title: '无作者', author: null });
  eq('parse 01-太短.png → null', parseKyaraFilename('01-太短.png'), null);
  eq('parse 001-x.bmp → null', parseKyaraFilename('001-x.bmp'), null);
  eq('format (2,"a/b",null,"png")', formatKyaraFilename(2, 'a/b', null, 'png'), '002-a／b.png');
  eq('format 作者 @ 前缀去掉, 标题里的 @ 转全角', formatKyaraFilename(12, 'x@y: z', '@fumi', '.JPG'), '012-x＠y： z@fumi.jpg');
  eq('format → parse 往返', pick(parseKyaraFilename(formatKyaraFilename(7, '春日 *夜*', 'ふーみ', 'webp'))), { seq: 7, title: '春日 ＊夜＊', author: 'ふーみ' });

  const rid = (u) => findKyaraResolver(u)?.id ?? null;
  eq('pixiv artworks', rid('https://www.pixiv.net/artworks/123456789'), 'pixiv');
  eq('pixiv en/artworks', rid('https://www.pixiv.net/en/artworks/123456789'), 'pixiv');
  eq('pixiv /i/', rid('https://pixiv.net/i/123456789'), 'pixiv');
  eq('x.com status', rid('https://x.com/fumi_art/status/1850000000000000000'), 'x');
  eq('twitter.com status', rid('https://twitter.com/fumi_art/status/1850000000000000000?s=20'), 'x');
  eq('pbs.twimg.com 直链', rid('https://pbs.twimg.com/media/GabcDEF123.jpg'), 'direct');
  eq('图片直链', rid('https://example.com/a/001-春日@ふーみ.png'), 'direct');
  eq('非图片链接不匹配', rid('https://example.com/page.html'), null);
  eq('x 用户主页不匹配', rid('https://x.com/fumi_art'), null);
  eq('pixiv 用户页不匹配', rid('https://www.pixiv.net/users/123'), null);

  console.log(fail ? `\n${fail} 条失败` : '\n全部通过');
  return fail ? 1 : 0;
}

async function main(argv) {
  if (argv[0] === '--selftest') return selftest();
  if (argv[0] === '--list' && argv[1]) {
    const data = scanKyaraCharacter(siteDir, BASE_URL, argv[1], (m) => console.warn(m));
    if (!data) { console.error(`没有这个角色: static/img/kyara/${argv[1]}/`); return 1; }
    for (const i of data.images) console.log(`${String(i.seq).padStart(3, '0')}  ${i.title}${i.author ? `  @${i.author}` : ''}  ${i.w ? `${i.w}x${i.h}` : ''}`);
    return 0;
  }
  const [name, ...urls] = argv;
  if (!name || urls.length === 0 || name.startsWith('-')) {
    console.error('用法: node scripts/kyara-import.mjs <name> <url...> | --list <name> | --selftest');
    return 2;
  }
  let code = 0;
  for (const url of urls) {
    try {
      const imgs = await importKyaraUrl(siteDir, BASE_URL, name, url);
      for (const i of imgs) console.log(`+ static/img/kyara/${name}/${i.file}${i.w ? `  (${i.w}x${i.h})` : ''}`);
    } catch (e) {
      code = 1;
      console.error(`✗ ${url}: ${e instanceof Error ? e.message : e}`);
    }
  }
  return code;
}

process.exitCode = await main(process.argv.slice(2));
