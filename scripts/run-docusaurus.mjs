#!/usr/bin/env node
/**
 * Docusaurus 启动器: 在加载 Docusaurus 之前把 WEBPACK_URL_LOADER_LIMIT 设为 0。
 *
 * 为什么必须有这一层包装 (而不是写在 docusaurus.config.ts 里):
 *   @docusaurus/utils 的 constants.js 在模块被 require 的那一刻就把该值求值成常量
 *   (constants.js:79 `process.env.WEBPACK_URL_LOADER_LIMIT ?? 10000`), 而
 *   @docusaurus/core/bin/docusaurus.mjs 顶部第一件事就是 import @docusaurus/utils。
 *   等到 docusaurus.config.ts 被执行时, 常量与 FileLoaderUtilsMap 早已固化, 在那里改
 *   process.env 对本次进程无效 (实测: 仍是 limit=10000)。所以注入必须发生在
 *   加载任何 @docusaurus/* 之前 —— 也就是这个独立进程里。
 *
 * 为什么要把阈值关成 0:
 *   见 .agents/notes/implemented/bug-fix/2026-09-27-drawio-svg-inlined-loses-editor-shell.md
 *   小于阈值的 markdown 图片会被 url-loader 内联成 data URI, 于是 src 不再以 .svg 结尾,
 *   src/theme/MDXComponents/Img 的 draw.io 分支判定失败, 图退化成裸 <img>。
 *
 * 用法: 由 package.json 的 start / build / dev:private 调用, 参数原样透传。
 *
 * 引用落点: src/theme/MDXComponents/Img/index.tsx 的 draw.io 分支判定
 * (see .agents/notes/implemented/bug-fix/2026-09-27-drawio-svg-inlined-loses-editor-shell.md
 *  — 内联阈值必须为 0 的实现处)
 */
import { spawn } from 'node:child_process';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);

process.env.WEBPACK_URL_LOADER_LIMIT = '0';

let docusaurusBin;
try {
  docusaurusBin = require.resolve('@docusaurus/core/bin/docusaurus.mjs');
} catch {
  console.error('[hx-docusaurus] 找不到 @docusaurus/core/bin/docusaurus.mjs, 请先 npm install');
  process.exit(1);
}

const child = spawn(process.execPath, [docusaurusBin, ...process.argv.slice(2)], {
  stdio: 'inherit',
  env: process.env,
});

// 把终止信号转给子进程, 让 Ctrl-C / CI 取消能正常结束 webpack 与 dev server。
for (const signal of ['SIGINT', 'SIGTERM', 'SIGHUP']) {
  process.on(signal, () => {
    if (!child.killed) child.kill(signal);
  });
}

child.on('exit', (code, signal) => {
  if (signal) {
    process.kill(process.pid, signal);
    return;
  }
  process.exit(code ?? 0);
});

child.on('error', (error) => {
  console.error('[hx-docusaurus] 启动失败: ' + error.message);
  process.exit(1);
});
