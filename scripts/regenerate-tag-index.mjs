/* 重建 data/aiDocTags.ts
 *
 * 等价于构建期 tag-index-plugin 的 loadContent() 行为 (它在 dev 与 build 都会落盘).
 * 单独抽成脚本, 让 quality-gate 能在不跑完整构建的前提下校验该生成物是否新鲜.
 *
 * 运行: node scripts/regenerate-tag-index.mjs
 */
import tagIndexPlugin from '../plugins/tag-index-plugin.mjs';

const plugin = tagIndexPlugin({ siteDir: process.cwd() }, {});
await plugin.loadContent();
