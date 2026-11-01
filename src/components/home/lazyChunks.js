/**
 * 首页懒加载入口: 每个函数对应一个按需下载的 webpack chunk.
 *
 * 为什么是 .js 而不是 .ts: 仓库 tsconfig 是 module/moduleResolution = nodenext,
 * 在 .ts/.tsx 里写 import() 会被 tsc 按 ESM 规则解析, 要求相对路径带扩展名 (TS2834/TS2835),
 * 且 @site/* 路径映射在 import() 里失效 (TS2307). 补 '.js' 扩展名能过 tsc,
 * 但 Docusaurus 3.7 的 webpack 没配 resolve.extensionAlias, 构建时会找不到 index.js.
 * 所以 import() 只出现在这个 .js 里 (tsconfig 未开 allowJs, tsc 不检查它),
 * 类型由同名 lazyChunks.d.ts 提供, 那里用的是按 CJS 解析的 import 类型, 不受上述限制.
 *
 * 番剧本人标注编辑器的 dev-only 裁切也在这个文件里, 决策记录:
 * .agents/notes/implemented/feature/2026-11-01-anime-local-overlay.md
 */
export const loadWordCountChart = () => import('../chart/WordCountChart');
export const loadWordStats = () => import('@site/data/wordStats');
export const loadWakaPulse = () => import('../chart/WakaTimeChart/Pulse');
export const loadWakaTimeChart = () => import('../chart/WakaTimeChart');
export const loadEcosystemPanel = () => import('./EcosystemPanel');
export const loadHomeAnime = () => import('./HomeAnime');

/**
 * 番剧本人标注编辑器, 只在 dev 下有. 条件必须是 process.env.NODE_ENV 常量比较:
 * webpack 把它替换成字面量后由 ConstPlugin 判死分支, 死分支里的 import() 不会被解析, 生产构建不产生这个 chunk,
 * 写入地址 (/__hx/anime-overlay/) 因此不进任何产物. 不要改成运行时变量或函数里再判断.
 * 本地标注的完整决策见文件头注释结尾那行引用.
 */
export const loadAnimeOverlayEditor = process.env.NODE_ENV === 'development'
    ? () => import('../anime/Tag/OverlayEditor')
    : null;
