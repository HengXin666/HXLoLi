/**
 * HXLoLi CDN 加速层 — 基于 hx-cdn-forge
 *
 * 3 个仓库:
 * - HXLoLi       → 博客静态资源 (branch ref)
 * - HXLoLi-ANiMe → 番剧数据 + 图片 (tag ref)
 * - HXLoLi-Music → 音乐/ASS/字体 (tag ref, 大文件自动切片)
 */

import { ForgeEngine, createForgeConfig } from 'hx-cdn-forge';
import { LATEST_COMMIT_ID } from '@site/data/gitVersion';
import { ANIME_CDN_TAG } from '@site/data/animeCdnVersion';
import { MUSIC_CDN_TAG } from '@site/data/musicCdnVersion';
import { GITHUB_CONFIG } from '@site/src/config/github';

// ============================================================
// ForgeEngine 实例 (每个仓库一个)
// ============================================================

/** HXLoLi 主仓库 — 博客图片等静态资源 */
const mainEngine = new ForgeEngine(createForgeConfig({
  user: GITHUB_CONFIG.USER,
  repo: GITHUB_CONFIG.REPO,
  ref: GITHUB_CONFIG.BRANCH,
}));

/** HXLoLi 主仓库 (commit ref) — 精确缓存 */
const mainCommitEngine = new ForgeEngine(createForgeConfig({
  user: GITHUB_CONFIG.USER,
  repo: GITHUB_CONFIG.REPO,
  ref: LATEST_COMMIT_ID,
}));

/** HXLoLi-ANiMe — 番剧数据 + 图片 */
const animeEngine = new ForgeEngine(createForgeConfig({
  user: GITHUB_CONFIG.USER,
  repo: 'HXLoLi-ANiMe',
  ref: ANIME_CDN_TAG,
}));

/** HXLoLi-Music — 音乐/ASS/字体 (带切片支持) */
const musicEngine = new ForgeEngine(createForgeConfig(
  {
    user: GITHUB_CONFIG.USER,
    repo: 'HXLoLi-Music',
    ref: MUSIC_CDN_TAG,
  },
  {
    splitStoragePath: 'static/cdn',
    mappingPrefix: 'static',
    turboMode: true,
    turboConcurrentCDNs: 3,
  },
));

// 初始化 (自动测速选最快节点)
let _initPromise: Promise<void> | null = null;
function ensureInit(): Promise<void> {
  if (_initPromise) return _initPromise;
  _initPromise = (async () => {
    await Promise.all([
      mainEngine.initialize(),
      animeEngine.initialize(),
      musicEngine.initialize(),
    ]);
    // mainCommitEngine 共享 mainEngine 的节点选择，不需要重复测速
  })();
  return _initPromise;
}

if (typeof window !== 'undefined') {
  ensureInit();
}

// ============================================================
// URL 构建函数 (兼容旧 API)
// ============================================================

/** HXLoLi 仓库资源 URL (branch ref — 图片等) */
export function toJsDelivrUrl(relativePath: string): string {
  if (!relativePath) return '';
  return mainEngine.buildUrl(relativePath);
}

/** HXLoLi 仓库资源 URL (commit ref — JSON 等精确缓存) */
export function toJsDelivrLatestCommitUrl(relativePath: string): string {
  if (!relativePath) return '';
  return mainCommitEngine.buildUrl(relativePath);
}

/** HXLoLi-ANiMe 仓库资源 URL */
export function toAnimeCdnUrl(relativePath: string): string {
  if (!relativePath) return '';
  return animeEngine.buildUrl(relativePath);
}

/** HXLoLi-Music 仓库资源 URL */
export function toMusicCdnUrl(relativePath: string): string {
  if (!relativePath) return '';
  return musicEngine.buildUrl(relativePath);
}

// ============================================================
// 大文件透明请求 (reqByCDN — 自动检测切片)
// ============================================================

export type { DownloadResult, DownloadProgress } from 'hx-cdn-forge';

/** Music 仓库大文件透明请求 (ASS/字体等, 自动切片并行下载) */
export async function reqMusicByCDN(
  path: string,
  onProgress?: (p: import('hx-cdn-forge').DownloadProgress) => void,
) {
  await ensureInit();
  return musicEngine.reqByCDN(path, onProgress);
}

/** ANiMe 仓库大文件透明请求 (JSON 等) */
export async function reqAnimeByCDN(
  path: string,
  onProgress?: (p: import('hx-cdn-forge').DownloadProgress) => void,
) {
  await ensureInit();
  return animeEngine.reqByCDN(path, onProgress);
}

// ============================================================
// 本地开发服务器 (保持兼容)
// ============================================================

export const LOCAL_MUSIC_SERVER = 'http://localhost:9527';

export function isLocalDev(): boolean {
  if (typeof window === 'undefined') return false;
  const host = window.location.hostname;
  return host === 'localhost' || host === '127.0.0.1';
}

export function toMusicLocalUrl(relativePath: string): string {
  if (!relativePath) return '';
  const cleanPath = relativePath.startsWith('/') ? relativePath : `/${relativePath}`;
  return `${LOCAL_MUSIC_SERVER}${cleanPath}`;
}

// ============================================================
// 导出引擎实例 (供高级使用)
// ============================================================

export { mainEngine, animeEngine, musicEngine, ensureInit };
