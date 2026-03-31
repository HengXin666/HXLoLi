/**
 * HXLoLi CDN 加速层 — 基于 hx-cdn-forge
 *
 * 3 个仓库:
 * - HXLoLi       → 博客静态资源 (branch ref)
 * - HXLoLi-ANiMe → 番剧数据 + 图片 (tag ref)
 * - HXLoLi-Music → 音乐/ASS/字体 (tag ref, 大文件自动切片)
 */

import { LATEST_COMMIT_ID } from '@site/data/gitVersion';
import { ANIME_CDN_TAG } from '@site/data/animeCdnVersion';
import { MUSIC_CDN_TAG } from '@site/data/musicCdnVersion';
import { GITHUB_CONFIG } from '@site/src/config/github';

// SSR 安全: hx-cdn-forge 只在浏览器端加载
let ForgeEngine: any = null;
let createForgeConfig: any = null;

if (typeof window !== 'undefined') {
  try {
    const mod = require('hx-cdn-forge');
    ForgeEngine = mod.ForgeEngine;
    createForgeConfig = mod.createForgeConfig;
  } catch {}
}

// ============================================================
// ForgeEngine 实例
// ============================================================

const DEFAULT_NODE = 'jsd-mirror';

function createEngine(github: any, options?: any) {
  if (!ForgeEngine || !createForgeConfig) return null;
  return new ForgeEngine(createForgeConfig(github, { defaultNodeId: DEFAULT_NODE, ...options }));
}

const mainEngine = createEngine({
  user: GITHUB_CONFIG.USER, repo: GITHUB_CONFIG.REPO, ref: GITHUB_CONFIG.BRANCH,
});

const mainCommitEngine = createEngine({
  user: GITHUB_CONFIG.USER, repo: GITHUB_CONFIG.REPO, ref: LATEST_COMMIT_ID,
});

const animeEngine = createEngine({
  user: GITHUB_CONFIG.USER, repo: 'HXLoLi-ANiMe', ref: ANIME_CDN_TAG,
});

const musicEngine = createEngine(
  { user: GITHUB_CONFIG.USER, repo: 'HXLoLi-Music', ref: MUSIC_CDN_TAG },
  { splitStoragePath: 'static/cdn', mappingPrefix: 'static', turboMode: true, turboConcurrentCDNs: 3 },
);

/** 所有引擎列表 (方便同步操作) */
const allEngines = [mainEngine, mainCommitEngine, animeEngine, musicEngine].filter(Boolean);

// ============================================================
// 初始化
// ============================================================

let _initPromise: Promise<void> | null = null;
function ensureInit(): Promise<void> {
  if (_initPromise) return _initPromise;
  if (allEngines.length === 0) return Promise.resolve();
  _initPromise = Promise.all(
    allEngines.map((e) => e.initialize().catch(() => {})),
  ).then(() => {});
  return _initPromise;
}

if (typeof window !== 'undefined') {
  ensureInit();
}

/** 切换节点 — 同步到所有引擎 */
function selectNodeAll(nodeId: string) {
  for (const engine of allEngines) {
    try { engine.selectNode(nodeId); } catch {}
  }
  try { localStorage.setItem('hxloli-cdn-node', nodeId); } catch {}
}

// ============================================================
// URL 构建函数
// ============================================================

/** jsDelivr 直连 URL 构建 (SSR fallback) */
function directUrl(user: string, repo: string, ref: string, path: string): string {
  const p = path.startsWith('/') ? path : `/${path}`;
  return `https://cdn.jsdmirror.com/gh/${user}/${repo}@${ref}${p}`;
}

export function toJsDelivrUrl(relativePath: string): string {
  if (!relativePath) return '';
  if (mainEngine) return mainEngine.buildUrl(relativePath);
  return directUrl(GITHUB_CONFIG.USER, GITHUB_CONFIG.REPO, GITHUB_CONFIG.BRANCH, relativePath);
}

export function toJsDelivrLatestCommitUrl(relativePath: string): string {
  if (!relativePath) return '';
  if (mainCommitEngine) return mainCommitEngine.buildUrl(relativePath);
  return directUrl(GITHUB_CONFIG.USER, GITHUB_CONFIG.REPO, LATEST_COMMIT_ID, relativePath);
}

export function toAnimeCdnUrl(relativePath: string): string {
  if (!relativePath) return '';
  if (animeEngine) return animeEngine.buildUrl(relativePath);
  return directUrl(GITHUB_CONFIG.USER, 'HXLoLi-ANiMe', ANIME_CDN_TAG, relativePath);
}

export function toMusicCdnUrl(relativePath: string): string {
  if (!relativePath) return '';
  if (musicEngine) return musicEngine.buildUrl(relativePath);
  return directUrl(GITHUB_CONFIG.USER, 'HXLoLi-Music', MUSIC_CDN_TAG, relativePath);
}

// ============================================================
// 大文件透明请求
// ============================================================

export type { DownloadResult, DownloadProgress } from 'hx-cdn-forge';

export async function reqMusicByCDN(
  path: string,
  onProgress?: (p: import('hx-cdn-forge').DownloadProgress) => void,
) {
  await ensureInit();
  if (!musicEngine) throw new Error('musicEngine not available');
  return musicEngine.reqByCDN(path, onProgress);
}

export async function reqAnimeByCDN(
  path: string,
  onProgress?: (p: import('hx-cdn-forge').DownloadProgress) => void,
) {
  await ensureInit();
  if (!animeEngine) throw new Error('animeEngine not available');
  return animeEngine.reqByCDN(path, onProgress);
}

// ============================================================
// 本地开发服务器
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
// 导出
// ============================================================

export { mainEngine, animeEngine, musicEngine, allEngines, ensureInit, selectNodeAll };
