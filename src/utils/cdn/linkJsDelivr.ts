/**
 * HXLoLi CDN 加速层 — 基于 hx-cdn-forge
 *
 * 3 个仓库:
 * - HXLoLi       → 博客静态资源 (branch ref)
 * - HXLoLi-ANiMe → 番剧数据 + 图片 (tag ref) + CF Worker: hxloli-anime.woa.qzz.io
 * - HXLoLi-Music → 音乐/ASS/字体 (tag ref, 大文件自动切片 + 预压缩) + CF Worker: hxloli-music.woa.qzz.io
 *
 * Music 下载策略 (reqByCDNAuto):
 * - ★ 有预压缩 (info-zip.yaml) → Range 并行下载 .gz + DecompressionStream 解压 (最优)
 * - 有预切片 (info.yaml) → split 并行
 * - 二进制文件 → Range 多节点分段并行
 * - 文本文件 → direct (CDN gzip)
 *
 * CDN 节点同步策略:
 * - 只有 mainEngine 执行测速, 其他 engine 设 autoTest: false
 * - 测速结果通过 selectNodeAll() 同步到所有 engine
 * - 节点选择持久化到 localStorage, 跨 Tab 通过 storage 事件同步
 * - 新页面加载时: 有持久化节点 → 直接用不测速; 没有 → mainEngine 测速
 */

import { ANIME_CDN_TAG } from '@site/data/animeCdnVersion';
import { LATEST_COMMIT_ID } from '@site/data/gitVersion';
import { MUSIC_CDN_TAG } from '@site/data/musicCdnVersion';
import { GITHUB_CONFIG } from '@site/src/config/github';

// SSR 安全: hx-cdn-forge 只在浏览器端加载
let ForgeEngine: any = null;
let createForgeConfig: any = null;
let CDN_NODE_PRESETS: any[] = [];

if (typeof window !== 'undefined') {
  try {
    const mod = require('hx-cdn-forge');
    ForgeEngine = mod.ForgeEngine;
    createForgeConfig = mod.createForgeConfig;
    CDN_NODE_PRESETS = mod.CDN_NODE_PRESETS || [];
  } catch {}
}

// ============================================================
// ForgeEngine 实例
// ============================================================

const CDN_STORAGE_KEY = 'hxloli-cdn-node';
const DEFAULT_NODE = 'jsd-mirror';

/** 检查 localStorage 是否已有保存的节点 */
function getSavedNodeId(): string | null {
  if (typeof window === 'undefined') return null;
  try { return localStorage.getItem(CDN_STORAGE_KEY); } catch { return null; }
}

const savedNodeId = getSavedNodeId();
// 有持久化节点 → 所有 engine 都不测速, 直接用已选节点
// 没有 → 只让 mainEngine 测速, 其余跟随
const shouldAutoTest = !savedNodeId;

/**
 * CF Worker 直连节点 — 静态文件从 Workers 直接提供
 *
 * 仅用于子仓库 (主站已部署在 km.woa.qzz.io):
 * - HXLoLi-ANiMe → hxloli-anime.woa.qzz.io
 * - HXLoLi-Music → hxloli-music.woa.qzz.io
 */
const cfWorkerDomainMap: Record<string, string> = {
  'HXLoLi-ANiMe': 'hxloli-anime.woa.qzz.io',
  'HXLoLi-Music': 'hxloli-music.woa.qzz.io',
};

function buildCfWorkerNode(repo: string) {
  const domain = cfWorkerDomainMap[repo];
  if (!domain) return null; // 主站不需要 CF Worker 节点
  return {
    id: 'cf-worker',
    name: 'CF Worker',
    baseUrl: `https://${domain}`,
    region: 'china' as const,
    buildUrl: (_ctx: any, filePath: string) => {
      const path = filePath.startsWith('/') ? filePath : `/${filePath}`;
      return `https://${domain}${path}`;
    },
    maxFileSize: -1, // Workers 响应体上限 100MB
    supportsRange: true,
    description: `Cloudflare Workers 直连 (${domain})`,
  };
}

/** 构建引擎的 CDN 节点列表: CF Worker 直连 (仅子仓库) + jsDelivr 节点 */
function buildNodes(repo: string) {
  const cfNode = buildCfWorkerNode(repo);
  const jsDelivrNodes = CDN_NODE_PRESETS.filter((n: any) => n.id !== 'cf-worker');
  return cfNode ? [cfNode, ...jsDelivrNodes] : jsDelivrNodes;
}

function createEngine(github: any, options?: any) {
  if (!ForgeEngine || !createForgeConfig) return null;
  return new ForgeEngine(createForgeConfig(github, {
    defaultNodeId: savedNodeId || DEFAULT_NODE,
    autoTest: false,   // 所有 engine 禁用自动测速, 由统一初始化控制
    nodes: buildNodes(github.repo),
    ...options,
  }));
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
  {
    splitStoragePath: 'static/cdn/all',               // 切片 (info.yaml)
    preCompressionStoragePath: 'static/cdn/gzip',      // 预压缩 (info-zip.yaml + .gz)
    mappingPrefix: 'static',
    turboMode: true,
    turboConcurrentCDNs: 3,
    // enablePreCompression 默认 true: 自动检测 info-zip.yaml
    // → 对文本文件走 Range 并行下载 .gz + DecompressionStream 解压 (最优策略)
  },
);

/** 所有引擎列表 (方便同步操作) */
const allEngines = [mainEngine, mainCommitEngine, animeEngine, musicEngine].filter(Boolean);

// ============================================================
// 初始化 — 统一测速, 跨页面同步

type CdnListener = () => void;
const _cdnListeners: Set<CdnListener> = new Set();

/** 注册测速完成回调 (UI 组件用) */
function onCdnReady(fn: CdnListener): () => void {
  _cdnListeners.add(fn);
  return () => { _cdnListeners.delete(fn); };
}

function _notifyListeners() {
  _cdnListeners.forEach(fn => { try { fn(); } catch {} });
}

let _initPromise: Promise<void> | null = null;

function ensureInit(): Promise<void> {
  if (_initPromise) return _initPromise;
  if (allEngines.length === 0) return Promise.resolve();

  _initPromise = (async () => {
    if (shouldAutoTest && mainEngine) {
      // 没有持久化节点 → mainEngine 测速, 选最优, 同步到所有 engine
      try {
        await mainEngine.testAndSelectBest((result: any) => {
          // 每个节点测速完成时通知 UI
          if (result.success) {
            const best = mainEngine.getCurrentNode();
            if (best) selectNodeAll(best.id);
          }
          _notifyListeners();
        });
        const best = mainEngine.getCurrentNode();
        if (best) selectNodeAll(best.id);
        _notifyListeners();
      } catch (e) {
        // 测速失败不阻塞初始化
        if (typeof console !== 'undefined') console.warn('[CDN] testAndSelectBest failed:', e);
      }
    }
    // 标记所有 engine 就绪
    for (const engine of allEngines) {
      try {
        if (!engine.isInitialized()) {
          await engine.initialize();
        }
      } catch {}
    }
    _notifyListeners();
  })();

  return _initPromise;
}

if (typeof window !== 'undefined') {
  ensureInit();

  // 跨 Tab 同步: 其他 Tab 切换了节点 → 本 Tab 跟随
  window.addEventListener('storage', (e: StorageEvent) => {
    if (e.key === CDN_STORAGE_KEY && e.newValue) {
      for (const engine of allEngines) {
        try { engine.selectNode(e.newValue); } catch {}
      }
    }
  });
}

/** 切换节点 — 同步到所有引擎 + 持久化 + 触发跨 Tab */
function selectNodeAll(nodeId: string) {
  for (const engine of allEngines) {
    try { engine.selectNode(nodeId); } catch {}
  }
  try { localStorage.setItem(CDN_STORAGE_KEY, nodeId); } catch {}
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

export type { DownloadProgress, DownloadResult } from 'hx-cdn-forge';

/**
 * 智能模式下载 — 自动选择最优策略:
 * 1. 有预切片 (info.yaml) → split 并行
 * 2. 有预压缩 (info-zip.yaml) → Range 并行下载 .gz + DecompressionStream 解压
 * 3. 二进制文件 → Range 多节点分段并行
 * 4. 文本文件 → direct (CDN gzip)
 *
 * 适用于 ASS 字幕、音频、字体等各类文件
 */
export async function reqMusicByCDN(
  path: string,
  onProgress?: (p: import('hx-cdn-forge').DownloadProgress) => void,
) {
  await ensureInit();
  if (!musicEngine) throw new Error('musicEngine not available');
  return musicEngine.reqByCDNAuto(path, onProgress);
}

/**
 * 多 CDN 竞速下载 — 同一文件从所有节点并发请求, 最快的赢
 * 适用于字体等较小非切片文件
 */
export async function reqMusicRace(
  path: string,
): Promise<import('hx-cdn-forge').DownloadResult> {
  await ensureInit();
  if (!musicEngine) throw new Error('musicEngine not available');
  return musicEngine.reqByCDNRace(path);
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

export { allEngines, animeEngine, ensureInit, mainEngine, musicEngine, onCdnReady, selectNodeAll };
