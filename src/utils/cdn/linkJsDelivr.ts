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
 * - 所有 engine 设 autoTest: false, 测速统一由 ensureInit 用 probeEngine (节点全集) 执行, 结果写进同一个 store
 * - 测速结果通过 selectNodeAll() 同步到所有 engine
 * - 节点选择持久化到 localStorage, 跨 Tab 通过 storage 事件同步; 另存来源 (auto = 自动优选, manual = 用户在面板手选)
 * - 冷加载 (整页刷新 / 新开标签, 模块加载即执行, 不等交互):
 *   先把持久化节点 (没有则默认节点) 应用到所有引擎, ensureInit 立即完成, 资源请求不等测速;
 *   同时后台全量测速, 测完: 来源 auto → 切到最优; 来源 manual → 保留手选, 只把最优记进快照 (bestId) 供面板提示
 * - SPA 站内跳转: 模块不重新执行, 不重测、不切换
 * - 定时重测: 距上次测速超过 CDN_RETEST_INTERVAL_MS (15 分钟) 且标签页在前台时重测一轮 (每分钟检查一次 + 回到前台时检查),
 *   规则同冷加载; 后台标签页不测
 * - 面板 "重新测速" (retestAllNodes) 是用户主动要求重选: 测完切到最优, 来源改回 auto
 * - 面板手选节点 (selectNodeAll 默认来源 manual): 之后刷新不被自动优选覆盖, 直到用户点 "重新测速"
 * - 旧版本只存了节点 id 没存来源, 按 auto 处理 (旧逻辑下首访也是自动测出来的)
 */

import { ANIME_CDN_TAG } from '@site/data/animeCdnVersion';
import { LATEST_COMMIT_ID } from '@site/data/gitVersion';
import { MUSIC_CDN_TAG } from '@site/data/musicCdnVersion';
import { GITHUB_CONFIG } from '@site/src/config/github';

// SSR 安全: hx-cdn-forge 只在浏览器端加载
let ForgeEngine: any = null;
let createForgeConfig: any = null;
let DEFAULT_GITHUB_CDN_NODES: any[] = [];

if (typeof window !== 'undefined') {
  try {
    const mod = require('hx-cdn-forge');
    ForgeEngine = mod.ForgeEngine;
    createForgeConfig = mod.createForgeConfig;
    DEFAULT_GITHUB_CDN_NODES = mod.DEFAULT_GITHUB_CDN_NODES || [];
  } catch {}
}

// ============================================================
// ForgeEngine 实例
// ============================================================

const CDN_STORAGE_KEY = 'hxloli-cdn-node';
/** 节点来源: 'auto' | 'manual'; 缺省视为 auto */
const CDN_SOURCE_KEY = 'hxloli-cdn-node-source';
const DEFAULT_NODE = 'jsd-mirror';
/** 定时重新优选的间隔 */
const CDN_RETEST_INTERVAL_MS = 15 * 60 * 1000;
/** 检查是否到期的节拍 */
const CDN_RETEST_TICK_MS = 60 * 1000;

export type CdnNodeSource = 'auto' | 'manual';

/** 检查 localStorage 是否已有保存的节点 
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
function getSavedNodeId(): string | null {
  if (typeof window === 'undefined') return null;
  try { return localStorage.getItem(CDN_STORAGE_KEY); } catch { return null; }
}

const savedNodeId = getSavedNodeId();

function getSavedSource(): CdnNodeSource {
  if (typeof window === 'undefined') return 'auto';
  try { return localStorage.getItem(CDN_SOURCE_KEY) === 'manual' ? 'manual' : 'auto'; } catch { return 'auto'; }
}

let _source: CdnNodeSource = savedNodeId ? getSavedSource() : 'auto';

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
  const jsDelivrNodes = DEFAULT_GITHUB_CDN_NODES.filter((n: any) => n.id !== 'cf-worker');
  return cfNode ? [cfNode, ...jsDelivrNodes] : jsDelivrNodes;
}

function createEngine(github: any, options?: any) {
  if (!ForgeEngine || !createForgeConfig) return null;
  return new ForgeEngine(createForgeConfig(github, {
    defaultNodeId: savedNodeId || DEFAULT_NODE,
    autoTest: false,   // 所有 engine 禁用内置测速, 由 ensureInit 统一探测
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

/**
 * 测速与展示用同一个引擎: musicEngine 的节点表是全集 (CF Worker + 全部 jsDelivr),
 * mainEngine 只有 jsDelivr. 以前测速跑在 mainEngine, UI 却读 musicEngine 的延迟表,
 * 两张表不是同一个对象, 于是首屏永远读到空表.
 */
const probeEngine: any = musicEngine || mainEngine;

// ============================================================
// CDN 状态 — 单一来源, 供 Navbar 按钮与面板订阅 (useSyncExternalStore)
// ============================================================

export type CdnPhase = 'idle' | 'testing' | 'ready';

export interface CdnNodeView {
  id: string;
  name: string;
  region: string;
  /** null = 未测; -1 = 超时/失败 */
  latency: number | null;
  testing: boolean;
}

export interface CdnSnapshot {
  phase: CdnPhase;
  currentId: string | null;
  nodes: CdnNodeView[];
  /** 上一轮测速的最优节点; 来源 manual 时可能 != currentId, 面板据此提示 "建议切换到 X" */
  bestId: string | null;
  /** 当前节点来源 */
  source: CdnNodeSource;
  /** 上一轮全量测速完成时间 (ms), 未测过为 null */
  lastTestedAt: number | null;
}

/** SSR 与水合首帧共用的固定快照, 保证服务端与客户端首帧一致 */
const SERVER_SNAPSHOT: CdnSnapshot = { phase: 'idle', currentId: null, nodes: [], bestId: null, source: 'auto', lastTestedAt: null };

const _latency = new Map<string, number>();
const _testingIds = new Set<string>();
let _phase: CdnPhase = 'idle';
let _bestId: string | null = null;
let _lastTestedAt: number | null = null;
let _snapshot: CdnSnapshot = SERVER_SNAPSHOT;

type CdnListener = () => void;
const _cdnListeners: Set<CdnListener> = new Set();

function _rebuildSnapshot() {
  if (!probeEngine) { _snapshot = SERVER_SNAPSHOT; return; }
  const current = probeEngine.getCurrentNode();
  _snapshot = {
    phase: _phase,
    currentId: current?.id ?? null,
    nodes: probeEngine.getNodes().map((n: any) => ({
      id: n.id,
      name: n.name,
      region: n.region ?? 'global',
      latency: _latency.has(n.id) ? (_latency.get(n.id) as number) : null,
      testing: _testingIds.has(n.id),
    })),
    bestId: _bestId,
    source: _source,
    lastTestedAt: _lastTestedAt,
  };
}

function _notifyListeners() {
  _rebuildSnapshot();
  _cdnListeners.forEach(fn => { try { fn(); } catch {} });
}

/** 订阅 CDN 状态变化 (也兼容旧的 "测速完成回调" 用法) */
function onCdnReady(fn: CdnListener): () => void {
  _cdnListeners.add(fn);
  return () => { _cdnListeners.delete(fn); };
}

export function getCdnSnapshot(): CdnSnapshot {
  return _snapshot;
}

export function getCdnServerSnapshot(): CdnSnapshot {
  return SERVER_SNAPSHOT;
}

/** 单节点 HEAD 探测, 与 hx-cdn-forge 内置测速同口径 (no-cors, 5s 超时) */
async function probeNode(node: any): Promise<number> {
  const ctrl = new AbortController();
  const tid = setTimeout(() => ctrl.abort(), 5000);
  try {
    const start = performance.now();
    await fetch(node.baseUrl, { method: 'HEAD', mode: 'no-cors', cache: 'no-cache', signal: ctrl.signal });
    return Math.round(performance.now() - start);
  } catch {
    return -1;
  } finally {
    clearTimeout(tid);
  }
}

function _bestNodeId(engine: any): string | null {
  let best: string | null = null;
  let bestMs = Infinity;
  for (const n of engine.getNodes()) {
    const ms = _latency.get(n.id);
    if (ms != null && ms >= 0 && ms < bestMs) { best = n.id; bestMs = ms; }
  }
  return best;
}

/**
 * 一轮全量测速: 所有节点并发探测, 结果逐个推给订阅者 (面板能看到哪个在测、延迟逐个跳出),
 * 全部结束后才算最优. pick = true 无条件切到最优并把来源改回 auto (用户点 "重新测速");
 * pick = false 只在来源为 auto 时切换, manual 时保留手选、只记录 bestId.
 * .agents/notes/implemented/feature/2026-11-01-navbar-slim-and-cdn-first-load.md
 */
async function _runSpeedTest(pick: boolean): Promise<void> {
  if (!probeEngine || _phase === 'testing') return;
  const nodes = probeEngine.getNodes();
  _phase = 'testing';
  _latency.clear();
  nodes.forEach((n: any) => _testingIds.add(n.id));
  _notifyListeners();
  await Promise.all(nodes.map(async (n: any) => {
    const ms = await probeNode(n);
    _latency.set(n.id, ms);
    _testingIds.delete(n.id);
    _notifyListeners();
  }));
  _bestId = _bestNodeId(probeEngine);
  _lastTestedAt = Date.now();
  if (_bestId && (pick || _source === 'auto')) selectNodeAll(_bestId, 'auto');
  _phase = 'ready';
  _notifyListeners();
}

/** 面板 "重新测速": 用户主动重选, 测完切到最优 (覆盖手选) */
export async function retestAllNodes(): Promise<void> {
  return _runSpeedTest(true);
}

/** 自动优选 (冷加载 / 定时): 尊重手选 */
function _autoRetest(): Promise<void> {
  return _runSpeedTest(false);
}

// ============================================================
// 初始化 — 模块加载即在浏览器端执行, 不依赖任何交互
// ============================================================

let _initPromise: Promise<void> | null = null;

function ensureInit(): Promise<void> {
  if (_initPromise) return _initPromise;
  if (allEngines.length === 0) return Promise.resolve();

  _initPromise = (async () => {
    for (const engine of allEngines) {
      try {
        if (!engine.isInitialized()) await engine.initialize();
      } catch {}
    }
    // 先用上次的节点 (没有则默认节点) 保证首屏可用, 不写回存储, 来源不变
    _selectOnEngines(savedNodeId || DEFAULT_NODE);
    _notifyListeners();
    // 不 await: 资源请求 (reqMusicByCDN 等) 只等引擎初始化, 不等测速; 测完按来源决定是否切换
    void _autoRetest();
  })().catch((e) => {
    if (typeof console !== 'undefined') console.warn('[CDN] init failed:', e);
    _phase = 'ready';
    _notifyListeners();
  });

  return _initPromise;
}

/** 到期检查: 前台、未在测、距上次测速超过间隔 → 自动重测 */
function _retestIfStale() {
  if (typeof document === 'undefined' || document.visibilityState !== 'visible') return;
  if (_phase === 'testing' || _lastTestedAt == null) return;
  if (Date.now() - _lastTestedAt >= CDN_RETEST_INTERVAL_MS) void _autoRetest();
}

if (typeof window !== 'undefined') {
  ensureInit();

  // 定时器与监听只装一份: 模块在一次页面加载里只执行一次 (SPA 跳转不重跑);
  // 开发时 HMR 会重跑模块, 所以把句柄挂在 window 上, 重跑前先清掉旧的
  const w = window as any;
  if (w.__hxCdnRetestTimer) clearInterval(w.__hxCdnRetestTimer);
  if (w.__hxCdnVisibility) document.removeEventListener('visibilitychange', w.__hxCdnVisibility);
  w.__hxCdnRetestTimer = setInterval(_retestIfStale, CDN_RETEST_TICK_MS);
  w.__hxCdnVisibility = _retestIfStale;
  document.addEventListener('visibilitychange', _retestIfStale);

  // 跨 Tab 同步: 其他 Tab 切换了节点或来源 → 本 Tab 跟随
  window.addEventListener('storage', (e: StorageEvent) => {
    if (e.key === CDN_STORAGE_KEY && e.newValue) {
      _selectOnEngines(e.newValue);
      _notifyListeners();
    } else if (e.key === CDN_SOURCE_KEY) {
      _source = e.newValue === 'manual' ? 'manual' : 'auto';
      _notifyListeners();
    }
  });
}

/**
 * 把节点同步到每个引擎. 节点只存在于部分引擎时 (cf-worker 只在子仓库引擎里),
 * 缺它的引擎改用自己节点表里测得最快的, 再不行用默认节点, 不再静默失败.
 */
function _selectOnEngines(nodeId: string) {
  for (const engine of allEngines) {
    try {
      const has = engine.getNodes().some((n: any) => n.id === nodeId);
      const target = has ? nodeId : (_bestNodeId(engine) ?? DEFAULT_NODE);
      engine.selectNode(target);
    } catch {}
  }
}

/**
 * 切换节点 — 同步到所有引擎 + 持久化 (节点与来源) + 触发跨 Tab.
 * 默认来源 manual: 面板里点节点直接调这个, 不用改调用方; 自动优选显式传 'auto'.
 */
function selectNodeAll(nodeId: string, source: CdnNodeSource = 'manual') {
  _selectOnEngines(nodeId);
  _source = source;
  try {
    localStorage.setItem(CDN_STORAGE_KEY, nodeId);
    localStorage.setItem(CDN_SOURCE_KEY, source);
  } catch {}
  _notifyListeners();
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
