/**
 * 音乐播放列表数据加载器
 *
 * 从 HXLoLi-Music 仓库通过 jsDelivr CDN 加载 playlist.json
 * 本地开发时优先从本地文件服务器 (serve.py) 加载, 自动 fallback 到 CDN
 */
import type { MusicTrack, MusicTrackDetail } from '@site/src/config/musicData';
import { isLocalDev, LOCAL_MUSIC_SERVER, toMusicCdnUrl, toMusicLocalUrl } from '@site/src/utils/cdn/linkJsDelivr';

let cachedPlaylist: MusicTrack[] | null = null;
let loadingPromise: Promise<MusicTrack[]> | null = null;
/** 标记本地服务器是否可用 (避免每个资源都尝试一次) */
let localServerAvailable: boolean | null = null;
/** 缓存已加载的歌曲详细配置 (key: track.id) */
const detailCache = new Map<string, MusicTrackDetail>();

/**
 * 检测本地音乐文件服务器是否可用
 * 通过 HEAD 请求 playlist.json 快速判断
 */
async function checkLocalServer(): Promise<boolean> {
    if (localServerAvailable !== null) return localServerAvailable;
    try {
        const resp = await fetch(`${LOCAL_MUSIC_SERVER}/playlist.json`, {
            method: 'HEAD',
            signal: AbortSignal.timeout(500), // 500ms 超时, 本地服务器应该很快
        });
        localServerAvailable = resp.ok;
    } catch {
        localServerAvailable = false;
    }
    if (localServerAvailable) {
        console.log('%c🎵 [Music] 检测到本地音乐服务器, 使用本地资源', 'color: #4CAF50; font-weight: bold');
    }
    return localServerAvailable;
}

/**
 * 根据环境将相对路径转为绝对 URL
 * 本地开发且本地服务器可用时使用本地路径, 否则使用 CDN
 */
function resolveUrl(relativePath: string | undefined, useLocal: boolean): string | undefined {
    if (!relativePath) return undefined;
    return useLocal ? toMusicLocalUrl(relativePath) : toMusicCdnUrl(relativePath);
}

/** 判断是否使用本地服务器 (缓存结果) */
async function shouldUseLocal(): Promise<boolean> {
    if (!isLocalDev()) return false;
    return await checkLocalServer();
}

/**
 * 运行时加载播放列表 (仅元信息, 体积小)
 *
 * - 本地开发时: 优先从 localhost:9527 加载 (serve.py), 失败自动 fallback 到 CDN
 * - 生产环境: 直接从 jsDelivr CDN 加载
 * - 首次调用: fetch playlist.json 并将相对路径转为绝对路径
 * - 后续调用: 返回缓存的结果
 */
export function loadPlaylist(): Promise<MusicTrack[]> {
    if (cachedPlaylist) return Promise.resolve(cachedPlaylist);
    if (loadingPromise) return loadingPromise;

    loadingPromise = (async () => {
        const useLocal = await shouldUseLocal();

        const url = useLocal
            ? toMusicLocalUrl('/playlist.json')
            : toMusicCdnUrl('/playlist.json');

        const resp = await fetch(url);
        if (!resp.ok) throw new Error(`加载播放列表失败: ${resp.statusText}`);
        const data: MusicTrack[] = await resp.json();

        // 将相对路径转为绝对路径 (本地或 CDN)
        cachedPlaylist = data.map(track => ({
            ...track,
            audioUrl: resolveUrl(track.audioUrl, useLocal)!,
            assUrl: resolveUrl(track.assUrl, useLocal),
            coverUrl: resolveUrl(track.coverUrl, useLocal),
        }));
        return cachedPlaylist;
    })();

    return loadingPromise;
}

/**
 * 按需加载歌曲详细配置 (fonts, assBounds, assImageData 等)
 *
 * 从 static/info/{id}.json 加载, 首次加载后缓存
 * 在用户点击播放歌曲时才调用, 避免一次性加载所有歌曲的重数据
 */
export async function loadTrackDetail(trackId: string): Promise<MusicTrackDetail | null> {
    // 命中缓存
    const cached = detailCache.get(trackId);
    if (cached) return cached;

    const useLocal = await shouldUseLocal();
    const url = useLocal
        ? toMusicLocalUrl(`/static/info/${trackId}.json`)
        : toMusicCdnUrl(`/static/info/${trackId}.json`);

    try {
        const resp = await fetch(url);
        if (!resp.ok) {
            // 404 表示该歌曲没有详细配置 (纯音频无 ASS), 不是错误
            if (resp.status === 404) {
                detailCache.set(trackId, {});
                return {};
            }
            throw new Error(`HTTP ${resp.status}`);
        }
        const detail: MusicTrackDetail = await resp.json();

        // 将详细配置中的相对路径转为绝对路径
        if (detail.fonts) {
            detail.fonts = detail.fonts.map(f => (useLocal ? toMusicLocalUrl(f) : toMusicCdnUrl(f)));
        }
        if (detail.assFontMap) {
            const resolved: Record<string, string> = {};
            for (const [fontName, fontPath] of Object.entries(detail.assFontMap)) {
                resolved[fontName] = useLocal ? toMusicLocalUrl(fontPath) : toMusicCdnUrl(fontPath);
            }
            detail.assFontMap = resolved;
        }

        detailCache.set(trackId, detail);
        return detail;
    } catch (err) {
        console.warn(`[Music] 加载歌曲详细配置失败 (${trackId}):`, err);
        return null;
    }
}
