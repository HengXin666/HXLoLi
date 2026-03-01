/**
 * 音乐播放列表数据加载器
 *
 * 从 HXLoLi-Music 仓库通过 jsDelivr CDN 加载 playlist.json
 * 使用 commit_id 作为版本标识, 防止缓存过久不更新
 */
import type { MusicTrack } from '@site/src/config/musicData';
import { toMusicCdnUrl } from '@site/src/utils/cdn/linkJsDelivr';

let cachedPlaylist: MusicTrack[] | null = null;
let loadingPromise: Promise<MusicTrack[]> | null = null;

/**
 * 运行时从 CDN 加载播放列表
 *
 * - 首次调用: fetch playlist.json 并将相对路径转为 CDN 绝对路径
 * - 后续调用: 返回缓存的结果
 */
export function loadPlaylist(): Promise<MusicTrack[]> {
    if (cachedPlaylist) return Promise.resolve(cachedPlaylist);
    if (loadingPromise) return loadingPromise;

    loadingPromise = (async () => {
        const url = toMusicCdnUrl('/playlist.json');
        const resp = await fetch(url);
        if (!resp.ok) throw new Error(`加载播放列表失败: ${resp.statusText}`);
        const data: MusicTrack[] = await resp.json();

        // 将相对路径转为 CDN 绝对路径
        cachedPlaylist = data.map(track => ({
            ...track,
            audioUrl: toMusicCdnUrl(track.audioUrl),
            assUrl: track.assUrl ? toMusicCdnUrl(track.assUrl) : undefined,
            coverUrl: track.coverUrl ? toMusicCdnUrl(track.coverUrl) : undefined,
            fonts: track.fonts?.map(f => toMusicCdnUrl(f)),
        }));
        return cachedPlaylist;
    })();

    return loadingPromise;
}
