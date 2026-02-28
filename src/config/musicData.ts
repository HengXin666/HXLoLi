/**
 * 音乐播放列表数据配置
 *
 * ⚠️ 此文件由 scripts/gen_music_playlist.py 自动生成, 请勿手动修改!
 *
 * 使用方式:
 * 1. 将音频文件放在 static/music/ 目录下
 * 2. 将同名 ASS 歌词文件放在同目录下 (可选)
 * 3. 将同名封面图片放在同目录下 (可选)
 * 4. 将字体文件放在 static/music/fonts/ 或同目录下 (可选)
 * 5. 运行: python3 scripts/gen_music_playlist.py
 */

/** 单首歌曲的信息 */
export interface MusicTrack {
    /** 歌曲唯一ID */
    id: string;
    /** 歌曲标题 */
    title: string;
    /** 歌手/艺术家 */
    artist: string;
    /** 音频文件URL (相对于 baseUrl) */
    audioUrl: string;
    /** ASS 歌词文件URL (相对于 baseUrl, 可选) */
    assUrl?: string;
    /** 歌词所需的字体文件 URL 列表 (可选) */
    fonts?: string[];
    /** 封面图片 URL (可选) */
    coverUrl?: string;
}

/** 播放列表 */
export const playlist: MusicTrack[] = 
[
    {
        "id": "bd3f557257f7",
        "title": "test_audio",
        "artist": "Unknown",
        "audioUrl": "/HXLoLi/music/test_audio.mp3",
        "assUrl": "/HXLoLi/music/test_audio.ass"
    }
];
