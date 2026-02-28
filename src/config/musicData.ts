/**
 * 音乐播放列表数据配置
 *
 * ⚠️ 此文件由 py/music/gen_music_playlist.py 自动生成, 请勿手动修改!
 *
 * 使用方式:
 * 1. 将音频文件放在 static/music/ 目录下
 * 2. 将同名 ASS 歌词文件放在同目录下 (可选)
 * 3. 将同名封面图片放在同目录下 (可选)
 * 4. 将字体文件放在 static/music/fonts/ 或同目录下 (可选)
 * 5. 运行: cd py && uv run python music/gen_music_playlist.py
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
    /** ASS 歌词中使用的字体名列表 (用于 fallback 映射, 可选) */
    assFonts?: string[];
    /** ASS 预扫描边界框 (由 Python 脚本预计算, 固定 1920x1080 画布) */
    assBounds?: {
        topYMin: number;
        topYMax: number;
        btmYMin: number;
        btmYMax: number;
        left: number;
        right: number;
    };
}

/** 播放列表 */
export const playlist: MusicTrack[] = 
[
    {
        "id": "b4f1a9cf6842",
        "title": "ツナグキズナ (相连的羁绊)",
        "artist": "Team.ねこかん[猫]",
        "audioUrl": "/HXLoLi/music/Team.ねこかん[猫] - ツナグキズナ (相连的羁绊).mp3",
        "assUrl": "/HXLoLi/music/Team.ねこかん[猫] - ツナグキズナ (相连的羁绊).ass",
        "assFonts": [
            "TakaoPGothic"
        ],
        "assBounds": {
            "topYMin": 0,
            "topYMax": 0,
            "btmYMin": 624,
            "btmYMax": 1038,
            "left": 326,
            "right": 1646
        },
        "coverUrl": "/HXLoLi/music/Team.ねこかん[猫] - ツナグキズナ (相连的羁绊).jpg",
        "fonts": [
            "/HXLoLi/music/fonts/NotoSansSC-Regular.ttf"
        ]
    },
    {
        "id": "bd3f557257f7",
        "title": "test_audio",
        "artist": "Unknown",
        "audioUrl": "/HXLoLi/music/test_audio.mp3",
        "assUrl": "/HXLoLi/music/test_audio.ass",
        "assBounds": {
            "topYMin": 34,
            "topYMax": 540,
            "btmYMin": 1002,
            "btmYMax": 1056,
            "left": 20,
            "right": 1920
        },
        "fonts": [
            "/HXLoLi/music/fonts/NotoSansSC-Regular.ttf"
        ]
    },
    {
        "id": "a51ba6db6573",
        "title": "いとうかなこ - ファティマ ",
        "artist": "Unknown",
        "audioUrl": "/HXLoLi/music/いとうかなこ - ファティマ .mp3",
        "assUrl": "/HXLoLi/music/いとうかなこ - ファティマ .ass",
        "assFonts": [
            "@FOT-Greco Std B",
            "@方正楷体_GBK",
            "FOT-MatisseV Pro B",
            "FOT-TsukuGo Pro B",
            "FOT-TsukuMin Pro E",
            "方正中粗雅宋_GBK",
            "方正楷体_GBK",
            "方正韵动中黑_GBK"
        ],
        "assBounds": {
            "topYMin": 4,
            "topYMax": 192,
            "btmYMin": 986,
            "btmYMax": 1080,
            "left": 154,
            "right": 1762
        },
        "fonts": [
            "/HXLoLi/music/fonts/NotoSansSC-Regular.ttf"
        ]
    }
];
