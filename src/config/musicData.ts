/**
 * 音乐播放列表类型定义
 *
 * ⚠️ playlist 数据已迁移到 HXLoLi-Music 仓库, 运行时通过 CDN 加载
 * 类型定义仍保留在此文件中供项目使用
 */

/** 单首歌曲的信息 */
export interface MusicTrack {
    /** 歌曲唯一ID */
    id: string;
    /** 歌曲标题 */
    title: string;
    /** 歌手/艺术家 */
    artist: string;
    /** 音频文件URL */
    audioUrl: string;
    /** ASS 歌词文件URL (可选) */
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
        /** top 区域独立左边界 (可选, 用于精确裁剪) */
        leftT?: number;
        /** top 区域独立右边界 */
        rightT?: number;
        /** btm 区域独立左边界 */
        leftB?: number;
        /** btm 区域独立右边界 */
        rightB?: number;
    };
    /** ASS 预扫描边界框时间轴 (滑动窗口 + EMA 平滑, 每个关键点含时间戳) */
    assBoundsTimeline?: Array<{
        t: number;
        topYMin: number;
        topYMax: number;
        btmYMin: number;
        btmYMax: number;
        left: number;
        right: number;
        /** top/btm 独立左右 (用于分区域居中裁剪) */
        leftT?: number;
        rightT?: number;
        leftB?: number;
        rightB?: number;
    }>;
}
