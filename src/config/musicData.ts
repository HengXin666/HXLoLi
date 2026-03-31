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
    /** ASS 歌词文件的原始相对路径 (供 reqMusicByCDN 切片加载使用) */
    assRelativePath?: string;
    /** 封面图片 URL (可选) */
    coverUrl?: string;
    /** ASS 歌词中使用的字体名列表 (用于 fallback 映射, 可选) */
    assFonts?: string[];
}

/** 歌曲详细配置 (按需加载, 从 static/info/{id}.json 获取) */
export interface MusicTrackDetail {
    /** 歌词所需的字体文件 URL 列表 (可选) */
    fonts?: string[];
    /** ASS 字体名到字体文件 URL 的映射 (可选, 供 SubtitlesOctopus availableFonts 使用) */
    assFontMap?: Record<string, string>;
    /** ASS \\1img 引用的图片路径列表 (可选) */
    assImages?: string[];
    /** ASS \\1img 图片的 base64 数据, key=相对路径, value=data URI (可选, 供 SubtitlesOctopus 虚拟 FS 使用) */
    assImageData?: Record<string, string>;
    /** ASS \\1img 图片事件列表 (libass 不支持 \\1img, 需要前端自行叠加渲染) */
    assImageEvents?: Array<{
        /** 事件开始时间 (秒) */
        start: number;
        /** 事件结束时间 (秒) */
        end: number;
        /** 图片相对路径 (对应 assImageData 的 key) */
        img: string;
        /** 静态位置 [x, y] (基于 1920x1080 画布) */
        pos?: [number, number];
        /** 移动动画 [x1, y1, x2, y2] */
        move?: [number, number, number, number];
        /** 移动动画时间范围 [t1, t2] (ms, 可选) */
        moveT?: [number, number];
        /** 淡入时间 (ms) */
        fadIn?: number;
        /** 淡出时间 (ms) */
        fadOut?: number;
        /** 锚点 (1-9, 默认 5=居中) */
        an?: number;
        /** 绘图区域宽 (像素, 基于 ASS 脚本分辨率) */
        drawW?: number;
        /** 绘图区域高 (像素, 基于 ASS 脚本分辨率) */
        drawH?: number;
    }>;
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
