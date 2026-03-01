/** libass-wasm (JavascriptSubtitlesOctopus) 类型声明 */
declare module 'libass-wasm' {
    interface SubtitlesOctopusOptions {
        /** 用于渲染的 video 元素 (可选) */
        video?: HTMLVideoElement;
        /** 用于渲染的 canvas 元素 (可选) */
        canvas?: HTMLCanvasElement;
        /** ASS 字幕文件 URL */
        subUrl?: string;
        /** ASS 字幕文件内容 */
        subContent?: string;
        /** Worker 文件 URL */
        workerUrl?: string;
        /** Legacy Worker 文件 URL */
        legacyWorkerUrl?: string;
        /** 字体文件 URL 数组 */
        fonts?: string[];
        /** 可用字体映射 */
        availableFonts?: Record<string, string>;
        /** 回退字体 URL */
        fallbackFont?: string;
        /** 延迟加载文件 */
        lazyFileLoading?: boolean;
        /** 时间偏移 */
        timeOffset?: number;
        /** 就绪回调 */
        onReady?: () => void;
        /** 错误回调 */
        onError?: (error: any) => void;
        /** 调试模式 */
        debug?: boolean;
        /** 渲染模式 */
        renderMode?: 'js-blend' | 'wasm-blend' | 'lossy';
        /** 目标帧率 */
        targetFps?: number;
        /** 位图缓存内存限制 (MiB) */
        libassMemoryLimit?: number;
        /** 字形缓存内存限制 (MiB) */
        libassGlyphLimit?: number;
        /** 缩放因子 */
        prescaleFactor?: number;
        /** 缩放高度限制 */
        prescaleHeightLimit?: number;
        /** 最大渲染高度 */
        maxRenderHeight?: number;
        /** 丢弃所有动画 */
        dropAllAnimations?: boolean;
    }

    class SubtitlesOctopus {
        constructor(options: SubtitlesOctopusOptions);
        /** 设置当前渲染时间 */
        setCurrentTime(time: number): void;
        /** 调整渲染尺寸 */
        resize(width?: number, height?: number, top?: number, left?: number): void;
        /** 通过 URL 设置字幕 */
        setTrackByUrl(url: string): void;
        /** 通过内容设置字幕 */
        setTrack(content: string): void;
        /** 释放字幕轨道 */
        freeTrack(): void;
        /** 销毁实例 */
        dispose(): void;
        /** Canvas 元素 */
        canvas: HTMLCanvasElement;
    }

    export default SubtitlesOctopus;
}
