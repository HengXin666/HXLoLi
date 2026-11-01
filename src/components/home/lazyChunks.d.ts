/** lazyChunks.js 的类型声明, 原因见 lazyChunks.js 顶部注释 */
export declare const loadWordCountChart: () => Promise<typeof import('../chart/WordCountChart')>;
export declare const loadWordStats: () => Promise<typeof import('@site/data/wordStats')>;
export declare const loadWakaPulse: () => Promise<typeof import('../chart/WakaTimeChart/Pulse')>;
export declare const loadWakaTimeChart: () => Promise<typeof import('../chart/WakaTimeChart')>;
export declare const loadEcosystemPanel: () => Promise<typeof import('./EcosystemPanel')>;
export declare const loadHomeAnime: () => Promise<typeof import('./HomeAnime')>;
/** 仅 dev 有值, 生产构建为 null. 决策记录挂在 lazyChunks.js 的同名声明上, 这里不重复引用. */
export declare const loadAnimeOverlayEditor: null | (() => Promise<typeof import('../anime/Tag/OverlayEditor')>);
