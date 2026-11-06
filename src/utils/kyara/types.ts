/**
 * kyara 角色配图: 类型契约 (冻结, 路 4 只读). 加载原理:
 *   static/img/kyara/{name}/NNN-标题@作者.ext 按序号升序渲染, 可选 kyara.json 给元数据;
 *   本地模式把图片存在浏览器 IndexedDB, 排在仓库图片之后.
 * 函数的实现分在同目录 filename.ts (命名规则) / resolvers.ts (p站 / x.com / 直链解析) / localStore.ts (本地模式).
 * .agents/notes/implemented/feature/2026-11-06-anime-kyara-gallery.md
 */
export interface KyaraImage {
    /** 文件名里的序号 (本地条目是排在仓库图片之后的推算序号) */
    seq: number;
    title: string;
    author: string | null;
    /** 仓库条目是真实文件名; 本地条目是稳定的本地键 (local-xxxx.ext) */
    file: string;
    /** 可直接放进 <img src> 的地址 (仓库: baseUrl 下的静态路径; 本地: blob: URL) */
    src: string;
    source?: "static" | "local";
    sourceUrl?: string;
    w?: number;
    h?: number;
}

export interface KyaraMeta {
    /** 目录名, 也是 URL 段 */
    name: string;
    displayName: string;
    work?: string;
    bangumiCharacterId?: number;
    accent?: string;
    /** 封面: kyara.json 里写的文件名会被插件解析成 src; 不写则取第一张 */
    cover?: string;
}

export interface KyaraData {
    meta: KyaraMeta;
    images: KyaraImage[];
}

export interface KyaraIndex {
    characters: { meta: KyaraMeta; count: number; cover: string | null }[];
}

export interface KyaraImportItem {
    imageUrl: string;
    title: string;
    author: string;
    sourceUrl: string;
    referer?: string;
}

export interface KyaraSourceResolver {
    id: "pixiv" | "x" | "direct";
    match(url: string): boolean;
    resolve(url: string, fetchImpl?: typeof fetch): Promise<KyaraImportItem[]>;
}

export interface KyaraLocalStore {
    list(name: string): Promise<KyaraImage[]>;
    add(name: string, files: File[] | KyaraImportItem[]): Promise<KyaraImage[]>;
    remove(name: string, file: string): Promise<void>;
    reorder(name: string, files: string[]): Promise<void>;
    exportZip(name: string): Promise<Blob>;
}
