import React from "react";

import { loadKyaraDev } from "./devLoader";

/*
 * kyara 的 dev 专属能力 (写进仓库、仓库图排序 / 删除 / 设封面、写 kyara.json、目录约定说明), 经 devLoader.js 的 dev 分支加载,
 * 生产构建里整个 dev/ 目录与写入地址都不进产物. 本地模式本身 (IndexedDB) 不在这里, 线上照常可用.
 */
export type KyaraDevModule = NonNullable<Awaited<ReturnType<NonNullable<typeof loadKyaraDev>>>>;

let cached: KyaraDevModule | null = null;

/** 本机 dev 下返回 dev 模块, 其余情况 (生产构建 / 非本机访问 / SSR) 返回 null */
export function useKyaraDev (): KyaraDevModule | null {
    const [mod, setMod] = React.useState<KyaraDevModule | null>(cached);
    React.useEffect(() => {
        if (!loadKyaraDev || cached) return undefined;
        const h = window.location.hostname;
        if (h !== "localhost" && h !== "127.0.0.1" && h !== "[::1]" && h !== "::1") return undefined;
        let alive = true;
        loadKyaraDev().then((m) => {
            cached = m;
            if (alive) setMod(m);
        }).catch(() => { /* dev chunk 加载失败时只是没有写仓库功能 */ });
        return () => { alive = false; };
    }, []);
    return mod;
}
