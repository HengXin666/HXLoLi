import type { KyaraData, KyaraImage, KyaraLocalStore, KyaraMeta } from "@site/src/utils/kyara/types";
import { kyaraLocalFilename, kyaraMetaJson } from "@site/src/utils/kyara/localStore";

// dev 写入通道 (plugins/kyara-gallery-plugin.mjs) 的客户端. 只经 ../devChunks.ts 的 dev 分支加载.

const HEADER = "X-HX-Kyara";

async function call<T> (name: string, path: string, init: RequestInit = {}): Promise<T> {
    const headers = new Headers(init.headers);
    headers.set(HEADER, "1");
    const res = await fetch(`/__hx/kyara/${encodeURIComponent(name)}/${path}`, { ...init, headers });
    const data = await res.json().catch(() => ({ ok: false, error: `${res.status} ${res.statusText}` }));
    if (!res.ok || !data?.ok) throw new Error(data?.error || `HTTP ${res.status}`);
    return data as T;
}

const json = (method: string, body: unknown): RequestInit => ({
    method, headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
});

export const kyaraRepo = {
    /** 服务端解析 + 下载 (pixiv 只能走这里), 按下一个序号落盘 */
    async importUrl (name: string, url: string): Promise<KyaraImage[]> {
        return (await call<{ images: KyaraImage[] }>(name, "import", json("POST", { url }))).images;
    },
    async putFile (name: string, filename: string, blob: Blob, overwrite = false): Promise<KyaraImage> {
        return (await call<{ image: KyaraImage }>(name, `file/${encodeURIComponent(filename)}${overwrite ? "?overwrite=1" : ""}`, {
            method: "PUT", headers: { "Content-Type": blob.type || "application/octet-stream" }, body: blob,
        })).image;
    },
    async deleteFile (name: string, filename: string): Promise<KyaraImage[]> {
        return (await call<{ images: KyaraImage[] }>(name, `file/${encodeURIComponent(filename)}`, { method: "DELETE" })).images;
    },
    async reorder (name: string, order: string[]): Promise<KyaraImage[]> {
        return (await call<{ images: KyaraImage[] }>(name, "reorder", json("POST", { order }))).images;
    },
    /** 合并写 kyara.json; 值为 null 删掉该字段; 目录不存在时创建 */
    async putMeta (name: string, patch: Record<string, unknown>): Promise<KyaraData> {
        return (await call<{ kyara: KyaraData }>(name, "meta", json("PUT", patch))).kyara;
    },
    /** dev 清单: 写入后不等热重扫, 直接拿最新的目录内容 */
    async read (baseUrl: string, name: string): Promise<KyaraData | null> {
        const res = await fetch(`${baseUrl}kyara/data/${encodeURIComponent(name)}.json`, { cache: "no-store" });
        return res.ok ? ((await res.json()) as KyaraData) : null;
    },
};

/**
 * 把本地条目逐张写进仓库 (序号接在仓库最大序号之后), 成功的从本地库删掉.
 * meta 给出时 (纯本地角色) 先写 kyara.json; 本地封面键换成落盘后的文件名.
 */
export async function commitLocalToRepo (
    name: string,
    store: KyaraLocalStore,
    images: KyaraImage[],
    meta?: Partial<KyaraMeta>,
): Promise<KyaraImage[]> {
    const done: KyaraImage[] = [];
    let coverFile: string | undefined;
    for (const img of images) {
        const blob = await (await fetch(img.src)).blob();
        const file = kyaraLocalFilename(img);
        const r = await kyaraRepo.putFile(name, file, blob);
        if (meta?.cover && meta.cover === img.file) coverFile = r.file;
        done.push(r);
        await store.remove(name, img.file);
    }
    if (meta) {
        const body = kyaraMetaJson({ ...meta, cover: coverFile });
        if (Object.keys(body).length > 0) await kyaraRepo.putMeta(name, body);
    }
    return done;
}
