import { RecordGroup } from "@site/src/utils/anime/filters";
import { renderShareImage } from "./animeShareImage";

/*
 * 分享图的"下载 / 复制"出口, 番剧库分享面板与 /anime/share 共用. 画图本身在 animeShareImage.ts.
 * 决策见 src/components/anime/AnimeShareBoard.tsx 顶部引用的 note.
 */

export function todayText (): string {
    const d = new Date();
    return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, "0")}-${String(d.getDate()).padStart(2, "0")}`;
}

export interface ExportImageInput {
    title: string;
    conditions: readonly string[];
    groups: readonly RecordGroup[];
    withCovers: boolean;
    /** 页脚显示的站点地址, 不带协议 */
    siteUrl: string;
    onProgress?: (done: number, total: number) => void;
}

export interface ExportedImage {
    pages: Blob[];
    failedCovers: number;
}

export async function makeShareImage (input: ExportImageInput): Promise<ExportedImage> {
    const res = await renderShareImage({ ...input, dateText: todayText() });
    return { pages: res.pages, failedCovers: res.failedCovers };
}

/** 逐张触发下载; 多张时文件名带序号 */
export function downloadImages (pages: readonly Blob[], fileBase: string): void {
    pages.forEach((blob, i) => {
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = pages.length > 1 ? `${fileBase}-${i + 1}.png` : `${fileBase}.png`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        window.setTimeout(() => URL.revokeObjectURL(url), 10_000);
    });
}

export function canCopyImage (): boolean {
    return typeof window !== "undefined"
        && typeof window.ClipboardItem === "function"
        && typeof navigator.clipboard?.write === "function";
}

/** 把一张 PNG 写进剪贴板 (需要 https 与 ClipboardItem; Firefox 旧版不支持时由调用方隐藏按钮) */
export async function copyImage (blob: Blob): Promise<void> {
    await navigator.clipboard.write([new ClipboardItem({ "image/png": blob })]);
}
