import { ANiMeRecord } from "@site/src/utils/anime/types";

const displayName = (r: ANiMeRecord): string => r.anime_data.name_cn || r.anime_data.name;

/**
 * 推荐清单的两种纯文本格式, 输入就是分享图用的同一个数组 (番剧库 applyFilters 的结果与顺序).
 * 一行一部: 首行标题与部数, 每行一个名字, 末行附带当前筛选的分享页链接, 适合发群 / 私聊.
 * 逗号分隔: 只有名字, 用全角逗号连接 (中文名里常有半角逗号, 用半角会切错).
 * 中文名为空时回退原名.
 */
export function toLineList (title: string, records: readonly ANiMeRecord[], link: string): string {
    const lines = [`${title}（${records.length} 部）`, ...records.map(displayName)];
    if (link) lines.push("", link);
    return lines.join("\n");
}

export function toCommaList (records: readonly ANiMeRecord[]): string {
    return records.map(displayName).join("，");
}

/** 复制到剪贴板; 不支持 Clipboard API (非 https / 旧浏览器) 时退回 textarea + execCommand */
export async function copyText (text: string): Promise<void> {
    if (navigator.clipboard?.writeText) {
        try {
            await navigator.clipboard.writeText(text);
            return;
        } catch {
            // 权限被拒时走下面的回退
        }
    }
    const ta = document.createElement("textarea");
    ta.value = text;
    ta.setAttribute("readonly", "");
    ta.style.position = "fixed";
    ta.style.opacity = "0";
    document.body.appendChild(ta);
    ta.select();
    const ok = document.execCommand("copy");
    document.body.removeChild(ta);
    if (!ok) throw new Error("浏览器拒绝了复制");
}
