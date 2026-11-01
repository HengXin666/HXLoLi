import { ANiMeRecord, WatchStatus } from "@site/src/utils/anime/types";
import { animeSeasonOf, RecordGroup, statusLabel } from "@site/src/utils/anime/filters";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";

/**
 * 画布逻辑宽度 (CSS px); 实际像素 = 逻辑尺寸 x scale.
 * 分享图与文字清单的画法、封面回退链见下面 renderShareImage 的注释, 决策记录:
 * .agents/notes/implemented/feature/2026-11-01-anime-share-image-and-list.md
 */
const W = 1080;
const PAD = 48;
const HEADER_GAP = 28;
const FOOTER_H = 72;
const SECTION_H = 46;
/** iOS Safari 单张画布面积上限 (约 16.7M 像素), 超了 toBlob 会静默出空图 */
const MAX_AREA = 16_000_000;
/** 单张画布边长上限 (多数浏览器 16384 / 32767, 取保守值) */
const MAX_SIDE = 16_000;

const FONT = `"PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", "Noto Sans CJK SC", "Noto Sans SC", "Source Han Sans SC", system-ui, sans-serif`;

const COLOR = {
    bgTop: "#17141f",
    bgBottom: "#0e0d13",
    card: "rgba(255, 255, 255, 0.06)",
    cardBorder: "rgba(255, 255, 255, 0.08)",
    text: "#f4f1fa",
    sub: "rgba(244, 241, 250, 0.62)",
    faint: "rgba(244, 241, 250, 0.38)",
    accent: "#f472b6",
    chip: "rgba(244, 114, 182, 0.14)",
    placeholder: "#2a2633",
};

const STATUS_COLOR: Record<number, string> = {
    [WatchStatus.WATCHING]: "#38bdf8",
    [WatchStatus.WATCHED]: "#a78bfa",
    [WatchStatus.WANT_TO_WATCH]: "#fbbf24",
    [WatchStatus.ON_HOLD]: "#94a3b8",
    [WatchStatus.DROPPED]: "#f87171",
};

export interface ShareImageInput {
    /** 头部大标题, 如 "我看过的番" */
    title: string;
    /** 当前生效的筛选条件文案 (与番剧库的条件 chips 同源) */
    conditions: readonly string[];
    /** 已按当前排序排好、按番剧库同一规则分好段的记录 */
    groups: readonly RecordGroup[];
    /** true = 封面墙; false = 纯文字版, 全程不加载任何图片 */
    withCovers: boolean;
    /** 页脚的站点地址 */
    siteUrl: string;
    /** 页脚的生成日期 */
    dateText: string;
    onProgress?: (done: number, total: number) => void;
}

export interface ShareImageResult {
    /** 一般一张; 部数多到超出画布上限时按行切成多张, 序号连续 */
    pages: Blob[];
    /** 带封面时没加载上的封面数 (画了占位块) */
    failedCovers: number;
    total: number;
}

const displayName = (r: ANiMeRecord): string => r.anime_data.name_cn || r.anime_data.name;

function metaLine (r: ANiMeRecord): string {
    const s = animeSeasonOf(r.anime_data.date);
    const parts: string[] = [];
    if (s) parts.push(`${s.year} ${s.season}`);
    if (r.anime_data.score > 0) parts.push(`★${r.anime_data.score.toFixed(1)}`);
    return parts.join(" · ");
}

// ---------------------------------------------------------------- 封面

/**
 * 封面候选地址, 按顺序试:
 * 1. ANiMeRecord.json 的 anime_data.image_url 是 lain.bgm.tv 原图 (l 尺寸约 2000x2800 / 0.9 MB), 不带 CORS 头, 不能进画布;
 *    同路径插入 /r/400 是 Bangumi 的缩图服务, 带 Access-Control-Allow-Origin: *, 约 50 KB.
 * 2. HXLoLi-ANiMe 仓库镜像的 data/anime/<id>.jpg (站点本来就用它显示封面), jsDelivr 与 CF Worker 都带 CORS 头, 但单张可达数 MB.
 */
export function coverCandidates (r: ANiMeRecord): string[] {
    const out: string[] = [];
    const raw = (r.anime_data.image_url ?? "").replace(/^http:\/\//, "https://");
    const m = raw.match(/^https:\/\/lain\.bgm\.tv\/pic\/(.+)$/);
    if (m) out.push(`https://lain.bgm.tv/r/400/pic/${m[1]}`);
    const mirror = toAnimeCdnUrl(`/data/anime/${r.anime_data.id}.jpg`);
    if (mirror) out.push(mirror);
    return out;
}

function loadImage (url: string, timeoutMs: number): Promise<HTMLImageElement> {
    return new Promise((resolve, reject) => {
        const img = new Image();
        // 必须在 src 之前设: 没有 CORS 头的图会直接加载失败, 而不是加载成功后污染画布
        img.crossOrigin = "anonymous";
        img.decoding = "async";
        const timer = window.setTimeout(() => {
            img.src = "";
            reject(new Error(`timeout: ${url}`));
        }, timeoutMs);
        img.onload = () => { window.clearTimeout(timer); resolve(img); };
        img.onerror = () => { window.clearTimeout(timer); reject(new Error(`error: ${url}`)); };
        img.src = url;
    });
}

async function loadCover (r: ANiMeRecord): Promise<HTMLImageElement | null> {
    for (const url of coverCandidates(r)) {
        try {
            return await loadImage(url, 8000);
        } catch {
            // 换下一个候选
        }
    }
    return null;
}

/** 并发 limit 个加载全部封面; 结果与输入同序, 失败的位置是 null */
async function loadCovers (
    records: readonly ANiMeRecord[],
    limit: number,
    onProgress?: (done: number, total: number) => void,
): Promise<(HTMLImageElement | null)[]> {
    const out: (HTMLImageElement | null)[] = new Array(records.length).fill(null);
    let next = 0;
    let done = 0;
    const worker = async (): Promise<void> => {
        while (next < records.length) {
            const i = next++;
            out[i] = await loadCover(records[i]);
            done++;
            onProgress?.(done, records.length);
        }
    };
    await Promise.all(Array.from({ length: Math.min(limit, records.length) }, worker));
    return out;
}

// ---------------------------------------------------------------- 画笔工具

function roundRect (ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number): void {
    const rr = Math.min(r, w / 2, h / 2);
    ctx.beginPath();
    ctx.moveTo(x + rr, y);
    ctx.arcTo(x + w, y, x + w, y + h, rr);
    ctx.arcTo(x + w, y + h, x, y + h, rr);
    ctx.arcTo(x, y + h, x, y, rr);
    ctx.arcTo(x, y, x + w, y, rr);
    ctx.closePath();
}

/** 按字符断行 (中日文没有空格可断), 超出 maxLines 的最后一行截断加省略号 */
function wrapText (ctx: CanvasRenderingContext2D, text: string, maxW: number, maxLines: number): string[] {
    const chars = Array.from(text);
    const lines: string[] = [];
    let cur = "";
    for (let i = 0; i < chars.length; i++) {
        const next = cur + chars[i];
        if (ctx.measureText(next).width <= maxW || cur === "") {
            cur = next;
            continue;
        }
        if (lines.length === maxLines - 1) {
            lines.push(ellipsize(ctx, cur + chars.slice(i).join(""), maxW));
            return lines;
        }
        lines.push(cur);
        cur = chars[i];
    }
    if (cur) lines.push(cur);
    return lines;
}

function ellipsize (ctx: CanvasRenderingContext2D, text: string, maxW: number): string {
    if (ctx.measureText(text).width <= maxW) return text;
    const chars = Array.from(text);
    while (chars.length > 0 && ctx.measureText(chars.join("") + "…").width > maxW) chars.pop();
    return chars.join("") + "…";
}

/** object-fit: cover 居中裁切 */
function drawCover (ctx: CanvasRenderingContext2D, img: HTMLImageElement, x: number, y: number, w: number, h: number): void {
    const iw = img.naturalWidth;
    const ih = img.naturalHeight;
    if (!iw || !ih) return;
    const scale = Math.max(w / iw, h / ih);
    const sw = w / scale;
    const sh = h / scale;
    ctx.drawImage(img, (iw - sw) / 2, (ih - sh) / 2, sw, sh, x, y, w, h);
}

// ---------------------------------------------------------------- 排版

/** 一个可分页的块: 段标题或一行条目. 分页只在块之间切 */
interface Block {
    kind: "section" | "row";
    h: number;
    label?: string;
    /** 本行条目在全体中的下标 (从 0 起, 用于序号与封面) */
    items?: number[];
}

interface Layout {
    cols: number;
    cellW: number;
    cellH: number;
    coverH: number;
    rowGap: number;
    colGap: number;
    titleSize: number;
    titleLines: number;
    showMeta: boolean;
}

function wallLayout (n: number): Layout {
    const cols = n <= 12 ? 4 : n <= 30 ? 5 : n <= 60 ? 6 : n <= 120 ? 8 : 10;
    const colGap = cols >= 8 ? 10 : 16;
    const cellW = (W - PAD * 2 - colGap * (cols - 1)) / cols;
    const coverH = cellW * 1.5;
    const titleSize = Math.max(11, Math.min(19, Math.round(cellW * 0.085)));
    const titleLines = cols >= 8 ? 1 : 2;
    const showMeta = cols <= 6;
    const textH = 10 + titleLines * titleSize * 1.3 + (showMeta ? titleSize * 0.8 * 1.5 + 2 : 0);
    return { cols, cellW, cellH: coverH + textH, coverH, rowGap: cols >= 8 ? 14 : 22, colGap, titleSize, titleLines, showMeta };
}

function textLayout (n: number): Layout {
    const cols = n > 20 ? 2 : 1;
    const colGap = 16;
    const cellW = (W - PAD * 2 - colGap * (cols - 1)) / cols;
    return { cols, cellW, cellH: 64, coverH: 0, rowGap: 10, colGap, titleSize: 19, titleLines: 1, showMeta: true };
}

function buildBlocks (groups: readonly RecordGroup[], lay: Layout): Block[] {
    const blocks: Block[] = [];
    const showSections = groups.length > 1 || (groups.length === 1 && !!groups[0].label);
    let idx = 0;
    for (const g of groups) {
        if (showSections && g.label) blocks.push({ kind: "section", h: SECTION_H, label: `${g.label} · ${g.records.length} 部` });
        for (let i = 0; i < g.records.length; i += lay.cols) {
            const items: number[] = [];
            for (let j = i; j < Math.min(i + lay.cols, g.records.length); j++) items.push(idx + j);
            blocks.push({ kind: "row", h: lay.cellH + lay.rowGap, items });
        }
        idx += g.records.length;
    }
    return blocks;
}

// ---------------------------------------------------------------- 头尾

function measureHeader (ctx: CanvasRenderingContext2D, input: ShareImageInput): number {
    let h = PAD + 52 + 14 + 26; // 标题 + 副标题
    const rows = chipRows(ctx, input.conditions);
    if (rows.length) h += 18 + rows.length * 34 + (rows.length - 1) * 8;
    return h + HEADER_GAP;
}

function chipRows (ctx: CanvasRenderingContext2D, chips: readonly string[]): string[][] {
    ctx.font = `500 15px ${FONT}`;
    const rows: string[][] = [];
    let cur: string[] = [];
    let x = 0;
    const maxW = W - PAD * 2;
    for (const c of chips) {
        const w = Math.min(maxW, ctx.measureText(c).width + 28);
        if (cur.length && x + w > maxW) { rows.push(cur); cur = []; x = 0; }
        cur.push(c);
        x += w + 8;
    }
    if (cur.length) rows.push(cur);
    return rows;
}

function drawHeader (ctx: CanvasRenderingContext2D, input: ShareImageInput, total: number, page: number, pages: number): number {
    let y = PAD;
    ctx.fillStyle = COLOR.accent;
    ctx.fillRect(PAD, y + 6, 6, 40);
    ctx.fillStyle = COLOR.text;
    ctx.font = `800 44px ${FONT}`;
    ctx.textBaseline = "top";
    const title = pages > 1 ? `${input.title}（${page}/${pages}）` : input.title;
    ctx.fillText(ellipsize(ctx, title, W - PAD * 2 - 22), PAD + 22, y);
    y += 52 + 14;
    ctx.font = `500 20px ${FONT}`;
    ctx.fillStyle = COLOR.sub;
    ctx.fillText(`共 ${total} 部${input.withCovers ? "" : " · 纯文字版"}`, PAD + 22, y);
    y += 26;
    const rows = chipRows(ctx, input.conditions);
    if (rows.length) {
        y += 18;
        ctx.font = `500 15px ${FONT}`;
        for (const row of rows) {
            let x = PAD;
            for (const c of row) {
                const label = ellipsize(ctx, c, W - PAD * 2 - 28);
                const w = ctx.measureText(label).width + 28;
                roundRect(ctx, x, y, w, 34, 17);
                ctx.fillStyle = COLOR.chip;
                ctx.fill();
                ctx.fillStyle = COLOR.accent;
                ctx.textBaseline = "middle";
                ctx.fillText(label, x + 14, y + 17);
                x += w + 8;
            }
            y += 34 + 8;
        }
        y -= 8;
        ctx.textBaseline = "top";
    }
    return y + HEADER_GAP;
}

function drawFooter (ctx: CanvasRenderingContext2D, input: ShareImageInput, h: number): void {
    const y = h - FOOTER_H;
    ctx.fillStyle = COLOR.cardBorder;
    ctx.fillRect(PAD, y, W - PAD * 2, 1);
    ctx.textBaseline = "middle";
    ctx.font = `500 16px ${FONT}`;
    ctx.fillStyle = COLOR.sub;
    ctx.textAlign = "left";
    ctx.fillText(input.siteUrl, PAD, y + FOOTER_H / 2);
    ctx.textAlign = "right";
    ctx.fillStyle = COLOR.faint;
    ctx.fillText(`HXLoLi · アニメ · ${input.dateText}`, W - PAD, y + FOOTER_H / 2);
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
}

// ---------------------------------------------------------------- 条目

function drawSection (ctx: CanvasRenderingContext2D, label: string, y: number): void {
    ctx.font = `700 20px ${FONT}`;
    ctx.fillStyle = COLOR.text;
    ctx.textBaseline = "middle";
    ctx.fillText(label, PAD, y + SECTION_H / 2 - 4);
    const w = ctx.measureText(label).width;
    ctx.fillStyle = COLOR.cardBorder;
    ctx.fillRect(PAD + w + 16, y + SECTION_H / 2 - 4, W - PAD * 2 - w - 16, 1);
    ctx.textBaseline = "top";
}

function drawWallCell (
    ctx: CanvasRenderingContext2D, lay: Layout, r: ANiMeRecord, no: number,
    img: HTMLImageElement | null, x: number, y: number,
): void {
    const { cellW, coverH } = lay;
    const radius = Math.max(6, cellW * 0.05);
    ctx.save();
    roundRect(ctx, x, y, cellW, coverH, radius);
    ctx.clip();
    if (img) {
        drawCover(ctx, img, x, y, cellW, coverH);
    } else {
        // 占位: 色块 + 居中标题, 保证这一格仍然看得出是哪部
        ctx.fillStyle = COLOR.placeholder;
        ctx.fillRect(x, y, cellW, coverH);
        ctx.fillStyle = COLOR.sub;
        ctx.font = `600 ${lay.titleSize}px ${FONT}`;
        const lines = wrapText(ctx, displayName(r), cellW - 16, 4);
        const lh = lay.titleSize * 1.3;
        let ty = y + coverH / 2 - (lines.length * lh) / 2;
        ctx.textAlign = "center";
        for (const l of lines) { ctx.fillText(l, x + cellW / 2, ty); ty += lh; }
        ctx.textAlign = "left";
    }
    ctx.restore();

    // 角标: 左上状态, 右上序号
    const badge = Math.max(10, Math.round(lay.titleSize * 0.72));
    ctx.font = `700 ${badge}px ${FONT}`;
    const status = statusLabel(r.user_status.watch_status);
    const sw = ctx.measureText(status).width + badge;
    roundRect(ctx, x + 6, y + 6, sw, badge * 1.7, badge * 0.85);
    ctx.fillStyle = STATUS_COLOR[r.user_status.watch_status] ?? COLOR.faint;
    ctx.fill();
    ctx.fillStyle = "#0e0d13";
    ctx.textBaseline = "middle";
    ctx.fillText(status, x + 6 + badge / 2, y + 6 + badge * 0.85);
    const noText = String(no);
    const nw = ctx.measureText(noText).width + badge;
    roundRect(ctx, x + cellW - 6 - nw, y + 6, nw, badge * 1.7, badge * 0.85);
    ctx.fillStyle = "rgba(0, 0, 0, 0.62)";
    ctx.fill();
    ctx.fillStyle = "#fff";
    ctx.fillText(noText, x + cellW - 6 - nw + badge / 2, y + 6 + badge * 0.85);
    ctx.textBaseline = "top";

    // 封面下: 标题 + 季度 / 评分
    let ty = y + coverH + 10;
    ctx.font = `600 ${lay.titleSize}px ${FONT}`;
    ctx.fillStyle = COLOR.text;
    for (const l of wrapText(ctx, displayName(r), cellW, lay.titleLines)) {
        ctx.fillText(l, x, ty);
        ty += lay.titleSize * 1.3;
    }
    if (lay.showMeta) {
        ctx.font = `500 ${Math.round(lay.titleSize * 0.8)}px ${FONT}`;
        ctx.fillStyle = COLOR.faint;
        ctx.fillText(ellipsize(ctx, metaLine(r), cellW), x, ty + 2);
    }
}

function drawTextCell (ctx: CanvasRenderingContext2D, lay: Layout, r: ANiMeRecord, no: number, x: number, y: number): void {
    const { cellW, cellH } = lay;
    roundRect(ctx, x, y, cellW, cellH, 12);
    ctx.fillStyle = COLOR.card;
    ctx.fill();
    ctx.save();
    roundRect(ctx, x, y, cellW, cellH, 12);
    ctx.clip();
    ctx.fillStyle = STATUS_COLOR[r.user_status.watch_status] ?? COLOR.faint;
    ctx.fillRect(x, y, 5, cellH);
    ctx.restore();

    const noW = 44;
    ctx.textBaseline = "middle";
    ctx.font = `700 16px ${FONT}`;
    ctx.fillStyle = COLOR.faint;
    ctx.fillText(String(no).padStart(2, "0"), x + 18, y + cellH / 2);

    const tx = x + 18 + noW;
    const tw = cellW - (tx - x) - 16;
    // 双栏时状态只靠左侧色条表达, 把位置留给标题
    const meta = [lay.cols > 1 ? "" : statusLabel(r.user_status.watch_status), metaLine(r)].filter(Boolean).join(" · ");
    const orig = r.anime_data.name_cn && r.anime_data.name !== r.anime_data.name_cn ? r.anime_data.name : "";
    const titleY = orig ? y + 22 : y + cellH / 2;
    ctx.font = `500 13px ${FONT}`;
    const metaW = Math.min(tw * 0.45, ctx.measureText(meta).width);
    ctx.textAlign = "right";
    ctx.fillStyle = COLOR.sub;
    ctx.fillText(ellipsize(ctx, meta, metaW + 1), x + cellW - 16, titleY);
    ctx.textAlign = "left";

    ctx.font = `700 ${lay.titleSize}px ${FONT}`;
    ctx.fillStyle = COLOR.text;
    ctx.fillText(ellipsize(ctx, displayName(r), tw - metaW - 12), tx, titleY);
    if (orig) {
        ctx.font = `400 13px ${FONT}`;
        ctx.fillStyle = COLOR.faint;
        ctx.fillText(ellipsize(ctx, orig, tw), tx, y + 46);
    }
    ctx.textBaseline = "top";
}

// ---------------------------------------------------------------- 入口

function newCanvas (h: number, scale: number): { canvas: HTMLCanvasElement; ctx: CanvasRenderingContext2D } {
    const canvas = document.createElement("canvas");
    canvas.width = Math.round(W * scale);
    canvas.height = Math.round(h * scale);
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("浏览器不支持 Canvas 2D");
    ctx.scale(scale, scale);
    const bg = ctx.createLinearGradient(0, 0, 0, h);
    bg.addColorStop(0, COLOR.bgTop);
    bg.addColorStop(1, COLOR.bgBottom);
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, W, h);
    ctx.textBaseline = "top";
    return { canvas, ctx };
}

function toPng (canvas: HTMLCanvasElement): Promise<Blob> {
    return new Promise((resolve, reject) => {
        try {
            canvas.toBlob((b) => (b ? resolve(b) : reject(new Error("toBlob 返回空 (画布过大?)"))), "image/png");
        } catch (e) {
            // 画布被跨域图污染时 toBlob 抛 SecurityError; crossOrigin 加载已经挡掉这种图, 这里只是兜底
            reject(e instanceof Error ? e : new Error(String(e)));
        }
    });
}

/**
 * 把当前筛选结果画成分享 PNG (Canvas 2D, 不依赖 DOM 截图库).
 * 条目顺序 = groups 的扁平顺序 = 番剧库 applyFilters 的排序结果; 序号从 1 连续编到 N, 分页时也连续.
 * 带封面: 封面墙, 列数随部数 4/5/6/8/10 自适应, 2:3 居中裁切, 加载失败画占位块. 不带封面: 色块列表, 不发任何图片请求.
 * 优先 2x 输出; 画布面积超出浏览器上限时降到 1.5x / 1x, 仍超就按行切成多张.
 */
export async function renderShareImage (input: ShareImageInput): Promise<ShareImageResult> {
    const records = input.groups.flatMap((g) => g.records);
    const total = records.length;
    const lay = input.withCovers ? wallLayout(total) : textLayout(total);
    const blocks = buildBlocks(input.groups, lay);

    if (typeof document !== "undefined" && document.fonts?.ready) {
        try { await document.fonts.ready; } catch { /* 字体没就绪也照画, 用回退字体 */ }
    }

    const covers: (HTMLImageElement | null)[] = input.withCovers
        ? await loadCovers(records, 6, input.onProgress)
        : records.map(() => null);
    const failedCovers = input.withCovers ? covers.filter((c) => c === null).length : 0;

    const measure = document.createElement("canvas").getContext("2d");
    if (!measure) throw new Error("浏览器不支持 Canvas 2D");
    const headerH = measureHeader(measure, input);
    const bodyH = blocks.reduce((s, b) => s + b.h, 0);
    const fullH = headerH + bodyH + FOOTER_H;

    // 选倍率; 1x 仍放不下就分页
    const fits = (h: number, s: number): boolean => W * s * h * s <= MAX_AREA && h * s <= MAX_SIDE;
    const scale = [2, 1.5, 1].find((s) => fits(fullH, s)) ?? 1;
    const maxPageH = Math.min(MAX_SIDE, Math.floor(MAX_AREA / W));
    const pagesBlocks: Block[][] = [];
    if (fits(fullH, scale)) {
        pagesBlocks.push(blocks);
    } else {
        let cur: Block[] = [];
        let h = headerH + FOOTER_H;
        for (const b of blocks) {
            if (cur.length && h + b.h > maxPageH) { pagesBlocks.push(cur); cur = []; h = headerH + FOOTER_H; }
            cur.push(b);
            h += b.h;
        }
        if (cur.length || pagesBlocks.length === 0) pagesBlocks.push(cur);
    }

    const pages: Blob[] = [];
    for (let p = 0; p < pagesBlocks.length; p++) {
        const pb = pagesBlocks[p];
        const h = headerH + pb.reduce((s, b) => s + b.h, 0) + FOOTER_H;
        const { canvas, ctx } = newCanvas(h, scale);
        let y = drawHeader(ctx, input, total, p + 1, pagesBlocks.length);
        for (const b of pb) {
            if (b.kind === "section") {
                drawSection(ctx, b.label ?? "", y);
            } else {
                (b.items ?? []).forEach((i, col) => {
                    const x = PAD + col * (lay.cellW + lay.colGap);
                    if (input.withCovers) drawWallCell(ctx, lay, records[i], i + 1, covers[i], x, y);
                    else drawTextCell(ctx, lay, records[i], i + 1, x, y);
                });
            }
            y += b.h;
        }
        drawFooter(ctx, input, h);
        pages.push(await toPng(canvas));
    }
    return { pages, failedCovers, total };
}
