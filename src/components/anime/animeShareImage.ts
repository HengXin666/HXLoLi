import { ANiMeRecord } from "@site/src/utils/anime/types";
import { animeSeasonOf, RecordGroup } from "@site/src/utils/anime/filters";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";

/**
 * 画布逻辑宽度 (CSS px); 实际像素 = 逻辑尺寸 x scale.
 * 分享图与文字清单的画法、封面回退链见下面 renderShareImage 的注释, 决策记录:
 * .agents/notes/implemented/feature/2026-11-01-anime-share-image-and-list.md
 */
const W = 1080;
const PAD = 64;
const HEADER_GAP = 40;
const FOOTER_H = 84;
const SECTION_H = 64;
/** iOS Safari 单张画布面积上限 (约 16.7M 像素), 超了 toBlob 会静默出空图 */
const MAX_AREA = 16_000_000;
/** 单张画布边长上限 (多数浏览器 16384 / 32767, 取保守值) */
const MAX_SIDE = 16_000;

const FONT = `"PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", "Noto Sans CJK SC", "Noto Sans SC", "Source Han Sans SC", system-ui, sans-serif`;
/** 标题与段标题用宋体 / 明朝, 没有就落回无衬线; 正文、序号、页脚仍用 FONT */
const SERIF = `"Songti SC", "STSong", "Noto Serif CJK SC", "Noto Serif SC", "Source Han Serif SC", "SimSun", serif`;

/**
 * 印刷品式的克制配色: 纸色底 + 墨色字 + 两级灰 + 发丝线, 只有一个强调色 (朱红), 只用在标题下的粗线.
 * 不用渐变、圆角、阴影、色块角标; 层级只靠字号、字重与留白.
 */
const COLOR = {
    paper: "#f5f2ec",
    ink: "#1b1a18",
    sub: "#6f6a62",
    faint: "#a19b91",
    rule: "#d8d2c7",
    accent: "#b8382a",
    placeholder: "#e6e1d8",
    coverEdge: "rgba(27, 26, 24, 0.10)",
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
    /** 段内部数 (段标题右侧小字) */
    count?: number;
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
    // 列距与行距同一节奏: 行距 = 列距 + 封面下文字之后的呼吸, 让格子在两个方向上看起来一样松
    const colGap = cols >= 8 ? 14 : 22;
    const cellW = (W - PAD * 2 - colGap * (cols - 1)) / cols;
    const coverH = cellW * 1.5;
    const titleSize = Math.max(11, Math.min(18, Math.round(cellW * 0.08)));
    const titleLines = cols >= 8 ? 1 : 2;
    const showMeta = cols <= 6;
    const textH = 12 + titleLines * titleSize * 1.35 + (showMeta ? titleSize * 0.78 * 1.5 + 2 : 0);
    return { cols, cellW, cellH: coverH + textH, coverH, rowGap: colGap + 8, colGap, titleSize, titleLines, showMeta };
}

function textLayout (n: number): Layout {
    const cols = n > 20 ? 2 : 1;
    const colGap = 48;
    const cellW = (W - PAD * 2 - colGap * (cols - 1)) / cols;
    return { cols, cellW, cellH: 58, coverH: 0, rowGap: 0, colGap, titleSize: 18, titleLines: 1, showMeta: true };
}

function buildBlocks (groups: readonly RecordGroup[], lay: Layout): Block[] {
    const blocks: Block[] = [];
    const showSections = groups.length > 1 || (groups.length === 1 && !!groups[0].label);
    let idx = 0;
    for (const g of groups) {
        if (showSections && g.label) blocks.push({ kind: "section", h: SECTION_H, label: g.label, count: g.records.length });
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

const KICKER_H = 18;
const TITLE_SIZE = 60;

/** 条件不做 chip, 和部数排成一行小字, 用间隔号分开; 太长就断成多行 */
function metaRows (ctx: CanvasRenderingContext2D, input: ShareImageInput, total: number): string[] {
    ctx.font = `400 17px ${FONT}`;
    const parts = [`共 ${total} 部`, ...input.conditions, ...(input.withCovers ? [] : ["文字版"])];
    return wrapText(ctx, parts.join("　·　"), W - PAD * 2, 3);
}

function measureHeader (ctx: CanvasRenderingContext2D, input: ShareImageInput, total: number): number {
    const rows = metaRows(ctx, input, total).length;
    return PAD + KICKER_H + 20 + TITLE_SIZE * 1.15 + 18 + rows * 26 + 26 + 3 + HEADER_GAP;
}

function drawHeader (ctx: CanvasRenderingContext2D, input: ShareImageInput, total: number, page: number, pages: number): number {
    let y = PAD;
    ctx.textBaseline = "top";
    ctx.textAlign = "left";
    // 刊头: 小字号加字距, 像杂志栏目名
    ctx.font = `500 14px ${FONT}`;
    ctx.fillStyle = COLOR.sub;
    if ("letterSpacing" in ctx) (ctx as CanvasRenderingContext2D & { letterSpacing: string }).letterSpacing = "3px";
    ctx.fillText("HXLoLi　アニメ", PAD, y);
    if (pages > 1) {
        ctx.textAlign = "right";
        ctx.fillText(`${page} / ${pages}`, W - PAD, y);
        ctx.textAlign = "left";
    }
    if ("letterSpacing" in ctx) (ctx as CanvasRenderingContext2D & { letterSpacing: string }).letterSpacing = "0px";
    y += KICKER_H + 20;

    ctx.font = `700 ${TITLE_SIZE}px ${SERIF}`;
    ctx.fillStyle = COLOR.ink;
    ctx.fillText(ellipsize(ctx, input.title, W - PAD * 2), PAD, y);
    y += TITLE_SIZE * 1.15 + 18;

    ctx.font = `400 17px ${FONT}`;
    ctx.fillStyle = COLOR.sub;
    for (const line of metaRows(ctx, input, total)) {
        ctx.fillText(line, PAD, y);
        y += 26;
    }
    y += 26;
    // 唯一的强调色: 一条短粗线 + 一条通栏发丝线
    ctx.fillStyle = COLOR.ink;
    ctx.fillRect(PAD, y + 1, W - PAD * 2, 1);
    ctx.fillStyle = COLOR.accent;
    ctx.fillRect(PAD, y, 72, 3);
    return y + 3 + HEADER_GAP;
}

function drawFooter (ctx: CanvasRenderingContext2D, input: ShareImageInput, h: number): void {
    const y = h - FOOTER_H;
    ctx.fillStyle = COLOR.rule;
    ctx.fillRect(PAD, y + 8, W - PAD * 2, 1);
    ctx.textBaseline = "middle";
    ctx.font = `400 15px ${FONT}`;
    ctx.fillStyle = COLOR.sub;
    ctx.textAlign = "left";
    ctx.fillText(input.siteUrl, PAD, y + 8 + (FOOTER_H - 8) / 2 - 6);
    ctx.textAlign = "right";
    ctx.fillStyle = COLOR.faint;
    ctx.fillText(input.dateText, W - PAD, y + 8 + (FOOTER_H - 8) / 2 - 6);
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
}

// ---------------------------------------------------------------- 条目

/** 段标题: 宋体大字靠左, 部数小字靠右, 下面一条发丝线; 不画色块 */
function drawSection (ctx: CanvasRenderingContext2D, label: string, count: number, y: number): void {
    const base = y + SECTION_H - 22;
    ctx.textBaseline = "alphabetic";
    ctx.font = `700 26px ${SERIF}`;
    ctx.fillStyle = COLOR.ink;
    ctx.fillText(label, PAD, base);
    ctx.font = `400 14px ${FONT}`;
    ctx.fillStyle = COLOR.faint;
    ctx.textAlign = "right";
    ctx.fillText(`${count} 部`, W - PAD, base);
    ctx.textAlign = "left";
    ctx.fillStyle = COLOR.rule;
    ctx.fillRect(PAD, base + 10, W - PAD * 2, 1);
    ctx.textBaseline = "top";
}

const pad2 = (n: number): string => String(n).padStart(2, "0");

function drawWallCell (
    ctx: CanvasRenderingContext2D, lay: Layout, r: ANiMeRecord, no: number,
    img: HTMLImageElement | null, x: number, y: number,
): void {
    const { cellW, coverH } = lay;
    // 封面直角, 不叠任何角标; 一条极淡的描边让浅色封面在纸色底上有边界
    if (img) {
        drawCover(ctx, img, x, y, cellW, coverH);
    } else {
        ctx.fillStyle = COLOR.placeholder;
        ctx.fillRect(x, y, cellW, coverH);
        ctx.fillStyle = COLOR.sub;
        ctx.font = `500 ${lay.titleSize}px ${SERIF}`;
        const lines = wrapText(ctx, displayName(r), cellW - 20, 4);
        const lh = lay.titleSize * 1.35;
        let ty = y + coverH / 2 - (lines.length * lh) / 2;
        ctx.textAlign = "center";
        for (const l of lines) { ctx.fillText(l, x + cellW / 2, ty); ty += lh; }
        ctx.textAlign = "left";
    }
    ctx.strokeStyle = COLOR.coverEdge;
    ctx.lineWidth = 1;
    ctx.strokeRect(x + 0.5, y + 0.5, cellW - 1, coverH - 1);

    // 封面下: 序号与标题同一基线起排, 序号用等宽数字固定宽度, 各列标题因此左缘对齐
    let ty = y + coverH + 12;
    const noSize = Math.max(10, Math.round(lay.titleSize * 0.78));
    ctx.font = `500 ${noSize}px ${FONT}`;
    ctx.fillStyle = COLOR.faint;
    const noW = ctx.measureText("000").width + 6;
    ctx.fillText(pad2(no), x, ty + (lay.titleSize - noSize) * 0.6);
    ctx.font = `600 ${lay.titleSize}px ${FONT}`;
    ctx.fillStyle = COLOR.ink;
    for (const l of wrapText(ctx, displayName(r), cellW - noW, lay.titleLines)) {
        ctx.fillText(l, x + noW, ty);
        ty += lay.titleSize * 1.35;
    }
    if (lay.showMeta) {
        ctx.font = `400 ${Math.round(lay.titleSize * 0.78)}px ${FONT}`;
        ctx.fillStyle = COLOR.sub;
        ctx.fillText(ellipsize(ctx, metaLine(r), cellW - noW), x + noW, ty + 2);
    }
}

/** 纯文字版: 目录式, 序号 / 标题 (下一行原名) / 右侧季度评分, 行与行之间只有发丝线 */
function drawTextCell (ctx: CanvasRenderingContext2D, lay: Layout, r: ANiMeRecord, no: number, x: number, y: number): void {
    const { cellW, cellH } = lay;
    ctx.fillStyle = COLOR.rule;
    ctx.fillRect(x, y + cellH - 1, cellW, 1);

    const noW = 40;
    const orig = r.anime_data.name_cn && r.anime_data.name !== r.anime_data.name_cn ? r.anime_data.name : "";
    const titleBase = orig ? y + 27 : y + cellH / 2 + 7;
    ctx.textBaseline = "alphabetic";
    ctx.font = `500 14px ${FONT}`;
    ctx.fillStyle = COLOR.faint;
    ctx.fillText(pad2(no), x, titleBase);

    const tx = x + noW;
    const tw = cellW - noW;
    const meta = metaLine(r);
    ctx.font = `400 13px ${FONT}`;
    const metaW = Math.min(tw * 0.4, ctx.measureText(meta).width);
    ctx.textAlign = "right";
    ctx.fillStyle = COLOR.sub;
    ctx.fillText(ellipsize(ctx, meta, metaW + 1), x + cellW, titleBase);
    ctx.textAlign = "left";

    ctx.font = `600 ${lay.titleSize}px ${FONT}`;
    ctx.fillStyle = COLOR.ink;
    ctx.fillText(ellipsize(ctx, displayName(r), tw - metaW - 16), tx, titleBase);
    if (orig) {
        ctx.font = `400 13px ${FONT}`;
        ctx.fillStyle = COLOR.faint;
        ctx.fillText(ellipsize(ctx, orig, tw), tx, y + 47);
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
    ctx.fillStyle = COLOR.paper;
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
 * 版式走印刷品的克制风格: 纸色底、墨色字、宋体标题、发丝线分隔, 一个强调色只用在标题下; 没有渐变 / 圆角 / 阴影 / 色块角标.
 * 带封面: 封面墙, 列数随部数 4/5/6/8/10 自适应, 2:3 居中裁切直角, 加载失败画同色系占位块; 序号印在标题前. 不带封面: 目录式列表, 不发任何图片请求.
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
    const headerH = measureHeader(measure, input, total);
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
                drawSection(ctx, b.label ?? "", b.count ?? 0, y);
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
