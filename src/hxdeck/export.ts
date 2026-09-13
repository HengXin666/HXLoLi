/**
 * 把内联 SVG 导出为图片 / 矢量文件.
 *
 * 参考 archify viewer 的能力: PNG / JPEG / WebP / SVG.
 *
 * 关键难点: 我们的 SVG 依赖**主题 CSS 变量**着色 (fill: var(--frontend-stroke)).
 * 直接把 SVG 丢进 canvas 会因为拿不到变量而变成一片黑, 所以导出前必须:
 *   1. 把语义 class 的 CSS 内联成 <style> 塞进 SVG
 *   2. 把主题变量作为内联 style 写到 SVG 根上
 * 这样脱离页面也能正确渲染.
 */

export type ExportFormat = 'png' | 'jpeg' | 'webp' | 'svg';

export interface ExportOptions {
    format: ExportFormat;
    /** 导出倍率, 2 = 2x 高清 */
    scale?: number;
    /** jpeg/webp 的质量 0~1 */
    quality?: number;
    /** 文件名 (不含扩展名) */
    name?: string;
    /** 背景色: png 默认透明, 其它格式需要底色 */
    background?: string;
}

/** 取出一段 HTML 里的第一个 svg 元素 */
export function firstSvg(root: HTMLElement | null): SVGSVGElement | null {
    return root ? root.querySelector('svg') : null;
}

/**
 * 生成"自包含"的 SVG 字符串:
 * 内联语义 CSS + 主题变量 + 显式宽高 (否则 canvas 不知道画多大).
 */
export function toStandaloneSvg(
    svg: SVGSVGElement,
    css: string,
    vars: Record<string, string>,
): string {
    const clone = svg.cloneNode(true) as SVGSVGElement;
    clone.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
    clone.setAttribute('xmlns:xlink', 'http://www.w3.org/1999/xlink');

    // viewBox -> 显式宽高, 让 canvas 有确定尺寸
    const vb = (clone.getAttribute('viewBox') || '').split(/[\s,]+/).map(Number);
    const w = vb.length === 4 && vb[2] > 0 ? vb[2] : svg.clientWidth || 1200;
    const h = vb.length === 4 && vb[3] > 0 ? vb[3] : svg.clientHeight || 675;
    clone.setAttribute('width', String(w));
    clone.setAttribute('height', String(h));

    // 主题变量 -> 内联 style
    const varCss = Object.entries(vars)
        .filter(([k]) => k.startsWith('--'))
        .map(([k, v]) => k + ':' + v)
        .join(';');
    clone.setAttribute('style', (clone.getAttribute('style') || '') + ';' + varCss);

    // 语义 class 的 CSS 内联进 SVG 内部
    if (css) {
        const styleEl = document.createElementNS('http://www.w3.org/2000/svg', 'style');
        styleEl.textContent = css;
        clone.insertBefore(styleEl, clone.firstChild);
    }

    // 字体: 显式写死, 避免导出后字体漂移
    const cs = getComputedStyle(svg);
    const fam = cs.fontFamily;
    if (fam) {
        const styleEl2 = document.createElementNS('http://www.w3.org/2000/svg', 'style');
        styleEl2.textContent = 'svg,text,tspan{font-family:' + fam + '}';
        clone.insertBefore(styleEl2, clone.firstChild);
    }

    return new XMLSerializer().serializeToString(clone);
}

/** 触发浏览器下载 */
export function downloadBlob(blob: Blob, filename: string): void {
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
    // 稍后回收, 太早会取消下载
    setTimeout(() => URL.revokeObjectURL(url), 4000);
}

/**
 * 导出图为指定格式.
 * 位图走 canvas (SVG -> Image -> canvas.toBlob), 矢量直接下载.
 */
export async function exportSvg(
    svg: SVGSVGElement,
    css: string,
    vars: Record<string, string>,
    opts: ExportOptions,
): Promise<void> {
    const { format, scale = 2, quality = 0.95, name = 'diagram', background } = opts;
    const text = toStandaloneSvg(svg, css, vars);

    if (format === 'svg') {
        downloadBlob(new Blob([text], { type: 'image/svg+xml;charset=utf-8' }), name + '.svg');
        return;
    }

    // 位图: 先把 SVG 变成 Image
    const blob = new Blob([text], { type: 'image/svg+xml;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    try {
        const img = await new Promise<HTMLImageElement>((resolve, reject) => {
            const i = new Image();
            i.onload = () => resolve(i);
            i.onerror = () => reject(new Error('SVG 无法转换为图片'));
            i.src = url;
        });

        const w = Number(svg.getAttribute('width')) || svg.clientWidth || 1200;
        const h = Number(svg.getAttribute('height')) || svg.clientHeight || 675;
        const canvas = document.createElement('canvas');
        canvas.width = Math.max(1, Math.round(w * scale));
        canvas.height = Math.max(1, Math.round(h * scale));
        const ctx = canvas.getContext('2d');
        if (!ctx) throw new Error('无法创建画布上下文');

        // 有底色需求时先铺底 (jpeg 不支持透明)
        if (background) {
            ctx.fillStyle = background;
            ctx.fillRect(0, 0, canvas.width, canvas.height);
        }
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

        const mime = format === 'png' ? 'image/png' : format === 'jpeg' ? 'image/jpeg' : 'image/webp';
        const out = await new Promise<Blob | null>((resolve) => canvas.toBlob(resolve, mime, quality));
        if (!out) throw new Error('画布导出失败');
        const ext = format === 'jpeg' ? 'jpg' : format;
        downloadBlob(out, name + '.' + ext);
    } finally {
        URL.revokeObjectURL(url);
    }
}

/** 把主题变量对象转成纯 CSS 变量表 (供导出使用) */
export function pickCssVars(vars: Record<string, string>): Record<string, string> {
    const out: Record<string, string> = {};
    for (const [k, v] of Object.entries(vars)) if (k.startsWith('--')) out[k] = v;
    return out;
}