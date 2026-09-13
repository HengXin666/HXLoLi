import React from 'react';
import { useDeckTheme } from './theme/context';
import { Rise } from './blocks';

/**
 * 外挂嵌入控件.
 *
 * 支持两类外部图:
 *   1. .drawio.svg  —— 博客已支持的可在线编辑矢量图, 直接 <img> 即可保留可编辑性
 *   2. 任意自包含 HTML —— 用 iframe (老 #ppt 侧车 / 第三方导出图)
 *
 * 为什么不统一用 iframe:
 *   .drawio.svg 用 <img> 才能让浏览器的 SVG 渲染管线接管, 缩放清晰、体积小;
 *   而 HTML 必须 iframe 才不会污染宿主样式.
 */
export type EmbedKind = 'drawio' | 'html' | 'image';

export interface EmbedProps {
    src: string;
    /** 不传则按扩展名推断 */
    kind?: EmbedKind;
    caption?: React.ReactNode;
    /** iframe 宽高比, 默认 16:9 */
    ratio?: string;
    i?: number;
}

export function inferKind(src: string): EmbedKind {
    const p = src.split(/[?#]/, 1)[0].toLowerCase();
    if (p.endsWith('.drawio.svg') || p.endsWith('.drawio')) return 'drawio';
    if (p.endsWith('.html') || p.endsWith('.htm')) return 'html';
    return 'image';
}

export function Embed({ src, kind, caption, ratio = '16 / 9', i }: EmbedProps): React.ReactElement {
    const t = useDeckTheme();
    const k = kind ?? inferKind(src);

    const body = (
        <figure className="hxd-embed-fig" data-kind={k}>
            <div className="hxd-embed-fig__frame" style={{ aspectRatio: k === 'drawio' ? undefined : ratio }}>
                {k === 'html' ? (
                    <iframe className="hxd-embed-fig__iframe" src={src} title={typeof caption === 'string' ? caption : 'embed'} loading="lazy" />
                ) : (
                    <img className="hxd-embed-fig__img" src={src} alt={typeof caption === 'string' ? caption : ''} loading="lazy" />
                )}
            </div>
            {caption ? <figcaption className="hxd-embed-fig__cap">{caption}</figcaption> : null}
        </figure>
    );
    return i === undefined ? body : <Rise i={i}>{body}</Rise>;
}
