import React, { type ReactNode } from 'react';
// import Link from '@docusaurus/Link';
import type { Props } from '@theme/MDXComponents/A';

import HXLink from '@site/src/components/HXLink';
import { PptEmbed } from '@site/src/hxdeck/PptEmbed';
import { parsePptText, classifyHref, legacyWidth } from '@site/src/hxdeck/syntax';

function getNodeText (node: ReactNode): string {
    if (typeof node === 'string' || typeof node === 'number') {
        return String(node);
    }

    if (Array.isArray(node)) {
        return node.map(getNodeText).join('');
    }

    if (React.isValidElement(node)) {
        const props = node.props as { children?: ReactNode };
        return getNodeText(props.children);
    }

    return '';
}

function toCssWidth (value: string | undefined): string | undefined {
    if (!value) return undefined;
    return value.includes('%') ? value : `${value}px`;
}

/**
 * 还原被 Docusaurus 改写的资源链接.
 *
 * Docusaurus 会把文档里的相对文件引用编译成
 *   ./x.html  ->  /HXLoLi/assets/files/x-<hash>.html
 *   ./x.tsx   ->  /HXLoLi/assets/files/x-<hash>.tsx
 * 于是链接里拿到的已经不是原始相对路径.
 * 这里把"原始文件名"抽回来, 供上层按名字去注册表/侧车里找.
 */
function restoreAssetHref (href: string): { name?: string; isAsset: boolean } {
    const path = (href || '').split(/[?#]/, 1)[0] || '';
    const m = /\/assets\/files\/(.+?)-[0-9a-f]{8,}(\.[a-z0-9]+)$/i.exec(path);
    if (m) return { name: m[1] + m[2], isAsset: true };
    return { isAsset: false };
}

/**
 * 判定"指向本地 HTML 侧车".
 * 既认原始 .html, 也认被改写后的 assets 形式.
 */
function isHtmlHref (href: string): boolean {
    const r = restoreAssetHref(href);
    const path = (r.isAsset ? r.name : href)?.split(/[?#]/, 1)[0]?.toLowerCase() || '';
    return path.endsWith('.html') || path.endsWith('.htm');
}

function stripPptSyntax (text: string): string {
    return text
        .replace(/##[wW]\d+%?##/g, '')
        .replace(/(^|\s)#ppt(?=\s|##|$)/gi, ' ')
        .replace(/\s+/g, ' ')
        .trim();
}

/**
 * 链接分发.
 *
 * 判定完全按"相对路径 + 扩展名", 与本地文件一致:
 *   .tsx           -> 我们的演示页, React 内联渲染 (吃掉主题)
 *   .html / .htm   -> 通用本地 HTML, iframe 独立渲染 (不套主题)
 *
 * 指令 (##PPT ...##) 只贡献两件事: 默认展示哪一页、用什么主题.
 * 标题一律取自链接原文, 不接受"标题="参数.
 */
export default function MDXA (props: Props): ReactNode {
    const href = props.href || '';
    const text = getNodeText(props.children);

    const ppt = parsePptText(text);
    const restored = restoreAssetHref(href);
    const realHref = restored.isAsset ? (restored.name as string) : href;
    const target = classifyHref(realHref);
    const hasPptMark = /##\s*PPT\b/i.test(text) || /(^|\s)#ppt(?=\s|##|$)/i.test(text);

    // 我们的演示页 (.tsx)
    if (target === 'deck') {
        return (
            <PptEmbed
                title={ppt.title}
                index={ppt.directive.index}
                theme={ppt.directive.theme}
                deckName={realHref}
            />
        );
    }

    /*
      通用本地 HTML (.html / .htm).

      内容仍然用 iframe 独立渲染 (它有自己的样式, 强行套主题会坏),
      但**外框与 .tsx 演示页统一** —— 两者都是"一块可预览的演示内容",
      边界表现一致, 读者一眼能分清"文章"与"演示".
    */
    if (target === 'html' && hasPptMark) {
        return (
            <PptEmbed
                title={ppt.title || stripPptSyntax(text) || 'PPT'}
                src={href}
                width={legacyWidth(text) || '100%'}
            />
        );
    }

    return (
        <span className="tailwind">
            <HXLink
                title={text}
                url={href}
            >
                {props.children}
            </HXLink>
        </span>
    );
}