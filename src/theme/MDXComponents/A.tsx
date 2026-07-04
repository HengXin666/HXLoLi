import React, { type ReactNode } from 'react';
// import Link from '@docusaurus/Link';
import type { Props } from '@theme/MDXComponents/A';

import HXLink from '@site/src/components/HXLink';
import PptHtmlViewer from '@site/src/components/PptHtmlViewer';
import matchRegex from '@site/src/utils/matchRegex';

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

function isHtmlHref (href: string): boolean {
    const path = href.split(/[?#]/, 1)[0]?.toLowerCase() || '';
    return path.endsWith('.html');
}

function stripPptSyntax (text: string): string {
    return text
        .replace(/##[wW]\d+%?##/g, '')
        .replace(/(^|\s)#ppt(?=\s|##|$)/gi, ' ')
        .replace(/\s+/g, ' ')
        .trim();
}

export default function MDXA (props: Props): ReactNode {
    /*
    const {
        // 原生 <a> 标签核心属性
        href = '',          // 链接目标地址(必需)
        className,          // 自定义 CSS 类名
        target,             // 打开方式(_blank/_self/_parent/_top)
        rel,                // 链接关系(noopener/noreferrer 等)
        download,           // 下载文件名(布尔值或字符串)
        referrerPolicy,     // 引用策略(no-referrer 等)

        // 通用 HTML 属性
        id,                 // 元素唯一标识
        style,              // 内联样式对象
        title,              // 悬浮提示文本
        lang,               // 语言代码
        role,               // ARIA 角色

        // 事件处理器
        onClick,            // 点击事件回调
        onMouseOver,        // 鼠标悬停事件
        onFocus,            // 获得焦点事件

        children = '',

        // 其他扩展属性
        ...rest
    } = props;
    */

    const href = props.href || '';
    const text = getNodeText(props.children);

    if (href && isHtmlHref(href) && /(^|\s)#ppt(?=\s|##|$)/i.test(text)) {
        const width = toCssWidth(matchRegex("##[wW](\\d+%?)##", text)) || '80%';
        const title = stripPptSyntax(text) || href;

        return (
            <PptHtmlViewer
                title={title}
                src={href}
                width={width}
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
