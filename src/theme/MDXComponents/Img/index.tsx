import React, { type ReactNode } from 'react';
import type { Props } from '@theme/MDXComponents/Img';

import matchRegex from '@site/src/utils/matchRegex';
import config from '@generated/docusaurus.config';

import { FaEdit } from 'react-icons/fa'; // 引入勾选、还原、复制图标
import './leetcode.css';
import ZoomImage from '@site/src/components/common/ZoomImage';

export default function MDXImg (props: Props): ReactNode {
    const {
        alt,      // 获取到 ![alt]()
        src = '' as string, // 获取到 ![alt](src)
    } = props;

    const width = ((res: string | undefined) => {
        return res ? (res.includes('%') ? res : `${res}px`) : undefined;
    })(matchRegex("##[wW](\\d+%?)##", alt));
    const borderRadius = matchRegex("##[rR](\\d+)##", alt);

    // 注意由于混淆, 它 .drawio.svg 会变为 `${hash()}.svg`
    //
    // 判定依据是"src 以 .svg 结尾", 这要求 .drawio.svg 以**文件 URL** 形态到达浏览器。
    // 一旦被 webpack 内联成 data URI, src 变成 `data:image/svg+xml;base64,...`, 这里就判假,
    // 图静默退化成裸 <img>。保证它不被内联的是 scripts/run-docusaurus.mjs 注入的
    // WEBPACK_URL_LOADER_LIMIT=0 与 docusaurus.config.ts 的护栏 ——
    // 见 .agents/notes/implemented/bug-fix/2026-09-27-drawio-svg-inlined-loses-editor-shell.md
    if (!src.endsWith('.svg')) {
        return (
            // eslint-disable-next-line jsx-a11y/alt-text
            <ZoomImage
                {...props}
                src={src}
                style={{
                    width: width,
                    borderRadius: borderRadius ? `${borderRadius}px` : '10px',
                }}
            />
        );
    }

    const handleButtonClick = (src: string) => {
        window.open(`${config.baseUrl}drawio?src=${src}`, '_blank');
    };

    return (
        <div style={{
            width: '100%', height: 'auto',
            marginTop: '20px', marginBottom: '20px'
        }}>
            <div className="leetcode_tabs" style={{ display: 'flex', justifyContent: 'space-between', width: '100%' }}>
                {/* 左侧部分 */}
                <div
                    style={{ display: 'flex', alignItems: 'center' }}
                    className='leetcode-vscode-title'
                >
                    draw.io
                </div>

                {/* 右侧按钮部分 */}
                <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'flex-end' }}>
                    <button
                        onClick={() => handleButtonClick(src)}
                        className="leetcode_tabs tabs__item"
                    >
                        <FaEdit
                            style={{
                                marginRight: '5px',
                                animation: 'checkmark 0.5s ease-in-out', // 动画效果
                            }}
                        />
                        编辑
                    </button>
                </div>
            </div>
            <div className='leetcode-tabs-content'>
                <ZoomImage
                    {...props}
                    src={src}
                    style={{
                        width: width || 'auto',
                        borderRadius: borderRadius ? `${borderRadius}px` : '10px',
                    }}
                />
            </div>
        </div>
    );
}
