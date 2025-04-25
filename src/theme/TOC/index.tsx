import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import TOCItems from '@theme/TOCItems';
import type { Props } from '@theme/TOC';

import { useLocation } from "react-router-dom";
import ProgressBar from '@site/src/components/ProgressBar';
import styles from './styles.module.css';

// 使用自定义 className
// 这可以防止 TOCInline/TOCCollapsible 被错误地突出显示
const LINK_CLASS_NAME = 'table-of-contents__link toc-highlight';
const LINK_ACTIVE_CLASS_NAME = 'table-of-contents__link--active';

export default function TOC ({ className, ...props }: Props): ReactNode {
    return (
        <div className={clsx(styles.tableOfContents, 'thin-scrollbar', className)}>
            <ProgressBar />
            <TOCItems
                {...props}
                linkClassName={LINK_CLASS_NAME}
                linkActiveClassName={LINK_ACTIVE_CLASS_NAME}
            />
        </div>
    );
}
