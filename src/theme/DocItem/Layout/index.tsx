import Link from '@docusaurus/Link';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { useWindowSize } from '@docusaurus/theme-common';
import ContentVisibility from '@theme/ContentVisibility';
import DocBreadcrumbs from '@theme/DocBreadcrumbs';
import DocItemContent from '@theme/DocItem/Content';
import DocItemFooter from '@theme/DocItem/Footer';
import type { Props } from '@theme/DocItem/Layout';
import DocItemPaginator from '@theme/DocItem/Paginator';
import DocItemTOCDesktop from '@theme/DocItem/TOC/Desktop';
import DocItemTOCMobile from '@theme/DocItem/TOC/Mobile';
import DocVersionBadge from '@theme/DocVersionBadge';
import DocVersionBanner from '@theme/DocVersionBanner';
import clsx from 'clsx';
import React, { type ReactNode } from 'react';
import { FaProjectDiagram } from 'react-icons/fa';

import config from '@generated/docusaurus.config';
import HXGiscus from '../../../components/Giscus';
import MDXA from '../../MDXComponents/A';
import './hx.css';
import styles from './styles.module.css';

function useDocTOC () {
    const { frontMatter, toc } = useDoc();
    const windowSize = useWindowSize();

    const hidden = frontMatter.hide_table_of_contents;
    const canRender = !hidden && toc.length > 0;

    const mobile = canRender ? <DocItemTOCMobile /> : undefined;

    const desktop =
        canRender && (windowSize === 'desktop' || windowSize === 'ssr') ? (
            <DocItemTOCDesktop />
        ) : undefined;

    return {
        hidden,
        mobile,
        desktop,
    };
}

export default function DocItemLayout ({ children }: Props): ReactNode {
    const docTOC = useDocTOC();
    const { metadata } = useDoc();

    const docGraphUrl = (() => {
        const permalink = metadata.permalink || '';
        const base = config.baseUrl.replace(/\/$/, '');
        const docPath = permalink.startsWith(base) ? permalink.slice(base.length) : permalink;
        return `${config.baseUrl}docs-graph?doc=${encodeURIComponent(docPath)}`;
    })();

    return (
        <div className="row">
            <div className={clsx('col', !docTOC.hidden && styles.docItemCol)}>
                <ContentVisibility metadata={metadata} />
                <DocVersionBanner />
                <div className={styles.docItemContainer}>
                    <article>
                        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                            <DocBreadcrumbs />
                            <Link
                                to={docGraphUrl}
                                target="_blank"
                                style={{
                                    display: 'inline-flex',
                                    alignItems: 'center',
                                    gap: '4px',
                                    padding: '2px 8px',
                                    borderRadius: '4px',
                                    border: '1px solid var(--ifm-color-primary)',
                                    color: 'var(--ifm-color-primary)',
                                    fontSize: '11px',
                                    textDecoration: 'none',
                                    whiteSpace: 'nowrap',
                                    opacity: 0.8,
                                    transition: 'opacity 0.2s',
                                    flexShrink: 0,
                                }}
                                title="查看本笔记的引用关系图"
                                onMouseEnter={e => (e.currentTarget.style.opacity = '1')}
                                onMouseLeave={e => (e.currentTarget.style.opacity = '0.8')}
                            >
                                <FaProjectDiagram size={10} />
                                关系图
                            </Link>
                        </div>
                            <DocVersionBadge />
                            {docTOC.mobile}
                            <DocItemContent>{children}</DocItemContent>
                        <DocItemFooter />
                    </article>
                    <div style={{ display: 'flex', justifyContent: 'flex-end', alignItems: 'center', marginTop: '20px' }}>
                        请作者喝奶茶:
                        <div className="icon-container" style={{marginLeft: '10px'}}>
                            <img src={`${config.baseUrl}default-icons/alipay.svg`} alt="Alipay Icon" className="icon" />
                            <img src={`${config.baseUrl}img/alipay_qr_code.png`} alt="QR Code" className="qr-code" />
                        </div>
                        <div className="icon-container" style={{marginLeft: '10px'}}>
                            <img src={`${config.baseUrl}default-icons/wechat.svg`} alt="Alipay Icon" className="icon" />
                            <img src={`${config.baseUrl}img/wechat_qr_code.png`} alt="QR Code" className="qr-code" />
                        </div>
                    </div>
                    <div style={{ display: 'flex', justifyContent: 'flex-end' }}>
                        <span style={{fontSize: '12px'}}>本文遵循 <img src={`${config.baseUrl}default-icons/cc.svg`} alt="CC" style={{width: '14px'}} /> <MDXA href='https://creativecommons.org/licenses/by-sa/4.0/'>CC 4.0 BY-SA</MDXA> 版权协议, 转载请标明出处</span>
                    </div>
                    <div>
                        <DocItemPaginator />
                    </div>
                </div>
                <HXGiscus />
            </div>
            {docTOC.desktop && <div className="col col--3">{docTOC.desktop}</div>}
        </div>
    );
}
