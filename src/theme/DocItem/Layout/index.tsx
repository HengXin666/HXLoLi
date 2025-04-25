import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import { useWindowSize } from '@docusaurus/theme-common';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import DocItemPaginator from '@theme/DocItem/Paginator';
import DocVersionBanner from '@theme/DocVersionBanner';
import DocVersionBadge from '@theme/DocVersionBadge';
import DocItemFooter from '@theme/DocItem/Footer';
import DocItemTOCMobile from '@theme/DocItem/TOC/Mobile';
import DocItemTOCDesktop from '@theme/DocItem/TOC/Desktop';
import DocItemContent from '@theme/DocItem/Content';
import DocBreadcrumbs from '@theme/DocBreadcrumbs';
import ContentVisibility from '@theme/ContentVisibility';
import type { Props } from '@theme/DocItem/Layout';

import config from '@generated/docusaurus.config';
import MDXA from '../../MDXComponents/A';
import HXGiscus from '../../../components/Giscus';
import styles from './styles.module.css';
import './hx.css';

/**
 * Decide if the toc should be rendered, on mobile or desktop viewports
 */
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

    return (
        <div className="row">
            <div className={clsx('col', !docTOC.hidden && styles.docItemCol)}>
                <ContentVisibility metadata={metadata} />
                <DocVersionBanner />
                <div className={styles.docItemContainer}>
                    <article>
                        <DocBreadcrumbs />
                        <DocVersionBadge />
                        {docTOC.mobile}
                        <DocItemContent>{children}</DocItemContent>
                        <DocItemFooter />
                    </article>
                    <div style={{ display: 'flex', justifyContent: 'flex-end', marginTop: '20px' }}>
                        请作者喝奶茶:
                        <div className="icon-container" style={{marginLeft: '10px'}}>
                            <img src={`${config.baseUrl}/default-icons/alipay.svg`} alt="Alipay Icon" className="icon" />
                            <img src={`${config.baseUrl}/img/alipay_qr_code.png`} alt="QR Code" className="qr-code" />
                        </div>
                        <div className="icon-container" style={{marginLeft: '10px'}}>
                            <img src={`${config.baseUrl}/default-icons/wechat.svg`} alt="Alipay Icon" className="icon" />
                            <img src={`${config.baseUrl}/img/wechat_qr_code.png`} alt="QR Code" className="qr-code" />
                        </div>
                    </div>
                    <div style={{ display: 'flex', justifyContent: 'flex-end' }}>
                        <p style={{fontSize: '12px'}}>本文遵循 <img src={`${config.baseUrl}/default-icons/cc.svg`} alt="CC" style={{width: '14px'}} /> <MDXA href='https://creativecommons.org/licenses/by-sa/4.0/'>CC 4.0 BY-SA</MDXA> 版权协议, 转载请标明出处</p>
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
