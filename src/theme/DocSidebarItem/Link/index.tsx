import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import { ThemeClassNames } from '@docusaurus/theme-common';
import { isActiveSidebarItem } from '@docusaurus/plugin-content-docs/client';
import Link from '@docusaurus/Link';
import isInternalUrl from '@docusaurus/isInternalUrl';
import IconExternalLink from '@theme/Icon/ExternalLink';
import type { Props } from '@theme/DocSidebarItem/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { motion } from 'framer-motion';

import styles from './styles.module.css';
import {
    readSidebarCustomData,
    resolveSidebarIconSrc,
    routePathFromHref,
    sidebarRevealVariants,
    useSidebarPath,
} from '../sidebarItemUtils';

export default function DocSidebarItemLink ({
    item,
    onItemClick,
    activePath,
    level,
    index,
    ...props
}: Props): ReactNode {
    const { href, label, className, autoAddBaseUrl, customProps } = item;
    const isActive = isActiveSidebarItem(item, activePath);
    const isInternalLink = isInternalUrl(href);
    const { siteConfig } = useDocusaurusContext();
    const { icon, tags } = readSidebarCustomData(customProps);
    const iconSrc = resolveSidebarIconSrc(siteConfig.baseUrl, icon);
    const isAIDocsSidebar = activePath.includes('/knowledge-base') || href.includes('/knowledge-base');
    const parentPath = useSidebarPath();
    const route = routePathFromHref(href, siteConfig.baseUrl);
    // 非 ai 分支不需要 path; ai 分支 doc 用路由路径 (已是干净段路径), 相对父链只保留末段
    const myPath = route ?? (parentPath ? parentPath + '/' + label : label);

    if (!isAIDocsSidebar) {
        return (
            <li
                className={clsx(
                    ThemeClassNames.docs.docSidebarItemLink,
                    ThemeClassNames.docs.docSidebarItemLinkLevel(level),
                    'menu__list-item',
                    className,
                )}
                key={label}>
                <Link
                    className={clsx(
                        'menu__link',
                        !isInternalLink && styles.menuExternalLink,
                        {
                            'menu__link--active': isActive,
                        },
                    )}
                    autoAddBaseUrl={autoAddBaseUrl}
                    aria-current={isActive ? 'page' : undefined}
                    to={href}
                    {...(isInternalLink && {
                        onClick: onItemClick ? () => onItemClick(item) : undefined,
                    })}
                    {...props}>
                    {label}
                    {!isInternalLink && <IconExternalLink />}
                </Link>
            </li>
        );
    }

    return (
        <motion.li
            custom={{ index, level }}
            variants={sidebarRevealVariants}
            initial="hidden"
            animate="visible"
            data-sidebar-path={myPath}
            className={clsx(
                ThemeClassNames.docs.docSidebarItemLink,
                ThemeClassNames.docs.docSidebarItemLinkLevel(level),
                'menu__list-item',
                'ai-kb-sidebar-item',
                'ai-kb-sidebar-link',
                `ai-kb-sidebar-level-${level}`,
                className,
            )}
            key={label}>
            <Link
                className={clsx(
                    'menu__link',
                    !isInternalLink && styles.menuExternalLink,
                    {
                        'menu__link--active': isActive,
                    },
                )}
                autoAddBaseUrl={autoAddBaseUrl}
                aria-current={isActive ? 'page' : undefined}
                to={href}
                {...(isInternalLink && {
                    onClick: onItemClick ? () => onItemClick(item) : undefined,
                })}
                {...props}>
                <span className="ai-kb-sidebar-content">
                    <span className="ai-kb-sidebar-main-row">
                        {iconSrc && (
                            <img
                                className="ai-kb-sidebar-icon"
                                src={iconSrc}
                                alt=""
                                aria-hidden="true"
                            />
                        )}
                        <span className="ai-kb-sidebar-text">{label}</span>
                    </span>
                    {tags.length > 0 && (
                        <span className="ai-kb-sidebar-tags">
                            {tags.map((tag) => (
                                <span
                                    key={tag}
                                    className="ai-kb-sidebar-tag"
                                >
                                    {tag}
                                </span>
                            ))}
                        </span>
                    )}
                </span>
                {!isInternalLink && <IconExternalLink />}
            </Link>
        </motion.li>
    );
}
