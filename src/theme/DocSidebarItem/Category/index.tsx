import React, {
    type ComponentProps,
    type ReactNode,
    useEffect,
    useMemo,
} from 'react';
import clsx from 'clsx';
import {
    ThemeClassNames,
    useThemeConfig,
    usePrevious,
    Collapsible,
    useCollapsible,
} from '@docusaurus/theme-common';
import { isSamePath } from '@docusaurus/theme-common/internal';
import {
    isActiveSidebarItem,
    findFirstSidebarItemLink,
    useDocSidebarItemsExpandedState,
} from '@docusaurus/plugin-content-docs/client';
import Link from '@docusaurus/Link';
import { translate } from '@docusaurus/Translate';
import useIsBrowser from '@docusaurus/useIsBrowser';
import DocSidebarItems from '@theme/DocSidebarItems';
import type { Props } from '@theme/DocSidebarItem/Category';

// If we navigate to a category and it becomes active, it should automatically
// expand itself
function useAutoExpandActiveCategory ({
    isActive,
    collapsed,
    updateCollapsed,
}: {
    isActive: boolean;
    collapsed: boolean;
    updateCollapsed: (b: boolean) => void;
}) {
    const wasActive = usePrevious(isActive);
    useEffect(() => {
        const justBecameActive = isActive && !wasActive;
        if (justBecameActive && collapsed) {
            updateCollapsed(false);
        }
    }, [isActive, wasActive, collapsed, updateCollapsed]);
}

/**
 * When a collapsible category has no link, we still link it to its first child
 * during SSR as a temporary fallback. This allows to be able to navigate inside
 * the category even when JS fails to load, is delayed or simply disabled
 * React hydration becomes an optional progressive enhancement
 * see https://github.com/facebookincubator/infima/issues/36#issuecomment-772543188
 * see https://github.com/facebook/docusaurus/issues/3030
 */
function useCategoryHrefWithSSRFallback (
    item: Props['item'],
): string | undefined {
    const isBrowser = useIsBrowser();
    return useMemo(() => {
        if (item.href && !item.linkUnlisted) {
            return item.href;
        }
        // In these cases, it's not necessary to render a fallback
        // We skip the "findFirstCategoryLink" computation
        if (isBrowser || !item.collapsible) {
            return undefined;
        }
        return findFirstSidebarItemLink(item);
    }, [item, isBrowser]);
}

function CollapseButton ({
    collapsed,
    categoryLabel,
    onClick,
}: {
    collapsed: boolean;
    categoryLabel: string;
    onClick: ComponentProps<'button'>['onClick'];
}) {
    return (
        <button
            aria-label={
                collapsed
                    ? translate(
                        {
                            id: 'theme.DocSidebarItem.expandCategoryAriaLabel',
                            message: "Expand sidebar category '{label}'",
                            description: 'The ARIA label to expand the sidebar category',
                        },
                        { label: categoryLabel },
                    )
                    : translate(
                        {
                            id: 'theme.DocSidebarItem.collapseCategoryAriaLabel',
                            message: "Collapse sidebar category '{label}'",
                            description: 'The ARIA label to collapse the sidebar category',
                        },
                        { label: categoryLabel },
                    )
            }
            aria-expanded={!collapsed}
            type="button"
            className="clean-btn menu__caret"
            onClick={onClick}
        />
    );
}

export default function DocSidebarItemCategory ({
    item,
    onItemClick,
    activePath,
    level,
    index,
    ...props
}: Props): ReactNode {
    const { items, label, collapsible, className, href, customProps } = item;
    const {
        docs: {
            sidebar: { autoCollapseCategories },
        },
    } = useThemeConfig();
    const hrefWithSSRFallback = useCategoryHrefWithSSRFallback(item);

    const isActive = isActiveSidebarItem(item, activePath);
    const isCurrentPage = isSamePath(href, activePath);

    const { collapsed, setCollapsed } = useCollapsible({
        // Active categories are always initialized as expanded. The default
        // (`item.collapsed`) is only used for non-active categories.
        initialState: () => {
            if (!collapsible) {
                return false;
            }
            return isActive ? false : item.collapsed;
        },
    });

    const { expandedItem, setExpandedItem } = useDocSidebarItemsExpandedState();
    // Use this instead of `setCollapsed`, because it is also reactive
    const updateCollapsed = (toCollapsed: boolean = !collapsed) => {
        setExpandedItem(toCollapsed ? null : index);
        setCollapsed(toCollapsed);
    };
    useAutoExpandActiveCategory({ isActive, collapsed, updateCollapsed });
    useEffect(() => {
        if (
            collapsible &&
            expandedItem != null &&
            expandedItem !== index &&
            autoCollapseCategories
        ) {
            setCollapsed(true);
        }
    }, [collapsible, expandedItem, index, setCollapsed, autoCollapseCategories]);

    // @ts-ignore
    const iconSrc: string = customProps && customProps.icon;
    // @ts-ignore
    const tagsArr: [] = customProps && customProps.tags;

    return (
        <li
            className={clsx(
                ThemeClassNames.docs.docSidebarItemCategory,
                ThemeClassNames.docs.docSidebarItemCategoryLevel(level),
                'menu__list-item',
                {
                    'menu__list-item--collapsed': collapsed,
                },
                className,
            )}>
            <div
                className={clsx('menu__list-item-collapsible', {
                    'menu__list-item-collapsible--active': isCurrentPage,
                })}>
                <Link
                    className={clsx('menu__link', {
                        'menu__link--sublist': collapsible,
                        'menu__link--sublist-caret': !href && collapsible,
                        'menu__link--active': isActive,
                    })}
                    onClick={
                        collapsible
                            ? (e) => {
                                onItemClick?.(item);
                                if (href) {
                                    updateCollapsed(false);
                                } else {
                                    e.preventDefault();
                                    updateCollapsed();
                                }
                            }
                            : () => {
                                onItemClick?.(item);
                            }
                    }
                    aria-current={isCurrentPage ? 'page' : undefined}
                    role={collapsible && !href ? 'button' : undefined}
                    aria-expanded={collapsible && !href ? !collapsed : undefined}
                    href={collapsible ? hrefWithSSRFallback ?? '#' : hrefWithSSRFallback}
                    style={{fontSize: '14px'}}
                    {...props}
                >
                    <div
                        style={{
                            display: 'flex',
                            flexDirection: 'column'
                        }}
                    >
                        <div style={{
                            flex: '1',
                            display: 'flex',
                            alignItems: 'center',
                            flexDirection: 'row'
                        }}>
                            <div>
                                {iconSrc && <img 
                                    src={iconSrc} 
                                    style={{ width: '24px', marginRight: '5px' }} 
                                />}
                            </div>
                            <div>
                                {label}
                            </div>
                        </div>
                        {tagsArr && tagsArr.length > 0 && 
                        <div style={{flex: '1'}}>
                            {tagsArr.map((tag, index) => (
                                <span
                                    style={{
                                        marginRight: '8px',
                                        paddingLeft: '5px',
                                        paddingRight: '5px',
                                        fontSize: '12px',
                                        borderRadius: '6px',
                                        color: '#ffffff',
                                        backgroundColor: '#990099',
                                        boxShadow: '2px 2px 4px 1px rgba(255, 255, 255, 0.5)',
                                        whiteSpace: 'nowrap', // 防止文本拆分
                                    }}
                                >{tag}</span>
                            ))}
                        </div>
                        }
                    </div>
                </Link>

                {href && collapsible && (
                    <CollapseButton
                        collapsed={collapsed}
                        categoryLabel={label}
                        onClick={(e) => {
                            e.preventDefault();
                            updateCollapsed();
                        }}
                    />
                )}
            </div>

            <Collapsible lazy as="ul" className="menu__list" collapsed={collapsed}>
                <DocSidebarItems
                    items={items}
                    tabIndex={collapsed ? -1 : 0}
                    onItemClick={onItemClick}
                    activePath={activePath}
                    level={level + 1}
                />
            </Collapsible>
        </li>
    );
}
