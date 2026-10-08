import React, {type ReactNode} from 'react';
import clsx from 'clsx';
import {ThemeClassNames} from '@docusaurus/theme-common';
import type {Props} from '@theme/DocSidebarItem/Html';

import styles from './styles.module.css';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
export default function DocSidebarItemHtml({
  item,
  level,
  index,
}: Props): ReactNode {
  const {value, defaultStyle, className} = item;
  return (
    <li
      className={clsx(
        ThemeClassNames.docs.docSidebarItemLink,
        ThemeClassNames.docs.docSidebarItemLinkLevel(level),
        defaultStyle && [styles.menuHtmlItem, 'menu__list-item'],
        className,
      )}
      key={index}
      // eslint-disable-next-line react/no-danger
      dangerouslySetInnerHTML={{__html: value}}
    />
  );
}
