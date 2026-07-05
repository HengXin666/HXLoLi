/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

import Dock from '@site/src/components/reactbits/Dock';
import React, { type ReactNode } from 'react';
import { ErrorCauseBoundary, useThemeConfig } from '@docusaurus/theme-common';
import {
  splitNavbarItems,
  useNavbarMobileSidebar,
} from '@docusaurus/theme-common/internal';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import NavbarItem, { type Props as NavbarItemConfig } from '@theme/NavbarItem';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import NavbarSearch from '@theme/Navbar/Search';
import SearchBar from '@theme/SearchBar';

import styles from './styles.module.css';

function useNavbarItems() {
  // TODO temporary casting until ThemeConfig type is improved
  return useThemeConfig().navbar.items as NavbarItemConfig[];
}

function NavbarItemWithBoundary({
  item,
}: {
  item: NavbarItemConfig;
}): ReactNode {
  return (
    <ErrorCauseBoundary
      onError={(error) =>
        new Error(
          `A theme navbar item failed to render.
Please double-check the following navbar item (themeConfig.navbar.items) of your Docusaurus config:
${JSON.stringify(item, null, 2)}`,
          { cause: error },
        )
      }>
      <NavbarItem {...item} />
    </ErrorCauseBoundary>
  );
}

function NavbarDockItems({
  items,
  ariaLabel,
  className,
}: {
  items: NavbarItemConfig[];
  ariaLabel: string;
  className?: string;
}): ReactNode {
  if (items.length === 0) return null;

  return (
    <Dock
      className={`${styles.navbarDock} ${className ?? ''}`}
      panelHeight={40}
      baseItemSize={34}
      magnification={38}
      distance={78}
      ariaLabel={ariaLabel}
    >
      {items.map((item, i) => (
        <NavbarItemWithBoundary item={item} key={i} />
      ))}
    </Dock>
  );
}

function NavbarContentLayout({
  left,
  right,
}: {
  left: ReactNode;
  right: ReactNode;
}) {
  return (
    <div className="navbar__inner">
      <div className="navbar__items">{left}</div>
      <div className="navbar__items navbar__items--right">{right}</div>
    </div>
  );
}

export default function NavbarContent(): ReactNode {
  const mobileSidebar = useNavbarMobileSidebar();

  const items = useNavbarItems();
  const [leftItems, rightItems] = splitNavbarItems(items);

  const searchBarItem = items.find((item) => item.type === 'search');

  return (
    <NavbarContentLayout
      left={
        // TODO stop hardcoding items?
        <>
          {!mobileSidebar.disabled && <NavbarMobileSidebarToggle />}
          <NavbarLogo />
          <NavbarDockItems items={leftItems} ariaLabel="主导航" />
        </>
      }
      right={
        // TODO stop hardcoding items?
        // Ask the user to add the respective navbar items => more flexible
        <>
          <NavbarDockItems
            items={rightItems}
            ariaLabel="导航工具"
            className={styles.navbarDockRight}
          />
          <Dock
            className={`${styles.navbarDock} ${styles.navbarDockCompact} ${styles.colorModeToggle}`}
            panelHeight={40}
            baseItemSize={34}
            magnification={38}
            distance={70}
            ariaLabel="主题切换"
          >
            <NavbarColorModeToggle />
          </Dock>
          {!searchBarItem && (
            <NavbarSearch>
              <SearchBar />
            </NavbarSearch>
          )}
        </>
      }
    />
  );
}
