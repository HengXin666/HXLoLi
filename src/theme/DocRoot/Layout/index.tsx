/**
 * Swizzled DocRoot/Layout — 整个文档页面的最外层布局
 *
 * 覆盖原始组件, 在知识库页面时注入 ai-kb-page class,
 * 使得 CSS 选择器可以同时覆盖 Sidebar + Content + TOC。
 *
 * 原始 DocRoot/Layout 同时渲染 Sidebar 和 Main Content,
 * 在这里注入 class 可以避免 sidebar 被遗漏。
 */

import { useLocation } from '@docusaurus/router';
import OriginalDocRootLayout from '@theme-original/DocRoot/Layout';
import clsx from 'clsx';
import React, { useEffect, type ReactNode } from 'react';

import type { Props } from '@theme/DocRoot/Layout';

export default function DocRootLayout(props: Props): ReactNode {
  const location = useLocation();
  const isAIDocs = location.pathname.includes('/knowledge-base');

  // 注入 class 到 <html> — 影响 Navbar, Sidebar, Footer 等全局元素
  useEffect(() => {
    if (isAIDocs) {
      document.documentElement.classList.add('ai-kb-page');
      return () => {
        document.documentElement.classList.remove('ai-kb-page');
      };
    }
  }, [isAIDocs]);

  return (
    <div className={clsx(isAIDocs && 'ai-kb-page')}>
      <OriginalDocRootLayout {...props} />
    </div>
  );
}
