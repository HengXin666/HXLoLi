import React, {type ReactNode} from 'react';
import {useWindowSize} from '@docusaurus/theme-common';
import BlogSidebarDesktop from '@theme/BlogSidebar/Desktop';
import BlogSidebarMobile from '@theme/BlogSidebar/Mobile';
import type {Props} from '@theme/BlogSidebar';

export default function BlogSidebar({sidebar}: Props): ReactNode {
  const windowSize = useWindowSize();
  if (!sidebar?.items.length) {
    return null;
  }
  // 移动侧边栏不需要服务器渲染
  if (windowSize === 'mobile') {
    return <BlogSidebarMobile sidebar={sidebar} />;
  }
  return <BlogSidebarDesktop sidebar={sidebar} />;
}
