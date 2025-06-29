import React, {type ReactNode} from 'react';
import clsx from 'clsx';
import {blogPostContainerID} from '@docusaurus/utils-common';
import {useBlogPost} from '@docusaurus/plugin-content-blog/client';
import MDXContent from '@theme/MDXContent';
import type {Props} from '@theme/BlogPostItem/Content';

export default function BlogPostItemContent({
  children,
  className,
}: Props): ReactNode {
  const {isBlogPostPage} = useBlogPost();
  return (
    <div
      // 此 ID 用于生成 Feed 以定位主要内容
      id={isBlogPostPage ? blogPostContainerID : undefined}
      className={clsx('markdown', className)}>
      <MDXContent>{children}</MDXContent>
    </div>
  );
}
