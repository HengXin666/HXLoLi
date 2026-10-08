import React, {type ReactNode} from 'react';
import type {Props} from '@theme/BlogPostItem/Container';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
export default function BlogPostItemContainer({
  children,
  className,
}: Props): ReactNode {
  return <article className={className}>{children}</article>;
}
