import React, {type ReactNode} from 'react';
import {MDXProvider} from '@mdx-js/react';
import MDXComponents from '@theme/MDXComponents';
import type {Props} from '@theme/MDXContent';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
export default function MDXContent({children}: Props): ReactNode {
  return <MDXProvider components={MDXComponents}>
    {children}
  </MDXProvider>;
}
