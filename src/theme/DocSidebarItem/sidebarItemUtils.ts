import { createContext, useContext } from 'react';

export interface SidebarCustomData {
  icon?: string;
  tags: string[];
}

export const sidebarRevealVariants = {
  hidden: {
    opacity: 0,
    x: -8,
    filter: 'blur(3px)',
  },
  visible: ({ index, level }: { index: number; level: number }) => ({
    opacity: 1,
    x: 0,
    filter: 'blur(0px)',
    transition: {
      duration: 0.24,
      ease: 'easeOut',
      delay: Math.min(index * 0.018, 0.12) + Math.min(level * 0.01, 0.05),
    },
  }),
};

export function readSidebarCustomData(customProps: unknown): SidebarCustomData {
  if (!customProps || typeof customProps !== 'object') {
    return { tags: [] };
  }

  const data = customProps as Record<string, unknown>;
  return {
    icon: typeof data.icon === 'string' ? data.icon : undefined,
    tags: Array.isArray(data.tags)
      ? data.tags.filter((tag): tag is string => typeof tag === 'string')
      : [],
  };
}

export function resolveSidebarIconSrc(
  baseUrl: string,
  icon?: string,
): string | undefined {
  if (!icon) return undefined;

  if (icon.startsWith('/')) {
    return `${baseUrl.replace(/\/$/, '')}${icon}`;
  }

  return `${baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`}${icon}`;
}



/**
 * 当前条目在侧边栏树中的展示路径 (从知识库根算起, 段与段间用 /).
 * 用于「重建侧边栏」后定位 DOM: data-sidebar-path 与后端 diff 的 added/changed 对齐.
 */
export const SidebarPathContext = createContext<string>('');

export function useSidebarPath (): string {
  return useContext(SidebarPathContext);
}

/** 去掉 baseUrl 前缀, 得到文档路由路径 (不含 trailing slash) */
export function routePathFromHref (href: string | undefined, baseUrl: string): string | null {
  if (!href || href === '#' || href.startsWith('http')) return null;
  let clean = href;
  if (baseUrl && clean.startsWith(baseUrl)) clean = clean.slice(baseUrl.length);
  clean = clean.replace(/^\/+/, '').replace(/\/+$/, '');
  if (!clean) return null;
  // 去掉文档目录前缀 (knowledge-base / docs)
  const m = clean.match(/^(?:knowledge-base|docs)\/([\s\S]*)$/);
  return m ? m[1] : clean;
}

/** 从 href 路由拿条目自己的段名 (最后一段) */
export function segmentFromHref (href: string | undefined, baseUrl: string): string | null {
  const p = routePathFromHref(href, baseUrl);
  if (!p) return null;
  const segs = p.split('/');
  return segs[segs.length - 1] || null;
}
