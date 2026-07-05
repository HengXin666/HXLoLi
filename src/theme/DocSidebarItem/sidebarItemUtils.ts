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
