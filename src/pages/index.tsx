import type { ReactNode } from 'react';
import React from 'react';

import HomePage from '../components/home/HomePage';

/**
 * 首页只做入口, 实际内容在 src/components/home/HomePage.tsx
 * .agents/notes/implemented/feature/2026-10-31-homepage-profile-and-project-index.md
 */
export default function Home (): ReactNode {
    return <HomePage />;
}
