import type { DeckTheme } from './types';
import { assetUrl } from '../assets';
import blogLogo from '@site/static/img/logo.png';

/**
 * HXLoLi 默认主题.
 * 色相跟随博客自身主色 (#ff88ff 粉紫), 但**独立取值**, 不读 --ifm-* ,
 * 以免博客改色时演示页被动漂移.
 */
export const hxloliTheme: DeckTheme = {
    id: 'hxloli',
    name: 'HXLoLi',
    description: '博客默认粉紫, 通用技术演示',
    colors: {
        bg: '#0d0b12',
        bgAlt: '#1a1024',
        surface: 'rgba(255, 255, 255, 0.04)',
        surfaceAlt: 'rgba(255, 136, 255, 0.07)',
        text: '#f3eef8',
        textMuted: '#a99bb8',
        primary: '#ff88ff',
        primarySoft: 'rgba(255, 136, 255, 0.16)',
        accent: '#7ad9ff',
        success: '#5fd39a',
        warn: '#ffcc66',
        danger: '#ff6b81',
        border: 'rgba(255, 136, 255, 0.22)',
    },
    shape: {
        radius: '14px',
        radiusSm: '8px',
        radiusLg: '22px',
        borderWidth: '1px',
        shadow: '0 10px 30px rgba(0, 0, 0, 0.45)',
        glow: '0 0 24px rgba(255, 136, 255, 0.28)',
    },
    fonts: {
        heading: '"TencentSans", "Noto Sans SC", "PingFang SC", "Microsoft YaHei", system-ui, sans-serif',
        body: '"TencentSans", "Noto Sans SC", "PingFang SC", "Microsoft YaHei", system-ui, sans-serif',
        mono: '"JetBrains Mono", "Fira Code", Consolas, "Courier New", monospace',
    },
    scale: {
        display: '68px',
        h1: '40px',
        h2: '30px',
        body: '21px',
        sm: '17px',
        xs: '15px',
        num: '50px',
        sp2: '10px',
        sp3: '14px',
        sp4: '20px',
        sp5: '26px',
        sp7: '38px',
        lhTight: '1.2',
        lhNormal: '1.62',
        lsWide: '0.09em',
        strokeW7: '0.026em',
        label: '15px',
        caption: '14px',
        hero: '84px',
    },
    elevation: {
        e0: 'none',
        e1: '0 1px 2px rgba(0,0,0,0.28), 0 2px 8px rgba(0,0,0,0.22)',
        e2: '0 2px 6px rgba(0,0,0,0.34), 0 8px 22px rgba(0,0,0,0.28)',
        e3: '0 6px 16px rgba(0,0,0,0.42), 0 18px 44px rgba(0,0,0,0.34)',
        e4: '0 12px 32px rgba(0,0,0,0.5), 0 32px 72px rgba(0,0,0,0.42)',
        tint1: 'rgba(255,255,255,0.030)',
        tint2: 'rgba(255,255,255,0.055)',
        tint3: 'rgba(255,255,255,0.085)',
    },
    state: {
        hover: 'rgba(255,255,255,0.07)',
        press: 'rgba(255,255,255,0.13)',
        focusRing: 'color-mix(in srgb, var(--hxd-color-primary) 55%, transparent)',
    },
    motion: {
        ease: 'cubic-bezier(0.22, 0.61, 0.36, 1)',
        easeBounce: 'cubic-bezier(0.34, 1.56, 0.64, 1)',
        fast: '180ms',
        base: '320ms',
        slow: '560ms',
        stagger: '70ms',
        page: '820ms',
        easePage: 'cubic-bezier(0.65, 0, 0.35, 1)',
    },
    // HXLoLi 主题只用品牌资源, 不用玩梗皮肤
    assets: {
        logo: assetUrl(blogLogo),
        brand: 'HXLoLi',
    },
};