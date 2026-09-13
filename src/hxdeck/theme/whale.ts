import type { DeckTheme } from './types';
import { assetMap, assetUrl } from '../assets';
import mascotPeekSrc from '../assets/memes/052.webp';
import m007 from '../assets/memes/007.webp';
import m040 from '../assets/memes/040.webp';
import m071 from '../assets/memes/071.webp';
import m081 from '../assets/memes/081.webp';
import m083 from '../assets/memes/083.webp';
import m085 from '../assets/memes/085.webp';
import m052 from '../assets/memes/052.webp';
import m123 from '../assets/memes/123.webp';
import blogLogo from '@site/static/img/logo.png';

/**
 * 「鲸鱼娘」二次元主题.
 *
 * 形象出处与授权 (重要):
 *   角色原型 "溟月" 由 上善无形 于 2025-06 创作, 以 CC BY-NC-SA 4.0 开放二创;
 *   女仆装版本由 ZipZipPipe 于 2026-04 在溟月基础上融入 DeepSeek 元素二次设计.
 *   该协议要求: 署名 + 不得商用 + 衍生作品以相同方式共享.
 *
 * 因此本主题:
 *   1. credits 字段强制携带署名, 由渲染器输出到每一页页脚;
 *   2. 默认不含任何角色图片 —— 素材需由使用者自行放入并确认合规;
 *   3. 仅提炼其公开描述的**色彩语言** (蓝色渐变长发 / 蓝瞳 / 深蓝白女仆装 / 鲸尾),
 *      不复制具体美术作品.
 */
export const whaleTheme: DeckTheme = {
    id: 'whale',
    name: '鲸鱼娘',
    description: '二次元向, 深海蓝渐变; 角色素材需自行提供',
    colors: {
        bg: '#061428',
        bgAlt: '#0d2b52',
        surface: 'rgba(125, 200, 255, 0.07)',
        surfaceAlt: 'rgba(125, 200, 255, 0.13)',
        text: '#eaf4ff',
        textMuted: '#8fb4d6',
        primary: '#4d6bfe',
        primarySoft: 'rgba(77, 107, 254, 0.22)',
        accent: '#7fd4ff',
        success: '#4de0b0',
        warn: '#ffd479',
        danger: '#ff7a9c',
        border: 'rgba(127, 212, 255, 0.26)',
    },
    shape: {
        radius: '18px',
        radiusSm: '10px',
        radiusLg: '28px',
        borderWidth: '1px',
        shadow: '0 12px 40px rgba(3, 12, 28, 0.6)',
        glow: '0 0 30px rgba(77, 107, 254, 0.45)',
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
        fast: '200ms',
        base: '360ms',
        slow: '640ms',
        stagger: '85ms',
        page: '860ms',
        easePage: 'cubic-bezier(0.65, 0, 0.35, 1)',
    },
    // 素材槽: 指向随包附带的社区表情包 (非营利粉丝向, 见 credits)
    assets: {
        // 皮肤素材: 不单独展示, 由控件嵌进自己的槽位里
        logo: assetUrl(blogLogo),
        brand: 'HXLoLi',
        mascot: assetUrl(m052),          // 封面立绘槽
        mascotPeek: assetUrl(m123),      // 卡片角落探头槽
        pattern: assetUrl(m052),         // 卡片/页面底纹纹理槽
        emoji: assetMap({
            happy: m007,
            eat: m052,
            panic: m040,
            refuse: m071,
            rich: m081,
            laugh: m083,
            tired: m085,
        }),
    },
    credits: [
        { text: '角色原型 溟月 / 上善无形 (CC BY-NC-SA 4.0)', url: 'https://zh.moegirl.org.cn/DeepSeek%E5%A8%98' },
        { text: '女仆装二次设计 / ZipZipPipe', url: 'https://zh.moegirl.org.cn/DeepSeek%E5%A8%98' },
    ],
    css: `
        /* 深海光斑: 纯 CSS 装饰, 不依赖任何图片素材 */
        .hxd-deck[data-theme='whale']::before {
            content: '';
            position: absolute;
            inset: 0;
            pointer-events: none;
            background:
                radial-gradient(60% 45% at 82% 12%, rgba(77, 107, 254, 0.30), transparent 70%),
                radial-gradient(45% 40% at 12% 88%, rgba(127, 212, 255, 0.18), transparent 72%);
            z-index: 0;
        }
    `,
};