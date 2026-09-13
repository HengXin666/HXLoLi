import React from 'react';
import { useDeckTheme } from './theme/context';
import { assetUrl } from './assets';

/**
 * 表情包 / 吉祥物控件.
 *
 * 素材: 社区整理的「DeepSeek 鲸鱼娘」表情包 (非营利粉丝向, 见主题 credits).
 * 注意: 所有 src 都过 assetUrl() 收敛, 避免 webpack 的 Module 包装导致图片静默失效.
 */

type Src = string | { default?: string };

export function Meme({
    src,
    alt = '',
    width,
    tilt = false,
    float = false,
    className,
}: {
    src: Src;
    alt?: string;
    width?: number | string;
    tilt?: boolean;
    float?: boolean;
    className?: string;
}): React.ReactElement | null {
    const url = assetUrl(src as never);
    if (!url) return null;
    return (
        <img
            src={url}
            alt={alt}
            loading="lazy"
            className={['hxd-meme', tilt ? 'hxd-meme--tilt' : '', float ? 'hxd-meme--float' : '', className]
                .filter(Boolean)
                .join(' ')}
            style={{ width }}
        />
    );
}

/**
 * 吉祥物立绘 —— 属于**主题皮肤**, 站在页面右下角.
 * 图来自主题 assets.mascot, 不单独作为内容展示.
 */
export function Mascot({ src, alt = '' }: { src?: Src; alt?: string }): React.ReactElement | null {
    const t = useDeckTheme();
    const url = assetUrl((src ?? t.assets?.mascot) as never);
    if (!url) return null;
    return <img className="hxd-mascot" src={url} alt={alt} />;
}

/** 卡片右下角的探头槽位 (皮肤, 由 Card 的 slot 传入) */
export function SlotMascot({ src, alt = '' }: { src?: Src; alt?: string }): React.ReactElement | null {
    const t = useDeckTheme();
    const url = assetUrl((src ?? t.assets?.mascotPeek) as never);
    if (!url) return null;
    return <img className="hxd-slot-mascot" src={url} alt={alt} />;
}

/** 语录气泡 */
export function Quote({ children }: { children: React.ReactNode }): React.ReactElement {
    return <div className="hxd-quote">{children}</div>;
}