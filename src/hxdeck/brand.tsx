import React from 'react';
import { useDeckTheme } from './theme/context';
import { normalizeSlot } from './assets';

/**
 * 右上角品牌题头 —— 站点名 + 图标.
 *
 * 图标来自主题皮肤槽 assets.logo (默认指向博客自己的 logo),
 * 因此换主题 = 换品牌资产, 控件结构不动.
 */
export function Brand(): React.ReactElement | null {
    const t = useDeckTheme();
    const logo = normalizeSlot(t.assets?.logo)?.src;
    const name = t.assets?.brand ?? 'HXLoLi';

    return (
        <div className="hxd-brand">
            {logo ? <img className="hxd-brand__logo" src={logo} alt={name} /> : null}
            <span className="hxd-brand__name">{name}</span>
        </div>
    );
}

export default Brand;