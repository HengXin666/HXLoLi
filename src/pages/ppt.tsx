import React, { useEffect, useMemo, useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { Deck } from '../hxdeck/Deck';
import { defaultTheme, getTheme, loadManifest, loadTheme, loadThemeFile, listThemes } from '../hxdeck/theme/registry';
import type { ThemeInfo } from '../hxdeck/theme/registry';
import type { DeckTheme } from '../hxdeck/theme/types';
import { deckModules, findDeckKey } from '../hxdeck/decks.generated';

/**
 * 演示页独立播放页.
 *
 * 用途: 从笔记里的 PPT 外框点"新页面打开"跳到这 —— 一个干净的、可投屏的全屏页.
 * 地址形如: /ppt?deck=<key>&page=3&theme=whale
 *
 * 为什么需要它:
 *   笔记页里有导航栏、侧边栏、页脚等一切站点装饰, 投屏时全是干扰.
 *   独立页只渲染演示页本身, 并用 URL 携带状态, 因此可直接分享/收藏.
 */
export default function PptPage(): React.ReactElement {
    const { siteConfig } = useDocusaurusContext();
    const baseUrl = siteConfig.baseUrl || '/';

    const params = useMemo(() => {
        if (typeof window === 'undefined') return new URLSearchParams();
        return new URLSearchParams(window.location.search);
    }, []);

    const deckKey = params.get('deck') || '';
    const initialPage = Math.max(0, Number(params.get('page') || '1') - 1 || 0);
    const themeId = params.get('theme') || undefined;

    const [t, setT] = useState<DeckTheme>(() => (themeId ? getTheme(themeId) ?? defaultTheme() : defaultTheme()));
    const [themes, setThemes] = useState<ThemeInfo[]>([]);
    const [err, setErr] = useState('');

    // 主题清单 (顶栏下拉)
    useEffect(() => {
        let alive = true;
        loadManifest(baseUrl).then(() => { if (alive) setThemes(listThemes()); });
        return () => { alive = false; };
    }, [baseUrl]);

    /*
      指定主题.

      必须先等 loadManifest 完成再按名加载:
        用户放到 static/themes 的主题只在清单 (index.json) 里有文件名,
        清单没到就 loadTheme, 会因为它猜的 "<id>.yaml" 对不上而静默失败,
        表现为"URL 里写了 theme=sakura 却还是默认主题".
    */
    useEffect(() => {
        if (!themeId) return;
        const hit = getTheme(themeId);
        if (hit) { setT(hit); return; }
        let alive = true;
        loadManifest(baseUrl)
            .then(() => loadTheme(themeId, baseUrl))
            .then((got) => { if (alive && got) setT(got); });
        return () => { alive = false; };
    }, [themeId, baseUrl]);

    // 内容
    const content = useMemo(() => {
        const key = deckKey && deckModules[deckKey] ? deckKey : findDeckKey(deckKey);
        if (!key) {
            setErr(`找不到演示页: ${deckKey || '(未指定)'}`);
            return null;
        }
        const entry = deckModules[key];
        return entry.slides ? entry.slides() : null;
    }, [deckKey]);

    return (
        <div className="hxppt">
            <div className="hxppt__bar">
                <span className="hxppt__name">{deckKey.split('/').pop()?.replace(/\.tsx$/i, '') || '演示页'}</span>
                <span className="hxppt__spacer" />
                {/* 按需求: 页面里不放主题下拉, 主题由 URL 的 ?theme= 决定 */}
            </div>

            {err ? <div className="hxppt__err">{err}</div> : null}

            <div className="hxppt__body">
                {content ? (
                    <Deck theme={t} index={initialPage} syncUrl showBrand={false} fill>
                        {content}
                    </Deck>
                ) : null}
            </div>
        </div>
    );
}