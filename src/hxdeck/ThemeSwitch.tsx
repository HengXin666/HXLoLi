import React, { useEffect, useState } from 'react';
import { listThemes, getTheme, loadTheme, loadManifest, loadThemeFile, registerTheme } from './theme/registry';
import { readThemeFile } from './theme/serialize';
import type { DeckTheme } from './theme/types';

/**
 * 前端主题切换器.
 *
 * 允许读者自己换主题 —— 因为主题只是数据, 换主题不涉及任何代码路径变化.
 * 顺序: 已注册 -> 按名加载 (static/themes/*) -> 用户当场导入文件.
 */
export function ThemeSwitch({
    value,
    onChange,
    allowUpload = true,
}: {
    value: string;
    onChange: (theme: DeckTheme, id: string) => void;
    allowUpload?: boolean;
}): React.ReactElement {
    const [items, setItems] = useState(() => listThemes());
    const [busy, setBusy] = useState(false);

    // 首次挂载: 拉取主题清单, 于是"用户放进 static/themes 的主题"也会出现
    useEffect(() => {
        let alive = true;
        loadManifest().then((list) => { if (alive) setItems(list); });
        return () => { alive = false; };
    }, []);

    useEffect(() => {
        setItems(listThemes());
    }, [value]);

    const pick = async (id: string) => {
        const hit = getTheme(id);
        if (hit) {
            onChange(hit, id);
            return;
        }
        setBusy(true);
        // 清单里有文件名就按文件加载, 否则按 <id>.yaml 猜
        const info = items.find((t) => t.id === id);
        const loaded = info?.file ? await loadThemeFile(info.file) : await loadTheme(id);
        setBusy(false);
        if (loaded) {
            onChange(loaded, id);
            setItems(listThemes());
        }
    };

    return (
        <div className="hxd-themeswitch">
            {items.map((t) => (
                <button
                    key={t.id}
                    type="button"
                    data-on={t.id === value ? 'true' : undefined}
                    onClick={() => pick(t.id)}
                    title={t.builtin ? '内置主题' : '运行时加载'}
                >
                    {t.name}
                </button>
            ))}
            {allowUpload ? (
                <label className="hxd-themeswitch__up" title="导入主题文件">
                    +
                    <input
                        type="file"
                        accept=".json,.yaml,.yml"
                        onChange={async (e) => {
                            const f = e.target.files?.[0];
                            if (!f) return;
                            try {
                                const t = await readThemeFile(f);
                                registerTheme(t);
                                setItems(listThemes());
                                onChange(t, t.id);
                            } catch {
                                // 静默: 无效文件不打断演示
                            }
                        }}
                    />
                </label>
            ) : null}
            {busy ? <span className="hxd-themeswitch__busy">加载中…</span> : null}
        </div>
    );
}

export default ThemeSwitch;