import React, { useEffect, useRef, useState } from 'react';
import { listThemes, getTheme, loadTheme, loadManifest, loadThemeFile, registerTheme } from './theme/registry';
import type { ThemeInfo } from './theme/registry';
import { readThemeFile } from './theme/serialize';
import type { DeckTheme } from './theme/types';

/**
 * 主题选择器 (下拉).
 *
 * 交互顺序刻意如此:
 *   1. 先列出**本地已有**的主题 (static/themes 下的全部 + 内置)
 *   2. 最后一行才是"导入主题文件…"
 * 之前直接把"导入"摆在外面, 用户第一眼看到的是导入动作, 而不是自己已有的主题 —— 那是反的.
 */
export function ThemePicker({
    value,
    onChange,
    label = '主题',
}: {
    value: string;
    onChange: (theme: DeckTheme, id: string) => void;
    label?: string;
}): React.ReactElement {
    const [items, setItems] = useState<ThemeInfo[]>([]);
    const [open, setOpen] = useState(false);
    const [busy, setBusy] = useState(false);
    const [err, setErr] = useState('');
    const boxRef = useRef<HTMLDivElement>(null);
    const fileRef = useRef<HTMLInputElement>(null);

    // 先加载清单 -> 于是"用户放进 static/themes 的全部主题"都会被列出来
    useEffect(() => {
        let alive = true;
        loadManifest().then(() => {
            if (alive) setItems(listThemes());
        });
        return () => { alive = false; };
    }, []);

    useEffect(() => {
        if (!open) return;
        const onDoc = (e: MouseEvent) => {
            if (boxRef.current && !boxRef.current.contains(e.target as Node)) setOpen(false);
        };
        document.addEventListener('mousedown', onDoc);
        return () => document.removeEventListener('mousedown', onDoc);
    }, [open]);

    const local = items.filter((t) => t.file);
    const builtin = items.filter((t) => t.builtin && !t.file);
    const activeName = items.find((t) => t.id === value)?.name ?? value;

    const pick = async (t: ThemeInfo) => {
        setOpen(false);
        const hit = getTheme(t.id);
        if (hit) { onChange(hit, t.id); return; }
        setBusy(true);
        const got = t.file ? await loadThemeFile(t.file) : await loadTheme(t.id);
        setBusy(false);
        if (got) onChange(got, t.id);
    };

    return (
        <div className="hxpk" ref={boxRef}>
            <button type="button" className="hxpk__btn" onClick={() => setOpen((v) => !v)} aria-haspopup="listbox" aria-expanded={open}>
                <span className="hxpk__label">{label}</span>
                <b>{activeName}</b>
                <span className="hxpk__caret" aria-hidden="true">▾</span>
            </button>

            {open ? (
                <div className="hxpk__menu" role="listbox">
                    {local.length ? <div className="hxpk__group">我保存的主题</div> : <div className="hxpk__group">还没有保存的主题 —— 可在编辑器里做一个</div>}
                    {local.map((t) => (
                        <button key={t.id} type="button" role="option" aria-selected={t.id === value}
                            className="hxpk__item" data-on={t.id === value ? 'true' : undefined} onClick={() => pick(t)}>
                            {t.name}
                        </button>
                    ))}
                    {builtin.length ? <div className="hxpk__group">内置</div> : null}
                    {builtin.map((t) => (
                        <button key={t.id} type="button" role="option" aria-selected={t.id === value}
                            className="hxpk__item" data-on={t.id === value ? 'true' : undefined} onClick={() => pick(t)}>
                            {t.name}
                        </button>
                    ))}
                    <div className="hxpk__sep" />
                    <button type="button" className="hxpk__item hxpk__item--import" onClick={() => { setOpen(false); fileRef.current?.click(); }}>
                        导入主题文件…
                    </button>
                </div>
            ) : null}

            {busy ? <span className="hxpk__busy">载入中…</span> : null}
            {err ? <span className="hxpk__err">{err}</span> : null}

            <input ref={fileRef} type="file" accept=".json,.yaml,.yml" style={{ display: 'none' }}
                onChange={async (e) => {
                    const f = e.target.files?.[0];
                    if (!f) return;
                    try {
                        const t = await readThemeFile(f);
                        registerTheme(t);
                        setItems(listThemes());
                        onChange(t, t.id);
                        setErr('');
                    } catch (ex) {
                        setErr('文件无法识别');
                    }
                }} />
        </div>
    );
}

export default ThemePicker;