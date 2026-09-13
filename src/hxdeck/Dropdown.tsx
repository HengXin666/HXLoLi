import React, { useEffect, useRef, useState } from 'react';
import styles from './PptCard.module.css';

/**
 * 自绘下拉选择器.
 *
 * 为什么不用原生 <select>:
 *   能改的只有"闭合态"那一小块; **展开后的选项列表由浏览器原生渲染**,
 *   CSS 完全碰不到 —— 于是不管怎么调, 点开都还是系统菜单的样子.
 *   要真正符合站点风格, 只能自己画.
 *
 * 交互照抄原生 <select> 的可用性:
 *   · 点击展开 / 点击外部收起
 *   · ↑↓ 移动选项, Enter 选中, Esc 收起
 *   · 当前项打勾
 */
export interface DropdownOption {
    value: string;
    label: string;
}

export function Dropdown({
    value,
    options,
    onChange,
    label,
    title,
    className,
}: {
    value: string;
    options: DropdownOption[];
    onChange: (v: string) => void;
    /** 无障碍标签 */
    label?: string;
    title?: string;
    className?: string;
}): React.ReactElement {
    const [open, setOpen] = useState(false);
    const [active, setActive] = useState(() => Math.max(0, options.findIndex((o) => o.value === value)));
    const rootRef = useRef<HTMLSpanElement>(null);

    const current = options.find((o) => o.value === value) ?? options[0];

    // 点击外部 / Esc 收起
    useEffect(() => {
        if (!open) return;
        const onDoc = (e: MouseEvent) => {
            if (rootRef.current && !rootRef.current.contains(e.target as Node)) setOpen(false);
        };
        document.addEventListener('mousedown', onDoc);
        return () => document.removeEventListener('mousedown', onDoc);
    }, [open]);

    const commit = (v: string) => { onChange(v); setOpen(false); };

    const onKey = (e: React.KeyboardEvent) => {
        if (e.key === 'Escape') { setOpen(false); return; }
        if (!open && (e.key === 'Enter' || e.key === ' ' || e.key === 'ArrowDown')) {
            e.preventDefault(); setOpen(true); return;
        }
        if (!open) return;
        if (e.key === 'ArrowDown') { e.preventDefault(); setActive((i) => Math.min(options.length - 1, i + 1)); }
        else if (e.key === 'ArrowUp') { e.preventDefault(); setActive((i) => Math.max(0, i - 1)); }
        else if (e.key === 'Enter') { e.preventDefault(); commit(options[active]?.value ?? value); }
        else if (e.key === 'Home') { e.preventDefault(); setActive(0); }
        else if (e.key === 'End') { e.preventDefault(); setActive(options.length - 1); }
    };

    return (
        <span className={[styles.dropdown, className].filter(Boolean).join(' ')} ref={rootRef}>
            <button
                type="button"
                className={styles.dropdownBtn}
                onClick={() => { setOpen((v) => !v); setActive(Math.max(0, options.findIndex((o) => o.value === value))); }}
                onKeyDown={onKey}
                aria-haspopup="listbox"
                aria-expanded={open}
                aria-label={label ?? title ?? '选择'}
                title={title ?? label}
            >
                <span className={styles.dropdownLabel}>{current?.label ?? ''}</span>
                <svg className={styles.dropdownArrow} width="10" height="6" viewBox="0 0 10 6" aria-hidden="true">
                    <path d="M1 1l4 4 4-4" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" />
                </svg>
            </button>

            {open ? (
                <span className={styles.dropdownMenu} role="listbox">
                    {options.map((o, i) => (
                        <button
                            key={o.value}
                            type="button"
                            role="option"
                            aria-selected={o.value === value}
                            className={styles.dropdownItem}
                            data-on={o.value === value ? 'true' : undefined}
                            data-active={i === active ? 'true' : undefined}
                            onMouseEnter={() => setActive(i)}
                            onClick={() => commit(o.value)}
                        >
                            <span>{o.label}</span>
                            {o.value === value ? <span className={styles.dropdownTick} aria-hidden="true">✓</span> : null}
                        </button>
                    ))}
                </span>
            ) : null}
        </span>
    );
}

export default Dropdown;
