import React, { useCallback, useEffect, useId, useRef, useState } from 'react';
import { FloatingLayer } from './index';
import styles from './Select.module.css';

export interface SelectOption<T extends string | number> {
  value: T;
  label: React.ReactNode;
  /** 右侧淡色附注, 例如部数 */
  hint?: React.ReactNode;
  disabled?: boolean;
}

export interface SelectProps<T extends string | number> {
  value: T;
  options: readonly SelectOption<T>[];
  onChange: (value: T) => void;
  /** 读屏用的名字 (没有可见 label 时必填) */
  ariaLabel?: string;
  /** 指向可见 label 元素的 id */
  ariaLabelledBy?: string;
  className?: string;
  /** 列表最多显示的高度, 超出滚动; 默认 280 */
  maxHeight?: number;
}

/**
 * 替代原生 <select> 的单选下拉. 弹层复用 common/Tooltip 的 FloatingLayer (fixed 定位、上下翻转、视口内夹取),
 * 不另起一套定位. 按 WAI-ARIA 的 select-only combobox 模式: 焦点始终留在按钮上, 列表用 aria-activedescendant 指当前项.
 * 键盘: 方向上下移动 (关着时打开), Home / End 到首尾, Enter / 空格选中或打开, Esc 关闭, Tab 关闭并照常移走焦点,
 * 输入字符按标签首字跳转. 点外部关闭; 超长列表滚动且当前项自动滚入视野.
 * 在模态 <dialog> 里使用时列表挂进那个 dialog (顶层之外的元素是 inert 的).
 * .agents/notes/implemented/feature/2026-11-01-anime-custom-select.md
 */
export default function Select<T extends string | number>({
  value, options, onChange, ariaLabel, ariaLabelledBy, className = '', maxHeight = 280,
}: SelectProps<T>): React.ReactElement {
  const uid = useId().replace(/:/g, '');
  const listId = `hx-sel-${uid}`;
  const optId = (i: number) => `${listId}-o${i}`;
  const btnRef = useRef<HTMLButtonElement | null>(null);
  const cardRef = useRef<HTMLDivElement | null>(null);
  const listRef = useRef<HTMLUListElement | null>(null);
  const typed = useRef<{ text: string; at: number }>({ text: '', at: 0 });
  const [open, setOpen] = useState(false);
  const selectedIndex = Math.max(0, options.findIndex((o) => o.value === value));
  const [active, setActive] = useState(selectedIndex);
  const [anchor, setAnchor] = useState<HTMLButtonElement | null>(null);

  const selected = options.find((o) => o.value === value);

  const step = useCallback((from: number, dir: 1 | -1) => {
    for (let k = 1; k <= options.length; k++) {
      const i = (from + dir * k + options.length) % options.length;
      if (!options[i].disabled) return i;
    }
    return from;
  }, [options]);

  const openList = useCallback((at?: number) => {
    setActive(at ?? selectedIndex);
    setOpen(true);
  }, [selectedIndex]);

  const choose = useCallback((i: number) => {
    const o = options[i];
    if (!o || o.disabled) return;
    setOpen(false);
    btnRef.current?.focus();
    if (o.value !== value) onChange(o.value);
  }, [options, value, onChange]);

  // 点外部关闭
  useEffect(() => {
    if (!open) return undefined;
    const onDown = (e: PointerEvent) => {
      const t = e.target as Node;
      if (btnRef.current?.contains(t) || cardRef.current?.contains(t)) return;
      setOpen(false);
    };
    document.addEventListener('pointerdown', onDown, true);
    return () => document.removeEventListener('pointerdown', onDown, true);
  }, [open]);

  // 当前项滚入视野
  useEffect(() => {
    if (!open) return;
    const el = listRef.current?.querySelector<HTMLElement>(`#${CSS.escape(optId(active))}`);
    el?.scrollIntoView({ block: 'nearest' });
  }, [open, active]); // eslint-disable-line react-hooks/exhaustive-deps

  const typeahead = (ch: string) => {
    const now = Date.now();
    const t = typed.current;
    t.text = now - t.at > 600 ? ch : t.text + ch;
    t.at = now;
    const needle = t.text.toLowerCase();
    const start = open ? active : selectedIndex;
    for (let k = 1; k <= options.length; k++) {
      const i = (start + k) % options.length;
      const o = options[i];
      const text = typeof o.label === 'string' || typeof o.label === 'number' ? String(o.label) : String(o.value);
      if (!o.disabled && text.toLowerCase().startsWith(needle)) {
        if (open) setActive(i);
        else if (o.value !== value) onChange(o.value);
        return;
      }
    }
  };

  const onKeyDown = (e: React.KeyboardEvent<HTMLButtonElement>) => {
    switch (e.key) {
      case 'ArrowDown':
        e.preventDefault();
        if (!open) openList();
        else setActive((i) => step(i, 1));
        return;
      case 'ArrowUp':
        e.preventDefault();
        if (!open) openList();
        else setActive((i) => step(i, -1));
        return;
      case 'Home':
        if (!open) return;
        e.preventDefault();
        setActive(step(options.length - 1, 1));
        return;
      case 'End':
        if (!open) return;
        e.preventDefault();
        setActive(step(0, -1));
        return;
      case 'Enter':
      case ' ':
        e.preventDefault();
        if (open) choose(active);
        else openList();
        return;
      case 'Escape':
        if (open) {
          // 在 <dialog> 里时只关列表, 不顺带关掉 dialog
          e.preventDefault();
          e.stopPropagation();
          setOpen(false);
        }
        return;
      case 'Tab':
        if (open) setOpen(false);
        return;
      default:
        if (e.key.length === 1 && !e.metaKey && !e.ctrlKey && !e.altKey) typeahead(e.key);
    }
  };

  const list = (
    <ul
      ref={listRef}
      id={listId}
      role="listbox"
      aria-labelledby={ariaLabelledBy}
      aria-label={ariaLabelledBy ? undefined : ariaLabel}
      className={styles.list}
      style={{ maxHeight }}
      tabIndex={-1}
    >
      {options.map((o, i) => (
        <li
          key={String(o.value)}
          id={optId(i)}
          role="option"
          aria-selected={o.value === value}
          aria-disabled={o.disabled || undefined}
          data-active={i === active ? 'true' : undefined}
          className={styles.option}
          // 不抢按钮的焦点
          onMouseDown={(e) => e.preventDefault()}
          onPointerMove={() => { if (!o.disabled && i !== active) setActive(i); }}
          onClick={() => choose(i)}
        >
          <span className={styles.check} aria-hidden>{o.value === value ? '✓' : ''}</span>
          <span className={styles.label}>{o.label}</span>
          {o.hint !== undefined && <span className={styles.hint}>{o.hint}</span>}
        </li>
      ))}
    </ul>
  );

  return (
    <>
      <button
        ref={(el) => { btnRef.current = el; setAnchor(el); }}
        type="button"
        role="combobox"
        aria-haspopup="listbox"
        aria-expanded={open}
        aria-controls={listId}
        aria-activedescendant={open ? optId(active) : undefined}
        aria-label={ariaLabelledBy ? undefined : ariaLabel}
        aria-labelledby={ariaLabelledBy}
        className={`${styles.trigger} ${className}`.trim()}
        data-open={open ? 'true' : undefined}
        onClick={() => (open ? setOpen(false) : openList())}
        onKeyDown={onKeyDown}
      >
        <span className={styles.value}>{selected?.label ?? ''}</span>
        <svg className={styles.chevron} viewBox="0 0 24 24" width="14" height="14" aria-hidden>
          <path fill="currentColor" d="M7 10l5 5 5-5z" />
        </svg>
      </button>
      {open && anchor && (
        <FloatingLayer
          id={`${listId}-layer`}
          anchor={anchor}
          content={list}
          role="presentation"
          prefer="bottom"
          align="start"
          arrow={false}
          interactive
          width="auto"
          cardRef={cardRef}
          className={styles.layer}
          style={{ padding: 4, minWidth: anchor.offsetWidth, maxWidth: 360 }}
          container={anchor.closest('dialog') ?? undefined}
        />
      )}
    </>
  );
}
