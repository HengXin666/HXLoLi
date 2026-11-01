import React, { useEffect, useMemo, useRef, useState } from 'react';
import type { ANiMeRecord } from '@site/src/utils/anime/types';
import { searchGraph } from './buildGraph';
import styles from './graph.module.css';

// 关系图搜索: 番剧与角色, 选中即进入该番剧所在系列的聚焦层. "/" 聚焦输入框, 上下键选择, 回车确认, Esc 清空

interface Props {
    records: readonly ANiMeRecord[];
    preferCn: boolean;
    onPick: (animeId: number) => void;
}

export const GraphSearch: React.FC<Props> = ({ records, preferCn, onPick }) => {
    const [query, setQuery] = useState('');
    const [active, setActive] = useState(0);
    const [open, setOpen] = useState(false);
    const inputRef = useRef<HTMLInputElement>(null);

    const hits = useMemo(() => searchGraph(records, query, preferCn), [records, query, preferCn]);

    useEffect(() => { setActive(0); }, [query]);

    useEffect(() => {
        const onKey = (e: KeyboardEvent) => {
            if (e.key !== '/' || e.ctrlKey || e.metaKey || e.altKey) return;
            const el = e.target as HTMLElement | null;
            if (el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.tagName === 'SELECT' || el.isContentEditable)) return;
            e.preventDefault();
            inputRef.current?.focus();
        };
        window.addEventListener('keydown', onKey);
        return () => window.removeEventListener('keydown', onKey);
    }, []);

    const pick = (index: number) => {
        const hit = hits[index];
        if (!hit) return;
        onPick(hit.animeId);
        setQuery('');
        setOpen(false);
        inputRef.current?.blur();
    };

    const onKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === 'ArrowDown') {
            e.preventDefault();
            setActive(i => Math.min(i + 1, Math.max(hits.length - 1, 0)));
        } else if (e.key === 'ArrowUp') {
            e.preventDefault();
            setActive(i => Math.max(i - 1, 0));
        } else if (e.key === 'Enter') {
            e.preventDefault();
            pick(active);
        } else if (e.key === 'Escape') {
            e.stopPropagation();
            setQuery('');
            setOpen(false);
            inputRef.current?.blur();
        }
    };

    const showList = open && query.trim() !== '';

    return (
        <div className={styles.search}>
            <input
                ref={inputRef}
                className={styles.searchInput}
                type="search"
                placeholder="搜索番剧或角色  /"
                value={query}
                onChange={e => { setQuery(e.target.value); setOpen(true); }}
                onFocus={() => setOpen(true)}
                onBlur={() => window.setTimeout(() => setOpen(false), 120)}
                onKeyDown={onKeyDown}
                aria-label="搜索番剧或角色"
                aria-expanded={showList}
                aria-controls="anime-graph-search-list"
                role="combobox"
                aria-autocomplete="list"
            />
            {showList && (
                <ul id="anime-graph-search-list" className={styles.searchList} role="listbox">
                    {hits.length === 0 && <li className={styles.searchEmpty}>没有匹配的番剧或角色</li>}
                    {hits.map((hit, i) => (
                        <li
                            key={hit.key}
                            role="option"
                            aria-selected={i === active}
                            className={i === active ? styles.searchItemActive : styles.searchItem}
                            onMouseDown={e => { e.preventDefault(); pick(i); }}
                            onMouseEnter={() => setActive(i)}
                        >
                            <span className={styles.searchTitle}>{hit.title}</span>
                            <span className={styles.searchSub}>{hit.subtitle}</span>
                        </li>
                    ))}
                </ul>
            )}
        </div>
    );
};
