import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { FaApple, FaLinux, FaWindows } from 'react-icons/fa';

import { useHoverCard } from '../../common/Tooltip';
import { BRAND_COLORS, FALLBACK_COLORS } from './colors';
import styles from './pulse.module.css';
import {
    CODE_COLOR,
    DEVICE_COLOR,
    OS_COLORS,
    OS_ORDER,
    type WakaDay,
    fmtHours,
    fmtLong,
    loadWakaDays,
    topEntries,
} from './wakaData';

type Mode = 'code' | 'device' | 'both';

const MODES: { id: Mode; label: string }[] = [
    { id: 'code', label: '编码时长' },
    { id: 'device', label: '使用设备' },
    { id: 'both', label: '叠加' },
];

const WINDOWS = [
    { days: 30, label: '30 天' },
    { days: 90, label: '90 天' },
    { days: 365, label: '一年' },
] as const;

const WEEKS = 53;

const OS_ICON: Record<string, React.ReactNode> = {
    Linux: <FaLinux aria-hidden="true" />,
    Windows: <FaWindows aria-hidden="true" />,
    Mac: <FaApple aria-hidden="true" />,
};

type Cell = { date: string; day: WakaDay | null; future: boolean; code: number; device: number; codeLv: number; devLv: number };

function langColor (name: string, index: number): string {
    return BRAND_COLORS[name] ?? FALLBACK_COLORS[index % FALLBACK_COLORS.length];
}

function dateKey (d: Date): string {
    return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, '0')}-${String(d.getDate()).padStart(2, '0')}`;
}

/** 分位数分档: 极端长天不会把其余格子压成最浅一档 */
function leveler (values: number[]): (v: number) => number {
    const sorted = values.filter((v) => v > 0).sort((a, b) => a - b);
    const q = (p: number) => sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * p))] ?? 0;
    const cuts = [q(0.25), q(0.5), q(0.75)];
    return (v) => (v <= 0 ? 0 : v <= cuts[0] ? 1 : v <= cuts[1] ? 2 : v <= cuts[2] ? 3 : 4);
}

function CellCard ({ cell }: { cell: Cell }): React.ReactElement {
    const day = cell.day;
    const langs = day ? topEntries(day.langs, 3) : [];
    const os = day ? OS_ORDER.filter((k) => (day.os.get(k) ?? 0) > 0) : [];
    return (
        <div className={styles.card}>
            <strong>{cell.date}</strong>
            <div className={styles.cardRow}>
                <i style={{ background: CODE_COLOR }} />编码 <b>{fmtLong(cell.code)}</b>
            </div>
            <div className={styles.cardRow}>
                <i style={{ background: DEVICE_COLOR }} />设备 <b>{fmtLong(cell.device)}</b>
            </div>
            {os.length > 0 && (
                <div className={styles.cardMeta}>
                    {os.map((k) => (
                        <span key={k}>{OS_ICON[k]} {k} {fmtHours(day!.os.get(k) ?? 0)}</span>
                    ))}
                </div>
            )}
            {langs.length > 0 && (
                <div className={styles.cardMeta}>
                    {langs.map(([k, v]) => <span key={k}>{k} {fmtHours(v)}</span>)}
                </div>
            )}
            {!day && <div className={styles.cardMeta}>这天没有记录</div>}
        </div>
    );
}

/**
 * 首页的 WakaTime 摘要. 两个维度: 编码时长 (languages 之和) 与使用设备 (system 之和, 按系统拆).
 * 热力格可以只看一个维度, 也可以叠加: 叠加时格子底色是设备时长, 中间的方块是编码时长.
 * 悬停 / 聚焦格子出统一悬浮卡片; 热力格本身可聚焦, 方向键逐天移动.
 * 设备/编码两个维度的口径见同目录 index.tsx 顶部注释
 */
export default function WakaPulse (): React.ReactElement {
    const [days, setDays] = useState<WakaDay[] | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [mode, setMode] = useState<Mode>('both');
    const [windowDays, setWindowDays] = useState<number>(90);
    const [cursor, setCursor] = useState<number>(-1);
    const card = useHoverCard({ width: 'auto' });
    const cellRefs = useRef<(HTMLElement | null)[]>([]);

    useEffect(() => {
        let alive = true;
        loadWakaDays()
            .then((list) => alive && setDays(list))
            .catch((e) => alive && setError(String(e?.message ?? e)));
        return () => {
            alive = false;
        };
    }, []);

    const grid = useMemo(() => {
        if (!days?.length) return null;
        const byDate = new Map(days.map((d) => [d.date, d]));
        const last = new Date(`${days[days.length - 1].date}T00:00:00`);
        const end = new Date(last);
        end.setDate(end.getDate() + (6 - end.getDay()));
        const start = new Date(end);
        start.setDate(start.getDate() - WEEKS * 7 + 1);
        const raw: Omit<Cell, 'codeLv' | 'devLv'>[] = [];
        for (let d = new Date(start); d <= end; d.setDate(d.getDate() + 1)) {
            const key = dateKey(d);
            const day = byDate.get(key) ?? null;
            raw.push({ date: key, day, future: d > last, code: day?.code ?? 0, device: day?.device ?? 0 });
        }
        const codeLv = leveler(raw.map((c) => c.code));
        const devLv = leveler(raw.map((c) => c.device));
        const cells: Cell[] = raw.map((c) => ({ ...c, codeLv: codeLv(c.code), devLv: devLv(c.device) }));
        const yearCode = cells.reduce((s, c) => s + c.code, 0);
        const yearDevice = cells.reduce((s, c) => s + c.device, 0);
        const active = cells.filter((c) => c.code > 0).length;
        let streak = 0;
        for (let i = days.length - 1; i >= 0 && days[i].code > 0; i--) streak++;
        return { cells, yearCode, yearDevice, active, streak };
    }, [days]);

    const shares = useMemo(() => {
        if (!days?.length) return { langs: [], os: [] };
        const slice = days.slice(-windowDays);
        const langTotals = new Map<string, number>();
        const osTotals = new Map<string, number>();
        for (const d of slice) {
            for (const [k, v] of d.langs) langTotals.set(k, (langTotals.get(k) ?? 0) + v);
            for (const [k, v] of d.os) osTotals.set(k, (osTotals.get(k) ?? 0) + v);
        }
        const toShares = (m: Map<string, number>, n: number, color: (k: string, i: number) => string) => {
            const sorted = [...m.entries()].sort((a, b) => b[1] - a[1]);
            const sum = sorted.reduce((s, [, v]) => s + v, 0) || 1;
            const top = sorted.slice(0, n).map(([name, sec], i) => ({ name, sec, share: sec / sum, color: color(name, i) }));
            const rest = sorted.slice(n).reduce((s, [, v]) => s + v, 0);
            if (rest > 0) top.push({ name: '其他', sec: rest, share: rest / sum, color: '#555' });
            return top;
        };
        return {
            langs: toShares(langTotals, 6, langColor),
            os: toShares(osTotals, 3, (k) => OS_COLORS[k] ?? '#888'),
        };
    }, [days, windowDays]);

    const showCell = useCallback(
        (index: number) => {
            const cell = grid?.cells[index];
            const el = cellRefs.current[index];
            if (!cell || cell.future || !el) return;
            setCursor(index);
            card.open(el, <CellCard cell={cell} />);
        },
        [grid, card],
    );

    const onKey = (e: React.KeyboardEvent<HTMLDivElement>) => {
        if (!grid) return;
        const step = { ArrowUp: -1, ArrowDown: 1, ArrowLeft: -7, ArrowRight: 7 }[e.key];
        if (e.key === 'Escape') {
            card.close();
            return;
        }
        if (step === undefined) return;
        e.preventDefault();
        const lastReal = grid.cells.findLastIndex((c) => !c.future);
        const from = cursor < 0 ? lastReal : cursor;
        showCell(Math.min(lastReal, Math.max(0, from + step)));
    };

    if (error) return <p className={styles.note}>WakaTime 数据没取到 ({error}).</p>;
    if (!grid) return <div className={styles.skeleton} aria-busy="true" />;

    const showCode = mode !== 'device';
    const showDevice = mode !== 'code';

    return (
        <div className={styles.wrap} data-mode={mode}>
            <div className={styles.top}>
                <dl className={styles.kpis}>
                    <div data-dim="code"><dt>近一年编码</dt><dd>{fmtHours(grid.yearCode)}</dd></div>
                    <div data-dim="device"><dt>设备在线</dt><dd>{fmtHours(grid.yearDevice)}</dd></div>
                    <div><dt>编码占在线</dt><dd>{grid.yearDevice ? ((grid.yearCode / grid.yearDevice) * 100).toFixed(1) : '0'}%</dd></div>
                    <div><dt>连续编码</dt><dd>{grid.streak} 天</dd></div>
                </dl>
                <div className={styles.seg} role="radiogroup" aria-label="热力格显示的维度">
                    {MODES.map((m) => (
                        <button
                            key={m.id}
                            type="button"
                            role="radio"
                            aria-checked={mode === m.id}
                            className={mode === m.id ? styles.segOn : styles.segBtn}
                            onClick={() => setMode(m.id)}
                        >
                            {m.label}
                        </button>
                    ))}
                </div>
            </div>

            <div className={styles.heatScroll}>
                <div
                    className={styles.heat}
                    role="grid"
                    aria-label={`近一年每日${mode === 'code' ? '编码时长' : mode === 'device' ? '设备在线时长' : '编码与设备时长'}, 方向键逐天查看`}
                    aria-describedby={card.isOpen ? card.id : undefined}
                    tabIndex={0}
                    onKeyDown={onKey}
                    onBlur={() => card.close()}
                    onPointerLeave={() => card.close()}
                >
                    {grid.cells.map((c, i) => (
                        <i
                            key={c.date}
                            ref={(el) => {
                                cellRefs.current[i] = el;
                            }}
                            className={styles.cell}
                            data-future={c.future || undefined}
                            data-active={cursor === i && card.isOpen ? 'true' : undefined}
                            data-dev={showDevice ? c.devLv : undefined}
                            data-code={mode === 'code' ? c.codeLv : undefined}
                            onPointerEnter={() => showCell(i)}
                        >
                            {mode === 'both' && c.codeLv > 0 && <b data-code={c.codeLv} />}
                        </i>
                    ))}
                </div>
            </div>
            <div className={styles.scale} aria-hidden="true">
                {showDevice && (
                    <span className={styles.scaleGroup}>
                        设备 <i data-dev="1" /><i data-dev="2" /><i data-dev="3" /><i data-dev="4" />
                    </span>
                )}
                {showCode && (
                    <span className={styles.scaleGroup}>
                        编码 <i data-code="1" /><i data-code="2" /><i data-code="3" /><i data-code="4" />
                    </span>
                )}
                <span className={styles.scaleHint}>{mode === 'both' ? '底色 = 设备, 中间方块 = 编码' : '颜色越深时间越长'}</span>
            </div>

            <div className={styles.langHead}>
                <span>{mode === 'device' ? '设备占比' : mode === 'code' ? '语言占比' : '设备与语言占比'}</span>
                <div className={styles.seg} role="radiogroup" aria-label="统计区间">
                    {WINDOWS.map((w) => (
                        <button
                            key={w.days}
                            type="button"
                            role="radio"
                            aria-checked={windowDays === w.days}
                            className={windowDays === w.days ? styles.segOn : styles.segBtn}
                            onClick={() => setWindowDays(w.days)}
                        >
                            {w.label}
                        </button>
                    ))}
                </div>
            </div>

            {showDevice && (
                <div className={styles.shareBlock}>
                    <div className={styles.stack} aria-hidden="true">
                        {shares.os.map((l) => <span key={l.name} style={{ width: `${l.share * 100}%`, background: l.color }} />)}
                    </div>
                    <ul className={styles.langList}>
                        {shares.os.map((l) => (
                            <li key={l.name}>
                                <i style={{ background: l.color }} />
                                <span className={styles.langName}>{OS_ICON[l.name]} {l.name}</span>
                                <span className={styles.langVal}>{fmtHours(l.sec)}</span>
                                <span className={styles.langPct}>{(l.share * 100).toFixed(1)}%</span>
                            </li>
                        ))}
                    </ul>
                </div>
            )}
            {showCode && (
                <div className={styles.shareBlock}>
                    <div className={styles.stack} aria-hidden="true">
                        {shares.langs.map((l) => <span key={l.name} style={{ width: `${l.share * 100}%`, background: l.color }} />)}
                    </div>
                    <ul className={styles.langList}>
                        {shares.langs.map((l) => (
                            <li key={l.name}>
                                <i style={{ background: l.color }} />
                                <span className={styles.langName}>{l.name}</span>
                                <span className={styles.langVal}>{fmtHours(l.sec)}</span>
                                <span className={styles.langPct}>{(l.share * 100).toFixed(1)}%</span>
                            </li>
                        ))}
                    </ul>
                </div>
            )}
            {card.element}
        </div>
    );
}
