import React, { type ReactNode, useEffect, useState } from 'react';
import {
    ResponsiveContainer, BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, LabelList, Cell,
} from 'recharts';

/**
 * Chart: 数据对比图, 基于 recharts (仓库已有依赖).
 *
 * 只吃数据: 不接受颜色, 尺寸, 样式. 颜色取 --ifm-color-primary, 跟随明暗.
 *   kind="bar"   横向条形, data: { name, value, label?, group?, muted? }
 *                group 相同的放在一起, 每组一个小标题; muted 表示数字没核对 (只见于摘要), 画成虚条
 *   kind="range" 区间条, data: { name, from, to, label? }, 比如官方价到现价
 * source 和 caption 必填. 数据顺序就是显示顺序, 组件不排序.
 */

export interface BarDatum { name: string; value: number; label?: string; group?: string; muted?: boolean }
export interface RangeDatum { name: string; from: number; to: number; label?: string }

export interface ChartProps {
    kind: 'bar' | 'range';
    data: BarDatum[] | RangeDatum[];
    /** 数值单位, 显示在坐标轴和提示里, 比如 "h" "%" "元" */
    unit?: string;
    /** 坐标轴最大值, 不填按数据自动 */
    max?: number;
    min?: number;
    source: ReactNode;
    caption: ReactNode;
}

const ROW = 34;

/** 坐标轴: 自己算整齐的刻度, 不交给 recharts 猜 (区间条时它会把 0 拉进来) */
function axis (vals: number[], min?: number, max?: number, zero = true): { domain: [number, number]; ticks: number[] } {
    let lo = min ?? (zero ? Math.min(0, ...vals) : Math.min(...vals));
    let hi = max ?? Math.max(...vals);
    const raw = (hi - lo) / 4;
    const mag = 10 ** Math.floor(Math.log10(raw || 1));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((x) => x >= raw) ?? raw;
    if (min === undefined) lo = Math.floor(lo / (zero ? step : mag)) * (zero ? step : mag);
    if (max === undefined) hi = Math.ceil(hi / step) * step;
    const ticks: number[] = [];
    for (let t = Math.ceil(lo / step) * step; t <= hi + 1e-9; t += step) ticks.push(Number(t.toFixed(6)));
    return { domain: [lo, hi], ticks };
}

function useMounted (): boolean {
    const [m, setM] = useState(false);
    useEffect(() => { setM(true); }, []);
    return m;
}

function fmt (v: number, unit?: string): string {
    const n = Number.isInteger(v) ? String(v) : v.toFixed(1).replace(/\.0$/, '');
    return unit ? `${n}${unit}` : n;
}

function Tip ({ active, payload, unit, kind }: any): ReactNode {
    if (!active || !payload?.length) return null;
    const d = payload[0].payload;
    return (
        <div className="hxkb-chart-tip">
            <div className="hxkb-chart-tip-name">{d.name}</div>
            {kind === 'range'
                ? <div>{fmt(d.from, unit)} → {fmt(d.to, unit)}</div>
                : <div>{d.label ?? fmt(d.value, unit)}{d.muted ? ' (未核对)' : ''}</div>}
        </div>
    );
}

function Bars ({ rows, unit, max, min }: { rows: BarDatum[]; unit?: string; max?: number; min?: number }): ReactNode {
    const data = rows.map((r) => ({ ...r, text: r.label ?? fmt(r.value, unit) }));
    return (
        <ResponsiveContainer width="100%" height={data.length * ROW + 28}>
            <BarChart data={data} layout="vertical" margin={{ top: 4, right: 96, bottom: 0, left: 0 }} barCategoryGap={8}>
                <CartesianGrid horizontal={false} strokeDasharray="2 6" className="hxkb-chart-grid" />
                <XAxis type="number" allowDataOverflow {...axis(rows.map((r) => r.value), min, max)} tickFormatter={(v) => fmt(v, unit)} tickLine={false} axisLine={false} />
                <YAxis type="category" dataKey="name" width={172} tickLine={false} axisLine={false} interval={0} />
                <Tooltip cursor={{ className: 'hxkb-chart-cursor' }} content={<Tip unit={unit} kind="bar" />} />
                <Bar dataKey="value" radius={[0, 4, 4, 0]} isAnimationActive={false}>
                    {data.map((d) => <Cell key={d.name} className={d.muted ? 'hxkb-chart-bar is-muted' : 'hxkb-chart-bar'} />)}
                    <LabelList dataKey="text" position="right" className="hxkb-chart-label" />
                </Bar>
            </BarChart>
        </ResponsiveContainer>
    );
}

function Ranges ({ rows, unit, max, min }: { rows: RangeDatum[]; unit?: string; max?: number; min?: number }): ReactNode {
    const data = rows.map((r) => ({
        ...r, span: [Math.min(r.from, r.to), Math.max(r.from, r.to)] as [number, number],
        text: r.label ?? `${fmt(r.from, unit)} → ${fmt(r.to, unit)}`,
    }));
    return (
        <ResponsiveContainer width="100%" height={data.length * ROW + 28}>
            <BarChart data={data} layout="vertical" margin={{ top: 4, right: 72, bottom: 0, left: 0 }} barCategoryGap={12}>
                <CartesianGrid horizontal={false} strokeDasharray="2 6" className="hxkb-chart-grid" />
                <XAxis type="number" allowDataOverflow {...axis(rows.flatMap((r) => [r.from, r.to]), min, max, false)} tickFormatter={(v) => fmt(v, unit)} tickLine={false} axisLine={false} />
                <YAxis type="category" dataKey="name" width={172} tickLine={false} axisLine={false} interval={0} />
                <Tooltip cursor={{ className: 'hxkb-chart-cursor' }} content={<Tip unit={unit} kind="range" />} />
                <Bar dataKey="span" radius={4} isAnimationActive={false}>
                    {data.map((d) => <Cell key={d.name} className="hxkb-chart-bar" />)}
                    <LabelList dataKey="text" position="right" className="hxkb-chart-label" />
                </Bar>
            </BarChart>
        </ResponsiveContainer>
    );
}

export function Chart ({ kind, data, unit, max, min, source, caption }: ChartProps): ReactNode {
    const mounted = useMounted();
    let body: ReactNode = null;
    if (mounted) {
        if (kind === 'range') {
            body = <Ranges rows={data as RangeDatum[]} unit={unit} max={max} min={min} />;
        } else {
            const rows = data as BarDatum[];
            const groups: string[] = [];
            rows.forEach((r) => { if (!groups.includes(r.group ?? '')) groups.push(r.group ?? ''); });
            body = groups.map((g) => (
                <div key={g} className="hxkb-chart-group">
                    {g ? <div className="hxkb-k">{g}</div> : null}
                    <Bars rows={rows.filter((r) => (r.group ?? '') === g)} unit={unit} max={max} min={min} />
                </div>
            ));
        }
    }
    const rowsN = data.length;
    return (
        <figure className="hxkb hxkb-chart">
            <div className="hxkb-chart-body" style={mounted ? undefined : { minHeight: rowsN * ROW + 28 }}>{body}</div>
            <figcaption>
                <span>{caption}</span>
                <span className="hxkb-chart-src">来源: {source}</span>
            </figcaption>
        </figure>
    );
}
