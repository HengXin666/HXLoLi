import React, { useMemo, useState } from 'react';
import {
    ResponsiveContainer,
    AreaChart,
    Area,
    BarChart,
    Bar,
    Cell,
    XAxis,
    YAxis,
    Tooltip,
    Brush,
    ReferenceLine,
    CartesianGrid,
} from 'recharts';
import type { RecordItem } from '@site/data/wordStats';
import { processWordCountData, type ProcessedData } from '../../../utils/wordCountUtils';

interface WordCountChartProps {
    rawData: RecordItem[];
    /** 初始只展示后 percent 的数据, 区间以索引计算 */
    initialPercent?: number;
    /** 上方累计图的高度, 下方增量条固定 72px */
    height?: number;
}

/** 与原图保持一致的四个颜色 */
const C_DOCS = '#F7CE45';
const C_AI = '#8CFF00';
const C_UP = '#FF61FF';
const C_DOWN = 'rgb(97, 255, 97)';
const AXIS = { fontSize: 11, fill: 'var(--ifm-color-emphasis-600)' };

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
function wan (val: number): string {
    if (Math.abs(val) < 10000) return `${val}`;
    return `${(val / 10000).toFixed(Math.abs(val) >= 1e6 ? 0 : 1).replace(/\.0$/, '')}万`;
}

/** 把数值抬到 1/2/2.5/5 的整倍数 */
function niceCeil (val: number): number {
    if (!(val > 0)) return 1;
    const exp: number = Math.floor(Math.log10(val));
    const base: number = Math.pow(10, exp);
    const step: number = [1, 2, 2.5, 5, 10].find(s => val <= s * base) ?? 10;
    return step * base;
}

function shortDate (d: string): string {
    // processWordCountData 按浏览器语言出串: zh 是 "2026-10-09 02:07:00", en-US 是 "10-09-2026, 02:07:00"
    const ymd = d.match(/(\d{4})-(\d{1,2})-(\d{1,2})/);
    if (ymd) return `${ymd[1]}-${ymd[2].padStart(2, '0')}`;
    const mdy = d.match(/(\d{1,2})-(\d{1,2})-(\d{4})/);
    return mdy ? `${mdy[3]}-${mdy[1].padStart(2, '0')}` : d;
}

const Tip = ({ active, payload }: any) => {
    if (!active || !payload?.length) return null;
    const p: ProcessedData = payload[0].payload;
    return (
        <div style={{
            background: 'rgba(18,18,18,0.94)', border: '1px solid var(--ifm-color-emphasis-300)',
            borderRadius: 8, padding: '8px 10px', fontSize: 12, lineHeight: 1.6, maxWidth: 280,
        }}>
            <div style={{ color: 'var(--ifm-color-emphasis-700)' }}>{p.date}</div>
            {p.message && <div style={{ marginBottom: 4 }}>{p.message}</div>}
            <div>总量 <b>{p.total.toLocaleString()}</b></div>
            <div style={{ color: C_DOCS }}>docs/blog {p.docsBlogTotal.toLocaleString()}</div>
            <div style={{ color: C_AI }}>ai-docs {p.aiDocsTotal.toLocaleString()}</div>
            <div style={{ color: p.delta >= 0 ? C_UP : C_DOWN }}>本次 {p.delta >= 0 ? '+' : ''}{p.delta.toLocaleString()}</div>
        </div>
    );
};

/**
 * 字数图拆成上下两块共用一条时间轴: 上面是 docs/blog + ai-docs 的堆叠面积 (顶边即总量),
 * 下面是每次提交的增量柱, 正负分色, 0 线居中可见. 不再有双纵轴, 也就没有两根轴 0 对不齐的问题.
 * .agents/notes/implemented/bug-fix/2026-10-31-word-count-chart-zero-baseline.md
 */
export default function WordCountChart ({
    rawData,
    initialPercent = 0.8,
    height = 200,
}: WordCountChartProps) {
    const data: ProcessedData[] = useMemo(() => processWordCountData(rawData), [rawData]);
    const n = data.length;
    const [range, setRange] = useState(() => ({
        startIndex: Math.max(Math.floor(n * (1 - initialPercent)), 0),
        endIndex: Math.max(n - 1, 0),
    }));

    const view = useMemo(() => data.slice(range.startIndex, range.endIndex + 1), [data, range]);

    const totalTop = useMemo(() => niceCeil(Math.max(1, ...view.map(d => d.total)) * 1.02), [view]);
    // 增量轴: 正负各自取整, 负向最多是正向的一半, 再深的删字由 allowDataOverflow 压底, tooltip 给真值
    const deltaDomain = useMemo((): [number, number] => {
        const max = niceCeil(Math.max(1, ...view.map(d => d.delta)));
        const min = Math.min(0, ...view.map(d => d.delta));
        return [min < 0 ? -Math.min(niceCeil(-min), max / 2) : 0, max];
    }, [view]);

    const last = data[n - 1];
    const first = view[0];
    const gained = last && first ? last.total - first.total + first.delta : 0;

    if (!n) return null;

    return (
        <div className="word-count-chart" style={{ userSelect: 'none' }}>
            <div style={{ display: 'flex', flexWrap: 'wrap', gap: '4px 20px', alignItems: 'baseline', marginBottom: 8, fontSize: 12, color: 'var(--ifm-color-emphasis-700)' }}>
                <span><b style={{ font: '600 20px/1.2 var(--ifm-font-family-monospace)', color: 'var(--ifm-font-color-base)' }}>{wan(last.total)}</b> 字</span>
                <span>区间内 <b style={{ color: C_UP }}>+{wan(gained)}</b></span>
                <span><i style={{ display: 'inline-block', width: 8, height: 8, borderRadius: 2, background: C_DOCS, marginRight: 4 }} />docs/blog</span>
                <span><i style={{ display: 'inline-block', width: 8, height: 8, borderRadius: 2, background: C_AI, marginRight: 4 }} />ai-docs</span>
            </div>

            <ResponsiveContainer width="100%" height={height}>
                <AreaChart data={view} margin={{ top: 4, right: 4, bottom: 0, left: 0 }}>
                    <defs>
                        <linearGradient id="wcDocs" x1="0" y1="0" x2="0" y2="1">
                            <stop offset="0%" stopColor={C_DOCS} stopOpacity={0.45} />
                            <stop offset="100%" stopColor={C_DOCS} stopOpacity={0.05} />
                        </linearGradient>
                        <linearGradient id="wcAi" x1="0" y1="0" x2="0" y2="1">
                            <stop offset="0%" stopColor={C_AI} stopOpacity={0.6} />
                            <stop offset="100%" stopColor={C_AI} stopOpacity={0.15} />
                        </linearGradient>
                    </defs>
                    <CartesianGrid vertical={false} stroke="rgba(255,255,255,0.06)" />
                    <XAxis dataKey="date" hide />
                    <YAxis
                        width={44}
                        domain={[0, totalTop]}
                        ticks={[0, totalTop / 4, totalTop / 2, (totalTop * 3) / 4, totalTop]}
                        tickFormatter={wan}
                        tick={AXIS}
                        axisLine={false}
                        tickLine={false}
                    />
                    <Tooltip content={<Tip />} cursor={{ stroke: 'rgba(255,255,255,0.25)' }} />
                    <Area type="monotone" dataKey="docsBlogTotal" stackId="t" stroke={C_DOCS} strokeWidth={1.5} fill="url(#wcDocs)" isAnimationActive={false} />
                    <Area type="monotone" dataKey="aiDocsTotal" stackId="t" stroke={C_AI} strokeWidth={1.5} fill="url(#wcAi)" isAnimationActive={false} />
                </AreaChart>
            </ResponsiveContainer>

            <ResponsiveContainer width="100%" height={96}>
                <BarChart data={data} margin={{ top: 4, right: 4, bottom: 0, left: 0 }}>
                    <XAxis dataKey="date" tickFormatter={shortDate} tick={AXIS} minTickGap={48} axisLine={false} tickLine={false} />
                    <YAxis
                        width={44}
                        domain={deltaDomain}
                        ticks={[deltaDomain[0], 0, deltaDomain[1]].filter((v, i, a) => a.indexOf(v) === i)}
                        tickFormatter={wan}
                        allowDataOverflow
                        tick={AXIS}
                        axisLine={false}
                        tickLine={false}
                    />
                    <Tooltip content={<Tip />} cursor={{ fill: 'rgba(255,255,255,0.06)' }} />
                    <ReferenceLine y={0} stroke="rgba(255,255,255,0.25)" />
                    <Bar dataKey="delta" isAnimationActive={false} minPointSize={1}>
                        {data.map((d) => (
                            <Cell key={d.commit + d.date} fill={d.delta >= 0 ? C_UP : C_DOWN} />
                        ))}
                    </Bar>
                    <Brush
                        dataKey="date"
                        height={14}
                        stroke="var(--ifm-color-primary)"
                        fill="transparent"
                        travellerWidth={8}
                        tickFormatter={shortDate}
                        startIndex={range.startIndex}
                        endIndex={range.endIndex}
                        onChange={(r: any) => {
                            if (r?.startIndex != null && r?.endIndex != null) setRange({ startIndex: r.startIndex, endIndex: r.endIndex });
                        }}
                    />
                </BarChart>
            </ResponsiveContainer>
        </div>
    );
}
