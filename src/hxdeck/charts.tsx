import React, { useMemo } from 'react';
import {
    ResponsiveContainer,
    PieChart,
    Pie as RePie,
    Cell,
    LineChart,
    Line,
    BarChart,
    Bar,
    XAxis,
    YAxis,
    CartesianGrid,
    Tooltip,
    Legend,
} from 'recharts';
import { useDeckTheme, seriesColors } from './theme/context';
import { useIsSlideActive } from './slide-state';

/**
 * 关键: recharts 依赖容器尺寸测量. 在 visibility:hidden 的幻灯片里测得 0,
 * 图表会静默不渲染. 所以非活动页必须先占位, 激活后再挂载.
 */
function useChartReady(): boolean {
    const active = useIsSlideActive();
    const [ready, setReady] = React.useState(false);
    React.useEffect(() => {
        if (!active) { setReady(false); return; }
        // 翻页动画期间容器尺寸仍在变, 等动画结束再挂载, 否则 recharts 测到 0 宽而静默不画.
        // 860ms 是 --hxd-motion-page; 留一点余量.
        const t = window.setTimeout(() => setReady(true), 120);
        return () => window.clearTimeout(t);
    }, [active]);
    return ready;
}

/** 统一 tooltip 外观: 用主题 surface/border, 不引第三方样式 */
function TooltipStyle() {
    const t = useDeckTheme();
    return (
        <style>{`
            .hxd-chart .recharts-default-tooltip {
                background: ${t.colors.bg} !important;
                border: 1px solid ${t.colors.border} !important;
                border-radius: ${t.shape.radiusSm} !important;
                color: ${t.colors.text} !important;
                font-family: ${t.fonts.body};
                font-size: 13px;
            }
            .hxd-chart .recharts-tooltip-label { color: ${t.colors.textMuted} !important; }
            .hxd-chart .recharts-cartesian-axis-tick text { fill: ${t.colors.textMuted}; font-size: 12px; }
            .hxd-chart .recharts-legend-item-text { color: ${t.colors.text} !important; font-size: 12px; }
        `}</style>
    );
}

export interface Datum {
    name: string;
    value: number;
}

/** 饼图 / 环形图 —— 占比类 */
export function PieChartBlock({
    data,
    height = 300,
    donut = true,
}: {
    data: Datum[];
    height?: number;
    donut?: boolean;
}): React.ReactElement {
    const theme = useDeckTheme();
    const colors = seriesColors(theme, data.length);
    const ready = useChartReady();
    return (
        <div className="hxd-chart" style={{ width: '100%', height }}>
            <TooltipStyle />
            {!ready ? null : (
            <ResponsiveContainer>
                <PieChart>
                    <RePie
                        data={data}
                        dataKey="value"
                        nameKey="name"
                        innerRadius={donut ? '52%' : 0}
                        outerRadius="78%"
                        paddingAngle={2}
                        stroke={theme.colors.bg}
                        strokeWidth={2}
                        animationDuration={900}
                    >
                        {data.map((_, i) => (
                            <Cell key={i} fill={colors[i % colors.length]} />
                        ))}
                    </RePie>
                    <Tooltip />
                    <Legend />
                </PieChart>
            </ResponsiveContainer>
            )}
        </div>
    );
}

/** 折线图 —— 趋势类 */
export function LineChartBlock({
    data,
    keys,
    height = 300,
    xKey = 'name',
}: {
    data: Record<string, string | number>[];
    keys: string[];
    height?: number;
    xKey?: string;
}): React.ReactElement {
    const theme = useDeckTheme();
    const colors = seriesColors(theme, keys.length);
    const ready = useChartReady();
    return (
        <div className="hxd-chart" style={{ width: '100%', height }}>
            <TooltipStyle />
            {!ready ? null : (
            <ResponsiveContainer>
                <LineChart data={data} margin={{ top: 8, right: 16, bottom: 4, left: -12 }}>
                    <CartesianGrid stroke={theme.colors.border} strokeDasharray="3 5" vertical={false} />
                    <XAxis dataKey={xKey} tickLine={false} axisLine={false} />
                    <YAxis tickLine={false} axisLine={false} />
                    <Tooltip />
                    <Legend />
                    {keys.map((k, i) => (
                        <Line
                            key={k}
                            type="monotone"
                            dataKey={k}
                            stroke={colors[i % colors.length]}
                            strokeWidth={2.5}
                            dot={{ r: 3, strokeWidth: 0, fill: colors[i % colors.length] }}
                            activeDot={{ r: 6 }}
                            animationDuration={1100}
                        />
                    ))}
                </LineChart>
            </ResponsiveContainer>
            )}
        </div>
    );
}

/** 柱状图 —— 对比类 */
export function BarChartBlock({
    data,
    keys,
    height = 300,
    xKey = 'name',
}: {
    data: Record<string, string | number>[];
    keys: string[];
    height?: number;
    xKey?: string;
}): React.ReactElement {
    const theme = useDeckTheme();
    const colors = seriesColors(theme, keys.length);
    const ready = useChartReady();
    return (
        <div className="hxd-chart" style={{ width: '100%', height }}>
            <TooltipStyle />
            {!ready ? null : (
            <ResponsiveContainer>
                <BarChart data={data} margin={{ top: 8, right: 16, bottom: 4, left: -12 }}>
                    <CartesianGrid stroke={theme.colors.border} strokeDasharray="3 5" vertical={false} />
                    <XAxis dataKey={xKey} tickLine={false} axisLine={false} />
                    <YAxis tickLine={false} axisLine={false} />
                    <Tooltip />
                    <Legend />
                    {keys.map((k, i) => (
                        <Bar
                            key={k}
                            dataKey={k}
                            fill={colors[i % colors.length]}
                            radius={[6, 6, 0, 0]}
                            animationDuration={900}
                        />
                    ))}
                </BarChart>
            </ResponsiveContainer>
            )}
        </div>
    );
}

export interface TreeNode {
    name: string;
    children?: TreeNode[];
    value?: number;
}

/** 树状图 (纯 CSS 递归渲染, 便于主题化与打印) */
export function Tree({ root }: { root: TreeNode }): React.ReactElement {
    const theme = useDeckTheme();
    const render = (node: TreeNode, depth: number, i: number): React.ReactElement => (
        <li key={`${depth}-${i}-${node.name}`} style={{ listStyle: 'none', position: 'relative' }}>
            <div
                style={{
                    display: 'inline-flex',
                    alignItems: 'center',
                    gap: 8,
                    margin: '5px 0',
                    padding: '6px 14px',
                    borderRadius: theme.shape.radiusSm,
                    border: `1px solid ${depth === 0 ? theme.colors.primary : theme.colors.border}`,
                    background: depth === 0 ? theme.colors.primarySoft : theme.colors.surface,
                    fontSize: 15,
                    fontWeight: depth === 0 ? 700 : 500,
                }}
            >
                {node.name}
                {node.value !== undefined ? (
                    <span style={{ color: theme.colors.textMuted, fontSize: 12 }}>{node.value}</span>
                ) : null}
            </div>
            {node.children?.length ? (
                <ul style={{ margin: 0, paddingLeft: 26, borderLeft: `1px dashed ${theme.colors.border}` }}>
                    {node.children.map((c, k) => render(c, depth + 1, k))}
                </ul>
            ) : null}
        </li>
    );
    return <ul style={{ margin: 0, padding: 0 }}>{render(root, 0, 0)}</ul>;
}

/** 统计数字卡 —— 关键指标 */
export function Stat({
    label,
    value,
    unit,
    hint,
}: {
    label: React.ReactNode;
    value: React.ReactNode;
    unit?: React.ReactNode;
    hint?: React.ReactNode;
}): React.ReactElement {
    const theme = useDeckTheme();
    return (
        <div className="hxd-card" style={{ minWidth: 160 }}>
            <div style={{ fontSize: 13, color: theme.colors.textMuted, marginBottom: 6 }}>{label}</div>
            <div
                style={{
                    fontFamily: theme.fonts.numeric ?? theme.fonts.mono,
                    fontSize: 40,
                    fontWeight: 800,
                    lineHeight: 1.1,
                    background: `linear-gradient(120deg, ${theme.colors.primary}, ${theme.colors.accent})`,
                    WebkitBackgroundClip: 'text',
                    backgroundClip: 'text',
                    color: 'transparent',
                }}
            >
                {value}
                {unit ? <span style={{ fontSize: 18, marginLeft: 4 }}>{unit}</span> : null}
            </div>
            {hint ? <div style={{ fontSize: 12, color: theme.colors.textMuted, marginTop: 6 }}>{hint}</div> : null}
        </div>
    );
}