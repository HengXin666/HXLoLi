import React, { useEffect, useMemo, useRef, useState } from 'react';
import {
    ResponsiveContainer,
    LineChart,
    Line,
    XAxis,
    YAxis,
    Tooltip,
    Legend,
    Brush,
    ReferenceDot,
    ReferenceLine,
} from 'recharts';
import type { RecordItem } from '@site/data/wordStats';
import { processWordCountData, type ProcessedData } from '../../utils/wordCountUtils';

interface WordCountChartProps {
    rawData: RecordItem[];
    /** 初始只展示后 percent% 的数据, 区间以索引计算 */
    initialPercent?: number;
}

function wordCountFormat (val: number): string {
    const units: string[] = ["", "万"];
    let unitIndex = 0;
    let num = val;

    // 转换单位（当前支持 "", "万"）
    while (num >= 10000 && unitIndex < units.length - 1) {
        num /= 10000;
        unitIndex++;
    }

    // 使用 toFixed(2) 保留两位小数，再去掉末尾多余的 0 和小数点
    let numStr = num.toFixed(2).replace(/\.?0+$/, "");

    return `${numStr}${units[unitIndex]}`;
}

export default function WordCountChart ({
    rawData,
    initialPercent = 0.8,
}: WordCountChartProps) {
    // 数据处理：生成带 delta 的数组
    const processedData: ProcessedData[] = useMemo<ProcessedData[]>(
        () => processWordCountData(rawData),
        [rawData]
    );

    // Brush 默认展示后 initialPercent 区间
    const totalCount: number = processedData.length;
    const defaultStart: number = Math.floor(totalCount * (1 - initialPercent));
    const defaultEnd: number = totalCount - 1;
    const [brushRange, setBrushRange] = useState<{ startIndex: number; endIndex: number }>({
        startIndex: Math.max(defaultStart, 0),
        endIndex: defaultEnd,
    });

    // 计算增量轴 domain
    const deltaDomain: [number, number] = useMemo<[number, number]>(() => {
        const slice: ProcessedData[] = processedData.slice(
            brushRange.startIndex,
            brushRange.endIndex + 1
        );
        const deltas: number[] = slice.map(d => d.delta);
        const max: number = Math.max(...deltas);
        const min: number = Math.min(...deltas);
        return [min * 0.99, max * 1.01];
    }, [processedData, brushRange]);

    // 计算总量轴 domain
    const totalDomain: [number, number] = useMemo<[number, number]>(() => {
        const slice: ProcessedData[] = processedData.slice(
            brushRange.startIndex,
            brushRange.endIndex + 1
        );
        const totals: number[] = slice.map(d => d.total);
        const max: number = Math.max(...totals);
        const min: number = Math.min(...totals);
        return [min * 0.9, max * 1.1];
    }, [processedData, brushRange]);

    // -- 当前刷选区间数据和统计 --
    const rangeSlice: ProcessedData[] = processedData.slice(
        brushRange.startIndex,
        brushRange.endIndex + 1
    );
    const maxDeltaPoint = useMemo(() => {
        return rangeSlice.reduce((max, d) => d.delta > max.delta ? d : max, rangeSlice[0]);
    }, [rangeSlice]);

    const minDeltaPoint = useMemo(() => {
        return rangeSlice.reduce((min, d) => d.delta < min.delta ? d : min, rangeSlice[0]);
    }, [rangeSlice]);

    // -- 监听鼠标事件以拖动、缩放 --
    const chartRef = useRef<HTMLDivElement>(null);
    const [isDragging, setIsDragging] = useState(false);
    const [dragStartX, setDragStartX] = useState<number | null>(null);

    const handleWheelZoom = (e: WheelEvent) => {
        e.preventDefault();
        const delta = e.deltaY;
        const zoomStep = Math.ceil((brushRange.endIndex - brushRange.startIndex) * 0.05); // 每次缩放 5%
        if (delta > 0) {
            // 放大
            setBrushRange(prev => {
                const newStart = Math.min(prev.startIndex + zoomStep, prev.endIndex - 1);
                const newEnd = Math.max(prev.endIndex - zoomStep, newStart + 1);
                return { startIndex: newStart, endIndex: newEnd };
            });
        } else {
            // 缩小
            setBrushRange(prev => {
                const newStart = Math.max(prev.startIndex - zoomStep, 0);
                const newEnd = Math.min(prev.endIndex + zoomStep, totalCount - 1);
                return { startIndex: newStart, endIndex: newEnd };
            });
        }
    };

    useEffect(() => {
        const el = chartRef.current;
        if (!el) return;

        // 添加原生 wheel 事件监听
        el.addEventListener('wheel', handleWheelZoom, { passive: false });

        return () => {
            el.removeEventListener('wheel', handleWheelZoom);
        };
    }, [brushRange]);

    const handleMouseDown = (e: React.MouseEvent) => {
        setIsDragging(true);
        setDragStartX(e.clientX);
    };

    const handleMouseUp = () => {
        setIsDragging(false);
        setDragStartX(null);
    };

    return (
        <div className="word-count-chart"
            ref={chartRef}
            onMouseDown={handleMouseDown}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseUp}
            style={{ userSelect: 'none', textAlign: 'center' }}
        >
            <h3>历史笔记字数统计</h3>
            <ResponsiveContainer width="100%" height={400}>
                <LineChart data={processedData}>
                    <XAxis
                        dataKey="date"
                        tickFormatter={d => new Date(d).toLocaleDateString()}
                        tick={{ fontSize: 12, fill: '#666' }}
                    />

                    {/* 左轴: 增量 */}
                    <YAxis
                        yAxisId="left"
                        domain={deltaDomain}
                        tickFormatter={v => `${wordCountFormat(v)}`}
                        stroke="#FF61FF"
                        tick={{ fontSize: 12, fill: '#FF61FF' }}
                    />

                    {/* 右轴: 总量 */}
                    <YAxis
                        yAxisId="right"
                        orientation="right"
                        domain={totalDomain}
                        tickFormatter={v => `${wordCountFormat(v)}`}
                        stroke="#1AC5FF"
                        tick={{ fontSize: 12, fill: '#1AC5FF' }}
                    />

                    <Tooltip
                        formatter={(value: number, name: string) => {
                            if (name === '增量')
                                return [`${value.toLocaleString()} 字`, name];
                            if (name === '总量')
                                return [`${value.toLocaleString()} 字`, name];
                            return [value, name];
                        }}
                        labelFormatter={label => label}
                        contentStyle={{
                            borderRadius: 8,
                            borderColor: '#990099',
                            backgroundColor: '#2b2b2b',
                        }}
                    />

                    <Legend verticalAlign="top" height={36} />

                    {/* 平滑增量曲线 */}
                    <Line
                        yAxisId="left"
                        type="monotone"
                        dataKey="delta"
                        name="增量"
                        stroke="#FF61FF"
                        strokeWidth={2}
                        isAnimationActive={true}
                        dot={false}
                    />

                    {/* 平滑总量曲线 */}
                    <Line
                        yAxisId="right"
                        type="monotone"
                        dataKey="total"
                        name="总量"
                        stroke="#1AC5FF"
                        strokeWidth={2}
                        dot={false}
                        isAnimationActive={true}
                    />

                    {deltaDomain[0] <= 0 && deltaDomain[1] >= 0 && (
                        <ReferenceLine
                            y={0}
                            yAxisId="left"
                            stroke="#888"
                            strokeDasharray="4 4"
                            strokeWidth={1}
                            label={{
                                position: 'insideTopLeft',
                                value: '零增量',
                                fill: '#888',
                                fontSize: 12,
                            }}
                        />
                    )}

                    <ReferenceDot
                        x={maxDeltaPoint.date}
                        y={maxDeltaPoint.delta}
                        yAxisId="left"
                        r={3}
                        fill="rgb(255, 97, 97)"
                        stroke="#2b2b2b"
                        label={{
                            value: `▲ ${maxDeltaPoint.delta.toLocaleString()}`,
                            position: 'top',
                            fill: 'rgb(255, 97, 97)',
                            fontSize: 12,
                        }}
                    />

                    <ReferenceDot
                        x={minDeltaPoint.date}
                        y={minDeltaPoint.delta}
                        yAxisId="left"
                        r={3}
                        fill="rgb(97, 255, 97)"
                        stroke="#2b2b2b"
                        label={{
                            value: `▼ ${minDeltaPoint.delta.toLocaleString()}`,
                            position: 'bottom',
                            fill: 'rgb(97, 255, 97)',
                            fontSize: 12,
                        }}
                    />

                    <Brush
                        dataKey="date"
                        height={15}
                        stroke="#ff88ff"
                        fill="#2b2b2b"
                        startIndex={brushRange.startIndex}
                        endIndex={brushRange.endIndex}
                        onChange={range => {
                            const { startIndex, endIndex } = range;
                            if (startIndex != null && endIndex != null) {
                                setBrushRange({ startIndex, endIndex });
                            }
                        }}
                    />
                </LineChart>
            </ResponsiveContainer>
        </div>
    );
}
