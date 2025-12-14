import React, { useEffect, useMemo, useState } from "react";
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
} from "recharts";
import { BRAND_COLORS, FALLBACK_COLORS } from "./colors";
import { FaApple, FaLinux, FaWindows } from "react-icons/fa";

const OS_ICONS: { [key: string]: React.ReactNode } = {
    Linux: <FaLinux />,
    Windows: <FaWindows />,
    Mac: <FaApple />,
};

const WAKA_DATA_URL =
    "https://raw.githubusercontent.com/HengXin666/HengXin666/main/data/history.json";

// --- 类型定义 ---
interface RawWakaRecord {
    date: string;
    languages: { [key: string]: number }[];
    system: { [key: string]: number }[];
}
interface ProcessedWakaData {
    date: string;
    [key: string]: number | string;
}
type Precision = "minutes" | "seconds" | "milliseconds";

// --- 工具函数 ---
function formatSecondsToHMS (seconds: number, precision: Precision): string {
    if (isNaN(seconds) || seconds < 0) return "0m";
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    if (precision === "minutes") return `${h > 0 ? `${h}h ` : ""}${m}m`;
    const s = Math.floor(seconds % 60);
    if (precision === "seconds") return `${h > 0 ? `${h}h ` : ""}${m}m ${s}s`;
    const ms = Math.round((seconds * 1000) % 1000);
    return `${h > 0 ? `${h}h ` : ""}${m}m ${s}s ${ms}ms`;
}

// --- 自定义图例组件 (带图标) ---
const CustomIconLegend = (props: any) => {
    const { payload } = props;
    return (
        <ul
            style={{
                listStyle: "none",
                margin: 0,
                padding: 0,
                display: "flex",
                justifyContent: "center",
                gap: "16px",
            }}
        >
            {payload.map((entry: any, index: number) => {
                // '总和'曲线不显示在图例中
                if (entry.dataKey.startsWith("total")) return null;
                return (
                    <li
                        key={`item-${index}`}
                        style={{
                            color: entry.color,
                            display: "flex",
                            alignItems: "center",
                        }}
                    >
                        {OS_ICONS[entry.value] || (
                            <span style={{ marginRight: "4px", color: entry.color }}>●</span>
                        )}
                        <span style={{ marginLeft: "4px" }}>{entry.value}</span>
                    </li>
                );
            })}
        </ul>
    );
};

// --- 子组件: 可复用的图表 ---
interface DrilldownChartProps {
    title: string;
    data: ProcessedWakaData[];
    topItems: string[];
    dataKeyPrefix: string;
    precision: Precision;
    axisColor: string;
    customLegend?: React.ReactElement;
}

function DrilldownChart ({
    title,
    data,
    topItems,
    dataKeyPrefix,
    precision,
    axisColor,
    customLegend,
}: DrilldownChartProps) {
    const [brushRange, setBrushRange] = useState({
        startIndex: 0,
        endIndex: data.length - 1,
    });
    const { domain, maxPoint } = useMemo(() => {
        const slice = data.slice(brushRange.startIndex, brushRange.endIndex + 1);
        if (slice.length === 0) return { domain: [0, 1], maxPoint: null };
        const totalKey = `total${dataKeyPrefix}`;
        let maxTotal = 0;
        let pointWithMax: ProcessedWakaData = slice[0];
        slice.forEach((d) => {
            const totalValue = (d[totalKey] || 0) as number;
            if (totalValue > maxTotal) {
                maxTotal = totalValue;
                pointWithMax = d;
            }
        });
        return {
            domain: [0, Math.max(1, Math.ceil(maxTotal * 1.2))],
            maxPoint: pointWithMax,
        };
    }, [data, brushRange, dataKeyPrefix]);

    return (
        <div style={{ width: "50%", userSelect: "none" }}>
            <h4 style={{ textAlign: "center", color: axisColor }}>{title}</h4>
            <ResponsiveContainer width="100%" height={400}>
                <LineChart
                    data={data}
                    margin={{ top: 5, right: 30, left: 5, bottom: 5 }}
                >
                    <XAxis dataKey="date" tick={{ fontSize: 11, fill: "#9CA3AF" }} />
                    <YAxis
                        domain={domain}
                        tickFormatter={(val) => `${val.toFixed(1)}h`}
                        stroke={axisColor}
                        tick={{ fontSize: 11, fill: axisColor }}
                    />
                    <Tooltip
                        formatter={(value: number, name: string) => [
                            formatSecondsToHMS(value * 3600, precision),
                            name,
                        ]}
                        labelStyle={{ fontWeight: "bold" }}
                        contentStyle={{
                            backgroundColor: "rgba(30, 41, 59, 0.9)",
                            borderColor: "#475569",
                            borderRadius: "8px",
                        }}
                    />
                    <Legend verticalAlign="top" height={36} content={customLegend} />
                    <Line
                        type="monotone"
                        dataKey={`total${dataKeyPrefix}`}
                        name="每日总和"
                        stroke={axisColor}
                        strokeWidth={3}
                        dot={false}
                    />
                    {topItems.map((item, index) => (
                        <Line
                            key={item}
                            type="monotone"
                            dataKey={`${item}${dataKeyPrefix}`}
                            name={item}
                            stroke={
                                BRAND_COLORS[item] ||
                                FALLBACK_COLORS[index % FALLBACK_COLORS.length]
                            }
                            strokeWidth={1.5}
                            dot={false}
                        />
                    ))}
                    {maxPoint && (
                        <ReferenceDot
                            x={maxPoint.date}
                            y={maxPoint[`total${dataKeyPrefix}`] as number}
                            r={4}
                            fill="#F87171"
                            stroke="white"
                            label={{
                                value: `🚀 ${(
                                    (maxPoint[`total${dataKeyPrefix}`] as number) || 0
                                ).toFixed(2)}h`,
                                position: "top",
                                fill: "#F87171",
                                fontSize: 12,
                            }}
                        />
                    )}
                    <Brush
                        dataKey="date"
                        height={20}
                        stroke="#6366F1"
                        startIndex={brushRange.startIndex}
                        endIndex={brushRange.endIndex}
                        onChange={(range) => {
                            if (
                                range.startIndex !== undefined &&
                                range.endIndex !== undefined &&
                                (range.startIndex !== brushRange.startIndex ||
                                    range.endIndex !== brushRange.endIndex)
                            ) {
                                setBrushRange({
                                    startIndex: range.startIndex,
                                    endIndex: range.endIndex,
                                });
                            }
                        }}
                    />
                </LineChart>
            </ResponsiveContainer>
        </div>
    );
}

// --- 主仪表盘组件 ---
export default function WakaTimeDashboard () {
    const [rawData, setRawData] = useState<RawWakaRecord[]>([]);
    const [loading, setLoading] = useState<boolean>(true);
    const [error, setError] = useState<string | null>(null);
    const [topK, setTopK] = useState<number>(6);
    const [precision, setPrecision] = useState<Precision>("seconds");

    useEffect(() => {
        const fetchData = async () => {
            try {
                const response = await fetch(WAKA_DATA_URL);
                if (!response.ok) throw new Error(`Network Error: ${response.status}`);
                const data: RawWakaRecord[] = await response.json();
                setRawData(data.reverse());
            } catch (e) {
                setError(e instanceof Error ? e.message : String(e));
            } finally {
                setLoading(false);
            }
        };
        fetchData();
    }, []);

    const { processedLangData, topLanguages, processedOsData, topOses } =
        useMemo(() => {
            if (rawData.length === 0)
                return {
                    processedLangData: [],
                    topLanguages: [],
                    processedOsData: [],
                    topOses: [],
                };

            const processCategory = (
                category: "languages" | "system",
                k: number,
                dataKeyPrefix: string
            ) => {
                // 优化: 使用 Map 提升性能
                const totals = new Map<string, number>();

                for (const record of rawData) {
                    for (const obj of record[category] || []) {
                        const [key, seconds] = Object.entries(obj)[0];
                        totals.set(key, (totals.get(key) || 0) + seconds);
                    }
                }

                const effectiveK = category === "system" ? totals.size : k;

                // 优化: 避免多次转换
                const sortedEntries = Array.from(totals.entries())
                    .sort(([, a], [, b]) => b - a)
                    .slice(0, effectiveK);

                const topItems = sortedEntries.map(([key]) => key);
                const topItemsSet = new Set(topItems);

                const processedData: ProcessedWakaData[] = rawData.map((record) => {
                    const dailyData: ProcessedWakaData = { date: record.date };
                    let absoluteTotalSeconds = 0;

                    // 优化: 使用对象字面量初始化
                    for (const item of topItems) {
                        dailyData[`${item}${dataKeyPrefix}`] = 0;
                    }

                    for (const obj of record[category] || []) {
                        const [key, seconds] = Object.entries(obj)[0];
                        absoluteTotalSeconds += seconds;
                        if (topItemsSet.has(key)) {
                            dailyData[`${key}${dataKeyPrefix}`] = seconds / 3600;
                        }
                    }

                    dailyData[`total${dataKeyPrefix}`] = absoluteTotalSeconds / 3600;

                    return dailyData;
                });

                return { processedData, topItems };
            };

            const { processedData: langData, topItems: langs } = processCategory(
                "languages",
                topK,
                "_lang_hours"
            );
            const { processedData: osData, topItems: oses } = processCategory(
                "system",
                topK,
                "_os_hours"
            );

            return {
                processedLangData: langData,
                topLanguages: langs,
                processedOsData: osData,
                topOses: oses,
            };
        }, [rawData, topK]);

    if (loading) {
        return (
            <div style={{ textAlign: "center", padding: "40px" }}>
                📊 加载 WakaTime 历史时间统计...
            </div>
        );
    }
    if (error) {
        return (
            <div style={{ textAlign: "center", padding: "40px", color: "red" }}>
                ❌ 加载 WakaTime 历史时间统计失败: {error}
            </div>
        );
    }

    return (
        <div className="wakatime-dashboard" style={{ padding: "1rem" }}>
            <h3 style={{textAlign: 'center'}}>WakaTime 历史时间统计</h3>
            <div
                className="controls"
                style={{
                    display: "flex",
                    justifyContent: "center",
                    gap: "2rem",
                    marginBottom: "1rem",
                    alignItems: "center",
                }}
            >
                <div>
                    <label htmlFor="precision-select">时间精度: </label>
                    <select
                        id="precision-select"
                        value={precision}
                        onChange={(e) => setPrecision(e.target.value as Precision)}
                    >
                        <option value="minutes">分钟</option>
                        <option value="seconds">秒</option>
                        <option value="milliseconds">毫秒</option>
                    </select>
                </div>
                <div>
                    <label htmlFor="top-k-input">展示 Top K 项: </label>
                    <input
                        id="top-k-input"
                        type="number"
                        value={topK}
                        min="1"
                        step="1"
                        onChange={(e) => {
                            const val = Math.floor(Number(e.target.value));
                            if (val > 0) {
                                setTopK(val);
                            }
                        }}
                        style={{ width: "60px" }}
                    />
                </div>
            </div>

            <div
                className="charts-container"
                style={{ display: "flex", gap: "1rem" }}
            >
                <DrilldownChart
                    title="操作系统使用"
                    data={processedOsData}
                    topItems={topOses}
                    dataKeyPrefix="_os_hours"
                    precision={precision}
                    axisColor="#ee11ff"
                    customLegend={<CustomIconLegend />}
                />
                <DrilldownChart
                    title="语言使用 (挂机不计入)"
                    data={processedLangData}
                    topItems={topLanguages}
                    dataKeyPrefix="_lang_hours"
                    precision={precision}
                    axisColor="#990099"
                />
            </div>
        </div>
    );
}
