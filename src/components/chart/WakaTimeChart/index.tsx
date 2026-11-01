import React, { useEffect, useMemo, useState } from 'react';
import {
    Area,
    Brush,
    CartesianGrid,
    ComposedChart,
    Line,
    ResponsiveContainer,
    Tooltip,
    XAxis,
    YAxis,
} from 'recharts';

import { BRAND_COLORS, FALLBACK_COLORS } from './colors';
import styles from './trend.module.css';
import {
    CODE_COLOR,
    DEVICE_COLOR,
    OS_COLORS,
    OS_ORDER,
    type WakaDay,
    loadWakaDays,
    fmtLong,
} from './wakaData';

type Grain = 'day' | 'week' | 'month';
type Unit = 'h' | 'm';
/** 绝对时长 / 语言占比. 占比只对编码维度有意义 (设备在线不分语言), 所以占比模式只画语言 */
type Mode = 'abs' | 'share';

const GRAINS: { id: Grain; label: string }[] = [
    { id: 'day', label: '日' },
    { id: 'week', label: '周' },
    { id: 'month', label: '月' },
];

const UNITS: { id: Unit; label: string }[] = [
    { id: 'h', label: '小时' },
    { id: 'm', label: '分钟' },
];

const MODES: { id: Mode; label: string }[] = [
    { id: 'abs', label: '绝对时长' },
    { id: 'share', label: '语言占比' },
];

/** codeSec: 该期日均编码秒数, 不随单位换算, 占比模式的提示里给"该点总时长"用 */
type Row = { key: string; label: string; days: number; code: number; device: number; codeSec: number; [k: string]: number | string };

/**
 * 把一组时长换成保留 1 位小数的百分比, 合计恰好 100.0 (最大余数法).
 * 直接各自四舍五入会出现 99.9 / 100.1, 堆叠顶边就不是一条平线.
 * 合计为 0 (这一期没写代码) 时全部返回 0.
 */
function toPercents (values: number[]): number[] {
    const total = values.reduce((a, b) => a + b, 0);
    if (total <= 0) return values.map(() => 0);
    const tenths = values.map((v) => (v / total) * 1000);
    const floors = tenths.map(Math.floor);
    let rest = 1000 - floors.reduce((a, b) => a + b, 0);
    const order = tenths.map((t, i) => [t - floors[i], i] as const).sort((a, b) => b[0] - a[0]);
    for (const [, i] of order) {
        if (rest <= 0) break;
        floors[i] += 1;
        rest -= 1;
    }
    return floors.map((f) => f / 10);
}

function langColor (name: string, index: number): string {
    return BRAND_COLORS[name] ?? FALLBACK_COLORS[index % FALLBACK_COLORS.length];
}

function bucketKey (date: string, grain: Grain): string {
    if (grain === 'day') return date;
    if (grain === 'month') return date.slice(0, 7);
    // 周: 归到当周周一
    const d = new Date(`${date}T00:00:00`);
    const shift = (d.getDay() + 6) % 7;
    d.setDate(d.getDate() - shift);
    return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, '0')}-${String(d.getDate()).padStart(2, '0')}`;
}

function usePrefersReducedMotion (): boolean {
    const [reduced, setReduced] = useState(false);
    useEffect(() => {
        if (typeof window === 'undefined' || !window.matchMedia) return undefined;
        const mq = window.matchMedia('(prefers-reduced-motion: reduce)');
        setReduced(mq.matches);
        const on = () => setReduced(mq.matches);
        mq.addEventListener('change', on);
        return () => mq.removeEventListener('change', on);
    }, []);
    return reduced;
}

/**
 * 首页"每日曲线"展开后的趋势图. 一张满宽的图, 两个维度可单独看也可叠加:
 * - 只看编码: 前五种语言堆叠面积, 顶边 = 当期编码时长;
 * - 只看设备: Linux / Windows / Mac 堆叠面积, 顶边 = 当期在线时长;
 * - 叠加: 设备堆叠面积压暗做底, 编码时长画成一条粉色粗线压在上面. 两者同一单位同一根轴, 不用双轴.
 * 原来的"时间精度"原生下拉框 (分钟 / 秒 / 毫秒, 只影响 tooltip) 换成两组分段按钮: 粒度 日/周/月, 单位 小时/分钟.
 * 周、月粒度显示的是该期的日均值, 三种粒度的纵轴数值可以直接比较.
 * "数值"切换 绝对时长 / 语言占比: 占比模式只画语言, 每期各语言占当期编码时长的百分比 (合计恒为 100, 见 toPercents),
 * 纵轴固定 0-100%, 最大的语言放在堆叠最底层, 它的色带顶边对着纵轴就是它的份额; 提示按份额降序列出并给该期编码总时长.
 * .agents/notes/implemented/feature/2026-10-31-homepage-profile-and-project-index.md
 */
export default function WakaTimeTrend (): React.ReactElement {
    const [days, setDays] = useState<WakaDay[] | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [showCode, setShowCode] = useState(true);
    const [showDevice, setShowDevice] = useState(true);
    const [grain, setGrain] = useState<Grain>('week');
    const [unit, setUnit] = useState<Unit>('h');
    const [mode, setMode] = useState<Mode>('abs');
    const reduced = usePrefersReducedMotion();

    useEffect(() => {
        let alive = true;
        loadWakaDays()
            .then((list) => alive && setDays(list))
            .catch((e) => alive && setError(String(e?.message ?? e)));
        return () => {
            alive = false;
        };
    }, []);

    const topLangs = useMemo(() => {
        if (!days) return [];
        const totals = new Map<string, number>();
        for (const d of days.slice(-365)) for (const [k, v] of d.langs) totals.set(k, (totals.get(k) ?? 0) + v);
        return [...totals.entries()].sort((a, b) => b[1] - a[1]).slice(0, 5).map(([k]) => k);
    }, [days]);

    const rows = useMemo<Row[]>(() => {
        if (!days) return [];
        const scale = unit === 'h' ? 1 / 3600 : 1 / 60;
        const buckets = new Map<string, Row>();
        for (const d of days) {
            const key = bucketKey(d.date, grain);
            let row = buckets.get(key);
            if (!row) {
                row = { key, label: grain === 'month' ? key : key.slice(5), days: 0, code: 0, device: 0, codeSec: 0 };
                for (const os of OS_ORDER) row[`os:${os}`] = 0;
                for (const l of topLangs) row[`lang:${l}`] = 0;
                row['lang:其他'] = 0;
                buckets.set(key, row);
            }
            row.days += 1;
            row.code += d.code;
            row.device += d.device;
            for (const [k, v] of d.os) {
                const slot = `os:${OS_ORDER.includes(k as (typeof OS_ORDER)[number]) ? k : 'Linux'}`;
                row[slot] = (row[slot] as number) + v;
            }
            for (const [k, v] of d.langs) {
                const slot = topLangs.includes(k) ? `lang:${k}` : 'lang:其他';
                row[slot] = (row[slot] as number) + v;
            }
        }
        const langKeys = [...topLangs, '其他'];
        // 每期换成日均, 再换单位; 占比用换算前的原始秒数算, 不受单位与取整影响
        return [...buckets.values()].map((row) => {
            const out: Row = { ...row };
            for (const [k, v] of Object.entries(row)) {
                if (k === 'key' || k === 'label' || k === 'days' || k === 'codeSec') continue;
                out[k] = Math.round(((v as number) / row.days) * scale * 100) / 100;
            }
            out.codeSec = row.code / row.days;
            const pcts = toPercents(langKeys.map((l) => row[`lang:${l}`] as number));
            langKeys.forEach((l, i) => {
                out[`pct:${l}`] = pcts[i];
            });
            return out;
        });
    }, [days, grain, unit, topLangs]);

    const [range, setRange] = useState<{ startIndex: number; endIndex: number } | null>(null);
    useEffect(() => {
        if (!rows.length) return;
        // 默认看最近一段: 日 120 天 / 周 52 周 / 月 全部
        const span = grain === 'day' ? 120 : grain === 'week' ? 52 : rows.length;
        setRange({ startIndex: Math.max(0, rows.length - span), endIndex: rows.length - 1 });
    }, [rows.length, grain]);

    if (error) return <p className={styles.note}>WakaTime 数据没取到 ({error}).</p>;
    if (!days || !range) return <div className={styles.skeleton} aria-busy="true" />;

    const fmt = (v: number) => (unit === 'h' ? `${v.toFixed(v >= 10 ? 0 : 1)}h` : `${Math.round(v)}m`);
    const anim = !reduced;
    const share = mode === 'share';
    // 占比模式只画语言, 维度开关暂时失效 (状态保留, 切回绝对时长时恢复)
    const both = !share && showCode && showDevice;
    const codeOn = share || showCode;
    const deviceOn = !share && showDevice;
    const langKeys = [...topLangs, '其他'];
    const colorOf = (l: string, i: number) => (l === '其他' ? '#777' : langColor(l, i));

    const toggle = (which: 'code' | 'device') => {
        // 至少保留一个维度
        if (which === 'code') setShowCode((v) => (v && !showDevice ? v : !v));
        else setShowDevice((v) => (v && !showCode ? v : !v));
    };

    return (
        <div className={styles.wrap}>
            <div className={styles.toolbar}>
                <div className={styles.group} role="radiogroup" aria-label="数值">
                    <span className={styles.groupLabel}>数值</span>
                    <div className={styles.seg}>
                        {MODES.map((x) => (
                            <button key={x.id} type="button" role="radio" aria-checked={mode === x.id} className={mode === x.id ? styles.segOn : styles.segBtn} onClick={() => setMode(x.id)}>
                                {x.label}
                            </button>
                        ))}
                    </div>
                </div>
                <div className={styles.group} role="group" aria-label="维度" data-disabled={share}>
                    <span className={styles.groupLabel}>维度</span>
                    <button
                        type="button"
                        className={styles.dimBtn}
                        data-on={codeOn}
                        style={{ '--dim': CODE_COLOR } as React.CSSProperties}
                        aria-pressed={codeOn}
                        disabled={share}
                        onClick={() => toggle('code')}
                    >
                        <i />编码时长
                    </button>
                    <button
                        type="button"
                        className={styles.dimBtn}
                        data-on={deviceOn}
                        style={{ '--dim': DEVICE_COLOR } as React.CSSProperties}
                        aria-pressed={deviceOn}
                        disabled={share}
                        onClick={() => toggle('device')}
                    >
                        <i />使用设备
                    </button>
                </div>
                <div className={styles.group} role="radiogroup" aria-label="粒度">
                    <span className={styles.groupLabel}>粒度</span>
                    <div className={styles.seg}>
                        {GRAINS.map((g) => (
                            <button key={g.id} type="button" role="radio" aria-checked={grain === g.id} className={grain === g.id ? styles.segOn : styles.segBtn} onClick={() => setGrain(g.id)}>
                                {g.label}
                            </button>
                        ))}
                    </div>
                </div>
                <div className={styles.group} role="radiogroup" aria-label="单位" data-disabled={share}>
                    <span className={styles.groupLabel}>单位</span>
                    <div className={styles.seg}>
                        {UNITS.map((x) => (
                            <button key={x.id} type="button" role="radio" aria-checked={unit === x.id} disabled={share} className={unit === x.id ? styles.segOn : styles.segBtn} onClick={() => setUnit(x.id)}>
                                {x.label}
                            </button>
                        ))}
                    </div>
                </div>
            </div>

            <p className={styles.caption}>
                {grain === 'day' ? '每日' : grain === 'week' ? '每周日均' : '每月日均'} ·{' '}
                {share
                    ? `各语言占编码时长的百分比, 合计 100%; 最底层是 ${topLangs[0] ?? '最大语言'}, 色带顶边对着纵轴即其份额`
                    : both ? '底色是设备在线 (按系统堆叠), 粉线是编码时长' : showCode ? '按语言堆叠, 顶边是编码时长' : '按系统堆叠, 顶边是在线时长'}
            </p>

            <div className={styles.chart}>
                <ResponsiveContainer width="100%" height={340}>
                    <ComposedChart data={rows} margin={{ top: 8, right: 12, left: 0, bottom: 0 }}>
                        <defs>
                            <linearGradient id="hx-code-fill" x1="0" y1="0" x2="0" y2="1">
                                <stop offset="0%" stopColor={CODE_COLOR} stopOpacity={0.35} />
                                <stop offset="100%" stopColor={CODE_COLOR} stopOpacity={0} />
                            </linearGradient>
                        </defs>
                        <CartesianGrid stroke="rgba(255,255,255,0.06)" vertical={false} />
                        <XAxis dataKey="label" tick={{ fontSize: 11, fill: '#9CA3AF' }} tickLine={false} axisLine={{ stroke: 'rgba(255,255,255,0.12)' }} minTickGap={24} />
                        {share ? (
                            <YAxis
                                width={44}
                                domain={[0, 100]}
                                ticks={[0, 25, 50, 75, 100]}
                                allowDataOverflow
                                tick={{ fontSize: 11, fill: '#9CA3AF' }}
                                tickLine={false}
                                axisLine={false}
                                tickFormatter={(v: number) => `${v}%`}
                            />
                        ) : (
                            <YAxis
                                width={44}
                                tick={{ fontSize: 11, fill: '#9CA3AF' }}
                                tickLine={false}
                                axisLine={false}
                                tickFormatter={(v: number) => `${v}${unit === 'h' ? 'h' : 'm'}`}
                            />
                        )}
                        <Tooltip
                            cursor={{ stroke: 'rgba(255,255,255,0.25)' }}
                            content={({ active, payload, label }) => {
                                if (!active || !payload?.length) return null;
                                const row = payload[0].payload as Row;
                                const when = grain === 'day' ? row.key : grain === 'week' ? `${row.key} 起一周` : row.key;
                                if (share) {
                                    const parts = langKeys
                                        .map((k, i) => ({ k, i, p: row[`pct:${k}`] as number }))
                                        .filter((x) => x.p > 0)
                                        .sort((a, b) => b.p - a.p);
                                    return (
                                        <div className={styles.tip}>
                                            <strong>{when}</strong>
                                            <span className={styles.tipMeta}>
                                                {grain === 'day' ? '编码' : `${row.days} 天日均编码`} {row.codeSec > 0 ? fmtLong(row.codeSec) : '0 分'}
                                            </span>
                                            {parts.length === 0 && <span className={styles.tipMeta}>这一期没有编码记录</span>}
                                            {parts.map((x, n) => (
                                                <div key={x.k} className={styles.tipRow}>
                                                    <i style={{ background: colorOf(x.k, x.i) }} />
                                                    {n === 0 ? <span className={styles.tipLead}>{x.k}</span> : x.k}
                                                    <b>{x.p.toFixed(1)}%</b>
                                                </div>
                                            ))}
                                            <span className={styles.hidden}>{label}</span>
                                        </div>
                                    );
                                }
                                return (
                                    <div className={styles.tip}>
                                        <strong>{when}</strong>
                                        {grain !== 'day' && <span className={styles.tipMeta}>{row.days} 天的日均</span>}
                                        <div className={styles.tipRow}><i style={{ background: CODE_COLOR }} />编码<b>{fmt(row.code as number)}</b></div>
                                        <div className={styles.tipRow}><i style={{ background: DEVICE_COLOR }} />设备<b>{fmt(row.device as number)}</b></div>
                                        <div className={styles.tipMeta}>
                                            {(showDevice ? OS_ORDER.map((k) => [k, row[`os:${k}`] as number] as const) : langKeys.map((k) => [k, row[`lang:${k}`] as number] as const))
                                                .filter(([, v]) => v > 0)
                                                .map(([k, v]) => <span key={k}>{k} {fmt(v)}</span>)}
                                        </div>
                                        <span className={styles.hidden}>{label}</span>
                                    </div>
                                );
                            }}
                        />
                        {deviceOn && OS_ORDER.map((os) => (
                            <Area
                                key={os}
                                type="monotone"
                                dataKey={`os:${os}`}
                                name={os}
                                stackId="device"
                                stroke={OS_COLORS[os]}
                                strokeOpacity={both ? 0.35 : 0.9}
                                fill={OS_COLORS[os]}
                                fillOpacity={both ? 0.14 : 0.3}
                                isAnimationActive={anim}
                                animationDuration={900}
                            />
                        ))}
                        {codeOn && !both && langKeys.map((l, i) => (
                            <Area
                                key={`${mode}:${l}`}
                                type="monotone"
                                dataKey={share ? `pct:${l}` : `lang:${l}`}
                                name={l}
                                stackId={share ? 'share' : 'code'}
                                stroke={colorOf(l, i)}
                                fill={colorOf(l, i)}
                                fillOpacity={share ? 0.55 : 0.3}
                                isAnimationActive={anim}
                                animationDuration={900}
                            />
                        ))}
                        {both && (
                            <Area
                                type="monotone"
                                dataKey="code"
                                name="编码"
                                stroke={CODE_COLOR}
                                strokeWidth={2.5}
                                fill="url(#hx-code-fill)"
                                isAnimationActive={anim}
                                animationDuration={1100}
                                animationBegin={250}
                            />
                        )}
                        {deviceOn && !both && (
                            <Line type="monotone" dataKey="device" stroke={DEVICE_COLOR} strokeWidth={1.5} dot={false} isAnimationActive={anim} animationDuration={900} />
                        )}
                        <Brush
                            dataKey="label"
                            height={22}
                            travellerWidth={8}
                            stroke="rgba(255,136,255,0.6)"
                            fill="rgba(255,255,255,0.03)"
                            startIndex={range.startIndex}
                            endIndex={range.endIndex}
                            onChange={(r) => {
                                if (r.startIndex !== undefined && r.endIndex !== undefined) setRange({ startIndex: r.startIndex, endIndex: r.endIndex });
                            }}
                        />
                    </ComposedChart>
                </ResponsiveContainer>
            </div>

            <ul className={styles.legend}>
                {deviceOn && OS_ORDER.map((os) => (
                    <li key={os}><i style={{ background: OS_COLORS[os], opacity: both ? 0.5 : 1 }} />{os}</li>
                ))}
                {codeOn && !both && langKeys.map((l, i) => (
                    <li key={l}><i style={{ background: colorOf(l, i) }} />{l}</li>
                ))}
                {both && <li><i style={{ background: CODE_COLOR }} />编码时长</li>}
            </ul>
        </div>
    );
}
