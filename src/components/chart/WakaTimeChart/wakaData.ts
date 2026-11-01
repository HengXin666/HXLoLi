/**
 * WakaTime 历史数据: 首页摘要 (Pulse) 与展开后的趋势图 (index) 共用一次下载.
 *
 * 两个维度分开算, 不再混用:
 * - 编码时长 = 当天 languages 各项之和 (编辑器里真正在写代码的时间);
 * - 使用设备 = 当天 system 各项之和, 按 Linux / Windows / Mac 拆开 (开着编辑器、插件在计时的时间, 含挂机).
 * 实测一天的设备时长常是编码时长的 5 到 20 倍, 所以两者同轴画时编码在下、设备在上, 不会互相遮挡.
 * 设备/编码两个维度的口径见同目录 index.tsx 顶部注释
 */
// raw.githubusercontent.com 上这个路径 2026-11-01 实测 404; jsDelivr 同一文件 200, 带 CORS *.
// jsDelivr 回 max-age 7 天, 拉取时带上当天日期作查询参数, 浏览器每天最多重新拉一次.
export const WAKA_DATA_URL =
    'https://cdn.jsdelivr.net/gh/HengXin666/HengXin666@main/data/history.json';

type RawRecord = {
    date: string;
    languages?: { [key: string]: number }[];
    system?: { [key: string]: number }[];
};

export type WakaDay = {
    date: string;
    /** 编码时长, 秒 */
    code: number;
    /** 设备在线时长, 秒 */
    device: number;
    langs: Map<string, number>;
    os: Map<string, number>;
};

export const OS_ORDER = ['Linux', 'Windows', 'Mac'] as const;

export const OS_COLORS: Record<string, string> = {
    Linux: '#FCC624',
    Windows: '#4DA3FF',
    Mac: '#C7CBD1',
};

/** 编码维度的主色 = 站点主色 */
export const CODE_COLOR = '#ff88ff';
/** 设备维度的主色, 与编码的粉色互补, 叠加时一眼分得开 */
export const DEVICE_COLOR = '#5ee7ff';

function sumInto(list: { [key: string]: number }[] | undefined, into: Map<string, number>): number {
    let total = 0;
    for (const obj of list ?? []) {
        const entry = Object.entries(obj)[0];
        if (!entry) continue;
        const [k, v] = entry;
        into.set(k, (into.get(k) ?? 0) + v);
        total += v;
    }
    return total;
}

let cache: Promise<WakaDay[]> | null = null;

/** 按日期升序返回; 同一页多次调用只下载一次, 失败后允许重试 */
export function loadWakaDays(): Promise<WakaDay[]> {
    if (!cache) {
        cache = fetch(`${WAKA_DATA_URL}?d=${new Date().toISOString().slice(0, 10)}`)
            .then((r) => {
                if (!r.ok) throw new Error(`HTTP ${r.status}`);
                return r.json() as Promise<RawRecord[]>;
            })
            .then((raw) =>
                raw
                    .map((rec) => {
                        const langs = new Map<string, number>();
                        const os = new Map<string, number>();
                        const code = sumInto(rec.languages, langs);
                        const device = sumInto(rec.system, os);
                        return { date: rec.date, code, device, langs, os };
                    })
                    .sort((a, b) => a.date.localeCompare(b.date)),
            )
            .catch((e) => {
                cache = null;
                throw e;
            });
    }
    return cache;
}

/** 秒 -> "3.4h" / "12h" */
export function fmtHours(sec: number): string {
    const h = sec / 3600;
    if (h === 0) return '0h';
    if (h < 1) return `${Math.round(sec / 60)}m`;
    return h >= 10 ? `${Math.round(h)}h` : `${h.toFixed(1)}h`;
}

/** 秒 -> "3 小时 24 分" */
export function fmtLong(sec: number): string {
    const h = Math.floor(sec / 3600);
    const m = Math.round((sec % 3600) / 60);
    if (h === 0) return `${m} 分`;
    return m === 0 ? `${h} 小时` : `${h} 小时 ${m} 分`;
}

export function topEntries(map: Map<string, number>, n: number): [string, number][] {
    return [...map.entries()].sort((a, b) => b[1] - a[1]).slice(0, n);
}
