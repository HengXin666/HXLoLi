import { WatchStatus } from "@site/src/utils/anime/types";
import type { SortKey, TagMatch } from "@site/src/utils/anime/filters";
import type { MarkFilter, YearRange } from "@site/src/components/anime/animeShareLink";
/*
 * 分享番单 (/anime/share 与 /anime 的分享弹窗) 的「内定筛选」: 一个默认值 + 若干命名预设.
 *
 * 怎么改:
 *   - 改 ANIME_SHARE_DEFAULT = 别人打开 /anime/share (不带任何筛选参数) 时看到的初始状态, 也是 /anime 分享弹窗的初始状态.
 *   - 往 ANIME_SHARE_PRESETS 加一项 = 多一条短链接 /anime/share?p=<名字>. 名字只能是 [a-z0-9-], 1 到 16 个字符,
 *     不合规的名字不会生效 (页面当作"预设不存在"). 名字 "default" 保留给 ANIME_SHARE_DEFAULT, 不要再定义.
 *   - 每个字段都可省略, 省略 = 不筛 (sort 省略 = 首播日期, view 省略 = 卡片).
 *
 * 字段:
 *   label   页面上显示的预设名
 *   status  站长的观看状态, WatchStatus.WATCHED 看过 / WATCHING 在看 / WANT_TO_WATCH 想看 / ON_HOLD 搁置 / DROPPED 抛弃
 *   tags    标签 (与番剧库标签同名, 如 "ロリ" "日常"); match: "all" 全部命中 (默认) / "any" 任一命中
 *   q       搜索词
 *   sort    "update" 最近更新 / "air" 首播日期 / "score" 评分 / "name" 名称
 *   year    "5" | "10" | "15" 近 N 年 (含今年) / "all" 全部
 *   mark    勾选筛选: "all" / "mine" 我看过 / "notMine" 我没看过 / "theirs" TA 看过 / "both" 我们都看过
 *   view    "card" 卡片 / "table" 表格 (按年份)
 *   pick    预置的"TA 看过"集合, 用 encodeIds 编码 (在分享页勾好后点"生成我的链接", 复制链接里 pick= 后面那串贴过来);
 *           只在预设里用, 短链接仍只是 ?p=<名字>. 链接里自带 pick 时以链接为准.
 *
 * 规则: 链接带 p 就展开对应预设; 不带 p 且不带番单筛选参数 (q / status / tag / match / year / score / sort) 时用默认值;
 * 带番单筛选参数的老长链接原样生效, 不套默认值. 展开后的状态在页面上可以继续改, 改了就成了"自定义".
 * .agents/notes/implemented/feature/2026-11-01-anime-share-table-view-and-preview.md
 */
export interface AnimeShareStart {
    label: string;
    status?: WatchStatus[];
    tags?: string[];
    match?: TagMatch;
    q?: string;
    sort?: SortKey;
    year?: YearRange;
    mark?: MarkFilter;
    view?: "card" | "table";
    pick?: string;
}

export const ANIME_SHARE_DEFAULT: AnimeShareStart = {
    label: "站长看过的",
    status: [WatchStatus.WATCHED],
    sort: "air",
    year: "all",
};

export const ANIME_SHARE_PRESETS: Readonly<Record<string, AnimeShareStart>> = {
    /** /anime/share?p=mine */
    mine: { label: "站长看过的", status: [WatchStatus.WATCHED], sort: "air" },
    /** /anime/share?p=recent: 近 5 年首播, 不分观看状态, 表格按年份看 */
    recent: { label: "近 5 年", year: "5", sort: "air", view: "table" },
    /** /anime/share?p=loli: 站长看过的ロリ番 */
    loli: { label: "看过的ロリ番", status: [WatchStatus.WATCHED], tags: ["ロリ"], sort: "air" },
};
