import React from "react";
import { Tabs, TabItem } from "@site/src/components/common/Tabs";
import { WatchStatus } from "@site/src/utils/anime/types";

/** "全部"不是 WatchStatus 的成员 (那是番剧数据仓库的枚举), 只在页面筛选里用 */
export const ALL_WATCH_STATUS = 0;
export type WatchStatusFilter = WatchStatus | typeof ALL_WATCH_STATUS;

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
export interface WatchStatusTabProps {
    value: WatchStatusFilter;
    onChange: (value: WatchStatusFilter) => void;
}

/**
 * 观看状态页签. 末尾的"全部"给按标签筛选用: 从首页点"萝莉"进来时要看到所有状态下的萝莉番, 不只是在看的.
 * 标签与观看状态口径见同目录上级 AnimeCard/index.tsx 顶部注释
 */
const watchStatusTabs: readonly TabItem<WatchStatusFilter>[] = [
    { value: WatchStatus.WANT_TO_WATCH, label: "想看" },
    { value: WatchStatus.WATCHING, label: "在看" },
    { value: WatchStatus.WATCHED, label: "看过" },
    { value: WatchStatus.ON_HOLD, label: "搁置" },
    { value: WatchStatus.DROPPED, label: "抛弃" },
    { value: ALL_WATCH_STATUS, label: "全部" },
];

export const WatchStatusTabs: React.FC<WatchStatusTabProps> = ({
    value,
    onChange
}) => {
    return (
        <Tabs
            value={value}
            items={watchStatusTabs}
            onChange={onChange}
        />
    );
};
