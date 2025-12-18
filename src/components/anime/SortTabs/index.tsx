import React from "react";
import { Tabs, TabItem } from "@site/src/components/common/Tabs";
import { WatchStatus } from "@site/src/utils/anime/types";

export enum SortMode {
    BY_ANIME_DATE = 1,
    BY_LAST_UPDATE = 2
}

export interface SortTabProps {
    value: SortMode;
    onChange: (value: SortMode) => void;
}

const sortTabs: readonly TabItem<SortMode>[] = [
    { value: SortMode.BY_ANIME_DATE, label: "番剧首播日期" },
    { value: SortMode.BY_LAST_UPDATE, label: "用户最近更新" },
];

export const SortTabs: React.FC<SortTabProps> = ({
    value,
    onChange
}) => {
    return (
        <Tabs
            value={value}
            items={sortTabs}
            onChange={onChange}
        />
    );
};
