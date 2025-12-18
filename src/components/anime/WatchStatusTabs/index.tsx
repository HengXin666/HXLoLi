import React from "react";
import { Tabs, TabItem } from "@site/src/components/common/Tabs";
import { WatchStatus } from "@site/src/utils/anime/types";

export interface WatchStatusTabProps {
    value: WatchStatus;
    onChange: (value: WatchStatus) => void;
}

const watchStatusTabs: readonly TabItem<WatchStatus>[] = [
    { value: WatchStatus.WANT_TO_WATCH, label: "想看" },
    { value: WatchStatus.WATCHING, label: "在看" },
    { value: WatchStatus.WATCHED, label: "看过" },
    { value: WatchStatus.ON_HOLD, label: "搁置" },
    { value: WatchStatus.DROPPED, label: "抛弃" }
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
