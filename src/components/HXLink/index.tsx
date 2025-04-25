import React, { useRef, useState } from "react";
import { useLocation } from "@docusaurus/router";
import { useColorMode } from "@docusaurus/theme-common";
import classNames from "classnames";

interface Props {
    title: string;
    url: string;
    open?: "newTab" | "sameTab";
    onSameUrl?: "open" | "refresh" | "disable";
}

/**
 * @todo Heng_Xin 2025-4-25 00:55:03
 * @Bug 不知道为什么, 部署在 github page 时候, 链接颜色默认就是 color = 0
 * 必需要刷新页面或者切换页面才可以正常显示为 color = 1
 * 而本地无法测试出来, 因此我直接让 color = 0 = 1 了; 有需求的, 请自行修复...
 * 
 * 并且仅影响 main 和 hover 字段, 不影响 underScore 字段...
 */

const colors = [
    {   // 明亮主题的样式
        main: "text-fuchsia-400",
        hover: "hover:text-pink-700",
        underScore: "bg-purple-700",
    },
    {   // 暗黑主题的样式
        main: "text-fuchsia-400",
        hover: "hover:text-pink-700",
        underScore: "bg-purple-700",
    },
];

const HXLink = ({
    title,
    url,
    open = "newTab",
    onSameUrl = "refresh",
}: Props) => {
    const [hover, setHover] = useState(false);
    const [leaving, setLeaving] = useState(true);
    const timeoutId = useRef<number | null>(null);
    const color = colors[useColorMode().colorMode === "dark" ? 1 : 0];
    const isSameUrl = useLocation().pathname === url;

    return (
        <a
            href={isSameUrl ? (onSameUrl === "disable" ? undefined : url) : url}
            className={classNames({
                "relative inline-block transition-all duration-300 ": true,
                [color.main]: true,
                [color.hover]: true,
            })}
            onMouseEnter={() => {
                setHover(true);
                if (timeoutId.current) {
                    clearTimeout(timeoutId.current);
                }
                timeoutId.current = window.setTimeout(() => {
                    timeoutId.current = null;
                    setLeaving(false);
                }, 500);
            }}
            onMouseLeave={() => {
                if (timeoutId.current) {
                    clearTimeout(timeoutId.current);
                    timeoutId.current = window.setTimeout(() => {
                        setHover(false);
                        setLeaving(false);
                    }, 300);
                    window.setTimeout(() => {
                        setLeaving(true);
                    }, 350);
                } else {
                    setHover(false);
                    setLeaving(true);
                }
            }}
            target={
                open === "newTab"
                    ? isSameUrl
                        ? onSameUrl === "open"
                            ? "_blank"
                            : undefined
                        : "_blank"
                    : undefined
            }
            rel="noopener noreferrer"
        >
            {title}

            <span
                className={classNames({
                    "absolute right-0 bottom-0 h-0.5 ": true,
                    [color.underScore]: true,
                    "w-0 transition-all duration-500": leaving,
                    "w-full": !leaving,
                })}
            />
            <span
                className={classNames({
                    "absolute left-0 bottom-0 h-0.5": true,
                    [color.underScore]: true,
                    "w-full transition-all duration-500": hover,
                    "w-0": !hover,
                })}
            />
        </a>
    );
};

export default HXLink;