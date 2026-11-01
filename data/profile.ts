/**
 * 首页自我介绍区用的静态数据. 全部是本仓库或 gh 上能查到的事实, 不写估算值.
 *
 * 加一条链接: 往 links 追加. 加一块技术栈: 往 stack 的某一组追加.
 * 数字类的都带 asOf, 过时了直接改值, 不要留两套.
 */

export type ProfileStat = {
    label: string;
    value: string;
    hint: string;
};

export type ProfileLink = {
    label: string;
    href: string;
    /** 一句话说明, 直接显示在链接下方 (原来藏在原生 title 里, 触屏看不到) */
    note: string;
    /** 图标, 由首页映射到 react-icons 的 SVG 组件 */
    icon: "github" | "bilibili" | "leetcode" | "mail";
};

export type StackGroup = {
    group: string;
    items: string[];
};

/** 自我介绍用的口径: 数字的采样日期, 改数据时一起改 */
export const PROFILE_AS_OF = "2026-10-31";

export const profileStats: ProfileStat[] = [
    { label: "笔记字数", value: "442 万", hint: "docs/blog 与 ai-docs 累计, 来自 data/wordStats.ts" },
    { label: "公开仓库", value: "88", hint: "HengXin666 名下的非 fork 仓库" },
    { label: "近一年还在推", value: "69", hint: `2025-10-31 之后有推送的仓库, 统计于 ${PROFILE_AS_OF}` },
    { label: "在做的产品线", value: "7", hint: "C++ / 博客 / 自动化 / AI 工程 / 桌面 / 小说 / 其他" },
];

export const stackGroups: StackGroup[] = [
];

/**
 * 首页"找到我": 每条带 SVG 图标与一句说明, 说明直接显示在链接下方.
 * .agents/notes/implemented/feature/2026-10-31-homepage-profile-and-project-index.md
 */
export const profileLinks: ProfileLink[] = [
    { label: "GitHub", href: "https://github.com/HengXin666", note: "源码都在这里", icon: "github" },
    { label: "Bilibili", href: "https://space.bilibili.com/478917126", note: "偶尔投稿", icon: "bilibili" },
    { label: "LeetCode", href: "https://leetcode.cn/u/Heng_Xin/", note: "刷题记录 (已戒网瘾)", icon: "leetcode" },
    { label: "邮件", href: "mailto:hxloli@qq.com", note: "合作走这个", icon: "mail" },
];

/**
 * 首屏的一句话自我介绍. 原来三段 (约 190 字) 压成一句: 第二段的"笔记 -> 库 -> 应用"并进这一句,
 * 第三段讲站点历史, 与首屏无关, 删掉.
 */
export const profileLede =
    "目前正在调教 AI ...";
