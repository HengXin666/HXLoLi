import React from "react";
import { ANIME_TAG_GROUPS } from "@site/data/animeTags";
import { OTHER_TAG_GROUP, TagCount, TagGroupName } from "@site/src/utils/anime/tags";
import type { TagMatch } from "@site/src/utils/anime/filters";
import styles from "./FilterBar.module.css";

export interface TagFilterProps {
    /** 已选标签 (多选) */
    value: readonly string[];
    match: TagMatch;
    /** 分面计数: 其他条件不变时勾上该标签会剩几部 */
    counts: readonly TagCount[];
    onChange: (tags: string[]) => void;
    onMatchChange: (match: TagMatch) => void;
}

/**
 * 标签多选面板, 按 data/animeTags.ts 的分组 (角色 / 舞台 / 基调 / 其他) 分栏. 每个标签旁是分面计数, 为 0 且未选中的置灰不可点.
 * 选了两个及以上时出现"全部命中 / 任一命中"切换, 并用一句话写明当前语义. 选中本站定义的分类时在下方列出它的判定口径.
 * 只负责展示与回调, 状态在番剧页 (URL) 上.
 * 多选面板的决策见 src/components/anime/Tag/FilterBar.tsx 顶部注释
 */
export const TagFilter: React.FC<TagFilterProps> = ({ value, match, counts, onChange, onMatchChange }) => {
    const groups: TagGroupName[] = [...ANIME_TAG_GROUPS, OTHER_TAG_GROUP];
    const toggle = (tag: string) =>
        onChange(value.includes(tag) ? value.filter((t) => t !== tag) : [...value, tag]);
    const rules = counts.filter((c) => value.includes(c.tag) && c.rule && c.group !== OTHER_TAG_GROUP);

    return (
        <div className={styles.tagPanel}>
            {groups.map((g) => {
                const items = counts.filter((c) => c.group === g);
                if (items.length === 0) return null;
                return (
                    <fieldset key={g} className={styles.tagGroup}>
                        <legend className={styles.groupLabel}>{g}</legend>
                        <div className={styles.chipWrap}>
                            {items.map((c) => {
                                const on = value.includes(c.tag);
                                const dead = !on && c.count === 0;
                                return (
                                    <button
                                        key={c.tag}
                                        type="button"
                                        className={`${styles.chip} ${c.tag === "萝莉" ? styles.chipHot : ""}`}
                                        aria-pressed={on}
                                        disabled={dead}
                                        title={c.rule}
                                        onClick={() => toggle(c.tag)}
                                    >
                                        {c.tag}
                                        <span className={styles.count}>{c.count}</span>
                                    </button>
                                );
                            })}
                        </div>
                    </fieldset>
                );
            })}

            {value.length >= 2 && (
                <div className={styles.matchRow}>
                    <div className={styles.segment} role="radiogroup" aria-label="多个标签的组合方式">
                        <button type="button" role="radio" aria-checked={match === "all"} onClick={() => onMatchChange("all")}>
                            全部命中
                        </button>
                        <button type="button" role="radio" aria-checked={match === "any"} onClick={() => onMatchChange("any")}>
                            任一命中
                        </button>
                    </div>
                    <span className={styles.matchHint}>
                        {match === "all"
                            ? `同时带有 ${value.map((t) => `「${t}」`).join(" 和 ")} 的番`
                            : `带有 ${value.map((t) => `「${t}」`).join(" 或 ")} 任一个的番`}
                    </span>
                </div>
            )}

            {rules.length > 0 && (
                <ul className={styles.rules}>
                    {rules.map((c) => (
                        <li key={c.tag}><b>{c.tag}</b>: {c.rule}</li>
                    ))}
                </ul>
            )}
        </div>
    );
};
