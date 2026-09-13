import Link from '@docusaurus/Link';
import { HtmlClassNameProvider, PageMetadata, ThemeClassNames } from '@docusaurus/theme-common';
import Heading from '@theme/Heading';
import SearchMetadata from '@theme/SearchMetadata';
import type { Props } from '@theme/DocTagsListPage';
import React, { type ReactNode, useMemo, useState } from 'react';
import { FaHashtag, FaSearch, FaTimes } from 'react-icons/fa';

import { aiDocTagIndex } from '@site/data/aiDocTags';
import {
  compareTagLabels,
  getTagPinyinKeys,
  groupTagsByInitial,
  matchesTagQuery,
} from '@site/src/utils/tags/pinyin';

import styles from './styles.module.css';

/**
 * ai-docs 标签总览页 (swizzle 自 @theme/DocTagsListPage)
 *
 * 原生实现 (`listTagsByLetters`) 用 `label[0]` 分组 —— 对中文就是把所有标签按
 * 第一个汉字切开: 「多模态大模型」「短视频」「电脑包」全在"多"那一组, 索引等于没有。
 * 这个版本换成:
 *
 *   1. **拼音首字母索引** —— 记忆系统 -> J, 多模态大模型 -> D (见 src/utils/tags/pinyin.ts);
 *   2. **分级标签云** —— 字号随笔记数变化, 一眼看出哪些是主干概念;
 *   3. **即时搜索** —— 中文原文、拼音全拼 (jiyi)、首字母缩写 (jyxt) 都能命中。
 *
 * 标签本身与计数用页面 props (内容插件给的权威 permalink); 构建期索引
 * data/aiDocTags.ts 只用来补 props 里没有的东西 —— 标签的人工描述与笔记总数。
 */

/** 字号档位: 0 最弱 (1 篇), 3 最强 (主干概念) */
function getTagTier(count: number, maxCount: number): 0 | 1 | 2 | 3 {
  if (maxCount <= 1) return 1;
  const ratio = count / maxCount;
  if (ratio >= 0.6) return 3;
  if (ratio >= 0.3) return 2;
  if (ratio >= 0.12) return 1;
  return 0;
}

/** 由标签名派生一个稳定色相 —— 让每个标签有自己的霓虹色, 而不是清一色洋红 */
function hueOf(label: string): number {
  let hash = 0;
  for (let i = 0; i < label.length; i += 1) {
    hash = (hash * 31 + label.charCodeAt(i)) % 360;
  }
  return hash;
}

function TagChip({
  label,
  count,
  permalink,
}: {
  label: string;
  count: number;
  permalink: string;
}): ReactNode {
  const pinyinKeys = getTagPinyinKeys(label);
  return (
    <Link
      to={permalink}
      className={styles.tagChip}
      style={{ '--tag-hue': hueOf(label) } as React.CSSProperties}
      title={`${label} · ${pinyinKeys.full} · ${count} 篇`}
    >
      <span className={styles.tagChipLabel}>{label}</span>
      <span className={styles.tagChipCount}>{count}</span>
    </Link>
  );
}

export default function DocTagsListPage(props: Props): ReactNode {
  const [query, setQuery] = useState('');
  const [activeLetter, setActiveLetter] = useState<string | null>(null);

  // 构建期索引: 目前只用于标签描述和笔记总数 (props 里没有)
  const metaByLabel = useMemo(
    () => new Map(aiDocTagIndex.tags.map((tag) => [tag.label, tag])),
    [],
  );

  const allTags = props.tags;

  /**
   * 按大类的浏览视图。标签注册表 (.hx-tags.toml) 用 parent 声明了层级
   * (AI Agent -> Harness -> DSH), 构建期索引把它压成了 root (L1 名)。
   *
   * 为什么需要它: 拼音索引能回答"这个标签在哪一屏", 但回答不了"这个概念属于哪里"。
   * 84 个平级标签对新读者是一片没有结构的词云 —— 大类分组才是倒排索引的入口。
   */
  const topicGroups = useMemo(() => {
    const rootOf = new Map(aiDocTagIndex.tags.map((tag) => [tag.label, tag.root ?? '']));
    const buckets = new Map<string, typeof allTags>();
    for (const tag of allTags) {
      const key = rootOf.get(tag.label) || '';
      const bucket = buckets.get(key);
      if (bucket) bucket.push(tag);
      else buckets.set(key, [tag]);
    }
    return [...buckets.entries()]
      .map(([key, tags]) => ({
        key,
        label: key === '' ? '未归类' : key,
        desc: key === '' ? '' : (metaByLabel.get(key)?.description ?? ''),
        tags: [...tags].sort(
          (a, b) => (b.count ?? 0) - (a.count ?? 0) || compareTagLabels(a.label, b.label),
        ),
      }))
      .sort((a, b) => {
        // 大类按笔记数排, "未归类" 永远垫底
        if (a.key === '') return 1;
        if (b.key === '') return -1;
        const sum = (g: { tags: { count?: number }[] }) =>
          g.tags.reduce((acc, t) => acc + (t.count ?? 0), 0);
        return sum(b) - sum(a) || compareTagLabels(a.label, b.label);
      });
  }, [allTags, metaByLabel]);

  const maxCount = useMemo(
    () => Math.max(1, ...allTags.map((tag) => tag.count ?? 0)),
    [allTags],
  );

  const groups = useMemo(() => groupTagsByInitial(allTags), [allTags]);

  /** 字母索引的命中数, 用于在按钮上显示该组标签个数 */
  const groupSizes = useMemo(
    () => new Map(groups.map((group) => [group.key, group.tags.length])),
    [groups],
  );

  const visibleTags = useMemo(() => {
    const scoped = activeLetter
      ? allTags.filter((tag) => getTagPinyinKeys(tag.label).groupKey === activeLetter)
      : allTags;
    const matched = query.trim()
      ? scoped.filter((tag) => matchesTagQuery(tag.label, query))
      : scoped;
    return [...matched].sort(
      (a, b) => (b.count ?? 0) - (a.count ?? 0) || compareTagLabels(a.label, b.label),
    );
  }, [activeLetter, allTags, query]);

  const isBrowsing = !query.trim() && !activeLetter;

  return (
    /*
     * 这里**不能**再包一层 <Layout>: 内容插件已经用 @theme/DocsRoot 把标签页
     * 渲染在 <Layout> 里了 (DocsRoot -> DocVersionRoot -> 标签页), 再包一次
     * 就会出现两条顶部导航栏。
     * 与上游实现保持一致: 只交内容 + PageMetadata。
     */
    <HtmlClassNameProvider className={ThemeClassNames.page.docsTagsListPage}>
      <PageMetadata
        title="标签"
        description="HXLoLi 知识库全部标签 — 按拼音首字母索引, 支持拼音与首字母搜索"
      />
      <SearchMetadata tag="doc_tags_list" />
      <main className={styles.page}>
        <div className={styles.aurora} aria-hidden="true" />
        <div className={styles.grid} aria-hidden="true" />

        <div className={styles.inner}>
          <header className={styles.hero}>
            <p className={styles.kicker}>
              <FaHashtag aria-hidden="true" /> KNOWLEDGE BASE / TAGS
            </p>
            <Heading as="h1" className={styles.title}>
              标签索引
            </Heading>
            <p className={styles.subtitle}>
              <strong>{allTags.length}</strong> 个标签 ·{' '}
              <strong>{aiDocTagIndex.docs.length}</strong> 篇笔记。按
              <em>大类</em>分组浏览, 或按<em>拼音首字母</em>检索; 字号越大说明这个方向沉淀得越多。
            </p>
          </header>

          <section className={styles.toolbar} aria-label="标签筛选">
            <label className={styles.searchBox}>
              <FaSearch className={styles.searchIcon} aria-hidden="true" />
              <input
                type="search"
                className={styles.searchInput}
                value={query}
                onChange={(event) => setQuery(event.target.value)}
                placeholder="搜索标签或拼音, 例如 记忆 / jiyi / jyxt"
                aria-label="搜索标签"
              />
              {query !== '' && (
                <button
                  type="button"
                  className={styles.clearButton}
                  onClick={() => setQuery('')}
                  aria-label="清空搜索"
                >
                  <FaTimes aria-hidden="true" />
                </button>
              )}
            </label>

            <nav className={styles.letterIndex} aria-label="大类导航">
              <button
                type="button"
                className={styles.letterChipActive}
                onClick={() => {
                  setQuery('');
                  setActiveLetter(null);
                }}
              >
                全部
              </button>
              {topicGroups
                .filter((group) => group.key !== '')
                .map((group) => (
                  <button
                    key={group.key}
                    type="button"
                    className={styles.letterChip}
                    onClick={() => setQuery(group.key)}
                    title={group.desc}
                  >
                    {group.label}
                    <span className={styles.letterCount}>{group.tags.length}</span>
                  </button>
                ))}
            </nav>

            <nav className={styles.letterIndex} aria-label="拼音首字母索引">
              <button
                type="button"
                className={
                  activeLetter === null
                    ? `${styles.letterChip} ${styles.letterChipActive}`
                    : styles.letterChip
                }
                onClick={() => setActiveLetter(null)}
              >
                全部
              </button>
              {groups.map((group) => (
                <button
                  key={group.key}
                  type="button"
                  className={
                    activeLetter === group.key
                      ? `${styles.letterChip} ${styles.letterChipActive}`
                      : styles.letterChip
                  }
                  onClick={() =>
                    setActiveLetter((prev) => (prev === group.key ? null : group.key))
                  }
                  title={`${group.key} 组共 ${groupSizes.get(group.key) ?? 0} 个标签`}
                >
                  {group.key}
                  <span className={styles.letterCount}>{groupSizes.get(group.key)}</span>
                </button>
              ))}
            </nav>
          </section>

          {visibleTags.length === 0 ? (
            <p className={styles.empty}>
              没有匹配 “{query}” 的标签
              {activeLetter ? ` (当前限定在 ${activeLetter} 组)` : ''}。
            </p>
          ) : isBrowsing ? (
            <section className={styles.groupList} aria-label="按字母浏览">
              {groups.map((group) => (
                <article key={group.key} className={styles.group} id={`tag-group-${group.key}`}>
                  <Heading as="h2" className={styles.groupTitle}>
                    <span className={styles.groupLetter}>{group.key}</span>
                    <span className={styles.groupMeta}>
                      {group.tags.length} 个标签 ·{' '}
                      {group.tags.reduce((sum, tag) => sum + (tag.count ?? 0), 0)} 篇笔记
                    </span>
                  </Heading>
                  <div className={styles.groupTags}>
                    {group.tags.map((tag) => (
                      <span
                        key={tag.label}
                        className={styles.cloudItem}
                        data-tier={getTagTier(tag.count ?? 0, maxCount)}
                      >
                        <TagChip
                          label={tag.label}
                          count={tag.count ?? 0}
                          permalink={tag.permalink}
                        />
                      </span>
                    ))}
                  </div>
                </article>
              ))}
            </section>
          ) : (
            <section className={styles.cloud} aria-label="筛选结果">
              <p className={styles.resultHint}>
                命中 <strong>{visibleTags.length}</strong> 个标签
                {activeLetter ? ` · ${activeLetter} 组` : ''}
                {query.trim() ? ` · 关键词 “${query.trim()}”` : ''}
                {' · '}
                <button
                  type="button"
                  className={styles.resetLink}
                  onClick={() => {
                    setQuery('');
                    setActiveLetter(null);
                  }}
                >
                  重置
                </button>
              </p>
              <div className={styles.groupTags}>
                {visibleTags.map((tag) => {
                  const meta = metaByLabel.get(tag.label);
                  return (
                    <span
                      key={tag.label}
                      className={styles.cloudItem}
                      data-tier={getTagTier(tag.count ?? 0, maxCount)}
                    >
                      <TagChip
                        label={tag.label}
                        count={tag.count ?? 0}
                        permalink={tag.permalink}
                      />
                      {meta?.description ? (
                        <span className={styles.cloudDesc}>{meta.description}</span>
                      ) : null}
                    </span>
                  );
                })}
              </div>
            </section>
          )}
        </div>
      </main>
    </HtmlClassNameProvider>
  );
}

