import Link from '@docusaurus/Link';
import { HtmlClassNameProvider, PageMetadata, ThemeClassNames } from '@docusaurus/theme-common';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Heading from '@theme/Heading';
import Unlisted from '@theme/ContentVisibility/Unlisted';
import SearchMetadata from '@theme/SearchMetadata';
import type { Props } from '@theme/DocTagDocListPage';
import React, { type ReactNode, useCallback, useMemo } from 'react';
import { FaArrowLeft, FaFolderOpen, FaHashtag } from 'react-icons/fa';

import { aiDocTagIndex } from '@site/data/aiDocTags';
import { getTagPinyinKeys } from '@site/src/utils/tags/pinyin';

import styles from './styles.module.css';

/**
 * ai-docs 标签详情页 (swizzle 自 @theme/DocTagDocListPage)
 *
 * 原生实现只是一列大标题 + 描述, 看不出"这篇是什么时候写的、讲什么、还有哪些标签"。
 * 这个版本改成卡片网格:
 *   - 每张卡: 标题 + 摘要 + 创建日期 + 该文的其它标签 (可点, 直接"换乘"到别的标签);
 *   - 顶部: 标签名、拼音 (帮助确认读音)、笔记数、返回全部标签;
 *   - 侧边/底部: 相关标签 —— 与当前标签共现次数最多的几个, 用来继续探索。
 *
 * 摘要/日期/共现关系来自构建期索引 data/aiDocTags.ts (plugins/tag-index-plugin.mjs),
 * 标题与链接用页面 props (内容插件给的权威 permalink)。
 */

/** 从 id 里取分类路径, 例如 "知识沉淀/LLM与Agent/xxx" -> ["知识沉淀", "LLM与Agent"] */
function categoryOf(id: string): string[] {
  return id.split('/').slice(0, -1);
}

const DATE_FORMATTER = new Intl.DateTimeFormat('zh-CN', {
  year: 'numeric',
  month: '2-digit',
  day: '2-digit',
});

function formatDate(iso: string): string {
  const date = new Date(iso);
  return Number.isNaN(date.getTime()) ? iso : DATE_FORMATTER.format(date);
}

export default function DocTagDocListPage({ tag }: Props): ReactNode {
  const { label, count, items, allTagsPath } = tag;
  const { siteConfig } = useDocusaurusContext();
  const pinyinKeys = getTagPinyinKeys(label);

  // 构建期索引里这篇标签的元数据 (描述/别名) 与笔记正文摘要
  const meta = useMemo(
    () => aiDocTagIndex.tags.find((entry) => entry.label === label),
    [label],
  );

  /**
   * props 里的 permalink 带 baseUrl 且以 / 结尾, 索引里的不带 baseUrl ——
   * 统一剥成「以 / 开头, 不带 baseUrl」的形式再去索引里找,
   * 免得在两处各写一份"URL 该长什么样"的逻辑。
   */
  const findDoc = useCallback(
    (permalink: string) => {
      const base = siteConfig.baseUrl.replace(/\/$/, '');
      const withoutBase = permalink.startsWith(base) ? permalink.slice(base.length) : permalink;
      const normalized = `/${withoutBase.replace(/^\/+|\/+$/g, '')}`;
      return aiDocTagIndex.docs.find((doc) => doc.permalink === normalized);
    },
    [siteConfig.baseUrl],
  );

  /** 该标签下每篇笔记的索引数据: 摘要 / 日期 / 其它标签 */
  const cards = useMemo(
    () =>
      items.map((item) => {
        const entry = findDoc(item.permalink);
        return {
          id: item.id,
          title: item.title,
          permalink: item.permalink,
          description: entry?.description || item.description || '',
          date: entry?.date ?? '',
          categories: categoryOf(entry?.id ?? item.id),
          otherTags: (entry?.tags ?? []).filter((other) => other !== label),
        };
      }),
    [findDoc, items, label],
  );

  /** 相关标签: 统计与本标签共现的其它标签, 取前 12 个 */
  const relatedTags = useMemo(() => {
    const counter = new Map<string, number>();
    for (const card of cards) {
      for (const other of card.otherTags) {
        counter.set(other, (counter.get(other) ?? 0) + 1);
      }
    }
    return [...counter.entries()]
      .sort((a, b) => b[1] - a[1] || a[0].localeCompare(b[0], 'zh-Hans-CN'))
      .slice(0, 12)
      .map(([otherLabel, hits]) => ({
        label: otherLabel,
        hits,
        permalink: aiDocTagIndex.tags.find((entry) => entry.label === otherLabel)?.permalink,
      }))
      .filter((entry) => entry.permalink);
  }, [cards]);

  /**
   * 同一拼音字母下的兄弟标签, 用于"同字母还有哪些".
   *
   * 必须排除 count === 0 的纯结构 tag (编程语言 / 工程与工具 / 生活杂谈):
   * 它们只在注册表里当 parent 用, 没有笔记挂在自己名下, Docusaurus 不会为
   * count 为 0 的 tag 生成路由 —— 链接过去就是 404。
   */
  const siblingTags = useMemo(
    () =>
      aiDocTagIndex.tags
        .filter(
          (entry) =>
            entry.label !== label &&
            entry.count > 0 &&
            getTagPinyinKeys(entry.label).groupKey === pinyinKeys.groupKey,
        )
        .slice(0, 14),
    [label, pinyinKeys.groupKey],
  );

  const latestFirst = useMemo(
    () => [...cards].sort((a, b) => (a.date < b.date ? 1 : a.date > b.date ? -1 : 0)),
    [cards],
  );

  return (
    /*
     * 与上游一致, **不**包 <Layout>: 内容插件已用 @theme/DocsRoot 提供
     * Layout (navbar/footer), 再包一层会多出一条顶部导航栏。
     */
    <HtmlClassNameProvider className={ThemeClassNames.page.docsTagDocListPage}>
      <PageMetadata
        title={`${label} · 标签`}
        description={meta?.description || `HXLoLi 知识库中打上「${label}」标签的全部笔记`}
      />
      <SearchMetadata tag="doc_tag_doc_list" />
      {tag.unlisted && <Unlisted />}
      <main className={styles.page}>
        <div className={styles.aurora} aria-hidden="true" />
        <div className={styles.grid} aria-hidden="true" />

        <div className={styles.inner}>
          <nav className={styles.breadcrumb} aria-label="面包屑">
            <Link to={allTagsPath} className={styles.backLink}>
              <FaArrowLeft aria-hidden="true" /> 全部标签
            </Link>
            {siblingTags.slice(0, 6).map((sibling) => (
              <Link key={sibling.label} to={sibling.permalink} className={styles.siblingLink}>
                {sibling.label}
              </Link>
            ))}
          </nav>

          <header className={styles.hero}>
            <p className={styles.kicker}>
              <FaHashtag aria-hidden="true" /> TAG / {pinyinKeys.groupKey}
            </p>
            <Heading as="h1" className={styles.title}>
              {label}
            </Heading>
            <p className={styles.subtitle}>
              <code className={styles.pinyinHint}>{pinyinKeys.full}</code>
              <span className={styles.dot}>·</span>
              <strong>{count}</strong> 篇笔记
              {meta?.description ? (
                <>
                  <span className={styles.dot}>·</span>
                  {meta.description}
                </>
              ) : null}
            </p>
            {meta && meta.aliases.length > 0 && (
              <p className={styles.aliases}>
                同义写法: {meta.aliases.map((alias) => (
                  <span key={alias} className={styles.aliasChip}>
                    {alias}
                  </span>
                ))}
              </p>
            )}
          </header>

          {latestFirst.length === 0 ? (
            <p className={styles.empty}>这个标签下暂时还没有笔记。</p>
          ) : (
            <section className={styles.cardGrid} aria-label="笔记列表">
              {latestFirst.map((card) => (
                <article key={card.id} className={styles.card}>
                  <div className={styles.cardTop}>
                    {card.date ? (
                      <time className={styles.cardDate} dateTime={card.date}>
                        {formatDate(card.date)}
                      </time>
                    ) : (
                      <span />
                    )}
                    {card.categories.length > 0 && (
                      <span className={styles.cardCategory}>
                        <FaFolderOpen aria-hidden="true" /> {card.categories.join(' / ')}
                      </span>
                    )}
                  </div>

                  <Heading as="h2" className={styles.cardTitle}>
                    <Link to={card.permalink}>{card.title}</Link>
                  </Heading>

                  {card.description ? (
                    <p className={styles.cardDesc}>{card.description}</p>
                  ) : null}

                  {card.otherTags.length > 0 && (
                    <div className={styles.cardTags}>
                      {card.otherTags.map((other) => {
                        const info = aiDocTagIndex.tags.find((entry) => entry.label === other);
                        return info ? (
                          <Link key={other} to={info.permalink} className={styles.cardTag}>
                            #{other}
                          </Link>
                        ) : (
                          <span key={other} className={styles.cardTag}>
                            #{other}
                          </span>
                        );
                      })}
                    </div>
                  )}
                </article>
              ))}
            </section>
          )}

          {relatedTags.length > 0 && (
            <section className={styles.relatedSection} aria-label="相关标签">
              <Heading as="h2" className={styles.sectionTitle}>
                相关标签
                <span className={styles.sectionHint}>与本标签共现最多</span>
              </Heading>
              <div className={styles.relatedList}>
                {relatedTags.map((related) => (
                  <Link key={related.label} to={related.permalink!} className={styles.relatedChip}>
                    <span>{related.label}</span>
                    <span className={styles.relatedHits}>×{related.hits}</span>
                  </Link>
                ))}
              </div>
            </section>
          )}
        </div>
      </main>
    </HtmlClassNameProvider>
  );
}

