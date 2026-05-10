import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { PageMetadata, HtmlClassNameProvider, ThemeClassNames } from '@docusaurus/theme-common';
import BlogLayout from '@theme/BlogLayout';
import BlogListPaginator from '@theme/BlogListPaginator';
import BlogPostItems from '@theme/BlogPostItems';
import SearchMetadata from '@theme/SearchMetadata';
import type { Props } from '@theme/BlogListPage';
import BlogListPageStructuredData from '@theme/BlogListPage/StructuredData';
import { motion, type Variants } from 'framer-motion';
import clsx from 'clsx';
import React, { type ReactNode } from 'react';

import { latestDocs } from '@site/data/latestDocs';
import styles from './styles.module.css';

const variants: Variants = {
  from: { opacity: 0.01, y: 50 },
  to: (i) => ({
    opacity: 1,
    y: 0,
    transition: {
      type: 'spring',
      damping: 25,
      stiffness: 100,
      bounce: 0.2,
      duration: 0.3,
      delay: i * 0.1,
    },
  }),
};

function BlogListPageMetadata(props: Props): ReactNode {
  const { metadata } = props;
  const { blogDescription, blogTitle } = metadata;
  return (
    <>
      <PageMetadata title={blogTitle} description={blogDescription} />
      <SearchMetadata tag="blog_posts_list" />
    </>
  );
}

function LatestDocsSection(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  if (!latestDocs.length) return null;

  return (
    <div className={styles.latestDocsSection}>
      <h2 className={styles.latestDocsTitle}>📚 最新笔记</h2>
      <ul className={styles.latestDocsList}>
        {latestDocs.map((doc, i) => (
          <motion.li
            key={doc.to}
            className={styles.latestDocsItem}
            custom={i}
            initial="from"
            animate="to"
            variants={variants}
          >
            <Link to={siteConfig.baseUrl + doc.to.replace(/^\//, '')}>
              <time className={styles.latestDocsTime}>{doc.date}</time>
              <span className={styles.latestDocsCategory}>[{doc.category}]</span>
              <span>{doc.title}</span>
            </Link>
          </motion.li>
        ))}
      </ul>
    </div>
  );
}

function BlogListPageContent(props: Props): ReactNode {
  const { metadata, items, sidebar } = props;
  return (
    <BlogLayout sidebar={sidebar}>
      <LatestDocsSection />
      <BlogPostItems items={items} />
      <BlogListPaginator metadata={metadata} />
    </BlogLayout>
  );
}

export default function BlogListPage(props: Props): ReactNode {
  return (
    <HtmlClassNameProvider
      className={clsx(
        ThemeClassNames.wrapper.blogPages,
        ThemeClassNames.page.blogListPage,
      )}>
      <BlogListPageMetadata {...props} />
      <BlogListPageStructuredData {...props} />
      <BlogListPageContent {...props} />
    </HtmlClassNameProvider>
  );
}
