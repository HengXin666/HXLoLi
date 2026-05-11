import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { motion, type Variants } from 'framer-motion';
import type { ReactNode } from 'react';
import React from 'react';

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

export default function LatestDocsSection(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  if (!latestDocs.length) return null;

  return (
    <div className={styles.latestDocsSection}>
      <h2 className={styles.latestDocsTitle}>最新笔记</h2>
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
