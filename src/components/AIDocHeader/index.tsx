import Link from '@docusaurus/Link';
import { useDoc, useSidebarBreadcrumbs } from '@docusaurus/plugin-content-docs/client';
import { motion } from 'framer-motion';
import React, { type ReactNode, useMemo } from 'react';
import {
  FaExternalLinkAlt,
  FaMagic,
  FaRobot,
  FaTag,
} from 'react-icons/fa';

import { aiDocTagIndex } from '@site/data/aiDocTags';

import styles from './styles.module.css';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
interface AIDocHeaderProps {
  className?: string;
}

const GITHUB_REPO = 'https://github.com/HengXin666/HXLoLi';

export default function AIDocHeader({ className }: AIDocHeaderProps): ReactNode {
  const { frontMatter } = useDoc();
  // 封面头的分类路径: 取侧边栏面包屑里的分类, 不含当前文档
  const crumbs = useSidebarBreadcrumbs();
  const categoryPath = (crumbs ?? [])
    .filter((item) => item.type === 'category')
    .map((item) => item.label);
  // 自定义 frontmatter 字段 (Docusaurus 类型未收录)
  const fm = frontMatter as Record<string, unknown>;

  const safeStr = (val: unknown): string | undefined => {
    if (val === null || val === undefined) return undefined;
    if (val instanceof Date) return val.toISOString().split('T')[0];
    return String(val);
  };

  const toStrList = (val: unknown): string[] | undefined => {
    if (val === null || val === undefined) return undefined;
    const raw = Array.isArray(val) ? val : [val];
    const list = raw
      .flatMap((item) => (Array.isArray(item) ? item : [item]))
      .map((item) => safeStr(item))
      .filter((item): item is string => !!item);
    return list.length > 0 ? list : undefined;
  };

  const created = safeStr(fm.created_at);
  const model = safeStr(fm.model);
  const skills = toStrList(fm.skill);
  const tags: string[] | undefined = Array.isArray(fm.tags)
    ? (fm.tags as unknown[]).map((t) => safeStr(t) ?? String(t))
    : undefined;

  const skillUrls = useMemo(() => {
    if (!skills) return undefined;
    return skills.map(
      (skill) => `${GITHUB_REPO}/tree/main/.agents/skills/${skill}`,
    );
  }, [skills]);

  if (!created && !model && !skills?.length && !tags?.length) {
    return null;
  }

  const containerVariants = {
    hidden: { opacity: 0, y: -10 },
    visible: {
      opacity: 1,
      y: 0,
      transition: { duration: 0.4, ease: 'easeOut', staggerChildren: 0.06 },
    },
  };

  const itemVariants = {
    hidden: { opacity: 0, scale: 0.9, y: -4 },
    visible: {
      opacity: 1,
      scale: 1,
      y: 0,
      transition: { duration: 0.25, ease: 'easeOut' },
    },
  };

  // 渲染静态徽章 (不可点击)
  const renderStaticBadge = (
    badgeStyles: string,
    icon: ReactNode,
    text: string,
    title: string,
  ) => (
    <motion.span
      key={text}
      className={`${styles.metaBadge} ${badgeStyles}`}
      variants={itemVariants}
      title={title}
    >
      <span className={styles.badgeIcon}>{icon}</span>
      {text}
    </motion.span>
  );

  // 渲染可点击徽章 (外部链接 — Skill → GitHub)
  const renderLinkBadge = (
    badgeStyles: string,
    icon: ReactNode,
    text: string,
    title: string,
    href: string,
    external?: boolean,
  ) => (
    <motion.a
      key={text}
      className={`${styles.metaBadge} ${badgeStyles} ${styles.clickableBadge}`}
      variants={itemVariants}
      title={title}
      href={href}
      target={external ? '_blank' : undefined}
      rel={external ? 'noopener noreferrer' : undefined}
    >
      <span className={styles.badgeIcon}>{icon}</span>
      {text}
      {external && (
        <span className={styles.skillExternalIcon}>
          <FaExternalLinkAlt />
        </span>
      )}
    </motion.a>
  );

  // 渲染标签徽章 — 跳转到知识库标签页
  // permalink 必须来自构建期索引: Docusaurus 用 lodash.kebabCase 生成标签路由
  // ("现代C++" -> 现代c, "HXLibs" -> hx-libs), 直接 encodeURIComponent 会 404。
  const renderTagBadge = (tag: string) => {
    const tagUrl = aiDocTagIndex.tags.find((entry) => entry.label === tag)?.permalink;
    if (!tagUrl) return null;
    return (
      <Link
        key={tag}
        to={tagUrl}
        className={`${styles.metaBadge} ${styles.tagBadge} ${styles.clickableBadge}`}
        title={`查看标签: ${tag}`}
      >
        <motion.span
          style={{ display: 'inline-flex', alignItems: 'center', gap: '0.35rem' }}
          variants={itemVariants}
        >
          <span className={styles.badgeIcon}>
            <FaTag />
          </span>
          {tag}
        </motion.span>
      </Link>
    );
  };

  // 封面头: 左边 小标签 + 分类路径 + 大号等宽日期, 右边 元数据胶囊.
  // 样式跟首页一致, 见 styles.module.css 顶部说明.
  return (
    <motion.div
      className={`${styles.aiDocHeader}${className ? ` ${className}` : ''}`}
      variants={containerVariants}
      initial="hidden"
      animate="visible"
    >
      <div className={styles.cover}>
        <motion.div className={styles.coverMain} variants={itemVariants}>
          <div className={styles.kicker} title="AI 辅助生成内容">
            <FaRobot className={styles.kickerIcon} aria-hidden="true" />
            <span>AI 生成文档</span>
            {categoryPath.length > 0 && (
              <>
                <span className={styles.kickerSep} aria-hidden="true" />
                <span className={styles.kickerPath}>{categoryPath.join(' / ')}</span>
              </>
            )}
          </div>
          {created && (
            <time className={styles.bigDate} dateTime={created} title="文档创建时间">
              {created}
            </time>
          )}
        </motion.div>

        <div className={styles.metaRow}>
          {model &&
            renderStaticBadge(
              styles.modelBadge,
              <FaRobot />,
              model,
              `辅助编辑的 AI 模型: ${model}`,
            )}

          {skills &&
            skills.map((skill, idx) =>
              renderLinkBadge(
                styles.skillBadge,
                <FaMagic />,
                skill,
                `查看 Skill 源码: ${skill}`,
                skillUrls![idx],
                true,
              ),
            )}

          {tags?.map((tag) => renderTagBadge(tag))}
        </div>
      </div>
    </motion.div>
  );
}
