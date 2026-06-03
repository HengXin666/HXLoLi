import Link from '@docusaurus/Link';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { motion } from 'framer-motion';
import React, { type ReactNode, useMemo } from 'react';
import {
  FaCalendarAlt,
  FaCode,
  FaExternalLinkAlt,
  FaMagic,
  FaRobot,
  FaTag,
} from 'react-icons/fa';

import TypewriterText from '../TypewriterText';
import styles from './styles.module.css';

interface AIDocHeaderProps {
  className?: string;
}

const GITHUB_REPO = 'https://github.com/HengXin666/HXLoLi';

export default function AIDocHeader({ className }: AIDocHeaderProps): ReactNode {
  const { frontMatter } = useDoc();
  // 自定义 frontmatter 字段 (Docusaurus 类型未收录)
  const fm = frontMatter as Record<string, unknown>;

  const safeStr = (val: unknown): string | undefined => {
    if (val === null || val === undefined) return undefined;
    if (val instanceof Date) return val.toISOString().split('T')[0];
    return String(val);
  };

  const created = safeStr(fm.created_at);
  const model = safeStr(fm.model);
  const skill = safeStr(fm.skill);
  const tags: string[] | undefined = Array.isArray(fm.tags)
    ? (fm.tags as unknown[]).map((t) => safeStr(t) ?? String(t))
    : undefined;

  const skillUrl = useMemo(() => {
    if (!skill) return undefined;
    return `${GITHUB_REPO}/tree/main/.claude/skills/${skill}`;
  }, [skill]);

  if (!created && !model && !skill && !tags?.length) {
    return null;
  }

  const hasMeta = !!(created || model || skill);

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
  const renderTagBadge = (tag: string) => {
    const tagUrl = `/knowledge-base/tags/${encodeURIComponent(tag)}`;
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

  // 标题行 (纯展示, 不可点击)
  const titleRow = (
    <div className={styles.titleRow}>
      <motion.div
        className={styles.aiIcon}
        variants={itemVariants}
        title="AI 辅助生成内容"
      >
        <FaRobot />
      </motion.div>
      <TypewriterText
        text="AI 生成文档"
        speed={50}
        className={styles.typewriter}
        cursorClassName={styles.typewriterCursor}
      />
    </div>
  );

  return (
    <motion.div
      className={`${styles.aiDocHeader}${className ? ` ${className}` : ''}`}
      variants={containerVariants}
      initial="hidden"
      animate="visible"
    >
      <div className={styles.bgGrid} />
      <div className={styles.metalShine} />

      {hasMeta && (
        <div className={styles.headerContent}>
          {titleRow}

          <div className={styles.metaRow}>
            {created &&
              renderStaticBadge(
                styles.dateBadge,
                <FaCalendarAlt />,
                created,
                '文档创建时间',
              )}

            {model &&
              renderStaticBadge(
                styles.modelBadge,
                <FaRobot />,
                model,
                `辅助编辑的 AI 模型: ${model}`,
              )}

            {skill &&
              renderLinkBadge(
                styles.skillBadge,
                <FaMagic />,
                skill,
                `查看 Skill 源码: ${skill}`,
                skillUrl!,
                true,
              )}

            {tags?.map((tag) => renderTagBadge(tag))}
          </div>
        </div>
      )}

      {/* 仅标签 (无元数据时) */}
      {!hasMeta && tags && tags.length > 0 && (
        <div className={styles.headerContent}>
          <div className={styles.titleRow}>
            <motion.div className={styles.aiIcon} variants={itemVariants}>
              <FaCode />
            </motion.div>
            <TypewriterText
              text="AI 生成文档"
              speed={50}
              className={styles.typewriter}
              cursorClassName={styles.typewriterCursor}
            />
          </div>
          <div className={styles.metaRow}>
            {tags.map((tag) => renderTagBadge(tag))}
          </div>
        </div>
      )}
    </motion.div>
  );
}
