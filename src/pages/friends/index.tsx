import { friendLinks, type FriendLink } from '@site/data/friendLinks';
import BlogWithCats from '@site/src/components/BlogWithCats';
import Layout from '@theme/Layout';
import React from 'react';
import { FaExternalLinkAlt, FaGithub, FaPlus, FaUserFriends } from 'react-icons/fa';
import styles from './FriendLinks.module.css';

/** GitHub 直接编辑文件的链接 — 非 owner 点击后会自动 fork + 创建 PR */
const EDIT_FILE_URL =
  'https://github.com/HengXin666/HXLoLi/edit/main/data/friendLinks.ts';

/** 友链卡片 */
function FriendCard ({ link }: { link: FriendLink }) {
  return (
    <div className={styles.cardWrapper}>
      <a
        href={link.url}
        target="_blank"
        rel="noopener noreferrer"
        className={styles.card}
      >
        <img
          className={styles.avatar}
          src={link.avatar}
          alt={link.name}
          loading="lazy"
          onError={(e) => {
            (e.target as HTMLImageElement).src =
              `https://ui-avatars.com/api/?name=${encodeURIComponent(link.name)}&background=1a1a2e&color=ff88ff&size=120`;
          }}
        />
        <div className={styles.info}>
          <div className={styles.nameRow}>
            <span className={styles.name}>{link.name}</span>
            {/* 点击 @owner 跳转 GitHub 主页 */}
            <a
              href={link.github}
              target="_blank"
              rel="noopener noreferrer"
              className={styles.owner}
              onClick={(e) => e.stopPropagation()}
              title={`访问 ${link.owner} 的 GitHub`}
            >
              @{link.owner}
            </a>
          </div>
          <div className={styles.description}>{link.description}</div>
        </div>
        <FaExternalLinkAlt className={styles.linkIcon} size={14} />
      </a>
    </div>
  );
}

export default function FriendsPage (): React.ReactElement {
  return (
    <Layout title="友链" description="友情链接 - 一起交换链接吧!">
      <BlogWithCats style={{ padding: '20px', backgroundColor: '#2b2b2b', margin: '40px 0px' }}>
        <div className={styles.container}>
          {/* 页面头部 */}
          <div className={styles.header}>
            <div className={styles.headerBg} />
            <h1 className={styles.title}>
              <FaUserFriends className={styles.titleIcon} />
              友情链接
            </h1>
            <p className={styles.subtitle}>✦ 孤独よ溶けてゆけ 世界に続くソラの果てに — 《INNOCENT BLUE》 ✦</p>
            <div className={styles.headerImageWrapper}>
              <img
                src="img/kirakira_misaka.jpg"
                alt="友链页面头部图片"
                className={styles.headerImage}
              />
              <div className={styles.headerImageOverlay} />
              {/* 按钮叠加在图片右上角 —— 手指指向按钮 */}
              <a
                href={EDIT_FILE_URL}
                target="_blank"
                rel="noopener noreferrer"
                className={styles.addButton}
              >
                <FaPlus size={12} />
                申请友链
              </a>
            </div>
          </div>

          {/* 友链网格 */}
          {friendLinks.length > 0 ? (
            <div className={styles.grid}>
              {friendLinks.map((link, idx) => (
                <FriendCard key={`${link.url}-${idx}`} link={link} />
              ))}
            </div>
          ) : (
            <div className={styles.empty}>
              <div className={styles.emptyIcon}>🔗</div>
              <div className={styles.emptyText}>暂无友链</div>
              <p>快来成为第一个友链吧!</p>
            </div>
          )}

          {/* 底部提示 */}
          <div className={styles.footer}>
            <p className={styles.footerText}>
              💡 想要添加友链? 点击
              <a
                href={EDIT_FILE_URL}
                target="_blank"
                rel="noopener noreferrer"
                className={styles.footerLink}
              >
                {' '}📝 编辑友链文件{' '}
              </a>
              直接在 GitHub 上修改{' '}
              <code>data/friendLinks.ts</code>
              {' '}, 保存后会自动创建 PR 并自动合入!
              <br />
              <FaGithub style={{ verticalAlign: 'middle', marginRight: 4 }} />
              <a
                href="https://github.com/HengXin666/HXLoLi"
                target="_blank"
                rel="noopener noreferrer"
                className={styles.footerLink}
              >
                HengXin666/HXLoLi
              </a>
            </p>
          </div>
        </div>
      </BlogWithCats>
    </Layout>
  );
}
