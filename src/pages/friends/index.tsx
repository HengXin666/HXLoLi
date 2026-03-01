import { friendLinks, type FriendLink } from '@site/data/friendLinks';
import BlogWithCats from '@site/src/components/BlogWithCats';
import ZoomImage from '@site/src/components/common/ZoomImage';
import Layout from '@theme/Layout';
import React, { useState } from 'react';
import { FaExternalLinkAlt, FaGithub, FaPlus, FaTimes, FaUserFriends } from 'react-icons/fa';
import styles from './FriendLinks.module.css';

/** GitHub 直接编辑文件的链接 — 非 owner 点击后会自动 fork + 创建 PR */
const EDIT_FILE_URL =
  'https://github.com/HengXin666/HXLoLi/edit/main/data/friendLinks.ts';

/** 友链数据文件在 GitHub 上的查看链接 */
const FILE_VIEW_URL =
  'https://github.com/HengXin666/HXLoLi/blob/main/data/friendLinks.ts';

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

/** 申请友链引导弹窗 */
function ApplyModal ({ open, onClose }: { open: boolean; onClose: () => void }) {
  if (!open) return null;

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        {/* 关闭按钮 */}
        <button className={styles.modalClose} onClick={onClose} aria-label="关闭">
          <FaTimes size={16} />
        </button>

        <h2 className={styles.modalTitle}>📋 友链申请指南</h2>
        <p className={styles.modalSubtitle}>请按照以下步骤操作, 只需几分钟即可完成~</p>

        <div className={styles.modalSteps}>
          {/* 步骤 1 */}
          <div className={styles.stepItem}>
            <div className={styles.stepBadge}>1</div>
            <div className={styles.stepBody}>
              <div className={styles.stepTitle}>点击下方按钮, 前往 GitHub 编辑页面</div>
              <div className={styles.stepDesc}>
                首次操作会提示 <strong>Fork 仓库</strong>, 这是正常流程, 请放心点击!
              </div>
              <ZoomImage src="img/friends/do_pr_01.png" alt="Fork 提示" />
            </div>
          </div>

          {/* 步骤 2 */}
          <div className={styles.stepItem}>
            <div className={styles.stepBadge}>2</div>
            <div className={styles.stepBody}>
              <div className={styles.stepTitle}>
                在 <code>friendLinks</code> 数组末尾添加你的友链信息
              </div>
              <div className={styles.stepDesc}>
                按照文件中已有的格式, 填写站点名称、链接、GitHub 地址等信息即可。
              </div>
              <ZoomImage src="img/friends/do_pr_02.png" alt="Fork 提示" />
            </div>
          </div>

          {/* 步骤 3 */}
          <div className={styles.stepItem}>
            <div className={styles.stepBadge}>3</div>
            <div className={styles.stepBody}>
              <div className={styles.stepTitle}>保存并提交 PR</div>
              <div className={styles.stepDesc}>
                点击 <strong>Propose changes</strong> 按钮, PR 描述会
                <strong>自动填充友链模板</strong>, 按提示填写自检清单后提交即可,
                CI 会自动校验。
              </div>
              {/* TODO: 到时候贴图 — 步骤3示意图 */}
              <div className={styles.stepImgPlaceholder}>📸 示意图 — 提交 PR</div>
            </div>
          </div>
        </div>

        {/* 底部操作按钮 */}
        <div className={styles.modalFooter}>
          <a
            href={EDIT_FILE_URL}
            target="_blank"
            rel="noopener noreferrer"
            className={styles.modalGoBtn}
          >
            <FaGithub size={16} />
            前往 GitHub 编辑
          </a>
        </div>
      </div>
    </div>
  );
}

export default function FriendsPage (): React.ReactElement {
  const [showModal, setShowModal] = useState(false);

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
                src="img/friends/kirakira_misaka.jpg"
                alt="友链页面头部图片"
                className={styles.headerImage}
              />
              <div className={styles.headerImageOverlay} />
              {/* 按钮叠加在图片右上角 —— 手指指向按钮 */}
              <button
                className={styles.addButton}
                onClick={() => setShowModal(true)}
              >
                <FaPlus size={12} />
                申请友链
              </button>
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
              <button
                className={styles.footerLink}
                style={{ background: 'none', border: 'none', cursor: 'pointer', padding: 0, font: 'inherit' }}
                onClick={() => setShowModal(true)}
              >
                {' '}📝 申请友链{' '}
              </button>
              查看操作指南, 在 GitHub 上编辑{' '}
              <code>data/friendLinks.ts</code>
              {' '} 提交 PR 即可!
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

          {/* 申请友链引导弹窗 */}
          <ApplyModal open={showModal} onClose={() => setShowModal(false)} />
        </div>
      </BlogWithCats>
    </Layout>
  );
}
