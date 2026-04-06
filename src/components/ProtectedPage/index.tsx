/**
 * ProtectedPage - HXLoLi 受保护页面组件 v2
 *
 * 设计:
 *   - 此组件 **不包含任何解密逻辑**
 *   - 密文 (AES-256-GCM 加密后的 base64) 通过 props 传入, 嵌入 DOM data 属性中
 *   - 浏览器插件读取 DOM 中的密文 → 本地 AES 解密 → 通过 CustomEvent 发送 HTML
 *   - 零网络请求, 纯本地操作
 *   - 普通用户只看到锁定界面, 没有密码输入框
 *   - 解密算法只在插件代码中, 前端完全不知道加密方式
 */

import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

interface ProtectedPageProps {
  /** 魔术标记 (供插件识别) */
  magic: string;
  /** 页面标题 */
  title?: string;
  /** AES-256-GCM 加密后的 base64 密文 */
  cipher: string;
}

export default function ProtectedPage({
  magic,
  title = '受保护页面',
  cipher,
}: ProtectedPageProps) {
  const [status, setStatus] = useState<'locked' | 'decrypting' | 'decrypted'>('locked');
  const [decryptedHtml, setDecryptedHtml] = useState('');
  const [progress, setProgress] = useState('');
  const containerRef = useRef<HTMLDivElement>(null);

  // 监听来自浏览器插件的消息
  useEffect(() => {
    // 为每个组件生成唯一 ID (同一页面可能有多个受保护块)
    const instanceId = `hxloli-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;

    // 插件解密完成后, 通过 CustomEvent 将渲染好的 HTML 发送过来
    const handleDecrypted = (e: CustomEvent) => {
      const detail = e.detail;
      if (detail?.type === 'HXLOLI_DECRYPTED' && detail?.instanceId === instanceId) {
        setDecryptedHtml(detail.html);
        setStatus('decrypted');
      }
    };

    // 插件正在解密
    const handleDecrypting = (e: CustomEvent) => {
      const detail = e.detail;
      if (detail?.type === 'HXLOLI_DECRYPTING' && detail?.instanceId === instanceId) {
        setStatus('decrypting');
        setProgress(detail.message || '正在解密...');
      }
    };

    window.addEventListener('hxloli-decrypted', handleDecrypted as EventListener);
    window.addEventListener('hxloli-decrypting', handleDecrypting as EventListener);

    // 将 instanceId 写入 DOM, 让插件可以关联
    const dataEl = containerRef.current?.querySelector('[data-hx-protected]');
    if (dataEl) {
      dataEl.setAttribute('data-hx-instance', instanceId);
    }

    // 通知插件: 此页面有受保护内容, 需要解密
    window.dispatchEvent(new CustomEvent('hxloli-protected-page', {
      detail: {
        type: 'HXLOLI_PROTECTED_PAGE',
        magic,
        cipher,
        title,
        instanceId,
      }
    }));

    return () => {
      window.removeEventListener('hxloli-decrypted', handleDecrypted as EventListener);
      window.removeEventListener('hxloli-decrypting', handleDecrypting as EventListener);
    };
  }, [magic, cipher, title]);

  // 已解密 - 渲染插件发送过来的 HTML 内容
  if (status === 'decrypted') {
    return (
      <div className={styles.decryptedContent}>
        <div className={styles.decryptedBadge}>
          🔓 内容已解锁
        </div>
        <div
          className="markdown"
          dangerouslySetInnerHTML={{ __html: decryptedHtml }}
        />
      </div>
    );
  }

  // 锁定 / 解密中 界面
  return (
    <div ref={containerRef} className={styles.protectedContainer}>
      {/* 隐藏的密文数据 (供插件 Content Script 读取) */}
      <div
        id="hxloli-protected-data"
        data-hx-protected="true"
        data-hx-magic={magic}
        data-hx-cipher={cipher}
        style={{ display: 'none' }}
      />

      <div className={styles.lockCard}>
        {/* 图标 */}
        <div className={styles.lockIcon}>
          {status === 'decrypting' ? (
            <div className={styles.spinner} />
          ) : (
            <svg width="64" height="64" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 2C9.24 2 7 4.24 7 7V10H6C4.9 10 4 10.9 4 12V20C4 21.1 4.9 22 6 22H18C19.1 22 20 21.1 20 20V12C20 10.9 19.1 10 18 10H17V7C17 4.24 14.76 2 12 2ZM12 4C13.66 4 15 5.34 15 7V10H9V7C9 5.34 10.34 4 12 4ZM12 14C13.1 14 14 14.9 14 16C14 17.1 13.1 18 12 18C10.9 18 10 17.1 10 16C10 14.9 10.9 14 12 14Z"
                fill="currentColor" opacity="0.8"/>
            </svg>
          )}
        </div>

        {/* 标题 */}
        <h2 className={styles.lockTitle}>
          {status === 'decrypting' ? '正在解密...' : '🔒 此页面内容受保护'}
        </h2>

        {/* 说明 */}
        <p className={styles.lockSubtitle}>
          {status === 'decrypting'
            ? progress
            : '此页面内容仅限授权访问。如果你是博主，请确保已安装 HXLoLi-NaGaMe 浏览器插件并完成 GitHub 授权。'
          }
        </p>

        {/* 提示 */}
        {status === 'locked' && (
          <div className={styles.lockHints}>
            <div className={styles.hintItem}>
              <span className={styles.hintIcon}>🧩</span>
              <span>安装 <strong>HXLoLi-NaGaMe</strong> 浏览器插件并授权 GitHub 即可自动解锁</span>
            </div>
            <div className={styles.hintItem}>
              <span className={styles.hintIcon}>🔐</span>
              <span>此页面内容不会出现在搜索结果中</span>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
