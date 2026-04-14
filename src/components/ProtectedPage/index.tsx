/**
 * ProtectedPage - HXLoLi 受保护页面组件 v3
 *
 * 设计:
 *   - 此组件 **不包含任何解密逻辑**
 *   - 密文 (RSA-OAEP + AES-256-GCM 混合加密后的 base64) 通过 props 传入, 嵌入 DOM data 属性中
 *   - 浏览器插件读取 DOM 中的密文 → 本地混合解密 → 通过 DOM 属性 + CustomEvent 双通道发送 HTML
 *   - 主要通过 MutationObserver 监听 DOM 属性变化 (最可靠的 isolated ↔ main world 通信)
 *   - 备选通过 CustomEvent 接收解密结果
 *   - 零网络请求, 纯本地操作
 *   - 普通用户只看到锁定界面, 没有密码输入框
 *   - 解密算法只在插件代码中, 前端完全不知道加密方式
 */

import React, { useState, useEffect, useRef, useMemo } from 'react';
import styles from './styles.module.css';

interface ProtectedPageProps {
  /** 魔术标记 (供插件识别) */
  magic: string;
  /** 页面标题 */
  title?: string;
  /** 混合加密后的 base64 密文 */
  cipher: string;
}

/** 生成稳定的唯一 ID */
let idCounter = 0;
function generateInstanceId(): string {
  return `hxloli-${Date.now()}-${++idCounter}-${Math.random().toString(36).slice(2, 8)}`;
}

export default function ProtectedPage({
  magic,
  title = '受保护页面',
  cipher,
}: ProtectedPageProps) {
  const [status, setStatus] = useState<'locked' | 'decrypting' | 'decrypted'>('locked');
  const [decryptedHtml, setDecryptedHtml] = useState('');
  const [progress, setProgress] = useState('');

  // instanceId 在组件生命周期内保持稳定, 直接写入 JSX
  const instanceId = useMemo(() => generateInstanceId(), []);

  const markerRef = useRef<HTMLDivElement>(null);

  // 监听来自浏览器插件的解密结果
  useEffect(() => {
    const markerEl = markerRef.current;
    if (!markerEl) return;

    // ====== 方案 A (主要): 通过 MutationObserver 监听 DOM 属性变化 ======
    // 插件 content.js 会将解密结果写入 data-hx-decrypted-html 和 data-hx-status 属性
    // MutationObserver 是跨 world (isolated ↔ main) 最可靠的通信方式
    const checkDecrypted = () => {
      const html = markerEl.getAttribute('data-hx-decrypted-html');
      const s = markerEl.getAttribute('data-hx-status');
      if (s === 'decrypted' && html) {
        setDecryptedHtml(html);
        setStatus('decrypted');
        return true;
      }
      return false;
    };

    // 先检查是否已经解密 (插件可能在 React 挂载之前就完成了)
    if (checkDecrypted()) return;

    const observer = new MutationObserver(() => {
      checkDecrypted();
    });
    observer.observe(markerEl, { attributes: true, attributeFilter: ['data-hx-status', 'data-hx-decrypted-html'] });

    // ====== 方案 B (备选): CustomEvent 监听 ======
    const handleDecrypted = (e: CustomEvent) => {
      const detail = e.detail;
      if (detail?.type === 'HXLOLI_DECRYPTED' && detail?.instanceId === instanceId) {
        setDecryptedHtml(detail.html);
        setStatus('decrypted');
      }
    };

    const handleDecrypting = (e: CustomEvent) => {
      const detail = e.detail;
      if (detail?.type === 'HXLOLI_DECRYPTING' && detail?.instanceId === instanceId) {
        setStatus('decrypting');
        setProgress(detail.message || '正在解密...');
      }
    };

    window.addEventListener('hxloli-decrypted', handleDecrypted as EventListener);
    window.addEventListener('hxloli-decrypting', handleDecrypting as EventListener);

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
      observer.disconnect();
      window.removeEventListener('hxloli-decrypted', handleDecrypted as EventListener);
      window.removeEventListener('hxloli-decrypting', handleDecrypting as EventListener);
    };
  }, [magic, cipher, title, instanceId]);

  // 已解密 - 渲染插件发送过来的 HTML 内容
  if (status === 'decrypted') {
    return (
      <div className={styles.decryptedContent}>
        <div
          className="markdown"
          dangerouslySetInnerHTML={{ __html: decryptedHtml }}
        />
      </div>
    );
  }

  // 锁定 / 解密中 界面
  return (
    <div className={styles.protectedContainer}>
      {/* 隐藏的密文数据 (供插件 Content Script 读取)
          关键: data-hx-instance 在 JSX 渲染时就写入, 不依赖 useEffect
          ref 用于 MutationObserver 监听插件写入的 data-hx-decrypted-html 属性 */}
      <div
        ref={markerRef}
        data-hx-protected="true"
        data-hx-magic={magic}
        data-hx-cipher={cipher}
        data-hx-instance={instanceId}
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
