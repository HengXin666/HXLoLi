/**
 * EncryptedPage - HXLoLi 加密页面组件
 *
 * 功能:
 *   1. 检测页面是否携带加密数据
 *   2. 监听浏览器插件的自动解密消息 (CustomEvent)
 *   3. 提供密码输入弹窗供手动解密
 *   4. 解密后使用 dangerouslySetInnerHTML 渲染 Markdown HTML
 *
 * 加密格式:
 *   AES-256-GCM, 密钥由 scrypt 从密码派生
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import styles from './styles.module.css';

/** 加密数据结构 */
interface EncryptedDataPayload {
  magic: string;
  version: number;
  algorithm: string;
  kdf: string;
  kdfParams: { N: number; r: number; p: number };
  salt: string;
  iv: string;
  tag: string;
  ciphertext: string;
}

/** 组件 Props */
interface EncryptedPageProps {
  encryptedData: string;  // JSON string of EncryptedDataPayload
  title?: string;
}

/** 将 hex 字符串转为 Uint8Array */
function hexToBytes(hex: string): Uint8Array {
  const bytes = new Uint8Array(hex.length / 2);
  for (let i = 0; i < hex.length; i += 2) {
    bytes[i / 2] = parseInt(hex.substring(i, i + 2), 16);
  }
  return bytes;
}

/** Base64 解码为 Uint8Array */
function base64ToBytes(base64: string): Uint8Array {
  const binaryString = atob(base64);
  const bytes = new Uint8Array(binaryString.length);
  for (let i = 0; i < binaryString.length; i++) {
    bytes[i] = binaryString.charCodeAt(i);
  }
  return bytes;
}

/**
 * 使用 WebCrypto 的 scrypt 替代 (PBKDF2)
 * 注意: 浏览器原生不支持 scrypt, 我们在这里使用纯 JS 实现
 * 或者退回使用 PBKDF2 + 匹配的构建端参数
 *
 * 为了与 Node.js crypto.scryptSync 兼容, 这里使用一个轻量级纯 JS scrypt 实现
 */

/** PBKDF2-SHA256 密钥派生 (WebCrypto) - 用作 scrypt 的备选方案 */
async function deriveKeyPBKDF2(password: string, salt: Uint8Array): Promise<CryptoKey> {
  const enc = new TextEncoder();
  const keyMaterial = await crypto.subtle.importKey(
    'raw',
    enc.encode(password),
    'PBKDF2',
    false,
    ['deriveBits', 'deriveKey']
  );

  return crypto.subtle.deriveKey(
    {
      name: 'PBKDF2',
      salt,
      iterations: 600000,  // 高迭代次数以保证安全
      hash: 'SHA-256',
    },
    keyMaterial,
    { name: 'AES-GCM', length: 256 },
    false,
    ['decrypt']
  );
}

/**
 * 纯 JS scrypt 实现 (基于 RFC 7914)
 * 为了在浏览器中与 Node.js scryptSync 兼容
 */
async function scrypt(password: Uint8Array, salt: Uint8Array, N: number, r: number, p: number, dkLen: number): Promise<Uint8Array> {
  // PBKDF2-HMAC-SHA256 辅助函数
  async function pbkdf2(pass: Uint8Array, salt: Uint8Array, c: number, dkLen: number): Promise<Uint8Array> {
    const key = await crypto.subtle.importKey('raw', pass, 'PBKDF2', false, ['deriveBits']);
    const bits = await crypto.subtle.deriveBits(
      { name: 'PBKDF2', salt, iterations: c, hash: 'SHA-256' },
      key,
      dkLen * 8
    );
    return new Uint8Array(bits);
  }

  // Salsa20/8 核心
  function salsa20_8(B: Uint32Array): void {
    const x = new Uint32Array(16);
    for (let i = 0; i < 16; i++) x[i] = B[i];
    for (let i = 0; i < 8; i += 2) {
      x[ 4] ^= (x[ 0]+x[12])<<7  | (x[ 0]+x[12])>>>25;
      x[ 8] ^= (x[ 4]+x[ 0])<<9  | (x[ 4]+x[ 0])>>>23;
      x[12] ^= (x[ 8]+x[ 4])<<13 | (x[ 8]+x[ 4])>>>19;
      x[ 0] ^= (x[12]+x[ 8])<<18 | (x[12]+x[ 8])>>>14;
      x[ 9] ^= (x[ 5]+x[ 1])<<7  | (x[ 5]+x[ 1])>>>25;
      x[13] ^= (x[ 9]+x[ 5])<<9  | (x[ 9]+x[ 5])>>>23;
      x[ 1] ^= (x[13]+x[ 9])<<13 | (x[13]+x[ 9])>>>19;
      x[ 5] ^= (x[ 1]+x[13])<<18 | (x[ 1]+x[13])>>>14;
      x[14] ^= (x[10]+x[ 6])<<7  | (x[10]+x[ 6])>>>25;
      x[ 2] ^= (x[14]+x[10])<<9  | (x[14]+x[10])>>>23;
      x[ 6] ^= (x[ 2]+x[14])<<13 | (x[ 2]+x[14])>>>19;
      x[10] ^= (x[ 6]+x[ 2])<<18 | (x[ 6]+x[ 2])>>>14;
      x[ 3] ^= (x[15]+x[11])<<7  | (x[15]+x[11])>>>25;
      x[ 7] ^= (x[ 3]+x[15])<<9  | (x[ 3]+x[15])>>>23;
      x[11] ^= (x[ 7]+x[ 3])<<13 | (x[ 7]+x[ 3])>>>19;
      x[15] ^= (x[11]+x[ 7])<<18 | (x[11]+x[ 7])>>>14;
      x[ 1] ^= (x[ 0]+x[ 3])<<7  | (x[ 0]+x[ 3])>>>25;
      x[ 2] ^= (x[ 1]+x[ 0])<<9  | (x[ 1]+x[ 0])>>>23;
      x[ 3] ^= (x[ 2]+x[ 1])<<13 | (x[ 2]+x[ 1])>>>19;
      x[ 0] ^= (x[ 3]+x[ 2])<<18 | (x[ 3]+x[ 2])>>>14;
      x[ 6] ^= (x[ 5]+x[ 4])<<7  | (x[ 5]+x[ 4])>>>25;
      x[ 7] ^= (x[ 6]+x[ 5])<<9  | (x[ 6]+x[ 5])>>>23;
      x[ 4] ^= (x[ 7]+x[ 6])<<13 | (x[ 7]+x[ 6])>>>19;
      x[ 5] ^= (x[ 4]+x[ 7])<<18 | (x[ 4]+x[ 7])>>>14;
      x[11] ^= (x[10]+x[ 9])<<7  | (x[10]+x[ 9])>>>25;
      x[ 8] ^= (x[11]+x[10])<<9  | (x[11]+x[10])>>>23;
      x[ 9] ^= (x[ 8]+x[11])<<13 | (x[ 8]+x[11])>>>19;
      x[10] ^= (x[ 9]+x[ 8])<<18 | (x[ 9]+x[ 8])>>>14;
    }
    for (let i = 0; i < 16; i++) B[i] += x[i];
  }

  // BlockMix
  function blockMix(B: Uint32Array, r: number): Uint32Array {
    const len = 2 * r * 16;
    const Y = new Uint32Array(len);
    const X = new Uint32Array(16);
    for (let i = 0; i < 16; i++) X[i] = B[len - 16 + i];
    for (let i = 0; i < 2 * r; i++) {
      for (let j = 0; j < 16; j++) X[j] ^= B[i * 16 + j];
      salsa20_8(X);
      for (let j = 0; j < 16; j++) Y[(i < r ? i * 2 : (i - r) * 2 + 1) * 16 + j] = X[j];
    }
    // 重排: even 在前, odd 在后
    const result = new Uint32Array(len);
    for (let i = 0; i < r; i++) {
      for (let j = 0; j < 16; j++) {
        result[i * 16 + j] = Y[i * 2 * 16 + j];
        result[(r + i) * 16 + j] = Y[(i * 2 + 1) * 16 + j];
      }
    }
    return result;
  }

  // ROMix
  function roMix(B: Uint32Array, r: number, N: number): Uint32Array {
    const len = 2 * r * 16;
    let X = new Uint32Array(len);
    for (let i = 0; i < len; i++) X[i] = B[i];
    const V: Uint32Array[] = new Array(N);
    for (let i = 0; i < N; i++) {
      V[i] = new Uint32Array(X);
      X = blockMix(X, r);
    }
    for (let i = 0; i < N; i++) {
      const j = X[len - 16] & (N - 1);
      for (let k = 0; k < len; k++) X[k] ^= V[j][k];
      X = blockMix(X, r);
    }
    return X;
  }

  // 将字节数组转为 Uint32Array (little-endian)
  function bytesToUint32(bytes: Uint8Array): Uint32Array {
    const arr = new Uint32Array(bytes.length / 4);
    for (let i = 0; i < arr.length; i++) {
      arr[i] = bytes[i * 4] | (bytes[i * 4 + 1] << 8) | (bytes[i * 4 + 2] << 16) | (bytes[i * 4 + 3] << 24);
    }
    return arr;
  }

  function uint32ToBytes(arr: Uint32Array): Uint8Array {
    const bytes = new Uint8Array(arr.length * 4);
    for (let i = 0; i < arr.length; i++) {
      bytes[i * 4] = arr[i] & 0xff;
      bytes[i * 4 + 1] = (arr[i] >> 8) & 0xff;
      bytes[i * 4 + 2] = (arr[i] >> 16) & 0xff;
      bytes[i * 4 + 3] = (arr[i] >> 24) & 0xff;
    }
    return bytes;
  }

  // 主流程
  const B = await pbkdf2(password, salt, 1, p * 128 * r);
  const B32 = bytesToUint32(B);

  for (let i = 0; i < p; i++) {
    const offset = i * 2 * r * 16;
    const block = new Uint32Array(2 * r * 16);
    for (let j = 0; j < block.length; j++) block[j] = B32[offset + j];
    const mixed = roMix(block, r, N);
    for (let j = 0; j < mixed.length; j++) B32[offset + j] = mixed[j];
  }

  const Bfinal = uint32ToBytes(B32);
  return pbkdf2(password, Bfinal, 1, dkLen);
}

/**
 * AES-256-GCM 解密
 */
async function decryptContent(data: EncryptedDataPayload, password: string): Promise<string> {
  const salt = hexToBytes(data.salt);
  const iv = hexToBytes(data.iv);
  const tag = hexToBytes(data.tag);
  const ciphertext = base64ToBytes(data.ciphertext);

  // 使用 scrypt 派生密钥
  const enc = new TextEncoder();
  const keyBytes = await scrypt(
    enc.encode(password),
    salt,
    data.kdfParams.N,
    data.kdfParams.r,
    data.kdfParams.p,
    32
  );

  // 导入为 CryptoKey
  const key = await crypto.subtle.importKey(
    'raw',
    keyBytes,
    { name: 'AES-GCM' },
    false,
    ['decrypt']
  );

  // 拼接 ciphertext + tag (WebCrypto AES-GCM 期望 tag 附在密文末尾)
  const combined = new Uint8Array(ciphertext.length + tag.length);
  combined.set(ciphertext);
  combined.set(tag, ciphertext.length);

  try {
    const decrypted = await crypto.subtle.decrypt(
      { name: 'AES-GCM', iv, tagLength: 128 },
      key,
      combined
    );
    return new TextDecoder().decode(decrypted);
  } catch {
    throw new Error('解密失败: 密码错误或数据已损坏');
  }
}

/**
 * 简易 Markdown -> HTML 转换
 * 生产环境建议使用 marked 等库, 这里提供基础支持
 */
function simpleMarkdownToHtml(md: string): string {
  // 移除 frontmatter
  let content = md.replace(/^---\n[\s\S]*?\n---\n/, '');

  // 标题
  content = content.replace(/^######\s+(.+)$/gm, '<h6>$1</h6>');
  content = content.replace(/^#####\s+(.+)$/gm, '<h5>$1</h5>');
  content = content.replace(/^####\s+(.+)$/gm, '<h4>$1</h4>');
  content = content.replace(/^###\s+(.+)$/gm, '<h3>$1</h3>');
  content = content.replace(/^##\s+(.+)$/gm, '<h2>$1</h2>');
  content = content.replace(/^#\s+(.+)$/gm, '<h1>$1</h1>');

  // 代码块
  content = content.replace(/```(\w*)\n([\s\S]*?)```/gm, (_, lang, code) =>
    `<pre><code class="language-${lang}">${code.replace(/</g, '&lt;').replace(/>/g, '&gt;')}</code></pre>`
  );

  // 行内代码
  content = content.replace(/`([^`]+)`/g, '<code>$1</code>');

  // 加粗和斜体
  content = content.replace(/\*\*\*(.+?)\*\*\*/g, '<strong><em>$1</em></strong>');
  content = content.replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>');
  content = content.replace(/\*(.+?)\*/g, '<em>$1</em>');

  // 链接和图片
  content = content.replace(/!\[([^\]]*)\]\(([^)]+)\)/g, '<img src="$2" alt="$1" style="max-width:100%"/>');
  content = content.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener">$1</a>');

  // 无序列表
  content = content.replace(/^[-*+]\s+(.+)$/gm, '<li>$1</li>');
  content = content.replace(/(<li>.*<\/li>\n?)+/g, '<ul>$&</ul>');

  // 有序列表
  content = content.replace(/^\d+\.\s+(.+)$/gm, '<li>$1</li>');

  // 水平线
  content = content.replace(/^---+$/gm, '<hr/>');

  // 段落
  content = content.replace(/^(?!<[a-z])((?!^\s*$).+)$/gm, '<p>$1</p>');

  // 清理多余的空行
  content = content.replace(/\n{3,}/g, '\n\n');

  return content;
}

// ============ 组件 ============

export default function EncryptedPage({ encryptedData, title = '加密页面' }: EncryptedPageProps) {
  const [status, setStatus] = useState<'locked' | 'decrypting' | 'decrypted' | 'error'>('locked');
  const [decryptedHtml, setDecryptedHtml] = useState('');
  const [password, setPassword] = useState('');
  const [errorMsg, setErrorMsg] = useState('');
  const [progress, setProgress] = useState('');
  const containerRef = useRef<HTMLDivElement>(null);

  // 解析加密数据
  const data: EncryptedDataPayload = React.useMemo(() => {
    try {
      return JSON.parse(encryptedData);
    } catch {
      return null;
    }
  }, [encryptedData]);

  // 解密处理
  const handleDecrypt = useCallback(async (pwd: string) => {
    if (!data || !pwd) return;

    setStatus('decrypting');
    setProgress('正在派生密钥...');
    setErrorMsg('');

    try {
      // 使用 requestAnimationFrame 让 UI 更新
      await new Promise(r => requestAnimationFrame(r));

      setProgress('正在解密内容...');
      const plaintext = await decryptContent(data, pwd);

      setProgress('正在渲染...');
      const html = simpleMarkdownToHtml(plaintext);
      setDecryptedHtml(html);
      setStatus('decrypted');
    } catch (err: any) {
      setStatus('error');
      setErrorMsg(err.message || '解密失败');
      setTimeout(() => setStatus('locked'), 2000);
    }
  }, [data]);

  // 监听浏览器插件的解密事件
  useEffect(() => {
    const handler = (e: CustomEvent) => {
      if (e.detail?.type === 'HXLOLI_DECRYPT' && e.detail?.password) {
        handleDecrypt(e.detail.password);
      }
    };
    window.addEventListener('hxloli-decrypt', handler as EventListener);

    // 向插件广播: 此页面有加密内容
    window.dispatchEvent(new CustomEvent('hxloli-encrypted-page', {
      detail: {
        type: 'HXLOLI_ENCRYPTED_PAGE',
        magic: data?.magic,
        version: data?.version,
      }
    }));

    return () => {
      window.removeEventListener('hxloli-decrypt', handler as EventListener);
    };
  }, [data, handleDecrypt]);

  // 表单提交
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (password.trim()) {
      handleDecrypt(password.trim());
    }
  };

  // 已解密 - 渲染内容
  if (status === 'decrypted') {
    return (
      <div ref={containerRef} className={styles.decryptedContent}>
        <div className={styles.decryptedBadge}>
          🔓 已解密
        </div>
        <div
          className="markdown"
          dangerouslySetInnerHTML={{ __html: decryptedHtml }}
        />
      </div>
    );
  }

  // 加密锁定界面
  return (
    <div className={styles.encryptedContainer}>
      <div className={styles.lockCard}>
        {/* 锁图标 */}
        <div className={styles.lockIcon}>
          {status === 'decrypting' ? (
            <div className={styles.spinner} />
          ) : status === 'error' ? (
            <span className={styles.errorIcon}>✕</span>
          ) : (
            <svg width="64" height="64" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 2C9.24 2 7 4.24 7 7V10H6C4.9 10 4 10.9 4 12V20C4 21.1 4.9 22 6 22H18C19.1 22 20 21.1 20 20V12C20 10.9 19.1 10 18 10H17V7C17 4.24 14.76 2 12 2ZM12 4C13.66 4 15 5.34 15 7V10H9V7C9 5.34 10.34 4 12 4ZM12 14C13.1 14 14 14.9 14 16C14 17.1 13.1 18 12 18C10.9 18 10 17.1 10 16C10 14.9 10.9 14 12 14Z"
                fill="currentColor" opacity="0.8"/>
            </svg>
          )}
        </div>

        {/* 标题 */}
        <h2 className={styles.lockTitle}>
          {status === 'decrypting' ? '正在解密...' :
           status === 'error' ? '解密失败' :
           '🔒 此页面已加密'}
        </h2>

        {/* 副标题 */}
        <p className={styles.lockSubtitle}>
          {status === 'decrypting' ? progress :
           status === 'error' ? errorMsg :
           '此内容需要密码才能查看。如果你是博主，请使用浏览器插件自动解密。'}
        </p>

        {/* 密码输入 */}
        {(status === 'locked' || status === 'error') && (
          <form onSubmit={handleSubmit} className={styles.passwordForm}>
            <input
              type="password"
              value={password}
              onChange={e => setPassword(e.target.value)}
              placeholder="输入密码..."
              className={styles.passwordInput}
              autoFocus
              disabled={status !== 'locked'}
            />
            <button
              type="submit"
              className={styles.decryptButton}
              disabled={!password.trim() || status !== 'locked'}
            >
              解密
            </button>
          </form>
        )}

        {/* 提示信息 */}
        <div className={styles.lockHint}>
          <span>💡</span>
          <span>此页面使用 AES-256-GCM 加密。普通访客无法解密此内容。</span>
        </div>
      </div>

      {/* 隐藏的加密数据标记 (供浏览器插件识别) */}
      <div
        id="hxloli-encrypted-data"
        data-encrypted="true"
        data-magic={data?.magic}
        data-version={data?.version}
        style={{ display: 'none' }}
      />
    </div>
  );
}
