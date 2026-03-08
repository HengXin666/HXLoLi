import { LATEST_COMMIT_ID } from "@site/data/gitVersion";
import { MUSIC_COMMIT_ID } from "@site/data/musicVersion";
import { GITHUB_CONFIG } from "@site/src/config/github";

/**
 * 将相对于项目根目录的文件路径转换为 jsDelivr 的 GitHub CDN URL
 * @param relativePath - 相对于项目根目录的路径 (例如, '/static/img/logo.png')
 * @returns 完整的 jsDelivr CDN URL
 * // 返回: 'https://cdn.jsdelivr.net/gh/your-username/your-repo@main/static/img/cover.jpg'
 */
export function toJsDelivrUrl(relativePath: string): string {
  // 确保输入路径不为 null 或 undefined
  if (!relativePath) {
    return "";
  }
  // 确保路径以 '/' 开头, 以正确拼接
  const cleanPath = relativePath.startsWith("/")
    ? relativePath
    : `/${relativePath}`;
  return `https://cdn.jsdelivr.net/gh/${GITHUB_CONFIG.USER}/${GITHUB_CONFIG.REPO}@${GITHUB_CONFIG.BRANCH}${cleanPath}`;
}

/**
 * 将相对于项目根目录的文件路径转换为 jsDelivr 的 GitHub CDN URL (使用最近的commit 作为 id)
 * @param relativePath - 相对于项目根目录的路径 (例如, '/static/img/logo.png')
 * @returns 完整的 jsDelivr CDN URL
 * // 返回: 'https://cdn.jsdelivr.net/gh/your-username/your-repo@commit/static/img/cover.jpg'
 */
export function toJsDelivrLatestCommitUrl(relativePath: string): string {
  // 确保输入路径不为 null 或 undefined
  if (!relativePath) {
    return "";
  }
  // 确保路径以 '/' 开头, 以正确拼接
  const cleanPath = relativePath.startsWith("/")
    ? relativePath
    : `/${relativePath}`;
  return `https://cdn.jsdelivr.net/gh/${GITHUB_CONFIG.USER}/${GITHUB_CONFIG.REPO}@${LATEST_COMMIT_ID}${cleanPath}`;
}

/**
 * 将相对于 HXLoLi-Music 仓库根目录的文件路径转换为 jsDelivr CDN URL
 * @param relativePath - 相对于 Music 仓库根目录的路径 (例如, '/music/xxx.mp3')
 * @returns 完整的 jsDelivr CDN URL
 */
export function toMusicCdnUrl(relativePath: string): string {
  if (!relativePath) {
    return "";
  }
  const cleanPath = relativePath.startsWith("/")
    ? relativePath
    : `/${relativePath}`;
  return `https://cdn.jsdelivr.net/gh/${GITHUB_CONFIG.USER}/HXLoLi-Music@${MUSIC_COMMIT_ID}${cleanPath}`;
}

/**
 * 本地音乐文件服务器地址 (由 HXLoLi-Music/serve.py 启动)
 * 用于本地开发时直接读取本地 HXLoLi-Music 仓库的资源, 无需等待远程 CDN 更新
 */
export const LOCAL_MUSIC_SERVER = 'http://localhost:9527';

/**
 * 判断当前是否为本地开发环境
 */
export function isLocalDev(): boolean {
  if (typeof window === 'undefined') return false;
  const host = window.location.hostname;
  return host === 'localhost' || host === '127.0.0.1';
}

/**
 * 将相对路径转换为本地音乐服务器 URL
 * @param relativePath - 相对于 Music 仓库根目录的路径
 */
export function toMusicLocalUrl(relativePath: string): string {
  if (!relativePath) return "";
  const cleanPath = relativePath.startsWith("/")
    ? relativePath
    : `/${relativePath}`;
  return `${LOCAL_MUSIC_SERVER}${cleanPath}`;
}
