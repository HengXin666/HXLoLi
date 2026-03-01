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
