import { GITHUB_CONFIG } from '@site/src/config/github';

/**
 * 将相对于项目根目录的文件路径转换为 jsDelivr 的 GitHub CDN URL。
 *
 * @param relativePath - 相对于项目根目录的路径 (例如, '/static/img/logo.png')。
 * @param githubUser - 你的 GitHub 用户名。
 * @param githubRepo - 你的 GitHub 仓库名称。
 * @param branch - 分支名称, 默认为 'main'。
 * @returns 完整的 jsDelivr CDN URL。
 *
 * @example
 * toJsDelivrUrl(
 *   '/static/img/cover.jpg',
 *   'your-username',
 *   'your-repo'
 * );
 * // 返回: 'https://cdn.jsdelivr.net/gh/your-username/your-repo@main/static/img/cover.jpg'
 */
export function toJsDelivrUrl(
  relativePath: string
): string {
  // 确保输入路径不为 null 或 undefined
  if (!relativePath) {
    return '';
  }
  // 确保路径以 '/' 开头, 以正确拼接
  const cleanPath = relativePath.startsWith('/') ? relativePath : `/${relativePath}`;
  return `https:/fastly.jsdelivr.nett/gh/${GITHUB_CONFIG.USER}/${GITHUB_CONFIG.REPO}@${GITHUB_CONFIG.BRANCH}${cleanPath}`;
}
