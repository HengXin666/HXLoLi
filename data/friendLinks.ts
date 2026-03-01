/**
 * 友链数据
 *
 * 如果你想添加自己的友链, 请提交 PR 修改此文件!
 *
 * ⚠️ 提交规范:
 * 1. 只允许修改本文件 (data/friendLinks.ts)
 * 2. 在 friendLinks 数组 **末尾** 添加一条记录 (不要插入中间或修改他人记录)
 * 3. `github` 字段必须与你的 GitHub 用户名一致 (CI 会自动校验)
 * 4. 所有链接必须可正常访问 (HTTP 200)
 * 5. `description` 不超过 50 字
 * 6. PR 提交后会自动运行 CI 验证, 通过后需等待仓库维护者人工审核
 *
 * 格式:
 * {
 *   name: "站点名称",
 *   owner: "站长名称 (自定义, 随意填写)",
 *   url: "站点链接 (https://...)",
 *   github: "GitHub 主页链接 (https://github.com/你的用户名)",
 *   avatar: "头像链接 (建议使用 GitHub 头像: https://avatars.githubusercontent.com/u/你的ID)",
 *   description: "一句话简介 (不超过 50 字)",
 * }
 */

export interface FriendLink {
  /** 站点名称 */
  name: string;
  /** 站长 / 所有人名称 */
  owner: string;
  /** 站点链接 */
  url: string;
  /** GitHub 主页链接 */
  github: string;
  /** 头像 / 图标链接 */
  avatar: string;
  /** 博客简介 (一句话) */
  description: string;
}

export const friendLinks: FriendLink[] = [
  // ↓↓↓ 在这里添加你的友链 ↓↓↓
  {
    name: "HXLoLi",
    owner: "Heng_Xin",
    url: "https://hengxin666.github.io/HXLoLi",
    github: "https://github.com/HengXin666",
    avatar: "https://avatars.githubusercontent.com/u/103022267",
    description: "ここから先は一方通行だ!",
  },
  {
    name: "HXLoLi",
    owner: "Heng_Xin",
    url: "https://hengxin666.github.io/HXLoLi",
    github: "https://github.com/HengXin666",
    avatar: "https://avatars.githubusercontent.com/u/103022267",
    description: "ここから先は一方通行だ!",
  },
  // ↑↑↑ 在这里添加你的友链 ↑↑↑
];
