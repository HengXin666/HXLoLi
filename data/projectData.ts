/**
 * 首页项目轮播的数据源. 范围: 近一年 (2025-10-31 起) 仍在推进的公开仓库, 按最近推送倒序.
 *
 * 只放公开仓库: 私有仓库的 GitHub 链接对访客是 404 (2026-11-01 逐个匿名访问核对, 去掉了 HX-Jungle).
 * 没有流水线自动更新这份数据: .github/workflows/ 里没有任何 schedule/cron, 也没有脚本写这个文件, pushedAt、stats 和 stars 手工维护.
 * stars 是 GitHub 星数, 取自 `gh repo view <owner>/<repo> --json stargazerCount` (2026-11-01), 卡片直接读这个字段; 0 星照写 0.
 *
 * 加项目: 往 projects 追加一条. preview 用 static/ 下的相对路径 (不带前导斜杠),
 * 没有封面就把 preview 留空, 卡片会退化成纯文字.
 * 徽章 badges 只写能在 gh repo view 里对上的数字, 不要在文案里估算.
 * 生态图的关系数据不在这里, 那份在 data/ecosystem.ts.
 */

export const PROJECT_SINCE = "2025-10-31";

export type Tag = {
  label: string;
  description: string;
  color: string;
};

export type TagType =
  | "favorite"
  | "openSource"
  | "product"
  | "design"
  | "large"
  | "personal"
  | "tools";

export type ProjectType =
  | "web"
  | "app"
  | "lib"
  | "commerce"
  | "personal"
  | "toy"
  | "other";

export type Project = {
  title: string;
  description: string;
  preview?: string;
  website: string;
  url: string;
  tags: TagType[];
  type: ProjectType;
  /** GitHub 星数, 手工维护, 取自 gh repo view 的 stargazerCount; 卡片上直接显示, 0 也显示 */
  stars: number;
  /** 近一年的量化指标, 形如 "★ 96" 或 "2026-10-18" */
  stats?: string[];
  /** 最近一次推送日期, 卡片底部按它排序 */
  pushedAt: string;
};

export const projects: Project[] = [
  {
    title: "freebuff-proxy",
    description: "freebuff 反代的一键 Docker 部署. 支持 deepseek-v4.1-flash、GPT 5.6 Luna、gemini-3.8-flash 等模型, 带多账号管理、代理配置和超时自动中断, 避免空跑扣费.",
    preview: "img/ecosystem/freebuff.webp",
    website: "",
    url: "https://github.com/HengXin666/freebuff-proxy",
    tags: ["favorite", "openSource", "product"],
    type: "app",
    stars: 96,
    stats: ["★ 96", "2026-10-18"],
    pushedAt: "2026-10-18",
  },
  {
    title: "HX-UI",
    description: "React 组件库, 设计语言叫 open-vetta. 36 个组件、6 类图表和增量动画, 配一个可交互的文档站, 审查台和控制台都从这里取样式.",
    preview: "img/ecosystem/cover-hx-ui.webp",
    website: "https://hengxin666.github.io/HX-UI/",
    url: "https://github.com/HengXin666/HX-UI",
    tags: ["favorite", "openSource", "design"],
    type: "lib",
    stars: 1,
    stats: ["36 组件", "2026-10-31"],
    pushedAt: "2026-10-31",
  },
  {
    title: "HX-Memory",
    description: "自维护的可插拔记忆层. 用 Port-Adapter 内核接 DSH 和 Codex, 再带一层推广引擎, 把一个仓库里确认过的教训变成别的项目的规则.",
    preview: "img/ecosystem/cover-hx-memory.webp",
    website: "",
    url: "https://github.com/HengXin666/HX-Memory",
    tags: ["openSource", "tools"],
    type: "lib",
    stars: 0,
    stats: ["2026-10-31"],
    pushedAt: "2026-10-31",
  },
  {
    title: "HX-BCP",
    description: "Browser、Cache Proxy、Proxy 三件事合成一条人机双用的调试链路: 指纹浏览器、省流缓存代理和出口代理统一接入, 控制台用 HX-UI 搭.",
    preview: "img/ecosystem/cover-hx-bcp.webp",
    website: "",
    url: "https://github.com/HengXin666/HX-BCP",
    tags: ["openSource", "tools"],
    type: "app",
    stars: 2,
    stats: ["2026-10-31"],
    pushedAt: "2026-10-31",
  },
  {
    title: "HX-AI-Review",
    description: "部署在 Cloudflare 上的代码与上下文审查. 把 review 规则、上下文收集和评论落库收成可复用工作流, 拼在 PR 前面当第一道机器审查.",
    preview: "img/ecosystem/ai-review.webp",
    website: "",
    url: "https://github.com/HengXin666/HX-AI-Review",
    tags: ["openSource", "tools"],
    type: "web",
    stars: 0,
    stats: ["2026-10-31"],
    pushedAt: "2026-10-31",
  },
  {
    title: "HX-DataLock",
    description: "面向公共存储场景的加密 SDK: 存储侧可读但写入方不可读, 只有持有主密码的人在本地能解开. 密文放公开仓库也成立.",
    preview: "img/ecosystem/datalock.webp",
    website: "",
    url: "https://github.com/HengXin666/HX-DataLock",
    tags: ["openSource", "tools"],
    type: "lib",
    stars: 1,
    stats: ["2026-10-31"],
    pushedAt: "2026-10-31",
  },
  {
    title: "HX-Email",
    description: "邮箱聚合管理: 微软、谷歌以及支持 IMAP 的邮箱都能接, 可收发, 还能挂 Cloudflare 临时邮箱. Docker 一键部署.",
    preview: "img/ecosystem/hx-email.webp",
    website: "",
    url: "https://github.com/HengXin666/HX-Email",
    tags: ["openSource", "product"],
    type: "app",
    stars: 3,
    stats: ["★ 3", "2026-10-18"],
    pushedAt: "2026-10-18",
  },
  {
    title: "HXLoLi",
    description: "这份博客本身. 纯静态站, 接入 Bangumi 追番展示、带 ASS 特效歌词的播放器、番剧力导向图和可编辑的笔记系统, 字数已经超过四百万.",
    preview: "img/project/hxloli.png",
    website: "https://hengxin666.github.io/HXLoLi/",
    url: "https://github.com/HengXin666/HXLoLi",
    tags: ["favorite", "openSource", "large"],
    type: "web",
    stars: 18,
    stats: ["★ 18", "2026-10-31"],
    pushedAt: "2026-10-31",
  },
  {
    title: "HXLibs",
    description: "现代 C++20 协程写的网络库. Linux 走 io_uring, Windows 走 IOCP, 自带 HTTP/HTTPS、WebSocket、socks5 代理、JSON 和无宏反射, 支持分块传输与断点续传.",
    preview: "img/project/hxlibs.png",
    website: "",
    url: "https://github.com/HengXin666/HXLibs",
    tags: ["favorite", "openSource", "large"],
    type: "lib",
    stars: 22,
    stats: ["★ 22", "2026-07-30"],
    pushedAt: "2026-07-30",
  },
  {
    title: "HX-Resume",
    description: "简历编辑和版本管理平台. 可导出 HTML、PDF、Markdown 和高清 PNG, 导出时支持打码, 一份内容多种投递形态.",
    preview: "img/ecosystem/hx-resume.webp",
    website: "",
    url: "https://github.com/HengXin666/HX-Resume",
    tags: ["openSource", "product"],
    type: "web",
    stars: 3,
    stats: ["★ 3", "2026-07-27"],
    pushedAt: "2026-07-27",
  },
  {
    title: "hx-pxy",
    description: "Linux 上的多节点代理控制面: 订阅解析、去重、质量检测、分组和多端口出口. 能挂住宅代理, 多种轮换规则.",
    preview: "img/ecosystem/hx-pxy.webp",
    website: "",
    url: "https://github.com/HengXin666/hx-pxy",
    tags: ["openSource", "tools"],
    type: "app",
    stars: 1,
    stats: ["2026-10-21"],
    pushedAt: "2026-10-21",
  },
  {
    title: "HX-Workflows",
    description: "自用工作流总线. bilibili-api 负责 B 站 token 和接口, 其余仓库的定时任务和状态同步都挂在这条线上.",
    preview: "img/ecosystem/cover-hx-workflows.webp",
    website: "",
    url: "https://github.com/HengXin666/HX-Workflows",
    tags: ["openSource", "tools"],
    type: "app",
    stars: 1,
    stats: ["2026-10-25"],
    pushedAt: "2026-10-25",
  },
  {
    title: "HXLoLi-NaGaMe",
    description: "博客的配套浏览器插件, 用来解开站内受保护的页面. Chrome 和 Firefox 都在维护.",
    preview: "img/ecosystem/nagame.webp",
    website: "",
    url: "https://github.com/HengXin666/HXLoLi-NaGaMe",
    tags: ["openSource", "tools"],
    type: "toy",
    stars: 1,
    stats: ["2026-10-25"],
    pushedAt: "2026-10-25",
  },
  {
    title: "nvidia-register",
    description: "NVIDIA BUILD 的全自动注册机: 带识图过验证, 注册完顺手把 API key 建好. 自给自足的免费额度来源.",
    preview: "img/ecosystem/cover-nvidia-register.webp",
    website: "",
    url: "https://github.com/HengXin666/nvidia-register",
    tags: ["openSource", "tools"],
    type: "toy",
    stars: 11,
    stats: ["★ 11", "2026-08-04"],
    pushedAt: "2026-08-04",
  },
  {
    title: "cppreference-zh-cn",
    description: "cppreference 的中文镜像, 部署在 GitHub Pages 上自己查. 顺手把阅读体验和跳转修到能用.",
    preview: "img/ecosystem/cover-cppreference-zh-cn.webp",
    website: "",
    url: "https://github.com/HengXin666/cppreference-zh-cn",
    tags: ["openSource", "tools"],
    type: "web",
    stars: 16,
    stats: ["★ 16", "2026-10-26"],
    pushedAt: "2026-10-26",
  },
  {
    title: "HX-Recall",
    description: "B 站收藏夹清灰工具: 定时从收藏里挑出还没看的好内容推给你. 状态放 HX-Git-DB, 推送走 HX-HouTiKu 的加密通道.",
    preview: "img/ecosystem/cover-hx-recall.webp",
    website: "",
    url: "https://github.com/HengXin666/HX-Recall",
    tags: ["openSource", "tools"],
    type: "app",
    stars: 0,
    stats: ["2026-05-09"],
    pushedAt: "2026-05-09",
  },
  {
    title: "gpt-web-cf-make",
    description: "GPT 网页端 token 保活和注册机平台. refresh_token 续 access_token, 把网页额度变成能直接调用的接口.",
    preview: "img/ecosystem/gpt-web.webp",
    website: "",
    url: "https://github.com/HengXin666/gpt-web-cf-make",
    tags: ["openSource", "tools"],
    type: "app",
    stars: 35,
    stats: ["★ 35", "2026-06-06"],
    pushedAt: "2026-06-06",
  },
];

export const Tags: Record<TagType, Tag> = {
  favorite: {
    label: "喜爱",
    description: "我最喜欢的项目, 一定要去看看!",
    color: "#e9669e",
  },
  openSource: {
    label: "开源",
    description: "开源项目可以提供灵感!",
    color: "#39ca30",
  },
  product: {
    label: "产品",
    description: "与产品相关的项目!",
    color: "#dfd545",
  },
  design: {
    label: "设计",
    description: "有着不错的设计!",
    color: "#a44fb7",
  },
  large: {
    label: "大型",
    description: "大型项目",
    color: "#8c2f00",
  },
  personal: {
    label: "个人",
    description: "个人项目",
    color: "#12affa",
  },
  tools: {
    label: "工具",
    description: "个人编写的小工具",
    color: "#45814A"
  }
};

export const TagList = Object.keys(Tags) as TagType[];

export const projectTypeMap: Record<ProjectType, string> = {
  web: "🖥️ 网站",
  app: "💫 应用",
  lib: "库",
  commerce: "商业项目",
  personal: "👨‍💻 个人",
  toy: "🔫 玩具",
  other: "🗃️ 其他",
};

export const groupByProjects = projects.reduce((group, project) => {
  const { type } = project;
  group[type] = group[type] ?? [];
  group[type].push(project);
  return group;
}, {} as Record<ProjectType, Project[]>);
