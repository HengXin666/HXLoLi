export const projects: Project[] = [
  {
    title: "HXLibs",
    description: "基于C++20协程和io_uring的百万并发Web框架; 支持异步文件读写、Transfer-Encoding分块编码传输文件, 支持断点续传; 支持HTTPS/WebSockt; 聚合类无宏反射到Json等",
    preview: "img/project/hxlibs.png",
    website: "",
    url: "https://github.com/HengXin666/HXLibs",
    tags: ["openSource", "design"],
    type: "lib",
  },
  {
    title: "HXLoLi",
    description: "Heng_Xin的个人博客, 包含本人各种计算机的学习笔记, 超百万字.",
    preview: "img/project/hxloli.png",
    website: "",
    url: "https://github.com/HengXin666/HXLoLi",
    tags: ["openSource", "design"],
    type: "web",
  },
  {
    title: "B站历史弹幕爬虫",
    description: "Python编写, 带GUI的; 通过二分和经验算法, 加速弹幕爬取速度.",
    preview: "img/project/bilibili-danmaku.png",
    website: "",
    url: "https://github.com/HengXin666/BiLiBiLi_DanMu_Crawling",
    tags: ["openSource", "design"],
    type: "app",
  },
  {
    title: "HX-ANiMe",
    description: "Java Spring-Boot + Vue + Python爬虫 的前后端项目. 是一个支持私有部署的力导向图可视化程序, 可以将具有 n : n 关系的数据保存到服务器, 并支持您在任意设备之间实时同步.",
    preview: "img/project/hx-anime.png",
    website: "",
    url: "https://github.com/HengXin666/HX-ANiMe",
    tags: ["openSource", "design"],
    type: "web",
  },
  {
    title: "HXTest",
    description: "现代C++学习、实验、实践 | 个人代码存储库 (内有 C++17无宏反射、tbb并发库的初步学习、std标准库的重现(一点点)、QT-QML等)",
    preview: "img/project/hxtest.png",
    website: "",
    url: "https://github.com/HengXin666/HXTest",
    tags: ["openSource", "design"],
    type: "personal",
  },
];

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
  | "personal";

export type ProjectType =
  | "web"
  | "app"
  | "lib"
  | "commerce"
  | "personal"
  | "toy"
  | "other";

export const projectTypeMap = {
  web: "🖥️ 网站",
  app: "💫 应用",
  lib: "库",
  commerce: "商业项目",
  personal: "👨‍💻 个人",
  toy: "🔫 玩具",
  other: "🗃️ 其他",
};

export type Project = {
  title: string;
  description: string;
  preview?: string;
  website: string;
  url: string;
  tags: TagType[];
  type: ProjectType;
};

export const Tags: Record<TagType, Tag> = {
  favorite: {
    label: "喜爱",
    description: "我最喜欢的网站, 一定要去看看!",
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
    description: "设计漂亮的网站!",
    color: "#a44fb7",
  },
  large: {
    label: "大型",
    description: "大型项目，原多于平均数的页面",
    color: "#8c2f00",
  },
  personal: {
    label: "个人",
    description: "个人项目",
    color: "#12affa",
  },
};

export const TagList = Object.keys(Tags) as TagType[];

export const groupByProjects = projects.reduce((group, project) => {
  const { type } = project;
  group[type] = group[type] ?? [];
  group[type].push(project);
  return group;
}, {} as Record<ProjectType, Project[]>);
