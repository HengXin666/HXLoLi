export const projects: Project[] = [
  {
    title: "HXLibs",
    description: "现代C++20协程编写的基于io_uring/iocp百万并发服务器; 可异步读写, 支持分块编码传输文件, 支持断点续传; http/websocket、客户端socks5代理、JSON解析、聚合类无宏反射/宏反射支持别名",
    preview: "img/project/hxlibs.png",
    website: "",
    url: "https://github.com/HengXin666/HXLibs",
    tags: ["favorite", "openSource", "design"],
    type: "lib",
  },
  {
    title: "HXLoLi",
    description: "Heng_Xin的个人博客, 包含本人各种计算机的学习笔记, 超百万字.",
    preview: "img/project/hxloli.png",
    website: "",
    url: "https://github.com/HengXin666/HXLoLi",
    tags: ["favorite", "openSource"],
    type: "web",
  },
  {
    title: "B站历史弹幕爬虫",
    description: "Python编写, 带GUI的; 通过二分和经验算法, 加速弹幕爬取速度.",
    preview: "img/project/bilibili-danmaku.png",
    website: "",
    url: "https://github.com/HengXin666/BiLiBiLi_DanMu_Crawling",
    tags: ["openSource", "tools"],
    type: "app",
  },
  {
    title: "HX-ANiMe",
    description: "Java Spring-Boot + Vue + Python爬虫 的前后端项目. 是一个支持私有部署的力导向图可视化程序, 可以将具有 n : n 关系的数据保存到服务器, 并支持您在任意设备之间实时同步.",
    preview: "img/project/hx-anime.png",
    website: "",
    url: "https://github.com/HengXin666/HX-ANiMe",
    tags: ["openSource", "large"],
    type: "web",
  },
  {
    title: "HXTest",
    description: "现代C++学习、实验、实践 | 个人代码存储库 (内有 C++17无宏反射、tbb并发库的初步学习、std标准库的重现(一点点)、QT-QML、OpenGL、Linux-io_uring/win32-iocp+协程、线程池等) 注释清晰完善",
    preview: "img/project/hxtest.png",
    website: "",
    url: "https://github.com/HengXin666/HXTest",
    tags: ["openSource", "personal"],
    type: "personal",
  },
  {
    title: "string-replacer",
    description: "VsCode插件, 可配置的一键替换. (默认是替换部分中文标点为英文字符) (个人做笔记专供)",
    preview: "img/project/vscode-string-replacer.png",
    website: "",
    url: "https://github.com/HengXin666/vscode-string-replacer",
    tags: ["openSource", "personal"],
    type: "personal",
  }
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

export const groupByProjects = projects.reduce((group, project) => {
  const { type } = project;
  group[type] = group[type] ?? [];
  group[type].push(project);
  return group;
}, {} as Record<ProjectType, Project[]>);
