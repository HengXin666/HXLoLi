import type * as Preset from '@docusaurus/preset-classic';
import type { Config, PluginConfig } from '@docusaurus/types';
import { themes as prismThemes } from 'prism-react-renderer';
import rehypeKatex from 'rehype-katex'; // katex渲染
import remarkGithubAlerts from 'remark-github-alerts'; // Github tip标签渲染
import remarkMath from 'remark-math'; // 数学渲染

// 基础路径, 末尾不带 '/'
// 通过环境变量 DEPLOY_TARGET 来区分部署目标:
//   DEPLOY_TARGET=cloudflare => baseUrl = "", url = "https://km.woa.qzz.io"
//   默认 (GitHub Pages)     => baseUrl = "/HXLoLi", url = "https://HengXin666.github.io"
const isCloudflare = process.env.DEPLOY_TARGET === 'cloudflare';
const BaseUrl = isCloudflare ? "" : "/HXLoLi";
// Workers 部署: 使用 wrangler deploy (Static Assets 模式)
// 自定义域名: km.woa.qzz.io

type PptHtmlCopyPattern = {
  from: string;
  to: string;
  noErrorOnMissing: boolean;
};

type PptHtmlContentSection = {
  contentDir: string;
  routeBasePath: string;
  kind: 'docs' | 'blog';
};

function toPosixPath(filePath: string): string {
  return filePath.replace(/\\/g, '/');
}

function joinUrlPath(parts: Array<string | undefined>): string {
  return parts
    .filter((part): part is string => Boolean(part))
    .join('/')
    .replace(/\/+/g, '/')
    .replace(/^\/+|\/+$/g, '');
}

function stripNumberPrefix(name: string): string {
  if (/^\d+[-_.]\d+/.test(name)) return name;

  const match = /^(?<numberPrefix>\d+)\s*[-_.]+\s*(?<suffix>[^-_.\s].*)$/.exec(name);
  return match?.groups?.suffix ?? name;
}

function stripPathNumberPrefixes(filePath: string): string {
  return filePath
    .split('/')
    .map(stripNumberPrefix)
    .join('/');
}

function getDocsMarkdownRoute(rootDir: string, markdownFile: string, routeBasePath: string): string {
  const path = require('path') as typeof import('path');
  const relativeFile = toPosixPath(path.relative(rootDir, markdownFile));
  const sourceDirName = path.posix.dirname(relativeFile);
  const fileName = path.posix.basename(relativeFile, path.posix.extname(relativeFile));
  const baseId = stripNumberPrefix(fileName);
  const dirSlug = sourceDirName === '.' ? '' : stripPathNumberPrefixes(sourceDirName);
  const nearestDir = sourceDirName === '.' ? undefined : sourceDirName.split('/').at(-1)?.toLowerCase();
  const categoryIndexNames = ['index', 'readme', nearestDir].filter(Boolean);
  const isCategoryIndex = categoryIndexNames.includes(fileName.toLowerCase());

  return isCategoryIndex
    ? joinUrlPath([routeBasePath, dirSlug])
    : joinUrlPath([routeBasePath, dirSlug, baseId]);
}

function getBlogMarkdownRoute(rootDir: string, markdownFile: string, routeBasePath: string): string {
  const path = require('path') as typeof import('path');
  const relativeFile = toPosixPath(path.relative(rootDir, markdownFile));
  const dateFileNameMatch = relativeFile.match(
    /^(?<folder>.*)(?<date>\d{4}[-/]\d{1,2}[-/]\d{1,2})[-/]?(?<text>.*?)(?:\/index)?\.mdx?$/
  );

  if (dateFileNameMatch?.groups) {
    const slugDate = dateFileNameMatch.groups.date.replace(/-/g, '/');
    return joinUrlPath([
      routeBasePath,
      slugDate,
      `${dateFileNameMatch.groups.folder ?? ''}${dateFileNameMatch.groups.text ?? ''}`,
    ]);
  }

  return joinUrlPath([
    routeBasePath,
    relativeFile.replace(/(?:\/index)?\.mdx?$/, ''),
  ]);
}

function getPptHtmlCopyPatterns(siteDir: string): PptHtmlCopyPattern[] {
  const fs = require('fs') as typeof import('fs');
  const path = require('path') as typeof import('path');

  const sections: PptHtmlContentSection[] = [
    { contentDir: 'docs', routeBasePath: 'docs', kind: 'docs' },
    { contentDir: 'ai-docs', routeBasePath: 'knowledge-base', kind: 'docs' },
    { contentDir: 'blog', routeBasePath: 'blog', kind: 'blog' },
  ];

  function listFiles(dir: string, predicate: (file: string) => boolean): string[] {
    if (!fs.existsSync(dir)) return [];

    const files: string[] = [];
    const entries = fs.readdirSync(dir, { withFileTypes: true });

    for (const entry of entries) {
      if (entry.name.startsWith('.')) continue;

      const fullPath = path.join(dir, entry.name);
      if (entry.isDirectory()) {
        files.push(...listFiles(fullPath, predicate));
      } else if (entry.isFile() && predicate(fullPath)) {
        files.push(fullPath);
      }
    }

    return files;
  }

  const patterns = new Map<string, PptHtmlCopyPattern>();

  for (const section of sections) {
    const rootDir = path.join(siteDir, section.contentDir);
    const markdownFiles = listFiles(rootDir, file => /\.mdx?$/i.test(file));
    const htmlFiles = listFiles(rootDir, file => /\.html?$/i.test(file));
    const markdownFilesByDir = new Map<string, string[]>();

    for (const markdownFile of markdownFiles) {
      const dir = path.dirname(markdownFile);
      const dirFiles = markdownFilesByDir.get(dir) ?? [];
      dirFiles.push(markdownFile);
      markdownFilesByDir.set(dir, dirFiles);
    }

    for (const htmlFile of htmlFiles) {
      const markdownSiblings = markdownFilesByDir.get(path.dirname(htmlFile));
      if (!markdownSiblings) continue;

      for (const markdownFile of markdownSiblings) {
        const route = section.kind === 'docs'
          ? getDocsMarkdownRoute(rootDir, markdownFile, section.routeBasePath)
          : getBlogMarkdownRoute(rootDir, markdownFile, section.routeBasePath);
        const htmlBaseName = path.basename(htmlFile);
        const to = joinUrlPath([route, htmlBaseName]);
        patterns.set(`${htmlFile}->${to}`, {
          from: htmlFile,
          to,
          noErrorOnMissing: true,
        });

        const cleanRouteName = htmlBaseName.replace(/\.html?$/i, '');
        const cleanUrlTo = joinUrlPath([route, cleanRouteName, 'index.html']);
        patterns.set(`${htmlFile}->${cleanUrlTo}`, {
          from: htmlFile,
          to: cleanUrlTo,
          noErrorOnMissing: true,
        });
      }
    }
  }

  return [...patterns.values()];
}

// 插件配置
const plugins: PluginConfig[] = [
  // postcss 插件
  function myPlugin() {
    return {
      name: "postcss-tailwindcss-loader",
      configurePostCss(postcssOptions: any) {
        postcssOptions.plugins.push(
          require("postcss-import"),
          require("tailwindcss"),
          require("postcss-nested"),
          require("autoprefixer"),
        );
        return postcssOptions;
      },
    };
  },
  // 将与 Markdown 同目录的 HTML 资源发布到对应页面路由下, 供 #ppt iframe 使用
  function pptHtmlAssetsPlugin(context: { siteDir: string }) {
    const siteDir = context.siteDir;
    return {
      name: "ppt-html-assets",
      getPathsToWatch() {
        return [
          "docs/**/*.html",
          "ai-docs/**/*.html",
          "blog/**/*.html",
        ];
      },
      configureWebpack(_config: unknown, isServer: boolean) {
        if (isServer) return {};

        const patterns = getPptHtmlCopyPatterns(siteDir);
        if (patterns.length === 0) return {};

        const CopyWebpackPlugin = require("copy-webpack-plugin");
        return {
          plugins: [
            new CopyWebpackPlugin({ patterns }),
          ],
        };
      },
    };
  },
  // docs 链接关系图数据生成插件
  function docsGraphPlugin() {
    return {
      name: "docs-links-graph",
      async loadContent() {
        const { execSync } = require("child_process");
        try {
          execSync("node scripts/generate-docs-graph.mjs", { stdio: "inherit" });
        } catch {}
      },
    };
  },
  // docs RSS/Atom feed + latest docs data 生成插件
  // 类似博客插件的 feed 生成，但是针对 docs (笔记)
  require('./plugins/docs-rss-plugin.mjs'),
  // AI 知识库: 第二个 @docusaurus/plugin-content-docs 实例
  // 独立于主文档 (docs/), 内容存储在 ai-docs/ 目录
  // 路由: /knowledge-base
  [
    '@docusaurus/plugin-content-docs',
    {
      id: 'ai-docs',
      path: 'ai-docs',
      routeBasePath: 'knowledge-base',
      include: ['**/*.{md,mdx}'],
      sidebarPath: './sidebarsAiDocs.ts',
      remarkPlugins: [remarkGithubAlerts, remarkMath],
      rehypePlugins: [
        [
          rehypeKatex,
          {
            strict: false,
            errorColor: '#cc0000',
          },
        ],
      ],
      showLastUpdateTime: true,
      showLastUpdateAuthor: true,
    } satisfies import('@docusaurus/plugin-content-docs').Options,
  ],
];


// 站点配置
const config: Config = {
  title: "HXLoLi", // 项目名称
  tagline: "ここから先は一方通行だ!", // 项目的 tagline（副标题）
  favicon: "img/favicon.ico", // 项目图标, 可以根据实际情况更换

  // 站点的URL, 根据部署目标动态切换
  url: isCloudflare ? "https://km.woa.qzz.io" : "https://HengXin666.github.io",
  baseUrl: BaseUrl || "/", // Cloudflare 为 "/", GitHub Pages 为 "/HXLoLi"
  trailingSlash: false,

  // GitHub Pages 部署配置, 修改为你的 GitHub 项目名称
  organizationName: "HengXin666", // GitHub 用户名或组织名
  projectName: "HXLoLi", // GitHub 项目名称

  onBrokenLinks: "warn", // 如果链接损坏则发出警告
  onBrokenMarkdownLinks: "warn", // Markdown 链接损坏警告

  // 国际化配置
  i18n: {
    defaultLocale: "zh-Hans", // 默认语言为简体中文
    locales: ["zh-Hans"], // 只支持简体中文
  },

  plugins: plugins,

  // 使用 presets 配置
  presets: [
    [
      "@docusaurus/preset-classic",
      {
        docs: {
          include: ["**/*.{md,mdx}"],
          sidebarPath: "./sidebars.ts", // 引入自定义的侧边栏配置文件
          remarkPlugins: [remarkGithubAlerts, remarkMath],
          rehypePlugins: [
            [
              rehypeKatex,
              {
                strict: false, // 设置为 false 禁用严格模式
                errorColor: "#cc0000", // 可以自定义错误颜色
              },
            ],
          ],
          editUrl: "https://github.com/HengXin666/HXLoLi/edit/main/", // 文档编辑链接, 指向 GitHub 项目
          showLastUpdateTime: true,   // 显示最后编辑时间
          showLastUpdateAuthor: true, // 显示更新作者
        },
        blog: {
          blogSidebarTitle: '所有文章', // 侧边栏标题
          blogSidebarCount: 'ALL',     // 显示所有的文章
          remarkPlugins: [remarkGithubAlerts, remarkMath],
          rehypePlugins: [
            [
              rehypeKatex,
              {
                strict: false, // 设置为 false 禁用严格模式
                errorColor: "#cc0000", // 可以自定义错误颜色
              },
            ],
          ],
          feedOptions: {
            type: ["rss", "atom"], // 支持的博客订阅格式
            xslt: true,
            createFeedItems: async ({ blogPosts, defaultCreateFeedItems, siteConfig, outDir }) => {
              // 排除加密的私有博客文章
              const filtered = blogPosts.filter(
                (post) => !post.metadata.frontMatter.hx_protected,
              );
              return defaultCreateFeedItems({ blogPosts: filtered, siteConfig, outDir });
            },
          },
          editUrl: "https://github.com/HengXin666/HXLoLi/edit/main/", // 博客编辑链接, 指向 GitHub 项目
          showReadingTime: true,      // 显示博客阅读时间
          showLastUpdateTime: true,   // 显示最后编辑时间
          showLastUpdateAuthor: true, // 显示更新作者
        },
        pages: {
          remarkPlugins: [remarkGithubAlerts, remarkMath],
          rehypePlugins: [rehypeKatex],
          showLastUpdateTime: true,   // 显示最后编辑时间
          showLastUpdateAuthor: true, // 显示更新作者
        },
        theme: {
          // 可以放置自定义的 CSS 样式
          customCss: [
            "./src/css/custom.css",
            "./src/css/ai-kb.css",
            "./static/katex/katex.css",
          ],
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    colorMode: {
      defaultMode: 'dark',              // 默认黑夜模式
      disableSwitch: true,              // 禁用模式切换按钮
      respectPrefersColorScheme: false, // 不根据用户操作系统的偏好切换
    },
    // 评论设置
    giscus: {
      // 此处获取配置: https://giscus.app/zh-CN
      repo: 'HengXin666/HXLoLi',
      repoId: 'R_kgDOOfrrwA',
      category: 'General',
      categoryId: 'DIC_kwDOOfrrwM4CpdxE',
      // 颜色主题
      theme: 'light_high_contrast',
      darkTheme: 'dark_tritanopia',
    },
    // 项目的社交卡片图像
    image: "img/logo.png",

    // 导航栏配置
    navbar: {
      hideOnScroll: true, // 自动隐藏导航栏
      title: "HXLoLi", // 导航栏标题
      logo: {
        alt: "HXLoLi Logo",
        src: "img/logo.png", // 站点的 logo 图片
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar", // 侧边栏ID
          position: "left",
          label: "笔记", // 导航栏标签
        },
        {
          to: "/blog", // 跳转到博客页面
          label: "博客",
          position: "left",
        },
        {
          type: "docSidebar",
          sidebarId: "aiDocsSidebar",
          docsPluginId: "ai-docs",
          position: "left",
          label: "知识库",
        },
        {
          to: "/anime",
          label: "アニメ",
          position: "left",
        },
        {
          type: "custom-cdnSelector" as any,
          position: "right",
        },
        {
          type: "custom-musicPlayer" as any,
          position: "right",
        },
        {
          label: "更多",
          position: "right",
          items: [
            {
              to: "/blog/archive",
              label: "归档",
            },
            {
              to: "/docs-graph",
              label: "笔记关系图",
            },
            {
              to: "/friends",
              label: "友链",
            },
            {
              href: "https://github.com/HengXin666/HXLoLi",
              label: "GitHub",
            },
          ]
        },
      ],
    },

    // 页脚配置
    footer: {
      style: "dark", // 页脚风格
      links: [
        {
          title: "社区",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/HengXin666/HXLoLi",
            },
          ],
        },
        {
        },
        {
          title: '友情链接',
          items: [
            {
              html: `
                <a href="https://hengxin666.github.io/cppreference-zh-cn/" target="_blank" rel="noreferrer noopener"
                  style="display: flex; align-items: center; gap: 3px; text-decoration: none; color: inherit; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;">
                  <img src="${BaseUrl}/icons/ISO_C++_Logo.svg" alt="cppreference" width="36" height="36" style="flex-shrink: 0;"/>
                  <span style="font-size: 16px; font-weight: 600; position: relative; top: -1.75px;">cppreference</span>
                </a>
                <div style="height: 10px;"></div>
              `
            },
            {
              html: `
                <a href="https://bgm.tv" target="_blank" rel="noreferrer noopener">
                  <img src="${BaseUrl}/img/BanGuMi_Logo.png" alt="BanGuMi" height="50"/>
                </a>
              `
            },
            {
              html: `
                <a href="https://docusaurus.io" target="_blank" rel="noreferrer noopener">
                  <img src="${BaseUrl}/default-img/buildwith.png" alt="build with docusaurus" height="50"/>
                </a>
                `,
            },
          ],
        },
      ],
      // 页脚版权信息
      copyright: `版权所有 © 2025 - ${new Date().getFullYear()} HXLoLi, Inc. 在 🇨🇳 用 ❤️ 制作.`,
    },

    // 代码块配置
    prism: {
      theme: prismThemes.oneDark,
      // 它必须在 node_modules/prismjs/components 中
      // 一般是 https://prismjs.com/#supported-languages 中的每一项的第一个
      // 如: Markup - markup, html, xml, svg, mathml, ssml, atom, rss
      // 它们都是 markup
      additionalLanguages: [
        // C++系列
        'cpp', 'cmake', 'makefile',
        // 前端系列
        'markup', 'css', 'javascript', 'typescript', 'scss', 'sass', 'qml', 'jsx', 'tsx',
        // 主流语言
        'java', 'go', 'python', 'kotlin', 'lua', 'php', 'rust', 'csharp', 'ruby',
        // 配置文件语言
        'docker', 'ini', 'json', 'yaml',
        // 其他
        'sql', 'powershell', 'bash', 'markdown',
      ],
    },

    // TOC标题配置, 支持渲染 h2 ~ h6 的标题 (默认是 h2 ~ h4)
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 6,
    },
  } satisfies Preset.ThemeConfig,

  // 支持渲染 Mermaid 图表, 但是我们自己渲染! 以支持兼容组合代码块
  markdown: {
    mermaid: false,
    format: 'detect', // 自动根据文件扩展名选择格式 (而不是默认的mdx!)
  },
  themes: [
    '@docusaurus/theme-mermaid',
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      {
        // `hashed` 被推荐作为索引文件的长期缓存
        hashed: true,
        language: ["en", "zh"],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
        // 排除受保护页面不被全文搜索索引 (占位页面不含真实内容, 但标题也应排除)
        // 可以通过 CSS 选择器排除 ProtectedPage 组件的内容
        ignoreCssSelectors: [
          "[data-hx-protected='true']",    // 排除受保护标记元素
          ".protectedContainer",            // 排除锁定 UI
        ],
      },
    ],
  ],
};

export default config;
