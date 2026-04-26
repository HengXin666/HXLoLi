import type * as Preset from '@docusaurus/preset-classic';
import type { Config, PluginConfig } from '@docusaurus/types';
import { themes as prismThemes } from 'prism-react-renderer';
import rehypeKatex from 'rehype-katex'; // katex渲染
import remarkGithubAlerts from 'remark-github-alerts'; // Github tip标签渲染
import remarkMath from 'remark-math'; // 数学渲染

// 基础路径, 末尾不带 '/'
// 通过环境变量 DEPLOY_TARGET 来区分部署目标:
//   DEPLOY_TARGET=cloudflare => baseUrl = "", url = "https://hxloli.pages.dev"
//   默认 (GitHub Pages)     => baseUrl = "/HXLoLi", url = "https://HengXin666.github.io"
const isCloudflare = process.env.DEPLOY_TARGET === 'cloudflare';
const BaseUrl = isCloudflare ? "" : "/HXLoLi";
// Workers 部署: 使用 wrangler deploy (Static Assets 模式)
// Pages 部署已迁移至 Workers, URL 保持不变 (自定义域名或 .workers.dev)

// 插件配置
const plugins: PluginConfig[] = [
  // 你原来的 postcss 插件, 原样保留
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
];


// 站点配置
const config: Config = {
  title: "HXLoLi", // 项目名称
  tagline: "ここから先は一方通行だ!", // 项目的 tagline（副标题）
  favicon: "img/favicon.ico", // 项目图标, 可以根据实际情况更换

  // 站点的URL, 根据部署目标动态切换
  url: isCloudflare ? "https://hxloli.pages.dev" : "https://HengXin666.github.io",
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
              to: "/friends",
              label: "友链",
            },
            {
              href: "https://github.com/HengXin666/HXLoLi", // 项目的 GitHub 地址
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
