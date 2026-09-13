---
hxid: "hx-d5f4cd15"
title: "Python爬虫库选型调研"
created_at: "2026-07-04"
model: "GPT-5 Codex"
skill: ["hx-docs-sediment"]
authors: "Heng_Xin"
tags: ["Python", "爬虫", "技术选型"]
---

# Python爬虫库选型调研

> [!NOTE]
>
> 当准备写一个 Python 爬虫时, 真正需要先回答的问题不是「哪个库最强」, 而是: 目标页面是否静态? 是否需要登录和浏览器交互?
> 采集规模是否需要调度、重试和持久化? 最终要的是结构化字段, 还是网页正文?

## 0x00 背景

Python 爬虫生态里常见库很多, 但它们并不在同一个层级竞争:

- `requests`、`httpx`、`aiohttp` 解决的是 HTTP 下载问题.
- `BeautifulSoup`、`lxml`、`parsel`、`selectolax` 解决的是 HTML/XML/JSON 解析和字段提取问题.
- `Scrapy`、`Crawlee` 解决的是工程化爬虫调度、队列、重试、限速、持久化等问题.
- `Playwright`、`Selenium`、`DrissionPage` 解决的是浏览器自动化、JS 渲染、登录交互和复杂页面状态问题.
- `trafilatura`、`newspaper4k` 更偏网页正文和新闻文章抽取, 不是通用爬虫框架.

因此这篇笔记的目标不是做一个绝对排行榜, 而是沉淀一个选型基准: 看到目标网站后, 先判断页面形态和工程约束, 再选择库组合.

## 0x01 核心结论

优先按任务形态选库:

| 场景 | 推荐组合 | 取舍判断 |
| --- | --- | --- |
| 静态 HTML、小脚本、一次性采集 | `requests` + `BeautifulSoup` | 学习成本低, 容错好, 适合快速验证. |
| 静态 HTML、字段多、XPath/CSS 规则复杂 | `requests` / `httpx` + `lxml` / `parsel` | `lxml` 能力强, `parsel` 的 CSS/XPath 体验接近 Scrapy 选择器. |
| 高并发 HTTP/API 抓取 | `httpx` 或 `aiohttp` + `selectolax` / `parsel` | `httpx` 同时支持同步和异步 API, `aiohttp` 更 async-first, `selectolax` 偏性能. |
| 长期维护的工程化爬虫 | `Scrapy` | Spider、Request/Response、Downloader、Middleware、Pipeline、Settings 等组件完整, 是成熟基准线. |
| JS 渲染、登录、复杂交互 | `Playwright` | 新项目里优先考虑的浏览器自动化工具, 支持 Chromium、Firefox、WebKit, 同时提供同步和异步 Python API. |
| 传统 WebDriver 或企业测试生态 | `Selenium` | 浏览器兼容和历史生态强, 但写爬虫时通常比 Playwright 更重. |
| 希望 HTTP 爬虫和浏览器爬虫共用框架 | `Crawlee for Python` | 2026 年仍活跃, 适合关注一体化 crawler 体验, 但 Python 生态沉淀仍需和 Scrapy 对比评估. |
| 中文圈浏览器采集和自动化脚本 | `DrissionPage` | Python-only, 强调控制浏览器和收发数据包结合, 适合效率型脚本和 Chromium 场景. |
| 网页正文、新闻文章抽取 | `trafilatura` / `newspaper4k` | 关注正文、元数据、文章抽取, 不应当替代通用爬虫框架. |

不建议新项目优先选:

- `pyppeteer`: 官方仓库已提示长期无人维护, 并建议考虑 `playwright-python`.
- `requests-html`: 最新 PyPI 版本停在 2019 年, 且渲染能力依赖 `pyppeteer`.
- `newspaper3k`: 经典但偏旧, 新项目更应优先看 `newspaper4k`.

## 0x02 关键细节

### 2.1 分层模型

一个可维护的爬虫通常可以拆成 5 层:

| 层级 | 负责内容 | 常用库 |
| --- | --- | --- |
| 下载层 | 请求、连接池、超时、重试、代理、HTTP/2、异步并发 | `requests`, `httpx`, `aiohttp`, `curl_cffi` |
| 解析层 | HTML/XML/JSON 解析, CSS/XPath/JMESPath/正则提取 | `BeautifulSoup`, `lxml`, `parsel`, `selectolax`, `pyquery` |
| 调度层 | URL 队列、去重、限速、失败重试、增量采集、数据管道 | `Scrapy`, `Crawlee` |
| 浏览器层 | JS 渲染、登录态、点击输入、等待、截图、网络监听 | `Playwright`, `Selenium`, `DrissionPage` |
| 内容抽取层 | 正文抽取、新闻解析、元数据、boilerplate 清理 | `trafilatura`, `newspaper4k` |

重要判断: 如果页面真实数据在 XHR/Fetch API 中, 优先复现接口请求, 不要立刻上浏览器自动化. 浏览器层成本最高, 也最容易引入稳定性和资源占用问题.

### 2.2 HTTP 下载层

`requests` 仍然适合大多数同步脚本: API 简单, 资料多, 适合小规模任务和调试. 它不是爬虫框架, 不负责调度、解析和持久化.

`httpx` 更适合希望从同步代码平滑过渡到异步代码的项目. 官方定位是 next-generation HTTP client, 支持同步/异步 API, 也支持 HTTP/1.1 和 HTTP/2.

`aiohttp` 是 asyncio 生态里的成熟 HTTP client/server. 如果项目本身已经是 async-first, 或者需要同时复用 WebSocket、服务端组件, 它比把同步库硬塞进 async 任务更自然.

`curl_cffi` 属于更特殊的下载层工具, 常被用于需要 libcurl/curl-impersonate 能力的场景. 它不替代爬虫框架, 也不应当成为默认选项. 遇到访问限制时, 应先确认目标站点的授权、服务条款、robots 约束和采集频率边界.

### 2.3 解析层

`BeautifulSoup` 的优势是宽容和易读, 适合脏 HTML、快速脚本和教学场景. 它通常需要搭配具体 parser, 例如 Python 内置 `html.parser` 或 `lxml`.

`lxml` 是能力强、性能好的 HTML/XML 处理库, 适合 XPath、XML、结构复杂或性能敏感的解析任务. 学习曲线比 BeautifulSoup 稍高.

`parsel` 来自 Scrapy 生态, 支持 CSS、XPath、JMESPath 和正则, 适合希望用 Scrapy 风格 selector 但不想引入完整 Scrapy 项目的脚本.

`selectolax` 是 Cython 实现的快速 HTML5 parser, 使用 Lexbor/Modest 后端. 如果解析量很大, 且需求主要是 CSS selector 和文本抽取, 它值得单独 benchmark.

`pyquery` 提供接近 jQuery 的查询 API, 对熟悉 jQuery 的人直观. 但它不是 JS 执行环境, 也不负责动态页面渲染.

### 2.4 工程化爬虫层

`Scrapy` 的核心价值不是「更会解析 HTML」, 而是把爬虫工程里的公共问题收进框架: spider 定义爬取行为, request/response 在调度器、下载器、spider 之间流转, middleware 处理横切逻辑, pipeline 处理结果落地. 长期项目、批量站点、多任务采集优先评估它.

`Crawlee for Python` 是近年值得关注的一体化选择. 它覆盖 HTTP-only 和 browser-based scraping, 并提供存储、代理、队列、浏览器 crawler 等能力. 当前判断是: 新项目可以调研试用, 但如果团队需要最成熟的 Python 爬虫生态、插件和问题经验, `Scrapy` 仍是基准线.

### 2.5 浏览器自动化层

`Playwright` 适合 JS 渲染、登录态、前端交互复杂的页面. 它能驱动 Chromium、Firefox、WebKit, Python API 同时支持 sync 和 async. 对新爬虫项目, 如果确认必须使用真实浏览器, 它通常是优先评估项.

`Selenium` 的优势是历史生态、WebDriver 标准和企业测试基础设施. 如果项目已经有 Selenium Grid、WebDriver 经验或跨语言测试体系, 继续使用合理. 但如果只是新写一个 Python 爬虫脚本, Playwright 往往更顺手.

`DrissionPage` 在中文圈有较高可见度. 它的官方定位是基于 Python 的网页自动化工具, 既能控制浏览器, 也能收发数据包. 适合 Chromium 相关采集、自动化操作和效率脚本; 对跨浏览器、跨语言、企业级标准化能力的需求, 仍需和 Playwright/Selenium 对比.

### 2.6 内容抽取层

`trafilatura` 适合从网页中提取正文、元数据、评论等内容, 尤其适合「拿到 URL 后希望稳定抽正文」的任务.

`newspaper4k` 是 `newspaper3k` 的延续分支, 更偏新闻文章抓取、分析和处理. 如果目标是新闻标题、作者、发布日期、正文、图片等字段, 可以考虑它. 如果目标是通用字段采集, 不要把它当成 Scrapy 或 Playwright 的替代品.

### 2.7 当前 PyPI 活跃度快照

以下数据来自 2026-07-04 对 PyPI JSON 的查询, 只用于判断近期维护状态, 不代表稳定性、性能或生态排名:

| 包名 | 最新版本 | 最新版本上传时间 |
| --- | --- | --- |
| `requests` | `2.34.2` | `2026-05-14` |
| `httpx` | `0.28.1` | `2024-12-06` |
| `aiohttp` | `3.14.1` | `2026-06-07` |
| `scrapy` | `2.16.0` | `2026-05-19` |
| `playwright` | `1.61.0` | `2026-06-29` |
| `selenium` | `4.45.0` | `2026-06-16` |
| `beautifulsoup4` | `4.15.0` | `2026-06-07` |
| `lxml` | `6.1.1` | `2026-05-19` |
| `parsel` | `1.11.0` | `2026-01-29` |
| `selectolax` | `0.4.10` | `2026-05-26` |
| `crawlee` | `1.8.0` | `2026-07-02` |
| `DrissionPage` | `4.1.1.4` | `2026-05-27` |
| `trafilatura` | `2.1.0` | `2026-06-07` |
| `MechanicalSoup` | `1.4.0` | `2025-05-30` |
| `pyquery` | `2.0.1` | `2024-08-30` |
| `newspaper4k` | `0.9.5` | `2026-02-28` |
| `pyppeteer` | `2.0.0` | `2024-02-18` |
| `requests-html` | `0.10.0` | `2019-02-17` |
| `newspaper3k` | `0.2.8` | `2018-09-28` |

版本新不等于一定好, 版本旧也不等于不能用. 但对新项目而言, 维护状态会直接影响 Python 版本兼容、安全修复和未来迁移成本.

## 0x03 拓展升华展望

这篇笔记表面上在排一张库的座次表, 真正的方法论其实只有一句: 先判断任务形态, 再让库去匹配它. 往下推一层, 这其实是所有技术选型的通用问题 —— 你究竟是在选一个工具, 还是在选一条未来几年都要替它还债的依赖链.

**事实层面** … Python 爬虫生态已经稳定分层: HTTP 下载、HTML 解析、调度框架、浏览器自动化、正文抽取各占一层, 彼此不构成直接替代关系. 维护状态的差距是真实的: `requests-html` 的最新版本停在 2019 年, `pyppeteer` 官方仓库已提示长期无人维护, `newspaper3k` 已被 `newspaper4k` 接续; 而 `Scrapy`、`httpx`、`Playwright`、`Crawlee for Python` 仍在持续发版. 同时, 浏览器层在任何技术栈里都是成本最高的一层: 它引入真实的渲染进程、等待策略和资源占用.

**个人判断** … 未来爬虫工程的分化不会发生在"用哪个库"上, 而会发生在"要不要复现接口"这个判断上. 只要页面的真实数据走 XHR/Fetch, 复现接口请求就是成本最低、也最稳定的路径, 浏览器自动化应该被当成最后手段而不是默认起点. 与之相对, 真正的长期成本会转移到依赖的寿命上: 一个停更三年的解析库可能在下一个 Python 大版本上直接失效, 而迁移成本往往远高于当初省下的那点学习时间. 因此对新项目而言, "活跃度快照"值得和"功能对比表"放在同等重要的位置 —— 选型选的是未来三年的维护负担, 而不是今天跑得最快的那一次 benchmark.

## 0x04 参考来源

- [Requests documentation](https://requests.readthedocs.io/) —— 同步下载层的基准 API, 也是"简单脚本应该长什么样"的参照.
- [HTTPX documentation](https://www.python-httpx.org/) —— 核对同步/异步双 API 与 HTTP/2 支持, 判断它是否值得作为下一代默认客户端.
- [aiohttp documentation](https://docs.aiohttp.org/) —— asyncio 生态里 client/server 一体的成熟实现, 用于判断 async-first 项目该不该选它.
- [Beautiful Soup documentation](https://www.crummy.com/software/BeautifulSoup/bs4/doc/) —— 宽容解析与 parser 搭配方式, 对应脏 HTML 与快速脚本场景.
- [lxml documentation](https://lxml.de/) —— XPath/XML 能力与性能定位, 对应复杂规则和性能敏感解析.
- [Parsel documentation](https://parsel.readthedocs.io/) —— CSS、XPath、JMESPath、正则四合一 selector, 用于评估"要 Scrapy 体验但不要 Scrapy 项目"的路线.
- [selectolax documentation](https://selectolax.readthedocs.io/) —— Cython 实现的 HTML5 parser 与 Lexbor/Modest 后端, 大解析量下的 benchmark 候选.
- [Scrapy documentation](https://docs.scrapy.org/en/latest/) —— Spider、Downloader、Middleware、Pipeline 的组件边界, 工程化爬虫的成熟基准线.
- [Playwright for Python](https://playwright.dev/python/) —— Chromium/Firefox/WebKit 三引擎与 sync/async 双 API, 判断浏览器层是否必须上真实渲染的依据.
- [Selenium documentation](https://www.selenium.dev/documentation/) —— WebDriver 标准与企业测试生态, 决定已有测试体系的项目该不该迁移.
- [Crawlee for Python](https://crawlee.dev/python/docs/introduction) —— HTTP-only 与 browser-based 一体化 crawler 的官方说明, 用于评估它能否替代 Scrapy.
- [DrissionPage 官网](https://www.drissionpage.cn/) —— 控制浏览器与收发数据包结合的定位, 中文圈效率型脚本的代表方案.
- [MechanicalSoup documentation](https://mechanicalsoup.readthedocs.io/) —— 轻量表单交互库, 用于判断小规模登录场景是否需要动用完整浏览器自动化.
- [Trafilatura documentation](https://trafilatura.readthedocs.io/) —— 正文、元数据与评论抽取能力, 对应"拿到 URL 后稳定取正文"的任务.
- [Newspaper4k documentation](https://newspaper4k.readthedocs.io/) —— 新闻文章的标题、作者、日期、正文抽取, 以及它与通用爬虫框架的边界.
- [pyppeteer GitHub](https://github.com/pyppeteer/pyppeteer) —— 官方仓库的维护状态提示, 是否决它作为新项目选项的直接依据.
- [requests-html PyPI](https://pypi.org/project/requests-html/) —— 停更时间与渲染能力依赖链, 用于说明它为什么不再适合新项目.
