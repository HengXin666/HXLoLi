# material: Python 爬虫库选型 (采集要点)

每条带来源编号与定位, 编号见 `provenance.md`.

## requests

- requests 默认不设超时, 不显式传 `timeout` 时请求可以挂几分钟甚至更久; 单值同时作用于 connect 与 read, 也可传元组分开设 [S1 §Timeouts]
- connect 超时按每个 IP 地址计, 域名有多个地址 (如 IPv4 + IPv6) 时实际总等待会成倍放大; connect / read 超时都不是挂钟时间 [S1 §Timeouts Note]
- `Session` 复用 urllib3 连接池并跨请求保存 cookie, 对同一主机多次请求会复用 TCP 连接 [S1 §Session Objects]
- `stream=True` 时只有读完 body 或调用 `close` 才把连接还给池 [S1 §Body Content Workflow]
- requests 默认不重试失败连接, 要重试需挂 `HTTPAdapter(max_retries=urllib3.util.Retry(...))` [S1 §Example: Automatic Retries]
- 默认 Transport Adapter 下 requests 不提供任何非阻塞 IO, 文档自己把 httpx 等列为异步替代 [S1 §Blocking Or Non-Blocking?]
- `session.proxies` 可能被环境变量代理覆盖; 不覆盖时 requests 读 `http_proxy` / `https_proxy` / `no_proxy` [S1 §Proxies]
- 实测默认值: `timeout` 默认 None, `DEFAULT_POOLSIZE = 10`, `DEFAULT_RETRIES = 0` (requests 2.34.2) [S13 实测 1]
- 实测: 32 线程共用一个默认 Session 打同一主机, 请求都成功, 但出现 22 次 `Connection pool is full, discarding connection`; 把 `pool_maxsize` 调到 32 后为 0 次 [S13 实测 2]

## httpx

- httpx 默认对一切网络操作强制超时: 网络无活动 5 秒抛 `TimeoutException` [S2 §Timeouts]
- httpx 区分 connect / read / write / pool 四种超时; pool 超时是从连接池拿连接的最长等待, 与 `limits` 配合 [S2 §Fine tuning]
- 连接池默认 `max_connections=100`, `max_keepalive_connections=20`, `keepalive_expiry=5` [S3]; 本地自省一致 [S13 实测 1]
- HTTP/2 默认关闭, 原因是 HTTP/1.1 实现更成熟; 需要 `pip install httpx[http2]` 并 `http2=True`, 且只有服务端也支持才会真走 HTTP/2, 用 `response.http_version` 查 [S4]
- httpx 同时提供 `Client` 与 `AsyncClient` [S4 §Enabling HTTP/2 末段]
- 实测: `AsyncClient` 默认配置下 `asyncio.gather` 700 个请求打 1 秒延迟的服务, 198 个成功, 502 个 `PoolTimeout`; 套 `Semaphore(100)` 后 700 个全成功, 7.8s [S13 实测 3]

## aiohttp

- `ClientSession` 持有 cookie 存储与连接池, 应复用同一个 session [S5 §Client Session]
- `TCPConnector(limit=...)` 默认总连接 100, `limit=0` 不限; `limit_per_host` 默认 0 即不按主机限 [S5 §Limiting connection pool size]
- aiohttp 默认不读环境变量代理, 需要 `trust_env=True` [S5 §Proxy support]
- 关闭 session 后底层连接不会立即关; 事件循环先停会有 `ResourceWarning`, 带 SSL 时文档建议等约 250ms [S5 §Graceful Shutdown]
- 实测默认值: `ClientTimeout(total=300, sock_connect=30, ...)`, `limit=100`, `limit_per_host=0` [S13 实测 1]
- 实测: 默认配置 `gather` 700 个打 1 秒延迟服务, 全部成功, 7.1s (超出 limit 的请求排队等待, 没有抛池超时) [S13 实测 4]

## Scrapy (2.20.0)

- `CONCURRENT_REQUESTS` 默认 16 [S6 §CONCURRENT_REQUESTS]
- `CONCURRENT_REQUESTS_PER_DOMAIN`: `startproject` 生成的 settings 里是 1, 全局 fallback 是 8 [S6 §CONCURRENT_REQUESTS_PER_DOMAIN]
- `DOWNLOAD_DELAY`: startproject 里是 1 (秒), fallback 0; 叠加 `DOWNLOAD_DELAY_JITTER` 默认 0.5, 即延迟在 ±50% 内随机; 响应时间小于 DOWNLOAD_DELAY 时该域实际并发为 1 [S6 §DOWNLOAD_DELAY, §DOWNLOAD_DELAY_JITTER]
- `ROBOTSTXT_OBEY`: startproject 里是 True, fallback False [S6 §ROBOTSTXT_OBEY]
- `DOWNLOAD_TIMEOUT` 默认 180 秒 [S6 §DOWNLOAD_TIMEOUT]
- 默认 `RFPDupeFilter` 按规范化后的 url + method + body 去重; 大规模抓取可换 `DiskDupeFilter` (SQLite) [S6 §DUPEFILTER_CLASS]
- `JOBDIR` 保存抓取状态以暂停与恢复 [S6 §JOBDIR]
- 默认下载中间件链含 robots.txt、重试、重定向、cookie、代理、HTTP 缓存等 [S6 §DOWNLOADER_MIDDLEWARES_BASE]
- `TWISTED_REACTOR_ENABLED=False` 时直接用 asyncio 事件循环, 默认 http/https 下载器换成 aiohttp 实现; 该模式标为实验性 [S6 §TWISTED_REACTOR_ENABLED, §DOWNLOAD_HANDLERS_BASE]
- 同进程跑多个 crawler 时并发与礼貌设置按 crawler 各算一份, 会成倍放大对目标站的负载 [S8 §Running multiple spiders in the same process]
- 动态加载的内容, 官方推荐先找到数据源 (浏览器 Network 面板 → 复现请求, 可 `Request.from_curl()`), 实在难复现且不在乎速度时才用无头浏览器 [S7 §Finding the data source, §Reproducing requests, §Using a headless browser]
- 若 curl 能拿到数据而 Scrapy 拿不到, 先把 Scrapy 请求的 UA / headers 对齐 curl; 若「有时拿得到有时拿不到」, 问题多半在目标服务器 (过载或封禁) 而不在请求本身 [S7 §Inspecting the source code, §Reproducing requests]
- 在 Scrapy 回调里直接用 playwright-python 会绕过大部分 Scrapy 组件 (中间件、去重等), 官方推荐 scrapy-playwright [S7 §Using a headless browser]
- 避免被封: 首选用 `USER_AGENT` 表明身份; 其次轮换 UA、禁用 cookie、`DOWNLOAD_DELAY` 2 秒或以上、用 Common Crawl、IP 池、对齐浏览器 TLS 版本、或交给托管服务 [S8 §Avoiding getting banned]

## Playwright 与 scrapy-playwright

- Playwright 可监听页面发出的全部请求与响应 (含 XHR / fetch), 可用 `page.expect_response("**/api/...")` 在点击后等到某个接口的响应 [S9 §Network events]
- `page.route(...)` 可拦截、修改、`abort()` 请求, 例如按 `resource_type == "image"` 丢掉图片 [S9 §Abort requests]
- `Cookie`、`Host`、`Sec-Fetch-*` 等请求头由浏览器网络栈在发送前附加, route 处理函数看不到也改不了 [S9 §Headers owned by the network stack]
- Playwright 默认无头模式用的是 chromium headless shell, 与 Chrome 的 new headless (真 Chrome) 不同, 行为会有差异 [S10 §Chromium: headless shell, §Google Chrome & Microsoft Edge warning]
- 每个浏览器安装后占几百 MB 磁盘 (示例 chromium 281M) [S10 §Managing browser binaries]
- scrapy-playwright 是一个 Scrapy 下载处理器, 只处理 `meta` 里带 `playwright` 键的请求, 其余仍走普通下载器; 需要 asyncio reactor [S11 setup §Activation]
- `PLAYWRIGHT_MAX_PAGES_PER_CONTEXT` 默认等于 `CONCURRENT_REQUESTS`; `PLAYWRIGHT_ABORT_REQUEST` 可按谓词丢弃资源请求 [S11 settings]
- Windows 上 Playwright 必须跑在另一个线程的 ProactorEventLoop, 因为 asyncio reactor 要 SelectorEventLoop [S11 setup §Windows support]

## curl_cffi

- curl_cffi 绑定 curl-impersonate, 可模仿浏览器的 TLS/JA3 与 HTTP/2 指纹, 纯 Python 客户端 (httpx / requests) 做不到; `impersonate="chrome"` 跟随库更新到最新版本, `chrome124` 这种写法则固定版本 [S12 §Usage]
- 提供与 requests 相似的 API 与 `AsyncSession`; 支持 HTTP/2、HTTP/3、websocket [S12 §Features, §Asyncio]
- 可用 `curl-cffi update` 在不升级库的情况下拉取新指纹 [S12 §Supported impersonate browsers]
- 「比 requests/httpx 快很多」为项目自述, 未复核 [S12 §Features] (官方声称)

## 反例

- curl_cffi 自己的 README 里 (赞助商段落) 写着: 对现代反爬, 只做 TLS 指纹不够, Akamai / DataDome / Kasada / Incapsula 还要另外的 token [S12 赞助商段落, 属广告性质, 仅作反例线索]. 结论: 指纹模仿只解决 TLS/HTTP2 这一层
- httpx 开了 `http2=True` 不等于走 HTTP/2, 服务端不支持就回落 HTTP/1.1; httpx 官方自己认为 HTTP/1.1 实现更稳 [S4]
- Playwright 默认无头 ≠ 真 Chrome: 默认用 headless shell, 官方警告行为会不同 [S10]. 「上了浏览器就像真人」不成立
- 「异步 = 把请求全丢进 gather」不成立: httpx 默认配置下无信号量 gather 700 个请求, 502 个 PoolTimeout [S13 实测 3]
- 在 Scrapy 里直接调 playwright-python 会绕过中间件和去重 [S7]
- 结果时有时无时, Scrapy 文档认为问题多在目标服务器 (过载或封禁), 换库解决不了 [S7 §Reproducing requests]
