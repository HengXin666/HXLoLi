# provenance: crawler (Python 爬虫库选型调研)

收敛后的问题: 在静态页 / 动态页 / 反爬 / 并发四个场景下, requests、httpx、aiohttp、Scrapy、Playwright (及 curl_cffi、scrapy-playwright) 各自默认替使用者做了什么、没做什么, 以及据此怎么选. 时间点: 2026-11. 版本: requests 2.34.2, httpx 0.28.1, aiohttp 3.14.5 (本地 pip 安装实测), Scrapy 文档 2.20.0, Playwright Python 文档 2026 版, curl_cffi README (0.17.0 一节).

## 来源表

| 编号 | 来源 | 等级 | 获取方式 | 获取时间 | 实际读的 URL |
|---|---|---|---|---|---|
| S1 | Requests 官方文档 Advanced Usage (2.34.2) | T1 | web_fetch 正文 | 2026-11-08 | https://requests.readthedocs.io/en/latest/user/advanced/ |
| S2 | HTTPX 官方文档 Timeouts | T1 | web_fetch 正文 | 2026-11-08 | https://www.python-httpx.org/advanced/timeouts/ |
| S3 | HTTPX 官方文档 Resource Limits | T1 | web_fetch 正文 | 2026-11-08 | https://www.python-httpx.org/advanced/resource-limits/ |
| S4 | HTTPX 官方文档 HTTP/2 Support | T1 | web_fetch 正文 | 2026-11-08 | https://www.python-httpx.org/http2/ |
| S5 | aiohttp 官方文档 Advanced Client Usage (3.14.5) | T1 | web_fetch 正文 | 2026-11-08 | https://docs.aiohttp.org/en/stable/client_advanced.html |
| S6 | Scrapy 官方文档 Settings (2.20.0) | T1 | web_fetch 正文 | 2026-11-08 | https://docs.scrapy.org/en/latest/topics/settings.html |
| S7 | Scrapy 官方文档 Selecting dynamically-loaded content | T1 | web_fetch 正文 | 2026-11-08 | https://docs.scrapy.org/en/latest/topics/dynamic-content.html |
| S8 | Scrapy 官方文档 Common Practices (Avoiding getting banned 等) | T1 | web_fetch 正文 | 2026-11-08 | https://docs.scrapy.org/en/latest/topics/practices.html |
| S9 | Playwright for Python 官方文档 Network | T1 | web_fetch 正文 | 2026-11-08 | https://playwright.dev/python/docs/network |
| S10 | Playwright for Python 官方文档 Browsers | T1 | web_fetch 正文 | 2026-11-08 | https://playwright.dev/python/docs/browsers |
| S11 | scrapy-playwright 官方文档 (Initial setup + Settings) | T1 | web_fetch 正文 | 2026-11-08 | https://scrapy-playwright.readthedocs.io/en/latest/setup.html ; https://scrapy-playwright.readthedocs.io/en/latest/reference/settings.html |
| S12 | curl_cffi README (项目官方仓库; 性能对比与赞助商段落按「官方声称 / 广告」处理) | T1 | web_fetch 正文 | 2026-11-08 | https://github.com/lexiforest/curl_cffi |
| S13 | 本地实测: 库默认值自省 + 并发压测, 可复现 (见下方「实测记录」) | T1 | 本地 venv + 本地 aiohttp 测试服务 | 2026-11-08 | 无 URL; 脚本与结果记录在本文件末尾 |
| S14 | HX-Recall 与其依赖 bilibili-api (HX-Workflows) 源码 | T1 | curl raw.githubusercontent.com | 2026-11-08 | https://github.com/HengXin666/HX-Recall ; https://raw.githubusercontent.com/HengXin666/HX-Recall/main/hx_recall/rate_limiter.py ; https://raw.githubusercontent.com/HengXin666/HX-Recall/main/hx_recall/bilibili/fetcher.py ; https://raw.githubusercontent.com/HengXin666/HX-Workflows/main/bilibili-api/bilibili_api/_core.py |

S14 只用于步骤 3 算子三 (迁移), 不作为选型结论的依据.

## 实测记录 (S13)

环境: Python 3.12, 本地 venv; 测试服务是 aiohttp web 单路由, 每个请求 `asyncio.sleep(D)` 后返回 `ok`, backlog=4096, 监听 127.0.0.1.

注意: 沙箱有环境变量代理. httpx 与 requests 默认 `trust_env=True` 会把 127.0.0.1 的请求也送去代理 (第一轮实测因此全是 404, 已作废). 下列有效结果都显式关掉了 env 代理 (`trust_env=False` 或 `NO_PROXY=127.0.0.1`); aiohttp 默认不读 env 代理 (与 S5 一致), 不受影响.

1. 默认值自省 (`inspect` / 模块常量):
   - `requests.Session.request` 的 `timeout` 默认 `None`; `requests.adapters.DEFAULT_POOLSIZE = 10`, `DEFAULT_RETRIES = 0`
   - `httpx._config.DEFAULT_TIMEOUT_CONFIG = Timeout(timeout=5.0)`; `DEFAULT_LIMITS = Limits(max_connections=100, max_keepalive_connections=20, keepalive_expiry=5.0)`
   - `aiohttp.TCPConnector` 默认 `limit=100`, `limit_per_host=0`; `aiohttp.client.DEFAULT_TIMEOUT = ClientTimeout(total=300, connect=None, sock_read=None, sock_connect=30, ceil_threshold=5)`
2. requests, 服务延迟 0.2s, 32 线程共用一个 Session, 320 个请求:
   - 默认 pool_maxsize(10): 320 个全 200, 日志里 `Connection pool is full, discarding connection` 22 次, 2.1s
   - `HTTPAdapter(pool_maxsize=32)`: 320 个全 200, 该警告 0 次, 2.1s
3. httpx 0.28.1 AsyncClient 默认配置, 服务延迟 1.0s:
   - `asyncio.gather` 100 个: 100 个全 200, 1.1s
   - `asyncio.gather` 700 个 (无信号量): 198 个 200, 502 个 `PoolTimeout`, 13.0s
   - 同样 700 个, 外面套 `asyncio.Semaphore(100)`: 700 个全 200, 7.8s
4. aiohttp 3.14.5 ClientSession 默认配置, 服务延迟 1.0s, `asyncio.gather` 700 个 (无信号量): 700 个全 200, 7.1s
5. 侧车骨架 `source/crawl_skeleton.py` (httpx + Semaphore + `Timeout(10.0, pool=None)` + 抖动延迟), 服务延迟 1.0s:
   - concurrency=100, delay=0, 700 个: 700 个全 ok, 9.0s
   - concurrency=8, delay=1.0, jitter=0.5, 40 个: 40 个全 ok, 10.5s

## 已知缺口

- Scrapy、Playwright、curl_cffi 没有在本地装起来实跑, 它们的结论全部来自官方文档 (T1 文本), 未实测
- 没有对任何真实带反爬的站点做测试; 反爬一节只有文档陈述与逻辑推导, 没有「某指纹被某厂商拦截」的一手数据
- 未找到 aiohttp 官方关于 HTTP/2 的陈述; 「aiohttp 不支持 HTTP/2」只出现在 curl_cffi README 的对比表里 (第三方项目的陈述), 正文不采用
- curl_cffi「比 requests/httpx 快很多」是项目自述 + 其仓库内 benchmark, 未复核, 只能写「官方声称」
- 实测 3 中 700 并发时 PoolTimeout 的精确个数受机器调度影响, 只当量级参考; 可复现的是「无信号量时出现大量 PoolTimeout、有信号量时全成功」这一对照
