---
id: cs/crawler/scrapy-avoid-getting-banned
concepts: [Scrapy, 反爬]
type: decide
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L50]
action: check
---
避免被封首选用 `USER_AGENT` 表明身份, 其次轮换 UA、禁用 cookie、`DOWNLOAD_DELAY` 2 秒或以上等
> 避免被封: 首选用 `USER_AGENT` 表明身份; 其次轮换 UA、禁用 cookie、`DOWNLOAD_DELAY` 2 秒或以上、用 Common Crawl、IP 池、对齐浏览器 TLS 版本、或交给托管服务
动作:
- [ ] 先设 `USER_AGENT` 表明身份
- [ ] 仍被封再依次试轮换 UA、禁用 cookie、`DOWNLOAD_DELAY` 2 秒或以上、IP 池
