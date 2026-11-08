---
id: cs/crawler/scrapy-default-middlewares
concepts: [Scrapy, 重试, 代理]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L44]
action: check
---
Scrapy 默认下载中间件链含 robots.txt、重试、重定向、cookie、代理、HTTP 缓存等
> 默认下载中间件链含 robots.txt、重试、重定向、cookie、代理、HTTP 缓存等
动作:
- [ ] 自己写重试或代理逻辑前, 先查 `DOWNLOADER_MIDDLEWARES_BASE` 里是否已有
