---
id: cs/crawler/scrapy-align-headers-with-curl
concepts: [Scrapy, 反爬]
type: check
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L48]
action: check
---
若 curl 能拿到数据而 Scrapy 拿不到, 先把 Scrapy 请求的 UA / headers 对齐 curl
> 若 curl 能拿到数据而 Scrapy 拿不到, 先把 Scrapy 请求的 UA / headers 对齐 curl
动作:
- [ ] 逐项对比 curl 与 Scrapy 请求的 UA 和 headers
