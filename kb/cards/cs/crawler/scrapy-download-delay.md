---
id: cs/crawler/scrapy-download-delay
concepts: [Scrapy, 并发控制]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L39]
action: check
---
`DOWNLOAD_DELAY` 在 startproject 里是 1 秒, fallback 0, 叠加 `DOWNLOAD_DELAY_JITTER` 默认 0.5
> `DOWNLOAD_DELAY`: startproject 里是 1 (秒), fallback 0; 叠加 `DOWNLOAD_DELAY_JITTER` 默认 0.5, 即延迟在 ±50% 内随机
动作:
- [ ] 查项目 settings 里实际的 `DOWNLOAD_DELAY`, 并记住它会在 ±50% 内随机
