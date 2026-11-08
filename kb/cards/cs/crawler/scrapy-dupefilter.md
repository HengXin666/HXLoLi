---
id: cs/crawler/scrapy-dupefilter
concepts: [Scrapy]
type: decide
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L42]
action: check
---
默认 `RFPDupeFilter` 按规范化后的 url + method + body 去重, 大规模抓取可换 `DiskDupeFilter`
> 默认 `RFPDupeFilter` 按规范化后的 url + method + body 去重; 大规模抓取可换 `DiskDupeFilter` (SQLite)
动作:
- [ ] 大规模抓取时把 `DUPEFILTER_CLASS` 换成 `DiskDupeFilter`
