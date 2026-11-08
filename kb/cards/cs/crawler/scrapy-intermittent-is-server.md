---
id: cs/crawler/scrapy-intermittent-is-server
concepts: [Scrapy, 反爬]
type: pitfall
tier: B
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L48, raw/2026-11-08-crawler/material.md#L77]
action: check
---
结果时有时无时, 问题多在目标服务器 (过载或封禁), 换库解决不了
> 结果时有时无时, Scrapy 文档认为问题多在目标服务器 (过载或封禁), 换库解决不了
动作:
- [ ] 时有时无时先查目标服务器是否过载或封禁, 不急着换库
