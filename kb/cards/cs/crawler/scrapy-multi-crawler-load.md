---
id: cs/crawler/scrapy-multi-crawler-load
concepts: [Scrapy, 并发控制]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L46]
action: check
---
同进程跑多个 crawler 时并发与礼貌设置按 crawler 各算一份, 会成倍放大对目标站的负载
> 同进程跑多个 crawler 时并发与礼貌设置按 crawler 各算一份, 会成倍放大对目标站的负载
动作:
- [ ] 同进程多个 crawler 打同一站时, 按 crawler 个数下调各自并发
