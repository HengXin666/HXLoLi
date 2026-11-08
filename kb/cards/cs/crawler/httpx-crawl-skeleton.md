---
id: cs/crawler/httpx-crawl-skeleton
concepts: [httpx, 并发控制, HTTP超时]
type: cmd
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/crawl_skeleton.py#L7-L21, raw/2026-11-08-crawler/provenance.md#L44-L46]
action: cmd
tested: 2026-11-08 Python 3.12 httpx 0.28.1 本地 aiohttp 测试服务
---
侧车骨架 (httpx + Semaphore + `Timeout(10.0, pool=None)` + 抖动延迟) 700 个全 ok
> 侧车骨架 `source/crawl_skeleton.py` (httpx + Semaphore + `Timeout(10.0, pool=None)` + 抖动延迟)
> concurrency=100, delay=0, 700 个: 700 个全 ok, 9.0s
> concurrency=8, delay=1.0, jitter=0.5, 40 个: 40 个全 ok, 10.5s
动作:
- `await crawl(urls, concurrency=8, delay=1.0, jitter=0.5)`
- [ ] 结果里逐个检查异常, `gather` 用了 `return_exceptions=True`
