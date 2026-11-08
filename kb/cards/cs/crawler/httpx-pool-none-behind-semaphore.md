---
id: cs/crawler/httpx-pool-none-behind-semaphore
concepts: [httpx, HTTP超时, 并发控制]
type: decide
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/crawl_skeleton.py#L8-L11]
action: check
---
pool=None 时在信号量后面排队的请求不会因为等连接而超时
> pool=None: 在信号量后面排队的请求不会因为等连接而超时
动作:
- `timeout = httpx.Timeout(10.0, pool=None)`
- `limits = httpx.Limits(max_connections=concurrency)`
