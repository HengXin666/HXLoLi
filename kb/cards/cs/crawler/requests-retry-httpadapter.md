---
id: cs/crawler/requests-retry-httpadapter
concepts: [requests, 重试]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L11]
action: check
---
requests 默认不重试失败连接, 要重试需挂 `HTTPAdapter(max_retries=urllib3.util.Retry(...))`
> requests 默认不重试失败连接, 要重试需挂 `HTTPAdapter(max_retries=urllib3.util.Retry(...))`
动作:
- `HTTPAdapter(max_retries=urllib3.util.Retry(...))`
- [ ] 把这个 adapter 挂到所用的 `Session` 上
