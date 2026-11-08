---
id: cs/crawler/httpx-trust-env-localhost
concepts: [httpx, requests, 代理]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/provenance.md#L30]
action: check
---
有环境变量代理时 httpx 与 requests 默认 `trust_env=True` 会把 localhost 请求也送去代理
> 沙箱有环境变量代理
> httpx 与 requests 默认 `trust_env=True` 会把 127.0.0.1 的请求也送去代理
动作:
- [ ] 有环境变量代理时测本地服务, 显式 `trust_env=False` 或设 `NO_PROXY=127.0.0.1`
