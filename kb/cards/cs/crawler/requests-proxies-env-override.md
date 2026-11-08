---
id: cs/crawler/requests-proxies-env-override
concepts: [requests, 代理]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L13]
action: check
---
`session.proxies` 可能被环境变量代理覆盖, 不覆盖时 requests 读 `http_proxy` / `https_proxy` / `no_proxy`
> `session.proxies` 可能被环境变量代理覆盖; 不覆盖时 requests 读 `http_proxy` / `https_proxy` / `no_proxy`
动作:
- [ ] 机器上有环境变量代理时, 先确认 `session.proxies` 实际有没有生效
