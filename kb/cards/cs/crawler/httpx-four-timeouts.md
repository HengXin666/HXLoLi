---
id: cs/crawler/httpx-four-timeouts
concepts: [httpx, HTTP超时, 连接池]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L20]
action: check
---
httpx 区分 connect / read / write / pool 四种超时, pool 超时是从连接池拿连接的最长等待
> httpx 区分 connect / read / write / pool 四种超时; pool 超时是从连接池拿连接的最长等待, 与 `limits` 配合
动作:
- [ ] 设超时时单独决定 pool 超时, 并和 `limits` 一起看
