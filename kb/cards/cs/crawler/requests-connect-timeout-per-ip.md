---
id: cs/crawler/requests-connect-timeout-per-ip
concepts: [requests, HTTP超时]
type: pitfall
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L8]
action: check
---
requests 的 connect 超时按每个 IP 地址计, 域名有多个地址时实际总等待会成倍放大
> connect 超时按每个 IP 地址计, 域名有多个地址 (如 IPv4 + IPv6) 时实际总等待会成倍放大; connect / read 超时都不是挂钟时间
动作:
- [ ] 估算最坏等待时按域名的 IP 地址个数放大 connect 超时, 不把超时当挂钟时间
