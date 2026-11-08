---
id: cs/crawler/playwright-route-abort-images
concepts: [Playwright]
type: check
tier: A
as_of: 2026-11-08
src: [raw/2026-11-08-crawler/material.md#L55]
action: check
---
`page.route(...)` 可拦截、修改、`abort()` 请求, 例如按 `resource_type == "image"` 丢掉图片
> `page.route(...)` 可拦截、修改、`abort()` 请求, 例如按 `resource_type == "image"` 丢掉图片
动作:
- [ ] 不需要图片时用 `page.route` 按 `resource_type` 丢掉
