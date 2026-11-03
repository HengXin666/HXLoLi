# R1 引用原文的出处与原句

## 怎么抓的

- 命令: `curl -s -A "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/124 Safari/537.36" https://www.bilibili.com/opus/1265361021143351318`
- 结果: HTTP 200, 270905 字节, `<title>Super-agent 设计探索：迈向更自主的 Agent 系统 - 哔哩哔哩`
- 正文在 HTML 里直接可见, 没有用 api.bilibili.com 接口, 也没有解 `__INITIAL_STATE__`
- 页面标注: DeepSeekHarness, 编辑于 2026年10月31日 21:34
- 读到的范围: 全文第 1 到第 9 节加文末致谢, 没有缺段
- 下面每一句都在这份 HTML 里 `grep -c` 命中 (每句命中 2 次, 正文与页面内嵌数据各一次)

## 引用的原句 (逐字, 括号里是在 REVIEW.md 的哪一段用到)

| 原文位置 | 原句 | 用在 |
|---|---|---|
| 第 1 节 | 一个人通常只能同时跟进 3 到 5 个活跃 Session | 0x09 |
| 第 1 节 (四个特征, 高效) | 同时能并行、高质量地完成多个复杂任务 | 0x05 |
| 第 2 节 | 缺少全局上下文会让各个 Session 的结果彼此割裂 | 0x00 |
| 2.1 节 | 在全局记忆问题被解决之前，这一权衡始终存在 | 0x01 |
| 3.1 节 | 将已经明确且稳定的高频判断固化为确定性代码 | 0x03 |
| 3.4 节 | 结果有价值就交付，没有价值就丢弃 | 0x04 |
| 4.2 节 | 我们验证过的一条简单有效的硬性规定是单次回复不超过 100 词 | 0x06 |
| 5.2 节 | 仅在端侧部署一个不保存会话状态的轻量 daemon，它只负责执行命令与返回结果 | 0x07 |
| 7.1 节 | 编码工作用一个只消耗时间的工具代替 | 0x05 |
| 7.1 节 | Super-agent 的 p50 与 p95 分别为 12.4 分钟与 15.7 分钟 | 0x05 |
| 7.1 节 | 在中性提示下，模型在 5 次运行中一次都没有调用 Subagent ... p95 达到 29.5 分钟 | 0x02 |
| 7.1 节 | 改为提示优先委派后，模型在 5 次运行中有 4 次进行了委派 | 0x02 |
| 7.1 节 | 去掉直通机制后，任务完成时间与完整版基本一致 | 0x05 |
| 8.4 节 | 一旦云端的 Agent Loop 遭遇服务端入侵或提示词注入而被攻破 | 0x08 |
| 8.4 节 | 让 Agent 能够在完全不接触凭据本身的情况下使用凭据 | 0x08 |

## 原文里有、我没引用但核对过的数字

- 7.1 节: 去掉直通后 Router 决定增加约 30%, 读缓存 token 从 2323 万增加到 3177 万, 增幅约 37%
- 7.2 节: 到达速率每分钟约 1 条到约 40 条, batch 1.0 到 2.8, 单次请求 2.5 秒到 3.9 秒, 分摊 6.4 秒到 1.6 秒

## 仓库侧素材 (都是经文件网关 GET 读的原文, 未改)

- `.agents/notes/implemented/feature/2026-11-01-anime-share-on-index.md`
- `.agents/notes/implemented/feature/2026-11-01-anime-share-table-view-and-preview.md`
- `.agents/notes/implemented/feature/2026-11-01-anime-tags-and-home-anime.md`
- `.agents/notes/implemented/feature/2026-11-01-home-graph-close-transition.md`
- `.agents/notes/implemented/feature/2026-11-01-navbar-slim-and-cdn-first-load.md`
- `.agents/notes/implemented/feature/2026-11-01-hxdeck-custom-select.md`
- `.hxnote-trial/REPORT.md`, `.hxnote-trial/IMPACT.md`, `.hxnote-trial/AB/VERDICT.md`, `.hxnote-trial/FIG/B3/NOTE.md`, `.hxnote-trial/FIG/B3/measure.txt`, `.hxnote-trial/T2/gates.log`
- 提交记录: 没有 exec 通道, 读的是 `.git/logs/HEAD` 末尾, 不是 `git log --oneline -20` 的输出
- 没有打开 `.hxnote-trial/REVIEW/R2` 到 `R4`
