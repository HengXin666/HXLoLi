# SOURCE-NOTES (R2)

## 抓取来源

- 页面: https://www.bilibili.com/opus/1265361021143351318
- 方式: `curl -A 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 Chrome/124 Safari/537.36'`, 2026-11-02 01:0x CST
- 结果: HTTP 200, 270905 bytes, `<title>Super-agent 设计探索：迈向更自主的 Agent 系统 - 哔哩哔哩`
- 正文: 从 HTML 里 `window.__INITIAL_STATE__` 的 JSON 解出; 另用网页抓取工具取得一份纯文本做交叉核对. 下面每条引文都在 `__INITIAL_STATE__` 里做过子串命中检查
- 读到多少: 全文 1 到 9 节及作者致谢. 配图只有图注文字, 没看图本身; 7.1 / 7.2 的曲线图只用了正文里给出的数字
- 署名: DeepSeek Harness 团队, 编辑于 2026-10-31 21:34; 原文自述"并非对 DeepSeek Harness 现有产品功能的介绍"

## 引文 (按 REVIEW.md 出现顺序)

| 用在 | 原文 | 节 |
|---|---|---|
| 0x00 | Router 层"充当全局的序点（Sequence Point）、协调点和记忆" | 2.1 |
| 0x00 | 每条事件新开 Session 时"缺少全局上下文会让各个 Session 的结果彼此割裂" | 2 |
| 0x01 | 链式召回: "系统为 Trigger、Thread 和 Worker Session 等对象赋予唯一 ID，并在注入时内联紧密关联对象的 ID" | 3.3 |
| 0x02 | "浪费上下文窗口，就是浪费模型的智能" | 3.3 |
| 0x02 | "我们可以将有限的上下文窗口当作 Cache 来管理" | 3.3 |
| 0x03 | 端侧"仅在端侧部署一个不保存会话状态的轻量 daemon，它只负责执行命令与返回结果" | 5.2 |
| 0x03 | 8.4 节讨论"云端的 Agent Loop 遭遇服务端入侵或提示词注入"及"凭据泄露的风险" | 8.4 |
| 0x04 | "编码工作用一个只消耗时间的工具代替，因此实验衡量的是系统的调度与协作能力，而非对单个任务的执行能力" | 7.1 |
| 0x04 | 单次实验约 300 条消息, 约 20 条工作请求, 每方案 5 次 | 7.1 |
| 0x04 | Super-agent p50 12.4 分钟, p95 15.7 分钟; 单 Session 中性提示 p95 29.5 分钟 | 7.1 |
| 0x04 | 中性提示下"模型在 5 次运行中一次都没有调用 Subagent" | 7.1 |
| (备查) | 去掉直通: Router 决定增加约 30%, 读缓存 token 2323 万 -> 3177 万, 增幅约 37% | 7.1 |
| (备查) | Router 批处理: 到达速率约 1 -> 40 条/分钟, batch 1.0 -> 2.8, 单次请求 2.5 -> 3.9 秒, 分摊 6.4 -> 1.6 秒 | 7.2 |
| 0x05 | Watchdog "还会由模型判断各个 Worker 是否有实质进展、方向是否偏离整体目标" | 6 |
| 0x06 | 维护者"也可以让它在线修正主系统的行为，例如增删提示词、调整触发规则" | 6 |
| 0x07 | "据我们观察，一个人通常只能同时跟进 3 到 5 个活跃 Session" | 1 |
| (备查) | IM 信道"单次回复不超过 100 词" | 4.2 |
| (备查) | 原型: dsh-superagent-prototype (github.com/deepseek-ai), "并非 DeepSeek 的正式产品" | 9 后注 |

## 业务侧引用的仓库文件 (都经映射通道 GET 读取, 未改动)

- `.hxnote-trial/REPORT.md`, `.hxnote-trial/IMPACT.md`, `.hxnote-trial/AB/VERDICT.md`
- `.hxnote-trial/FIG/B3/NOTE.md`, `FIGS.md`, `measure.txt`; `.hxnote-trial/T2/gates.log`
- `.agents/notes/implemented/feature/2026-11-01-navbar-slim-and-cdn-first-load.md`
- `.agents/notes/implemented/feature/2026-11-01-anime-graph-refactor.md`
- `.git/logs/HEAD` (映射通道只能读文件, 不能执行 `git log`, 所以最近提交取自 reflog 末尾)
- 18:00 后 `fs_write` 为 0、MKCOL 501、`.css` GET 命中 CDN 缓存: 出自前几轮 Hark 自己的工作记录, 不是仓库文件
