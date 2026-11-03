# R3 引用来源

## 抓取方式

- 原文: https://www.bilibili.com/opus/1265361021143351318
- 标题: Super-agent 设计探索：迈向更自主的 Agent 系统 (作者署名 DeepSeekHarness, 编辑于 2026年10月31日 21:34)
- 抓取: 2026-11-02 01:01 CST, Hark 的网页抓取工具 (服务端带浏览器 UA 抓取并抽正文), 拿到正文约 40151 字符, 从第 1 节读到第 9 节结语与致谢, 正文完整. 页面里的配图只拿到图注文字, 没拿到图片本身, 所以 7.1 / 7.2 的曲线我只引了正文里写出的数字.
- 业务素材: 经文件隧道 GET 读取本仓库 `.hxnote-trial/REPORT.md`, `.hxnote-trial/IMPACT.md`, `.hxnote-trial/AB/VERDICT.md`, `.hxnote-trial/FIG/B3/NOTE.md`, `.hxnote-trial/FIG/B3/measure.txt`, `.hxnote-trial/T2/gates.log`, `.agents/notes/implemented/feature/2026-11-01-anime-page-refactor.md`, `2026-11-01-anime-share-link.md`, `2026-11-01-navbar-slim-and-cdn-first-load.md`, `.git/logs/HEAD`. 没有 exec 通道, 所以 git 历史读的是 reflog 文件而不是 `git log --oneline`. 没有读 R1 / R2 / R4 目录.

## 原文引用 (逐字)

1. 第 1 节: "单任务的上限取决于模型能力，多任务的上限取决于人的注意力。"
2. 第 2 节: "Session 应当是系统的实现细节，而不应是用户的认知负担。" 以及 "让用户手动管理 Session，在很大程度上是从底层 LLM API 延续下来的设计惯性"
3. 2.1 节: Router 层 "充当全局的序点（Sequence Point）、协调点和记忆"
4. 2.1 节: "从我们的实践来看，Router 层的任务常常没有想象中复杂，使用较轻量的模型往往就能兼顾速度与效果。"  (REVIEW.md 0x03 明确不同意此句)
5. 3.4 节: "通过消耗额外的后台算力来减少人类的等待与介入成本。结果有价值就交付，没有价值就丢弃。"
6. 3.4 节: "只有在确认问题并找到简单的修复方案后，才会把结果交付给用户，否则全程保持安静。"
7. 5.2 节: 端侧 daemon "不保存会话状态" , "只负责执行命令与返回结果"
8. 6 节: "许多异常并非进程崩溃，而是模型在执行中陷入无效循环或偏离目标"
9. 6 节: Watchdog "也可以让它在线修正主系统的行为，例如增删提示词、调整触发规则"  (REVIEW.md 0x08 提出保留意见)
10. 7.1 节: "编码工作用一个只消耗时间的工具代替，因此实验衡量的是系统的调度与协作能力，而非对单个任务的执行能力。"
11. 7.1 节: "Super-agent 的 p50 与 p95 分别为 12.4 分钟与 15.7 分钟" ; 单 Session 中性提示 "p95 达到 29.5 分钟" ; 去掉直通后 "总读缓存 token 从 2323 万增加到 3177 万，增幅约 37%"
12. 7.2 节: Trigger 速率从每分钟约 1 条到约 40 条, "平均 batch 大小从 1.0 升至 2.8", 单次请求 "从 2.5 秒增至 3.9 秒", 单个决定分摊 "从 6.4 秒降至 1.6 秒"
13. 8.3 节: "组织在决定如何配备 Super-agent 时实际上是在划分上下文共享与隔离的边界。"
14. 8.4 节: "只要端侧环境中的凭据能够进入模型的上下文，正常执行过程中同样存在凭据泄露的风险"
15. 结尾说明: 部分设计已在 dsh-superagent-prototype (github.com/deepseek-ai/dsh-superagent-prototype) 中实现, "并非 DeepSeek 的正式产品"

## 仓库数字出处

| 数字 | 出处 |
|---|---|
| doctor 16/16, 三道门禁 exit 0 | `.hxnote-trial/T2/gates.log` |
| 第 4 步 `--force`, 答题卡 A 栏留空 | `.hxnote-trial/REPORT.md` 结论段 |
| 保真盲审 T1 2 轮, T2 3 轮 (4 新增 2 歪曲 / 3 新增 2 歪曲) | `.hxnote-trial/REPORT.md` 对照表 |
| A1 1 张 drawio, A2 3 张生图 + 1 张数据图, 中文字数 281 / 611 | `.hxnote-trial/AB/VERDICT.md` |
| 去图后 18 个点剩 3 个, 10 个点剩 2 个, `E [section-visual] 2 处` | `.hxnote-trial/FIG/B3/NOTE.md` |
| 44 篇, 旧门禁 9/44, 新门禁 0/44, section-visual 39 篇 | `.hxnote-trial/IMPACT.md` |
| `scripts/cli/sources/` 改为 `scripts/lib/sources_core.py`; 11 个平铺文件; `.trial` 0 字节 | `.hxnote-trial/REPORT.md` 已知问题 |
| pick 148 部 402 字符, 整条链接 456 字符 | `.agents/notes/implemented/feature/2026-11-01-anime-share-link.md` |
| CDN 两张延迟表, probeEngine + useSyncExternalStore | `.agents/notes/implemented/feature/2026-11-01-navbar-slim-and-cdn-first-load.md` |
| 三套筛选状态收进 URL | `.agents/notes/implemented/feature/2026-11-01-anime-page-refactor.md` |
