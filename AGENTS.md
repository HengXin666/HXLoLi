# Agent Notes — 决策记录

非平凡改动的定义: 改变行为、架构、跨文件的契约、流程或工具、测试策略, 或任何落盘 / 线上 / 配置格式。
每一次非平凡改动都在**同一次提交里**新增或更新一篇 Agent Note; 纯机械的局部编辑豁免。

1. 改一个声明之前, 先找它旁边引用的 note —— `.agents/notes/<lifecycle>/<class>/<file>.md` 路径,
   通常写在注释或 JSDoc 里。先读它: 它记着已经否决过什么、为什么。
2. 优先更新已经拥有那条决策的 note。过时的事实**就地重写**, 不要追加变更历史;
   绝不把一篇 note 改写成另一条决策 —— 要取代它, 并双向互链。
3. 新方向从 `.agents/notes/proposed/{class}/` 开始; 落地时在同一提交里移到
   `implemented/{class}/`, 用现在时陈述, 并从它约束的代码里引用它 (见第 1 条)。
   只引用一次, 引在"读者否则就会删掉这个约束"的那个位置。
4. 每一篇 active note 都带 `## Alternatives considered`, 其中必须有"什么都不做 / 复用现有"
   这一项, 且每个被否掉的选项都要先给出它最强的理由, 再否决。
5. note 被完全取代时用 `notes:archive` 归档; rejected 提案不再能拦住一个
   有人可能重犯的错误时, 直接删除。

推送前跑 `npm run verify-notes`。受保护的源码改动若同一次改动里没有 note, 门禁会失败;
要刻意豁免, 写 `.agents/notes/NOTE-EXEMPT.md`, 内容为 `note-exempt: <为什么这次不需要 note>`。
规则细节见 `.agents/notes/AGENTS.md`。
