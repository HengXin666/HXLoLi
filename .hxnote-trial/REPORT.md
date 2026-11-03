# hx-note 改造实跑报告 (T1 基线 vs T2 改造后)

日期: 2026-11-01. 执行环境: 仓库镜像到 Hark 沙箱, 在沙箱里真跑 `hx_flow.py` 九步与三道门禁, 产物与 skill 改动 PUT 回仓库

## 结论

改造后的 skill 产出的 T2 更短、图文比更高、来源更硬. 正文纯文字从 15 行降到 6 行, 中文字数从 649 降到 423, 每段都有视觉; 盲审机器味分数从 28 降到 15. 代价是保真盲审多跑一轮 (T1 两轮过, T2 三轮过): 文字压短以后, 每句话更容易"讲过头"

两篇都能拿给人看, 但都不是定稿: 第 4 步人类逐题对齐两篇都没有真的发生 (按 2026-11-01 21:48 的"直接实跑"指示用 `--force` 推进, 答题卡 A 栏留空), drawio 图在沙箱里无法渲染中文, 需要人类在站点上看一眼

## 主题

| 篇 | 主题 | 用的 skill | 是否与 ai-docs 现存笔记重复 |
|---|---|---|---|
| T1 | HTTP 缓存四个指令 (no-store / no-cache / max-age=0 / must-revalidate) 的真实语义 | 改造前 | 否, 只看过 ai-docs 目录名, 没读任何笔记正文 |
| T2 | SQLite WAL 的检查点饥饿 | 改造后 | 否, 同上 |

## T1 / T2 对照表

| 指标 | T1 (改造前) | T2 (改造后) | 怎么数的 |
|---|---|---|---|
| 正文行数 (frontmatter 后非空行) | 61 | 36 | 去掉 frontmatter 数非空行 |
| 正文纯文字行 | 15 | 6 | `voice_checks._tally`, 不含代码/表/图/标题 |
| 中文字数 (不含代码块) | 649 | 423 | 正则数 CJK 字符 |
| 正文段数 (`## 0xNN`) | 5 | 4 | |
| 视觉总数 (图 + 代码块 + 表) | 7 | 4 | |
| 纯文字行 / 视觉 | 2.1 | 1.5 | |
| 没有任何视觉的段 | 1 (0x02) | 0 | 新 lint `section-visual` |
| 图张数 | 1 | 1 | `![` 计数 |
| 表格 | 0 (旧规则禁表) | 1 (3 行数据) | |
| 侧车张数 | 2 (cache-offline.drawio.svg, cache_decide.py) | 2 (wal-readers.drawio.svg, wal_starve.py) | 同目录且被正文引用 |
| 带分组名的代码块 | 6 | 2 | |
| 全稿来源数 | 4 | 5 | provenance 行数 |
| T1 级来源数 | 3 (RFC 9111 / 5861 / 9110; 按新定义回标) | 4 (sqlite.org 三页 + 本次实测) | T2 另有 1 个 T2 级 (Fly.io) |
| 反例条目 | 0 (只在边界里提) | 3 | material.md `## 反例` |
| 实测数字 | 脚本判定表 (规则推演, 非实测系统行为) | 真实 SQLite 3.46.1 运行: 4031 / 20362 / 40737 KB, (1, 5061, 52) | |
| 发散产出 (带判据) | 5 | 4 | `divergence_report` |
| AI 味盲审机器味分数 | 28 (PASS) | 15 (PASS) | 新鲜 sub-agent, 只给 index.md |
| 保真盲审轮数 | 2 (第 1 轮 FAIL: 1 新增 1 歪曲) | 3 (第 1 轮 4 新增 2 歪曲, 第 2 轮 3 新增 2 歪曲) | |
| doctor | 16/16 PASS | 16/16 PASS | |
| 用改造后的门禁复查 | lint FAIL (`section-visual` 1 处); 来源门槛 FAIL (无编号, 无反例) | 全 PASS | 见 T1/gates.log 末尾 |

## 三道门禁 (两篇各自的 gates.log 有原始输出)

| 门禁 | T1 (改造前 skill) | T2 (改造后 skill) |
|---|---|---|
| `validate_skill.py --strict` | exit 0 | exit 0 |
| `prose_rules.py --check` | exit 0 | exit 0 |
| `check_layout.py` | exit 0 | exit 0 |

## 这篇能不能拿给人看

- **T1: 能看, 偏长.** 依据: 盲审 28 分 PASS, 保真 PASS, doctor 全绿. 弱点: 0x02 整段只有文字; 判定脚本是对 RFC 规则的推演, 没有在真实浏览器或 CDN 上验证; 图的连线在同一列汇成一条竖线, 读者分不清哪条指令走到哪 (drawio 脚本的走线如此; T2 靠调节点顺序避开, 脚本本身没改)
- **T2: 能看, 更接近作者口味.** 依据: 盲审 15 分 PASS (评审原话: "(1, 5061, 52) 这种零碎的真实返回值编不出来"), 保真 PASS, doctor 全绿, 每段一个视觉, 全部数字可用同目录 `wal_starve.py` 复跑. 弱点: 保真漏讲 14 条 (单写者、网络文件系统不可用等), 由 `.hx-mitemite.md` 记为接受; 引入是执行者实测的真事, 不是作者本人的遭遇

## skill 改了什么 (P1 到 P4, 按试的口径, 未当定稿)

- P1 真人文样与写作顺序: 新增 `shared/voice/impl/exemplars.md` (四段作者 blog 真文与"好在哪"); `templates/interest-outlook.md` 从 14KB 缩到 3KB, 理由与反例移到 `shared/voice/impl/outlook-rationale.md`; 步骤 6 顺序改为 分镜, 仿写引入, 逐段先放视觉再写字
- P2 工具名换能力描述加可数预算: `steps/2-collect/impl/article.md` 与 `research.md` 去掉 argo_fetch / wide_research 等工具名, 改成探测表; 模板槽位给出段数 3 到 6, 每段至少 1 个视觉, 全文纯文字不超过 30 行
- P3 图先行: 新增 `assets/storyboard-template.md`, 步骤 6 的产物加 `source/storyboard.md`, `done derive` 检查它; lint 新增 `section-visual` (E), `text-ratio` (W, 4:1), `prose-budget` (W, 30 行); `table` 规则改为超过 5 行数据才报 E; 生图后端不可用时降级为 drawio, 表格, 代码块, 不再退成分点
- P4 来源分级: provenance 加 T1 到 T3 等级列与 S 编号, material 每条要点引用编号; `scripts/lib/sources_core.py` 实现广度门槛 (至少 3 源, 至少 1 个 T1, 每条要点有 T1 或两个 T2, 至少 1 条反例), `done collect` 对 article / research 自动查, `hx_flow.py sources` 单独查

## 已知问题与偏差

- 远端写接口不能建目录, 所以 P4 原计划的新域目录 `scripts/cli/sources/` 改为 `scripts/lib/sources_core.py` 加 `hx_flow.py sources` 子命令
- 两篇的第 4 步都是 `--force` 过的, 没有人类逐题作答
- 两个新 tag (HTTP / SQLite) 只加在沙箱的 `.hx-tags.toml` 里, 没有回写仓库; 笔记也只落在沙箱的 ai-docs, 没有回写
- 盲审 learn 进的语料 (2 条坏、1 条好) 只在沙箱的 `.hx-voice.toml` 里, 没有回写
- 正式产物的唯一位置是 `HXLoLi/.hxnote-trial/` (T1/, T2/, REPORT.md)
- `HXLoLi/.agents/research/hxnote-trial-*` 共 11 个平铺文件是我选错位置的临时落点, 内容与 `.hxnote-trial/` 重复, 可以直接删除
- `HXLoLi/.trial` 是我探测建目录方法时误建的 0 字节文件, 应删除
- 过程产物 (暂存区 staging, 三轮保真盲审原文, 发散记录, 分镜) 没有回写进 HXLoLi. 我在 2026-11-01 22:55 通过网关列目录看到 `/home/hx/Loli/code/HXLoLis/.research_scratch/` 下有 `hx-note-trial.tar.gz` (27890 字节) 和 5 个 `hx-note-trial-*` 平铺副本, 这是外层 HXLoLis 仓库的目录, 不在 HXLoLi 里; 你那边查不到时以你盘上为准. 同一份过程产物在我的沙箱里写这份报告时仍在
- 这些 `.research_scratch/hx-note-trial*` 文件同样是临时副本, 可以删
- `PLAN.md` 原先有 5 处相对路径让 `doctor` 的"技能文档路径可达"一项变红, 已改成全路径

提交前需清掉 skill 目录下的 `.hark-bak` (仓库根的 `.hark-bak` 里还有 `.trial` 的备份, 同样该清), 它们是写接口覆盖文件时留的备份, 会混进 diff
