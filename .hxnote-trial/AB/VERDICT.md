# A/B 判断: hx-note 是不是把配图能力限制住了

日期: 2026-11-02 00:10 CST. 同一主题 (fork 之后 CPython 子进程只读也会复制内存), 同一执行者 (Hark 沙箱), 同一份实测数字 (`measure.txt`, Linux 6.12 aarch64, Python 3.12.15, 连跑两次相同). 唯一变量是 skill 版本: A1 用盘上原版, A2 套 `A2/skill-A2.patch` (只动 `media.md`、`SKILL.md` description 一句、模板示例图名).

## 结论

**用户的判断成立: 是 skill 把配图压成了框线图, 不是执行者做不到.** 同一个执行者、同一个环境里生图后端一直可用, 原版 skill 下交出 1 张 drawio, 改了配图规则后交出 3 张生图加 1 张数据图. 执行者的能力没变, 变的只有规则.

但"原版 media.md 那一句'能用 .drawio.svg 的场合优先用它'"**不是唯一原因, 是三处同向信号里最直接的一处**. 真正决定性的是 media.md 的分类表: 它按"画什么"分流, 把"架构 / 流程 / 分层 / 数据流"划给 drawio, 而技术笔记几乎每一段都能被说成流程或数据流, 于是生图只剩"封面 / 类比"这一小格. 那句"优先 drawio"再把模糊地带也推给 drawio.

## 两篇对照

| 指标 | A1 (原版) | A2 (改配图规则) |
|---|---|---|
| 生图 | 0 | 3 (cow-hero / refcount-touch / gc-freeze) |
| 数据图 | 0 (数字进了表) | 1 (cow-bars.png, matplotlib 读 measure.txt) |
| drawio | 1 (cow-write.drawio.svg) | 0 |
| 表格 / 代码块 | 1 / 1 | 1 / 1 |
| 视觉总数 (`_tally` 口径) | 3 | 6 |
| 纯文字行 (`_tally`) | 5 | 8 |
| 中文字数 (不含代码块) | 281 | 611 |
| `hx_voice lint --profile article` | 0 E, 1 W (long-sentence) | 0 E, 1 W (long-sentence) |
| 出图脚本 | spec.json (drawio) | make_figures.py (提示词 + 数据图代码) |

A1 的 drawio 在沙箱里无法栅格化预览 (`magick: delegate failed 'rsvg-convert'`), 需要在站点上看一眼; 它和 T1/T2 的 drawio 一样, 只能是 3 列方框加箭头.

## 挡住生图的约束, 逐条

| 约束 | 在哪 | 挡没挡 | 依据 |
|---|---|---|---|
| 分类表把"架构 / 流程 / 数据流"划给 drawio | 原版 `media.md` 第一张表 | **挡, 主因** | 本主题三段 (读也会写 / 引用计数 / gc 与 freeze) 都能归进"流程", 按表就全走 drawio |
| "能用 .drawio.svg 的场合优先用它", 理由是位图改不动 | 原版 `media.md` 表下 | **挡, 直接原因** | 把模糊地带推给 drawio; 理由 (人类可改) 对概念图不成立, 概念图本来就很少改 |
| description 写死"配图走 drawio.svg" | `SKILL.md` description | **挡** | skill 触发时最先读到的一句, 定下整篇的预期 |
| 模板骨架的示例图是 `xxx.drawio.svg` | `templates/interest-outlook.md` 第 54 行 | **挡 (弱)** | 照模板写的执行者会照抄后缀 |
| 生图后端排第 1 但要"先确认它真的在当前工具列表里", 否则降级 drawio | 原版 `media.md` 生图后端一节 | **看环境** | 在没有生图工具的执行环境 (多数 CLI agent) 会合理降级; 本次 A/B 后端可用, 所以不是本次差异来源 |
| 没有"实测数字 -> 数据图"这一路 | 原版 `media.md` 第一张表只有"实测结果 -> 截图" | **挡** | A1 的六个数字只能进表或代码块, A2 才出现 cow-bars.png |
| drawio 脚本的硬限 (最多 3 列 × 4 行, 边标签 6 字内, 节点 3 行) | `media.md` drawio 一节 | **挡表现力, 不挡选择** | 一旦选了 drawio, 图只能是方框加短箭头, 画不出"读一下就复制一页"的直觉 |
| `section-visual` 只认 图 / 代码块 / 表格 | `voice_rules.py` 第 110 行, `_tally` | **不挡** | `_tally` 对任何 `![` 都记一个视觉, png 生图照算; A2 四张 png 都被数进去, lint 0 E |
| `prose-budget` (30 行, W) / `text-ratio` (4:1, W) | `voice_rules.py` 第 113, 116 行 | **不挡, 方向相反** | 两篇纯文字都远低于 30 行 (5 / 8). 生图反而让文字变多: media.md 要求"每张图后紧跟一句话接进论点", A2 多了 3 张图就多了 3 处接话, 中文字数 281 -> 611 |

## 改法 (已写在 `A2/skill-A2.patch`, 本轮不动仓库)

1. 分类表改为按"以后要不要人手改"分流: 概念 / 主视觉走生图, 实测数字走 matplotlib 数据图, 只有要人手维护的结构图走 drawio
2. 删掉"能用 drawio 优先", 改为"生图后端可用时每篇至少一张生图当主视觉"
3. description 与模板示例图名同步改
4. 出图脚本 (提示词 + 数据图代码) 作为侧车必须交付

## 出图手段 (本沙箱实测, 原文见 `A2/IMAGE-BACKENDS.md`)

| 手段 | 结果 |
|---|---|
| 生图后端 (会话内 generate_image) | 通, 3 张 1376x768, 缩到 1200 宽、192 色后 177 到 241 KB |
| matplotlib | 通, 但沙箱原本没装 (`ModuleNotFoundError: No module named 'matplotlib'`), pip 装 3.11.2; 无中文字体, 下载 NotoSansSC-Regular.otf 传 `--font` 后正常 |
| graphviz | 不通: `apt-get install graphviz` -> `Could not open lock file /var/lib/dpkg/lock-frontend - open (13: Permission denied)`; `sudo -n apt-get install graphviz` -> `Package 'graphviz' has no installation candidate` |
| 手写 SVG | 能写, 但没有 rsvg-convert / inkscape / cairosvg, 无法本地栅格化自查 |
| drawio | 能生成 (`hx_drawio.py`), 不能本地预览 (同上, 缺 rsvg-convert) |

## 这个判断的边界

- 只有一个主题、一次运行, 没做盲审; 执行者知道自己在做 A/B, 可能下意识把 A2 做得更用心
- A2 更长 (611 vs 281 字), "图文并茂"的代价是篇幅; 是否更合作者口味要人看
- 生图里不放文字是 A2 的规定 (中文会被画错), 所以图只负责直觉, 机制细节仍靠正文和代码
- 在没有生图工具的执行环境里, 改过的规则也会降级回 drawio / 数据图; 那时差距会缩小到"有没有数据图"
