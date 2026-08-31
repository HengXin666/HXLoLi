---
title: "RTK项目使用说明与Codex本地心得"
created_at: "2026-07-05"
model: "GPT-5 Codex"
skill: ["hx-make-ai-docs"]
authors: "Heng_Xin"
tags: ["AI工具", "RTK", "Codex", "CLI"]
---

# RTK项目使用说明与Codex本地心得

> [!NOTE]
> 本文由 AI 辅助沉淀, 需要用户 review 后再提交.

## 0x00 背景

本文沉淀的是 [`rtk-ai/rtk`](https://github.com/rtk-ai/rtk) 项目, 即 **RTK / Rust Token Killer**. 它不是 Redux Toolkit, 也不是别的同名 Rust Type Kit. 这个项目的核心定位是: 在 AI 编程助手执行 shell 命令时, 先把命令输出过滤、压缩、分组、截断或去重, 再送入 LLM 上下文, 从而减少 token 消耗.

RTK 的典型场景是 Claude Code、Cursor、Gemini CLI、OpenCode、Pi、Hermes 等工具的命令调用. 对支持 hook/plugin 的工具, 它可以在命令执行前透明改写, 例如把 `git status` 改成 `rtk git status`. 对 Codex CLI, 当前项目给出的集成方式是 prompt-level guidance: 通过 `AGENTS.md` 引用 `RTK.md`, 告诉 Codex 优先显式执行 `rtk <cmd>`.

从 Codex 角度看, RTK 的价值不是“把所有输出都变短”, 而是把常见高噪声命令变成更适合模型消费的形状. 这能降低上下文污染, 但也会带来一个新的工程边界: 压缩输出不是原始事实全集. 当需要精确日志、完整 diff、机器可读 JSON 或安全审计细节时, 应该保留回退到原始输出的能力.

## 0x01 核心结论

RTK 最适合被当成“AI 终端输出压缩层”使用. 对 Codex 来说, 推荐采用显式命令习惯, 而不是假设它能像 Claude Code hook 一样自动拦截所有 shell 调用.

### 1.1 安装与确认

推荐安装方式按平台选择:

```bash
# macOS / Linux, Homebrew
brew install rtk-ai/tap/rtk

# Linux / macOS quick install
curl -fsSL https://raw.githubusercontent.com/rtk-ai/rtk/master/install.sh | sh

# Cargo, 注意不要 cargo install rtk
cargo install --git https://github.com/rtk-ai/rtk rtk
```

安装后先确认拿到的是 Rust Token Killer:

```bash
rtk --version
rtk gain
```

`cargo install rtk` 有同名包风险, 可能装到另一个 Rust Type Kit. 只要 `rtk gain` 不能显示 token savings dashboard, 就应先排查是否装错包或 PATH 指向错误.

本次阅读的源码 commit 中, `Cargo.toml` 标注版本是 `0.42.4`; README 里的 `rtk --version` 示例仍写着旧版本号. 实操时不要硬记 README 示例版本, 以 release、`Cargo.toml` 或本地二进制输出为准.

### 1.2 为 Codex 初始化

Codex CLI 不是透明 hook 集成, 而是规则文件集成:

```bash
# 项目级: 当前项目写 AGENTS.md + RTK.md
rtk init --codex

# 全局级: 写到 $CODEX_HOME, 否则 ~/.codex
rtk init --global --codex

# 查看 Codex 侧配置状态
rtk init --show --codex
```

Codex 初始化会写入 `RTK.md`, 并在 `AGENTS.md` 里加入对它的引用. 其中 `RTK.md` 的核心规则很直接:

```bash
rtk git status
rtk cargo test
rtk npm run build
rtk pytest -q
```

也就是说, 在 Codex 中不要期待每个 Bash 命令都被自动重写. 稳定做法是让 Codex 自己显式敲 `rtk <cmd>`.

### 1.3 常用命令速记

| 场景 | 命令 | 作用 |
| --- | --- | --- |
| 看 Git 状态 | `rtk git status` | 按状态聚合, 比原始 status 短很多 |
| 看提交 | `rtk git log -n 10` | 一行一个 commit |
| 看 diff | `rtk git diff` | 降低上下文和头部噪声 |
| 读文件 | `rtk read file.rs` | 智能读取, 可配过滤级别 |
| 激进读代码结构 | `rtk read file.rs -l aggressive` | 主要保留签名, 去掉函数体 |
| 找文件 | `rtk find "*.rs" .` | 紧凑树形结果 |
| 搜索 | `rtk grep "pattern" .` | 按文件分组并截断 |
| 跑测试 | `rtk cargo test` / `rtk pytest` / `rtk vitest` | 重点保留失败信息 |
| 只看错误 | `rtk err <cmd>` | 通用错误过滤 |
| 通用测试包装 | `rtk test <cmd>` | 失败优先 |
| 统计收益 | `rtk gain` | 查看节省 token |
| 找漏用机会 | `rtk discover` | 扫历史记录找可被 RTK 优化的命令 |
| 原样运行但纳入统计 | `rtk proxy <cmd>` | 不过滤输出 |

### 1.4 什么时候不要用压缩输出

以下情况应考虑原始命令或 `rtk proxy`:

- 要把完整日志贴给人类或保存为证据.
- 要消费严格机器可读 JSON、`--json`、`--jq`、`--template` 输出.
- 要看完整 `git diff` 上下文做细粒度 review.
- 要排查 RTK filter 本身是否误删了关键信息.
- 命令输出很短, 压缩收益不明显.

RTK 的正确心智模型是“默认减少噪声, 必要时回原文”, 而不是“永远替代原始命令”.

## 0x02 关键细节

### 2.1 它如何节省 token

RTK 的命令生命周期可以理解为六步:

```text
parse -> route -> execute raw command -> filter -> print -> track
```

过滤策略按命令类型不同而不同:

- Git 类: 提取状态、commit、diff 统计, 去掉冗余 header.
- 测试类: 隐藏通过项, 保留失败、断言、panic 和 trace 入口.
- lint/typecheck 类: 按文件、规则、错误码分组.
- 日志类: 去重重复行, 保留计数.
- JSON 类: 可只保留结构或压缩大值.
- 文件读取类: 支持 `none` / `minimal` / `aggressive` 过滤级别.
- `ls` / `find` 类: 转为更紧凑的树形或分组输出.

源码上, 命令过滤逻辑主要在 `src/cmds/`; 通用能力在 `src/core/`; hook/init/rewrite 相关能力在 `src/hooks/`; 命令重写和历史分析规则在 `src/discover/`.

### 2.2 Codex 集成的真实边界

RTK 对不同 agent 的集成能力不一样. Claude Code、Cursor、Gemini、OpenCode、Pi、Hermes 等有 hook/plugin/extension 能力, 可以在命令执行前透明改写. Codex CLI 在当前文档里属于 rules file/prompt-level integration:

```text
rtk init --codex
  -> 写 RTK.md
  -> patch AGENTS.md, 加 @RTK.md
  -> Codex 读到规则后, 由模型自己选择是否使用 rtk
```

这意味着 Codex 场景有两个实践结论:

1. 不能假设 raw `git status` 会自动变成 `rtk git status`.
2. 想稳定省 token, 就要在给 Codex 的本地规则里明确要求常用 shell 命令走 `rtk`.

项目的 `hooks/codex/rtk-awareness.md` 也体现了这个简化策略: “Always prefix shell commands with `rtk`.” 这比复杂 hook 更脆弱, 但也更容易解释和排错.

### 2.3 如何证明 Codex 已经安装并使用上 RTK

这里要把“安装了 RTK”和“Codex 已经用上 RTK”拆成三层证据.

第一层是证明本机有正确的 RTK 二进制:

```bash
command -v rtk
rtk --version
rtk gain
```

本机这次核对到的结果是:

```text
/home/hx/.local/bin/rtk
rtk 0.43.0
```

这只能证明 RTK 已安装并在 `PATH` 中, 不能证明 Codex 已经读取了 RTK 规则.

第二层是证明 Codex 的规则文件已经落盘:

```bash
rtk init --show --codex
```

项目级成功时, 关键输出应该类似:

```text
[ok] Local RTK.md: RTK.md
[ok] Local AGENTS.md: @RTK.md reference
```

全局级成功时, 关键输出应该类似:

```text
[ok] Global RTK.md: ...
[ok] Global AGENTS.md: RTK.md reference
```

如果看到:

```text
[--] Local RTK.md: not found
[--] Local AGENTS.md: exists but rtk not configured
```

那就说明当前项目的 Codex 还没有安装 RTK 规则. 这次在当前工作区实际看到的就是这个状态: RTK 二进制已安装, 但 Codex local/global 配置都没有完成. 要把项目级规则装上, 才运行:

```bash
rtk init --codex
```

要把全局规则装到 `$CODEX_HOME`, 没有设置时回退到 `~/.codex`, 才运行:

```bash
rtk init --global --codex
```

第三层是证明 Codex 实际使用了 RTK. 对 Codex 而言, 最直接的证据不是“存在 hook”, 而是本轮 Codex 发起的 shell 命令本身带了 `rtk` 前缀, 例如终端日志里能看到:

```bash
rtk git status
rtk cargo test
rtk pytest -q
```

执行过 RTK 命令后, 还可以用本地统计交叉验证:

```bash
rtk gain --history
```

如果历史里出现最近的 `rtk ...` 命令和 token savings, 就能证明最近确实通过 RTK 跑过. 但这个命令依赖本地 tracking 数据库; 数据库路径不可写或尚未初始化时, 它不能作为唯一证据.

容易混淆的是 `rtk hook check`:

```bash
rtk hook check "git status"
```

它会输出:

```text
rtk git status
```

这只能证明 RTK 的 rewrite engine 知道如何把 `git status` 改写成 `rtk git status`, 不能证明 Codex 有运行时 hook. 本机 `rtk hook --help` 列出的 hook processor 包括 `claude`、`cursor`、`gemini`、`copilot`、`check`, 没有 `codex`. 上游 `hooks/codex/README.md` 也明确写的是 “Prompt-level guidance via awareness document -- no programmatic hook”.

所以如果要证明“Codex 通过 hook 透明改写了命令”, 反而要做反证实验: 让 Codex 原样发起 `git status`, 再观察最终执行行是否被宿主改成了 `rtk git status`. 只要执行行仍是 `git status`, 那就不是透明 hook. 按 `rtk-ai/rtk` 当前实现, Codex 集成是 `AGENTS.md` + `RTK.md` 的项目/全局规则文件, 由模型按规则显式写 `rtk <cmd>`.

### 2.4 配置、回退与恢复

RTK 配置文件位置:

| 平台 | 路径 |
| --- | --- |
| Linux | `~/.config/rtk/config.toml` |
| macOS | `~/Library/Application Support/rtk/config.toml` |

常用配置:

```toml
[tracking]
enabled = true
history_days = 90

[filters]
ignore_dirs = [".git", "node_modules", "target", "__pycache__", ".venv", "vendor"]
ignore_files = ["*.lock", "*.min.js", "*.min.css"]

[tee]
enabled = true
mode = "failures"

[hooks]
exclude_commands = ["git rebase", "git cherry-pick"]
```

单次禁用:

```bash
RTK_DISABLED=1 git status
```

失败输出恢复是一个关键设计. 当命令失败且输出被压缩时, RTK 会把完整原始输出保存到 tee 文件, 并在压缩结果里给出路径:

```text
FAILED: 2/15 tests
[full output: ~/.local/share/rtk/tee/1707753600_cargo_test.log]
```

这对 Codex 很重要: 先读短输出判断方向, 需要细节时再读 tee 文件, 不要直接重跑一次大输出命令.

### 2.5 自定义 filters

RTK 支持项目级和用户级自定义 TOML filter:

```text
.rtk/filters.toml
~/.config/rtk/filters.toml
```

内置 filter 编译进 binary; 自定义 filter 需要显式 trust:

```bash
rtk trust
rtk untrust
```

trust 绑定文件内容 hash. filter 文件被改过后需要重新 trust. 这不是 sandbox, 但能防止“随便 clone 一个仓库后它的 `.rtk/filters.toml` 立刻改写 agent 看到的输出”.

一个极简 filter 形状:

```toml
[filters.my-tool]
description = "Strip noisy lines from my-tool"
match_command = "^my-tool\\b"
strip_ansi = true
strip_lines_matching = ["^debug:", "^\\s*$"]
max_lines = 40
on_empty = "my-tool: ok"
```

### 2.6 隐私与遥测

RTK 本地 tracking 和远程 telemetry 要分开理解:

- `rtk gain` 使用本地 SQLite 历史数据, 默认记录命令输入/输出 token 估算、节省比例、执行时间等.
- telemetry 是匿名聚合数据, 文档声明默认关闭, 需要 `rtk init` 期间或 `rtk telemetry enable` 显式同意.
- telemetry 不采集源码、文件内容、完整命令行、参数、路径、secret、仓库名或 URL.

管理命令:

```bash
rtk telemetry status
rtk telemetry enable
rtk telemetry disable
rtk telemetry forget
export RTK_TELEMETRY_DISABLED=1
```

从本地使用角度, 推荐先保持 telemetry 关闭, 等确认自己愿意贡献匿名使用统计后再启用.

### 2.7 Codex 本地心得

我更倾向把 RTK 配成 Codex 的“默认 shell 习惯”, 但保留几条硬边界:

1. **读事实时先省 token, 做最终判断前看原文**: `rtk grep` 找方向, 关键结论再打开原文件.
2. **长测试输出先用 RTK**: `rtk cargo test`、`rtk pytest` 对 agent 友好, 失败后再按 tee path 深挖.
3. **结构化输出别乱压缩**: `gh --json`、`jq` pipeline、脚本消费 JSON 时应 raw 或 `rtk proxy`.
4. **权限/破坏性操作不靠 RTK 简化决策**: `rm`、`git reset`、迁移脚本这类操作, 输出短不代表风险低.
5. **把 RTK 当输出层, 不当执行隔离层**: 它不会替你 sandbox 命令, 也不会替你判断命令是否该运行.
6. **Codex 不等于 hook**: 在 Codex 里最稳的是显式写 `rtk <cmd>`, 或把 `@RTK.md` 放到能被 Codex 稳定读取的位置.

一句话总结: RTK 对 Codex 的最大价值是减少“看目录、搜代码、跑测试、读失败日志”这些高频动作的上下文成本; 它不应该介入需要完整证据链或机器精确输出的路径.

## 0x03 引用与本地核对

- 上游项目: [`rtk-ai/rtk`](https://github.com/rtk-ai/rtk).
- 本次核对的上游 commit: [`31f9d43d81f90d29e89142f3306473e786e59f6c`](https://github.com/rtk-ai/rtk/tree/31f9d43d81f90d29e89142f3306473e786e59f6c).
- Codex 集成说明: [`hooks/codex/README.md`](https://github.com/rtk-ai/rtk/blob/31f9d43d81f90d29e89142f3306473e786e59f6c/hooks/codex/README.md).
- Codex 注入规则正文: [`hooks/codex/rtk-awareness.md`](https://github.com/rtk-ai/rtk/blob/31f9d43d81f90d29e89142f3306473e786e59f6c/hooks/codex/rtk-awareness.md).
- Codex 初始化实现: [`src/hooks/init.rs`](https://github.com/rtk-ai/rtk/blob/31f9d43d81f90d29e89142f3306473e786e59f6c/src/hooks/init.rs).
- 本机核对命令: `command -v rtk`, `rtk --version`, `rtk init --show --codex`, `rtk hook --help`, `rtk hook check "git status"`.
