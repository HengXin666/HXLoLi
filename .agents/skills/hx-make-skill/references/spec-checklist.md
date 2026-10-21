# Agent Skills 规范速查 (validation source)

本文是 `scripts/validate_skill.py` 每一条检查的规范依据. 规范来自公开的 Agent Skills 标准与 Anthropic 官方说明, 二者互相一致

## 1. 目录形态

A skill 是一个目录, 至少含一个 `SKILL.md`

```
skill-name/
├── SKILL.md          # 必需: metadata + instructions
├── scripts/          # 可选: 可执行代码
├── references/       # 可选: 按需读入上下文的文档
├── assets/           # 可选: 只用于产出, 不进上下文
└── ...               # 任意其他文件
```

## 2. Frontmatter 字段

`默认` 是 DSH 真正强制的最小集合, `--strict` 才是社区约定. 两者都要看: 默认档决定"装得上", strict 决定"写得好"

| 字段 | 必需 | 约束 | 默认 | `--strict` |
|---|---|---|---|---|
| `name` | 是 | 1-64 字符; 只允许 `a-z` `0-9` `-`; 不能以 `-` 开头或结尾; 不能有连续 `--`; 必须与父目录名一致 | ERROR (缺失, 形态) / WARN (超长, 与目录名不一致) | 同默认 |
| `description` | 是 | 1-1024 字符; 必须同时描述"做什么"和"什么时候用" | ERROR (缺失) / WARN (超长, 缺 Use when, 过短) | ERROR (超长, 缺 Use when, 过短) |
| `license` | 否 | 许可证名或指向 bundle 内 license 文件 | - | - |
| `compatibility` | 否 | 1-500 字符; 只在有特定环境要求时才写 | WARN (类型, 超长) | ERROR (类型, 超长) |
| `metadata` | 否 | string -> string 映射 (作者、版本等) | ERROR (非映射) | 同默认 |
| `allowed-tools` | 否 | 空格分隔的预授权工具; **实验性**, 各家支持不一 | - | - |

### 易踩的命名坑

```yaml
name: PDF-Processing   # 大写, 无效
name: -pdf             # 以连字符开头, 无效
name: pdf--processing  # 连续连字符, 无效
name: pdf-processing   # 有效
```

## 3. 三级渐进式披露与预算

| 级别 | 内容 | 何时进上下文 | 预算 | 默认 | `--strict` |
|---|---|---|---|---|---|
| L1 | `name` + `description` | 所有已安装 skill 启动时预载 | 约 100 词 | - | - |
| L2 | `SKILL.md` body | 触发时整篇读入 | 少于 5000 token, 少于 500 行 | WARN (>500 行) | ERROR (>500 行) |
| L3+ | `scripts/` `references/` `assets/` | 按需读取 | 基本无上限 | WARN (>10k 词的 reference) | 同默认 |

关键点: **"内容太多"的正确解法是拆分, 不是压缩.** 有文件系统时, 一个 skill 能携带的上下文总量实际上没有上限  前提是按"是否总是需要"把内容分到 L2 和 L3

## 4. 信息架构规则

| 规则 | 依据 | 默认 | `--strict` |
|---|---|---|---|
| "什么时候用"只能写在 `description`, **不能写在 body** | 官方: body 只在触发后才加载, 所以 body 里的 "When to Use" 段永远不会被读到 | WARN | ERROR |
| 不放 `README.md` / `CHANGELOG.md` / `INSTALLATION_GUIDE.md` / `QUICK_REFERENCE.md` | 官方明确禁止: skill 目录是给 agent 用的, 额外文档只增加混乱 | WARN | ERROR |
| `references/` 与 `scripts/` 下每个文件都要在 SKILL.md 里被引用并说明何时读 | 官方: 模型必须知道它存在、以及什么时候用 | WARN | ERROR |
| 信息不能同时在 SKILL.md 和 `references/` 里重复 | 官方: 二者只能存一份 | 人工检查 | 人工检查 |
| 文件引用用相对路径, 且**不超过一层** | 规范原文: keep file references one level deep, avoid deeply nested chains | WARN | ERROR |
| 一行列出三个以上裸路径且没有描述 | 本仓约定: 索引写"路径 + 一句这是什么, 什么时候读" | WARN | ERROR |
| 大 reference 文件 (>10k 词) 应在 SKILL.md 给出 grep 提示 | 官方建议 | WARN | WARN |

### 为什么默认档这么松

默认档只对齐 loader 真正强制的集合: 一个一级目录、可解析的 `name` 与 `description`、`name` 形态合法. 其余全是社区约定, loader 从不检查, 所以默认只报 WARN 且退出码为 0. 反过来, 给外部或 vendored skill 跑 `--strict` 会把上游的写法报成 ERROR, 那没有意义

## 5. 三家 loader 对照

排障时先分清是哪一家的行为, 再改 skill. 两家行为相反时两列都写, 不要用一家覆盖另一家

| 维度 | DSH | Claude Code | Codex |
|---|---|---|---|
| 扫描根 | `<project>/.dsh/skills`, `<project>/.agents/skills`, `~/.dsh/skills`, `~/.agents/skills`, `Config.customSkillDirs` | `.claude/skills` | 从当前目录逐级向上到仓库根, 每级看 `.agents/skills` |
| 嵌套 `**/SKILL.md` | 不发现, 必须放在扫描根的一级目录下 | 不发现 | 不发现 |
| 目录名与 `name` 不一致 | 仍加载 | 仍加载 | 仍加载 |
| frontmatter 解析失败 | 整个 skill 被静默丢弃, 只打一行 `logger.warn`, 调用时得到 unknown skill | 以空元数据继续加载 body | 跳过该 skill |
| 隐式调用开关 | `disable-model-invocation` | `disable-model-invocation` (为 true 时 description 不进模型上下文) | `agents/openai.yaml` 的 `allow_implicit_invocation` |
| 符号链接目录 | 支持 | 支持 | 未核实 |

`[冲突]` DSH 一列以本仓实测为准; Claude Code 与 Codex 两列来自各自公开文档, 版本变动后需复核

## 6. 校验器用法

```bash
uv run scripts/validate_skill.py <skill-dir>
uv run scripts/validate_skill.py <skill-dir> --json
uv run scripts/validate_skill.py <skill-dir> --strict     # hx 自家 skill 用这一档
```

退出码: `0` = 通过 (可能有 WARN), `1` = 有 ERROR, `2` = 参数错误

实现要点: 优先用 PyYAML; 若无 PyYAML 则退化为内置的顶层 `key: value` 解析器, 因此脚本无第三方依赖. 该回落解析器**显式拒绝**裸标量里含 `": "` 的值 (给整个值加引号), 并支持 `|` 与 `>` 两种块标量. 理由: 那种写法在 DSH 侧会让整个 skill 被静默丢弃, 而回落解析器若宽松吃掉它, 校验器就会为"加载不了的 skill"发出 PASS  这个洞在本仓真实发生过 (见 `references/legacy-audit.md`)
rences/legacy-audit.md`)
