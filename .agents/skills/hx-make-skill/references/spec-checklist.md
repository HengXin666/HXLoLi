# Agent Skills 规范速查 (validation source)

本文是 `scripts/validate_skill.py` 每一条检查的规范依据. 规范来自公开的 Agent Skills 标准与 Anthropic 官方说明, 二者互相一致.

## 1. 目录形态

A skill 是一个目录, 至少含一个 `SKILL.md`:

```
skill-name/
├── SKILL.md          # 必需: metadata + instructions
├── scripts/          # 可选: 可执行代码
├── references/       # 可选: 按需读入上下文的文档
├── assets/           # 可选: 只用于产出, 不进上下文
└── ...               # 任意其他文件
```

## 2. Frontmatter 字段

| 字段 | 必需 | 约束 | 校验器对应 |
|---|---|---|---|
| `name` | 是 | 1-64 字符; 只允许 `a-z` `0-9` `-`; 不能以 `-` 开头或结尾; 不能有连续 `--`; **必须与父目录名一致** | ERROR |
| `description` | 是 | 1-1024 字符; 必须同时描述"做什么"和"什么时候用" | ERROR (长度) / WARN (缺 Use when) |
| `license` | 否 | 许可证名或指向 bundle 内 license 文件 | - |
| `compatibility` | 否 | 1-500 字符; 只在有特定环境要求时才写 | ERROR |
| `metadata` | 否 | string -> string 映射 (作者、版本等) | ERROR (非映射) |
| `allowed-tools` | 否 | 空格分隔的预授权工具; **实验性**, 各家支持不一 | - |

### 易踩的命名坑

```yaml
name: PDF-Processing   # 大写, 无效
name: -pdf             # 以连字符开头, 无效
name: pdf--processing  # 连续连字符, 无效
name: pdf-processing   # 有效
```

## 3. 三级渐进式披露与预算

| 级别 | 内容 | 何时进上下文 | 预算 | 校验器对应 |
|---|---|---|---|---|
| L1 | `name` + `description` | 所有已安装 skill 启动时预载 | 约 100 词 | - |
| L2 | `SKILL.md` body | 触发时整篇读入 | 少于 5000 token, 少于 500 行 | ERROR (>500 行) |
| L3+ | `scripts/` `references/` `assets/` | 按需读取 | 基本无上限 | WARN (>10k 词的 reference) |

关键点: **"内容太多"的正确解法是拆分, 不是压缩.** 有文件系统时, 一个 skill 能携带的上下文总量实际上没有上限 —— 前提是按"是否总是需要"把内容分到 L2 和 L3.

## 4. 信息架构规则

| 规则 | 依据 | 校验器对应 |
|---|---|---|
| "什么时候用"只能写在 `description`, **不能写在 body** | 官方: body 只在触发后才加载, 所以 body 里的 "When to Use" 段永远不会被读到 | ERROR |
| 不放 `README.md` / `CHANGELOG.md` / `INSTALLATION_GUIDE.md` / `QUICK_REFERENCE.md` | 官方明确禁止: skill 目录是给 agent 用的, 额外文档只增加混乱 | ERROR |
| `references/` 与 `scripts/` 下每个文件都要在 SKILL.md 里被引用并说明何时读 | 官方: 模型必须知道它存在、以及什么时候用 | ERROR |
| 信息不能同时在 SKILL.md 和 `references/` 里重复 | 官方: 二者只能存一份 | 人工检查 (近重复检测) |
| 文件引用用相对路径, 且**不超过一层** | 规范原文: keep file references one level deep, avoid deeply nested chains | ERROR |
| 大 reference 文件 (>10k 词) 应在 SKILL.md 给出 grep 提示 | 官方建议 | WARN |

## 5. 校验器用法

```bash
uv run scripts/validate_skill.py <skill-dir>
uv run scripts/validate_skill.py <skill-dir> --json
```

退出码: `0` = 通过 (可能有 WARN), `1` = 有 ERROR, `2` = 参数错误.

实现要点: 优先用 PyYAML; 若无 PyYAML 则退化为内置的顶层 `key: value` 解析器, 因此脚本无第三方依赖.
