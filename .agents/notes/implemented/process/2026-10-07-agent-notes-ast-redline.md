# Agent Note: Agent Notes 以 AST 双向图执行红线

Status: implemented

Decision-ID: agent-notes-ast-redline

## Problem

任意 note 改动都满足源码覆盖率时, 无关记录可以掩盖决策事实过期. 仅搜索路径和声明关键词不能证明引用属于函数, 花括号路径与重复锚点也无法形成精确可扫描的关联

## Code

- `.agents/skills/hx-agent-notes/scripts/redline/graph.py`
- `.agents/skills/hx-agent-notes/scripts/setup/install.py`
- `.agents/skills/hx-agent-notes/scripts/tests/test_gate.py`
- `.agents/skills/hx-agent-notes/scripts/cli/verify-all.ts`
- `.agents/skills/hx-agent-notes/scripts/authoring/init-agent-notes.ts`


## Decision

v2 门禁以精确 Code 列表和源码声明顶部多行注释构成双向图. Tree-sitter 检查 cpp, ts, tsx, js, mjs, go, py, rs 的函数或纯声明归属, Python 允许函数首条多行 docstring 或紧邻定义的连续注释. 缺依赖和语法错误都失败

每条 Decision-ID 只由一篇 active note 拥有, 每个直接父目录只选择一个代表文件和一个引用位置. 同目录兄弟源码 diff 归属于该目录的决策, 子目录独立计算. 新旧图共同约束删除和改名, 每篇 note 的每个目录都必须两端同时变化或同时不变, 单边变化失败并要求 review

全量扫描与 guarded 配置变更要求每个有受保护源码的直接父目录都由 active note 的 Code 路径拥有. 未关联决策的存量目录报告 unowned-directory 并要求 review, 不因没有源码 diff 而漏检. 上级目录的决策不递归覆盖子目录

项目红线入口委托 vendored skill 实现. Python 的安装器, 生成入口, npm 脚本, hook 和开发期校验通过 uv 运行, 测试子进程继承 uv 解释器. 离线运行预置 uv 缓存并使用 `UV_OFFLINE=1`. 统一入口支持 diff, 全工作区, 精确 Git index 和精确提交树. Code 路径无效或反向锚点缺失时, 诊断定位到 note 的精确 Code 行并关联源码路径. JSON 分开保存比较树 base, 实际基线提交 base_commit 和已删除的相关路径 deleted, 为报告提供删除前的文件位置

CI 收集同一严格扫描器的诊断并保持工作流成功, 本地扫描仍以非零退出码暴露无效双链. 默认分支的可信脚本发布对应代码片段评论, 具体运行与评论契约由 `.agents/notes/implemented/process/2026-10-08-agent-notes-advisory-comments.md` 拥有

废弃 note 直接删除并修复全部入站引用, Git 保留历史. 精确文件保护, 纯声明节点与首次 v1 比较边界由 `.agents/notes/implemented/process/2026-10-08-agent-notes-v2-migration-boundaries.md` 约束. v1 配置必须显式迁移, 旧命令委托新红线, 不继承禁用开关和全局豁免. 模板将短约束写入项目 AGENTS.md, 独立 CODEOWNERS review 审核机制本身

## Alternatives considered

- 什么都不做 / 复用现有: 无需迁移也没有新增 parser 依赖, 但关键词位置判断与任意 note 覆盖源码无法证明对应关系
- 正则匹配函数头: 依赖少且便于调研, 但装饰器, 模板, 箭头函数和普通字符串会产生歧义, 不足以承担 AST 位置红线
- 每个代码文件重复同一决策: 单文件阅读时能直接看到理由, 但同目录重复路径和锚点会扩大维护面, 代表文件与目录 diff 归属保持每目录一次
- 自动接受作者填写的豁免理由: 单边文字纠错更方便, 但作者声明不能证明独立 review, 因此保持失败并交由仓库受控例外流程处理
- 永久冻结 archive: 能在当前树保留历史, 但失效记录增加调研负担, 本机制直接淘汰并通过 Git 追溯

## Consequences

机器能证明路径, 位置, 唯一性和 diff 配对, 不能证明两条不同 ID 是否语义重复, 也不能证明理由真实. 空洞的双边改动仍需人审. 多目录决策的每个目录都参与配对, 文字单边修改也需要独立 review

运行期需要固定版本的 Tree-sitter 依赖, 离线环境必须预置依赖或缓存. 不完整语法及 parser 不支持的扩展不会静默回落. 旧仓库配置和存量 note 需要显式迁移, 本地脚本本身不能代替人审

## Testing

使用隔离 Git 仓库检查八种语言, 伪引用, 增删改名, 单边 diff, 无关 note, 同目录兄弟文件, 存量无主目录和暂存区隔离. 独立试用以两个 Python 文件验证共享锚点, 正反例和安装幂等. GitHub 报告使用模拟 API 验证, 实际远端触发依赖采用仓库启用工作流
