# Agent Note: hxid 裸链接渲染后无法跳转 (前端点不动)

Status: implemented

- 影响: `ai-docs/002-AI/004-记忆/003-obsidian-second-brain/index.md` / `ai-docs/002-AI/004-记忆/005-Agent-Memory框架选型/index.md` / `scripts/quality-gate.mjs`
- 现象: 正文里形如 `[标题](hxid:hx-3ac6bdbe)` 的跨文章链接**在前端点不动**

## Problem

用户报告: "hxid 目前使用这种的链接, 我好像无法直接在本地进行跳转呢 我的意思是说, 前端我没法跳转".

关键在"前端"两个字: 源文件本身没问题(`hx_docs_id.py check` 全绿), 坏的是**渲染结果**。

## Root cause

hxid 链接有两种形态, 只有一种能渲染:

| 形态 | 源码写法 | 渲染出的 href | 能跳转 |
|---|---|---|---|
| bare | `[标题](hxid:hx-3ac6bdbe)` | `href="hxid:hx-3ac6bdbe"` | **否** |
| tagged | `[标题](../005-x/index.md "hxid:hx-3ac6bdbe")` | `href="/HXLoLi/knowledge-base/AI/记忆/005-x"` | 是 |

`hxid:` 是**自定义 scheme, 不是 URL**。Docusaurus 把它原样当 `href` 输出, 浏览器既不认识这个
协议, 前端也**没有任何拦截器**(全仓 `src/` 里 `grep 'hxid:'` 零命中), 于是点击无反应 ——
既不报错也不跳转, 所以特别难察觉。

设计意图在 `.agents/skills/hx-docs-organize/references/migration.md` 里写得很清楚:

> `[标题](hxid:hx-3f9a2c71)` —— 源码形态 (**人写这个**)
> `[标题](../002-乙/index.md "hxid:hx-3f9a2c71")` —— resolve 之后 (**可渲染**)
>
> 第二种形态**同时携带可渲染路径和持久身份**: 读者点得到, 脚本认得出。

也就是说 **bare 只是中间态, 必须跑 `resolve` 才能上站**。这次漏掉的正是这一步:
两篇笔记里各有裸链接(003 三处、005 四处, 共 **7 条**), 全库另有 18 条是 tagged 形态 ——
所以只有这两篇点不动, 其余正常, 进一步掩盖了问题。

### 为什么既有检查没拦住

`hx_docs_id.py check` **故意**接受两种形态: 它对 bare 链接只校验目标 hxid 存在
(`target is None` 才算错), `path` 组为空时直接跳过"路径是否陈旧"的比对。
这是合理设计 —— bare 在源码阶段是合法的、可移植的写法 —— 但它意味着
**"check 全绿" 不等于 "链接能点"**。门禁缺的是"已经 resolve 过"这一维。

## Decision

1. **就地修复**: 对全库跑 `hx_docs_id.py resolve --write`, 把 7 条 bare 全部转成 tagged。
   这是幂等的: 目录改名/移动后重跑即可把路径重算到最新, 而 hxid 永不丢失。
2. **补门禁**: 在 `scripts/quality-gate.mjs` 新增一项
   `hxid 链接已 resolve 为可跳转形态`, 扫描 `ai-docs/**/*.md` 里的 `](hxid:hx-XXXXXXXX)` 裸形态,
   发现即失败并直接给出修复命令。

## Evidence

构建产物对比(同一个 `npm run build` 里, 两篇文章):

```
007-检索/自建Agent检索栈与凭证治理.html  (tagged 形态)
  href="/HXLoLi/knowledge-base/程序语言/现代C++/HXLibs编写串行协程调度器"   <- 可跳转
  href="hxid:..." 出现 0 次

记忆/obsidian-second-brain.html  (修复前, bare 形态)
  href="hxid:hx-37f17262"   <- 点不动
  href="hxid:hx-3ac6bdbe"
  href="hxid:hx-462ef6c0"
```

修复后重新构建, 同一文件:

```
href="/HXLoLi/knowledge-base/AI/记忆/对话记忆与知识库增量沉淀"     <- 可跳转
href="/HXLoLi/knowledge-base/AI/记忆/自维护可插拔记忆层设计"       <- 可跳转
href="/HXLoLi/knowledge-base/AI/记忆/Agent-Memory框架选型"        <- 可跳转
href="hxid:" 出现 0 次
```

新增门禁同样做了故障注入: 把其中一条改回 bare, 门禁立刻失败并定位到行号 ——

```
[fail] hxid 链接已 resolve 为可跳转形态 :: 1 条裸链接渲染后无法跳转:
       ai-docs/002-AI/004-记忆/003-obsidian-second-brain/index.md:128 (hx-3ac6bdbe)
       ==> 运行: ... hx_docs_id.py resolve --write
```

## Consequences

- 裸 `hxid:` 链接的作用域被限定为**纯源码中间态**: 它只校验目标存在, 不保证能点;
  "check 全绿" 从此不等于 "链接能点", 上站前必须跑一次 `resolve`。
- 门禁新增"是否已 resolve"这一维, 补上了既有检查缺的那一格; 代价是多一条需要随 hxid 治理流程维护的规则。
- 本次只修这两篇的 7 条裸链接, 没有改 `hx_docs_id.py` 接受 bare 的既有行为 —— 那个改动影响面更大,
  留待单独决策。
- 没有为 `hxid:` 写前端兜底拦截器: 那需要把 hxid→路由映射整体打包进前端并保持同步, 现在不做。

## Alternatives considered

- **写前端拦截器, 让 `hxid:` 协议在客户端解析跳转**: 需要把 hxid→路由的映射表整体打包进前端
  (新增一份须与构建保持同步的产物), 且每个链接多一次运行期解析。而 `resolve` 已经在**源码层**
  一步到位解决问题, 产物是普通静态链接。选后者。
- **把 bare 形态定为非法, 让 `check` 直接报错**: 会破坏"人写 bare、脚本转 tagged"的既有工作流,
  而且要改的是 skill 里的公共脚本, 影响面超出本次范围。改为在**本项目门禁**里加一条, 更聚焦。
- **只修这两篇, 不动门禁**: 同一类问题会在下次整理后重演 —— 而它恰恰是"check 全绿但前端点不动"
  这种最难靠肉眼发现的一类。门禁是这次真正的产出。
- **把 hxid 换成站内 URL 或 `createRedirects`**: 与既有 hxid 治理体系冲突, 见 `hx-docs-organize`
  的设计前提(目录一动路径就断, 站内又没有重定向机制)。不采纳。

## 未做

- 没有改 `hx_docs_id.py` 本身的行为(它接受 bare 是有意为之)。
- 没有为 `hxid:` 写前端兜底拦截器; 若将来希望"源码里写 bare 也能直接上站", 那是另一个决策。
