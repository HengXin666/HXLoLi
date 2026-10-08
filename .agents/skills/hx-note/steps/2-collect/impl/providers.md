# 数据源 (provider) 清单与扩展方式

## 现有 provider

| provider | 数据来源 | 可靠性 | 降级行为 |
|---|---|---|---|
| `blog` | `blog/**/*.md` frontmatter 的 `date` / `title` / `tags` | 最高 (作者亲手写的) | 窗口内不足 6 篇时自动放宽到最近 6 篇, 并在产物里标注放宽了 |
| `projects` | `blog` 正文里的 `github.com/<owner>/<repo>` 链接 + 同行文字 | 高 | 没有链接就整节缺失 |
| `ai-docs` | `ai-docs/**/index.md` 的 `created_at` / `tags` | 高 | 目录不存在就整节缺失 |
| `voice` | `blog` 正文里带口吻标记的原句 | 高 | 抽不到就整节缺失 |
| `git` | `git log --since=<窗口起点>` | 中 | 不是 git 仓库 / 无提交 -> 写进"数据缺口" |
| `github` | `gh api /user/repos?sort=pushed` | 低 (需 `gh` + 登录态) | 没装 `gh` 或未登录 -> 写进"数据缺口" |

**这个仓库的实测情况**: `git` 与 `github` 两个 provider 都不可用 (工作副本不是 git 仓库,
本机没有 `gh`). 这正是 provider 必须能独立降级的原因  缺两个数据源画像依然可用

## 被刻意排除的数据源

**浏览器历史记录不做. ** 理由不是做不到, 是不该做

- 需要读 `~/Library/Application Support/<浏览器>/Default/History` 这个 SQLite 文件, 浏览器运行时会加锁, 且 macOS 会弹权限请求
- 它包含大量与写作无关的隐私数据, 为了两句"引入"去读全量浏览历史不成比例
- 想要这个信号的正确做法是: 人类自己把"最近在看什么"写进 `ai-docs/.hx-persona.extra.md`. 一行手写胜过一个会失败的抓取器

如果日后确实要加, 也应该做成一个**只读取用户显式导出的书签/历史文件**的 provider, 而不是直接去翻浏览器目录

## 新增一个 provider

改**一个文件**, 加**一个函数**, 登记**一行**

```python
def collect_xxx(ctx: Ctx) -> Section:
    # ctx.root  仓库根
    # ctx.since 窗口起点 (date)
    # ctx.months 窗口月数
    if 拿不到数据:
        return Section("xxx", False, note="为什么拿不到, 一句话说清")
    return Section("xxx", True, ["- 每行一条, markdown"])

PROVIDERS = [
    ...,
    ("这一节在画像里的标题", collect_xxx),
]
```

硬约束

1. **只返回, 不抛异常.** 任何一个 provider 抛异常都会毁掉整次构建. 捕获所有`OSError` / `SubprocessError` / 超时, 转成 `Section(ok=False, note=...)`
2. **失败要带原因.** `note` 会原样进产物的"数据缺口"一节, 人类靠它判断哪些结论不可信. 写"不可用"是没用的, 要写"gh 未登录"
3. **必须有超时.** 所有外部调用带 `timeout=`, 画像构建不允许挂住写作流程
4. **不产生副作用.** provider 只读; 不写文件、不发网络写请求、不动 git 状态

## 人类覆盖

`ai-docs/.hx-persona.extra.md` 存在时, 内容会被原样并入产物末尾, 并标注"优先于以上全部自动结论". 用它来

- 补脚本拿不到的事实 (在调研什么、最近在读什么、当前工作项目)
- 纠正自动结论 (例如某个仓库已经弃坑了)

这个文件是手写的, `build` 不会覆盖它
