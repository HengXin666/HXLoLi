# 6 derive — 怎么讲给人听

## 只做这一件事

把 `.hx-info.md` 派生成面向人类阅读的 `index.md`。

**事实只能来自 `.hx-info.md`。** 发现需要新事实 -> **先回写 `.hx-info.md` 并补依据**, 再用。
这条由步骤 8 的保真盲审兜住。

## 产物

`index.md`, 结构按模板 (默认「兴趣&展望」)。

## 实现

| 要解决的问题 | 读 |
|---|---|
| 文章该分几层、主线与支线怎么放 | `impl/layering.md` |
| 配什么图、生图后端怎么降级 | `impl/media.md` |
| 演示页的版式与配色 | `impl/theme-spec.md` |
| `.tsx` 内联演示页怎么写 | `impl/tsx-deck.md` |

文章结构模板在 `templates/`: `templates/interest-outlook.md` (默认) 与
`templates/problem-solution.md` (踩坑复盘)。模板索引 `templates/_registry.md`。

动笔前要拿到**三样**输入:

1. **用户画像** —— `scripts/hx_persona.py` 产出。
2. **已积累的好/坏表达** —— `scripts/hx_voice.py samples`。
3. **引入的素材** —— 步骤 1 记在 `intake.md` 里的那次具体遭遇。

第 3 样最容易被跳过, 也最要命。**引入不许在这里现编。** 到这里你手上只有一份整理好的知识,
如果现在才想开头, 只能"为主题回找一个合适的入口" —— 那是倒推的。

判据 (详见 `templates/interest-outlook.md` 的引入一节):

> 这个引入写的那件事, 在我决定写这篇文章之前就发生过吗?

`intake.md` 里没有相关遭遇时: 回去问人类, 或把引入退成"我为什么要整理这个"的真实动机 ——
**不要编一段。** 编的会被当真, 比没有更坏。

## 过关条件

```bash
uv run .agents/skills/hx-note/scripts/hx_voice.py lint <index.md> --profile article
```

**零 E 级命中**才推进。

## 扩展方式

新文章结构 = 加一个 `templates/*.md` 并在 `_registry.md` 登记。**不改本文件。**
