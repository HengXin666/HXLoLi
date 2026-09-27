# 3 atom — 哪些是知识点

## 只做这一件事

写 `.hx-info.md`: 面向知识库的纯原子知识点。

**这一步不要想文章怎么写。** 想文章的那一刻就会开始铺垫, 而铺垫是知识库的污染物。

## 产物

`ai-docs/.hx-staging/<slug>/.hx-info.md` —— 每块一个 `## <完整结论句>`, 末条固定是
`- 依据: <能一次翻到原处的定位>`, 末尾有「边界」章。

## 实现

判据与禁止清单: `impl/rules.md` (五条判据、标题必须是答案、自包含、依据不是可选项)。

## 过关条件

```bash
uv run .agents/skills/hx-note/scripts/hx_voice.py lint <file> --profile atom
```

**零 E 级命中**才推进。

## 扩展方式

判据要演进就改 `impl/rules.md`, 不要在 SKILL.md 里加规则。