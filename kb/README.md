# kb: 卡片 + 原件 + 概念索引

方案见 `.hark/plan.md` (v3). 这里只放结构说明.

```
kb/
  raw/YYYY-MM-DD-<slug>/   原件. 只读, 不改写. 卡片里的 > 摘录必须是这里文件的逐字子串
  raw/self/                作者自己的记录 (笔记, git log, shell history, 口述原文). 只能由作者或采集脚本写入
  cards/<领域>/<id>.md     卡. 一句结论 + 逐字摘录 + 动作, ≤ 15 行
  concepts.yaml            概念表 (作者维护). AI 只能往 concepts.proposed.yaml 提议
  index.json               scripts/kb index 生成, 不手改
```

命令: `python3 scripts/kb.py lint|index|find|use`.

## 原件的已知缺口

`raw/2026-11-08-*` 来自 hx-note 试跑 (ai-docs/.hx-staging/hx-note-trial-20261108/facts/*/source/).
其中 material.md 已经是采集时的要点摘录 (中文, 带 [Sn §节] 来源标记), 不是网页原文;
网页原文当时没有落盘. 所以对这三个主题, 卡片的逐字依据最多追到 material.md 这一层,
再往上只能按 provenance.md 里的 URL 回查. 以后新主题要把网页快照 / 转写原样存进 raw/.
divergence.md 与 .hx-info.md 是 AI 推论产物, 不算原件, 没有迁入.
