# organize — 整理存量笔记

**第二条入口. 不重叠于九步沉淀流水线. **

## 只做这一件事

改**已有**笔记的归类与身份: 重新分类 / 改名 / 搬家 / 整理 tag / 修断链

**不要用沉淀流水线去改已有笔记的归属. **

## 三档自主权 (先定档再动手)

| 档 | 什么 | 要不要问 |
|---|---|---|
| 卫生 | 补 `NNN-` 前缀、去非法字符、修可解析的断链 | 直接做 |
| 分类内重排 | 同一分类下的顺序与命名 | 列清单待批 |
| 结构变更 | 拆分类 / 新建一级 / 跨类搬家 | **逐条同意** |

## 实现

| 要解决的事 | 读 |
|---|---|
| 这篇该留、该聚合、还是该独立新建分类 | `impl/decision-matrix.md` |
| 移动/改名后链接还对不对 (hxid 机制) | `impl/migration.md` |
| tag 该不该存在、上级大类是什么 | `impl/taxonomy.md` |
| 目录树按什么维度切、层级多深 | `impl/taxonomy-directory.md` |
| 移动前先做全库体检 (10 项) | `impl/batch-audit.md` |
| 只体检单篇 (8 项) | `impl/single-note-audit.md` |

## 脚本

- `scripts/cli/identity/hx_docs_id.py`  hxid 分配与校验 (`assign` / `check` / `resolve` / `links`)
- `scripts/cli/taxonomy/hxloli_tags.py`  tag 注册表 (`suggest` / `scan` / `generate` / `check`)
- `scripts/cli/textfmt/format_cn_punct.py`  标点归一化

## 收尾

与步骤 9 相同: 重建侧边栏 -> 跑 `doctor` 或等价的库级体检
