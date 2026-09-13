# tag 分类学: 从大类到细分的倒排树

tag 不是"这篇讲了什么"的关键词堆, 而是**一张可导航的倒排索引**. 索引的读者是"想找某一类知识的人", 所以它必须先有大类, 再有细分 —— 一上来就是 `偏特化` 这种词, 等于没有索引.

## 铁律

1. **tag 是概念, 不是目录名.** 禁止把分类目录名当 tag: `003-编程语言` `项目学习` `想法探索` `商品选购` 这类词删除. 目录负责位置, tag 负责概念, 两者不重复表达同一件事.
2. **tag 是概念, 不是标题碎片.** 只出现一次且是某篇标题里的专有词组 (如 `事件可信度` `检测面建模`) 应上升为它真正所属的上级概念, 或作为 `aliases` 并入已有 tag.
3. **每篇 3~6 个 tag.** 少于 3 检索不到, 多于 6 说明没想清楚这篇到底属于什么.
4. **每个 tag 必须能回答"它的上级是谁".** 答不出上级的 tag 是孤儿, 要么并入某个大类, 要么删除.
5. **优先复用.** 新增前先 `suggest`. 一次整理最多新增 2 个规范 tag.

## 树形规则 (倒排索引的层级)

```
L1 大类 (个位数, 稳定不变)      编程语言 / AI Agent / 逆向与风控 / ...
 └─ L2 领域 (每类 3~10 个)      C++ / Python / 记忆系统 / 浏览器指纹 / ...
     └─ L3 机制或概念 (按需)     模板元编程 / 双时态 / 检测面建模 / ...
```

- **L1 只在需要时新增, 且必须人类同意.** 现有大类够用就不要开新的 —— 新开一个大类会让全库 tag 需要重新归属.
- **L2 是日常用到的那一层.** 新笔记优先挂 L2; 只有当同一 L2 下已经聚集 ≥3 篇讲同一机制时, 才值得引入 L3.
- **L3 不许凭空发明.** 它的存在依据是"至少 3 篇笔记需要它区分", 而不是"这个词听起来很专业".
- **粒度自上而下收敛**: 先问"这属于哪个大类", 再问"大类下哪个领域", 最后才问"要不要再细分". 反过来从细词开始归类, 必然长出孤岛.

## 判定一个 tag 该不该存在 (机械可查)

对候选 tag `T` 依次回答:

| 问题 | 否 -> 动作 |
|---|---|
| 库里有没有 ≥2 篇笔记真的需要它? | 合并进上级, 或作为 aliases |
| 它是否只等于某个目录名? | 删除 |
| 它是否有明确的上级 (L1/L2)? | 并入最近的上级 |
| 它和已有 tag 的字符相似度 ≥0.7 且语义相同? | `merge "T" --into "<已有>"` |
| 换一个作者来读, 他会把同一篇笔记挂到 T 吗? | 删掉, 它太主观 |

## curated 层义务

`ai-docs/.hx-tags.toml` 的 curated 层**必须**写描述 —— 站点标签页靠它解释这个 tag 管什么. 先跑 `check --health` 看当前有多少 tag 真的写了 `desc`; 覆盖率低就是体系失效的直接症状.

整理时, 对**每一个保留下来的 L1/L2 tag** 补一行:

```toml
[tags."记忆系统"]
desc = "长期记忆的存储/召回/演化机制, 不含提示词技巧"
parent = "AI Agent"
aliases = ["记忆架构", "Memory"]
```

`aliases` 写要并入的近义词; `parent` 写它的上级 (L1 的 parent 留空). 整理收尾跑 `generate` 重建 generated 层.

## 执行顺序 (脚本)

```bash
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py scan        # 看频次 + 候选簇
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py suggest "<新词>"
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py merge "<别名>" --into "<规范名>"
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py apply        # dry-run 看差异
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py apply --write
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py generate
uv run .agents/skills/hx-docs-layout/scripts/hxloli_tags.py check
```

**`check` 通过不代表 tag 健康.** 它只校验"正文用的 tag 在注册表里存在", 而注册表是从正文自动生成的 —— 自己校验自己, 永远通过. 真正的健康指标是: 出现 1 次的 tag 占比、curated `desc` 覆盖率、候选簇是否已判定.

## 待合并候选只信一半

`scan` 的候选簇是**字符层面**的证据 (公共词缀/整词包含/字级相似). 它会给出 `Turnstile | Turnstile防御` 这种明确该合并的, 也会给出 `上下文工程 | 提示词优化 | 逆向工程` 这种只是都带"工程"二字、语义无关的. 逐簇判定, 由人拍板, 脚本不猜语义.
