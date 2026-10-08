# 5 place — 放哪里

## 只做这一件事

**纯机械, 不与人类讨论内容. ** 定目录 -> 建目录 -> 初始化模板 -> 搬迁产物

## 产物

正式目录 `ai-docs/<NNN-一级>/<NNN-二级>/<NNN-标题>/`, 内含 `index.md` 骨架与搬进去的`.hx-info.md` / `.hx-mitemite.md`

## 做法

```bash
# 先看现有分类与编号 (编号有跳号, 不要凭猜)
ls ai-docs/*/ ai-docs/*/*/

F="uv run .agents/skills/hx-note/scripts/cli/flow/hx_flow.py"
$F place --slug "<slug>" --to "ai-docs/<NNN-一级>/<NNN-二级>/<NNN-标题>" \
  --title "<标题>" --tag "<规范tag>" --model "<实际模型名>"
```

`place` 做四件事: 校验 `--to` 在 `ai-docs/` 下、调 `scripts/cli/authoring/makeDoc.py` 生成 `index.md`、搬迁产物、把 `.hx-info.md` 的 `hxid` 对齐成 `index.md` 的

## 规则

- 新建**一级/二级分类目录**必须人类同意; 在已有分类下建文章目录不必
- `--tag` 只能用 `ai-docs/.hx-tags.toml` 里的规范名. 先跑`scripts/cli/taxonomy/hxloli_tags.py suggest "<词>"` 查, **不要现编近义词**
- 编号取该目录下现有最大值 +1. 仓库里存在跳号 (如 `004-记忆` 缺 `004`), **不要回填**
- `--model` 按实际用的模型填; 确实不知道就 `Unknown`, **禁止编造**

## 实现

命名、编号、tag、可见性与双链的细则: `impl/naming-and-identity.md`

## 过关条件

`target_dir` 已记录, `index.md` 模板已生成, `.hx-info.md` 已搬进去
