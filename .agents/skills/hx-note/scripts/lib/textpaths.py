"""hx-note 各脚本共用的「CLI 参数 -> 待处理文件」展开。

`hx_voice.py` (AI 味检测 / 折行合并) 与 `format_cn_punct.py` (标点归一化)
默认扫的是**同一批文件**。排除规则一旦各写一份, "指一个目录递归跑"就会给出
互相矛盾的结论  那比根本不支持目录更坏, 因为它看起来是对的。

三种参数形态: 文件原样收下; 目录递归收 `*.md` 并跳过 SKIP_DIR_NAMES 与隐藏目录;
含 `*?[` 的串相对 cwd 用 rglob 展开。

隐藏目录默认不进 (ai-docs/.hx-staging/ 实测占 ai-docs 全部 md 的 51%)。
真要扫隐藏目录就把它自己当参数传。
"""
from __future__ import annotations

from pathlib import Path


SKIP_DIR_NAMES: set[str] = {
    '__pycache__', 'node_modules', '.git', '.venv', 'venv',
    'dist', 'build', 'out', 'coverage', 'target',
}

GLOB_CHARS = '*?['


# Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织; 沉淀必须产出可复用物
# 
# 
def _keep(p: Path, base: Path) -> bool:
    """
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
判断 rglob 命中的路径要不要保留。只看目录段 (末段是文件名, 不参与)。"""
    dirs = p.relative_to(base).parts[:-1]
    if any(part in SKIP_DIR_NAMES for part in dirs):
        return False
    return not any(part.startswith('.') for part in dirs)


def _under_dir(target: Path) -> list[Path]:
    return [p for p in sorted(target.rglob('*.md')) if _keep(p, target)]


def _glob(pattern: str) -> list[Path]:
    base = Path()
    return [p for p in sorted(base.rglob(pattern)) if p.is_file()]


def _dedup(files: list[Path]) -> list[Path]:
    """按实体去重, 保留首次出现的路径写法。

    需要它的场合: 参数里同时给了一个目录和指向它的软链 (本仓 `.agents/skills/`
    下大量是软链, 指向 `HXLoLi/.agents/skills/`), 否则同一份文件会被检查两遍,
    报告里出现两条互相印证的"命中", 看起来像两个问题。
    """
    seen: set[str] = set()
    out: list[Path] = []
    for p in files:
        try:
            key = str(p.resolve())
        except OSError:
            key = str(p)
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out


def expand_markdown(raws: list[str]) -> tuple[list[Path], list[str]]:
    """
    .agents/notes/implemented/architecture/2026-10-03-doc-path-expansion-single-source.md
把 CLI 参数展开成 (待处理文件, 找不到的参数)。

    **不要把它退回成"每个脚本各写一份"**  两个脚本的默认扫描面必须一致,
    排除规则分叉的代价是静默的。见 ``。
    """
    # Agent Notes: 文风脚本的目录递归只留一份路径展开实现
    # 
    files: list[Path] = []
    missing: list[str] = []
    for raw in raws:
        target = Path(raw)
        if target.is_dir():
            files.extend(_under_dir(target))
        elif target.is_file():
            files.append(target)
        elif any(ch in raw for ch in GLOB_CHARS):
            hits = _glob(raw)
            files.extend(hits)
            if not hits:
                missing.append(raw)
        else:
            missing.append(raw)
    return _dedup(files), missing
