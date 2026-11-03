"""来源分级与广度门槛 (跨域共享, 被 flow 的 done collect 与 sources 子命令 import): 解析 provenance 来源表与 material 要点引用, 给出不达标的原因。"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

MIN_SOURCES = 3
MIN_T1 = 1
MIN_COUNTER = 1
KINDS = ("article", "research")
ROW_RE = re.compile(r"^\|\s*(S\d+)\s*\|", re.M)
TIER_RE = re.compile(r"^T[123]$")
CITE_RE = re.compile(r"\[(S\d+(?:\s*[,，]\s*S\d+)*)[^\]]*\]")
COUNTER_TITLE = "## 反例"


@dataclass
class Breadth:
    tiers: dict[str, str] = field(default_factory=dict)
    points: int = 0
    counter: int = 0
    problems: list[str] = field(default_factory=list)

    @property
    def t1(self) -> int:
        return sum(1 for t in self.tiers.values() if t == "T1")


def read_tiers(path: Path, out: Breadth) -> None:
    """来源表每行形如 | S1 | 名称 | T1 | ...; 等级格必须是 T1/T2/T3。"""
    if not path.is_file():
        out.problems.append("缺 " + path.name)
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        if not ROW_RE.match(line):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        tier = next((c for c in cells[1:] if TIER_RE.match(c)), "")
        if not tier:
            out.problems.append(cells[0] + " 没有标等级 (T1/T2/T3)")
        out.tiers[cells[0]] = tier


def cited(line: str) -> list[str]:
    ids: list[str] = []
    for grp in CITE_RE.findall(line):
        ids += [x.strip() for x in re.split(r"[,，]", grp) if x.strip()]
    return ids


def read_points(path: Path, out: Breadth) -> None:
    """要点 = 列表项; ## 反例 一节里的列表项只计数, 不要求 T1。"""
    if not path.is_file():
        out.problems.append("缺 " + path.name)
        return
    in_counter = False
    for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if line.startswith("## "):
            in_counter = line.strip() == COUNTER_TITLE
            continue
        if not re.match(r"^\s*[-*]\s+\S", line):
            continue
        if in_counter:
            out.counter += 1
            continue
        out.points += 1
        ids = cited(line)
        unknown = [i for i in ids if i not in out.tiers]
        if not ids or unknown:
            out.problems.append("material.md:%d 没有引用已登记的来源编号: %s" % (n, line.strip()[:40]))
            continue
        tiers = [out.tiers[i] for i in set(ids)]
        if "T1" not in tiers and tiers.count("T2") < 2:
            out.problems.append("material.md:%d 只有 T2 单源或 T3: %s" % (n, line.strip()[:40]))


def breadth_report(source_dir: Path) -> Breadth:
    out = Breadth()
    read_tiers(source_dir / "provenance.md", out)
    read_points(source_dir / "material.md", out)
    if len(out.tiers) < MIN_SOURCES:
        out.problems.append("来源只有 %d 个, 要求 >= %d" % (len(out.tiers), MIN_SOURCES))
    if out.t1 < MIN_T1:
        out.problems.append("没有 T1 来源 (规范 / RFC / 官方文档 / 源码 / 论文)")
    if out.counter < MIN_COUNTER:
        out.problems.append("material.md 缺 `## 反例` 一节或其中没有条目")
    return out
