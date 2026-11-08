#!/usr/bin/env python3
"""skill_lint.py: 用 Agent Skills 规范 (agentskills.io/specification) 与 Claude Code / Codex 文档里
能机械检查的那几条, 体检一个 skill 目录. 只用标准库.

用法: python3 skill_lint.py <skill-dir> [<skill-dir> ...]
退出码: 有 E 级命中时为 1, 否则 0.
"""
import re
import sys
from pathlib import Path

SPEC_KEYS = {"name", "description", "license", "compatibility", "metadata", "allowed-tools"}
NAME_RE = re.compile(r"^[a-z0-9]+(-[a-z0-9]+)*$")       # 小写字母数字, 连字符不在首尾也不连续
REF_RE = re.compile(r"(?:\]\(|`)((?:[\w.-]+/)*[\w.-]+\.(?:md|py|sh|json|html))")
ABS_RE = re.compile(r"(/home/[^\s`'\"]+|/Users/[^\s`'\"]+|[A-Z]:\\[^\s`'\"]+)")


def split_frontmatter(text):
    if not text.startswith("---\n"):
        return None, text
    end = text.find("\n---", 4)
    if end < 0:
        return None, text
    return text[4:end], text[end + 4:].lstrip("\n")


def parse_simple_yaml(block):
    """只解析顶层 key: value, 足够拿到 name / description / 布尔开关. 不处理多行折叠块以外的复杂 YAML."""
    out, key = {}, None
    for line in block.splitlines():
        m = re.match(r"^([A-Za-z_][\w-]*):\s*(.*)$", line)
        if m:
            key, val = m.group(1), m.group(2).strip()
            out[key] = val.strip('"').strip("'") if val not in (">", "|", ">-", "|-") else ""
        elif key and line.startswith((" ", "\t")) and out.get(key) is not None and not isinstance(out[key], dict):
            out[key] = (out[key] + " " + line.strip()).strip()
    return out


def refs_in(path):
    try:
        return set(REF_RE.findall(path.read_text(encoding="utf-8", errors="ignore")))
    except OSError:
        return set()


def lint(skill_dir):
    d = Path(skill_dir)
    hits = []
    f = d / "SKILL.md"
    if not f.exists():
        return [("E", "no-skill-md", f"{d} 下没有 SKILL.md")]
    text = f.read_text(encoding="utf-8", errors="ignore")
    fm_raw, body = split_frontmatter(text)
    if fm_raw is None:
        return [("E", "frontmatter", "首行不是 ---, 整个文件会被当成正文, name/description 全部失效")]
    fm = parse_simple_yaml(fm_raw)

    name = fm.get("name", "")
    if not name:
        hits.append(("E", "name-missing", "缺 name (规范必填; Claude Code 会退回目录名)"))
    else:
        if len(name) > 64 or not NAME_RE.match(name):
            hits.append(("E", "name-format", f"name={name!r} 不满足 1-64 位小写字母数字+单个连字符"))
        if name != d.name:
            hits.append(("E", "name-dir", f"name={name!r} 与目录名 {d.name!r} 不一致 (规范要求一致)"))

    desc = fm.get("description", "")
    if not desc:
        hits.append(("E", "desc-missing", "缺 description: 隐式触发没有任何依据"))
    else:
        n = len(desc)
        if n > 1024:
            hits.append(("E", "desc-1024", f"description {n} 字符 > 1024 (规范上限)"))
        if not re.search(r"(use when|use this|当|触发|时使用|Use when)", desc, re.I):
            hits.append(("W", "desc-when", "description 里找不到 \"什么时候用\" 的句子"))
        hits.append(("I", "desc-len", f"description {n} 字符; 前 120 字符: {desc[:120]!r}"))

    extra = sorted(set(fm) - SPEC_KEYS)
    if extra:
        hits.append(("W", "non-spec-keys", f"非规范字段 {extra}: claude.ai 上传 / Skills API 打包会直接报错"))
    if str(fm.get("disable-model-invocation", "")).lower() in ("true", "yes", "on", "1"):
        hits.append(("W", "manual-only", "disable-model-invocation=true: Claude Code 不把 description 放进上下文, "
                                         "description 里的 Use when 触发词在 Claude Code 里不起作用"))

    lines = body.count("\n") + 1
    hits.append(("W" if lines > 500 else "I", "body-lines", f"正文 {lines} 行 (建议 < 500)"))

    for m in sorted(set(ABS_RE.findall(body))):
        hits.append(("W", "abs-path", f"正文写死绝对路径 {m}: 换一台机器就失效"))

    # 引用深度: SKILL.md 直接引用的文件算第 1 层, 它们再引用的算第 2 层
    lvl1 = {r for r in refs_in(f) if (d / r).exists()}
    lvl2 = set()
    for r in lvl1:
        if r.endswith(".md"):
            base = (d / r).parent
            for r2 in refs_in(d / r):
                for cand in (base / r2, d / r2):
                    if cand.exists() and cand.resolve() != f.resolve():
                        rel = str(cand.resolve().relative_to(d.resolve()))
                        if rel not in lvl1:
                            lvl2.add(rel)
                        break
    hits.append(("I", "ref-depth1", f"SKILL.md 直接引用 {len(lvl1)} 个存在的文件"))
    if lvl2:
        hits.append(("W", "ref-depth2", f"{len(lvl2)} 个文件只能经第二跳到达 (建议引用只深一层), 例: {sorted(lvl2)[:3]}"))
    return hits


def main(argv):
    worst = 0
    for p in argv[1:] or ["."]:
        print(f"== {p}")
        for level, rule, msg in lint(p):
            print(f"  [{level}] {rule}: {msg}")
            worst = max(worst, {"I": 0, "W": 0, "E": 1}[level])
    return worst


if __name__ == "__main__":
    sys.exit(main(sys.argv))
