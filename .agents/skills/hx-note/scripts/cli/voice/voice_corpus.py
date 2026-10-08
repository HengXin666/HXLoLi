"""项目级表达语料库: 读取、宽松解析、追加 (learn / samples)。"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

VOICE_DB_CANDIDATES = ("ai-docs/.hx-voice.toml", ".hx-voice.toml")


def find_voice_db(start: Path | None = None) -> Path:
    cur = (start or Path.cwd()).resolve()
    while True:
        for rel in VOICE_DB_CANDIDATES:
            cand = cur / rel
            if cand.is_file():
                return cand
        if cur.parent == cur:
            break
        cur = cur.parent
    # 不存在时返回首选写入位置
    cur = (start or Path.cwd()).resolve()
    while True:
        if (cur / "ai-docs").is_dir():
            return cur / "ai-docs" / ".hx-voice.toml"
        if cur.parent == cur:
            return (start or Path.cwd()).resolve() / ".hx-voice.toml"
        cur = cur.parent

def _mini_toml_arrays(text: str) -> dict:
    """只认 [[bad]] / [[good]] 与其下的 key = "value"。

    存在的理由: 语料库是这个 skill 的记忆, 解析失败就等于把积累过的表达
    静默丢掉  比报错更坏。tomllib 要 Python 3.11, 而 `python3` 可能是 3.9。
    """
    out: dict[str, list[dict]] = {"bad": [], "good": []}
    cur: dict | None = None
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        m = re.match(r"^\[\[(bad|good)\]\]$", s)
        if m:
            cur = {}
            out[m.group(1)].append(cur)
            continue
        if cur is None:
            continue
        kv = re.match(r'^([A-Za-z_]+)\s*=\s*"(.*)"\s*$', s)
        if kv:
            cur[kv.group(1)] = kv.group(2).replace('\\"', '"').replace("\\n", "\n").replace("\\\\", "\\")
    return out

def load_voice_db(path: Path) -> dict:
    if not path.is_file():
        return {"bad": [], "good": [], "meme": []}
    raw = path.read_text(encoding="utf-8")
    try:
        import tomllib
        data = tomllib.loads(raw)
        return {"bad": data.get("bad", []) or [], "good": data.get("good", []) or [], "meme": data.get("meme", []) or []}
    except ImportError:
        return _mini_toml_arrays(raw)
    except Exception as exc:  # noqa: BLE001
        print(f"warn: 语料库 TOML 有语法错误 ({exc}), 退化为宽松解析", file=sys.stderr)
        return _mini_toml_arrays(raw)

def _toml_str(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'

def cmd_learn(args) -> int:
    path = Path(args.db) if args.db else find_voice_db()
    if getattr(args, "meme", None):
        kind, text = "meme", args.meme
    elif args.bad:
        kind, text = "bad", args.bad
    else:
        kind, text = "good", args.good
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.is_file():
        path.write_text(
            "# HXLoLi 表达语料库 (可增长)\n"
            "# 由 hx_voice.py learn 追加; 写作前读 `hx_voice.py samples`, 盲审后补 learn。\n"
            "# [[bad]] 带 pattern 时会被 lint 当成额外的 E 级规则。\n"
            "schema_version = 1\n",
            encoding="utf-8",
        )
    block = [f"\n[[{kind}]]", f"text = {_toml_str(text)}"]
    if args.why:
        block.append(f"why = {_toml_str(args.why)}")
    if args.pattern:
        try:
            re.compile(args.pattern)
        except re.error as exc:
            print(f"error: pattern 不是合法正则: {exc}", file=sys.stderr)
            return 2
        block.append(f"pattern = {_toml_str(args.pattern)}")
    if args.source:
        block.append(f"source = {_toml_str(args.source)}")
    with path.open("a", encoding="utf-8") as fh:
        fh.write("\n".join(block) + "\n")
    print(f"learned[{kind}] -> {path}")
    return 0

def cmd_samples(args) -> int:
    path = Path(args.db) if args.db else find_voice_db()
    db = load_voice_db(path)
    if args.json:
        print(json.dumps({"db": str(path), **db}, ensure_ascii=False, indent=2))
        return 0
    print(f"# 语料库: {path}")
    for kind, label in (
        ("good", "值得模仿"),
        ("bad", "必须避免"),
        ("meme", "梗 / 热词 (只用作者确认过的; 编错会变成新的 AI 味)"),
    ):
        items = db.get(kind, [])
        if not items:
            continue
        print(f"\n## {label} ({len(items)})")
        for it in items:
            why = f"   <- {it['why']}" if it.get("why") else ""
            print(f"- {it.get('text', '')}{why}")
    return 0
