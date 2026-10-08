"""注册表读写: 两层结构 (curated 由人类维护, generated 由脚本重建)。"""
from __future__ import annotations

import re
import sys
import tomllib
from pathlib import Path

from taxonomy.tag_cluster import GEN_BEGIN, GEN_END, build_generated_block
from taxonomy.tag_notes import display_tag, normalize_tag
from taxonomy.tag_notes import Note, note_files, read_frontmatter_tags

REGISTRY_FILENAME = ".hx-tags.toml"



class Registry:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.text = path.read_text(encoding="utf-8") if path.is_file() else ""
        try:
            self.data = tomllib.loads(self.text) if self.text else {}
        except tomllib.TOMLDecodeError as error:
            print(f"错误: 注册表不是合法 TOML: {error}", file=sys.stderr)
            raise SystemExit(2)
        self.ignore: list[str] = [str(item) for item in
                                  ((self.data.get("settings") or {}).get("ignore_notes") or [])]
        self.canonical: dict[str, str] = {}
        self.aliases: dict[str, str] = {}
        self.descriptions: dict[str, str] = {}
        self.parents: dict[str, str] = {}
        for name, entry in (self.data.get("tags") or {}).items():
            display = display_tag(name)
            self.canonical[normalize_tag(display)] = display
            if isinstance(entry, dict):
                if entry.get("desc"):
                    self.descriptions[display] = str(entry["desc"])
                if entry.get("parent"):
                    self.parents[display] = str(entry["parent"])
                for alias in entry.get("aliases") or []:
                    self.aliases[normalize_tag(alias)] = display
        conflicts = [alias for alias in self.aliases if alias in self.canonical]
        self.conflicts = conflicts

    def resolve(self, tag: str) -> tuple[str, str]:
        """返回 (规范化后的规范名, 判定): 'canonical' | 'alias' | 'unknown'。"""
        key = normalize_tag(tag)
        if key in self.canonical:
            return self.canonical[key], "canonical"
        if key in self.aliases:
            return self.aliases[key], "alias"
        return display_tag(tag), "unknown"

    @property
    def digest(self) -> str:
        return str((self.data.get("generated") or {}).get("source_digest") or "")


def write_generated(registry: Registry, notes: list[Note]) -> None:
    block = build_generated_block(notes)
    text = registry.text
    pattern = re.compile(re.escape(GEN_BEGIN) + r".*?" + re.escape(GEN_END) + r" <<<", re.DOTALL)
    if pattern.search(text):
        text = pattern.sub(lambda _: block, text)
    else:
        # 兼容: 存在不带 marker 的 [generated] 表时整段替换, 避免追加出重复表而弄坏 TOML
        bare = re.search(r"^\[generated\][ \t]*(?:#.*)?$", text, re.MULTILINE)
        if bare:
            rest = text[bare.end():]
            stop = re.search(r"^(?=\[)(?!\[generated)", rest, re.MULTILINE)
            end = bare.end() + (stop.start() if stop else len(rest))
            text = text[:bare.start()] + block + "\n" + text[end:]
        else:
            text = text.rstrip("\n") + "\n\n" + block + "\n"
    registry.path.write_text(text, encoding="utf-8")
    registry.text = text
    registry.data = tomllib.loads(text)


def registry_path(args) -> Path:
    return Path(args.registry) if args.registry else Path(args.docs_dir) / REGISTRY_FILENAME


def load(args) -> tuple[Registry, list[Note]]:
    path = registry_path(args)
    if not path.is_file():
        print(f"错误: 注册表不存在: {path}\n      先运行: hxloli_tags.py init", file=sys.stderr)
        raise SystemExit(2)
    registry = Registry(path)
    docs_dir = Path(args.docs_dir)
    notes = [Note(path, read_frontmatter_tags(path.read_text(encoding="utf-8")))
             for path in note_files(docs_dir, registry.ignore)]
    return registry, notes
