"""frontmatter 的值处理: 环境清理、模型名推断、YAML 渲染、tag/skill 归一。"""
from __future__ import annotations

import json
import os
import re
import tomllib
from pathlib import Path

DEFAULT_SKILL = "hx-note"
DEFAULT_AUTHOR = "Heng_Xin"
MODEL_ENV_KEYS: tuple[str, ...] = (
    "HX_AI_DOCS_MODEL",
    "CODEX_MODEL",
    "OPENAI_MODEL",
    "ANTHROPIC_MODEL",
    "CLAUDE_MODEL",
    "MODEL",
)

REASONING_EFFORT_ENV_KEYS: tuple[str, ...] = (
    "HX_AI_DOCS_MODEL_REASONING_EFFORT",
    "CODEX_MODEL_REASONING_EFFORT",
    "OPENAI_REASONING_EFFORT",
    "MODEL_REASONING_EFFORT",
)

ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")

CONTROL_CHAR_RE = re.compile(r"[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]")

LITERAL_SGR_RE = re.compile(r"\[(?:\d|;)+m\]?")

def clean_meta(value: str) -> str:
    value = ANSI_ESCAPE_RE.sub("", value)
    value = CONTROL_CHAR_RE.sub("", value)
    value = LITERAL_SGR_RE.sub("", value)
    return value.strip()

def _format_model(model: str, reasoning_effort: str | None = None) -> str:
    model = clean_meta(model)
    effort = clean_meta(reasoning_effort or "")

    if not model:
        return ""

    if not effort:
        return model

    suffix = f"-{effort}"
    if model.endswith(suffix):
        return model

    return f"{model}{suffix}"

def _first_env(keys: tuple[str, ...]) -> str:
    for key in keys:
        value = os.getenv(key)
        if value:
            return clean_meta(value)
    return ""

def get_env_model() -> str:
    model = _first_env(MODEL_ENV_KEYS)
    if not model:
        return ""

    return _format_model(model, _first_env(REASONING_EFFORT_ENV_KEYS))

MODEL_REGISTRY = Path(__file__).resolve().parents[3] / "shared" / "models.toml"
EFFORT_SUFFIXES = ("-low", "-medium", "-high", "-xhigh")


def load_model_registry(path: Path = MODEL_REGISTRY) -> list[dict]:
    """Agent Notes
    .agents/notes/implemented/process/2026-11-08-model-field-enum.md
    """
    try:
        return tomllib.loads(path.read_text(encoding="utf-8")).get("model", [])
    except (OSError, tomllib.TOMLDecodeError):
        return []

def normalize_model(raw: str, registry: list[dict] | None = None) -> tuple[str, bool] | None:
    """把任意写法归一成登记表里的 id, 返回 (取值, 是否 legacy). 表外取值返回 None"""
    reg = registry if registry is not None else load_model_registry()
    value = clean_meta(raw)
    effort = ""
    for suf in EFFORT_SUFFIXES:
        if value.lower().endswith(suf):
            value, effort = value[: -len(suf)], suf
            break
    for item in reg:
        names = [item["id"], *item.get("aliases", [])]
        if any(value.lower() == n.lower() for n in names):
            return item["id"] + effort, bool(item.get("legacy", False))
    return None

def get_project_model() -> str:
    """只认环境变量. 读不到就是 unknown, 由 doctor 拦下逼人显式传 --model

    以前会去翻 config.toml, 那正是 model-selection.md 禁止的「编一个看起来对的模型名」
    """
    env_model = get_env_model()
    if not env_model:
        return "unknown"
    hit = normalize_model(env_model)
    return hit[0] if hit else env_model

def strip_order_prefix(name: str) -> str:
    return clean_meta(re.sub(r"^\d+[-_]", "", name))

def infer_title(output: Path | None) -> str:
    if output is not None:
        if output.name in ("index.md", "index.mdx"):
            return strip_order_prefix(output.parent.name)
        return strip_order_prefix(output.stem)
    return strip_order_prefix(Path.cwd().name) or "未命名笔记"

def yaml_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)

def yaml_list(values: list[str]) -> str:
    if not values:
        return "[]"
    return "[" + ", ".join(yaml_string(value) for value in values) + "]"

def parse_tags(raw_tags: list[str]) -> list[str]:
    tags: list[str] = []
    seen: set[str] = set()

    for raw in raw_tags:
        for item in raw.split(","):
            tag = clean_meta(item)
            if tag and tag not in seen:
                seen.add(tag)
                tags.append(tag)

    return tags

def parse_skills(raw_skills: list[str]) -> list[str]:
    skills: list[str] = []
    seen: set[str] = set()

    for raw in raw_skills:
        for item in raw.split(","):
            skill = clean_meta(item)
            if skill and skill not in seen:
                seen.add(skill)
                skills.append(skill)

    return skills or [DEFAULT_SKILL]
