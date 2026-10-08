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

def _read_codex_toml(config_path: Path) -> str:
    try:
        data = tomllib.loads(config_path.read_text(encoding="utf-8"))
    except Exception:
        return ""

    model = data.get("model")
    if not model:
        return ""

    return _format_model(str(model), data.get("model_reasoning_effort"))

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

def find_project_model_config() -> Path | None:
    current = Path.cwd().resolve()

    while True:
        candidate = current / ".agents" / "skills" / "hx-note 的步骤 2" / "config.toml"
        if candidate.is_file():
            return candidate

        if current.parent == current:
            return None

        current = current.parent

def get_project_model() -> str:
    env_model = get_env_model()
    if env_model:
        return env_model

    config_path = find_project_model_config()
    if config_path is None:
        return "Unknown"

    model = _read_codex_toml(config_path)
    return model or "Unknown"

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
