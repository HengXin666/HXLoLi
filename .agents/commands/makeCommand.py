#!/usr/bin/env python3

from pathlib import Path
import sys

AUTHOR = "Heng_Xin"
EMAIL = "hxloli@qq.com"
VERSION = "0.0.1"


def findAgentsRoot() -> Path:
    current = Path.cwd().resolve()

    while True:
        candidate = current / ".agents"

        if candidate.is_dir():
            return candidate

        if current.parent == current:
            raise RuntimeError(
                "Cannot find '.agents' from current path"
            )

        current = current.parent


SKILL_TEMPLATE = """---
name: {name}
description: TODO: 描述这个 Codex skill 的能力和触发场景.
---

# {name}

## 目标

TODO

## 工作流

1. Analyze
2. Execute
3. Verify
"""


OPENAI_YAML_TEMPLATE = """interface:
  display_name: "{name}"
  short_description: "TODO: 补充 25-64 字简介"
  default_prompt: "Use ${name} to TODO."

policy:
  allow_implicit_invocation: true
"""


def createSkill(commandName: str) -> None:
    agentsRoot = findAgentsRoot()

    skillRoot = agentsRoot / "skills" / commandName

    skillRoot.mkdir(parents=True, exist_ok=False)

    (skillRoot / "agents").mkdir()
    (skillRoot / "docs").mkdir()
    (skillRoot / "scripts").mkdir()
    (skillRoot / "templates").mkdir()

    (skillRoot / "SKILL.md").write_text(
        SKILL_TEMPLATE.format(
            name=commandName,
        ),
        encoding="utf-8",
    )

    (skillRoot / "agents" / "openai.yaml").write_text(
        OPENAI_YAML_TEMPLATE.format(
            name=commandName,
        ),
        encoding="utf-8",
    )

    print(f"已创建 Codex skill 于: {skillRoot}")


def main() -> int:
    if len(sys.argv) != 2:
        print(
            f"Usage: {Path(sys.argv[0]).name} <skill-name>"
        )
        return 1
    createSkill(sys.argv[1])
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
