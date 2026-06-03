#!/usr/bin/env python3

from pathlib import Path
import sys

AUTHOR = "Heng_Xin"
EMAIL = "hxloli@qq.com"
VERSION = "0.0.1"


def findSkillsRoot() -> Path:
    current = Path.cwd().resolve()

    while True:
        candidate = current / ".claude" / "skills"

        if candidate.exists():
            return candidate

        if current.parent == current:
            raise RuntimeError(
                "Cannot find '.claude/skills' from current path"
            )

        current = current.parent


SKILL_TEMPLATE = """---
name: {name}
description: 描述和触发词语
metadata:
    version: {version}
    author: {author}
    email: {email}
---

# {name}

## 目标

TODO

## 工作流

1. Analyze
2. Execute
3. Verify
"""


README_TEMPLATE = """# {name}

## 说明

TODO

## 使用方法

TODO
"""


def createSkill(skillName: str) -> None:
    skillsRoot = findSkillsRoot()

    skillRoot = skillsRoot / skillName

    skillRoot.mkdir(parents=True, exist_ok=False)

    (skillRoot / "docs").mkdir()
    (skillRoot / "scripts").mkdir()
    (skillRoot / "templates").mkdir()

    (skillRoot / "SKILL.md").write_text(
        SKILL_TEMPLATE.format(
            name=skillName,
            version=VERSION,
            author=AUTHOR,
            email=EMAIL,
        ),
        encoding="utf-8",
    )

    (skillRoot / "README.md").write_text(
        README_TEMPLATE.format(
            name=skillName,
        ),
        encoding="utf-8",
    )

    print(f"已创建 skill 于: {skillRoot}")


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