#!/usr/bin/env python3

from pathlib import Path
import sys

AUTHOR = "Heng_Xin"
EMAIL = "hxloli@qq.com"
VERSION = "0.0.1"


def findCommandsRoot() -> Path:
    current = Path.cwd().resolve()

    while True:
        candidate = current / ".agents" / "commands"

        if candidate.exists():
            return candidate

        if current.parent == current:
            raise RuntimeError(
                "Cannot find '.agents/commands' from current path"
            )

        current = current.parent


COMMAND_TEMPLATE = """---
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


def createCommand(commandName: str) -> None:
    commandsRoot = findCommandsRoot()

    commandRoot = commandsRoot / commandName

    commandRoot.mkdir(parents=True, exist_ok=False)

    (commandRoot / "docs").mkdir()
    (commandRoot / "scripts").mkdir()
    (commandRoot / "templates").mkdir()

    (commandRoot / "COMMAND.md").write_text(
        COMMAND_TEMPLATE.format(
            name=commandName,
            version=VERSION,
            author=AUTHOR,
            email=EMAIL,
        ),
        encoding="utf-8",
    )

    (commandRoot / "README.md").write_text(
        README_TEMPLATE.format(
            name=commandName,
        ),
        encoding="utf-8",
    )

    print(f"已创建 command 于: {commandRoot}")


def main() -> int:
    if len(sys.argv) != 2:
        print(
            f"Usage: {Path(sys.argv[0]).name} <command-name>"
        )
        return 1
    createCommand(sys.argv[1])
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
