from __future__ import annotations

import json
import os
from pathlib import Path


def getCurrentModel() -> str:
    envKeys: tuple[str, ...] = (
        "ANTHROPIC_MODEL",
        "CLAUDE_MODEL",
        "MODEL",
    )

    for key in envKeys:
        value: str | None = os.getenv(key)
        if value:
            return value

    configPaths: tuple[Path, ...] = (
        Path.cwd() / ".claude" / "settings.json",
        Path.home() / ".claude" / "settings.json",
    )

    for configPath in configPaths:
        if not configPath.is_file():
            continue

        try:
            data: dict = json.loads(
                configPath.read_text(encoding="utf-8")
            )

            for key in envKeys:
                value = data.get(key)
                if value:
                    return str(value)

            env: dict = data.get("env", {})

            for key in envKeys:
                value = env.get(key)
                if value:
                    return str(value)
                
            data: dict = json.loads(
                configPath.read_text(encoding="utf-8")
            )

            model: str | None = data.get("model")
            if model:
                return str(model)

        except Exception:
            pass

    return "Unknown"

print(getCurrentModel())