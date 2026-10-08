#!/usr/bin/env python3
"""Prepare transcript artifacts for the transcribe skill (entry point).

The script intentionally stops at transcript preparation. The agent reads the
generated transcript/provenance files and performs the final summary.
"""
from __future__ import annotations

import sys
from pathlib import Path

# scripts/ 与 scripts/cli/ 都上 path: 前者供 `lib.*`, 后者供各域包。
for _p in (Path(__file__).resolve().parents[1], Path(__file__).resolve().parents[2]):
    sys.path.insert(0, str(_p))

from transcribe.transcribe_cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
