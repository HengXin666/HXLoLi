"""素材类型判定、路径工具与外部命令执行。"""
from __future__ import annotations

import hashlib
import re
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from urllib.parse import unquote, urlparse

VIDEO_EXTS = {".mp4", ".mkv", ".avi", ".mov", ".flv", ".wmv", ".webm", ".m4v"}
AUDIO_EXTS = {".mp3", ".m4a", ".wav", ".flac", ".ogg", ".aac", ".opus", ".wma"}
SUBTITLE_EXTS = {".srt", ".vtt", ".ass", ".ssa", ".txt", ".md"}
DEFAULT_SUB_LANGS = "zh-Hans,zh-CN,zh,en.*"
MIN_USEFUL_TRANSCRIPT_CHARS = 80


@dataclass
class Provenance:
    input: str
    input_type: str
    created_at: str
    work_dir: str
    transcript_source: str = ""
    transcript_path: str = ""
    metadata_path: str = ""
    media_path: str = ""
    audio_path: str = ""
    tools: dict[str, str] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)


def classify_input(source: str) -> str:
    if is_url(source):
        ext = suffix_from_url(source)
        if ext in SUBTITLE_EXTS:
            return "url-subtitle"
        return "url"

    path = Path(source).expanduser()
    ext = path.suffix.lower()
    if ext in {".txt", ".md"}:
        return "local-text"
    if ext in {".srt", ".vtt", ".ass", ".ssa"}:
        return "local-subtitle"
    if ext in VIDEO_EXTS:
        return "local-video"
    if ext in AUDIO_EXTS:
        return "local-audio"
    return "local-unknown"

def is_url(source: str) -> bool:
    parsed = urlparse(source)
    return parsed.scheme in {"http", "https"} and bool(parsed.netloc)

def suffix_from_url(url: str) -> str:
    return Path(unquote(urlparse(url).path)).suffix.lower()

def source_filename(url: str, fallback_stem: str) -> str:
    name = Path(unquote(urlparse(url).path)).name
    if name:
        return safe_filename(name)
    return f"{fallback_stem}.txt"

def make_work_dir(source: str, base: str | None) -> Path:
    root = Path(base).expanduser() if base else Path(tempfile.gettempdir()) / "transcribe"
    digest = hashlib.sha1(source.encode("utf-8", errors="ignore")).hexdigest()[:10]
    slug = safe_slug(source)[:48] or "source"
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    work_dir = root / f"{stamp}-{slug}-{digest}"
    work_dir.mkdir(parents=True, exist_ok=False)
    return work_dir

def safe_slug(value: str) -> str:
    if is_url(value):
        parsed = urlparse(value)
        value = f"{parsed.netloc}{parsed.path}"
    else:
        value = Path(value).name or value
    value = unquote(value)
    value = re.sub(r"[^\w.-]+", "-", value, flags=re.UNICODE).strip("-_.")
    return value or "source"

def safe_filename(value: str) -> str:
    return re.sub(r'[<>:"/\\|?*\x00-\x1f]+', "_", value).strip(" .") or "source"

def detect_tools() -> dict[str, str]:
    tools: dict[str, str] = {}
    for name in ("python", "ffmpeg", "yt-dlp", "uvx"):
        path = shutil.which(name)
        if path:
            tools[name] = path
    return tools

def run(cmd: list[str], check: bool) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if check and result.returncode != 0:
        raise RuntimeError(trim(result.stderr or result.stdout))
    return result

def trim(value: str, limit: int = 4000) -> str:
    value = value.strip()
    if len(value) <= limit:
        return value
    return value[-limit:]
