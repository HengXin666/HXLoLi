#!/usr/bin/env python3
"""Prepare transcript artifacts for the hx-look-video skill.

The script intentionally stops at transcript preparation. The agent reads the
generated transcript/provenance files and performs the final summary.
"""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import re
import shutil
import subprocess
import sys
import tempfile
import urllib.request
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


def main(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)
    source = args.input.strip()
    if not source:
        print("Input is empty.", file=sys.stderr)
        return 2

    input_type = classify_input(source)
    if input_type.startswith("local"):
        local_path = Path(source).expanduser()
        if not local_path.exists():
            print(f"Local path does not exist: {local_path}", file=sys.stderr)
            return 2

    work_dir = make_work_dir(source, args.work_dir)
    provenance = Provenance(
        input=source,
        input_type=input_type,
        created_at=datetime.now().isoformat(timespec="seconds"),
        work_dir=str(work_dir),
    )
    provenance.tools = detect_tools()

    transcript_path = work_dir / "transcript.md"
    metadata_path = work_dir / "metadata.json"
    provenance.transcript_path = str(transcript_path)
    provenance.metadata_path = str(metadata_path)

    try:
        if input_type in {"local-subtitle", "local-text"}:
            local = Path(source).expanduser()
            write_normalized_transcript(
                local,
                transcript_path,
                source_label=str(local),
                transcript_source="provided-transcript",
            )
            provenance.transcript_source = "provided-transcript"

        elif input_type == "url-subtitle":
            downloaded = download_direct_url(source, work_dir / source_filename(source, "subtitle"))
            write_normalized_transcript(
                downloaded,
                transcript_path,
                source_label=source,
                transcript_source="direct-subtitle-url",
            )
            provenance.transcript_source = "direct-subtitle-url"

        elif input_type == "url":
            handle_url(args, source, work_dir, transcript_path, metadata_path, provenance)

        elif input_type in {"local-video", "local-audio"}:
            handle_local_media(args, Path(source).expanduser(), work_dir, transcript_path, provenance)

        else:
            provenance.errors.append(f"Unsupported input type: {input_type}")
            write_provenance(work_dir, provenance)
            print(f"Unsupported input type: {input_type}", file=sys.stderr)
            return 2

        if not transcript_path.exists() or transcript_path.stat().st_size == 0:
            provenance.errors.append("Transcript was not generated.")
            write_provenance(work_dir, provenance)
            print("Transcript was not generated.", file=sys.stderr)
            print_paths(work_dir, transcript_path, work_dir / "provenance.json", metadata_path, provenance)
            return 1

        text = transcript_path.read_text(encoding="utf-8", errors="replace")
        if len(strip_markdown_meta(text)) < MIN_USEFUL_TRANSCRIPT_CHARS:
            provenance.warnings.append(
                f"Transcript is short ({len(strip_markdown_meta(text))} chars); summary may be thin."
            )
        if not metadata_path.exists():
            write_minimal_metadata(metadata_path, source, input_type)

    except KeyboardInterrupt:
        provenance.errors.append("Interrupted by user.")
        write_provenance(work_dir, provenance)
        raise
    except Exception as exc:
        provenance.errors.append(f"{type(exc).__name__}: {exc}")
        write_provenance(work_dir, provenance)
        print(f"Failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        print_paths(work_dir, transcript_path, work_dir / "provenance.json", metadata_path, provenance)
        return 1

    provenance_path = write_provenance(work_dir, provenance)
    print_paths(work_dir, transcript_path, provenance_path, metadata_path, provenance)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Prepare transcript artifacts from video/audio URL or local media.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("input", help="Video/audio URL, local media path, or transcript file path.")
    parser.add_argument("--work-dir", help="Base directory for generated artifacts.")
    parser.add_argument("--sub-langs", default=DEFAULT_SUB_LANGS, help="yt-dlp subtitle language list.")
    parser.add_argument("--force-asr", action="store_true", help="Ignore subtitles and force ASR.")
    parser.add_argument("--no-asr", action="store_true", help="Do not download/extract audio or run ASR.")
    parser.add_argument("--cookies", help="Netscape cookies.txt file for yt-dlp.")
    parser.add_argument("--cookies-from-browser", help="Browser cookies source for yt-dlp, e.g. chromium.")
    parser.add_argument("--funasr-model", default="paraformer-zh", help="FunASR model name.")
    return parser


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
    root = Path(base).expanduser() if base else Path(tempfile.gettempdir()) / "hx-look-video"
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


def handle_url(
    args: argparse.Namespace,
    source: str,
    work_dir: Path,
    transcript_path: Path,
    metadata_path: Path,
    provenance: Provenance,
) -> None:
    fetch_metadata(args, source, metadata_path, provenance)
    if not args.force_asr:
        subtitle = fetch_platform_subtitle(args, source, work_dir, provenance)
        if subtitle:
            write_normalized_transcript(
                subtitle,
                transcript_path,
                source_label=source,
                transcript_source="platform-subtitle",
            )
            provenance.transcript_source = "platform-subtitle"
            return

    if args.no_asr:
        raise RuntimeError("No platform subtitle was available and --no-asr was set.")

    media = download_url_audio(args, source, work_dir, provenance)
    provenance.media_path = str(media)
    audio = convert_to_wav(media, work_dir / "audio_16k_mono.wav")
    provenance.audio_path = str(audio)
    transcribe_with_funasr(audio, transcript_path, source, args.funasr_model)
    provenance.transcript_source = "funasr-asr"


def handle_local_media(
    args: argparse.Namespace,
    media: Path,
    work_dir: Path,
    transcript_path: Path,
    provenance: Provenance,
) -> None:
    if args.no_asr:
        raise RuntimeError("Input is local media and --no-asr was set.")
    provenance.media_path = str(media)
    audio = convert_to_wav(media, work_dir / "audio_16k_mono.wav")
    provenance.audio_path = str(audio)
    transcribe_with_funasr(audio, transcript_path, str(media), args.funasr_model)
    provenance.transcript_source = "funasr-asr"


def ytdlp_cmd(args: argparse.Namespace) -> list[str]:
    if shutil.which("yt-dlp"):
        cmd = ["yt-dlp"]
    elif shutil.which("uvx"):
        cmd = ["uvx", "yt-dlp"]
    else:
        raise RuntimeError("yt-dlp is required for URL input. Install yt-dlp or uv.")

    if args.cookies:
        cmd.extend(["--cookies", args.cookies])
    if args.cookies_from_browser:
        cmd.extend(["--cookies-from-browser", args.cookies_from_browser])
    return cmd


def fetch_metadata(
    args: argparse.Namespace,
    source: str,
    metadata_path: Path,
    provenance: Provenance,
) -> None:
    cmd = ytdlp_cmd(args) + ["--dump-single-json", "--no-playlist", source]
    result = run(cmd, check=False)
    if result.returncode != 0:
        provenance.warnings.append("yt-dlp metadata fetch failed; continuing without metadata.")
        metadata_path.write_text(
            json.dumps(
                {
                    "source_url": source,
                    "error": trim(result.stderr),
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        return
    try:
        data = json.loads(result.stdout)
    except json.JSONDecodeError:
        data = {"source_url": source, "raw": result.stdout[:4000]}
    metadata_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def write_minimal_metadata(metadata_path: Path, source: str, input_type: str) -> None:
    metadata_path.write_text(
        json.dumps(
            {
                "source": source,
                "input_type": input_type,
                "title": Path(source).expanduser().name if not is_url(source) else "",
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )


def fetch_platform_subtitle(
    args: argparse.Namespace,
    source: str,
    work_dir: Path,
    provenance: Provenance,
) -> Path | None:
    subs_dir = work_dir / "subtitles"
    subs_dir.mkdir(parents=True, exist_ok=True)
    output_template = str(subs_dir / "%(title).120s.%(id)s.%(ext)s")
    cmd = ytdlp_cmd(args) + [
        "--skip-download",
        "--no-playlist",
        "--write-subs",
        "--write-auto-subs",
        "--sub-langs",
        args.sub_langs,
        "--convert-subs",
        "srt",
        "-o",
        output_template,
        source,
    ]
    result = run(cmd, check=False)
    candidates = [
        p
        for p in subs_dir.rglob("*")
        if p.is_file() and p.suffix.lower() in {".srt", ".vtt", ".ass", ".ssa", ".txt"}
    ]
    if not candidates:
        provenance.warnings.append(
            "No platform subtitle found."
            + (f" yt-dlp stderr: {trim(result.stderr)}" if result.stderr else "")
        )
        return None
    candidates.sort(key=subtitle_score)
    return candidates[0]


def subtitle_score(path: Path) -> tuple[int, int, str]:
    name = path.name.lower()
    language_score = 50
    for idx, token in enumerate(("zh-hans", "zh-cn", ".zh.", "zh", "en")):
        if token in name:
            language_score = idx
            break
    ext_score = {".srt": 0, ".vtt": 1, ".ass": 2, ".ssa": 3, ".txt": 4}.get(path.suffix.lower(), 9)
    return (language_score, ext_score, name)


def download_url_audio(
    args: argparse.Namespace,
    source: str,
    work_dir: Path,
    provenance: Provenance,
) -> Path:
    output_template = str(work_dir / "source.%(ext)s")
    cmd = ytdlp_cmd(args) + [
        "--no-playlist",
        "-f",
        "ba/bestaudio/best",
        "-o",
        output_template,
        source,
    ]
    result = run(cmd, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"yt-dlp audio download failed: {trim(result.stderr)}")

    candidates = [p for p in work_dir.glob("source.*") if p.is_file()]
    if not candidates:
        raise RuntimeError("yt-dlp finished but no source media file was found.")
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0]


def download_direct_url(url: str, output_path: Path) -> Path:
    with urllib.request.urlopen(url, timeout=60) as response:
        data = response.read()
    output_path.write_bytes(data)
    return output_path


def convert_to_wav(input_path: Path, output_path: Path) -> Path:
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg is required for audio extraction/conversion.")
    cmd = [
        "ffmpeg",
        "-y",
        "-i",
        str(input_path),
        "-vn",
        "-acodec",
        "pcm_s16le",
        "-ar",
        "16000",
        "-ac",
        "1",
        str(output_path),
    ]
    result = run(cmd, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"ffmpeg conversion failed: {trim(result.stderr)}")
    return output_path


def transcribe_with_funasr(
    audio_path: Path,
    transcript_path: Path,
    source_label: str,
    model_name: str,
) -> None:
    try:
        from funasr import AutoModel
    except ImportError as exc:
        raise RuntimeError(
            "FunASR is required for ASR. Re-run with: "
            "uv run --with funasr --with modelscope --with torch --with torchaudio "
            f"python {Path(__file__).as_posix()} <input>"
        ) from exc

    print("Loading FunASR model; first run may download model files.", file=sys.stderr)
    model = AutoModel(
        model=model_name,
        vad_model="fsmn-vad",
        punc_model="ct-punc",
        disable_update=True,
    )
    result = model.generate(
        input=str(audio_path),
        batch_size_s=300,
        hotword="",
    )
    transcript_path.write_text(
        format_funasr_output(result, source_label, "funasr-asr"),
        encoding="utf-8",
    )


def write_normalized_transcript(
    input_path: Path,
    transcript_path: Path,
    source_label: str,
    transcript_source: str,
) -> None:
    ext = input_path.suffix.lower()
    if ext in {".srt", ".vtt"}:
        body = parse_srt_vtt(input_path.read_text(encoding="utf-8", errors="replace"))
    elif ext in {".ass", ".ssa"}:
        body = parse_ass(input_path.read_text(encoding="utf-8", errors="replace"))
    else:
        body = input_path.read_text(encoding="utf-8", errors="replace").strip()

    transcript_path.write_text(
        transcript_header(source_label, transcript_source) + "\n\n" + body.strip() + "\n",
        encoding="utf-8",
    )


def transcript_header(source_label: str, transcript_source: str) -> str:
    return "\n".join(
        [
            "# Transcript",
            "",
            f"- Source: {source_label}",
            f"- Transcript source: {transcript_source}",
            f"- Generated at: {datetime.now().isoformat(timespec='seconds')}",
        ]
    )


TIMECODE_RE = re.compile(
    r"(?P<start>(?:\d{1,2}:)?\d{2}:\d{2}[,.]\d{3})\s+-->\s+"
    r"(?P<end>(?:\d{1,2}:)?\d{2}:\d{2}[,.]\d{3})"
)


def parse_srt_vtt(body: str) -> str:
    cues: list[tuple[str, str, str]] = []
    current_start = ""
    current_end = ""
    current_lines: list[str] = []

    def flush() -> None:
        nonlocal current_start, current_end, current_lines
        if current_start and current_lines:
            text = clean_caption_text(" ".join(current_lines))
            if text and (not cues or cues[-1][2] != text):
                cues.append((current_start, current_end, text))
        current_start = ""
        current_end = ""
        current_lines = []

    for raw_line in body.splitlines():
        line = raw_line.strip()
        if not line or line.startswith(("WEBVTT", "NOTE")):
            continue
        match = TIMECODE_RE.search(line)
        if match:
            flush()
            current_start = normalize_timecode(match.group("start"))
            current_end = normalize_timecode(match.group("end"))
            continue
        if re.fullmatch(r"\d+", line):
            continue
        if current_start:
            cleaned = clean_caption_text(line)
            if cleaned:
                current_lines.append(cleaned)
    flush()

    if cues:
        return "\n\n".join(f"[{start} -> {end}]\n{text}" for start, end, text in cues)
    return clean_caption_text(body)


def parse_ass(body: str) -> str:
    cues: list[str] = []
    for raw_line in body.splitlines():
        if not raw_line.startswith("Dialogue:"):
            continue
        parts = raw_line.split(",", 9)
        if len(parts) < 10:
            continue
        start = normalize_ass_time(parts[1].strip())
        end = normalize_ass_time(parts[2].strip())
        text = clean_caption_text(parts[9].replace("\\N", " "))
        if text:
            cues.append(f"[{start} -> {end}]\n{text}")
    return "\n\n".join(cues)


def clean_caption_text(value: str) -> str:
    value = html.unescape(value)
    value = re.sub(r"<[^>]+>", "", value)
    value = re.sub(r"\{[^}]*\}", "", value)
    value = value.replace("\\h", " ")
    value = re.sub(r"\s+", " ", value)
    return value.strip()


def normalize_timecode(value: str) -> str:
    value = value.replace(",", ".")
    parts = value.split(":")
    if len(parts) == 2:
        hours = 0
        minutes = int(parts[0])
        seconds = float(parts[1])
    else:
        hours = int(parts[0])
        minutes = int(parts[1])
        seconds = float(parts[2])
    whole_seconds = int(seconds)
    millis = int(round((seconds - whole_seconds) * 1000))
    return f"{hours:02d}:{minutes:02d}:{whole_seconds:02d}.{millis:03d}"


def normalize_ass_time(value: str) -> str:
    parts = value.split(":")
    if len(parts) != 3:
        return value
    hours = int(parts[0])
    minutes = int(parts[1])
    seconds = float(parts[2])
    whole_seconds = int(seconds)
    millis = int(round((seconds - whole_seconds) * 1000))
    return f"{hours:02d}:{minutes:02d}:{whole_seconds:02d}.{millis:03d}"


def format_funasr_output(result: object, source_label: str, transcript_source: str) -> str:
    if isinstance(result, dict):
        items = [result]
    elif isinstance(result, list):
        items = [item for item in result if isinstance(item, dict)]
    else:
        items = []

    lines = [transcript_header(source_label, transcript_source), ""]
    for item in items:
        text = str(item.get("text", "")).strip()
        timestamp = item.get("timestamp")
        if not text:
            continue
        if isinstance(timestamp, list) and timestamp:
            lines.extend(format_timestamped_text(text, timestamp))
        else:
            lines.extend(split_plain_text(text))
    return "\n".join(lines).strip() + "\n"


def format_timestamped_text(text: str, timestamp: list[object]) -> list[str]:
    out: list[str] = []
    current_chars: list[str] = []
    current_start = timestamp[0][0] if is_span(timestamp[0]) else 0
    current_end = timestamp[0][1] if is_span(timestamp[0]) else 0

    for idx, char in enumerate(text):
        current_chars.append(char)
        if idx < len(timestamp) and is_span(timestamp[idx]):
            current_end = timestamp[idx][1]
        if char in "。！？；!?;\n":
            sentence = "".join(current_chars).strip()
            if sentence:
                out.append(f"[{format_ms(current_start)} -> {format_ms(current_end)}]")
                out.append(sentence)
                out.append("")
            current_chars = []
            if idx + 1 < len(timestamp) and is_span(timestamp[idx + 1]):
                current_start = timestamp[idx + 1][0]

    if current_chars:
        sentence = "".join(current_chars).strip()
        if sentence:
            out.append(f"[{format_ms(current_start)} -> {format_ms(current_end)}]")
            out.append(sentence)
            out.append("")
    return out


def is_span(value: object) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) >= 2
        and isinstance(value[0], (int, float))
        and isinstance(value[1], (int, float))
    )


def split_plain_text(text: str) -> list[str]:
    paragraphs: list[str] = []
    current: list[str] = []
    for char in text:
        current.append(char)
        if char in "。！？；!?;\n":
            paragraph = "".join(current).strip()
            if paragraph:
                paragraphs.extend([paragraph, ""])
            current = []
    if current:
        paragraph = "".join(current).strip()
        if paragraph:
            paragraphs.extend([paragraph, ""])
    return paragraphs


def format_ms(ms: float) -> str:
    total_ms = int(ms)
    seconds, millis = divmod(total_ms, 1000)
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}.{millis:03d}"


def strip_markdown_meta(text: str) -> str:
    lines = []
    for line in text.splitlines():
        if line.startswith("# Transcript") or line.startswith("- Source:") or line.startswith("- Transcript source:"):
            continue
        lines.append(line)
    return "\n".join(lines).strip()


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


def write_provenance(work_dir: Path, provenance: Provenance) -> Path:
    path = work_dir / "provenance.json"
    path.write_text(
        json.dumps(provenance.__dict__, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return path


def print_paths(
    work_dir: Path,
    transcript_path: Path,
    provenance_path: Path,
    metadata_path: Path,
    provenance: Provenance,
) -> None:
    print(f"WORK_DIR={work_dir}")
    print(f"TRANSCRIPT_PATH={transcript_path}")
    print(f"PROVENANCE_PATH={provenance_path}")
    print(f"METADATA_PATH={metadata_path}")
    print(f"TRANSCRIPT_SOURCE={provenance.transcript_source or '(none)'}")
    if provenance.warnings:
        print("WARNINGS=" + json.dumps(provenance.warnings, ensure_ascii=False))
    if provenance.errors:
        print("ERRORS=" + json.dumps(provenance.errors, ensure_ascii=False))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
