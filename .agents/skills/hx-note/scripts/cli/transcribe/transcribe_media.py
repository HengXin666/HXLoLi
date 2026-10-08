"""下载、转码与 ASR: 每一步都是外部工具调用, 失败即抛。"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import urllib.request
from pathlib import Path

from transcribe.transcribe_core import (Provenance, is_url, run, trim)
from transcribe.transcribe_subs import (format_funasr_output, parse_ass,
                                        parse_srt_vtt, transcript_header)


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
