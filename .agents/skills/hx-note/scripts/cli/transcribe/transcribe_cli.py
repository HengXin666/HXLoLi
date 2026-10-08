"""命令行入口: 参数解析与主流程。"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

from transcribe.transcribe_core import (DEFAULT_SUB_LANGS,
                                        MIN_USEFUL_TRANSCRIPT_CHARS, Provenance,
                                        classify_input, detect_tools, make_work_dir,
                                        source_filename)
from transcribe.transcribe_flow import (handle_local_media, handle_url, print_paths,
                                        write_provenance)
from transcribe.transcribe_media import (download_direct_url,
                                         write_minimal_metadata,
                                         write_normalized_transcript)
from transcribe.transcribe_subs import strip_markdown_meta


# Agent Notes: 流程型 skill 用「步骤 = 文件夹 = index.md + impl/」组织; 沉淀必须产出可复用物
# 
# 
def main(argv: list[str]) -> int:
    """Agent Notes
    .agents/notes/implemented/architecture/2026-09-26-skill-steps-as-template-method.md
    .agents/notes/implemented/architecture/2026-09-27-sediment-must-yield-reusable-artifacts.md
    """
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
