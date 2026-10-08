"""编排: 按输入类型选链路, 写 provenance, 汇总输出路径。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from transcribe.transcribe_core import Provenance
from transcribe.transcribe_media import (convert_to_wav, download_url_audio,
                                         fetch_metadata, fetch_platform_subtitle,
                                         transcribe_with_funasr,
                                         write_normalized_transcript)


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
