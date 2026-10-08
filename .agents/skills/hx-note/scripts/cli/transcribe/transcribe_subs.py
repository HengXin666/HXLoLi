"""字幕/转写文本的解析与归一 (SRT / VTT / ASS / 纯文本)。"""
from __future__ import annotations

import html
import re
from datetime import datetime

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
