#!/usr/bin/env python3
"""
音乐播放列表生成脚本

扫描 static/music/ 目录下的音频文件, 提取元数据, 自动生成 src/config/musicData.ts

使用方法:
    cd py && uv run python music/gen_music_playlist.py

依赖 (由 uv 管理):
    mutagen, requests

支持的音频格式: mp3, flac, ogg, m4a, wav, opus
"""

import os
import re
import sys
import json
import hashlib
from pathlib import Path

try:
    from mutagen import File as MutagenFile
    from mutagen.mp3 import MP3
    from mutagen.flac import FLAC
    from mutagen.oggvorbis import OggVorbis
    from mutagen.mp4 import MP4
    from mutagen.wave import WAVE
    from mutagen.oggopus import OggOpus
    from mutagen.id3 import ID3
    HAS_MUTAGEN = True
except ImportError:
    HAS_MUTAGEN = False
    print("[警告] 未安装 mutagen, 将使用文件名作为元数据")
    print("  安装: cd py && uv sync")
    print()

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False

import struct
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

# 项目根目录 (py/music/gen_music_playlist.py -> py/music -> py -> 项目根)
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
STATIC_MUSIC_DIR = PROJECT_ROOT / "static" / "music"
FONTS_DIR = STATIC_MUSIC_DIR / "fonts"
OUTPUT_FILE = PROJECT_ROOT / "src" / "config" / "musicData.ts"

# 支持的音频文件扩展名
AUDIO_EXTENSIONS = {'.mp3', '.flac', '.ogg', '.m4a', '.wav', '.opus'}
# ASS 字幕扩展名
ASS_EXTENSIONS = {'.ass', '.ssa'}
# 封面图片扩展名
COVER_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.webp', '.gif'}
# 字体扩展名
FONT_EXTENSIONS = {'.ttf', '.otf', '.woff', '.woff2'}

# baseUrl (和 docusaurus.config.ts 中保持一致)
BASE_URL = "/HXLoLi"

# CJK Fallback 字体下载 URL (Noto Sans SC - 支持简体中文、日文假名和常用汉字)
CJK_FONT_NAME = "NotoSansSC-Regular.ttf"
CJK_FONT_URL = (
    "https://github.com/google/fonts/raw/main/ofl/notosanssc/NotoSansSC%5Bwght%5D.ttf"
)

# 封面图片的 MIME -> 扩展名映射
MIME_TO_EXT = {
    'image/jpeg': '.jpg',
    'image/png': '.png',
    'image/webp': '.webp',
    'image/gif': '.gif',
}


def get_file_id(filepath: Path) -> str:
    """生成文件唯一 ID (基于相对路径的 hash)"""
    rel = filepath.relative_to(STATIC_MUSIC_DIR)
    return hashlib.md5(str(rel).encode()).hexdigest()[:12]


def extract_metadata(filepath: Path) -> dict:
    """从音频文件中提取元数据 (标题、艺术家、时长)"""
    title = filepath.stem  # 默认使用文件名
    artist = "Unknown"
    duration = 0

    if not HAS_MUTAGEN:
        return {"title": title, "artist": artist, "duration": duration}

    try:
        audio = MutagenFile(str(filepath))
        if audio is None:
            return {"title": title, "artist": artist, "duration": duration}

        # 获取时长
        if hasattr(audio, 'info') and hasattr(audio.info, 'length'):
            duration = round(audio.info.length, 2)

        # 根据不同格式提取标签
        if isinstance(audio, MP3):
            # ID3 tags
            if audio.tags:
                title = str(audio.tags.get('TIT2', title))
                artist = str(audio.tags.get('TPE1', artist))
        elif isinstance(audio, FLAC):
            title = audio.get('title', [title])[0]
            artist = audio.get('artist', [artist])[0]
        elif isinstance(audio, (OggVorbis, OggOpus)):
            title = audio.get('title', [title])[0]
            artist = audio.get('artist', [artist])[0]
        elif isinstance(audio, MP4):
            # iTunes-style tags
            title = audio.tags.get('\xa9nam', [title])[0] if audio.tags else title
            artist = audio.tags.get('\xa9ART', [artist])[0] if audio.tags else artist
        elif isinstance(audio, WAVE):
            # WAV 文件一般没有标签
            pass
        else:
            # 尝试通用方式
            tags = getattr(audio, 'tags', None)
            if tags:
                if hasattr(tags, 'get'):
                    title = tags.get('title', [title])[0] if isinstance(tags.get('title', title), list) else tags.get('title', title)
                    artist = tags.get('artist', [artist])[0] if isinstance(tags.get('artist', artist), list) else tags.get('artist', artist)

    except Exception as e:
        print(f"  [警告] 无法读取 {filepath.name} 的元数据: {e}")

    return {"title": str(title), "artist": str(artist), "duration": duration}


def extract_cover_from_audio(audio_path: Path) -> Path | None:
    """从音频文件中提取嵌入的封面图片，保存到同目录下

    如果同名封面已经存在则跳过。
    返回封面文件路径，无封面则返回 None。
    """
    if not HAS_MUTAGEN:
        return None

    stem = audio_path.stem
    parent = audio_path.parent

    # 如果同名封面已存在，跳过提取
    for ext in COVER_EXTENSIONS:
        if (parent / (stem + ext)).exists():
            return parent / (stem + ext)

    try:
        audio = MutagenFile(str(audio_path))
        if audio is None:
            return None

        img_data = None
        img_mime = None

        if isinstance(audio, MP3):
            # ID3 APIC frames
            if audio.tags:
                for key in audio.tags:
                    if key.startswith('APIC'):
                        apic = audio.tags[key]
                        img_data = apic.data
                        img_mime = apic.mime
                        break

        elif isinstance(audio, FLAC):
            # FLAC pictures
            if audio.pictures:
                pic = audio.pictures[0]
                img_data = pic.data
                img_mime = pic.mime

        elif isinstance(audio, MP4):
            # MP4/M4A covr atom
            if audio.tags and 'covr' in audio.tags:
                covers = audio.tags['covr']
                if covers:
                    cover = covers[0]
                    img_data = bytes(cover)
                    # MP4Cover.FORMAT_JPEG = 13, FORMAT_PNG = 14
                    if hasattr(cover, 'imageformat'):
                        img_mime = 'image/png' if cover.imageformat == 14 else 'image/jpeg'
                    else:
                        img_mime = 'image/jpeg'

        elif isinstance(audio, (OggVorbis, OggOpus)):
            # Ogg 格式嵌入封面 (metadata_block_picture)
            import base64
            from mutagen.flac import Picture
            pics = audio.get('metadata_block_picture', [])
            if pics:
                try:
                    pic = Picture(base64.b64decode(pics[0]))
                    img_data = pic.data
                    img_mime = pic.mime
                except Exception:
                    pass

        if img_data and img_mime:
            ext = MIME_TO_EXT.get(img_mime, '.jpg')
            cover_path = parent / (stem + ext)
            cover_path.write_bytes(img_data)
            print(f"    └─ 提取封面: {cover_path.name} ({len(img_data)} bytes)")
            return cover_path

    except Exception as e:
        print(f"  [警告] 提取封面失败 {audio_path.name}: {e}")

    return None


def extract_ass_fonts(ass_path: Path) -> list[str]:
    """从 ASS 文件中解析 Style 行使用的字体名列表

    返回去重后的字体名列表 (保留原始大小写)
    """
    fonts = set()
    try:
        # ASS 文件可能用不同编码
        for encoding in ('utf-8-sig', 'utf-8', 'gbk', 'shift_jis', 'latin-1'):
            try:
                text = ass_path.read_text(encoding=encoding)
                break
            except (UnicodeDecodeError, UnicodeError):
                continue
        else:
            return []

        # 匹配 Style 行，提取 Fontname 字段 (Style 格式的第 2 个字段)
        # Format: Name, Fontname, Fontsize, ...
        for line in text.splitlines():
            line = line.strip()
            if line.startswith('Style:'):
                # Style: name,fontname,fontsize,...
                parts = line[6:].split(',')
                if len(parts) >= 2:
                    font_name = parts[1].strip()
                    if font_name and font_name != 'Arial':  # Arial 已有 fallback
                        fonts.add(font_name)

    except Exception as e:
        print(f"  [警告] 解析 ASS 字体失败 {ass_path.name}: {e}")

    return sorted(fonts)


def ensure_cjk_fallback_font() -> bool:
    """确保 CJK fallback 字体文件存在

    如果不存在，尝试从 GitHub 下载 Noto Sans SC。
    返回字体是否可用。
    """
    font_path = FONTS_DIR / CJK_FONT_NAME
    if font_path.exists():
        print(f"[信息] CJK 字体已存在: {font_path.name}")
        return True

    if not HAS_REQUESTS:
        print(f"[警告] CJK 字体不存在且无法下载 (缺少 requests 库)")
        print(f"  请手动下载 Noto Sans SC 到: {font_path}")
        return False

    print(f"[信息] 下载 CJK 字体: {CJK_FONT_NAME} ...")
    FONTS_DIR.mkdir(parents=True, exist_ok=True)

    try:
        resp = requests.get(CJK_FONT_URL, stream=True, timeout=60)
        resp.raise_for_status()
        total = int(resp.headers.get('content-length', 0))
        downloaded = 0
        with open(font_path, 'wb') as f:
            for chunk in resp.iter_content(chunk_size=8192):
                f.write(chunk)
                downloaded += len(chunk)
                if total > 0:
                    pct = downloaded * 100 // total
                    print(f"\r  下载进度: {pct}% ({downloaded}/{total})", end='', flush=True)
        print()
        print(f"[完成] CJK 字体已下载: {font_path.name} ({font_path.stat().st_size} bytes)")
        return True
    except Exception as e:
        print(f"\n[错误] 下载 CJK 字体失败: {e}")
        print(f"  请手动下载到: {font_path}")
        print(f"  URL: {CJK_FONT_URL}")
        if font_path.exists():
            font_path.unlink()
        return False


def find_matching_ass(audio_path: Path) -> Path | None:
    """查找与音频文件同名的 ASS 歌词文件"""
    stem = audio_path.stem
    parent = audio_path.parent
    for ext in ASS_EXTENSIONS:
        ass_path = parent / (stem + ext)
        if ass_path.exists():
            return ass_path
    return None


def find_matching_cover(audio_path: Path) -> Path | None:
    """查找与音频文件同名的封面图片"""
    stem = audio_path.stem
    parent = audio_path.parent
    for ext in COVER_EXTENSIONS:
        cover_path = parent / (stem + ext)
        if cover_path.exists():
            return cover_path
    # 也查找目录中的 cover.* / folder.*
    for name_prefix in ['cover', 'folder', 'albumart']:
        for ext in COVER_EXTENSIONS:
            cover_path = parent / (name_prefix + ext)
            if cover_path.exists():
                return cover_path
    return None


def find_fonts_in_dir(directory: Path) -> list[Path]:
    """查找目录及 fonts/ 子目录下的字体文件"""
    fonts = []
    # 检查 fonts/ 子目录
    fonts_dir = directory / "fonts"
    if fonts_dir.exists():
        for f in fonts_dir.iterdir():
            if f.suffix.lower() in FONT_EXTENSIONS:
                fonts.append(f)
    # 检查当前目录下的字体
    for f in directory.iterdir():
        if f.suffix.lower() in FONT_EXTENSIONS:
            fonts.append(f)
    return sorted(set(fonts))


def path_to_url(filepath: Path) -> str:
    """将文件路径转换为 URL"""
    rel = filepath.relative_to(STATIC_MUSIC_DIR)
    return f"{BASE_URL}/music/{rel.as_posix()}"


# ========== ASS 预扫描 (用 ffmpeg + libass 计算边界框) ==========
# 对应 C++ 版 preprocessLyricBoundingBoxes: 固定 1920x1080 画布, 采样帧扫描像素

# 预扫描分辨率 (与 C++ 版 _assParse.setFrameSize(1920, 1080) 一致)
PRESCAN_WIDTH = 1920
PRESCAN_HEIGHT = 1080
PRESCAN_FPS = 2  # 每秒采样帧数


def _scan_frame_rgba(data: bytes, width: int, height: int) -> dict:
    """扫描单帧 RGBA 原始数据, 返回该帧的 TwoBlockBounds.

    上下分区: y < midLine 归上区块, y >= midLine 归下区块.
    像素 RGB 非黑 视为有内容 (ffmpeg 渲染在黑底上, alpha 恒为 255).
    """
    mid_line = height >> 1
    step = 2  # 隔行扫描加速
    top_y_min = float('inf')
    top_y_max = 0
    btm_y_min = float('inf')
    btm_y_max = 0
    left = float('inf')
    right = 0

    row_bytes = width * 4  # RGBA, 每像素4字节
    for y in range(0, height, step):
        row_start = y * row_bytes
        for x in range(0, width, step):
            px = row_start + x * 4
            # 检查 RGB 任一通道非零 (ffmpeg 黑底上的字幕)
            if data[px] > 0 or data[px + 1] > 0 or data[px + 2] > 0:
                if x < left:
                    left = x
                if x + step > right:
                    right = x + step
                if y < mid_line:
                    if y < top_y_min:
                        top_y_min = y
                    if y + step > top_y_max:
                        top_y_max = y + step
                else:
                    if y < btm_y_min:
                        btm_y_min = y
                    if y + step > btm_y_max:
                        btm_y_max = y + step

    # 裁剪到画布范围
    if right > width:
        right = width
    if top_y_max > height:
        top_y_max = height
    if btm_y_max > height:
        btm_y_max = height

    return {
        'topYMin': top_y_min if top_y_min != float('inf') else 0,
        'topYMax': top_y_max,
        'btmYMin': btm_y_min if btm_y_min != float('inf') else 0,
        'btmYMax': btm_y_max,
        'left': left if left != float('inf') else 0,
        'right': right,
    }


def _merge_bounds(a: dict, b: dict) -> dict:
    """合并两个 bounds (取并集)"""
    def safe_min(x, y):
        if x == 0 and y == 0:
            return 0
        if x == 0:
            return y
        if y == 0:
            return x
        return min(x, y)

    return {
        'topYMin': safe_min(a['topYMin'], b['topYMin']),
        'topYMax': max(a['topYMax'], b['topYMax']),
        'btmYMin': safe_min(a['btmYMin'], b['btmYMin']),
        'btmYMax': max(a['btmYMax'], b['btmYMax']),
        'left': safe_min(a['left'], b['left']),
        'right': max(a['right'], b['right']),
    }


def prescan_ass_bounds(ass_path: Path, duration_sec: float) -> dict | None:
    """用 ffmpeg 渲染 ASS 字幕并扫描像素, 计算全局 TwoBlockBounds.

    类似 C++ 版 preprocessLyricBoundingBoxes:
    - 固定 1920x1080 画布
    - 以 2fps 采样整首歌
    - 每帧扫描像素, 合并得到全局边界框

    Args:
        ass_path: ASS 字幕文件路径
        duration_sec: 歌曲总时长 (秒)

    Returns:
        TwoBlockBounds dict, 如果没有内容则返回 None
    """
    if duration_sec <= 0:
        print(f"    └─ [预扫描] 时长为 0, 跳过")
        return None

    # 检查 ffmpeg 是否可用
    try:
        subprocess.run(['ffmpeg', '-version'], capture_output=True, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        print(f"    └─ [预扫描] ffmpeg 不可用, 跳过")
        return None

    w, h = PRESCAN_WIDTH, PRESCAN_HEIGHT
    frame_size = w * h * 4  # RGBA 每帧字节数

    # ffmpeg 命令: 黑底 + ASS 字幕渲染, 输出 rawvideo RGBA
    # 注意: subtitles 滤镜需要对特殊字符转义
    ass_path_escaped = str(ass_path).replace('\\', '/').replace(':', '\\:').replace("'", "\\'")
    cmd = [
        'ffmpeg',
        '-f', 'lavfi',
        '-i', f'color=c=black:s={w}x{h}:d={duration_sec:.2f}:r={PRESCAN_FPS}',
        '-vf', f"subtitles=filename='{ass_path_escaped}'",
        '-f', 'rawvideo',
        '-pix_fmt', 'rgba',
        '-v', 'error',
        '-',
    ]

    try:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
    except Exception as e:
        print(f"    └─ [预扫描] ffmpeg 启动失败: {e}")
        return None

    global_bounds = {
        'topYMin': 0, 'topYMax': 0,
        'btmYMin': 0, 'btmYMax': 0,
        'left': 0, 'right': 0,
    }
    frame_count = 0
    has_content = False

    try:
        while True:
            data = proc.stdout.read(frame_size)
            if len(data) < frame_size:
                break
            frame_count += 1
            fb = _scan_frame_rgba(data, w, h)
            # 只有有内容才合并
            if fb['topYMax'] > 0 or fb['btmYMax'] > 0 or fb['right'] > 0:
                if not has_content:
                    global_bounds = fb
                    has_content = True
                else:
                    global_bounds = _merge_bounds(global_bounds, fb)
    finally:
        proc.stdout.close()
        proc.wait()

    stderr_output = proc.stderr.read().decode(errors='replace').strip()
    proc.stderr.close()
    if stderr_output:
        print(f"    └─ [预扫描] ffmpeg stderr: {stderr_output[:200]}")

    if not has_content:
        print(f"    └─ [预扫描] {frame_count} 帧, 无字幕内容")
        return None

    print(f"    └─ [预扫描] {frame_count} 帧, bounds: "
          f"top=[{global_bounds['topYMin']},{global_bounds['topYMax']}] "
          f"btm=[{global_bounds['btmYMin']},{global_bounds['btmYMax']}] "
          f"lr=[{global_bounds['left']},{global_bounds['right']}]")
    return global_bounds


def scan_music_dir() -> list[dict]:
    """扫描 static/music/ 目录, 收集所有音频文件信息"""
    if not STATIC_MUSIC_DIR.exists():
        print(f"[错误] 音乐目录不存在: {STATIC_MUSIC_DIR}")
        sys.exit(1)

    tracks = []
    # 递归扫描所有音频文件
    audio_files = sorted([
        f for f in STATIC_MUSIC_DIR.rglob('*')
        if f.suffix.lower() in AUDIO_EXTENSIONS and f.is_file()
    ])

    if not audio_files:
        print("[信息] 没有找到音频文件")
        return tracks

    print(f"[信息] 找到 {len(audio_files)} 个音频文件")

    # 收集全局字体
    global_fonts = find_fonts_in_dir(STATIC_MUSIC_DIR)

    for audio_path in audio_files:
        print(f"  处理: {audio_path.relative_to(STATIC_MUSIC_DIR)}")

        # 提取元数据
        meta = extract_metadata(audio_path)

        # 查找关联的 ASS 歌词
        ass_path = find_matching_ass(audio_path)

        # 尝试从音频文件中提取封面 (如果本地没有)
        cover_path = extract_cover_from_audio(audio_path)

        # 如果提取失败，查找已有封面文件
        if not cover_path:
            cover_path = find_matching_cover(audio_path)

        # 查找目录级字体 + 全局字体
        local_fonts = find_fonts_in_dir(audio_path.parent)
        all_fonts = sorted(set(global_fonts + local_fonts))

        track = {
            "id": get_file_id(audio_path),
            "title": meta["title"],
            "artist": meta["artist"],
            "audioUrl": path_to_url(audio_path),
        }

        if ass_path:
            track["assUrl"] = path_to_url(ass_path)
            print(f"    └─ 歌词: {ass_path.name}")

            # 解析 ASS 中使用的字体名
            ass_fonts = extract_ass_fonts(ass_path)
            if ass_fonts:
                track["assFonts"] = ass_fonts
                print(f"    └─ ASS 字体: {', '.join(ass_fonts)}")

            # 用 ffmpeg 预扫描 ASS 字幕边界框
            duration = meta.get("duration", 0)
            if duration > 0:
                bounds = prescan_ass_bounds(ass_path, duration)
                if bounds:
                    track["assBounds"] = bounds

        if cover_path:
            track["coverUrl"] = path_to_url(cover_path)
            print(f"    └─ 封面: {cover_path.name}")

        if all_fonts:
            track["fonts"] = [path_to_url(f) for f in all_fonts]
            print(f"    └─ 字体文件: {len(all_fonts)} 个")

        tracks.append(track)

    return tracks


def generate_ts(tracks: list[dict]) -> str:
    """生成 TypeScript 文件内容"""
    lines = [
        '/**',
        ' * 音乐播放列表数据配置',
        ' *',
        ' * ⚠️ 此文件由 py/music/gen_music_playlist.py 自动生成, 请勿手动修改!',
        ' *',
        ' * 使用方式:',
        ' * 1. 将音频文件放在 static/music/ 目录下',
        ' * 2. 将同名 ASS 歌词文件放在同目录下 (可选)',
        ' * 3. 将同名封面图片放在同目录下 (可选)',
        ' * 4. 将字体文件放在 static/music/fonts/ 或同目录下 (可选)',
        ' * 5. 运行: cd py && uv run python music/gen_music_playlist.py',
        ' */',
        '',
        '/** 单首歌曲的信息 */',
        'export interface MusicTrack {',
        '    /** 歌曲唯一ID */',
        '    id: string;',
        '    /** 歌曲标题 */',
        '    title: string;',
        '    /** 歌手/艺术家 */',
        '    artist: string;',
        '    /** 音频文件URL (相对于 baseUrl) */',
        '    audioUrl: string;',
        '    /** ASS 歌词文件URL (相对于 baseUrl, 可选) */',
        '    assUrl?: string;',
        '    /** 歌词所需的字体文件 URL 列表 (可选) */',
        '    fonts?: string[];',
        '    /** 封面图片 URL (可选) */',
        '    coverUrl?: string;',
        '    /** ASS 歌词中使用的字体名列表 (用于 fallback 映射, 可选) */',
        '    assFonts?: string[];',
        '    /** ASS 预扫描边界框 (由 Python 脚本预计算, 固定 1920x1080 画布) */',
        '    assBounds?: {',
        '        topYMin: number;',
        '        topYMax: number;',
        '        btmYMin: number;',
        '        btmYMax: number;',
        '        left: number;',
        '        right: number;',
        '    };',
        '}',
        '',
        '/** 播放列表 */',
        'export const playlist: MusicTrack[] = ',
    ]

    # JSON 序列化曲目列表, 然后格式化为 TS
    json_str = json.dumps(tracks, ensure_ascii=False, indent=4)
    lines.append(json_str + ';')

    return '\n'.join(lines) + '\n'


def main():
    print("=" * 50)
    print("🎵 音乐播放列表生成器")
    print("=" * 50)
    print()

    # 确保 CJK fallback 字体可用
    ensure_cjk_fallback_font()
    print()

    tracks = scan_music_dir()

    print()
    print(f"[信息] 共 {len(tracks)} 首歌曲")

    ts_content = generate_ts(tracks)

    # 确保输出目录存在
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)

    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write(ts_content)

    print(f"[完成] 已写入: {OUTPUT_FILE.relative_to(PROJECT_ROOT)}")
    print()

    # 打印摘要
    for i, t in enumerate(tracks, 1):
        ass_mark = "🎤" if "assUrl" in t else "  "
        cover_mark = "🖼️" if "coverUrl" in t else "  "
        font_mark = "🔤" if "assFonts" in t else "  "
        print(f"  {i:2d}. {ass_mark}{cover_mark}{font_mark} {t['title']} - {t['artist']}")


if __name__ == '__main__':
    main()
