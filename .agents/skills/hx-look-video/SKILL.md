---
name: hx-look-video
description: 从视频、音频链接、本地媒体文件或字幕/transcript 文件中提取可引用 transcript, 并基于 transcript 生成可靠中文结构化总结. Use when the user asks to summarize, inspect, transcribe, or extract key points from video/audio URLs, local mp4/mp3/m4a/wav/mkv/webm files, or .srt/.vtt/.ass/.txt/.md transcript files.
---

# hx-look-video

## 目标

把用户给出的视频链接、音频链接、本地视频、本地音频或字幕文件, 转换为可引用的 transcript, 再基于 transcript 输出可靠的中文文本总结.

这个 skill 不是笔记沉淀入口. 默认只在对话中输出总结, 并把中间 transcript 放在临时工作目录. 只有用户明确要求保存到 `ai-docs`、`docs` 或指定文件时, 才落盘成正式文档.

## 输入

接受以下输入形态:

- 视频平台 URL: YouTube、Bilibili、X/Twitter、抖音、快手、Vimeo 等 `yt-dlp` 支持的平台.
- 直链媒体 URL: `.mp4`、`.webm`、`.mkv`、`.mp3`、`.m4a`、`.wav`、`.flac`、`.ogg` 等.
- 本地媒体文件: 本地 mp4/mp3/m4a/wav/mkv/webm 等.
- 字幕或转写文件: `.srt`、`.vtt`、`.ass`、`.ssa`、`.txt`、`.md`.

可理解的附加意图:

- `--force-asr`: 即使平台字幕存在, 也强制走语音识别.
- `--no-asr`: 只尝试字幕/已有 transcript, 不下载或转写音频.
- `--cookies FILE` 或 `--cookies-from-browser chromium`: 需要登录态或反爬 cookies 时传给 `yt-dlp`.
- `--sub-langs "zh-Hans,zh-CN,zh,en.*"`: 覆盖字幕语言优先级.
- `--work-dir PATH`: 指定中间产物目录.

如果用户没有给输入, 只问一个问题: "给我视频/音频链接或本地文件路径。"

## 处理原则

严格按证据链总结, 不凭标题、简介、评论或常识补内容.

Transcript 获取优先级:

1. 用户直接给出的字幕/转写文件.
2. 平台已有字幕或自动字幕, 通过 `yt-dlp --write-subs --write-auto-subs --skip-download` 获取.
3. URL 或本地媒体抽取音频, 用 FunASR 做本地语音转文本.
4. 如果语言明显不是中文且没有可用平台字幕, 优先使用可用的 Whisper 类转写能力; 若当前环境没有对应能力, 明确说明不能可靠转写, 不要假装 FunASR 适合所有语种.

FunASR 流程: `ffmpeg` 统一转成 16kHz 单声道 WAV, 再用 `AutoModel(model="paraformer-zh", vad_model="fsmn-vad", punc_model="ct-punc")` 生成带标点和时间戳的文本.

能拿到发布方或平台 transcript 时优先使用, 只有缺失时才做成本更高、误差更大的 ASR.

## 执行步骤

### 0. 解析输入和选项

从用户消息中提取第一个 URL 或本地路径作为 `INPUT`. 识别附加选项并原样传给准备脚本. 本地路径必须先确认存在; 如果不存在, 直接指出路径不存在并请用户给正确路径.

### 1. 准备 transcript

在 `HXLoLi` 仓库根目录运行:

```bash
python .agents/skills/hx-look-video/scripts/hx_look_video_prepare.py "<INPUT>"
```

如果需要 ASR 且当前 Python 环境没有 FunASR, 使用:

```bash
uv run --with funasr --with modelscope --with torch --with torchaudio \
  python .agents/skills/hx-look-video/scripts/hx_look_video_prepare.py "<INPUT>"
```

常用示例:

```bash
python .agents/skills/hx-look-video/scripts/hx_look_video_prepare.py \
  "https://www.bilibili.com/video/BV..." \
  --cookies-from-browser chromium
```

```bash
python .agents/skills/hx-look-video/scripts/hx_look_video_prepare.py \
  "/path/to/video.mp4" \
  --force-asr
```

脚本成功后会打印:

- `WORK_DIR`: 本次中间产物目录.
- `TRANSCRIPT_PATH`: 规范化 transcript.
- `PROVENANCE_PATH`: transcript 来源、工具和错误信息.
- `METADATA_PATH`: URL 元数据, 如果可获取.

### 2. 读取产物

必须读取 `TRANSCRIPT_PATH` 和 `PROVENANCE_PATH`.

如果 `METADATA_PATH` 存在且非空, 读取标题、作者/频道、发布时间、原始 URL 等元数据. 元数据只用于上下文和引用, 不能替代 transcript 成为总结依据.

如果 transcript 为空、过短或脚本报告失败, 停止总结并把失败原因、建议的下一步告诉用户. 不要从标题或简介编造总结.

### 3. 长 transcript 分块

当 transcript 超过当前上下文可稳定处理的长度时, 先按时间戳或段落分块:

- 每块保持 12k 到 20k 字符左右.
- 每块输出一份局部摘要, 保留关键时间戳和原话.
- 最后只基于局部摘要和必要原文片段合并总摘要.

不能跨块臆造时间线. 时间点只来自 transcript 中明确存在的时间戳.

### 4. 输出总结

默认用中文输出, 保留专有名词原文. 结构固定如下:

```markdown
# <视频/音频标题或文件名>

**来源**: <URL 或本地路径>
**Transcript 来源**: <platform-subtitle | funasr-asr | provided-transcript | direct-subtitle-url>
**可靠性备注**: <字幕/ASR 质量、缺失、语言、时间戳情况>

## 0x00 一句话总结

<2-4 句, 只写视频核心价值和结论>

## 0x01 核心要点

- <具体观点/事实/方法, 5-12 条>

## 0x02 时间线

- `<HH:MM:SS>` <这一段讲了什么>

## 0x03 关键原话

- `<HH:MM:SS>` "<短原话>" - <为什么重要>

## 0x04 可继续追问

- <可以进一步深挖的问题或待验证点>
```

如果 transcript 没有时间戳, `时间线` 改为 `内容脉络`, 不要虚构时间.

## 平台处理细节

### YouTube

优先抓取人工字幕或自动字幕. 若字幕语言有多种, 优先级为中文简体、中文、英文. 如果用户要求英文总结或保留英文 transcript, 尊重用户要求.

### Bilibili

优先尝试公开字幕. 如果 `yt-dlp` 被反爬或需要登录态, 询问用户是否允许使用本机浏览器 cookies, 或让用户显式传 `--cookies` / `--cookies-from-browser`. 不要自动进行登录操作.

### 其他平台

先走 `yt-dlp` 通用能力. 如果平台不支持或 DRM/登录限制导致无法下载, 明确报告限制, 并建议用户提供本地文件或字幕文件.

### 本地媒体

使用 `ffmpeg` 提取 16kHz 单声道 WAV 后再 ASR. 本地音频也统一转 WAV, 不直接把压缩音频交给 ASR.

## 反幻觉约束

- 总结、关键点、时间线、原话必须来自 transcript.
- 原话必须短引, 不要大段复述 transcript.
- 无法确认的人名、术语、数字要标注 `[不确定]`.
- ASR 结果可能错字, 对专有名词保持谨慎; 不要自行纠正成看似合理但 transcript 不支持的内容.
- 当用户要求"全文转写"时, 给出 transcript 文件路径和必要片段, 不要把超长全文直接刷到对话里.

## 依赖

基础依赖:

- `python`
- `ffmpeg`
- `yt-dlp` 或 `uvx yt-dlp`

ASR 依赖:

- `funasr`
- `modelscope`
- `torch`
- `torchaudio`

如果依赖缺失, 先给出可执行的安装/运行命令, 再停止当前步骤.
