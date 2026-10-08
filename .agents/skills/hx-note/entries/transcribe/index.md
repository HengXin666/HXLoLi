# transcribe — 视频/音频 -> 可引用 transcript

**第三条入口. 不重叠于九步沉淀流水线. **

## 只做这一件事

把视频、音频链接、本地媒体文件或字幕文件转成可引用的 transcript, 再基于它输出中文总结

## 关键约定

**默认只在对话里输出, 不落盘. ** 只有用户**明确要求**保存到 `ai-docs` / `docs` / 指定文件时, 才写成正式文档

需要沉淀成笔记时, 走九步流水线的步骤 2 (`--kind video`), 本入口只负责出 transcript

## 实现

平台差异 (YouTube / Bilibili / 其他)、ASR 依赖与运行方式、长 transcript 分块策略、输出格式、反幻觉约束: `impl/transcribe.md`

## 脚本

`scripts/cli/transcribe/hx_look_video_prepare.py`  解析输入、准备 transcript
