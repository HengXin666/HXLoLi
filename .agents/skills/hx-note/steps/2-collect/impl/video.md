# 视频 / B站 沉淀

## 固定路线

1. **拿 transcript**: `.agents/skills/hx-note/scripts/cli/transcribe/hx_look_video_prepare.py`. 它产出三个路径变量: `TRANSCRIPT_PATH` (内容唯一依据) / `PROVENANCE_PATH` (来源类型与警告) / `METADATA_PATH` (标题、UP主、时间、URL, 只作引用)
2. **字幕来源决定可信度**: 平台字幕 / ASR / 人给的字幕 三者可信度不同, 在参考资料里一句说明用的是哪种
3. **重排主线, 不照搬视频结构**: 视频有大量铺垫与跑题. 先定它真正要回答的那一个问题, 再拆章节
4. **关键画面截图**: 见下

## 截图 (这一步最容易返工, 记几条踩过的)

抽帧

```
ffmpeg -ss 00:03:12 -i source.mp4 -frames:v 1 -q:v 2 shot-03-12.jpg
```

- **看不清就标 `[不确定]`, 不要按上下文猜图里写了什么. ** 编出来的画面比没有画面更坏
- 看不清又是关键内容时, 退一步用 OCR: `tesseract shot.jpg - -l chi_sim+eng --psm 6`. 幻灯片类视频可用场景检测一次拿到全部页面文字: `select='gt(scene,0.25)'` 配合 `metadata=print`
- **中间产物别和 `index.md` 同前缀、别用 `.html`**: 平台会把同目录所有 `.html` 发布到笔记路由, 用同名会多出莫名其妙的条目 (表现为「加载失败」). 抽帧、OCR 输出放临时目录, 收尾清一遍
- 图是**证据不是装饰**: 一张图对应正文一个论点, 且来自 transcript 里真实存在的时刻

## 反幻觉

- 时间点只来自 transcript 里明确存在的戳, 没有就不写时间线
- ASR 有错字, 专有名词尤其危险: 不确定标 `[不确定]`, **不要「修正」成看似合理但 transcript 不支持的内容**
- 标题、简介、评论**不能**当作内容依据 (它们是元数据, 不是口播内容)
- transcript 为空或过短 -> **停止沉淀并说明原因**, 不要凭标题硬写

## 不重复的部分

平台能力边界 (cookies、字幕语言优先级、ASR 支持范围) 以 `entries/transcribe/impl/transcribe.md` 为准