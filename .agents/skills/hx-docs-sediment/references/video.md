# 视频 / B站 沉淀

## 链路

1. **拿 transcript**: 交给 `hx-look-video` skill (脚本 `.agents/skills/hx-look-video/scripts/hx_look_video_prepare.py`). 它按证据链产出三个契约路径:

   | 输出 | 用途 |
   |---|---|
   | `TRANSCRIPT_PATH` | 规范化 transcript, **唯一**的内容依据 |
   | `PROVENANCE_PATH` | 来源类型 (platform-subtitle / funasr-asr / provided-transcript) 与警告 |
   | `METADATA_PATH` | 标题/UP主/发布时间/URL, 仅作上下文与引用 |

2. **读 provenance**: 来源是平台字幕还是 ASR, 直接决定可信度, 必须写进笔记的可靠性说明 (可在正文里以"资料来源"口吻一句带过).
3. **定主线**: 视频往往有大量铺垫. 先抽出"它真正要回答的一个问题", 再拆章节; 不要把视频结构照搬成文章结构.
4. **重点画面截图识别** (视频类专属): 口播讲不清楚的界面/架构图/数据, 用画面说明.
5. 回到固定契约: 模板初始化 → 成文 → 标点检查 → sidebar 注册.

## 关键画面截图识别

目的: 让笔记能说清"视频里那一屏到底是什么", 而不是只转述口播.

1. 从 transcript 的时间戳里挑出**信息密度最高**的 3~8 个点 (讲架构图/表格/演示界面的时刻).
2. 用 `ffmpeg` 在该时刻抽帧 (先确认本地已有媒体文件; 没有就先让 hx-look-video 下载):

   ```bash
   ffmpeg -ss 00:03:12 -i source.mp4 -frames:v 1 -q:v 2 shot-03-12.jpg
   ```

3. **识别画面内容, 按当前能力选路子** (不要假设自己一定看得见图):
   - **有视觉输入能力** (`read_image` 能读) → 直接读图.
   - **没有视觉能力** (`read_image` 报 "does not declare image input") → 走本机 OCR, 不要跳过这一步:

     ```bash
     tesseract shot-03-12.jpg - -l chi_sim+eng --psm 6
     ```

     批量时先跑 `ffmpeg` 场景检测定位"换页时刻", 再按时刻抽帧, 最后逐帧 OCR:

     ```bash
     ffmpeg -v error -i source.mp4 -vf "select='gt(scene,0.25)',metadata=print:file=scenes.txt" -f null -
     grep -o 'pts_time:[0-9.]*' scenes.txt
     ```

     幻灯片类视频这样能一次拿到**全部页面文字**, 比逐帧读图更省. OCR 对中英混排、数字、排版的识别足够支撑"找出哪几页值得引用"; 错字按第 4 条处理.
4. **只采信能看清的内容**: 看不清就标注, 不要按上下文猜图里写了什么. OCR 结果同样会被错认 (尤其专有名词), 拿不准就标 `[不确定]`.
5. 截图与 `index.md` **同目录**存放, 正文用平台图片语法引用, 并写清图注 (这一帧在讲什么、时间点).
   **命名要与其它产物区分开**: 平台会把"与 md 同目录的**所有** `.html`"发布到笔记路由下 —— 图/OCR/转换脚本的中间产物若用 `.html` 或与侧车同名的前缀, 会在页面上多出莫名其妙的条目 (表现为"加载失败"). 中间产物一律用独立前缀或放进临时目录, 收尾时清一遍目录.

   ```markdown
   ![调度点示意 ##w80%##](shot-03-12.jpg)
   ```

6. 截图是**辅助证据**, 不是装饰: 一张图要对应正文一个具体论点, 且来自 transcript 里真实存在的时刻.

## 反幻觉

- 时间点只来自 transcript 里明确存在的时间戳; 没有时间戳就不写时间线.
- ASR 有错字, 专有名词尤其危险: 不确定就标 `[不确定]`, 不要"修正"成看似合理但 transcript 不支持的内容.
- 元数据 (标题/简介/评论) **不能**替代 transcript 作为内容依据.
- transcript 为空、过短或脚本报错时, **停止沉淀**并说明原因, 不要凭标题硬写.

## 平台注意

- **采集层面** (cookies、字幕语言优先级、ASR 能力边界) 一律以 `hx-look-video` 为准, 本文件不重复.
- 完整脚本/超长转写不要刷进正文, 也不要整段贴进笔记.