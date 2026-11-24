# Agent Note: 音乐播放器 v2: 大文件切片播放, 多份歌词, 面板重做

Status: implemented

Decision-ID: music-player-v2

## Code

- `src/utils/music/musicStore.ts`
- `src/components/MusicPlayer/MusicPlayerBar.tsx`

## Problem

用户原话: 「加载较慢, 且由于大小限制, 好像只能加载 20MB 以下的歌曲」「希望后续一首歌能支持多个歌词」「根据类似于我主页的风格, 把它适配成更好, 更有交互, 更现代的前端播放器风格」。

`loadTrack` 直接把 jsDelivr 直链赋给 `audio.src`。jsDelivr 对 20MB 以上的文件回 403, 所以 `ろりこんでよかった~.flac` (26.7MB) 和 `adrenaline!!! - TrySail.flac` (56.1MB) 一直放不了。音乐库早就为它们生成了切片 (`static/cdn/all/.../info.yaml`), 但只有歌词和字体走 `reqMusicByCDN`, 音频没用上。

其它问题: 快速连切时上一首的 `canplay` 监听没摘, 会按旧的 seek 和自动播放再执行一次; 播放状态只存曲目索引, 音乐库增删歌后恢复到错的歌; 面板颜色写死白色透明度, 亮色主题下看不清; 进度条拖动时界面不跟手。

## Decision

**音频先直链, 被拒再切片; 歌词变成列表; 面板按首页视觉重写。**

- `loadTrack` 每次带 token, 过期回调全部作废。直链 `error` 时 (非本地服务) 改用 `reqMusicByCDN(audioRelativePath)` 切片下载, 得到 Blob URL 再播, 下载进度写进 `downloadProgress`, 换歌时回收旧 Blob URL。
- 播放列表条目可带 `audioWebUrl` (音乐库以后为大无损文件生成的网页版)。有它时默认播它, 面板上的 "无损" 开关改播原文件 (`reload` 命令保持进度)。
- 多份歌词: `MusicTrack.lyrics: [{ key, label, assUrl, detailId? }]`。旧数据只有 `assUrl` 时派生一份 `default`。选择按歌存 localStorage (`hxloli-music-lyric`), 各 Tab 通过 storage 事件同步。`AssLyrics` 用 `pickLyric` 取当前歌词, 详细配置读 `static/info/<detailId 或歌曲 id>.json`。
- 状态额外存 `trackId`, 恢复时先按 id 找。
- Media Session: 锁屏, 耳机按键和键盘媒体键能播放, 暂停, 切歌和拖进度。
- 面板: 64px 封面, 歌名和 "歌手 · 番剧" 自动滚动, 格式标签和加载状态; 自绘拖动条 (已缓冲段, 悬停时间提示, 拖动时只改本地值松手再跳); 歌词分段选择; 音量; 播放列表加搜索, 正在播放的一行显示跳动小条, 打开时滚到当前歌。全部颜色走 `--ifm-*` 令牌, 亮色下文字用压暗的粉。
- `static/sw.js` 增加音乐库缓存: 字体和封面按路径缓存, 歌词, 详细配置和切片按 tag 加路径缓存, 带 Range 或查询串的请求不碰 (见 image-cache-sw 记录)。

验证: tsc 对改动文件无报错; 面板用 esbuild 单独打包, 在用户机器的 headless chromium 截图看过亮暗两套和分片下载状态。

## Alternatives considered

- 什么都不做: 两首大 FLAC 继续放不了。
- 复用现有方案 (只靠 `reqMusicByCDN` 下载所有音频): 小文件也要整首下完才能播, 首播更慢; 直链对 20MB 以下文件能边下边播。
- 用 MediaSource 流式播放切片: FLAC 原始流在 MSE 里支持不一, 收益要等音乐库出网页版才明显。
- 多份歌词用单独的 playlist 条目: 同一首歌会在列表里出现多次, 播放进度和收藏都要拆开。

## Consequences

- 大 FLAC 第一次要整首下完才开始播 (56MB 要等下载)。之后切片走 Service Worker 缓存。真正解决要等音乐库生成 `audioWebUrl`。
- 音乐库要支持多歌词, `gen_music_playlist.py` 需要按 `歌名.标签.ass` 产出 `lyrics` 数组; 每份歌词的字体和边界框不同时, 另出 `static/info/<id>.<key>.json` 并在 `detailId` 里写明。
- Follower Tab 看不到加载进度和缓冲段, 只显示 Leader 同步来的时间。

## Revision 2026-11-25

**歌词字体与 ASS 同时下载, 走多 CDN 竞速; 进度条和音量分开。**

- 实测 (オンリー): ASS 9.5s 到, 特效字体到 70s 才齐。原因是字体要等 Worker 就绪才开始下, 绝对 URL 直连 fastly, NotoSansSC (17.8MB) 被 `assFontMap`/`fonts` 和兜底各下一次, 20MB 以上的字体被 403 且没有切片回退, 每到一个字体就 `setTrack` 重解析 3MB 的 ASS。
- 现在拿到详细配置就开始下字体, 绝对 URL 还原成 `/static/...` 走 `reqMusicRace` (自建节点无 20MB 限制), 失败再走 `reqMusicByCDN`。按路径去重, NotoSansSC 等本曲字体结束后再下。热重载合成至多三次: 4s 时先用已到的, 本曲字体全齐, 兜底字体到。
- 根治仍在音乐库: 按曲子集化字体为 woff2。
- 面板: 进度条 24px 命中区, 悬停加粗, 缓冲段, rAF 外推, 键盘 Home/End/PageUp/PageDown, 点时长切剩余时间。音量从独立一行改成控制行右端的图标, 悬停弹出竖向滑条, 点击静音, 滚轮调节。
