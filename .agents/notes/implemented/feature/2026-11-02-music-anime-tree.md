# Agent Note: 播放器按番剧树形展开

Status: implemented

Decision-ID: music-anime-tree

## Problem

用户 2026-11-02: 播放器的播放列表是一列平铺的 15 首歌, 希望"按照番剧进行归类, 以树形的模式进行展开", 形态由我们决定。
HXLoLi-Music 的 `playlist.json` 现在给每首歌带 `anime: { title } | null` (取自 `static/music/<番剧名>/` 目录, 见该仓库 note playlist-anime-group)。

## Decision

- `src/config/musicData.ts`: `MusicTrack.anime?: { title: string } | null`。
- `src/components/MusicPlayer/MusicPlayerBar.tsx`: 播放列表改为两级树。一级是番剧 (封面缩略图取组内第一张 `coverUrl`、番名、曲数、展开箭头), 二级是该番的歌 (保留原序号, 点击 `setTrack(原索引)`)。`anime` 为空的归到最后的"其它"组。组的顺序按该组第一首在原列表里出现的位置, 不重排歌曲, 所以上一首 / 下一首 / 随机仍按原列表走。
- 默认只展开正在播放的那一组; 切歌到别的组时自动展开那一组, 已手动展开的保持。
- 展开动画用 `grid-template-rows: 0fr -> 1fr` (只有一层内容, 不量高度), `prefers-reduced-motion` 时取消。
- 播放列表最大高度从 150px 放到 280px, 放得下组头。
- 只有一组时 (旧数据没有 anime 字段) 退化为原来的平铺列表。

## Consequences

- playlist.json 的 anime 字段值直接从 audioUrl 的目录名解析, 不另立一份番剧表, 也不做人工映射.
  目录名是什么就写什么, 日文保留日文.
- 目录名其实是歌手名时 (如 ちぃむdmp☆) anime 为 null, 归入"其它"分组, 不硬塞进某部番.
- 播放器分组只依赖这个字段, 源数据改了就跟着变, 前端不用维护映射.
- 番剧名与 ANiMeRecord 的对应不做自动猜测: 匹配得上就关联封面和详情, 匹配不上就只显示名字.

## Alternatives considered

- 什么都不做: 最强理由是 15 首歌平铺也能一屏看完。否决: 用户明确要按番剧归类; 天使の3P 一部就有 4 首, 平铺看不出归属, 歌越多越乱。
- 复用现有方案 (平铺列表里每行加一个番名小标签): 最强理由是改动最小、不改交互。否决: 只是贴标签, 不能折叠, 也没有"一部番一组"的结构, 不是用户要的树。
- 番剧封面卡片墙 (一级是封面网格): 好看但面板只有约 320px 宽, 一行放两三张就满, 15 首要来回翻; 否决, 树形列表更省空间、更好扫。

## Code

- `src/components/MusicPlayer/MusicPlayerBar.tsx`
- `src/config/musicData.ts`
