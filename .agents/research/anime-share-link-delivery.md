# 番单分享链接 · 交付说明 (2026-11-01)

决策记录: `.agents/notes/implemented/feature/2026-11-01-anime-share-link.md`

## 1. 分享链接的 URL 格式

```
<站点根>/anime/share?<番剧库同款筛选>&pick=<encodeIds 压缩的 subject id 集合>
```

站点根取自 docusaurus.config.ts: Cloudflare 分支 `url = https://km.woa.qzz.io`、`baseUrl = "/"`; GitHub Pages 分支 `url = https://HengXin666.github.io`、`baseUrl = "/HXLoLi"`. 代码里 `shareUrl()` 把两者拼成根, 再接 `/anime/share`.

数据来源 (实测, 2026-11-01): 从站点 CDN 节点 `cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@<tag>/data/ANiMeRecord.json` (linkJsDelivr.ts 的 `directUrl`) 拉取. 注意 `data/animeCdnVersion.ts` 里的 `HX-20260906113630-E1EgjV` 这个 tag 在 jsdmirror 与 raw.githubusercontent 上都返回 404 (GitHub 上已不存在该 tag), 所以用的是仓库现存最新 tag `HX-20261101141415-poqRvw` (13,486,403 字节). 共 177 部: 看过 148、在看 14、搁置 10、想看 4、抛弃 1. `data/animeOverlay.json` 当前为空, 不改变状态, 所以站点上 `status=watched` 的番单就是这 148 部.

### 真实链接一: 全部看过 (148 部全勾)

番单 = `status=watched` (148 部), pick = 这 148 个真实 subject id. pick 402 字符, 整条 Cloudflare 链接 456 字符, GitHub Pages 链接 470 字符.

```
https://km.woa.qzz.io/anime/share?status=watched&pick=1SvWcTz1iGA5Rw2MgD6-U2Fxb--1YU2_-dOD4TN3yU3QUgJA6Ly4_J1SG3GA5MQ1LS4VcSP2HP3EM2ES2YuNy5YV1ynSSaTCfEKoEG1P-5ZH1WF3TE3Ww6CN9S3yT6QS1J9JF2PW1EbK-1IR5Sx1GU4AQ3yfIz4-F1QvRX5SJ1I1Zw3UN1_A6Z2Lx2G21GpDH2NC6RF1DcUD3ZeMobOU3OR2Q7X2_I6Jk1L2Gy3JuU1Mcxw2EF4zL5PJ6XoXhAV7-E1HB2UoSV1AGw1PC8AJ3ArDD1HU7FP2x3xI1KF4OB2YY2RW6KU3zT2ZM1z_3-wnPcSmAG1YLdUN7_z3WT5zE1zQ9CKaxM3Bz8Cewz3XeVC1B5V3VA1AV6_M3PU1VwiRAgBH1OD3LJazxH1-T9
```

```
https://HengXin666.github.io/HXLoLi/anime/share?status=watched&pick=1SvWcTz1iGA5Rw2MgD6-U2Fxb--1YU2_-dOD4TN3yU3QUgJA6Ly4_J1SG3GA5MQ1LS4VcSP2HP3EM2ES2YuNy5YV1ynSSaTCfEKoEG1P-5ZH1WF3TE3Ww6CN9S3yT6QS1J9JF2PW1EbK-1IR5Sx1GU4AQ3yfIz4-F1QvRX5SJ1I1Zw3UN1_A6Z2Lx2G21GpDH2NC6RF1DcUD3ZeMobOU3OR2Q7X2_I6Jk1L2Gy3JuU1Mcxw2EF4zL5PJ6XoXhAV7-E1HB2UoSV1AGw1PC8AJ3ArDD1HU7FP2x3xI1KF4OB2YY2RW6KU3zT2ZM1z_3-wnPcSmAG1YLdUN7_z3WT5zE1zQ9CKaxM3Bz8Cewz3XeVC1B5V3VA1AV6_M3PU1VwiRAgBH1OD3LJazxH1-T9
```

### 真实链接二: 看过番单里勾 30 部

番单同样是 `status=watched` (M = 148), pick = 首播日期最新的 30 部真实看过番 (赛博朋克：边缘行者 2、Re：从零开始的异世界生活 第四季 夺还篇、碧蓝之海 第三季、无职转生 第三季 ～到了异世界就拿出真本事～、LV999的村民、Re：从零开始的异世界生活 第四季 丧失篇、女神“异世界转生想成为什么”我“勇者的肋骨”、关于我转生变成史莱姆这档事 第四季、欢迎来到实力至上主义教室 第四季、葬送的芙莉莲 第二季、【我推的孩子】 第三季、Fate/strange Fake、判处勇者刑 惩罚勇者9004队刑务纪录、辉夜大小姐想让我告白 通往大人的阶梯、异世界四重奏 第三季、一拳超人 第三季、间谍过家家 第三季、剧场版 链锯人 蕾塞篇、拔作岛、坂本日常 第2部分、盾之勇者成名录 第四季、碧蓝之海 第二季、与游戏中心的少女异文化交流的故事、胆大党 第二季、章鱼哔的原罪、小林家的龙女仆 怕寂寞的龙、打了300年的史莱姆，不知不觉就练到了满级 第二季、平凡职业造就世界最强 第三季、香格里拉·弗陇提亚～屎作猎人向神作发起挑战～ 第二季、胆大党). 打开后显示"TA 看过 30 部". pick 92 字符, Cloudflare 链接 146 字符, GitHub Pages 链接 160 字符.

```
https://km.woa.qzz.io/anime/share?status=watched&pick=1ZxT8LDJ2QYy1TDG1Ww2Qxl_z3WT5zE1zQ9CKaxM3Bz8Cewz3XeVC1B5V3VA1AV6_M3PU1VwiRAgBH1OD3LJazxH1-T9
```

```
https://HengXin666.github.io/HXLoLi/anime/share?status=watched&pick=1ZxT8LDJ2QYy1TDG1Ww2Qxl_z3WT5zE1zQ9CKaxM3Bz8Cewz3XeVC1B5V3VA1AV6_M3PU1VwiRAgBH1OD3LJazxH1-T9
```

两条链接都用 `decodeIds(encodeIds(ids))` 做过往返校验, 解出的 id 集合与原集合完全一致.

为什么选 `/anime/share?筛选&pick=`, 不选 `/anime?pick=` 或单个 `?s=`:
- 番剧库是站长自己的筛选视图. 把访客勾选混进去语义不清, 还要改 index.tsx 的列表逻辑;
- 筛选部分本来就短、可读, 并且和 /anime 地址栏互通 (在番剧库筛好, 把 `/anime` 换成 `/anime/share` 就得到番单), 以后番剧库加新维度时分享链接自动跟上. 真正需要压缩的只有 id 集合, 放进 `pick`.

## 2. 编码方案与长度

`encodeIds` (`src/components/anime/animeShareLink.ts`):
1. id 去重、升序;
2. 相邻差分 (第一个就是自身);
3. 每个差值写成小端变长整数: 5 bit 数据 + 1 bit"后面还有", 每组映射成 1 个 base64url 字符 (`0-9a-zA-Z-_`, 都是 URL 免转义字符);
4. 前缀版本号 `1`, 无分隔符.

差 < 32 占 1 字符, < 1024 占 2, < 32768 占 3, < 1048576 占 4. 解码另兼容手写的 `pick=12,345,6789`; 非法输入返回空数组; 上限 20000 个.

整条 URL 长度按 `https://km.woa.qzz.io/anime/share?status=watched&pick=` (54 字符) 加 pick 计算; GitHub Pages 前缀多 14 字符.

| N 部 | 数据 | pick 字符数 | 整条 URL (Cloudflare) | 整条 URL (GitHub Pages) |
| --- | --- | --- | --- | --- |
| 30 | 真实: 看过番里首播最新 30 部 | 92 | 146 | 160 |
| 100 | 模拟: 从 177 个真实 id 无放回抽 100 个, 200 次取中位数 (区间 272~292) | 281 | 335 | 349 |
| **148** | **真实: 全部看过** | **402** | **456** | **470** |
| 177 | 真实: 全库 (所有状态) | 471 | 525 | 539 |
| 239 | 模拟: 真实 id 只有 177 个, 按真实 id 的经验分布 (相邻真实 id 之间均匀插值) 生成, 200 次中位数 (区间 565~604) | 586 | 640 | 654 |
| 300 | 模拟, 方法同上 (区间 690~739) | 713 | 767 | 781 |
| 500 | 模拟, 方法同上 (区间 1070~1135) | 1103 | 1157 | 1171 |

结论: 真实 148 部看过番的整条链接是 456 字符; 按真实 id 分布外推, 500 部也只有约 1157 字符, 都在 2000 以内. 平均每部约 2.2~2.7 字符. 对照: 148 部用逗号分隔十进制需要 1006 字符.

## 3. 分享页的可交互能力 (/anime/share)

- 每部番是一个可点条目 (封面 + 名称 + 年份), 点一下切换"我看过", 未勾的封面灰化, 勾上有 ✓;
- 顶部吸顶进度: "我看过 N/M 部" + 进度条 (M = 这份番单总部数, 不随页内筛选变化); 链接带 pick 时另显示"TA 看过 K 部 · 我们都看过 Z 部", 条目角标"TA 看过";
- 页内筛选 (只改显示, 不改番单):
  - 年份范围: 近 5 年 / 近 10 年 / 近 15 年 / 全部
  - 勾选: 全部 / 我看过 / 我没看过 / TA 看过 / 我们都看过 (后两项仅链接带 pick 时出现)
  - 站长观看状态: 在看 / 看过 / 想看 / 搁置 / 抛弃, 多选, 带分面计数
  - 标签: 按部数排序, 默认 18 个可展开全部, 多选时可切"全部命中 / 任一命中"
  - 搜索番名、排序 (最近更新 / 首播日期 / 评分 / 名称; 默认按首播分季度分组)
- 批量: 全选当前、取消当前、照抄 TA 的勾选、清空我的 (二次确认)、重置筛选;
- 勾选自动存 localStorage, 键 `<baseUrl>anime/share:v1:<番单标识>`, 番单标识 = 筛选去掉排序后的 FNV-1a 哈希: 同一份番单无论谁转发都读自己那份, 不同番单互不污染;
- "生成我的链接": 自动复制, 显示链接与字符数, 附"在本页打开";
- 附带"复制图片 / 下载图片" (复用 animeShareImage.ts, 可选带封面): 有勾选画"我看过的番 · N/M", 没勾画当前显示的部分; 浏览器不支持 ClipboardItem 时隐藏"复制图片".

## 4. 别人勾选后怎么再分享

打开别人的链接 -> 点番勾自己看过的 (可先"照抄 TA 的勾选"再改) -> 底部"生成我的链接". 新链接 = 原番单筛选不变 + 自己的勾选重新编码进 `pick`, 已复制到剪贴板. 下一个人打开时看到的"TA 看过"就是你的勾选, 如此传播. 勾选一变, 已生成的旧链接自动收起, 避免复制过期链接. 原链接地址栏不会被改写.

## 5. 改动文件

新建:
- `src/components/anime/AnimeShareBoard.tsx` — 分享页主体 (可交互番单)
- `src/components/anime/AnimeShareBoard.module.css`
- `src/components/anime/animeShareLink.ts` — URL 编解码、番单标识、localStorage 读写、年份范围 / 勾选筛选
- `src/components/anime/animeShareExport.ts` — 图片生成后的下载 / 复制 (两处共用)
- `.agents/notes/implemented/feature/2026-11-01-anime-share-link.md`
- `.agents/research/anime-share-link-delivery.md` (本文件)
- `src/components/anime/animeShareLink.ts` 本轮小修: `storageKeyOf` 统一给 baseUrl 补尾斜杠 (GitHub Pages 的 baseUrl 是 `/HXLoLi`, 原来会拼成 `/HXLoLianime/share:...`)

改动:
- `src/pages/anime/share.tsx` — 改挂 AnimeShareBoard
- `src/components/anime/AnimeSharePanel.tsx` — 改为"按当前筛选生成分享链接" + 复制链接 / 打开分享页; 下载图片 (点了才画) 与复制清单降为次要
- `src/components/anime/AnimeSharePanel.module.css`
- `.agents/notes/implemented/feature/2026-11-01-anime-share-image-and-list.md` — Code 去掉 share.tsx, Consequences 补更新说明
- `.agents/notes/implemented/feature/2026-11-01-anime-share-on-index.md` — 更新面板行为描述

未动: `filters.ts` / `tags.ts` / `overlay.ts` / `index.tsx` (挂载点 `<AnimeSharePanel embedded />` 接口不变) / `details.tsx` / `AnimeGraphPage/` / `home/` / `animeShareImage.ts` / `animeShareText.ts`. 无新增依赖.

Agent Note 双链: 按 `.agents/notes/AGENTS.md` "同一决策在每个直接父目录仅有一个代表文件和一个引用位置", `## Code` 列两个代表文件 (`src/pages/anime/share.tsx`、`src/components/anime/AnimeShareBoard.tsx`), 二者声明顶部注释反向引用 note; 同目录兄弟文件 (animeShareLink.ts 等) 在 Decision 中逐一点名.

## 6. tsc 结果

- `npx tsc --noEmit` (你在本机跑, 2026-11-01): 10 条错误, 与 `scripts/quality-baseline.json` 的 knownErrors 一一对应, **新增 0**.
- 沙箱里另用 TypeScript 5.6 strict + @types/react 19 + Docusaurus 模块桩检查 share.tsx / AnimeShareBoard.tsx / AnimeSharePanel.tsx 及依赖: 0 错误.
- Agent Notes: `2026-11-01-anime-share-on-index.md` 已补 `# Agent Note:` 标题、`Status: implemented`、`Decision-ID: anime-share-on-index`, `## Code` 改成 `- \`路径\`` 格式, Alternatives 补了"什么都不做"和"复用现有独立页"两项. 用仓库 `graph.py` 的 `parse_note` 原样校验, 四篇相关 note (share-on-index / share-link / share-image-and-list) 都通过. AnimeSharePanel.tsx:41 的 note-path 来自这篇 note 解析失败, 现在应消失; 请再跑一次 `uv run scripts/redlines/agent_notes.py --diff` 确认.
- 遗留: `.agents/notes/implemented/feature/2026-11-01-anime-share-link-delivery.md` (上一轮按要求放的交付说明副本) 的文件名符合 note 路径规则, 会被当成决策记录解析并报 note-format. 映射通道不支持删除, 请手动 `rm` 它; 交付说明以本文件为准.
